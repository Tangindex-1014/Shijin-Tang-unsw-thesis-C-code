clear; clc; close all;

%% =========================================================
%  Parameters
% ==========================================================
dataFolder = 'C:\Users\13168\Desktop\毕业设计\控制系统代码\test2';
filePattern = 'heart_data_speed_*.csv';

% Motor calibration:
% The number in the CSV file name is PWM command, not real motor speed.
% PWM 145~255 corresponds approximately to 90~200 rpm.
% Therefore: real RPM = PWM - 55.
pwmToRpmOffset = 55;

P_low  = 65;
P_high = 115;
RPM_min = 1000;
RPM_max = 2000;

P0     = P_low;

simDuration = 5;
dt = 0.01;

speedCurveExponent = 1.8;

pressureSmoothWindow = 7;
curveSmoothWindow = 5;

midBandLow  = 0.35;
midBandHigh = 0.65;

clampVmToTrainingRange = false;

saveTables = true;
outputFolder = 'C:\Users\13168\Desktop\毕业设计\控制系统代码';

pressurePlotMargin = 3;
vmPlotMargin = 12;
vpPlotMargin = 60;

% Resting heartbeat: relaxation time is about twice contraction time.
restRelaxToContractTimeRatio = 2.0;
restRelaxRateScale = 1 / restRelaxToContractTimeRatio;

%% =========================================================
%  1) Read CSV and extract training data
% ==========================================================
files = dir(fullfile(dataFolder, filePattern));
if isempty(files)
    error(['No CSV files found under ', fullfile(dataFolder, filePattern)]);
end

%% =========================================================
%  0) Pressure calibration: CSV pressure -> real mmHg
% ==========================================================
trueLowPressureMmHg = 30;
trueHighPressureMmHg = 45;

calibrationSmoothWindow = 9;
calibrationMinProminence = 10;

[csvLowRef, csvHighRef, calibrationSummary] = estimatePressureCalibration( ...
    files, calibrationSmoothWindow, calibrationMinProminence);

pressureCsvGain = (csvHighRef - csvLowRef) / (trueHighPressureMmHg - trueLowPressureMmHg);
pressureCsvOffset = csvLowRef - pressureCsvGain * trueLowPressureMmHg;

csvPressureToMmHg = @(Pcsv) (Pcsv - pressureCsvOffset) ./ pressureCsvGain;
mmHgToCsvPressure = @(PmmHg) pressureCsvGain .* PmmHg + pressureCsvOffset;

allRiseX = [];
allRiseDxdt = [];
allRiseVm = [];

allFallX = [];
allFallDxdt = [];
allFallVm = [];

vmLevels = [];
riseBaseDxdtList = [];
fallBaseDxdtList = [];

for i = 1:numel(files)
    fullName = fullfile(files(i).folder, files(i).name);
    T = readtable(fullName);

    t = getColumnData(T, {'Time_s','Time','time','Time_ms'});
    if max(t) > 1000
        t = t / 1000;
    end

    Pcsv = getColumnData(T, {'Pressure','pressure'});
    lowCsv = getColumnData(T, {'LowLimit','lowlimit','Low','low'});
    highCsv = getColumnData(T, {'HighLimit','highlimit','High','high'});
    M1 = getColumnData(T, {'Motor1','motor1','Motor','motor'});

    P = csvPressureToMmHg(Pcsv);
    low = csvPressureToMmHg(lowCsv);
    high = csvPressureToMmHg(highCsv);

    thisLow = mean(low, 'omitnan');
    thisHigh = mean(high, 'omitnan');
    thisSpan = thisHigh - thisLow;
    if ~isfinite(thisSpan) || thisSpan <= 0
        continue;
    end

    pwmNominal = parseSpeedFromFilename(files(i).name);
    if isnan(pwmNominal)
        pwmNominal = median(abs(M1(abs(M1) > 0)), 'omitnan');
    end
    RPM_nominal = pwmNominal - pwmToRpmOffset;
    if ~isfinite(RPM_nominal) || RPM_nominal <= 0
        continue;
    end

    Psm  = smoothdata(P, 'movmean', pressureSmoothWindow);
    dPdt = gradient(Psm, t);

    x = (Psm - thisLow) / thisSpan;
    x = min(max(x, 0), 1);
    dxdt = dPdt / thisSpan;

    riseMask = (M1 > 0) & isfinite(x) & isfinite(dxdt) & (dxdt > 0);
    fallMask = (M1 < 0) & isfinite(x) & isfinite(dxdt) & (dxdt < 0);

    riseDxdt = dxdt(riseMask);
    fallDxdt = -dxdt(fallMask);

    allRiseX = [allRiseX; x(riseMask)]; %#ok<AGROW>
    allRiseDxdt = [allRiseDxdt; riseDxdt]; %#ok<AGROW>
    allRiseVm = [allRiseVm; RPM_nominal * ones(sum(riseMask),1)]; %#ok<AGROW>

    allFallX = [allFallX; x(fallMask)]; %#ok<AGROW>
    allFallDxdt = [allFallDxdt; fallDxdt]; %#ok<AGROW>
    allFallVm = [allFallVm; RPM_nominal * ones(sum(fallMask),1)]; %#ok<AGROW>

    riseMidMask = riseMask & x >= midBandLow & x <= midBandHigh;
    fallMidMask = fallMask & x >= midBandLow & x <= midBandHigh;

    riseBase = median(dxdt(riseMidMask), 'omitnan');
    fallBase = median(-dxdt(fallMidMask), 'omitnan');

    if isnan(riseBase) || riseBase <= 0
        riseBase = median(riseDxdt, 'omitnan');
    end
    if isnan(fallBase) || fallBase <= 0
        fallBase = median(fallDxdt, 'omitnan');
    end

    if isfinite(riseBase) && isfinite(fallBase)
        vmLevels(end+1,1) = RPM_nominal; %#ok<SAGROW>
        riseBaseDxdtList(end+1,1) = riseBase; %#ok<SAGROW>
        fallBaseDxdtList(end+1,1) = fallBase; %#ok<SAGROW>
    end
end

if isempty(vmLevels)
    error('No valid training data extracted from CSV files.');
end

%% =========================================================
%  2) Vm -> base dx/dt
% ==========================================================
pRiseVm = polyfit(vmLevels, riseBaseDxdtList, 1);
pFallVm = polyfit(vmLevels, fallBaseDxdtList, 1);

baseRiseDxdtFcn = @(Vm) max(polyval(pRiseVm, Vm), 1e-8);
baseFallDxdtFcn = @(Vm) max(polyval(pFallVm, Vm), 1e-8);

trainingVmMin = min(vmLevels);
trainingVmMax = max(vmLevels);

%% =========================================================
%  3) Pressure term: in both rise/fall, larger P -> smaller Vp
% ==========================================================
riseFactorRaw = allRiseDxdt ./ max(baseRiseDxdtFcn(allRiseVm), 1e-8);
fallFactorRaw = allFallDxdt ./ max(baseFallDxdtFcn(allFallVm), 1e-8);

xGrid = linspace(0, 1, 60)';
riseFactor = buildFactorCurveMonotonic(allRiseX, riseFactorRaw, xGrid, curveSmoothWindow, "decreasing");
fallFactor = buildFactorCurveMonotonic(allFallX, fallFactorRaw, xGrid, curveSmoothWindow, "decreasing");

riseFactorFcn = @(x) interp1(xGrid, riseFactor, min(max(x,0),1), 'pchip', 'extrap');
fallFactorFcn = @(x) interp1(xGrid, fallFactor, min(max(x,0),1), 'pchip', 'extrap');

%% =========================================================
%  4) Vm, P -> Vp models
% ==========================================================
VpRiseFcn = @(P, Vm, lowP, highP) ...
    (highP - lowP) .* baseRiseDxdtFcn(Vm) .* riseFactorFcn((P - lowP) ./ max(highP - lowP, eps));

VpFallFcn = @(P, Vm, lowP, highP) ...
    (highP - lowP) .* baseFallDxdtFcn(Vm) .* fallFactorFcn((P - lowP) ./ max(highP - lowP, eps));


VpFallIdealFcn = @(P, Vm, lowP, highP) ...
    (highP - lowP) .* baseRiseDxdtFcn(Vm) .* riseFactorFcn((P - lowP) ./ max(highP - lowP, eps));


VpFallRestFcn = @(P, Vm, lowP, highP) ...
    restRelaxRateScale .* (highP - lowP) .* baseRiseDxdtFcn(Vm) .* riseFactorFcn((P - lowP) ./ max(highP - lowP, eps));

%% =========================================================
%  5) Tables
% ==========================================================
vmGrid = unique(sort(vmLevels(:)))';
pressureGrid = linspace(P_low, P_high, 20)';

riseVpTableValues = zeros(numel(pressureGrid), numel(vmGrid));
fallVpTableValues = zeros(numel(pressureGrid), numel(vmGrid));
idealFallVpTableValues = zeros(numel(pressureGrid), numel(vmGrid));
restFallVpTableValues = zeros(numel(pressureGrid), numel(vmGrid));

for i = 1:numel(pressureGrid)
    for j = 1:numel(vmGrid)
        riseVpTableValues(i,j) = VpRiseFcn(pressureGrid(i), vmGrid(j), P_low, P_high);
        fallVpTableValues(i,j) = VpFallFcn(pressureGrid(i), vmGrid(j), P_low, P_high);
        idealFallVpTableValues(i,j) = VpFallIdealFcn(pressureGrid(i), vmGrid(j), P_low, P_high);
        restFallVpTableValues(i,j) = VpFallRestFcn(pressureGrid(i), vmGrid(j), P_low, P_high);
    end
end

riseVpTable = array2table(riseVpTableValues, 'VariableNames', matlab.lang.makeValidName("RPM_" + string(vmGrid)));
riseVpTable = addvars(riseVpTable, pressureGrid, 'Before', 1, 'NewVariableNames', 'Pressure');

fallVpTable = array2table(fallVpTableValues, 'VariableNames', matlab.lang.makeValidName("RPM_" + string(vmGrid)));
fallVpTable = addvars(fallVpTable, pressureGrid, 'Before', 1, 'NewVariableNames', 'Pressure');

idealFallVpTable = array2table(idealFallVpTableValues, 'VariableNames', matlab.lang.makeValidName("RPM_" + string(vmGrid)));
idealFallVpTable = addvars(idealFallVpTable, pressureGrid, 'Before', 1, 'NewVariableNames', 'Pressure');

restFallVpTable = array2table(restFallVpTableValues, 'VariableNames', matlab.lang.makeValidName("RPM_" + string(vmGrid)));
restFallVpTable = addvars(restFallVpTable, pressureGrid, 'Before', 1, 'NewVariableNames', 'Pressure');

%% =========================================================
%  6) Figure 1: fitted curves
% ==========================================================
figure('Name','Figure 1: RPM-P-Vp Training','Color','w');

subplot(2,2,1);
hold on; grid on; box on;
scatter(vmLevels, riseBaseDxdtList, 35, 'r', 'filled');
scatter(vmLevels, fallBaseDxdtList, 35, 'b', 'filled');
vmFitGrid = linspace(min(vmLevels)-5, max(vmLevels)+5, 200);
plot(vmFitGrid, baseRiseDxdtFcn(vmFitGrid), 'r-', 'LineWidth', 1.5);
plot(vmFitGrid, baseFallDxdtFcn(vmFitGrid), 'b-', 'LineWidth', 1.5);
xlabel('Motor Speed Vm (rpm)');
ylabel('Base dP/dt');
title('RPM -> Vp');
legend('Rise data','Fall data','Rise fit','Fall fit','Location','best');

subplot(2,2,2);
hold on; grid on; box on;
plotColors = lines(numel(vmGrid));
for j = 1:numel(vmGrid)
    vmNow = vmGrid(j);
    plot(pressureGrid, VpRiseFcn(pressureGrid, vmNow, P_low, P_high), '-', ...
        'Color', plotColors(j,:), 'LineWidth', 1.4, ...
        'DisplayName', sprintf('Rise RPM=%d', round(vmNow)));
end
for j = 1:numel(vmGrid)
    vmNow = vmGrid(j);
    plot(pressureGrid, VpFallFcn(pressureGrid, vmNow, P_low, P_high), ':', ...
        'Color', plotColors(j,:), 'LineWidth', 1.0, ...
        'DisplayName', sprintf('Original Fall RPM=%d', round(vmNow)));
end
for j = 1:numel(vmGrid)
    vmNow = vmGrid(j);
    plot(pressureGrid, VpFallIdealFcn(pressureGrid, vmNow, P_low, P_high), '--', ...
        'Color', plotColors(j,:), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('Ideal Fall RPM=%d', round(vmNow)));
end
xlabel('Pressure P (mmHg)');
ylabel('Vp = dP/dt');
title('P -> Vp');
legend('Location','eastoutside');

subplot(2,2,3);
scatter(allRiseX, allRiseDxdt, 5, allRiseVm, 'filled');
grid on; box on;
xlabel('Normalized Pressure Position');
ylabel('dx/dt');
title('Rise Samples');
cb1 = colorbar;
ylabel(cb1, 'RPM');

subplot(2,2,4);
scatter(allFallX, allFallDxdt, 5, allFallVm, 'filled');
grid on; box on;
xlabel('Normalized Pressure Position');
ylabel('|dx/dt|');
title('Fall Samples');
cb2 = colorbar;
ylabel(cb2, 'RPM');

%% =========================================================
%  7) Figure 2: original rise/fall surfaces
% ==========================================================
[vmMesh, pMesh] = meshgrid(vmGrid, pressureGrid);

figure('Name','Figure 2: Original Rise/Fall Surface','Color','w');

subplot(1,2,1);
surf(vmMesh, pMesh, riseVpTableValues, 'EdgeColor', 'none');
grid on; box on; view(45,25);
xlabel('Vm (rpm)');
ylabel('Pressure P (mmHg)');
zlabel('Vp = dP/dt');
title('Original Rise Surface');
colorbar;

subplot(1,2,2);
surf(vmMesh, pMesh, fallVpTableValues, 'EdgeColor', 'none');
grid on; box on; view(45,25);
xlabel('Vm (rpm)');
ylabel('Pressure P (mmHg)');
zlabel('|Vp| = |dP/dt|');
title('Original Fall Surface');
colorbar;

%% =========================================================
%  8) Figure 3: three fall surfaces
% ==========================================================
figure('Name','Figure 3: Three Fall Surfaces','Color','w');

subplot(1,3,1);
surf(vmMesh, pMesh, fallVpTableValues, 'EdgeColor', 'none');
grid on; box on; view(45,25);
xlabel('Vm (rpm)');
ylabel('Pressure P (mmHg)');
zlabel('|Vp| = |dP/dt|');
title('Original Fall Surface');
colorbar;

subplot(1,3,2);
surf(vmMesh, pMesh, idealFallVpTableValues, 'EdgeColor', 'none');
grid on; box on; view(45,25);
xlabel('Vm (rpm)');
ylabel('Pressure P (mmHg)');
zlabel('|Vp| = |dP/dt|');
title('Ideal Symmetric Fall');
colorbar;

subplot(1,3,3);
surf(vmMesh, pMesh, restFallVpTableValues, 'EdgeColor', 'none');
grid on; box on; view(45,25);
xlabel('Vm (rpm)');
ylabel('Pressure P (mmHg)');
zlabel('|Vp| = |dP/dt|');
title('Resting Fall (2x time)');
colorbar;

%% =========================================================
%  8) Original experimental simulation
% ==========================================================
N = round(simDuration / dt) + 1;
tSim = (0:N-1)' * dt;

P_sim_original  = zeros(N,1);
RPM_sim_original = zeros(N,1);
Vp_sim_original = zeros(N,1);
dirSim_original = zeros(N,1);

P_sim_original(1) = P0;
RPM_sim_original(1) = RPM_min;
dirSim_original(1) = +1;

for k = 1:N-1
    P_now = P_sim_original(k);
    RPM_now = RPM_sim_original(k);
    dir_now = dirSim_original(k);

    if clampVmToTrainingRange
        RPM_now = min(max(RPM_now, trainingVmMin), trainingVmMax);
    end

    if dir_now == +1
        Vp_now = VpRiseFcn(P_now, RPM_now, P_low, P_high);
    else
        Vp_now = -VpFallFcn(P_now, RPM_now, P_low, P_high);
    end
    if ~isfinite(Vp_now)
        Vp_now = 0;
    end
    Vp_sim_original(k) = Vp_now;

    P_next = P_now + Vp_now * dt;
    dir_next = dir_now;
    if P_next >= P_high
        P_next = P_high;
        dir_next = -1;
    elseif P_next <= P_low
        P_next = P_low;
        dir_next = +1;
    end

    posNext = (P_next - P_low) / max(P_high - P_low, eps);
    posNext = min(max(posNext, 0), 1);
    RPM_next = pressureScheduledSpeed(posNext, RPM_min, RPM_max, speedCurveExponent);

    if clampVmToTrainingRange
        RPM_next = min(max(RPM_next, trainingVmMin), trainingVmMax);
    end

    P_sim_original(k+1) = P_next;
    RPM_sim_original(k+1) = RPM_next;
    dirSim_original(k+1) = dir_next;
end

if N > 1
    Vp_sim_original(end) = Vp_sim_original(end-1);
end

%% =========================================================
%  9) Ideal symmetric simulation
% ==========================================================
P_sim_ideal  = zeros(N,1);
RPM_sim_ideal = zeros(N,1);
Vp_sim_ideal = zeros(N,1);
dirSim_ideal = zeros(N,1);

P_sim_ideal(1) = P0;
RPM_sim_ideal(1) = RPM_min;
dirSim_ideal(1) = +1;

for k = 1:N-1
    P_now = P_sim_ideal(k);
    RPM_now = RPM_sim_ideal(k);
    dir_now = dirSim_ideal(k);

    if clampVmToTrainingRange
        RPM_now = min(max(RPM_now, trainingVmMin), trainingVmMax);
    end

    if dir_now == +1
        Vp_now = VpRiseFcn(P_now, RPM_now, P_low, P_high);
    else
        Vp_now = -VpFallIdealFcn(P_now, RPM_now, P_low, P_high);
    end
    if ~isfinite(Vp_now)
        Vp_now = 0;
    end
    Vp_sim_ideal(k) = Vp_now;

    P_next = P_now + Vp_now * dt;
    dir_next = dir_now;
    if P_next >= P_high
        P_next = P_high;
        dir_next = -1;
    elseif P_next <= P_low
        P_next = P_low;
        dir_next = +1;
    end

    posNext = (P_next - P_low) / max(P_high - P_low, eps);
    posNext = min(max(posNext, 0), 1);
    RPM_next = pressureScheduledSpeed(posNext, RPM_min, RPM_max, speedCurveExponent);

    if clampVmToTrainingRange
        RPM_next = min(max(RPM_next, trainingVmMin), trainingVmMax);
    end

    P_sim_ideal(k+1) = P_next;
    RPM_sim_ideal(k+1) = RPM_next;
    dirSim_ideal(k+1) = dir_next;
end

if N > 1
    Vp_sim_ideal(end) = Vp_sim_ideal(end-1);
end

%% =========================================================
%  10) Resting heartbeat simulation
% ==========================================================
P_sim_rest  = zeros(N,1);
RPM_sim_rest = zeros(N,1);
Vp_sim_rest = zeros(N,1);
dirSim_rest = zeros(N,1);

P_sim_rest(1) = P0;
RPM_sim_rest(1) = RPM_min;
dirSim_rest(1) = +1;

for k = 1:N-1
    P_now = P_sim_rest(k);
    RPM_now = RPM_sim_rest(k);
    dir_now = dirSim_rest(k);

    if clampVmToTrainingRange
        RPM_now = min(max(RPM_now, trainingVmMin), trainingVmMax);
    end

    if dir_now == +1
        Vp_now = VpRiseFcn(P_now, RPM_now, P_low, P_high);
    else
        Vp_now = -VpFallRestFcn(P_now, RPM_now, P_low, P_high);
    end
    if ~isfinite(Vp_now)
        Vp_now = 0;
    end
    Vp_sim_rest(k) = Vp_now;

    P_next = P_now + Vp_now * dt;
    dir_next = dir_now;
    if P_next >= P_high
        P_next = P_high;
        dir_next = -1;
    elseif P_next <= P_low
        P_next = P_low;
        dir_next = +1;
    end

    posNext = (P_next - P_low) / max(P_high - P_low, eps);
    posNext = min(max(posNext, 0), 1);
    RPM_next = pressureScheduledSpeed(posNext, RPM_min, RPM_max, speedCurveExponent);

    if clampVmToTrainingRange
        RPM_next = min(max(RPM_next, trainingVmMin), trainingVmMax);
    end

    P_sim_rest(k+1) = P_next;
    RPM_sim_rest(k+1) = RPM_next;
    dirSim_rest(k+1) = dir_next;
end

if N > 1
    Vp_sim_rest(end) = Vp_sim_rest(end-1);
end

%% =========================================================
%  12) Figure 4: original simulation
% ==========================================================
figure('Name','Figure 4: Original Experimental Simulation','Color','w');

subplot(4,1,1);
plot(tSim, P_sim_original, 'k', 'LineWidth', 1.8); hold on; grid on; box on;
yline(P_low, '--', 'LowLimit', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
yline(P_high, '--', 'HighLimit', 'Color', [0.93 0.69 0.13], 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Pressure P (mmHg)');
title('Original Experimental Pressure');
ylim([min(P_sim_original) - pressurePlotMargin, max(P_sim_original) + pressurePlotMargin]);

subplot(4,1,2);
plot(tSim, RPM_sim_original, 'b', 'LineWidth', 1.5); grid on; box on;
xlabel('Time (s)');
ylabel('RPM');
title('Motor Speed');
ylim([min(RPM_sim_original) - vmPlotMargin, max(RPM_sim_original) + vmPlotMargin]);

subplot(4,1,3);
plot(tSim, Vp_sim_original, 'r', 'LineWidth', 1.5); grid on; box on;
xlabel('Time (s)');
ylabel('Vp');
title('Pressure Change Rate');
ylim([min(Vp_sim_original) - vpPlotMargin, max(Vp_sim_original) + vpPlotMargin]);

subplot(4,1,4);
plot(tSim, dirSim_original, 'm', 'LineWidth', 1.5); grid on; box on;
xlabel('Time (s)');
ylabel('Direction');
ylim([-1.1 1.1]);
yticks([-1 0 1]);
title('+1 Pressurize, -1 Depressurize');

%% =========================================================
%  13) Figure 5: ideal symmetric simulation
% ==========================================================
figure('Name','Figure 5: Ideal Symmetric Simulation','Color','w');

subplot(4,1,1);
plot(tSim, P_sim_ideal, 'k', 'LineWidth', 1.8); hold on; grid on; box on;
yline(P_low, '--', 'LowLimit', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
yline(P_high, '--', 'HighLimit', 'Color', [0.93 0.69 0.13], 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Pressure P (mmHg)');
title('Ideal Symmetric Pressure');
ylim([min(P_sim_ideal) - pressurePlotMargin, max(P_sim_ideal) + pressurePlotMargin]);

subplot(4,1,2);
plot(tSim, RPM_sim_ideal, 'b', 'LineWidth', 1.5); grid on; box on;
xlabel('Time (s)');
ylabel('RPM');
title('Motor Speed');
ylim([min(RPM_sim_ideal) - vmPlotMargin, max(RPM_sim_ideal) + vmPlotMargin]);

subplot(4,1,3);
plot(tSim, Vp_sim_ideal, 'r', 'LineWidth', 1.5); grid on; box on;
xlabel('Time (s)');
ylabel('Vp');
title('Pressure Change Rate');
ylim([min(Vp_sim_ideal) - vpPlotMargin, max(Vp_sim_ideal) + vpPlotMargin]);

subplot(4,1,4);
plot(tSim, dirSim_ideal, 'm', 'LineWidth', 1.5); grid on; box on;
xlabel('Time (s)');
ylabel('Direction');
ylim([-1.1 1.1]);
yticks([-1 0 1]);
title('+1 Pressurize, -1 Depressurize');

%% =========================================================
%  14) Figure 6: resting heartbeat simulation
% ==========================================================
figure('Name','Figure 6: Resting Heartbeat Simulation','Color','w');

subplot(4,1,1);
plot(tSim, P_sim_rest, 'k', 'LineWidth', 1.8); hold on; grid on; box on;
yline(P_low, '--', 'LowLimit', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
yline(P_high, '--', 'HighLimit', 'Color', [0.93 0.69 0.13], 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Pressure P (mmHg)');
title('Resting Heartbeat Pressure');
ylim([min(P_sim_rest) - pressurePlotMargin, max(P_sim_rest) + pressurePlotMargin]);

subplot(4,1,2);
plot(tSim, RPM_sim_rest, 'b', 'LineWidth', 1.5); grid on; box on;
xlabel('Time (s)');
ylabel('RPM');
title('Motor Speed');
ylim([min(RPM_sim_rest) - vmPlotMargin, max(RPM_sim_rest) + vmPlotMargin]);

subplot(4,1,3);
plot(tSim, Vp_sim_rest, 'r', 'LineWidth', 1.5); grid on; box on;
xlabel('Time (s)');
ylabel('Vp');
title('Pressure Change Rate');
ylim([min(Vp_sim_rest) - vpPlotMargin, max(Vp_sim_rest) + vpPlotMargin]);

subplot(4,1,4);
plot(tSim, dirSim_rest, 'm', 'LineWidth', 1.5); grid on; box on;
xlabel('Time (s)');
ylabel('Direction');
ylim([-1.1 1.1]);
yticks([-1 0 1]);
title('+1 Pressurize, -1 Depressurize');

%% =========================================================
%  15) Figure 7: original vs ideal
% ==========================================================
figure('Name','Figure 7: Original vs Ideal Compare','Color','w');

subplot(2,1,1);
plot(tSim, P_sim_original, 'k', 'LineWidth', 1.6); hold on; grid on; box on;
plot(tSim, P_sim_ideal, '--', 'Color', [0 0.45 0.74], 'LineWidth', 1.6);
yline(P_low, '--', 'LowLimit', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
yline(P_high, '--', 'HighLimit', 'Color', [0.93 0.69 0.13], 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Pressure P (mmHg)');
title('Pressure Comparison');
legend('Original experimental','Ideal symmetric','LowLimit','HighLimit','Location','best');
ylim([min([P_sim_original; P_sim_ideal]) - pressurePlotMargin, ...
    max([P_sim_original; P_sim_ideal]) + pressurePlotMargin]);

subplot(2,1,2);
plot(tSim, Vp_sim_original, 'r', 'LineWidth', 1.5); hold on; grid on; box on;
plot(tSim, Vp_sim_ideal, '--', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Vp');
title('Pressure Change Rate Comparison');
legend('Original experimental','Ideal symmetric','Location','best');
ylim([min([Vp_sim_original; Vp_sim_ideal]) - vpPlotMargin, ...
    max([Vp_sim_original; Vp_sim_ideal]) + vpPlotMargin]);

%% =========================================================
%  16) Figure 8: original vs resting
% ==========================================================
figure('Name','Figure 8: Original vs Resting Compare','Color','w');

subplot(2,1,1);
plot(tSim, P_sim_original, 'k', 'LineWidth', 1.6); hold on; grid on; box on;
plot(tSim, P_sim_rest, '--', 'Color', [0 0.45 0.74], 'LineWidth', 1.6);
yline(P_low, '--', 'LowLimit', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
yline(P_high, '--', 'HighLimit', 'Color', [0.93 0.69 0.13], 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Pressure P (mmHg)');
title('Pressure Comparison');
legend('Original experimental','Resting heartbeat','LowLimit','HighLimit','Location','best');
ylim([min([P_sim_original; P_sim_rest]) - pressurePlotMargin, ...
    max([P_sim_original; P_sim_rest]) + pressurePlotMargin]);

subplot(2,1,2);
plot(tSim, Vp_sim_original, 'r', 'LineWidth', 1.5); hold on; grid on; box on;
plot(tSim, Vp_sim_rest, '--', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Vp');
title('Pressure Change Rate Comparison');
legend('Original experimental','Resting heartbeat','Location','best');
ylim([min([Vp_sim_original; Vp_sim_rest]) - vpPlotMargin, ...
    max([Vp_sim_original; Vp_sim_rest]) + vpPlotMargin]);

%% =========================================================
%  17) Export
% ==========================================================
assignin('base', 'riseVpTable', riseVpTable);
assignin('base', 'fallVpTable', fallVpTable);
assignin('base', 'idealFallVpTable', idealFallVpTable);
assignin('base', 'restFallVpTable', restFallVpTable);
assignin('base', 'pressureCalibrationSummary', calibrationSummary);
assignin('base', 'pressureCsvGain', pressureCsvGain);
assignin('base', 'pressureCsvOffset', pressureCsvOffset);
assignin('base', 'pressureGridModel', pressureGrid);
assignin('base', 'vmGridModel', vmGrid);
assignin('base', 'rpmGridModel', vmGrid);

assignin('base', 'P_sim_original', P_sim_original);
assignin('base', 'RPM_sim_original', RPM_sim_original);
assignin('base', 'Vp_sim_original', Vp_sim_original);
assignin('base', 'dirSim_original', dirSim_original);

assignin('base', 'P_sim_ideal', P_sim_ideal);
assignin('base', 'RPM_sim_ideal', RPM_sim_ideal);
assignin('base', 'Vp_sim_ideal', Vp_sim_ideal);
assignin('base', 'dirSim_ideal', dirSim_ideal);

assignin('base', 'P_sim_rest', P_sim_rest);
assignin('base', 'RPM_sim_rest', RPM_sim_rest);
assignin('base', 'Vp_sim_rest', Vp_sim_rest);
assignin('base', 'dirSim_rest', dirSim_rest);

if saveTables
    writetable(riseVpTable, fullfile(outputFolder, 'rise_RPM_p_vp_table.csv'));
    writetable(fallVpTable, fullfile(outputFolder, 'fall_RPM_p_vp_table.csv'));
    writetable(idealFallVpTable, fullfile(outputFolder, 'ideal_symmetric_fall_RPM_p_vp_table.csv'));
    writetable(restFallVpTable, fullfile(outputFolder, 'resting_heartbeat_fall_RPM_p_vp_table.csv'));
    writetable(calibrationSummary, fullfile(outputFolder, 'pressure_calibration_summary.csv'));
end

disp('======================================');
disp('Original + ideal symmetric + resting heartbeat comparison model built');
disp('Pressure calibration: real mmHg = (CSV pressure - offset) / gain');
disp(['CSV low reference = ', num2str(csvLowRef), ', CSV high reference = ', num2str(csvHighRef)]);
disp(['gain = ', num2str(pressureCsvGain), ', offset = ', num2str(pressureCsvOffset)]);
disp(['Resting relaxation-to-contraction time ratio = ', num2str(restRelaxToContractTimeRatio)]);
disp(['Resting relaxation rate scale = ', num2str(restRelaxRateScale)]);
disp('======================================');

%% =========================================================
%  Local functions
% ==========================================================
function data = getColumnData(T, candidates)
    names = T.Properties.VariableNames;
    data = [];
    for i = 1:numel(candidates)
        idx = find(strcmpi(names, candidates{i}), 1);
        if ~isempty(idx)
            data = T.(names{idx});
            return;
        end
    end
    error(['Missing column: ', strjoin(candidates, ' / ')]);
end

function speed = parseSpeedFromFilename(nameStr)
    speed = NaN;
    tok = regexp(nameStr, 'speed_(\d+)', 'tokens', 'once');
    if ~isempty(tok)
        speed = str2double(tok{1});
    end
end

function [csvLowRef, csvHighRef, summaryTable] = estimatePressureCalibration(files, smoothWindow, minProminence)
    speeds = zeros(numel(files),1);
    meanLows = zeros(numel(files),1);
    meanHighs = zeros(numel(files),1);
    lowCounts = zeros(numel(files),1);
    highCounts = zeros(numel(files),1);

    for i = 1:numel(files)
        fullName = fullfile(files(i).folder, files(i).name);
        T = readtable(fullName);
        Pcsv = getColumnData(T, {'Pressure','pressure'});
        Psm = smoothdata(Pcsv, 'movmean', smoothWindow);

        isHigh = islocalmax(Psm, 'MinProminence', minProminence);
        isLow = islocalmin(Psm, 'MinProminence', minProminence);

        if nnz(isLow) < 2
            lowValues = min(Pcsv);
        else
            lowValues = Psm(isLow);
        end

        if nnz(isHigh) < 2
            highValues = max(Pcsv);
        else
            highValues = Psm(isHigh);
        end

        speeds(i) = parseSpeedFromFilename(files(i).name);
        meanLows(i) = mean(lowValues, 'omitnan');
        meanHighs(i) = mean(highValues, 'omitnan');
        lowCounts(i) = numel(lowValues);
        highCounts(i) = numel(highValues);
    end

    csvLowRef = mean(meanLows, 'omitnan');
    csvHighRef = mean(meanHighs, 'omitnan');

    summaryTable = table(speeds, meanLows, meanHighs, lowCounts, highCounts, ...
        'VariableNames', {'PWM_Command','MeanLow_CSV','MeanHigh_CSV','LowCount','HighCount'});
end

function factorCurve = buildFactorCurveMonotonic(xData, factorData, xGrid, smoothWindow, modeName)
    factorCurve = nan(size(xGrid));

    if numel(xGrid) > 1
        halfStep = (xGrid(2) - xGrid(1)) / 2;
    else
        halfStep = 0.02;
    end

    for i = 1:numel(xGrid)
        x0 = xGrid(i);
        mask = abs(xData - x0) <= halfStep;
        if any(mask)
            factorCurve(i) = median(factorData(mask), 'omitnan');
        end
    end

    good = isfinite(factorCurve);
    if nnz(good) < 4
        factorCurve(:) = 1;
    else
        factorCurve = interp1(xGrid(good), factorCurve(good), xGrid, 'pchip', 'extrap');
    end

    factorCurve = smoothdata(factorCurve, 'movmean', smoothWindow);

    switch string(modeName)
        case "decreasing"
            for i = numel(factorCurve)-1:-1:1
                factorCurve(i) = max(factorCurve(i), factorCurve(i+1));
            end
        case "increasing"
            for i = 2:numel(factorCurve)
                factorCurve(i) = max(factorCurve(i), factorCurve(i-1));
            end
    end

    midMask = xGrid >= 0.35 & xGrid <= 0.65;
    midMean = mean(factorCurve(midMask), 'omitnan');
    if isfinite(midMean) && midMean > 0
        factorCurve = factorCurve / midMean;
    end

    factorCurve = max(factorCurve, 0.05);
end

function speedNow = pressureScheduledSpeed(posNorm, speedMin, speedMax, expo)
    posNorm = min(max(posNorm,0),1);
    centerBoost = 1 - abs(2*posNorm - 1)^expo;
    centerBoost = min(max(centerBoost,0),1);
    speedNow = speedMin + centerBoost * (speedMax - speedMin);
end
