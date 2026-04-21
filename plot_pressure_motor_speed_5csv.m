clear; clc; close all;

%% =========================================================
%  Plot calibrated pressure and real motor speed from five CSV files
% ==========================================================

dataFiles = [
    "C:\Users\13168\Desktop\毕业设计\控制系统代码\test2\heart_data_speed_215_20260419_090053.csv"
    "C:\Users\13168\Desktop\毕业设计\控制系统代码\test2\heart_data_speed_225_20260419_090231.csv"
    "C:\Users\13168\Desktop\毕业设计\控制系统代码\test2\heart_data_speed_235_20260419_090401.csv"
    "C:\Users\13168\Desktop\毕业设计\控制系统代码\test2\heart_data_speed_245_20260419_090524.csv"
    "C:\Users\13168\Desktop\毕业设计\控制系统代码\test2\heart_data_speed_255_20260419_090648.csv"
];

numFiles = numel(dataFiles);

%% =========================================================
%  Calibration settings
% ==========================================================

% Motor calibration:
% The file name speed is PWM command. The real motor speed is approximately:
% RPM = PWM - 55.
pwmToRpmOffset = 55;

% Pressure calibration:
% The CSV pressure is amplified and zero-shifted. The real measured pressure
% range corresponding to the five CSV datasets is 30~45 mmHg.
trueLowPressureMmHg = 30;
trueHighPressureMmHg = 45;

calibrationSmoothWindow = 9;
calibrationMinProminence = 10;

[csvLowRef, csvHighRef, calibrationSummary] = estimatePressureCalibration( ...
    dataFiles, calibrationSmoothWindow, calibrationMinProminence);

pressureCsvGain = (csvHighRef - csvLowRef) / (trueHighPressureMmHg - trueLowPressureMmHg);
pressureCsvOffset = csvLowRef - pressureCsvGain * trueLowPressureMmHg;

csvPressureToMmHg = @(Pcsv) (Pcsv - pressureCsvOffset) ./ pressureCsvGain;
pwmToRpm = @(pwm) sign(pwm) .* max(abs(pwm) - pwmToRpmOffset, 0);

%% =========================================================
%  Read and convert data
% ==========================================================

expData = struct([]);

for i = 1:numFiles
    fileName = dataFiles(i);

    if ~isfile(fileName)
        error("File not found: %s", fileName);
    end

    T = readtable(fileName, "VariableNamingRule", "preserve");

    pwmValue = parseSpeedFromFilename(fileName);
    if isnan(pwmValue)
        pwmValue = median(abs(T.Motor1(T.Motor1 ~= 0)), "omitnan");
    end
    rpmValue = pwmValue - pwmToRpmOffset;

    time = T.Time_s;
    time = time - time(1);  % Start each experiment from 0 s.

    expData(i).fileName = fileName;
    expData(i).pwm = pwmValue;
    expData(i).rpm = rpmValue;
    expData(i).time = time;
    expData(i).pressure = csvPressureToMmHg(T.Pressure);
    expData(i).lowLimit = csvPressureToMmHg(T.LowLimit);
    expData(i).highLimit = csvPressureToMmHg(T.HighLimit);
    expData(i).motor1 = pwmToRpm(T.Motor1);
    expData(i).motor2 = pwmToRpm(T.Motor2);
    expData(i).cycle = T.Cycle;
end

%% =========================================================
%  Figure 1: pressure comparison
% ==========================================================

colors = lines(numFiles);

figure("Name", "Pressure Comparison - Five Speeds", "Color", "w");
hold on; grid on; box on;

for i = 1:numFiles
    plot(expData(i).time, expData(i).pressure, ...
        "LineWidth", 1.4, ...
        "Color", colors(i,:), ...
        "DisplayName", sprintf("%d rpm", round(expData(i).rpm)));
end

xlabel("Time (s)");
ylabel("Pressure (mmHg)");
title("Calibrated Pressure Response at Different Motor Speeds");
legend("Location", "best");

%% =========================================================
%  Figure 2: motor speed comparison
% ==========================================================

figure("Name", "Motor Speed Comparison - Five Speeds", "Color", "w");
hold on; grid on; box on;

for i = 1:numFiles
    plot(expData(i).time, expData(i).motor1, ...
        "LineWidth", 1.2, ...
        "Color", colors(i,:), ...
        "DisplayName", sprintf("Motor1 %d rpm", round(expData(i).rpm)));
end

yline(0, "k:");
xlabel("Time (s)");
ylabel("Motor Speed (rpm)");
title("Motor Speed at Different Motor Speeds");
legend("Location", "best");

%% =========================================================
%  Figure 3: pressure and motor speed for each experiment
% ==========================================================

figure("Name", "Pressure and Motor Speed Details", "Color", "w");
tiledlayout(numFiles, 2, "TileSpacing", "compact", "Padding", "compact");

for i = 1:numFiles
    nexttile;
    plot(expData(i).time, expData(i).pressure, "k", "LineWidth", 1.2);
    hold on; grid on; box on;
    plot(expData(i).time, expData(i).lowLimit, "--", "Color", [0.85 0.33 0.10], "LineWidth", 1.0);
    plot(expData(i).time, expData(i).highLimit, "--", "Color", [0.93 0.69 0.13], "LineWidth", 1.0);
    xlabel("Time (s)");
    ylabel("Pressure (mmHg)");
    title(sprintf("Pressure - %d rpm", round(expData(i).rpm)));

    nexttile;
    plot(expData(i).time, expData(i).motor1, "b", "LineWidth", 1.2);
    hold on; grid on; box on;
    plot(expData(i).time, expData(i).motor2, "r--", "LineWidth", 1.0);
    yline(0, "k:");
    xlabel("Time (s)");
    ylabel("Motor Speed (rpm)");
    title(sprintf("Motor Speed - %d rpm", round(expData(i).rpm)));
    legend("Motor1", "Motor2", "Location", "best");
end

%% =========================================================
%  Output summary
% ==========================================================

fprintf("\nLoaded %d CSV files.\n", numFiles);
fprintf("Pressure calibration: real mmHg = (CSV pressure - %.4f) / %.4f\n", ...
    pressureCsvOffset, pressureCsvGain);
fprintf("CSV low/high reference: %.4f / %.4f\n", csvLowRef, csvHighRef);

for i = 1:numFiles
    fprintf("PWM %d -> %d rpm: %.2f s, pressure %.2f ~ %.2f mmHg, cycle %d ~ %d\n", ...
        round(expData(i).pwm), ...
        round(expData(i).rpm), ...
        expData(i).time(end), ...
        min(expData(i).pressure), ...
        max(expData(i).pressure), ...
        min(expData(i).cycle), ...
        max(expData(i).cycle));
end

assignin("base", "expData", expData);
assignin("base", "pressureCalibrationSummary", calibrationSummary);
assignin("base", "pressureCsvGain", pressureCsvGain);
assignin("base", "pressureCsvOffset", pressureCsvOffset);

%% =========================================================
%  Local functions
% ==========================================================

function speedValue = parseSpeedFromFilename(fileName)
    speedValue = NaN;
    token = regexp(char(fileName), 'speed_(\d+)', 'tokens', 'once');
    if ~isempty(token)
        speedValue = str2double(token{1});
    end
end

function [csvLowRef, csvHighRef, summaryTable] = estimatePressureCalibration(dataFiles, smoothWindow, minProminence)
    speeds = zeros(numel(dataFiles),1);
    meanLows = zeros(numel(dataFiles),1);
    meanHighs = zeros(numel(dataFiles),1);
    lowCounts = zeros(numel(dataFiles),1);
    highCounts = zeros(numel(dataFiles),1);

    for i = 1:numel(dataFiles)
        T = readtable(dataFiles(i), "VariableNamingRule", "preserve");
        Pcsv = T.Pressure;
        Psm = smoothdata(Pcsv, "movmean", smoothWindow);

        isHigh = islocalmax(Psm, "MinProminence", minProminence);
        isLow = islocalmin(Psm, "MinProminence", minProminence);

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

        speeds(i) = parseSpeedFromFilename(dataFiles(i));
        meanLows(i) = mean(lowValues, "omitnan");
        meanHighs(i) = mean(highValues, "omitnan");
        lowCounts(i) = numel(lowValues);
        highCounts(i) = numel(highValues);
    end

    csvLowRef = mean(meanLows, "omitnan");
    csvHighRef = mean(meanHighs, "omitnan");

    summaryTable = table(speeds, meanLows, meanHighs, lowCounts, highCounts, ...
        'VariableNames', {'PWM_Command','MeanLow_CSV','MeanHigh_CSV','LowCount','HighCount'});
end
