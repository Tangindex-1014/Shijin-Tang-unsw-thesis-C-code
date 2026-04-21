clear; clc; close all;

% =========================
% Configurable parameters
% =========================
port = "COM3";        % Arduino port
baudRate = 115200;    % Must match Arduino
runTime = 60;         % Max MATLAB run time in seconds
maxPoints = 5000;     % Max number of buffered points
startupWait = 2;      % Wait after opening serial
xWindow = 10;         % Visible x-axis window in seconds
saveCsv = true;       % Export CSV after the run
speedTag = "255";     % Change this when you change motorSpeed in Arduino
outputDir = fileparts(mfilename('fullpath'));

% =========================
% Check serial ports
% =========================
disp("Available serial ports:");
disp(serialportlist("available"));

% =========================
% Open serial port
% =========================
s = serialport(port, baudRate);
configureTerminator(s, "LF");
s.Timeout = 2;
flush(s);

disp("Serial opened successfully.");
pause(startupWait);
flush(s);

% =========================
% Send start command to Arduino
% =========================
disp("Sending start command to Arduino...");
writeline(s, "S");

% =========================
% Figure 1: Pressure
% =========================
fig1 = figure('Name','Pressure Monitor', ...
              'NumberTitle','off', ...
              'Color','w');

axP = axes(fig1);
hold(axP, 'on');
grid(axP, 'on');
xlabel(axP, 'Time (s)');
ylabel(axP, 'Pressure');
title(axP, 'Pressure / LowLimit / HighLimit');

hPressure = animatedline(axP, 'LineWidth', 1.5);
hLow      = animatedline(axP, 'LineWidth', 1.2);
hHigh     = animatedline(axP, 'LineWidth', 1.2);
legend(axP, {'Pressure','LowLimit','HighLimit'}, 'Location','best');

% =========================
% Figure 2: Motors
% =========================
fig2 = figure('Name','Motor Monitor', ...
              'NumberTitle','off', ...
              'Color','w');

tiledlayout(fig2, 2, 1);

axM1 = nexttile;
hMotor1 = animatedline(axM1, 'LineWidth', 1.5);
grid(axM1, 'on');
xlabel(axM1, 'Time (s)');
ylabel(axM1, 'Motor1');
title(axM1, 'Motor1 Speed');

axM2 = nexttile;
hMotor2 = animatedline(axM2, 'LineWidth', 1.5);
grid(axM2, 'on');
xlabel(axM2, 'Time (s)');
ylabel(axM2, 'Motor2');
title(axM2, 'Motor2 Speed');

% =========================
% Data buffer
% =========================
timeData = [];
pressureData = [];
lowData = [];
highData = [];
motor1Data = [];
motor2Data = [];
cycleData = [];

disp("Start live plotting...");

tic;

% =========================
% Real-time read and plot
% =========================
while ishandle(fig1) && ishandle(fig2) && toc < runTime
    try
        rawLine = readline(s);
        line = strtrim(string(rawLine));

        if strlength(line) == 0
            continue;
        end

        if line == "WAITING_FOR_START"
            disp("Arduino is waiting for start.");
            continue;
        end

        if line == "START_BASELINE"
            disp("Arduino is measuring baseline pressure for 3 seconds...");
            continue;
        end

        if line == "FINISHED"
            disp("Arduino finished all cycles.");
            break;
        end

        if contains(line, "Time_ms")
            disp("Header received: " + line);
            continue;
        end

        parts = split(line, ",");

        if numel(parts) ~= 7
            continue;
        end

        vals = str2double(parts);

        if any(isnan(vals))
            continue;
        end

        t    = vals(1) / 1000;
        p    = vals(2);
        low  = vals(3);
        high = vals(4);
        m1   = vals(5);
        m2   = vals(6);
        cyc  = vals(7);

        timeData(end+1)     = t;
        pressureData(end+1) = p;
        lowData(end+1)      = low;
        highData(end+1)     = high;
        motor1Data(end+1)   = m1;
        motor2Data(end+1)   = m2;
        cycleData(end+1)    = cyc;

        if numel(timeData) > maxPoints
            timeData     = timeData(end-maxPoints+1:end);
            pressureData = pressureData(end-maxPoints+1:end);
            lowData      = lowData(end-maxPoints+1:end);
            highData     = highData(end-maxPoints+1:end);
            motor1Data   = motor1Data(end-maxPoints+1:end);
            motor2Data   = motor2Data(end-maxPoints+1:end);
            cycleData    = cycleData(end-maxPoints+1:end);
        end

        clearpoints(hPressure);
        clearpoints(hLow);
        clearpoints(hHigh);

        addpoints(hPressure, timeData, pressureData);
        addpoints(hLow,      timeData, lowData);
        addpoints(hHigh,     timeData, highData);

        clearpoints(hMotor1);
        clearpoints(hMotor2);

        addpoints(hMotor1, timeData, motor1Data);
        addpoints(hMotor2, timeData, motor2Data);

        xStart = max(0, timeData(end) - xWindow);
        xEnd   = timeData(end) + 0.5;

        xlim(axP,  [xStart, xEnd]);
        xlim(axM1, [xStart, xEnd]);
        xlim(axM2, [xStart, xEnd]);

        drawnow limitrate;

    catch ME
        disp("Read error: " + string(ME.message));
    end
end

% =========================
% Save CSV
% =========================
if isempty(timeData)
    disp("No valid data received.");
else
    if saveCsv
        timestampTag = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        outputFile = fullfile(outputDir, "heart_data_speed_" + speedTag + "_" + timestampTag + ".csv");

        T = table(timeData(:), pressureData(:), lowData(:), highData(:), ...
                  motor1Data(:), motor2Data(:), cycleData(:), ...
                  'VariableNames', {'Time_s','Pressure','LowLimit','HighLimit', ...
                                    'Motor1','Motor2','Cycle'});

        writetable(T, outputFile);
        disp("CSV saved to: " + outputFile);
    end
end

% =========================
% Close serial
% =========================
clear s;
disp("Finished.");
