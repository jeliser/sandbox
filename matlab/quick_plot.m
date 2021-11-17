function [arr, timestamps, data]= quick_plot(filename)
    data = readtable(filename);
    arr = table2array(data(:,2:end));
    timestamps = data(:,1);

    % Note and filter out the bad measurements
    out_of_limit = [];
    idx = 2;
    for val = arr(:, 2:end)
        if mean(val) > 1000
            out_of_limit = [out_of_limit, idx];
        end
        idx = idx + 1;
    end

    for idx = out_of_limit
        % The +1 is for the original "Time" column at the start of the CSV file.
        fprintf('%s is out of limit: %f\n', string(data.Properties.VariableNames(idx+1)), mean(arr(:, idx)));
    end

    %% Plot all the thermocouples
    fig = figure();
    title('All Thermocouples');
    fields = [];
    hold on;

    plot(data.HEATER01,"Color",'red','LineWidth',2.0);
    plot(data.PELTIER01,"Color",'blue','LineWidth',2.0);
    plot(data.PELTIER02,"Color",'cyan','LineWidth',2.0);

    idx = 2;
    for raw = arr(:, 2:29)
        if sum(any(idx ~= out_of_limit))
            plot(arr(:,idx));
            fields = [fields, data.Properties.VariableNames(idx+1)];
        end
        idx = idx + 1;
    end


    %legend('HEATER01', 'PELTIER01', 'PELTIER02');
    %legend(fields)
    legend(['HEATER01', 'PELTIER01', 'PELTIER02', fields]);

%     %% Plot the control thermocouples
%     fig = figure();
%     title('Control Thermocouples');
%     hold on;
%     plot(data.HEATER01,"Color",'red','LineWidth',2.0);
%     plot(data.PELTIER01,"Color",'blue','LineWidth',2.0);
%     plot(data.PELTIER02,"Color",'cyan','LineWidth',2.0);
%     legend('HEATER01', 'PELTIER01', 'PELTIER02');
    