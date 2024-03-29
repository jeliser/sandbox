function [arr, timestamps, data] = quick_plot(filename)
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

%% Image plotting
close all;
figure();

images = {
    'C:\Users\Joshua Eliser\Desktop\code\sandbox\matlab\img_focus_none.tif';
    'C:\Users\Joshua Eliser\Desktop\code\sandbox\matlab\img_focus_some.tif';
    'C:\Users\Joshua Eliser\Desktop\code\sandbox\matlab\img_focus_mostly.tif';
    };
% Reduce the range of samples to the ones around the spike.
range_lower = 1500;
range_upper = 2500;

images = {
    'C:\Users\Joshua Eliser\Desktop\code\sandbox\matlab\images\1_01_focus.png';
    'C:\Users\Joshua Eliser\Desktop\code\sandbox\matlab\images\1_02_less.png';
    'C:\Users\Joshua Eliser\Desktop\code\sandbox\matlab\images\1_03_blurry.png';
    };
range_lower = 1;
range_upper = 2000;


num_images = numel(images);
sums = [];

for i = 1:num_images
    img = imread(images{i});
    subplot(num_images, 4, ((i-1)*4)+1);
    imshow(img);

    log_shifted = log(abs(fftshift(fft2(double(img)))));
    sums = [sums; sum(log_shifted, 1)];
    subplot(num_images, 4, ((i-1)*4)+2);
    imshow(log_shifted, []);
    
    subplot(num_images, 4, ((i-1)*4)+3);
    plot(sums(i, :));

    subplot(num_images, 4, ((i-1)*4)+4);
    bar(std(sums(i, :)'));
    ylim([0 1200]);
end

% 
figure();

subplot(2, 1, 1);
hold on;
for i = 1 : size(sums, 1)
    plot(sums(i, range_lower:range_upper));
end

subplot(2, 1, 2);
sosums = [];
for sosum = 1 : size(sums, 1)
    sosums = [sosums sum(sums(i, range_lower:range_upper))];
end
bar(std(sums'));

