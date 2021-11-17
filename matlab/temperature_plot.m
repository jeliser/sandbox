
%%
close all; clear all; [data, timestamps, table] = quick_plot('C:\Users\Joshua Eliser\Desktop\EMI Testing\21_11_16__16_00_45.4704_data.csv');

%%
close all; clc
fig = figure();
fig.Position(1:2) = [10 10];
fig.Position = [100 100 1600 900];
%fig.Position = [124 -879 1600 900];

spX = 3;
spY = 3;
start_row = 10;
clear F;

image_files = dir('C:\Users\Joshua Eliser\Desktop\sample_images\*.tif')';

range = 1:30;
range = 1:size(data, 1)-10;
for idx = range
    if mod(idx, 10) == 0
        fprintf('%02d%% complete\n', round((idx/size(range, 2))*100))
    end

    clf;
    row = idx + 10;

    subplot(spX, spY, [1, 2]);
    
    x=[2:18];
    y=[-0.02 -0.01 0 0.01 0.02];
    T=data(row, 2:18);

    z=ones(size(y'))*T;
    contourf(x,y,z)
    title('Cuvette'),xlabel('Thermocouple'),zlim([-10,250]);
    % TODO:
    % Turn the x-axix labels and ticks off
    % Make the cuvette temp plot thinner to match the cuvette in reality
    % Set the color range
    colorbar
    
    subplot(spX, spY, [4, 5, 7, 8]);
    % Just cycle through the images in the directory right now, we'll do a timestamp lookup later.
    % Images are taken every 5 seconds, so only change the image every 5th one.
    iidx = mod(round(idx / 5), size(image_files, 2)) + 1;
    imshow([image_files(iidx).folder '/' image_files(iidx).name]);
    title('Microscope Image')

    subplot(spX, spY, [6]);
    hold on;
    for col = [2:18]
        plot(data(start_row:row, col));
    end
    title('Cuvette TCs')
    xlabel('Elapsed Time (secs)')

    subplot(spX, spY, [3]);
    hold on;
    plot(table.HEATER01(start_row:row),"Color",'red','LineWidth',2.0);
    plot(table.PELTIER01(start_row:row),"Color",'blue','LineWidth',2.0);
    plot(table.PELTIER02(start_row:row),"Color",'cyan','LineWidth',2.0);
    title('Control TCs')
    xlabel('Elapsed Time (secs)')

    subplot(spX, spY, [9]);
    plot(data(start_row:row, 34)); % The PT channel
    title('Pressure')
    xlabel('Elapsed Time (secs)')

    F(idx) = getframe(fig);
end

writerObj = VideoWriter('cuvette', 'MPEG-4');
writerObj.FrameRate = 25;
open(writerObj);
writeVideo(writerObj, F)
close(writerObj);