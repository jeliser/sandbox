
%%
close all; clear all; [data, timestamps, table] = quick_plot('C:\Users\Joshua Eliser\Desktop\EMI Testing\21_11_16__16_00_45.4704_data.csv');

%%
close all; clc
fig = figure();
fig.Position = [100 100 1600 900];
fig.Position = [124 -879 1600 900];

spX = 3;
spY = 3;
start_row = 10;
clear F;

%for idx = 1:size(data, 1)-10
range = 1:300;
for idx = range
    if mod(idx, 10) == 0
        fprintf('%02d%% complete\n', round((idx/size(range, 2))*100))
    end

    clf;
    row = idx + 10;

    subplot(spX, spY, [1, 4]);
    
    x=[-0.02 -0.01 0 0.01 0.02]; % x-coordinates
    y=[2:18]; % y-coordinates
    
    T=data(row, 2:18);
    z=(ones(5,1)*T)';
    contourf(x,y,z)
    title('Cuvette'),ylabel('Thermocouple'),zlim([-10,250]);
    % TODO:
    % Turn the x-axix labels and ticks off
    % Make the cuvette temp plot thinner to match the cuvette in reality
    % Set the color range
    colorbar
    
    subplot(spX, spY, [2, 3, 5, 6]);
    image(imread('sample_image.tif'));
    title('Microscope Image')
    % Change the image to greyscale

    subplot(spX, spY, [7]);
    hold on;
    for col = y
        plot(data(start_row:row, col));
    end
    title('Cuvette TCs')

    subplot(spX, spY, [8]);
    hold on;
    plot(table.HEATER01(start_row:row),"Color",'red','LineWidth',2.0);
    plot(table.PELTIER01(start_row:row),"Color",'blue','LineWidth',2.0);
    plot(table.PELTIER02(start_row:row),"Color",'cyan','LineWidth',2.0);
    title('Control TCs')

    subplot(spX, spY, [9]);
    plot(data(start_row:row, 34)); % The PT channel
    title('Pressure')

    F(idx) = getframe(fig);
end

writerObj = VideoWriter('test2.avi');
writerObj.FrameRate = 25;
open(writerObj);
writeVideo(writerObj, F)
close(writerObj);