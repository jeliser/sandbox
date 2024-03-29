function quick_plot(name, struct)
try

    %%
    %figure
    figure('Name', name, 'Units','normalized','Position',[0 0 1 1]);
    subplot(2, 1, 1);
    yyaxis left;
    plot([struct.ED01_PRESS, struct.ED02_PRESS, struct.ED03_PRESS]);
    yyaxis right;
    plot([struct.ED01_CURR, struct.ED02_CURR, struct.ED03_CURR]);
    title('Descent Thrusters');
    legend({'ED01_PRESS', 'ED02_PRESS', 'ED03_PRESS', 'ED01_CURR', 'ED02_CURR', 'ED03_CURR'}, 'Interpreter', 'none');
    
    subplot(2, 1, 2);
    plot([struct.THRUSTERS_FEED_PRESS, struct.PROP_TANKS_PRESS, struct.N2_TANKS_PRESS, struct.N2_PROP_TANKS_FEED_PRESS, struct.N2_CTRL_VALVE_FEED_PRESS]);
    title('System Pressures');
    legend({'THRUSTERS_FEED_PRESS', 'N2_PROP_TANKS_PRESS', 'N2_TANKS_PRESS', 'N2_PROP_TANKS_FEED_PRESS', 'N2_CTRL_VALVE_FEED_PRESS'}, 'Interpreter', 'none');
    savefig(gcf, sprintf('output/%s_System.fig', name), 'compact');
    saveas(gcf, sprintf('output/%s_System.png', name));

    %%
    start = max(1, find(struct.VEHICLE_STATE==6, 1, 'first'));
    if isempty(start)
        start = 1;
    end

    stop  = min(find(struct.VEHICLE_STATE==7, 1, 'first'), numel(struct.VEHICLE_STATE));
    if isempty(stop)
        stop = numel(struct.UNIX_TIME);
    end

    %figure;
    figure('Name', name, 'Units','normalized','Position',[0 0 1 1]);
    subplot(3, 1, 1);
    yyaxis left;
    data = [struct.ED01_PRESS, struct.ED02_PRESS, struct.ED03_PRESS];
    plot(data(start:min(stop+100, numel(struct.UNIX_TIME)), :));
    yyaxis right;
    data = [struct.ED01_CURR, struct.ED02_CURR, struct.ED03_CURR];
    plot(data(start:min(stop+100, numel(struct.UNIX_TIME)), :));
    title('Descent Thrusters');
    legend({'ED01_PRESS', 'ED02_PRESS', 'ED03_PRESS', 'ED01_CURR', 'ED02_CURR', 'ED03_CURR'}, 'Interpreter', 'none');
    
    subplot(3, 1, 2);
    data = [struct.THRUSTERS_FEED_PRESS, struct.PROP_TANKS_PRESS, struct.N2_TANKS_PRESS, struct.N2_PROP_TANKS_FEED_PRESS, struct.N2_CTRL_VALVE_FEED_PRESS];
    plot(data(start:min(stop+100, numel(struct.UNIX_TIME)), :));
    title('System Pressures');
    legend({'THRUSTERS_FEED_PRESS', 'N2_PROP_TANKS_PRESS', 'N2_TANKS_PRESS', 'N2_PROP_TANKS_FEED_PRESS', 'N2_CTRL_VALVE_FEED_PRESS'}, 'Interpreter', 'none');

    subplot(3, 1, 3);
    data = [struct.SC_POS_0, struct.SC_POS_1, struct.SC_POS_2, struct.SC_VEL_0, struct.SC_VEL_1, struct.SC_VEL_2, struct.SC_ACC_0, struct.SC_ACC_1, struct.SC_ACC_2, struct.SC_ALT];
    plot(data(start:min(stop+100, numel(struct.UNIX_TIME)), :));
    title('Navigation');
    legend({'SC_POS_0', 'SC_POS_1', 'SC_POS_2', 'SC_VEL_0', 'SC_VEL_1', 'SC_VEL_2', 'SC_ACC_0', 'SC_ACC_1', 'SC_ACC_2', 'SC_ALT'}, 'Interpreter', 'none');

    savefig(gcf, sprintf('output/%s_Navigation.fig', name), 'compact');
    saveas(gcf, sprintf('output/%s_Navigation.png', name));

    %figure;
    %plot(struct.VEHICLE_STATE);


    %figure;
    %plot([struct.GNCT_XCM_0, struct.GNCT_XCM_1, struct.GNCT_XCM_2, struct.GNCT_XCM_3, struct.GNCT_XCM_4, struct.GNCT_XCM_5, struct.GNCT_XCM_6, struct.GNCT_XCM_7, struct.GNCT_XCM_8, struct.GNCT_XCM_9]);
catch e
    getReport(e)
end

close all;
