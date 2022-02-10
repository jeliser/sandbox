function quick_plot(struct)
    close all;

    %%
    %figure
    figure('Units','normalized','Position',[0 0 1 1]);
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

    %%
    start = find(struct.VEHICLE_STATE==6, 1, 'first');
    stop  = find(struct.VEHICLE_STATE==7, 1, 'first');

    %figure;
    figure('Units','normalized','Position',[0 0 1 1]);
    subplot(3, 1, 1);
    yyaxis left;
    data = [struct.ED01_PRESS, struct.ED02_PRESS, struct.ED03_PRESS];
    plot(data(start:stop+100, :));
    yyaxis right;
    data = [struct.ED01_CURR, struct.ED02_CURR, struct.ED03_CURR];
    plot(data(start:stop+100, :));
    title('Descent Thrusters');
    legend({'ED01_PRESS', 'ED02_PRESS', 'ED03_PRESS', 'ED01_CURR', 'ED02_CURR', 'ED03_CURR'}, 'Interpreter', 'none');
    
    subplot(3, 1, 2);
    data = [struct.THRUSTERS_FEED_PRESS, struct.PROP_TANKS_PRESS, struct.N2_TANKS_PRESS, struct.N2_PROP_TANKS_FEED_PRESS, struct.N2_CTRL_VALVE_FEED_PRESS];
    plot(data(start:stop+100, :));
    title('System Pressures');
    legend({'THRUSTERS_FEED_PRESS', 'N2_PROP_TANKS_PRESS', 'N2_TANKS_PRESS', 'N2_PROP_TANKS_FEED_PRESS', 'N2_CTRL_VALVE_FEED_PRESS'}, 'Interpreter', 'none');

    subplot(3, 1, 3);
    data = [struct.SC_POS_0, struct.SC_POS_1, struct.SC_POS_2, struct.SC_VEL_0, struct.SC_VEL_1, struct.SC_VEL_2, struct.SC_ACC_0, struct.SC_ACC_1, struct.SC_ACC_2, struct.SC_ALT];
    plot(data(start:stop+100, :));
    title('Navigation');
    legend({'SC_POS_0', 'SC_POS_1', 'SC_POS_2', 'SC_VEL_0', 'SC_VEL_1', 'SC_VEL_2', 'SC_ACC_0', 'SC_ACC_1', 'SC_ACC_2', 'SC_ALT'}, 'Interpreter', 'none');
    
    %figure;
    %plot(struct.VEHICLE_STATE);


    %figure;
    %plot([struct.GNCT_XCM_0, struct.GNCT_XCM_1, struct.GNCT_XCM_2, struct.GNCT_XCM_3, struct.GNCT_XCM_4, struct.GNCT_XCM_5, struct.GNCT_XCM_6, struct.GNCT_XCM_7, struct.GNCT_XCM_8, struct.GNCT_XCM_9]);
