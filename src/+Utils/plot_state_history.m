function plot_state_history(cav_set, tf, i_m,save_location)
arguments
    cav_set
    tf
    i_m
    save_location = []
end
%PLOT_STATE_HISTORY 
% f = cell(5,1);
num_vehicles = length(cav_set);
%% Extract triplet
cav_1 = cav_set(i_m-1);
cav_2 = cav_set(i_m);
cav_c = cav_set(end-1);
veh_u = cav_set(end);
%% Extract lateral maneuver times
t0_lc = cav_c.LaneChangeTime_start;
tf_lc = cav_c.LaneChangeTime_end;
%% Position History Plot
f(1) = figure('Name','Position Plot', 'Position',[100 100 700 1200]);
for i = 1:num_vehicles
    subplot(num_vehicles, 1, i)
    % Add speed history
    plot(cav_set(i).get_state_history('x'),'DisplayName','Applied')
    hold on
    % if ~ contains(cav_set(i).VehicleID,'u','IgnoreCase',true)
    %     plot(cav_set(i).get_state_history("x_ref"),'DisplayName','Reference')
    % end
    % Add terminal position
    if ~ contains(cav_set(i).VehicleID,'u','IgnoreCase',true)
        plot(tf, cav_set(i).x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
    end
    % Add time markers for lane change
    xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize',18,...
        'FontWeight','bold','LabelOrientation','horizontal')
    xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize',18,...
        'FontWeight','bold','LabelOrientation','horizontal')
    grid minor
    ylabel(strcat("$\mathbf{x_",cav_set(i).VehicleID, "}\; [m]$"),...
           'Interpreter','latex','FontSize',15,'FontWeight','bold')
    title('')
    if i ~=num_vehicles
        xlabel('',"HandleVisibility","off")
    end

end
% Add x label
xlabel("Time [s]")

%% Speed History Plot
f(2)= figure('Name','Speed Plot', 'Position',[100 100 700 1200]);
for i = 1:num_vehicles
    subplot(num_vehicles, 1, i)
    % Add position history
    plot(cav_set(i).get_state_history('v'),'DisplayName','Applied')
    hold on
    % if ~ contains(cav_set(i).VehicleID,'u','IgnoreCase',true)
    %     plot(cav_set(i).get_state_history("v_ref"),'DisplayName','Reference')
    % end
    % Add terminal position
    if ~ strcmp(cav_set(i).VehicleID,'u')
        plot(tf, cav_set(i).DesSpeed, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
    end
    % Add time markers for lane change
    xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize',18,...
        'FontWeight','bold','LabelOrientation','horizontal')
    xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize',18,...
        'FontWeight','bold','LabelOrientation','horizontal')
    % Add labels
    grid minor
    ylabel(strcat("$\mathbf{v_",cav_set(i).VehicleID, "}\; [m/s]$"),...
           'Interpreter','latex','FontSize',15,'FontWeight','bold')
    title('')
    if i ~=num_vehicles
        xlabel('',"HandleVisibility","off")
    end
end
% Add x label
xlabel("Time [s]")

%% Acceleration History Plot
f(3) = figure('Name','Control Input Plot', 'Position',[100 100 700 1200]);
for i = 1:num_vehicles
    subplot(num_vehicles, 1, i)
    % Add acceleration history
    plot(cav_set(i).get_state_history("accel"),'DisplayName','Applied')
    hold on
    % if ~ contains(cav_set(i).VehicleID,'u','IgnoreCase',true)
    %     plot(cav_set(i).get_state_history("u_ref"),'DisplayName','Reference')
    % end
    % Add time markers for lane change
    xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize',18,...
        'FontWeight','bold','LabelOrientation','horizontal')
    xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize',18,...
        'FontWeight','bold','LabelOrientation','horizontal')
    % Add labels
    grid minor
    ylabel(strcat("$\mathbf{u_",cav_set(i).VehicleID, "}\; [m/s^2]$"),...
           'Interpreter','latex','FontSize',15,'FontWeight','bold')
    title('')
    if i ~=num_vehicles
        xlabel('', "HandleVisibility","off")
    end
end
% Add x label
xlabel("Time [s]")

%% Acceleration Difference History Plot
f(4) = figure('Name','Control Input Difference Plot', 'Position',[100 100 700 1200]);
for i = 1:num_vehicles
    if ~ contains(cav_set(i).VehicleID,'u','IgnoreCase',true)
        subplot(num_vehicles, 1, i)
        % Add acceleration history
        plot(cav_set(i).get_state_history("ocp_difference"))
        hold on
        % Add time markers for lane change
        xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize',18,...
        'FontWeight','bold','LabelOrientation','horizontal')
        xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize',18,...
        'FontWeight','bold','LabelOrientation','horizontal')
        % Add labels
        grid minor
        ylabel(strcat("$\mathbf{\Delta u_",cav_set(i).VehicleID, "}\;[m/s^2]$"),...
           'Interpreter','latex','FontSize',15,'FontWeight','bold')
        title('')
        if i ~=num_vehicles
            xlabel('', "HandleVisibility","off")
        end
    end
end
% Add x label
xlabel("Time [s]")

%% Triplet plots
f(5) = figure('Name','Optimal Triplet History');
% Position subplot
subplot(2, 1, 1)
% Add position history
plot(cav_1.get_state_history("x"),"DisplayName","CAV 3")
hold on
grid minor
plot(tf, cav_1.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(tf, cav_2.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_2.get_state_history("x"),"DisplayName","CAV 4")
plot(cav_2.get_state_history("safety"),"--","DisplayName","CAV 4 Safe Set")
% Add time markers for lane change
xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize', 18,...
    'FontWeight','bold','LabelOrientation','horizontal', ...
    'HandleVisibility','off')
xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize', 18,...
    'FontWeight','bold','LabelOrientation','horizontal', ...
    'HandleVisibility','off')
% Add formatting
ylabel("Position (x) [m]")
legend('Location','best')
title("Optimal Triplet Position History")
subplot(2, 1, 2)
% Extract terminal positions triplet
xf_1 = cav_1.get_state_history("x");
xf_2 = cav_2.get_state_history("x");
% Add position history
plot(veh_u.get_state_history("x"),"DisplayName","Veh U")
hold on
grid minor
plot(tf, cav_c.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_c.get_state_history("x"),"DisplayName","CAV C")
plot(cav_c.get_state_history("safety"),"--","DisplayName","CAV C Safe Set")
name_1 = strcat("$x_",num2str(i_m-1),"(t^l_0)$");
name_2 = strcat("$x_",num2str(i_m),"(t^l_0)$");
yline(xf_1.getsampleusingtime(t0_lc,t0_lc+0.05).Data(1),...
    ":",name_1, 'Interpreter','latex','FontSize',12,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off','Color','#00841a')
yline(xf_2.getsampleusingtime(t0_lc,t0_lc+0.05).Data(1),...
    ":",name_2, 'Interpreter','latex','FontSize',12,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off','Color',[0.5 0 0.8])
% Add time markers for lane change
xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize',18,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off')
xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize',18,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off')
% Add formatting
title('')
ylabel("Position (x) [m]")
xlabel("Time [s]")
legend('Location','best')

f(6) = figure('Name','Optimal Position Vehicle History');
subplot(4, 1, 1)
% Add position history
plot(cav_set(i_m-3).get_state_history("x"),"DisplayName","CAV 1")
hold on
grid minor
plot(tf, cav_set(i_m-3).x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_set(i_m-2).get_state_history("x"),"DisplayName","CAV 2")
plot(tf, cav_set(i_m-2).x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_set(i_m-2).get_state_history("safety"),"--","DisplayName","CAV 2 Safe Set")
% Add formatting
ylabel("Position (x) [m]")
xlabel('')
title('')
legend('Location','best')
subplot(4, 1, 2)
% Add position history
plot(cav_set(i_m-2).get_state_history("x"),"DisplayName","CAV 2")
hold on
grid minor
plot(tf, cav_set(i_m-2).x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_1.get_state_history("x"),"DisplayName","CAV 3")
plot(tf, cav_1.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_1.get_state_history("safety"),"--","DisplayName","CAV 3 Safe Set")
% Add formatting
ylabel("Position (x) [m]")
legend('Location','best')
xlabel('')
title('')
% Position subplot
subplot(4, 1, 3)
% Add position history
plot(cav_1.get_state_history("x"),"DisplayName","CAV 3")
hold on
grid minor
plot(tf, cav_1.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_2.get_state_history("x"),"DisplayName","CAV 4")
plot(tf, cav_2.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_2.get_state_history("safety"),"--","DisplayName","CAV 4 Safe Set")
% Add time markers for lane change
xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize', 18,...
    'FontWeight','bold','LabelOrientation','horizontal', ...
    'HandleVisibility','off','LineWidth',1.5)
xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize', 18,...
    'FontWeight','bold','LabelOrientation','horizontal', ...
    'HandleVisibility','off','LineWidth',1.5)
% Add formatting
ylabel("Position (x) [m]")
legend('Location','best')
xlabel('')
title('')
subplot(4, 1, 4)
% Extract terminal positions triplet
xf_1 = cav_1.get_state_history("x");
xf_2 = cav_2.get_state_history("x");
% Add position history
plot(veh_u.get_state_history("x"),"DisplayName","Veh U")
hold on
grid minor
plot(tf, cav_c.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_c.get_state_history("x"),"DisplayName","CAV C")
plot(cav_c.get_state_history("safety"),"--","DisplayName","CAV C Safe Set")
name_1 = strcat("$x_",num2str(3),"(t^l_0)$");
name_2 = strcat("$x_",num2str(4),"(t^l_0)$");
yline(xf_1.getsampleusingtime(t0_lc,t0_lc+0.05).Data(1),...
    ":",name_1, 'Interpreter','latex','FontSize',12,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off','Color','#00841a','LineWidth',1.5)
yline(xf_2.getsampleusingtime(t0_lc,t0_lc+0.05).Data(1),...
    ":",name_2, 'Interpreter','latex','FontSize',12,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off','Color',[0.5 0 0.8],'LineWidth',1.5)
% Add time markers for lane change
xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize',18,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off')
xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize',18,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off')
% Add formatting
title('')
ylabel("Position (x) [m]")
xlabel("Time [s]")
legend('Location','best')

f(7) = figure('Name','Optimal state Vehicle History');
subplot(5, 1, 1)
plot(cav_c.get_state_history("x",tf_lc))
hold on
grid minor
% plot(tf, cav_c.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
% Add time markers for lane change
xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize', 18,...
    'FontWeight','bold','LabelOrientation','horizontal', ...
    'HandleVisibility','off','LineWidth',1.5)
xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize', 18,...
    'FontWeight','bold','LabelOrientation','horizontal', ...
    'HandleVisibility','off','LineWidth',1.5)
% Add formatting
xlabel('')
%xlim([0,10])
ylabel("$x_C(t)\, [m]$",'Interpreter','latex')
title('')
subplot(5, 1, 2)
% Add position history
plot(cav_c.get_state_history("y",tf_lc))
hold on
grid minor
yline(1.8,...
    "--","$y_{des}$", 'Interpreter','latex','FontSize',12,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off','Color','black','LineWidth',2)
% Add formatting
xlabel('')
%xlim([0,10])
ylabel("$y_C(t)\, [m]$",'Interpreter','latex')
title('')
% Position subplot
subplot(5, 1, 3)
% Add position history
plot(cav_c.get_state_history("v",tf_lc))
hold on
grid minor
% Add formatting
ylabel("$v_C(t)\, [m/s]$",'Interpreter','latex')
%xlim([0,10])
xlabel('')
title('')
subplot(5, 1, 4)
% Add position history
plot(cav_c.get_state_history("accel",tf_lc))
hold on
grid minor
% Add formatting
ylabel("$u_C(t)\, [m/s^2]$",'Interpreter','latex')
xlabel('')
title('')
%xlim([0,10])
subplot(5, 1, 5)
% Add position history
plot(cav_c.get_state_history("steering",tf_lc))
hold on
grid minor
%xlim([0,10])
% Add formatting
ylabel("$\phi_C(t)\, [deg]$",'Interpreter','latex')
xlabel('Time [s]')
title('')

%% CAV C history
f(8) = figure('Name','CAV C History');
% Position subplot
subplot(3, 3, [1,2,3])
% Add position history
plot(cav_1.get_state_history("x"),"DisplayName","CAV 3")
hold on
grid minor
plot(tf, cav_1.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(tf, cav_2.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_2.get_state_history("x"),"DisplayName","CAV 4")
plot(cav_2.get_state_history("safety"),"--","DisplayName","CAV 4 Safe Set")
% Add time markers for lane change
xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize', 18,...
    'FontWeight','bold','LabelOrientation','horizontal', ...
    'HandleVisibility','off')
xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize', 18,...
    'FontWeight','bold','LabelOrientation','horizontal', ...
    'HandleVisibility','off')
% Add formatting
ylabel("$x_3(t),x_4(t)\, [m]$",'Interpreter','latex')
xlabel('')
title('')
legend('Location','best')
subplot(3, 3, [4,5,6])
% Extract terminal positions triplet
xf_1 = cav_1.get_state_history("x");
xf_2 = cav_2.get_state_history("x");
% Add position history
plot(veh_u.get_state_history("x"),"DisplayName","Veh U")
hold on
grid minor
plot(tf, cav_c.x_f, 'Marker','x','MarkerSize',10,'HandleVisibility','off')
plot(cav_c.get_state_history("x"),"DisplayName","CAV C")
plot(cav_c.get_state_history("safety"),"--","DisplayName","CAV C Safe Set")
name_1 = strcat("$x_",num2str(i_m-1),"(t^l_0)$");
name_2 = strcat("$x_",num2str(i_m),"(t^l_0)$");
yline(xf_1.getsampleusingtime(t0_lc,t0_lc+0.05).Data(1),...
    ":",name_1, 'Interpreter','latex','FontSize',12,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off','Color','#00841a','LineWidth',2)
yline(xf_2.getsampleusingtime(t0_lc,t0_lc+0.05).Data(1),...
    ":",name_2, 'Interpreter','latex','FontSize',12,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off','Color',[0.5 0 0.8],'LineWidth',2)
% Add time markers for lane change
xline(t0_lc,'-.',"$t^l_0$",'Interpreter','latex','FontSize',18,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off')
xline(tf_lc,'-.',"$t^l_f$",'Interpreter','latex','FontSize',18,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off')
% Add formatting
title('')
ylabel("$x_c(t), x_u(t)\, [m]$",'Interpreter','latex')
xlabel("Time [s]")
legend('Location','best')

subplot(3, 3, 7)
% Add position history
plot(cav_c.get_state_history("v"))
hold on
grid minor
yline(29,...
    "--","$v_{des}$", 'Interpreter','latex','FontSize',12,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off','Color','black','LineWidth',2)
% Add formatting
title('')
ylabel("$v_c(t)\, [m]$",'Interpreter','latex')
xlabel("Time [s]")

subplot(3, 3, 8)
% Add position history
plot(cav_c.get_state_history("y").getsampleusingtime(t0_lc, tf_lc))
hold on
grid minor
yline(1.8,...
    "--","$y_{des}$", 'Interpreter','latex','FontSize',12,...
    'FontWeight','bold','LabelOrientation','horizontal',...
    'HandleVisibility','off','Color','black','LineWidth',2)
% Add formatting
title('')
ylabel("$y_c(t)\, [m]$",'Interpreter','latex')
xlabel("Time [s]")

subplot(3, 3, 9)
% Extract terminal positions triplet
% Add position history
plot(cav_c.get_state_history("steering").getsampleusingtime(t0_lc, tf_lc))
hold on
grid minor
% Add formatting
title('')
ylabel("$\phi_c(t)\, [deg]$",'Interpreter','latex')
xlabel("Time [s]")


%% Save plots if requested
names = ["position", "speed", "acceleration", "ocp_diff",...
    "triplet_history","position_history", "state_hist_c", "cav_c"];
if ~isempty(save_location)
    for i=1:length(f)
        savefig(f(i),strcat(save_location,filesep,names(i)))
        print(f(i),strcat(save_location,filesep,names(i)),'-depsc')
    end
end

end

