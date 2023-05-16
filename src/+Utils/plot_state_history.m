function plot_state_history(cav_set, tf, i_m, save_location)
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
    plot(cav_set(i).get_state_history('x'))
    hold on
    % Add terminal position
    if ~ strcmp(cav_set(i).VehicleID,'u')
        plot(tf, cav_set(i).x_f, 'Marker','x','MarkerSize',10)
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
    plot(cav_set(i).get_state_history('v'))
    hold on
    % Add terminal position
    if ~ strcmp(cav_set(i).VehicleID,'u')
        plot(tf, cav_set(i).DesSpeed, 'Marker','x','MarkerSize',10)
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
    plot(cav_set(i).get_state_history("accel"))
    hold on
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
    if ~ strcmp(cav_set(i).VehicleID,'u')
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
plot(cav_1.get_state_history("x"),"DisplayName","CAV 1")
hold on
grid minor
plot(cav_2.get_state_history("x"),"DisplayName","CAV 2")
plot(cav_2.get_state_history("safety"),"--","DisplayName","CAV 2 Safe Set")
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

%% Save plots if requested
names = ["position", "speed", "acceleration", "ocp_diff","triplet_history"];
if ~isempty(save_location)
    for i=1:5
        savefig(f(i),strcat(save_location,filesep,names(i)))
        print(f(i),strcat(save_location,filesep,names(i)),'-dpdf','-bestfit')
    end
end

end

