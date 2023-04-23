function plot_state_history(veh_env, tf, i_m, save_location)
arguments
    veh_env
    tf
    i_m
    save_location = []
end
%PLOT_STATE_HISTORY 
num_vehicles = length(veh_env);
%% Position History Plot

f(1) = figure('Name','Position Plot', 'Position',[100 100 700 1200]);
for i = 1:num_vehicles
    subplot(num_vehicles, 1, i)
    % Add position history
    plot(veh_env(i).get_state_history('x'))
    hold on
    % Add terminal position
    if ~ strcmp(veh_env(i).VehicleID,'u')
        plot(tf, veh_env(i).x_f, 'Marker','x','MarkerSize',10)
    end
    grid minor
    ylabel(strcat("x_",veh_env(i).VehicleID, " [m]"))
end
% Add x label
xlabel("Time [s]")

%% Position History Plot
f(2)= figure('Name','Speed Plot', 'Position',[100 100 700 1200]);
for i = 1:num_vehicles
    subplot(num_vehicles, 1, i)
    % Add position history
    plot(veh_env(i).get_state_history('v'))
    hold on
    % Add terminal position
    if ~ strcmp(veh_env(i).VehicleID,'u')
        plot(tf, veh_env(i).DesSpeed, 'Marker','x','MarkerSize',10)
    end
    grid minor
    ylabel(strcat("v_",veh_env(i).VehicleID, " [m/s]"))
end
% Add x label
xlabel("Time [s]")

%% Position History Plot
f(3) = figure('Name','Control Input Plot', 'Position',[100 100 700 1200]);
for i = 1:num_vehicles
    subplot(num_vehicles, 1, i)
     % Add position history
    plot(veh_env(i).get_state_history("accel"))
    hold on
    grid minor
    ylabel(strcat("u_",veh_env(i).VehicleID, " [m/s^2]"))
end
% Add x label
xlabel("Time [s]")

%% Triplet plots
% Extract triplet
cav_1 = veh_env(i_m-1);
cav_2 = veh_env(i_m);
cav_c = veh_env(end-1);
veh_u = veh_env(end);

f(4) = figure('Name','Optimal Triplet History');
% Position subplot
subplot(2, 1, 1)
% Add position history
plot(cav_1.get_state_history("x"),"DisplayName","CAV 1")
hold on
grid minor
plot(cav_2.get_state_history("x"),"DisplayName","CAV 2")
plot(cav_2.get_state_history("safety"),"--","DisplayName","CAV 2 Safe Set")
ylabel("Position [x]")
legend('Location','best')
subplot(2, 1, 2)
% Extract terminal positions triplet
xf_1 = cav_1.get_state_history("x");
xf_2 = cav_2.get_state_history("x");
xf_1.Data = xf_1.Data(end);
xf_2.Data = xf_2.Data(end);
% Add position history
plot(veh_u.get_state_history("x"),"DisplayName","Veh U")
hold on
grid minor
plot(cav_c.get_state_history("x"),"DisplayName","CAV C")
plot(cav_c.get_state_history("safety"),"--","DisplayName","CAV C Safe Set")
plot(xf_1,"-.","DisplayName","Terminal Position 1")
plot(xf_2,"-.","DisplayName","Terminal Position 2")
ylabel("Position [x]")
xlabel("Time [s]")
legend('Location','best')
%% Save plots if requested
names = ["position", "speed", "acceleration", "triplet_history"];
if ~isempty(save_location)
    for i=1:4
        savefig(f(i),strcat(save_location,filesep,names(i),'pdf'))
    end
end

end

