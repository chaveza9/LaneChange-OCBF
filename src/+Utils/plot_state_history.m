function plot_state_history(cav_set, tf, i_m, save_location)
arguments
    cav_set
    tf
    i_m
    save_location = []
end
%PLOT_STATE_HISTORY 
num_vehicles = length(cav_set);
%% Position History Plot

f(1) = figure('Name','Position Plot', 'Position',[100 100 700 1200]);
for i = 1:num_vehicles
    subplot(num_vehicles, 1, i)
    % Add position history
    plot(cav_set(i).get_state_history('x'))
    hold on
    % Add terminal position
    if ~ strcmp(cav_set(i).VehicleID,'u')
        plot(tf, cav_set(i).x_f, 'Marker','x','MarkerSize',10)
    end
    grid minor
    ylabel(strcat("x_",cav_set(i).VehicleID, " [m]"))
    title('')
    if i ~=num_vehicles
        xlabel('',"HandleVisibility","off")
    end

end
% Add x label
xlabel("Time [s]")

%% Position History Plot
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
    grid minor
    ylabel(strcat("v_",cav_set(i).VehicleID, " [m/s]"))
    title('')
    if i ~=num_vehicles
        xlabel('',"HandleVisibility","off")
    end
end
% Add x label
xlabel("Time [s]")

%% Position History Plot
f(3) = figure('Name','Control Input Plot', 'Position',[100 100 700 1200]);
for i = 1:num_vehicles
    subplot(num_vehicles, 1, i)
     % Add position history
    plot(cav_set(i).get_state_history("accel"))
    hold on
    grid minor
    ylabel(strcat("u_",cav_set(i).VehicleID, " [m/s^2]"))
   title('')
    if i ~=num_vehicles
        xlabel('', "HandleVisibility","off")
    end
end
% Add x label
xlabel("Time [s]")

%% Triplet plots
% Extract triplet
cav_1 = cav_set(i_m-1);
cav_2 = cav_set(i_m);
cav_c = cav_set(end-1);
veh_u = cav_set(end);

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
title("Optimal Triplet Position History")
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
title('')
ylabel("Position [x]")
xlabel("Time [s]")
legend('Location','best')
%% Save plots if requested
names = ["position", "speed", "acceleration", "triplet_history"];
if ~isempty(save_location)
    for i=1:4
        savefig(f(i),strcat(save_location,filesep,names(i)))
        print(f(i),strcat(save_location,filesep,names(i)),'-dpdf','-bestfit')
    end
end

end

