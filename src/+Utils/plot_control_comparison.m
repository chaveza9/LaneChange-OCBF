function plot_control_comparison(cav_c_set, save_location)
arguments
    cav_c_set
    save_location = []
end
%PLOT_STATE_HISTORY 
num_vehicles = length(cav_c_set);
%% Extract CAV  history based on index
% allocate buffers
u_ocp_hist = cav_c_set(1).ocp_prob.u_t_hist; % stays constant
method = ["FxT-OCBF", "OCBF", "CBF"];
lines = ["--","-.",":"];
color = ["black","red","green"];
% Acceleration history
f(1) = figure('Name','Optimal Control History Comparison');
plot(u_ocp_hist,'DisplayName','OCP','LineWidth',2.5)
hold on
grid minor
for i =1:num_vehicles
    t0_l = cav_c_set(i).LaneChangeTime_start;
    vec_hist = cav_c_set(i).get_state_history('accel');%.getsampleusingtime(0,t0_l);
    plot(vec_hist,'DisplayName',method(i),'LineStyle',lines(i),...
        'LineWidth',2,'Color',color(i))
    
end
title('')
legend('Location','best');
ylabel('Acceleration $u(t)$ $[m/s^2]$','Interpreter','latex','FontSize',12)
xlabel('Time $[s]$','Interpreter','latex','FontSize',12)

% State Difference History
y_label = ["$\Delta x^2(t)$ $[m^2]$","$\Delta v^2(t)$ $[m^2/s^2]$",...
    "$\Delta u^2(t)$ $[m^2/s^4]$"];
state_list = ["x_ref_diff","v_ref_diff","ocp_difference"];

f(2) = figure('Name','Optimal Control Difference History Comparison');
for k=1:3
    subplot(3,1,k)
    for i =1:num_vehicles
        t0_l = cav_c_set(i).LaneChangeTime_start;
        vec_hist = ...
            cav_c_set(i).get_state_history(state_list(k)...
            );%.getsampleusingtime(0,t0_l);
        plot(vec_hist.Time, vec_hist.Data.^2,'DisplayName',method(i),'LineStyle',lines(i),...
            'LineWidth',2.5,'Color',color(i))
        hold on
        grid minor
    end
    title('')
    if k ==1
        legend('Location','best');
    end
    ylabel(y_label(k),'Interpreter','latex','FontSize',12);
end
xlabel('Time $[s]$','Interpreter','latex','FontSize',12)



%% Save plots if requested
names = ["acceleration", "ocp_diff"];
if ~isempty(save_location)
    for i=1:2
        savefig(f(i),strcat(save_location,filesep,names(i)))
        print(f(i),strcat(save_location,filesep,names(i)),'-depsc')
    end
end

end

