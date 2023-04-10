function [opti, tf, x_C, v_C, x_f, v_f, B] = define_termial_conditions (self, x_C_0, v_C_0, x_0, v_0, x_U_0, v_U_0)
       % ------------------------------------
       % x_0 = initial conditions [numVehicles,1]
       % v_0 = initial conditions [numVehicles,1]
       % x_f, v_f = terminal conditions [numVehicles,1]
       % ------------------------------------
       arguments
           self
           x_C_0
           v_C_0
           x_0
           v_0
           x_U_0
           v_U_0
       end
       opti = casadi.Opti(); % Optimization problem
       % define simulation parameters
       N = self.N;  
       u_max = self.u_max;   % speed and control limits
       u_min = self.u_min;
       v_max = self.v_max;
       v_min = self.v_min;
       T = self.T;     % maximum allowable time
       alpha_t = self.alpha_t;
       gammax = self.gammax;   % weights in disruption
       gammav = self.gammav;
       phi = self.phi;   % reaction time 
       delta = self.delta;  
       v_d = self.desSpeed; %desired speed
       numCandidates = size(x_0,1);

       %% Setup optimization variables
        % Dynamics Variables
        tf = opti.variable();
        x = opti.variable(numCandidates, 1); % control variables
        v = opti.variable(numCandidates, 1); % control variables
        B = opti.variable(numCandidates, 1); % binary variables
        discrete = [zeros(numCandidates*2+1,1);ones(numCandidates,1)];
        
%         % Define dynamics
%         f = @(v,u) [v;u]; %dx/dt = f(v,u)
%         dt = tf/N;
       
        % cost (disruption)
        cost = alpha_t*tf+gammax*(x_C-x_C_0-v_C_0*tf)^2+gammav*(v_C-v_d)^2;
        for k=1:length(numCandidates)
            cost = cost + gammax*(x(k,1)-x_0(k,1)-v_0(k,1)*tf)^2+gammav*(v(k,1)-v_d)^2;
        end
        opti.minimize(cost);

        %% Define Safety Conditions
        if numCandidates > 1
            for i=1:length(numCandidates)-1
                opti.subject_to(x(i,1)-x(i+1,1)>=phi*v(i+1,1)+delta);
                opti.subject_to(x_C-x(i,1)+(1-B(i))*M>=phi*phi*v(i,1)+delta);
                opti.subject_to(x(i,1)-x_C+B(i)*M>=phi*v_C+delta);
                [p_u(i),p_l(i)] = reachable_set_p(self,x(i,1),x_0(i,1),v(i,1),v_0(i,1),tf);
                opti.subject_to(p_u(i)<=0);
                opti.subject_to(p_l(i)>=0);
                opti.subject_to(v_min<=v(i,1)<=v_max);
            end
        elseif numCandidates == 1
            opti.subject_to(x(1,1)-x_C>=phi*v(1,1)+delta);
            [p_u(1),p_l(1)] = reachable_set_p(self,x(1,1),x_0(1,1),v(1,1),v_0(1,1),tf);
            opti.subject_to(p_u(1)<=0);
            opti.subject_to(p_l(1)>=0);
            opti.subject_to(v_min<=v(1,1)<=v_max);
        end
        opti.subject_to(x(length(numCandidates)-1)-x(length(numCandidates))>=phi*v(length(numCandidates))+delta);
        opti.subject_to(x_C-x(length(numCandidates),1)+(1-B(length(numCandidates)))*M>=phi*v(length(numCandidates),1)+delta);
        opti.subject_to(x(length(numCandidates),1)-x_C+B(length(numCandidates))*M>=phi*v_C+delta);
        [p_u(length(numCandidates)),p_l(length(numCandidates))] = reachable_set_p(self,x(length(numCandidates),1),x_0(length(numCandidates),1),v(length(numCandidates),1),v_0(length(numCandidates),1),tf);
        opti.subject_to(p_u(length(numCandidates))<=0);
        opti.subject_to(p_l(length(numCandidates))>=0);
        opti.subject_to(v_min<=v(length(numCandidates),1)<=v_max);

        opti.subject_to(x_C-x_U_0-v_U_0*tf>=phi*v_C+delta);
     
end


function [p_u_i,p_l_i] = reachable_set_p(self,x_i,x_i_0,v_i,v_i_0,t)
        nu = (self.u_max+self.u_min)/2;
        mu = (self.u_max-self.u_min)/2;
        p_u_i = -0.5*t^2 + 0.25*((v_i-v_i_0-nu*t)/mu+t)^2-(x_i-x_i_0-t*v_i_0-nu*0.5*t^2)/nu;
        p_l_i = 0.5*t^2 - 0.25*((-v_i+v_i_0+nu*t)/mu+t)^2-(x_i-x_i_0-t*v_i_0-nu*0.5*t^2)/mu;
end

function [tf,x_C,v_C,x_f,v_f,B] = solve_minlp (opti, tf_var, x_C_var, v_C_var, x_f_var, v_f_var, B_var)
      arguments
          opti
          tf_var
          x_C_var
          v_C_var
          x_f_var
          v_f_var
          B_var
      end

      % Define optimizer settings
      solver_options = struct('print_time', 1, 'discrete',discrete);
      opti.solver('bonmin',solver_options);
      sol = opti.solve();

      tf = sol.value(tf_var);
      x_C = sol.value(x_C_var);
      v_C = sol.value(v_C_var);
      x_f = sol.value(x_f_var);
      v_f = sol.value(v_f_var);
      B = sol.value(B_var);

end