function rt = Widcontrol_function(xi,vi,ai,xip,vip,aip,ip)
%WIDCONTROL Summary of this function goes here
%   Detailed explanation goes here
global u
global dt
%% parameters
dt=0.05;


l_v = 4.5; %Veh length
cc0 = 1.5; 
cc1 = 1.8;
cc2 = 3.99;
cc3 = -8;
cc4 = -0.35;
cc5 = 0.35;
cc6 = 11.44*10^-4;
cc7 = 0.25;
cc8 = 3.49;
cc9 = 1.49;
RND1 = normrnd(0.5,0.15);
VDES = 28; %Desired speed

%% control

if(numel(ip))      %ip is the preceding vehicle if exists
            dv = vip-vi; 
            dx = xip-xi-l_v;
else
            xip = 10^6;
            vip = 10^6;
            aip = 10^6;
            dv = vip-vi;
            dx = xip-xi-l_v;            
end
            
            if(dv >0) && (aip < -1)
                V_slower = vi;
            else
                V_slower = vip - dv*RND1; 
            end
            SDX_c = cc0+cc1*V_slower;
            SDV = cc6 * (dx - l_v)^2;
            SDX_o = SDX_c+cc2;
            SDX_v = SDX_o+cc3*(dv-cc4);
            if(vip > 0)
                CLDV = (-SDV + cc4);
            else
                CLDV = 0;
            end
            if(vi > cc5)
                OPDV = SDV + cc5;
            else
                OPDV = SDV;
            end
            u = 0;
            if(dv< OPDV) && (dx < SDX_c ) % Emergency brake
                if (vi > 0) && (dv < 0)
                    if(dx > cc0)
                        u = min(aip + dv^2/(cc0-dx),ai);
                    else
                        u = min(aip + 0.5*(dv-OPDV),ai);
                        if(u > - cc7)
                            u = -cc7;
                        else
                            u = max (u, -10+0.5*vi^0.5);
                        end
                    end                    
                end
            else
                if(dv < CLDV) && (dx < SDX_v) % closing in
                    u = max (dv^2/(2*(SDX_c-dx-0.1)),-10);
                else
                    if(dv < OPDV) && (dx < SDX_o) %following 
                        if (ai >= 0)
                            u = min(ai,-cc7);
                        else
                            u = max(ai,cc7);
                            u = max(u,VDES-vi);
                        end
                    else
                        if(dx > SDX_c) %?Free driving
                            if(vi > VDES)
                                u = cc7;
                            else
                                umax=cc8+ 0.05*cc9*min(vi,22.2)+(2*RND1+1);
                                if(dx < SDX_o)
                                    u = min(dv^2/(SDX_o-dx),umax);
                                else
                                    u = umax;
                                end
                            end
                            u = min (u,VDES-vi);
                        end
                    end
                end
            end
               x0(1) = xi;
               x0(2) = vi;
               t = [0 dt];
               [~,xx] = ode45('updatedynamic',t,x0(1:2));
               rt = [xx(end, 1), xx(end, 2),u];
end

