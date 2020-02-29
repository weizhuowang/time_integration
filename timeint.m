% https://s3.amazonaws.com/szmanuals/6c05fe5e978d2e2e229955e3a35e9717
% https://web.archive.org/web/20140802021208/http://www.boeing.com/assets/pdf/commercial/startup/pdf/777_perf.pdf
% https://www.predictivemobility.com/family-777

clc,clear;
% ===================Sim setup====================
t = 0; dt = 0.1; tend = 1680*60; stage = 1; stopsim = false;
g = 32.17405;DISA = 0;
% ===================Init====================
V = 0; L_to = 0; h = 0;D = 0;Fuel_burnt = 0;R = 0;L_land = 0;
% ===================Logger====================
log_t = zeros(1,tend/dt); log_ROC = zeros(1,tend/dt);
log_h = zeros(1,tend/dt); log_V = zeros(1,tend/dt);
log_Fuel = zeros(1,tend/dt); log_Cl = zeros(1,tend/dt);
% ===================Aircraft====================
m0 = 545000; Vr = 199*1.6878;
mu_roll = 0.04; mu_slide = 0.45;
V_cruise = 813; h_cruise = 35000;
R_cruise_end = 5100; m_maxFuel = 207700;
V_init_approach = 230*1.6878; h_approach = 5000;
Cl_max_land = 2.49;
global S ThrustSL ThrustMinSL SFC_Cruise Cd0 K
S = 4605; ThrustSL = 77000; ThrustMinSL = 2700; SFC_Cruise = 0.528;
Cd0 = 0.0145; K = 0.044152; %0.053
    
tic
for i = 1:tend/dt
    
    switch stage
        case 1   % take off
            Thrust = thrustCalc(h);
            SFC = SFCcalc(h);
            Fuel_burnt = Fuel_burnt + SFC*Thrust*2/3600*dt;
            m = m0-Fuel_burnt;
            if V < Vr
                Cl = 0.5;
                ROC = 0;
                L = 0.5*AltRho(h,DISA)*V^2*S*Cl;
                F_forward = Thrust*2 - mu_roll*(m-L);
                t_rotate = t;
            else
                Cl = (2*m/(AltRho(h,DISA)*V^2*S))*1;
                ROC = ((Thrust*2-D)*g*V)/(m*g)*60*min(dt*i-t_rotate,3)/3; ROC = min(ROC,1500);
                F_forward = (Thrust*2*V-ROC/60*m)/V;
            end

            D = Dcalc(V,Cl,h);
            a = (F_forward-D)*g/m;
            V = V + a*dt;
            h = h + ROC/60*dt;
            L_to = L_to + V*dt;

            if h>50
                t
                stage = stage + 1;
            end
            
        case 2   % climb
            SFC = SFCcalc(h);
            Thrust = min(thrustCalc(h),ThrustSL*0.9);
            m = m0-Fuel_burnt;
            Cl = (2*m/(AltRho(h,0)*V^2*S));
            D = Dcalc(V,Cl,h);

            if V < V_cruise
                a = 0.5;
%                 fprintf('Thrust used: %2.0f Total Thrust: %2.0f ROC: %2.0f acce: %2.0f\n',...
%                         [Thrust_used,Thrust*2,ROC/60*m/V,a*m/g])
            else
                a = 0;
            end
            ROC = ((Thrust*2-D-m*a/g)*g*V)/(m*g)*60; ROC = min(ROC,1700);
            F_forward_max = (Thrust*2*V-ROC/60*m)/V;
            Thrust_used = Thrust*2 - (F_forward_max-D-a*m/g);
            
            
            V = V + a*dt;
            h = h + ROC/60*dt;
            R = R + (sqrt(V^2-(ROC/60)^2)*dt)/6076.11;
            Fuel_burnt = Fuel_burnt + SFC*Thrust_used*2/3600*dt;
            
            fprintf('%5.0f %2.0f %2.0f %2.0f\n',[h,ROC,V,a])
            
            if ROC < 100
                fprintf('Reached service ceiling at %2.0f\n',[h])
                t_climb = t
                ROC = 0;
                stage = 3;
            end
            
            if h > h_cruise
                t_climb = t
                R
                Fuel_burnt
                ROC = 0;
                stage = 3;
            end
            

            
        case 3   % cruise
            SFC = SFCcalc(h);
            R = R + V*dt/6076.11;
            
            m = m0-Fuel_burnt;
            Cl = (2*m/(AltRho(h,0)*V^2*S));
            D = Dcalc(V,Cl,h);
            Thrust_used = D;
            Fuel_burnt = Fuel_burnt + SFC*Thrust_used/3600*dt;
            
            
            if R > R_cruise_end
                t_cruise = t
                Fuel_burnt
                stage = 4;
            end
            
            if Fuel_burnt > m_maxFuel
                t_cruise = t
                fprintf('Out of Fuel at range %2.0f\n',[R])
                stage = 4;
            end
            
            
        case 4   % descent
            SFC = SFCcalc(h);
            Thrust = thrustMinCalc(h);
            m = m0-Fuel_burnt;
            Cl = (2*m/(AltRho(h,0)*V^2*S));
            D = Dcalc(V,Cl,h);

            if V > V_init_approach
                a = -0.4;
%                 fprintf('Thrust used: %2.0f Total Thrust: %2.0f ROC: %2.0f acce: %2.0f\n',...
%                         [Thrust_used,Thrust*2,ROC/60*m/V,a*m/g])
            else
                a = 0;
            end
            ROC = ((Thrust*2-D-m*a/g)*g*V)/(m*g)*60; ROC = max(ROC,-2000);
            Thrust_used = 0.5*(D + a*m/g + ROC*m/(60*V));
            
            
            V = V + a*dt;
            h = h + ROC/60*dt;
            R = R + (sqrt(V^2-(ROC/60)^2)*dt)/6076.11;
            Fuel_burnt = Fuel_burnt + SFC*Thrust_used*2/3600*dt;
           
%             fprintf('%5.0f %2.0f %2.0f %2.2f %2.2f\n',[h,ROC,V,a,Thrust_used])
            
            if h < h_approach
                stage = 5;
            end
            
            
        case 5   % approach
            rho = AltRho(h,0);
            Vstall = sqrt(2*m/(rho*S*2.49));
            Vapproach = Vstall*1.3;
            SFC = SFCcalc(h);
            Thrust = thrustMinCalc(h);
            m = m0-Fuel_burnt;
            Cl = (2*m/(AltRho(h,0)*V^2*S));
            [D,Cd] = Dcalc(V,Cl,h);
            
            if V > Vapproach
                a = max(-abs(V-Vapproach),-0.4);
            else
                a = 0;
            end
            
            ROC = ((Thrust*2-D-m*a/g)*g*V)/(m*g)*60; ROC = max(ROC,-500);
            Thrust_used = 0.5*(D + a*m/g + ROC*m/(60*V));
            
            
            V = V + a*dt;
            h = h + ROC/60*dt;
            R = R + (sqrt(V^2-(ROC/60)^2)*dt)/6076.11;
            Fuel_burnt = Fuel_burnt + SFC*Thrust_used*2/3600*dt;
            
            if h<50
                stage = 6;
                t_flair = t;
            end
        case 6 % land
            
            Thrust = thrustMinCalc(h);
            SFC = SFCcalc(h);
            Fuel_burnt = Fuel_burnt + SFC*Thrust*2/3600*dt;
            m = m0-Fuel_burnt;
            
            if h > 0.1
                Cl = (2*m/(AltRho(h,DISA)*V^2*S));
                fprintf('t: %2.2f h: %2.2f cl:%2.2f \n',[t-t_flair,h,Cl])
                ROC = ((Thrust*2-D)*g*V)/(m*g)*60;
                ROC = max(ROC,-500+175*min(t-t_flair,2));
                F_forward = (Thrust*2*V-ROC/60*m)/V;
            else
                Cl = 0.2;
                ROC = 0;
                mu = 0.45;
                L = 0.5*AltRho(h,DISA)*V^2*S*Cl;
                F_forward = Thrust*2 - mu*(m-L);
                t_rotate = t;
            end

            D = Dcalc(V,Cl,h);
            if Cl > Cl_max_land
                a = 0;
                Thrust_used = 0.5*(D+ROC/60*m/V);
            else
                a = (F_forward-D)*g/m;
            end
            V = V + a*dt;
            h = h + ROC/60*dt;
            L_land = L_land + V*dt;
            R = R + (sqrt(V^2-(ROC/60)^2)*dt)/6076.11;
            if h > 0.1
                Fuel_burnt = Fuel_burnt + SFC*Thrust_used*2/3600*dt;
            else
                Fuel_burnt = Fuel_burnt + SFC*Thrust*2/3600*dt;
            end
            
            if V<30
                stage = 7;
                L_land
            end
            
        otherwise
            stopsim = true;
    end
    
    % ==========Log stuff============
    log_t(i) = t; log_ROC(i) = ROC; log_h(i) = h;
    log_V(i) = V; log_Fuel(i) = m_maxFuel-Fuel_burnt;
    log_Cl(i) = Cl;
    
    if stopsim
        break
    end
    t = t + dt;
    
end
toc

L_to
h

%% test
figure(1);clf;
yyaxis left
plot(log_t(1:i),log_h(1:i));grid on;hold on;
ylabel('Altitude [ft]')
yyaxis right
plot(log_t(1:i),log_V(1:i));grid on;hold on;
plot(log_t(1:i),log_ROC(1:i));grid on;hold on;
xlabel('time [s]');ylabel('ROC [fpm] & V [ft/s]')


% yyaxis left
% ylim([0,200])
% xlim([2.8935e4,2.8975e4])

function sfc = SFCcalc(h)

    global SFC_Cruise
    sfc = 0.324 + (SFC_Cruise-0.324)*h/35000;

end

function [D,Cd] = Dcalc(V,Cl,alt)
    
    global S Cd0 K
    cdw = 0;
    
    Cd = Cd0 + K*Cl^2;
    
    [rho,~,sigma,~,theta] = AltRho(alt,0);
    Veq = V*sqrt(sigma);
    
    T = 518.67*theta;
    M = V/sqrt(1.4*1716*T);
    
    if M > 0.8
        cdw = 0.075*sin((M-0.8)*pi/0.2-0.5*pi) + 0.075;
    end
    Cd = Cd + cdw*0.4;
    D = 0.5*rho*V^2*S*Cd;


end

function thrust = thrustMinCalc(h)

    global ThrustMinSL
    [~,~,~,Thrust_lapse,~] = AltRho(h,0);
    thrust = ThrustMinSL*Thrust_lapse;

end

function thrust = thrustCalc(h)

    global ThrustSL
    [~,~,~,Thrust_lapse,~] = AltRho(h,0);
    thrust = ThrustSL*Thrust_lapse;

end

% Find density at certain altitude
function [rho,viscosity,sigma,delta,theta] = AltRho(h,DISA)
    
    if h<36089
        theta = 1-6.87535e-6*h+DISA;
        delta = (theta-DISA)^5.2561;
    else
        theta = 0.75187+DISA;
        delta = 0.22336*exp((36089-h)/20806.7);
    end
    sigma = delta/theta;
    
    temperature = 518.67*theta;
    rho = 0.0023769*sigma;
    viscosity = (2.2697e-8*temperature^1.5)/(temperature+198.72);
    
    
end










