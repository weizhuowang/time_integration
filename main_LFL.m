clc,clear;
format bank
[fname,badpt] = time_integration_TO(@aircrafts.SetupSim_AIAA_folding300,true,true,{-42300,0,0,3500,0.8,35000});
TI = load(fname);
TI.L_land
% MLDW-42300
% EOM-95000
%%

figure(2);clf
plot(TI.log_t,TI.log_h,'linewidth',2);grid on;hold on;

% plot([0,9000],[0,0]-1,'k','linewidth',2);hold on;
% axis equal
%% Plot

figure(1);clf;
yyaxis left
plot(TI.log_t,TI.log_h);grid on;hold on;
ylabel('Altitude [ft]')
yyaxis right
plot(TI.log_t,TI.log_V);grid on;hold on;
plot(TI.log_t,TI.log_ROC);grid on;hold on;
xlabel('time [s]');ylabel('ROC [fpm] & V [ft/s]')

figure(4);clf;
plot(TI.log_d-TI.R_land,TI.log_h,'linewidth',2);grid on;hold on;
xlabel('distance [ft]');ylabel('Altitude [ft]')

% figure(2);clf
% plot(TI.log_t,TI.log_Fuel);grid on;hold on;

figure(3); clf
plot(TI.log_t,TI.log_thrustUsed);grid on;hold on;

%%


%%

function [fname,bad] = time_integration_TO(SetupSim,logflag,verbose,args)
    % ===================Sim setup====================
    t = 0; dt = 0.01; tend = 1200*60; stage = 1; stopsim = false;
    g = 32.17405; DISA = 15/288;bad = false; abort = false;
    % ===================Init====================
    [V,L_to,h,D,Fuel_burnt,R,L_land,Thrust_used,Cl,ROC,a,t_rotate,...
     t_flair,t_loiter_start,Cd,BlockFuel,R_land] = deal(0);
    % ===================Logger====================
    if logflag
        [log_t,log_ROC,log_h,log_V,log_Fuel,log_Cl,log_Cd,log_d,log_thrustUsed] = deal(zeros(1,tend/dt));
    end
    fname = SetupSim(args{:});
    load(fname,...
         'm0','m','mu_roll','mu_slide','V_cruise','h_cruise',...
         'R_cruise_end','m_maxFuel','V_init_approach','h_approach',...
         'Cl_max_land','S','ThrustSL','ThrustMinSL','SFC_Cruise',...
         'Cdp','K')
     
%     mu_slide = 0.27; % wet
    mu_slide = 0.09; % icy
     setinit()

    for i = 1:tend/dt
        
        % ==========Prepare============
        [rho,~,~,~,~,V_a] = AltRho(h,DISA);
        SFC = SFCcalc(h,V/V_a);

        switch stage
            case 1 % divert approach
                approach(0)
            case 2 % land
                land(15/288)
            otherwise
                stopsim = true;
        end
        
        if stopsim break, end
        if bad break, end
        
        % ==========AfterSim=============
        StepSim() % 62-77 is for speed
        
    end

    ExitSim()
    
    
    % =============Flight Stages===============
    function approach(DISA)
        Vstall = sqrt(2*m/(rho*S*2.49*0.9));
        Vapproach = Vstall*1.3;
        ThrustMin = thrustMinCalc(h,DISA);
        Cl = (2*m/(rho*V^2*S));
        [D,Cd] = Dcalc(V,Cl,h,DISA);
        if V > Vapproach
            a = max(-abs(V-Vapproach),-0.4);
        else
            a = 0;
        end
        ROCMax = ((ThrustMin*2-D-m*a/g)*g*V)/(m*g)*60; ROC = max(ROCMax,-500);
        Thrust_used = 0.5*(D + a*m/g + ROC*m/(60*V));
        if h< 50
            L_land = L_land + V*dt;
        end
        if h<20
            if verbose fprintf('block end at range %2.0f fuel burnt %2.0f\n',[R,Fuel_burnt]); end
            stage = stage+1;
            t_flair = t;
            R_land = R*6076.11;
            if BlockFuel == 0
                BlockFuel = Fuel_burnt;
            end
        end
    end

    function land(DISA)
        ThrustMin = thrustMinCalc(h,DISA);
        if h > 0.1
            Cl = (2*m/(rho*V^2*S));
%             fprintf('t: %2.2f h: %2.2f cl:%2.2f \n',[t-t_flair,h,Cl])
            ROCMax = ((ThrustMin*2-D)*g*V)/(m*g)*60;
            ROC = max(ROCMax,-500+175*min(t-t_flair,2));
            F_forward = (ThrustMin*2*V-ROC/60*m)/V;
        else
            ThrustMin = -20000;
            Cl = 0.2; ROC = 0;
            L = 0.5*rho*V^2*S*Cl;
            F_forward = ThrustMin*2 - mu_slide*(m-L);
            t_rotate = t;
        end
        [D,Cd] = Dcalc(V,Cl,h,DISA);
        if Cl > Cl_max_land
            a = 0;
            Thrust_used = 0.5*(D+ROC/60*m/V);
        else
            a = (F_forward-D)*g/m;
        end
        L_land = L_land + V*dt;
        if h <0.1 Thrust_used = ThrustMin; end
        if V<5
            stage = stage+1;
        end
    end

    % ============Sim module=============
    function setinit()
        
        h = 500;
%         V = 300;
        [rho,~,~,~,~,V_a] = AltRho(h,DISA);
        Vstall = sqrt(2*m/(rho*S*2.49));
        Vapproach = Vstall*1.3;
        V = Vapproach;
        
    end
    function StepSim()
        V = V + a*dt;
        h = h + ROC/60*dt;
        
        R = R + (sqrt(V^2-(ROC/60)^2)*dt)/6076.11;
        Fuel_burnt = Fuel_burnt + SFC*Thrust_used*2/3600*dt;
        m = m0-Fuel_burnt;
        t = t + dt;
        if Fuel_burnt > m_maxFuel
            if verbose fprintf('Out of Fuel at stage %2.0f range %2.0f \n',[stage,R]); end
            bad = true;
        end
        
        if logflag
            log_t(i) = t; log_ROC(i) = ROC; log_h(i) = h;
            log_V(i) = V; log_Fuel(i) = m_maxFuel-Fuel_burnt;
            log_Cl(i) = Cl; log_Cd(i) = Cd; log_d(i) = R*6076.11;
            log_thrustUsed(i) = Thrust_used;
        end
    end

    function ExitSim()
        i = i-2;
        if logflag
            log_t    = log_t(1:i);
            log_ROC  = log_ROC(1:i);
            log_h    = log_h(1:i);
            log_V    = log_V(1:i); 
            log_Fuel = log_Fuel(1:i);
            log_Cl   = log_Cl(1:i); 
            log_Cd   = log_Cd(1:i);
            log_d    = log_d(1:i);
            log_thrustUsed = log_thrustUsed(1:i);
        end
%         save('temp/timeint_temp.mat')
        save(fname);
    end

    % Helper Functions
    function sfc = SFCcalc(h,M)
        sfc = (0.324*0.85 + (SFC_Cruise-0.324*0.85)*M/0.83)*(1.15-h/35000*0.15);
    end

    function [D,Cd] = Dcalc(V,Cl,alt,DISA)
        cdw = 0;

        Cd = Cdp + 1.1*K*Cl^2; % Cdp = Cd0 + Cdexcr, Cdtrim = 0.1*Cdi

        [rho,~,sigma,~,theta] = AltRho(alt,DISA);
        Veq = V*sqrt(sigma);

        T = 518.67*theta;
        M = V/sqrt(1.4*1716*T);

%         if M > 0.8
%             cdw = 0.075*sin((M-0.8)*pi/0.2-0.5*pi) + 0.075;
%         end
        if M > 0.75
            cdw = (M-0.75)*0.0013/0.05; % delta method p28
        end
        Cd = Cd + cdw;
        D = 0.5*rho*V^2*S*Cd;
    end

    function thrust = thrustMinCalc(h,DISA)
        Thrust_lapse = AltLapse(h,DISA);
        thrust = ThrustMinSL*Thrust_lapse;
    end

    function thrust = thrustMaxCalc(h,DISA)
        Thrust_lapse = AltLapse(h,DISA);
        thrust = ThrustSL*Thrust_lapse;
    end

end


%% Helper Modules

% Find density at certain altitude
function [rho,viscosity,sigma,delta,theta,V_a] = AltRho(h,DISA)
    
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
    viscosity = 9999999; %(2.2697e-8*temperature^1.5)/(temperature+198.72); % put dummy value here to speed up
    V_a = sqrt(1.4*1716*temperature);
    
end

function [delta] = AltLapse(h,DISA)
    
    if h<36089
        theta = 1-6.87535e-6*h+DISA;
        delta = (theta-DISA)^5.2561;
    else
        delta = 0.22336*exp((36089-h)/20806.7);
    end
    
end