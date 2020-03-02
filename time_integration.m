% https://s3.amazonaws.com/szmanuals/6c05fe5e978d2e2e229955e3a35e9717
% https://web.archive.org/web/20140802021208/http://www.boeing.com/assets/pdf/commercial/startup/pdf/777_perf.pdf
% https://www.predictivemobility.com/family-777


function time_integration(SetupSim)
    % ===================Sim setup====================
    t = 0; dt = 0.1; tend = 1630*60; stage = 1; stopsim = false;
    g = 32.17405;DISA = 0;
    % ===================Init====================
    [V,L_to,h,D,Fuel_burnt,R,L_land,Thrust_used,Cl,ROC,a,t_rotate,t_flair] = deal(0);
    % ===================Logger====================
    [log_t,log_ROC,log_h,log_V,log_Fuel,log_Cl] = deal(zeros(1,tend/dt));
    SetupSim()
    load('temp/setup.mat',...
         'm0','Vr','m','mu_roll','mu_slide','V_cruise','h_cruise',...
         'R_cruise_end','m_maxFuel','V_init_approach','h_approach',...
         'Cl_max_land','S','ThrustSL','ThrustMinSL','SFC_Cruise',...
         'Cd0','K')
    
    tic
    for i = 1:tend/dt
        
        % ==========Prepare============
        [rho,~,~,~,~,V_a] = AltRho(h,DISA);
        SFC = SFCcalc(h,V/V_a);
        
        switch stage
            case 1   % take off
                takeoff()
            case 2   % climb
                climb()
            case 3   % cruise
                cruise()
            case 4   % descent
                descent()
            case 5   % approach
                approach()
            case 6 % land
                land()
            otherwise
                stopsim = true;
        end
        
        % ==========AfterSim=============
        StepSim()
        LogSim()
        if stopsim break, end
        
    end
    toc

    save('temp/timeint_temp.mat')

    % Flight stages
    function takeoff()
        DISA = 15/288;
        Thrust = thrustMaxCalc(h,DISA);
        Thrust_used = Thrust;
        if V < Vr
            Cl = 0.5;
            ROC = 0;
            L = 0.5*rho*V^2*S*Cl;
            F_forward = Thrust*2 - mu_roll*(m-L);
            t_rotate = t;
        else
            Cl = (2*m/(rho*V^2*S))*1;
            ROC = ((Thrust*2-D)*g*V)/(m*g)*60*min(dt*i-t_rotate,3)/3; ROC = min(ROC,1500);
            F_forward = (Thrust*2*V-ROC/60*m)/V;
        end
        D = Dcalc(V,Cl,h,DISA);
        a = (F_forward-D)*g/m;
        L_to = L_to + V*dt;
        if h>50
            t
            stage = 2;
            Fuel_burnt
        end 
    end

    function climb()
        DISA = 0;
        Thrust = min(thrustMaxCalc(h,DISA),ThrustSL*0.9);
        Cl = (2*m/(rho*V^2*S));
        D = Dcalc(V,Cl,h,DISA);

        if V < V_cruise  a = 0.5; else a = 0; end
        ROC = ((Thrust*2-D-m*a/g)*g*V)/(m*g)*60; ROC = min(ROC,1700);
        F_forward_max = (Thrust*2*V-ROC/60*m)/V;
        Thrust_used = Thrust*2 - (F_forward_max-D-a*m/g);

    %                 fprintf('%5.0f %2.0f %2.0f %2.0f\n',[h,ROC,V,a])

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
    end

    function cruise()   
        ROC = 0;a = 0;DISA = 0;
        Cl = (2*m/(rho*V^2*S));
        D = Dcalc(V,Cl,h,DISA);
        Thrust_used = D/2;
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
    end

    function descent()
        DISA = 0;
        Thrust = thrustMinCalc(h,DISA);
        Cl = (2*m/(rho*V^2*S));
        D = Dcalc(V,Cl,h,DISA);
        if V > V_init_approach a = -0.4; else a = 0; end
        ROC = ((Thrust*2-D-m*a/g)*g*V)/(m*g)*60; ROC = max(ROC,-2000);
        Thrust_used = 0.5*(D + a*m/g + ROC*m/(60*V));
%             fprintf('%5.0f %2.0f %2.0f %2.2f %2.2f\n',[h,ROC,V,a,Thrust_used])
        if h < h_approach
            stage = 5;
        end 
    end

    function approach()
        DISA = 0;
        Vstall = sqrt(2*m/(rho*S*2.49));
        Vapproach = Vstall*1.3;
        Thrust = thrustMinCalc(h,DISA);
        Cl = (2*m/(rho*V^2*S));
        [D,Cd] = Dcalc(V,Cl,h,DISA);
        if V > Vapproach
            a = max(-abs(V-Vapproach),-0.4);
        else
            a = 0;
        end
        ROC = ((Thrust*2-D-m*a/g)*g*V)/(m*g)*60; ROC = max(ROC,-500);
        Thrust_used = 0.5*(D + a*m/g + ROC*m/(60*V));
        if h<50
            stage = 6;
            t_flair = t;
        end
    end

    function land()
        DISA = 15/288;
        Thrust = thrustMinCalc(h,DISA);
        if h > 0.1
            Cl = (2*m/(rho*V^2*S));
%             fprintf('t: %2.2f h: %2.2f cl:%2.2f \n',[t-t_flair,h,Cl])
            ROC = ((Thrust*2-D)*g*V)/(m*g)*60;
            ROC = max(ROC,-500+175*min(t-t_flair,2));
            F_forward = (Thrust*2*V-ROC/60*m)/V;
        else
            Cl = 0.2; ROC = 0;
            L = 0.5*rho*V^2*S*Cl;
            F_forward = Thrust*2 - mu_slide*(m-L);
            t_rotate = t;
        end
        D = Dcalc(V,Cl,h,DISA);
        if Cl > Cl_max_land
            a = 0;
            Thrust_used = 0.5*(D+ROC/60*m/V);
        else
            a = (F_forward-D)*g/m;
        end
        L_land = L_land + V*dt;
        if h <0.1 Thrust_used = Thrust; end
        if V<30
            stage = 7;
            L_land
        end
    end

    % Sim module
    function StepSim()
        V = V + a*dt;
        h = h + ROC/60*dt;
        R = R + (sqrt(V^2-(ROC/60)^2)*dt)/6076.11;
        Fuel_burnt = Fuel_burnt + SFC*Thrust_used*2/3600*dt;
        m = m0-Fuel_burnt;
        t = t + dt;
    end

    function LogSim()
        log_t(i) = t; log_ROC(i) = ROC; log_h(i) = h;
        log_V(i) = V; log_Fuel(i) = m_maxFuel-Fuel_burnt;
        log_Cl(i) = Cl;
    end

    % Helper Functions
    function sfc = SFCcalc(h,M)
        sfc = (0.324*0.85 + (SFC_Cruise-0.324*0.85)*M/0.83)*(1.15-h/35000*0.15);
    end

    function [D,Cd] = Dcalc(V,Cl,alt,DISA)
        cdw = 0;

        Cd = Cd0 + K*Cl^2;

        [rho,~,sigma,~,theta] = AltRho(alt,DISA);
        Veq = V*sqrt(sigma);

        T = 518.67*theta;
        M = V/sqrt(1.4*1716*T);

        if M > 0.8
            cdw = 0.075*sin((M-0.8)*pi/0.2-0.5*pi) + 0.075;
        end
        Cd = Cd + cdw*0.4;
        D = 0.5*rho*V^2*S*Cd;
    end

    function thrust = thrustMinCalc(h,DISA)
        [~,~,~,Thrust_lapse,~] = AltRho(h,DISA);
        thrust = ThrustMinSL*Thrust_lapse;
    end

    function thrust = thrustMaxCalc(h,DISA)
        [~,~,~,Thrust_lapse,~] = AltRho(h,DISA);
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
    viscosity = (2.2697e-8*temperature^1.5)/(temperature+198.72);
    V_a = sqrt(1.4*1716*temperature);
    
end


