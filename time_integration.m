% https://s3.amazonaws.com/szmanuals/6c05fe5e978d2e2e229955e3a35e9717
% https://web.archive.org/web/20140802021208/http://www.boeing.com/assets/pdf/commercial/startup/pdf/777_perf.pdf
% https://www.predictivemobility.com/family-777
% https://www.boeing.com/history/products/777.page
% https://www.fzt.haw-hamburg.de/pers/Scholz/GF/SEECKT-RE-KTH_Re-Design_B777_08-09-28.pdf

function [fname,bad] = time_integration(SetupSim,logflag,verbose,stepflg,args)
    fname = SetupSim(args{:});
    load(fname,...
         'm0','m','mu_roll','mu_slide','V_cruise','h_cruise',...
         'R_cruise_end','m_maxFuel','V_init_approach','h_approach',...
         'Cl_max_land','Clmax_TO','S','ThrustSL','ThrustMinSL','SFC_Cruise',...
         'Cdp','K')    
    % ===================Sim setup====================
    t = 0; dt = 0.1; tend = 1200*60; stage = 1; stopsim = false;
    g = 32.17405; DISA = 0;bad = false;thrustAval = ThrustSL;
    % ===================Init====================
    [V,L_to,h,D,Fuel_burnt,R,L_land,Thrust_used,Cl,ROC,a,t_rotate,...
     t_flair,t_loiter_start,Cd,BlockFuel,BlockRange,M] = deal(0);
    % ===================Logger====================
    if logflag
        [log_t,log_ROC,log_h,log_V,log_Fuel,log_Cl,log_Cd,log_thrustUsed,log_stage,...
         log_M,log_D,log_thrustAval] = deal(zeros(1,tend/dt));
    end
    
    for i = 1:tend/dt
        
        % ==========Prepare============
        [rho,~,~,~,~,V_a] = AltRho(h,DISA);
        SFC = SFCcalc(h,V/V_a,Thrust_used/(thrustAval*0.70));
        
        switch stage
            case 1   % take off
                takeoff(15/288)
            case 2   % climb
                climb(0,V_cruise,h_cruise)
            case 3   % cruise
                cruise(0,R_cruise_end-130*(h_cruise/35000))
                if stepflg && R > 1000
                    need_climb = stepclimb(DISA);
                    if need_climb && R<R_cruise_end-200
                        V_cruise = V;
                        h_cruise = h+2000;
                        stage = 2;
                    end
                end
            case 4   % descent
                descent(0,V_init_approach,h_approach)
            case 5
                loiter(DISA,5*60)
            case 6   % approach
                approach(0)
            
            case 7 % divert climb
                climb(0,500,15000)
            case 8 % divert cruise
                cruise(0,R_cruise_end+133)
            case 9 % divert descent
                descent(0,V_init_approach,h_approach)
            case 10 % divert loiter
                loiter(DISA,35*60)
            case 11 % divert approach
                approach(0)
            
            case 12 % land
                land(15/288)
            otherwise
                stopsim = true;
        end
        
        if stopsim break, end
        if bad break, end
        
        % ==========AfterSim=============
        StepSim() % 62-77 is for speed
%         V = V + a*dt;
%         h = h + ROC/60*dt;
%         R = R + (sqrt(V^2-(ROC/60)^2)*dt)/6076.11;
%         Fuel_burnt = Fuel_burnt + SFC*Thrust_used*2/3600*dt;
%         m = m0-Fuel_burnt;
%         t = t + dt;
%         if Fuel_burnt > m_maxFuel
%             if verbose fprintf('Out of Fuel at stage %2.0f range %2.0f \n',[stage,R]); end
%             bad = true;
%         end
%         
%         if logflag
%             log_t(i) = t; log_ROC(i) = ROC; log_h(i) = h;
%             log_V(i) = V; log_Fuel(i) = m_maxFuel-Fuel_burnt;
%             log_Cl(i) = Cl; log_Cd(i) = Cd;log_thrustUsed(i) = Thrust_used;
%             log_stage(i) = stage; log_M(i) = M;log_D(i) = D;
%             log_thrustAval(i) = ;
%         end
        
    end

    ExitSim()

    % Flight stages
    function takeoff(DISA)
%         DISA = 15/288;
        Vr = sqrt(2*m0/(rho*S*Clmax_TO*0.9))*1.12;
        Thrust = thrustMaxCalc(h,DISA);
        Thrust_used = Thrust; % TODO: update thrust used
        if V < Vr
            Cl = 0.5;
            ROC = 0;
            L = 0.5*rho*V^2*S*Cl*0.9;
            F_forward = Thrust*2 - mu_roll*(m-L);
            t_rotate = t;
        else
            Cl = (2*m/(rho*V^2*S))*1.2;
            ROC = ((Thrust*2-D)*g*V)/(m*g)*60*min(dt*i-t_rotate,3)/9; ROC = min(ROC,1000);
            F_forward = (Thrust*2*V-ROC/60*m)/V;
        end
        [D,Cd] = Dcalc(V,Cl,h,DISA);
        a = (F_forward-D)*g/m;
        L_to = L_to + V*dt;
        if h>35
            stage = stage+1;
        end 
    end

    function climb(DISA,V_cruise,h_cruise)
        Thrust = min(thrustMaxCalc(h,DISA),ThrustSL*0.9);
        Cl = (2*m/(rho*V^2*S));
        [D,Cd] = Dcalc(V,Cl,h,DISA);

        if V < V_cruise  a = 0.5; else a = 0; end
        ROC = ((Thrust*2-D-m*a/g)*g*V)/(m*g)*60; ROC = min(ROC,1700);
        F_forward_max = (Thrust*2*V-ROC/60*m)/V;
        Thrust_used = Thrust - (F_forward_max-D-a*m/g)/2;

    %                 fprintf('%5.0f %2.0f %2.0f %2.0f\n',[h,ROC,V,a])

        if ROC < 100
            if verbose fprintf('Reached service ceiling at %2.0f\n',[h]); end
            t_climb = t
            ROC = 0;
            stage = stage+1;
            bad = true;
        end
        if h > h_cruise
            t_climb = t;
            if verbose fprintf('climb time %2.0f range: %2.1f Fuel_burnt: %2.0f\n',[t_climb,R,Fuel_burnt]); end
            ROC = 0;
            stage = stage+1;
        end
        if (R > R_cruise_end-20) && (stage < 3)
            if verbose fprintf('Climb terminated due to range\n'); end
            stage = stage+1;
        end    
    end

    function cruise(DISA,R_cruise_end)
        dt = 2;
        ROC = 0;a = 0;
        Cl = (2*m/(rho*V^2*S));
        [D,Cd] = Dcalc(V,Cl,h,DISA);
        Thrust_used = D/2;
        
        ThrustMax = thrustMaxCalc(h,DISA);
        if Thrust_used > ThrustMax
            if verbose fprintf('Unable to cruise at this altitude, T_req: %2.0f T_avail: %2.2f\n',[Thrust_used,ThrustMax]); end
            bad = true;
        end
        if R > R_cruise_end
            if verbose fprintf('cruise end at: %2.1f Fuel burnt: %2.0f\n',[R,Fuel_burnt]); end
            stage = stage+1;
            dt = 0.1;
            t_loiter_start = t;
        end     
    end

    function flg = stepclimb(DISA)
        [nextrho,~,~,~,~,~] = AltRho(h+2000,DISA);
        nextThrust = thrustMaxCalc(h+2000,DISA);
        nextCl = (2*m/(nextrho*V^2*S));
        [nextD,~] = Dcalc(V,nextCl,h+2000,DISA);
        next_ROC = ((nextThrust*2-nextD-m*a/g)*g*V)/(m*g)*60;
        if next_ROC > 100
            flg = true;
        else
            flg = false;
        end
    end

    function descent(DISA,V_init_approach,h_approach)
        ThrustMin = thrustMinCalc(h,DISA);
        Cl = (2*m/(rho*V^2*S));
        [D,Cd] = Dcalc(V,Cl,h,DISA);
        if V > V_init_approach a = -0.4; else a = 0; end
        ROCMax = ((ThrustMin*2-D-m*a/g)*g*V)/(m*g)*60; ROC = max(ROCMax,-2000);
        Thrust_used = 0.5*(D + a*m/g + ROC*m/(60*V));
%             fprintf('%5.0f %2.0f %2.0f %2.2f %2.2f\n',[h,ROC,V,a,Thrust_used])
        if h < h_approach
            stage = stage+1;
        end 
    end

    function loiter(DISA,t_loiter)
        ROC = 0;a = 0;
        Cl = (2*m/(rho*V^2*S));
        [D,Cd] = Dcalc(V,Cl,h,DISA);
        Thrust_used = D/2;
        
        ThrustMax = thrustMaxCalc(h,DISA);
        R = R - (sqrt(V^2-(ROC/60)^2)*dt)/6076.11;
        if Thrust_used > ThrustMax
            if verbose fprintf('Unable to cruise at this altitude, T_req: %2.0f T_avail: %2.2f\n',[Thrust_used,ThrustMax]); end
        end
        if t-t_loiter_start > t_loiter
            stage = stage+1;
        end
    end

    function approach(DISA)
        Vstall = sqrt(2*m/(rho*S*2.49*0.9));
        Vapproach = Vstall*1.3;
        if stage == 6
%             fprintf('%2.2f %2.2f\n',[h,Vapproach])
        end
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
            if BlockFuel == 0
                BlockFuel = Fuel_burnt;
                BlockRange = R;
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
        if V<30
            stage = stage+1;
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
        if Fuel_burnt > m_maxFuel
            if verbose fprintf('Out of Fuel at stage %2.0f range %2.0f \n',[stage,R]); end
            bad = true;
        end
        
        thrustAval = thrustMaxCalc(h,0);

        if logflag
            log_t(i) = t; log_ROC(i) = ROC; log_h(i) = h;
            log_V(i) = V; log_Fuel(i) = m_maxFuel-Fuel_burnt;
            log_Cl(i) = Cl; log_Cd(i) = Cd;log_thrustUsed(i) = Thrust_used;
            log_stage(i) = stage; log_M(i) = M;log_D(i) = D;
            log_thrustAval(i) = thrustAval;
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
            log_stage= log_stage(1:i);
            log_M    = log_M(1:i);
            log_D    = log_D(1:i);
            log_thrustUsed = log_thrustUsed(1:i);
            log_thrustAval = log_thrustAval(1:i);
        end
%         save('temp/timeint_temp.mat')
        save(fname);
    end

    % Helper Functions
    %     throttle [0,1]
    function sfc = SFCcalc(h,M,throttle)
        sfc = (0.324*0.85 + (SFC_Cruise-0.324*0.85)*M/0.83)*(1.15-h/35000*0.15);
        corr = 0.3504*throttle^2-0.6984*throttle + 1.3486;
        sfc = sfc*corr;
    end

    function [D,Cd] = Dcalc(V,Cl,alt,DISA)
        cdw = 0;

        Cd = Cdp + 1.1*K*Cl^2; % Cdp = Cd0 + Cdexcr, Cdtrim = 0.1*Cdi

        [rho,~,sigma,~,theta] = AltRho(alt,DISA);
        Veq = V*sqrt(sigma);

        T = 518.67*theta;
        M = V/sqrt(1.4*1716*T);

%         if M > 0.75
% %             fitted delta method
%             cdw = 3.153*M^3 -7.468*M^2 + 5.915*M -1.565671875;
%         end
        if M > 0.61
        % fitted delta method
            cdw = [4.31023888595203,-12.0494768849716,12.5961775780605,-5.83015669954579,1.007571399885269]*[M^4,M^3,M^2,M,1]';
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

