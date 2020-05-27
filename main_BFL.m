clc,clear;
format bank
OEI = 1
d_ag = [];
d_as = [];

V1range = 1:10:320;
for accelgo = 0:1
    for V1 = V1range
        [fname,badpt] = time_integration_TO(@aircrafts.SetupSim_AIAA_folding300,true,true,{0,0,0,3500,0.8,35000},OEI,accelgo,V1);

        TI = load(fname);
        TI.Vr
        if accelgo
            idx_crit = find(TI.log_V>min(TI.Vr,TI.V1),1);
            d_crit = TI.log_d(idx_crit)
            d_ag(end+1) = TI.log_d(end);
        else
            idx_crit = find(TI.log_V>min(TI.Vr,TI.V1),1);
            d_crit = TI.log_d(idx_crit)
            d_as(end+1) = TI.log_d(end);
        end
    end
end

%% BFL
figure(5);clf;
BFLpt = [165.5,7850];
plot(V1range/1.6878,d_ag,'linewidth',2);grid on; hold on;
plot(V1range/1.6878,d_as,'linewidth',2);grid on; hold on;
h1 = scatter(V1range/1.6878,d_ag,80,'o','filled');grid on; hold on;
h2 = scatter(V1range/1.6878,d_as,80,'s','filled');grid on; hold on;
% scatter(BFLpt(1),BFLpt(2),80,'filled');hold on;
plot(BFLpt(1)*[1,1],[0,BFLpt(2)],'--k','linewidth',2); hold on;
plot(BFLpt(1)*[0,1],BFLpt(2)*[1,1],'--k','linewidth',2); hold on;
plot((BFLpt(1)+1)*[1,1],[0,15000],'--r','linewidth',2); hold on;
text(BFLpt(1)+1,10000,'Vr','fontsize',20)
xlabel('V1 [KCAS]'); ylabel('BFL [ft]')
legend([h1,h2],{'Accelerate-Go','Accelerate-Stop'})
% saveas(gcf,'plots\BFL.png')
%%
figure(6);clf;
[~,idx] = min(abs(d_ag-d_as));
V1 = V1range(idx)
V1 = 279
BFL = (d_ag(idx) + d_as(idx))/2

[fname,badpt] = time_integration_TO(@aircrafts.SetupSim_AIAA_folding,true,true,{0,0,0,3500,0.8,35000},1,accelgo,V1);
TI = load(fname);
[fname,badpt] = time_integration_TO(@aircrafts.SetupSim_AIAA_folding,true,true,{0,0,0,3500,0.8,35000},1,~accelgo,V1);
TI1 = load(fname);

idx_V1 = find(TI.log_V>min(TI.Vr,TI.V1),1);
d_V1 = TI.log_d(idx_V1)
idx_Vr = find(TI.log_V>TI.Vr,1);
d_Vr = TI.log_d(idx_Vr)

h1 = plot(TI.log_d,TI.log_h,'linewidth',2);grid on;hold on;
h2 = plot(TI1.log_d,TI1.log_h,'linewidth',2);grid on;hold on;
plot([0,9000],[0,0]-1,'k','linewidth',2);hold on;
plot([9000,9000],[0,35],'r','linewidth',2)
scatter(d_V1,0,100,'filled');% text(d_V1-50,5,'V1','FontSize',15)
scatter(d_Vr,0,100,'filled');% text(d_Vr-50,5,'Vr OEI','FontSize',15)
% axis equal

[fname,badpt] = time_integration_TO(@aircrafts.SetupSim_AIAA_folding,true,true,{0,0,0,3500,0.8,35000},0,accelgo,V1);
TI = load(fname);

idx_Vr = find(TI.log_V>TI.Vr,1);
d_Vr = TI.log_d(idx_Vr)

h3 = plot(TI.log_d,TI.log_h,'--','linewidth',2);grid on;hold on;
scatter(d_Vr,0,100,'filled');% text(d_Vr-100,5,'Vr AEO','FontSize',15)

legend([h1,h2,h3],{'OEI continue','OEI abort','AEO continue'})
xlabel('Runway used [ft]');ylabel('Altitude [ft]')
saveas(gcf,'plots\TOpath.png')
%%

%%

function [fname,bad] = time_integration_TO(SetupSim,logflag,verbose,args,OEI,accelgo,V1)
    % ===================Sim setup====================
    t = 0; dt = 0.05; tend = 1200*60; stage = 1; stopsim = false;
    g = 32.17405; DISA = 15/288;bad = false; abort = false;
    % ===================Init====================
    [V,L_to,h,D,Fuel_burnt,R,L_land,Thrust_used,Cl,ROC,a,t_rotate,...
     t_flair,t_loiter_start,Cd,BlockFuel,Vr] = deal(0);
    % ===================Logger====================
    if logflag
        [log_t,log_ROC,log_h,log_V,log_Fuel,log_Cl,log_Cd,log_d,log_thrustUsed] = deal(zeros(1,tend/dt));
    end
    fname = SetupSim(args{:});
    load(fname,...
         'm0','m','mu_roll','mu_slide','V_cruise','h_cruise',...
         'R_cruise_end','m_maxFuel','V_init_approach','h_approach',...
         'Cl_max_land','Clmax_TO','S','ThrustSL','ThrustMinSL','SFC_Cruise',...
         'Cdp','K')
    ThrustMinSL_Full = ThrustMinSL;
    ThrustSL_Full = ThrustSL;
%     mu_slide = 0.27; % wet
    mu_slide = 0.09; % icy
    for i = 1:tend/dt
        
        % ==========Prepare============
        [rho,~,~,~,~,V_a] = AltRho(h,DISA);
        SFC = SFCcalc(h,V/V_a);
        
        if OEI && V>V1
            ThrustMinSL = ThrustMinSL_Full/2;
            ThrustSL = ThrustSL_Full/2;
        end
        
        switch stage
            case 1   % take off
                if accelgo
                    takeoff(15/288)
                else
                    abort_takeoff(15/288)
                end
%             case 2   % climb
%                 climb(0,V_cruise,h_cruise)
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
    function takeoff(DISA)
%         DISA = 15/288;
        Vr = sqrt(2*m0/(rho*S*Clmax_TO*0.9))*1.12;
        Thrust = thrustMaxCalc(h,DISA);
        Thrust_used = Thrust;
        if V < Vr
            Cl = 0.5;
            ROC = 0;
            L = 0.5*rho*V^2*S*Cl*0.9;
            F_forward = Thrust*2 - mu_roll*(m-L);
            t_rotate = t;
        else
            Cl = (2*m/(rho*V^2*S))*1.2;
            ROC = ((Thrust*2-D)*g*V)/(m*g)*60*min(dt*i-t_rotate,3)/9; ROC = min(ROC,2000);
            F_forward = (Thrust*2*V-ROC/60*m)/V;
        end
        [D,Cd] = Dcalc(V,Cl,h,DISA);
        a = (F_forward-D)*g/m;
        L_to = L_to + V*dt;
        if h>35
            stage = stage+1;
        end 
    end
    
    function abort_takeoff(DISA)
%         DISA = 15/288;
        Vr = sqrt(2*m0/(rho*S*Clmax_TO))*1.2;
        Thrust = thrustMaxCalc(h,DISA);
        ThrustMin = thrustMinCalc(h,DISA);
        
        if ~abort && (V < min(V1,Vr))
            Cl = 0.5;
            ROC = 0;
            L = 0.5*rho*V^2*S*Cl;
            F_forward = Thrust*2 - mu_roll*(m-L);
            t_rotate = t;
            abort = false;
            Thrust_used = Thrust;
        else
            Cl = 0.2; ROC = 0;
            L = 0.5*rho*V^2*S*Cl;
            F_forward = ThrustMin*2 - mu_slide*(m-L);
            t_rotate = t;
            abort = true;
            Thrust_used = ThrustMin;
        end
        [D,Cd] = Dcalc(V,Cl,h,DISA);
        a = (F_forward-D)*g/m;
        L_to = L_to + V*dt;
        
        if abort && V<10
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

        if ROC < 200
            if verbose fprintf('Reached service ceiling at %2.0f\n',[h]); end
            t_climb = t
            ROC = 0;
            stage = stage+1;
            bad = true;
        end
        if h > 100
            t_climb = t;
            if verbose fprintf('climb time %2.0f range: %2.1f Fuel_burnt: %2.0f\n',[t_climb,R,Fuel_burnt]); end
            ROC = 0;
            stage = stage+1;
        end
    end

    % ============Sim module=============
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