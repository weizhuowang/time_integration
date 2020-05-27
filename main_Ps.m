clc,clear;
% Ps vs M
fname = aircrafts.SetupSim_AIAA_folding300(0,0,0,3500,0.8,35000);
load(fname,...
     'm0','m','mu_roll','mu_slide','V_cruise','h_cruise',...
     'R_cruise_end','m_maxFuel','V_init_approach','h_approach',...
     'Cl_max_land','Clmax_TO','S','ThrustSL','ThrustMinSL','SFC_Cruise',...
     'Cdp','K')

global M Cdp K S
figure(1);clf;
for altitude = 0:10000:40000
    [rho,~,sigma,delta,theta,V_a] = AltRho(altitude,0)
    T = delta*ThrustSL*2;
    Ps = [];

    for V = 200:V_a
        M = V/V_a;
        q = 0.5*rho*V^2;
        Cl = m0/(q*S);
        [D,Cd] = Dcalc(V,Cl,altitude,0);
        Ps(end+1) = V*(T-D)/m0;
    end
    
    plot([200:V_a]/V_a,Ps);grid on;hold on;
end

ylim([0,140])


%%
clc,clear;
% Alt vs M
maxQ = 0.5*0.0023769*541.6^2; % psf
fname = aircrafts.SetupSim_AIAA_folding300(0,0,0,3500,0.8,35000);
load(fname,...
     'm0','m','mu_roll','mu_slide','V_cruise','h_cruise',...
     'R_cruise_end','m_maxFuel','V_init_approach','h_approach',...
     'Cl_max_land','Clmax_TO','S','ThrustSL','ThrustMinSL','SFC_Cruise',...
     'Cdp','K')

global M Cdp K S
altlist = 0:100:40000;
Mlist = 0:0.001:1;
Ps = [];
for i = 1:length(altlist)
    altitude = altlist(i)
    [rho,~,sigma,delta,theta,V_a] = AltRho(altitude,0);
    T = delta*ThrustSL*2;
    
    V_maxQ = sqrt(maxQ/(0.5*rho));
    M_maxQ = V_maxQ./V_a;
    maxQline(i,:) = [altitude,M_maxQ];
    for j = 1:length(Mlist)
        M = Mlist(j);
%         M = V/V_a;
        V = M*V_a;
        q = 0.5*rho*V^2;
        Cl = m0/(q*S);
        [D,Cd] = Dcalc(V,Cl,altitude,0);
        Ps(i,j) = V*(T-D)/m0;
        
        if M > M_maxQ
            Ps(i,j) = NaN;
        end
    end
end

%%
figure(1);clf;
Ps(Ps<0) = NaN;
[X,Y] = meshgrid(Mlist,altlist);
contourf(X,Y,Ps,'showtext','on');grid on;hold on;
% ylim([0,140])
% figure(2);clf;
% surf(X,Y,Ps);grid on;hold on;
% shading interp

delta = 3
optmal_alt = [];
optmal_M = [];
for Pslevel = 0:10:110
    [x,y] = find(Ps>Pslevel-delta & Ps<Pslevel+delta);
    optmal_alt(end+1) = altlist(max(x));
    optmal_M(end+1) = Mlist(round(median(y(x == max(x)))));
end
y = find(Ps(1,:)==max(Ps(1,:)));
optmal_alt(end+1) = 0;
optmal_M(end+1) = Mlist(y);
figure(1);
plot(optmal_M,optmal_alt,'r','Linewidth',2);hold on;
plot(maxQline(1:end-100,2),maxQline(1:end-100,1),':m','Linewidth',5);hold on;
ax = gca; ax.YAxis.Exponent = 0; xlim([0,0.95])
xlabel('Mach');ylabel('Altitude [ft]')
legend('Specific Excess Power [ft/s]','Line of Max Specific Excess Power','Structural Limit')
%% Helper Modules

function [D,Cd] = Dcalc(V,Cl,alt,DISA)
    global M Cdp K S
    cdw = 0;

    Cd = Cdp + 1.1*K*Cl^2; % Cdp = Cd0 + Cdexcr, Cdtrim = 0.1*Cdi

    [rho,~,sigma,~,theta] = AltRho(alt,DISA);
    Veq = V*sqrt(sigma);

    T = 518.67*theta;
    M = V/sqrt(1.4*1716*T);

    if M > 0.61
    % fitted delta method
        cdw = [4.31023888595203,-12.0494768849716,12.5961775780605,-5.83015669954579,1.007571399885269]*[M^4,M^3,M^2,M,1]';
    end
    Cd = Cd + cdw;
    D = 0.5*rho*V^2*S*Cd;
end

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

