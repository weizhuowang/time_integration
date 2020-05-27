clc,clear;warning off
% Loop Preperation
fname = aircrafts.SetupSim_AIAA_folding300(0,0,0,3500,0.8,35000);
load(fname)
gamma = 1.4;
P0 = 2116.22; m0 = m0-000;
maxQ = 0.5*0.0023769*541.6^2; % psf
[h,Ta,rho,delta,Va] = deal([]);

for alt = [1:250:38500,38501:10:40900,40901:1:41100]
    h(end+1) = alt;
    Ta(end+1) = thrustMaxCalc(ThrustSL,alt,0)*2;
    [rho(end+1),~,~,delta(end+1),~,Va(end+1)] = AltRho(alt,0);
end

syms V real
M = V./Va;
% Cdw = 3.153*M.^3 -7.468*M.^2 + 5.915*M -1.565671875;
Cdw = 4.31023888595203*M.^4 - 12.0494768849716*M.^3 + 12.5961775780605*M.^2 - 5.83015669954579*M + 1.007571399885269;

Cdp = Cdp + 0.002;
A = (1/2).*rho.*(S*(Cdp));
A_cdw = (1/2).*rho.*(S*(Cdp+Cdw));
B = (2*m0^2*K)./(rho.*S);

% V_min
V_min = sqrt((Ta - sqrt(Ta.^2-4.*A.*B))./(2.*A));
V_min = V_min./Va;

% V_stall
V_stall = sqrt(2.*m0./(rho.*S.*Clmax_TO));
V_stall = V_stall./Va;
% V_stall = V_stall(1:i-1);

% V_maxQ
for i = 1:length(h)
    V_maxQ(i) = sqrt(maxQ/(0.5*rho(i)));
end
V_maxQ = V_maxQ./Va;

% V_max
V_max = sqrt((Ta + sqrt(Ta.^2-4.*A.*B))./(2.*A));
V_max = V_max./Va;

eqn = V == sqrt((Ta + sqrt(Ta.^2-4.*A_cdw.*B))./(2.*A_cdw));
guess = 1000;
for i = 1:length(eqn)
    h(i)
    soln = vpasolve(eqn(i),V,guess);
    guess = soln;
    if length(soln) > 0
        V_max_cdw(i) = soln;
    else
        break;
    end
end
V_max_cdw = double(V_max_cdw./Va(1:i-1));
V_max = V_max(1:i-1);
V_max(V_max>0.61) = V_max_cdw(V_max>0.61);

%% ===================================
[~,abshidx] = min(V_max_cdw)
h_Plot = h(abshidx);

V_min_Plot = V_min(V_min>V_stall);
h_min_Plot = h(V_min>V_stall);
V_min_Plot = V_min_Plot(h_min_Plot<=h_Plot);
h_min_Plot = h_min_Plot(h_min_Plot<=h_Plot);

% V_max = min(V_max,V_maxQ(1:length(V_max)));
V_max_Plot = V_max(h(1:length(V_max))<=h_Plot);
h_max_Plot = h(h(1:length(V_max))<=h_Plot);


[~,stall_idx] = min(abs(V_stall-V_min));
% clf;
figure(1)
set(gcf, 'PaperUnits', 'centimeters');set(gcf, 'Position', [400 100 1100 800]);
plot(V_stall(1:stall_idx),h(1:stall_idx),'-','Linewidth',2.5);hold on;grid on;
plot(V_min_Plot,h_min_Plot,':','Linewidth',2.5)
plot(V_max_Plot,h_max_Plot,'--','Linewidth',2.5)
plot(V_maxQ(1:length(V_max)),h_max_Plot,'--','Linewidth',2.5)
plot([0,max(V_max_Plot)+0.2],[h_Plot,h_Plot],'Linewidth',2.5)
ylim([0,40000])
xlim([0,max(V_max_Plot)+0.15])
legend('V_{Stall}','V_{Min}','V_{Max}','Structural limit','Absolute Ceiling')
ax = gca;
ax.YAxis.Exponent = 0;
ylabel('Altitude [ft]')
xlabel('Mach Number')
set(gca,'fontsize',12,'ticklabelinterpreter','latex')

% saveas(gcf,'plots\FED.png')
function thrust = thrustMaxCalc(ThrustSL,h,DISA)
    Thrust_lapse = AltLapse(h,DISA);
    thrust = ThrustSL*Thrust_lapse;
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

function [delta] = AltLapse(h,DISA)
    
    if h<36089
        theta = 1-6.87535e-6*h+DISA;
        delta = (theta-DISA)^5.2561;
    else
        delta = 0.22336*exp((36089-h)/20806.7);
    end
    
end