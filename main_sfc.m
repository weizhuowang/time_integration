clc,clear;
% SFC plot
altlist = [2.5e4,3e4,4e4,4.5e4];
throlist = 0.25:0.01:1.42857;
sfc = []; Ta = [];

for i = 1:length(altlist)
    alt = altlist(i);
    Ta(i,:) = thrustMaxCalc(alt,0)*throlist;
    for j = 1:length(throlist)
        throttle = throlist(j);
        sfc(i,j) = SFCcalc(alt,0.8,throttle);
    end
end

linelabel = {'25k ft','30k ft','40k ft','45k ft'}
linelist = {'-','--','-.',':'};
figure(1);clf;
for i = 1:length(altlist)

    plot(Ta(i,:),sfc(i,:),linelist{i},'linewidth',2);hold on;grid on;
    text(Ta(i,end-45),sfc(i,end-45)-0.007,linelabel{i},'fontsize',14)
    
end

legend('25k ft','30k ft','40k ft','45k ft')
xlabel('Thrust Available Per Engine [lbf]');ylabel('SFC [lb/lbf-hr]')
ax = gca; ax.XAxis.Exponent = 0;

function sfc = SFCcalc(h,M,throttle)
    SFC_Cruise = 0.43;
    sfc = (0.324*0.85 + (SFC_Cruise-0.324*0.85)*M/0.83)*(1.15-h/35000*0.15);
    corr = 0.3504*throttle^2-0.6984*throttle + 1.3486;
    sfc = sfc*corr;
end

function thrust = thrustMaxCalc(h,DISA)
    ThrustSL = 62500*0.7;
    Thrust_lapse = AltLapse(h,DISA);
    thrust = ThrustSL*Thrust_lapse;
end

function [delta] = AltLapse(h,DISA)
    
    if h<36089
        theta = 1-6.87535e-6*h+DISA;
        delta = (theta-DISA)^5.2561;
    else
        delta = 0.22336*exp((36089-h)/20806.7);
    end
    
end