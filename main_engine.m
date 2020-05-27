clc,clear;
addpath(genpath('.'))
% tic;
% [fname,badpt] = time_integration(@aircrafts.SetupSim_AIAA_folding,false,true,{100,0,3500,0.8,35000});
% toc;
% load(fname)
figure(4);clf
alt = [0:10000:30000,35000]
spec = {'-','--','-.',':','+'};
textlb = {'SL','10k ft','20k ft','30k ft','35k ft'}
for i = 1:length(alt)
    SFC = [];
    for M = 0:0.02:1
        SFC(end+1) = SFCcalc(alt(i),M);
    end

    plot(0:0.02:1,SFC,spec{i},'linewidth',2);hold on;grid on;
    text(0.96,SFC(end),textlb(i),'fontsize',13)
end
xlabel('Mach Number');ylabel('SFC [lb/lbf-hr]');
saveas(gcf,'plots\sfc.png')

%%
figure(4);clf
Ta = [];
for alt = 0:44000
    
    [lapse] = AltLapse(alt,0);
    Ta(end+1) = lapse*62500;
    
end
plot(Ta/1000,[0:44000]/1000,'linewidth',2);hold on;grid on;
xlabel('Thrust Available [x1000 lbf]');ylabel('Altitude [x1000 ft]');
saveas(gcf,'plots\AltTa.png')

function [delta] = AltLapse(h,DISA)
    
    if h<36089
        theta = 1-6.87535e-6*h+DISA;
        delta = (theta-DISA)^5.2561;
    else
        delta = 0.22336*exp((36089-h)/20806.7);
    end
    
end

function sfc = SFCcalc(h,M)
    SFC_Cruise = 0.43;
    sfc = (0.324*0.85 + (SFC_Cruise-0.324*0.85)*M/0.83)*(1.15-h/35000*0.15);
end