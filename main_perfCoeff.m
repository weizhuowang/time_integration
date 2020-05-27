clc,clear;
% Inits
Cdp = 0.0171794274781596;
K = 0.0463130927082483;
cdw = 0;
alt = 35000;
S = 3880;
DISA = 0;
m = 442590;

log_Cl = [];log_Cd = [];log_V = [];log_M = [];
for V = 0:1000

    [rho,~,sigma,~,theta] = AltRho(alt,DISA);
    Veq = V*sqrt(sigma);

    T = 518.67*theta;
    M = V/sqrt(1.4*1716*T);

    if M > 0.61
    % fitted delta method
        cdw = [4.31023888595203,-12.0494768849716,12.5961775780605,-5.83015669954579,1.007571399885269]*[M^4,M^3,M^2,M,1]';
    end
    
    Cl = (2*m/(rho*V^2*S));
    Cd = Cdp + 1.1*K*Cl^2 + cdw;
    log_Cl(end+1) = Cl; log_Cd(end+1) = Cd;
    log_V(end+1) = V;   log_M(end+1) = M;
end

%%
figure(6); clf
plot(log_M,log_Cl./log_Cd,'-','lineWidth',2);grid on;hold on;
plot(log_M,sqrt(log_Cl)./log_Cd,'--','lineWidth',2);grid on;hold on;
plot(log_M,(log_Cl).^1.5./log_Cd,':','lineWidth',2);grid on;hold on;

[val,idx] = max(log_Cl./log_Cd);
scatter(log_M(idx),val,80,'s','filled');hold on;
text(log_M(idx)-0.02,val+1,num2str(val,3),'fontsize',13)
[val,log_M(idx)]
[val,idx] = max(sqrt(log_Cl)./log_Cd);
scatter(log_M(idx),val,80,'filled');hold on;
text(log_M(idx)-0.02,val+1,num2str(val,3),'fontsize',13)
[val,log_M(idx)]
[val,idx] = max((log_Cl).^1.5./log_Cd);
scatter(log_M(idx),val,80,'>','filled');hold on;
text(log_M(idx)-0.02,val+1,num2str(val,3),'fontsize',13)
[val,log_M(idx)]
legend('CL/CD','CL^{1/2}/CD','CL^{3/2}/CD')
xlabel('Mach Number');ylabel('Coefficient Magnitude')

saveas(gcf,'plots\perfcoeffs.png')
%%
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