clc,clear;
tic
% [fname,badpt] = time_integration(@aircrafts.SetupSim_AIAA_folding300,true,true,false,{-20000,+55700,0,700,0.78,35000});
[fname,badpt] = time_integration(@aircrafts.SetupSim_AIAA_folding300,true,true,true,{-11500,0,0,3500,0.78,35000});
toc
TI = load(fname);
fuel_remain = TI.m_maxFuel - TI.Fuel_burnt - 0.05*TI.BlockFuel

%% Plot
figure(1);clf;
yyaxis left
plot(TI.log_t/60,TI.log_h,'Linewidth',2);grid on;hold on;
ylabel('Altitude [ft]')
ax = gca; ax.XAxis.Exponent = 0; yyleft = ax.YAxis(1); yyleft.Exponent = 0;
yyaxis right
plot(TI.log_t/60,TI.log_V,'Linewidth',2);grid on;hold on;
plot(TI.log_t/60,TI.log_ROC,'Linewidth',2);grid on;hold on;
xlabel('time [min]');ylabel('ROC [fpm] & V [ft/s]')
xlim([0,29370/60])

figure(2);clf
plot(TI.log_t,TI.log_Fuel);grid on;hold on;
xlabel('time [s]');ylabel('Fuel remaining [lb]');
ax = gca; ax.XAxis.Exponent = 0;ax.YAxis.Exponent = 0;

% figure(3); clf
% plot(TI.log_t,TI.log_Cl./TI.log_Cd,'lineWidth',2);grid on;hold on;
% plot(TI.log_t,sqrt(TI.log_Cl)./TI.log_Cd,'lineWidth',2);grid on;hold on;
% plot(TI.log_t,(TI.log_Cl).^1.5./TI.log_Cd,'lineWidth',2);grid on;hold on;
% drawsegment(TI.log_t,TI.log_stage,[0,25])
% legend('CL/CD','CL^{1/2}/CD','CL^{3/2}/CD')
% 
% figure(6); clf
% scatter(TI.log_M,TI.log_Cl./TI.log_Cd,'lineWidth',2);grid on;hold on;
% scatter(TI.log_M,sqrt(TI.log_Cl)./TI.log_Cd,'lineWidth',2);grid on;hold on;
% scatter(TI.log_M,(TI.log_Cl).^1.5./TI.log_Cd,'lineWidth',2);grid on;hold on;
% % drawsegment(TI.log_t,TI.log_stage,[0,25])
% legend('CL/CD','CL^{1/2}/CD','CL^{3/2}/CD')

figure(4); clf
plot(TI.log_t,TI.log_Cd);grid on;hold on;
plot(TI.log_t,TI.log_Cl);grid on;hold on;
xlabel('time [s]');ylabel('Cl & Cd');legend('Cd','Cl')

figure(5);%clf
% plot(TI.log_t,TI.log_thrustUsed./TI.log_thrustAval*100,'Linewidth',2);grid on;hold on;
pltidx = find((TI.log_t<27305).*(TI.log_t>2645));
plot(TI.log_t(pltidx),TI.log_thrustUsed(pltidx)./TI.log_thrustAval(pltidx)*100,':','Linewidth',2);grid on;hold on;
xlabel('time [s]');ylabel('Throttle [%]')
ylim([0,105])
%%
clc,clear;
% M = [0.75,0.8,0.83,0.85,0.875,0.9]'
% Cdw = [0,0.0013,0.00193,0.0023,0.0050,0.0071]'
M = [0.61 0.63	0.67	0.71	0.75	0.79	0.8	0.83	0.85	0.875	0.9]'
Cdw = [0 1.46E-04 3.51E-04 4.97E-04 7.93E-04 1.05E-03 1.32E-03 0.001929346 0.002338602 4.97E-03 7.13E-03]';

figure(10);clf;
plot(M,Cdw)
f = fit(M,Cdw,'poly4')
plot(f,M,Cdw)

%      Linear model Poly3:
%      f(x) = p1*x^3 + p2*x^2 + p3*x + p4
%      Coefficients (with 95% confidence bounds):
%        p1 =       3.153  (-7.747, 14.05)
%        p2 =      -7.468  (-34.47, 19.53)
%        p3 =       5.915  (-16.33, 28.16)
%        p4 =      -1.565  (-7.662, 4.531)

%%
figure(1);clf;
yyaxis left
plot(TI.log_t,TI.log_h);grid on;hold on;
ylabel('Altitude [ft]')
yyaxis right
plot(TI.log_t,TI.log_V);grid on;hold on;
plot(TI.log_t,TI.log_ROC);grid on;hold on;
xlabel('time [s]');ylabel('ROC [fpm] & V [ft/s]')

figure(3); clf
plot(TI.log_t,TI.log_Cd,'lineWidth',2);grid on;hold on;
yyaxis right
plot(TI.log_t,TI.log_D,'lineWidth',2);grid on;hold on;
drawsegment(TI.log_t,TI.log_stage,[0,5e4])
legend('Cd','D')

%%
figure(2);clf
idx = find(TI.log_stage<6.5);
plot(TI.log_t(idx),TI.log_Fuel(idx),'linewidth',2);grid on;hold on;
xlabel('time [s]');ylabel('Fuel remaining [lb]');
drawsegment(TI.log_t,TI.log_stage,[0,12e4])
ax = gca; ax.XAxis.Exponent = 0;ax.YAxis.Exponent = 0;
ylim([0e4,10e4]);xlim([0,29280])

%%
function drawsegment(log_time,log_stage,segY)

    stageprog = diff(log_stage);
    seg_time = log_time(stageprog>0)
    for t = seg_time
        plot([t,t],segY,'--b');hold on;
    end

end