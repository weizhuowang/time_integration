clc,clear;

time_integration(@aircrafts.SetupSim_AIAA_nonfolding)
load('temp/timeint_temp.mat')
Fuel_burnt
L_to

% figure(1);clf;
% yyaxis left
% plot(log_t(1:i),log_h(1:i));grid on;hold on;
% ylabel('Altitude [ft]')
% yyaxis right
% plot(log_t(1:i),log_V(1:i));grid on;hold on;
% plot(log_t(1:i),log_ROC(1:i));grid on;hold on;
% xlabel('time [s]');ylabel('ROC [fpm] & V [ft/s]')
% 
% figure(2);
% plot(log_t(1:i),log_Fuel(1:i));grid on;hold on;

% yyaxis left
% ylim([0,200])
% xlim([2.8935e4,2.8975e4])
