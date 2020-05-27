clc,clear;
[dFuel] = SolveFuel(700,0,true,@aircrafts.SetupSim_AIAA_folding300 , 0)


%% Plot
figure(1);clf;
yyaxis left
plot(TI.log_t,TI.log_h);grid on;hold on;
ylabel('Altitude [ft]')
yyaxis right
plot(TI.log_t,TI.log_V);grid on;hold on;
plot(TI.log_t,TI.log_ROC);grid on;hold on;
xlabel('time [s]');ylabel('ROC [fpm] & V [ft/s]')

figure(2);
plot(TI.log_t,TI.log_Fuel);grid on;hold on;

figure(3); clf
plot(TI.log_t,TI.log_Cd);grid on;hold on;

% yyaxis left
% ylim([0,200])
% xlim([2.8935e4,2.8975e4])

function fuel_remain = logParam(fname)

    load(fname,'m_maxFuel','Fuel_burnt','BlockFuel');
    fuel_remain = m_maxFuel - Fuel_burnt - 0.05*BlockFuel;
    delete(fname)
    
end