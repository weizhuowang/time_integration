clc,clear;
% dFuel = SolveFuel(3000);
% TI = load('temp/timeint_temp.mat');

fuel_remain = [];
Mlist = 0.7:0.01:0.84;
hlist = 30000:1000:38000;

for i = 1:length(Mlist)
    parfor j = 1:length(hlist)
        fprintf('\n======Parameters: M %4.2f h %5.0f========\n',[Mlist(i),hlist(j)])
        [fname,badpt] = time_integration(@aircrafts.SetupSim_AIAA_folding300,false,false,false,{-7000,0,0,3500,Mlist(i),hlist(j)});
        if badpt
            fuel_remain(i,j) = nan;
        else
            fuel_remain(i,j) = logParam(fname);
        end
        fprintf('Fuel Remain %2.0f lb\n',[fuel_remain(i,j)])
    end
end

fuel_remain(fuel_remain<0) = nan;
%%
figure(4);clf;
[X,Y] = meshgrid(Mlist,hlist);
minpos = find(fuel_remain' == min(fuel_remain,[],'All'))
% subplot(1,2,1)
contourf(X,Y,fuel_remain','showtext','on');grid on;hold on;
scatter(X(minpos),Y(minpos),130,'rx','linewidth',2);hold on;
xlabel('Cruising Mach');ylabel('Cruising Altitude [ft]')
ylim([3.1e4,3.8e4])
ax = gca; ax.YAxis.Exponent = 0;

% ytickformat('%.5d')
% subplot(1,2,2)
% surf(X,Y,fuel_remain');grid on;
% xlabel('Cruising Mach');ylabel('Cruising Altitude [ft]'); zlabel('Fuel Used for mission [lb]')
% ax = gca; ax.YAxis.Exponent = 0; ax.ZAxis.Exponent = 0;
% saveas(gcf,'plots\MHtrade.png')
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

function [fuel_remain] = logParam(fname)

    load(fname,'m_maxFuel','Fuel_burnt','BlockFuel','BlockRange');
    fuel_remain = m_maxFuel - Fuel_burnt - 0.05*BlockFuel;
    if fuel_remain < 0
        fuel_remain = nan
    else
        fuel_remain = BlockFuel;
    end
    fprintf("Fuel remain %2.0f range % 2.1f\n",[fuel_remain,BlockRange])
    delete(fname)
    
end