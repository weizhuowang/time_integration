clc,clear;
% dFuel = SolveFuel(3000);
% TI = load('temp/timeint_temp.mat');
figure(5);clf
for dMTOW = 0:-10000:-30000
    fuel_remain = [];
    Wpay = 94300; % default payload
    midpt = Wpay+dMTOW;
    Payload = [0,midpt,150000];
    PR_log = [];
    for i = 1:length(Payload)
        dPay = Payload(i)-Wpay;
        R0 = 3500;
        for j = 1:20
            [dFuel,badpt] = SolveFuel(R0,dPay,false,@aircrafts.SetupSim_AIAA_folding , dMTOW);
            if badpt
                R0 = R0-200;
            else
                R0 = R0 + (-dFuel)/27;
                if abs(dFuel/27)<5
                    break
                end
            end
        end
        fprintf('\nPayload %2.0f R0: %2.1f \n\n',[Payload(i),R0]);
        PR_log(i,:) = [Payload(i),R0];
    end


    plot(PR_log(:,2),PR_log(:,1),'b','linewidth',2);grid on;hold on;
    plot([0,PR_log(end,2)],PR_log(end,1)*[1 1],'b','linewidth',2);hold on;
    ylim([0,16e4]);xlim([0,5000])
    drawnow
end

% saveas(gcf,'plots\PRD.png')

%% Plot the rest

xlabel('Range [nmi]');ylabel('Payload Weight [lb]');

dp = [1451,1084,707,321];
dist_dp = mean(diff(dp))

lp = [3883,94300;3983,84300;4083,74300;4183,64300];
dist_lp = mean(diff(lp(:,1)))
for i = 1: 1
    pt1 = [dp(end) + dist_dp*i,150000]
    pt2 = [lp(end,1) + dist_lp*i,lp(end,2)-10000*i]
    plot([pt1(1),pt2(1)],[pt1(2),pt2(2)],'b','linewidth',2);grid on;hold on;
end
ax = gca;ax.YAxis.Exponent = 0;
xlim([0,5000]);ylim([0,16e4]);

function fuel_remain = logParam(fname)

    load(fname,'m_maxFuel','Fuel_burnt','BlockFuel');
    fuel_remain = m_maxFuel - Fuel_burnt - 0.05*BlockFuel;
    delete(fname)
    
end