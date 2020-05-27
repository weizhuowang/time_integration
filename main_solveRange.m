clc,clear;
[verbose,dFuel,dPay,dMTOW] = deal(true,0,0,0);
plotRM = []
Mlist = 0.6:0.01:0.85;
for M = Mlist
    [Range,badpt] = SolveRange(false,@aircrafts.SetupSim_AIAA_folding,0,0,0,M,35000);
    if ~badpt
        plotRM(end+1,:) = [M,Range]
    end
end

%%
figure(1);clf;
plot(plotRM(:,1),plotRM(:,2),'linewidth',2);grid on;hold on;
[~,maxidx] = max(plotRM(:,2))
maxpt = plotRM(maxidx,:)
scatter(maxpt(1),maxpt(2),'filled');hold on;
xlabel('Mach');ylabel('Range [nmi]')

