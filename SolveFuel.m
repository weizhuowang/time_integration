% dFuel is suggested change in fuel. E.g. dFuel = -5000 means you can carry
% 5000 lb less fuel
function [dFuel,badpt] = SolveFuel(R,dPay,verbose,SetupSim,dMTOW)
    dFuel = 0;
    for i = 1:8
        if verbose fprintf('\n=============Iteration %2.0f ===========\n',[i]);end
        [fname,badpt] = time_integration(SetupSim,false,verbose,false,{dFuel,dPay,dMTOW,R,0.78,35000}); % 0.792 37500
        if badpt
            fprintf('!!!!!!!!!!!!!!badpoint, burnt all the fuel\n');
            break
        else
            TI = load(fname,'m_maxFuel','Fuel_burnt','BlockFuel','L_to','L_land',...
                'R','m_maxFuel');

            fuel_remain = TI.m_maxFuel - TI.Fuel_burnt - 0.05*TI.BlockFuel;
            fprintf('Fuel remain %6.0f R %6.1f\n',[fuel_remain,R])
            if abs(fuel_remain) < 10
                break
            end
            dFuel = dFuel - fuel_remain*1.05;
        end
    end
    
    if ~badpt
        fprintf('========Converged Result==========\n')
        fprintf('TOFL %2.1f LFL %2.1f\n',[TI.L_to,TI.L_land])
        fprintf('Fuel Remain %2.1f lb, Block Range %2.1f nm, Total range %2.1f nm\n',[fuel_remain,R,TI.R])
        fprintf('Fuel carried : %2.2f burnt: %2.2f\n',[TI.m_maxFuel,TI.BlockFuel])
    end
end