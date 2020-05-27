function [Range,badpt] = SolveRange(verbose,SetupSim,dFuel,dPay,dMTOW,M,alt)
    
    Range = 300;
    for i = 1:12
        if verbose fprintf('\n=============Iteration %2.0f ===========\n',[i]);end
        [fname,badpt] = time_integration(SetupSim,false,verbose,{dFuel,dPay,dMTOW,Range,M,alt}); % 0.792 37500
        if badpt
            fprintf('!!!!!!!!!!!!!!badpoint, decrease the starting range\n');
            break
        else
            TI = load(fname,'m_maxFuel','Fuel_burnt','BlockFuel','L_to','L_land',...
                'R','m_maxFuel');
            fuel_remain = TI.m_maxFuel - TI.Fuel_burnt - 0.05*TI.BlockFuel;

            fprintf('Fuel remain %6.0f R %6.1f\n',[fuel_remain,Range])
            if abs(fuel_remain) < 1
                break
            end

            Range = Range + fuel_remain/26;
        end
    end

    if ~badpt
        fprintf('========Converged Result==========\n')
        fprintf('TOFL %2.1f LFL %2.1f\n',[TI.L_to,TI.L_land])
        fprintf('Fuel Remain %2.1f lb, Block Range %2.1f nm, Total range %2.1f nm\n',[fuel_remain,Range,TI.R])
        fprintf('Fuel carried : %2.2f burnt: %2.2f\n',[TI.m_maxFuel,TI.BlockFuel])
    end

end