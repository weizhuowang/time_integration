classdef aircrafts
    %All aircrafts data are stored in this class
    
    properties

    end
    
    methods(Static)
        function SetupSim_777()

            % ===================Aircraft====================
            m0 = 545000; Vr = 199*1.6878; m = m0; mu_roll = 0.04; mu_slide = 0.45;
            V_cruise = 813; h_cruise = 45000;
            R_cruise_end = 3340; m_maxFuel = 207700;
            V_init_approach = 230*1.6878; h_approach = 5000;
            Cl_max_land = 2.49;
            %     global S ThrustSL ThrustMinSL SFC_Cruise Cd0 K
            S = 4605; ThrustSL = 77000; ThrustMinSL = 2700; SFC_Cruise = 0.52;
            Cd0 = 0.0146; K = 0.044152; %0.053
            save('temp/setup.mat')

        end
        
        % FDR
        function fname = SetupSim_AIAA_nonfolding(dFuel,dpay,dMTOW,R,M,h_cruise)
            
            % ===================Aircraft====================
            WPayload = 94300 + dpay;
            WEmpty = 230520;
            MFuel = 113230;
            MTOW = 438050+dMTOW;
            m_maxFuel = min(MFuel,MTOW-(WEmpty+WPayload))+dFuel;
            m0 = WEmpty+WPayload + m_maxFuel; 
            
%             global S ThrustSL ThrustMinSL SFC_Cruise Cd0 K
            S = 3565; ThrustSL = 66500; ThrustMinSL = 0.035*ThrustSL; SFC_Cruise = 0.43;
            Cdp =  0.018061;%0.01713; % Cd0 + Cdexcr
            K =  0.049137;%0.04421; %0.053
            
            m = m0; mu_roll = 0.04; mu_slide = 0.45;
            V_cruise = 969*M; h_cruise = h_cruise + 0*35000;
            R_cruise_end = R; 
            V_init_approach = 230*1.6878; h_approach = 5000;
            Cl_max_land = 2.49;Clmax_TO = 1.89;
            fname = ['temp/setup',datestr(now,'HHMMSSFFF'),num2str(randi(100)),'.mat'];
            save(fname)

        end
        
                % FDR
        function fname = SetupSim_AIAA_folding300(dFuel,dpay,dMTOW,R,M,h_cruise)
            
            % ===================Aircraft====================
            WPayload = 94300 + dpay;
            WEmpty = 238295;
            MFuel = 109995;
            MTOW = 442590+dMTOW;
            m_maxFuel = min(MFuel,MTOW-(WEmpty+WPayload))+dFuel;
            m0 = WEmpty+WPayload + m_maxFuel; 
            
            m = m0; mu_roll = 0.04; mu_slide = 0.45;
            V_cruise = 969*M; h_cruise = h_cruise + 0*35000;
            R_cruise_end = R; 
            V_init_approach = 230*1.6878; h_approach = 5000;
            Cl_max_land = 2.49;Clmax_TO = 1.89;
%             global S ThrustSL ThrustMinSL SFC_Cruise Cd0 K
            S = 3880; ThrustSL = 62500; ThrustMinSL = 0.035*ThrustSL; SFC_Cruise = 0.43;
            Cdp = 0.0171794274781596;%0.01713; % Cd0 + Cdexcr
            K = 0.0463130927082483;%0.04421; %0.053
            fname = ['temp/setup',datestr(now,'HHMMSSFFF'),num2str(randi(100)),'.mat'];
            save(fname)

        end
        
        % FDR
        function fname = SetupSim_AIAA_folding(dFuel,dpay,dMTOW,R,M,h_cruise)
            
            % ===================Aircraft====================
            WPayload = 94300 + dpay;
            WEmpty = 238548;
            MFuel = 110065;
            MTOW = 442913+dMTOW;
            m_maxFuel = min(MFuel,MTOW-(WEmpty+WPayload))+dFuel;
            m0 = WEmpty+WPayload + m_maxFuel; 
            
            m = m0; mu_roll = 0.04; mu_slide = 0.45;
            V_cruise = 969*M; h_cruise = h_cruise + 0*35000;
            R_cruise_end = R; 
            V_init_approach = 230*1.6878; h_approach = 5000;
            Cl_max_land = 2.49;Clmax_TO = 1.89;
%             global S ThrustSL ThrustMinSL SFC_Cruise Cd0 K
            S = 3880; ThrustSL = 62500; ThrustMinSL = 0.035*ThrustSL; SFC_Cruise = 0.43;
            Cdp = 0.0171525;%0.01713; % Cd0 + Cdexcr
            K = 0.0462395;%0.04421; %0.053
            fname = ['temp/setup',datestr(now,'HHMMSSFFF'),num2str(randi(100)),'.mat'];
            save(fname)

        end
        
        % PDR
        function fname = SetupSim_AIAA_folding_PDR(dFuel,dpay,dMTOW,R,M,h_cruise)
            
            % ===================Aircraft====================
            WPayload = 94300 + dpay;
            WEmpty = 249715;
            MFuel = 111897;
            MTOW = 455912+dMTOW;
            m_maxFuel = min(MFuel,MTOW-(WEmpty+WPayload))+dFuel;
            m0 = WEmpty+WPayload + m_maxFuel; 
            
            m = m0; mu_roll = 0.04; mu_slide = 0.45;
            V_cruise = 969*M; h_cruise = h_cruise + 0*35000;
            R_cruise_end = R; 
            V_init_approach = 230*1.6878; h_approach = 5000;
            Cl_max_land = 2.49;Clmax_TO = 1.89;
%             global S ThrustSL ThrustMinSL SFC_Cruise Cd0 K
            S = 3660; ThrustSL = 67110; ThrustMinSL = 0.035*ThrustSL; SFC_Cruise = 0.43;
            Cdp = 0.0176;%0.01713; % Cd0 + Cdexcr
            K = 0.0452;%0.04421; %0.053
            fname = ['temp/setup',datestr(now,'HHMMSSFFF'),num2str(randi(100)),'.mat'];
            save(fname)

        end
        
        function SetupSim_AIAA_nonfolding1()
            
            % ===================Aircraft====================
            m0 = 470026; Vr = 199*1.6878; m = m0; mu_roll = 0.04; mu_slide = 0.45;
            V_cruise = 775; h_cruise = 35000;
            R_cruise_end = 3410; m_maxFuel = 121125;
            V_init_approach = 230*1.6878; h_approach = 5000;
            Cl_max_land = 2.49;
            global S ThrustSL ThrustMinSL SFC_Cruise Cd0 K
            S = 3900; ThrustSL = 67400; ThrustMinSL = 2250; SFC_Cruise = 0.508;
            Cd0 = 0.016459; K = 0.053052; %0.053
            save('temp/setup.mat')

        end
    end
end

