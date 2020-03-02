classdef aircrafts
    %All aircrafts data are stored in this class
    
    properties

    end
    
    methods(Static)
        function SetupSim_777()

            % ===================Aircraft====================
            m0 = 545000; Vr = 199*1.6878; m = m0; mu_roll = 0.04; mu_slide = 0.45;
            V_cruise = 813; h_cruise = 35000;
            R_cruise_end = 3340; m_maxFuel = 207700;
            V_init_approach = 230*1.6878; h_approach = 5000;
            Cl_max_land = 2.49;
            %     global S ThrustSL ThrustMinSL SFC_Cruise Cd0 K
            S = 4605; ThrustSL = 77000; ThrustMinSL = 2700; SFC_Cruise = 0.52;
            Cd0 = 0.0146; K = 0.044152; %0.053
            save('temp/setup.mat')

        end
        
        function SetupSim_AIAA_folding()
            
            % ===================Aircraft====================
            m0 = 482345; Vr = 199*1.6878; m = m0; mu_roll = 0.04; mu_slide = 0.45;
            V_cruise = 775; h_cruise = 35000;
            R_cruise_end = 3350; m_maxFuel = 116926;
            V_init_approach = 230*1.6878; h_approach = 5000;
            Cl_max_land = 2.49;
            global S ThrustSL ThrustMinSL SFC_Cruise Cd0 K
            S = 3660; ThrustSL = 75000; ThrustMinSL = 2500; SFC_Cruise = 0.508;
            Cd0 = 0.01713; K = 0.04421; %0.053
            save('temp/setup.mat')

        end
        
        function SetupSim_AIAA_nonfolding()
            
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

