function [J1, J2] = calculate_objectives(sim_results)
    % CALCULATE_OBJECTIVES - Returns Cost Values for MOO
    % J1: Comfort (RMS Acceleration)
    % J2: Handling (RMS Suspension Deflection + Tire Deflection penalty)
    
    % Calculate RMS (Root Mean Square)
    rms_acc = rms(sim_results.sprung_acc);
    rms_def = rms(sim_results.sus_def);
    
    % J1: Comfort Objective
    J1 = rms_acc;
    
    % J2: Handling Objective (Deflection)
    J2 = rms_def; 
    
    % Optional: You can add penalties here (e.g., if force > limit)
end