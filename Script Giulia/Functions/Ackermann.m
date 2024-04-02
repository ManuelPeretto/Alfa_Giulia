

function [deltal,deltar]=Ackermann(deltaf,Vehicle)
        
        toe= deg2rad(Vehicle.toe_f);
        percentuale_Ack = Vehicle.percentuale_Ack;
        Wf = Vehicle.Wf;
        L = Vehicle.L;
    

        delta_e_A = atan( L * tan( deltaf ) ./ (L + 0.5 * Wf * tan(deltaf)));

        delta_i_A = atan( L * tan( deltaf ) ./ (L - 0.5 * Wf * tan(deltaf)));

        delta_mean = mean ([delta_e_A ; delta_i_A]);
        
        differenza = delta_i_A - delta_mean;

        delta_e = deltaf - percentuale_Ack * differenza + toe;
        delta_i = deltaf + percentuale_Ack * differenza - toe;   
        
        deltal = delta_e;
        deltar = delta_i;
            
 end        