

function [deltal,deltar]=Ackermann(deltaf,toe,percentuale_Ack,Wf,L)
        
    

        delta_e_A = atan( L * tan( deltaf ) ./ (L + 0.5 * Wf * tan(deltaf)));

        delta_i_A = atan( L * tan( deltaf ) ./ (L - 0.5 * Wf * tan(deltaf)));

        delta_mean = mean ([delta_e_A ; delta_i_A]);
        
        differenza = delta_i_A - delta_mean;

        delta_e = deltaf - percentuale_Ack * differenza + toe;
        delta_i = deltaf + percentuale_Ack * differenza - toe;   
        
        deltal = delta_e;
        deltar = delta_i;
        
%         segno = sign(deltaf);
%         switch segno
%             case 1
%                 deltal = delta_e;
%                 deltar = delta_i;
%             case -1
%                 deltal = delta_i;
%                 deltar = delta_e;
%             case 0
%                 deltal = 0;
%                 deltar = 0;
%         end     
 end        