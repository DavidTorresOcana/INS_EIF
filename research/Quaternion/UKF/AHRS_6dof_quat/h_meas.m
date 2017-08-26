function [h]=h_meas(X,AT)

   
    
%     p = X(5);
%     q = X(6);
%     r = X(7);
%     
    
    g=1;
    g_v =[0;0;g];
    
   
    Prod= HamiltonProduct( [X(1);-X(2:4)]   , HamiltonProduct([0;g_v], X(1:4)) );
    h = [X(5:7);
        Prod(2:4)  +  0]; % Add anything to the model accel???




end