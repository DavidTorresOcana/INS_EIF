function [X_pred] = f_pred(X,AT)
    
   
    p = X(5);
    q = X(6);
    r = X(7);
    
    
    X_pred = X;
    X_pred(1:4,1) = X_pred(1:4,1)+ (AT/2)*( HamiltonProduct( X(1:4),[0;X(5:7)] ) );
   
    


end