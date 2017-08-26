function [X_pred,F,Gamma] = f_and_F_pred(X,AT)
    
    qu=X(1:4);
    p = X(5);
    q = X(6);
    r = X(7);
    % f
    M_q = Matrix_4_product(qu); % Hamilton(a,b) == Matrix_4_product(a)*b
    X_pred = X + AT/2.*[M_q*[0;p;q;r];0;0;0];
    
    % F
    Delta_Prod_M_q = d_Hamilton_a_b(qu,[0;p;q;r]);  % Derivative of hamilton(a,b) in terms of a OR Derivative of Matrix_4_product(a)*b in terms of a
    
    F=eye(7,7);
    F(1:4,1:4) = eye(4,4) + AT/2.*Delta_Prod_M_q;
    F(1:4,5:7) = AT/2.*M_q(:,2:4);
    
    % Gamma: It is the noise model
    Gamma = [AT/2.*M_q(:,2:4), AT^2/2.*M_q(:,2:4);
        zeros(3) ,AT*eye(3) ];
    


end