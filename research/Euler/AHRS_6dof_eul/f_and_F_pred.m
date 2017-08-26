function [X_pred,F,Gamma] = f_and_F_pred(X,AT)
    
    sp=sin(X(1));
    cp=cos(X(1));
    st=sin(X(2));
    ct=cos(X(2));
    tt=tan(X(2));
    p = X(4);
    q = X(5);
    r = X(6);
    
    T_1 = [1,sp*tt,cp*tt;...
        0,cp,-sp;...
        0,sp/ct,cp/ct];
    % f
    X_pred = X;
    X_pred(1:3,1) = X_pred(1:3,1)+ ( T_1*X(4:6,1)).*AT;
    % F
    F = zeros(6);
    F(1,1) = 1+AT*(q*cp*tt-r*sp*tt);
    F(1,2) = AT*(q*sp+r*cp)*(1+tt^2);
    F(1,4)=AT;
    F(1,5)=AT*sp*tt;
    F(1,6)=AT*cp*tt;
    F(2,1)=AT*(-q*sp-r*cp);
    F(2,2)=1;
    F(2,5)=AT*cp;
    F(2,6)=-AT*sp;
    F(3,1) = AT*(q*cp/ct-r*sp/ct);
    F(3,2)=AT*(q*sp*st/(ct^2)+r*cp+st/(ct^2));
    F(3,3)=1;
    F(3,5)=AT*sp/ct;
    F(3,6)=AT*cp/ct;
    F(4:6,4:6)=eye(3);
    % Gamma: It is the noise model
    Gamma = AT*[T_1,AT*T_1;
        zeros(3),eye(3)];
    


end