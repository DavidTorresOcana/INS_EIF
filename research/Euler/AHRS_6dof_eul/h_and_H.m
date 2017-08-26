function [h,H]=h_and_H(X,AT)

    sp=sin(X(1));
    cp=cos(X(1));
    st=sin(X(2));
    ct=cos(X(2));
    tt=tan(X(2));
    sps=sin(X(3));
    cps=cos(X(3));
    
    p = X(4);
    q = X(5);
    r = X(6);
    
    
    g=1;
    g_v =[0;0;g];
    
    T_2 = [ct*cps,ct*sps,-st;
            sp*ct*cps-cp*sps,sp*st*sps+cp*cps,sp*ct;
            cp*st*cps+sp*sps,cp*st*sps-sp*cps,cp*ct]; % T_f_h->F_b

    h = [X(4:6);
        T_2*g_v];
    
    H = zeros(6);
    H(1:3,4:6)=eye(3);

   H(4:6,1:3) = [ 0 ,[-st*cps,-st*sps,-ct]*g_v,[-ct*sps,ct*cps,0]*g_v;
                [cp*ct*cps+sp*sps,cp*st*sps-sp*cps,cp*ct]*g_v,[-sp*st*cps,sp*ct*sps,-sp*st]*g_v,[-sp*ct*sps-cp*cps,sp*st*cps-cp*sps,0]*g_v;
                [-sp*st*cps+cp*sps,-sp*st*sps-cp*cps,-sp*ct]*g_v,[cp*ct*cps,cp*ct*sps,-cp*st]*g_v,[-cp*st*sps+sp*cps,cp*st*cps+sp*sps,0]*g_v];
            
%     H(4:6,1:3) = g*[ 0 ,-ct,0;
%                     cp*ct,-sp*st,0;
%                     -sp*ct,-cp*st,0];





end