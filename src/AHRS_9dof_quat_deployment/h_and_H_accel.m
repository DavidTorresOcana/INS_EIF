function [h,H]=h_and_H_accel(X,AT)
global B_0
    qu=X(1:4);

    g=1;
    g_v =[0;0;g];
    
    % h
    M_q = Matrix_4_product(qu); % Hamilton(a,b) == Matrix_4_product(a)*b
    M_g = Matrix_4_product([0;g_v]);

    g_qua = (M_q')*(M_g*qu); % Hamilton(qu*,Hamilton(gv,qu))

    h = g_qua(2:4);
    % H
    H = zeros(3,7);
    Delta_acc_prod = (M_q')*M_g + d_Hamilton_a_conj_b(qu,M_g*qu);
    H(1:3,1:4)=Delta_acc_prod(2:4,1:4);





end