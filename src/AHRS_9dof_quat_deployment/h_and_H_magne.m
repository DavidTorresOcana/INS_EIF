function [h,H]=h_and_H_magne(X,AT)
global B_0
B_0=[0.2102;
    0.0360;
   -0.4447];

    qu=X(1:4);

    % h
    M_q = Matrix_4_product(qu); % Hamilton(a,b) == Matrix_4_product(a)*b
    M_B = Matrix_4_product([0;B_0]);
    B_qua = (M_q')*(M_B*qu); % Hamilton(qu*,Hamilton(B_0,qu))

    h = B_qua(2:4);
    % H
    H = zeros(3,7);
    Delta_mag_prod = (M_q')*M_B + d_Hamilton_a_conj_b(qu,M_B*qu);
    H(1:3,1:4)=Delta_mag_prod(2:4,1:4);




end