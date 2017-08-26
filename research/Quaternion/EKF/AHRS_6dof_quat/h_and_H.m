function [h,H]=h_and_H(X,AT)
    qu=X(1:4);
%     p = X(5);
%     q = X(6);
%     r = X(7);

    g=1;
    g_v =[0;0;g];
    
    % h
    M_q = Matrix_4_product(qu); % Hamilton(a,b) == Matrix_4_product(a)*b
    M_g = Matrix_4_product([0;g_v]);

    g_qua = (M_q')*(M_g*qu); % Hamilton(qu*,Hamilton(gv,qu))
%     g_qua = HamiltonProduct([qu(1);-qu(2:4)],HamiltonProduct([0;g_v],qu)); % Hamilton(qu*,Hamilton(gv,qu))

    h = [X(5:7);
        g_qua(2:4)];
    % H
    H = zeros(6,7);
    H(1:3,5:7)=eye(3);
    Delta_acc_prod = (M_q')*M_g + d_Hamilton_a_conj_b(qu,M_g*qu);
    H(4:6,1:4)=Delta_acc_prod(2:4,1:4);




end