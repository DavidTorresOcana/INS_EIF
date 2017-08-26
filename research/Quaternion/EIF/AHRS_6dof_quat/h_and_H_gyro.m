function [h,H]=h_and_H_gyro(X,AT)
global B_0

    % h

    h = [X(5:7)];
    % H
    H = zeros(3,7);
    H(1:3,5:7)=eye(3);




end