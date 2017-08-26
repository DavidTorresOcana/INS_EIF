function [X_0,P_0,Q,sigmas]=EIF_init(eul_0,sensors,number_sensors)

%% EIF initialization
sigma_quat_dot  = 1; %rad
sigma_omega_dot = 0.3;
Q = [ eye(3)*sigma_quat_dot, zeros(3,3);...
    zeros(3,3) , eye(3)*sigma_omega_dot];

sigma_gyro = 0.01; % rad
sigma_acc = 0.01;% g's
sigma_mag = 0.1;%

sigmas = zeros(size(sensors,1),1);

for i=1:number_sensors
    if isequal(sensors(i,:),'gyro')
        sigmas(i) = sigma_gyro;
    elseif isequal(sensors(i,:),'acce')
        sigmas(i) = sigma_acc;
    elseif isequal(sensors(i,:),'magn')
        sigmas(i) = sigma_mag;
    end
end


% R = blkdiag(R_gyro,R_accel,R_magne) ;
X_0=[eul2quatern(eul_0);0;0;0]; 

P_0= 0.01*eye(7);

% Y_info=inv(P_0);
% y_info=Y_info*X_0;

end