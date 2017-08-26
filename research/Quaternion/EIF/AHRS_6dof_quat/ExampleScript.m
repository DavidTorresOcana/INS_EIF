close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal
global B_0
%% Import and plot sensor data

load('ExampleData.mat');
Gyroscope =deg2rad(Gyroscope);

figure('Name', 'Sensor Data');
axis(1) = subplot(2,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (rad/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(2,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;


linkaxes(axis, 'x');
% return


%% Mean freq
diff(1)=0;
for i=2:size(time,1)
    diff(i)=time(i)-time(i-1);
end
T_sys = mean(diff);
%% North mag vector
B_0 = mean(Magnetometer(1:200,:),1)';
%% EIF initialization
sigma_quat_dot  = 0.9; %rad
sigma_omega_dot = 0.2;
Q = [ eye(3)*sigma_quat_dot, zeros(3,3);...
    zeros(3,3) , eye(3)*sigma_omega_dot];

sigma_gyro = 0.01; % rad
sigma_acc = 0.01;% g's
R_gyro =eye(3)*sigma_gyro;
R_accel=eye(3)*sigma_acc;

R = blkdiag(R_gyro,R_accel) ;
eul(:,1) =[0;0;0];
X(:,1)=[eul2quatern(eul(:,1));0;0;0]; % phi,theta,psi,p,q,r

P(:,:,1)= 0.01*eye(7);

Y_info(:,:,1)=inv(P(:,:,1));
y_info(:,1)=Y_info(:,:,1)*X(:,1);

% return

%% ====================== Extended INFORMATION FILTER ===================================
for i=1:size(Accelerometer,1)-1
    
    %% Pred
    [X_p,F,Gamma] = f_and_F_pred(X(:,i),time(i+1)-time(i));
    P_p = F*P(:,:,i)*F' + Gamma*Q*Gamma';
    
    Y_info_p=inv(P_p);
    y_info_p=Y_info_p*X_p;
    %% Update
    [h_gyro,H_gyro]=h_and_H_gyro(X_p,time(i+1)-time(i));
    [h_accel,H_accel]=h_and_H_accel(X_p,time(i+1)-time(i));
    [h_magne,H_magne]=h_and_H_magne(X_p,time(i+1)-time(i));
    
    G_accel = Adaptive_term(Accelerometer(i,:)');
        
    i_info = H_gyro'*inv(R_gyro)*(Gyroscope(i,:)'-h_gyro + H_gyro*X_p) +...
                H_accel'*inv(G_accel*R_accel*G_accel')*(Accelerometer(i,:)'-h_accel +H_accel*X_p);
    
    I_info = H_gyro'*inv(R_gyro)*H_gyro + H_accel'*inv(G_accel*R_accel*G_accel')*H_accel ;
    y_info(:,i+1)= y_info_p + i_info;
    Y_info(:,:,i+1)= Y_info_p + I_info;
    
    % Retrieving the phisical variables
    P(:,:,i+1)=inv(Y_info(:,:,i+1));
    X(:,i+1)=P(:,:,i+1)*y_info(:,i+1);
   

    % Required normalization
    X(1:4,i+1)=X(1:4,i+1)./norm(X(1:4,i+1));
    eul(:,i+1)= quatern2eul( X(1:4,i+1) );

    
end




figure
subplot(2,1,1)
plot( rad2deg(eul'))
subplot(2,1,2)
plot( rad2deg(X(5:7,:)'))


