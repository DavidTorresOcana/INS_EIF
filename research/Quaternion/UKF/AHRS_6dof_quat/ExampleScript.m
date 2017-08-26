close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

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

%% UKF initialization
sigma_quat_dot  = 0.0001; %rad
sigma_omega_dot = 0.0001;
Q = [ eye(4)*sigma_quat_dot, zeros(4,3);...
    zeros(3,4) , eye(3)*sigma_omega_dot];

sigma_gyro = 0.05; % rad
sigma_acc = 0.05;% g's

R = [ eye(3)*sigma_gyro, zeros(3);...
    zeros(3) , eye(3)*sigma_acc];

eul(:,1) =[0;0;0];
X(:,1)=[eul2quatern(eul(:,1));0;0;0]; % phi,theta,psi,p,q,r

P(:,:,1)= 0.1*eye(7);
% return
%% UKF Filtering simulation
for i=1:size(Accelerometer,1)-1
    
    %% Pred

    [X_p,P_p] = ukf_predict1(X(:,i),P(:,:,i),'f_pred',Q,time(i+1)-time(i));
    
    %% Update
    G = Adaptive_term(Accelerometer(i,:)');
    
    [X(:,i+1),P(:,:,i+1)] = ukf_update1(X_p,P_p,[Gyroscope(i,:),Accelerometer(i,:)]','h_meas',R,time(i+1)-time(i));
    % Required normalization
    X(1:4,i+1)=X(1:4,i+1)./norm(X(1:4,i+1));
    
    eul(:,i+1)= quatern2eul( X(1:4,i+1) );
    
    
end
figure
subplot(2,1,1)
plot( rad2deg(eul'))
subplot(2,1,2)
plot( rad2deg(X(5:7,:)'))

