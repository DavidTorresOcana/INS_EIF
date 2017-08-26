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

%% EKF initialization
sigma_omega = 1; %rad
sigma_omega_dot =0.5;
Q = [ eye(3)*sigma_omega, zeros(3);...
    zeros(3) , eye(3)*sigma_omega_dot];

sigma_gyro = 0.1; % rad
sigma_acc = 0.1;% g's

R = [ eye(3)*sigma_gyro, zeros(3);...
    zeros(3) , eye(3)*sigma_acc];

X(:,1)=[0;0;0;0;0;0]; % phi,theta,psi,p,q,r
P(:,:,1)= 0.1*eye(6);

%% EKF Filtering simulation
for i=1:size(Accelerometer,1)-1
    
    %% Pred
    [X_p,F,Gamma] = f_and_F_pred(X(:,i),time(i+1)-time(i));
    P_p = F*P(:,:,i)*F' + Gamma*Q*Gamma';
    
    %% Update
    [h,H]=h_and_H(X_p,time(i+1)-time(i));
    % Check the gradient
% % %     A = zeros(size(H));
% % %     epsilon=0.01;
% % %     for j=1:6
% % %         xx =X_p;
% % %         xx(j) = xx(j) -epsilon;
% % %         A_left = h_and_h(xx,time(i+1)-time(i));
% % %         
% % %         xx =X_p;
% % %         xx(j) = xx(j) +epsilon;
% % %         A_right =h_and_h(xx,time(i+1)-time(i));
% % %         
% % %         A(:,j)= (A_right-A_left)/(2*epsilon);
% % %         
% % %     end
% % %     H./A
    G = Adaptive_term(Accelerometer(i,:)');
    
    K = P_p*H'*(H*P_p*H' + G*R*G')^(-1);
    inno=([Gyroscope(i,:),Accelerometer(i,:)]' - h);
    X(:,i+1)= X_p +K*inno;
    P(:,:,i+1) = (eye(6) -K*H)*P_p;
    
    
end
figure
subplot(2,1,1)
plot( rad2deg(X(1:2,:)'))
subplot(2,1,2)
plot( rad2deg(X(4:6,:)'))

