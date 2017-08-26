close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal
global B_0
%% Import and plot sensor data

load('ExampleData.mat');
Gyroscope =deg2rad(Gyroscope);

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (rad/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
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
%% EKF initialization
sigma_quat_dot  = 0.9; %rad
sigma_omega_dot = 0.2;
Q = [ eye(3)*sigma_quat_dot, zeros(3,3);...
    zeros(3,3) , eye(3)*sigma_omega_dot];

sigma_gyro = 0.01; % rad
sigma_acc = 0.01;% g's
sigma_mag = 2;%

R = blkdiag(eye(3)*sigma_gyro,eye(3)*sigma_acc,eye(3)*sigma_mag) ;
eul(:,1) =[0;0;0];
X(:,1)=[eul2quatern(eul(:,1));0;0;0]; % phi,theta,psi,p,q,r

P(:,:,1)= 0.01*eye(7);
% return

%% EKF Filtering simulation
for i=1:size(Accelerometer,1)-1
    
    %% Pred
    [X_p,F,Gamma] = f_and_F_pred(X(:,i),time(i+1)-time(i));
    P_p = F*P(:,:,i)*F' + Gamma*Q*Gamma';
    
% %     % Check the gradient
% %     A = zeros(size(F));
% %     epsilon=0.01;
% %     for j=1:7
% %         xx =X(:,i);
% %         xx(j) = xx(j) -epsilon;
% %         A_left = f_and_F_pred(xx,time(i+1)-time(i));
% %         
% %         xx =X(:,i);
% %         xx(j) = xx(j) +epsilon;
% %         A_right =f_and_F_pred(xx,time(i+1)-time(i));
% %         
% %         A(:,j)= (A_right-A_left)/(2*epsilon);
% %         
% %     end
% %     F./A

    %% Update
    [h,H]=h_and_H(X_p,time(i+1)-time(i));
    % Check the gradient
% %     A = zeros(size(H));
% %     epsilon=0.01;
% %     for j=1:7
% %         xx =X_p;
% %         xx(j) = xx(j) -epsilon;
% %         A_left = h_and_H(xx,time(i+1)-time(i));
% %         
% %         xx =X_p;
% %         xx(j) = xx(j) +epsilon;
% %         A_right =h_and_H(xx,time(i+1)-time(i));
% %         
% %         A(:,j)= (A_right-A_left)/(2*epsilon);
% %         
% %     end
% %     H./A
    
    G = Adaptive_term(Accelerometer(i,:)');
    
    K = P_p*H'*(H*P_p*H' + G*R*G')^(-1);
    inno=([Gyroscope(i,:),Accelerometer(i,:),Magnetometer(i,:)]' - h);
    X(:,i+1)= X_p +K*inno;
    P(:,:,i+1) = (eye(7) -K*H)*P_p;
    
    % Required normalization
    X(1:4,i+1)=X(1:4,i+1)./norm(X(1:4,i+1));
    
    eul(:,i+1)= quatern2eul( X(1:4,i+1) );

    
end




figure
subplot(2,1,1)
plot( rad2deg(eul'))
subplot(2,1,2)
plot( rad2deg(X(5:7,:)'))


