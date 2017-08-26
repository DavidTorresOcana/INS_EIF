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

%% Inputs 
% eul(:,1), name of sensors and measurements
eul(:,1) =[0;0;0];
number_sensors = 2;
sensors =['gyro';'acce'];
[X(:,1),P(:,:,1),Q,sigmas]=EIF_init(eul(:,1),sensors,number_sensors);

% return

%% ====================== Extended INFORMATION FILTER ===================================
for i=1:size(Accelerometer,1)-1
    % Emulate we put all measures toghether
    measures=zeros(size(sensors,2),1);
    for j=1:number_sensors
        switch lower(sensors(j,:))
            case 'gyro'
                measures(3*(j-1)+1:3*j)=Gyroscope(i,:)';
            case 'acce'
                measures(3*(j-1)+1:3*j)=Accelerometer(i,:)';
            case 'magn'
                measures(3*(j-1)+1:3*j)=Magnetometer(i,:)';
        end
    end

    % Filter MATLAB
%     tic;
%     [eul(:,i+1),X(:,i+1),P(:,:,i+1)] =...
%         EIF_n_dof(X(:,i),P(:,:,i),Q,sigmas,measures,sensors,time(i+1)-time(i),number_sensors);
%     delta(i)=toc;
    % Filter in MEX
    tic;
    [eul(:,i+1),X(:,i+1),P(:,:,i+1)] =...
        EIF_n_dof_mex(X(:,i),P(:,:,i),Q,sigmas,measures,sensors,time(i+1)-time(i),int32(number_sensors));
    delta(i)=toc;
    
end

%% Pllotting results
fprintf(' Mean frequency %f ', 1/mean(delta) );

figure
h(1)=subplot(2,1,1);
plot( time, rad2deg(eul'))
ylabel('Attitude (deg)')
xlabel('Time (s)')
legend('\phi','\theta','\psi')
h(2)=subplot(2,1,2);
plot( time,rad2deg(X(5:7,:)'))
ylabel('body rates (deg/s)')
xlabel('Time (s)')
legend('p','q','r')

linkaxes(h,'x');
set(h(1),'XLim',[0 time(end)])



