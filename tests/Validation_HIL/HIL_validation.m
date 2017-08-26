close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

addpath(genpath('HorizonArtificiel'))
SimulateurInitialisation;


% if isvalid(arduino)

%      fclose(arduino)
%     clear arduino
%     delete(instrfindall);
%     
% end
%% Get sensor data: Arduino UART
   
    
    % Initialize serial port
    arduino = serial('COM10','BaudRate',9600,'Parity','none');
    set(arduino, 'Terminator', 'LF'); % Default terminator is \n
    set(arduino,'DataBits', 8);
    set(arduino,'StopBits', 1);
%     set(arduino, 'InputBufferSize',10);
%     set(arduino, 'FlowControl', 'none');
%     set(arduino, 'Timeout',4);
    

    fopen(arduino);
    arduino.ReadAsyncMode = 'continuous';
    
    data = [0;0];
    i = 1;
    
    % Start asynchronous reading
    readasync(arduino);
    fscanf(arduino, '%d');
    while(i<2000)
        data(:,i)=[fscanf(arduino, '%d');fscanf(arduino, '%d')];
%         fprintf(' %i   %i \n',fscanf(arduino, '%d'),fscanf(arduino, '%d'))
        DATA(2,i)=data(2,i)*180/672-90;
        DATA(1,i)=data(1,i)*180/672-90;% In degs
        eul(:,i)=[deg2rad(  data(2,i)*180/672-90  );deg2rad(  data(1,i)*180/672-90  ) ];
%         
        Display=functionDisplay(eul(2,i),eul(1,i),2000,120);
        drawnow
        %Increment the counter
        i=i+1;
        
        
    end
    
%     plot(1:i-1,DATA(2,:)./45')
%    std(DATA(1,:))
%    std(DATA(2,:)./45')
    fclose(arduino)
    clear arduino
    delete(instrfindall);
%     

 
%%  Display
%     Display=functionDisplay(eul(2,i+1),eul(1,i+1),2000,120);
%     drawnow
    
