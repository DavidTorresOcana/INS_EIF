function  [eul,X,P] = EIF_n_dof(X,P,Q,sigmas,measures,sensors,AT,number_sensors)
global B_0

% Define variables
X_p=zeros(size(X));
P_p=zeros(size(P));
F=zeros(size(P));
Gamma = zeros(size(P));
Y_info_p = zeros(size(P));
y_info_p = zeros(size(X));

eul = zeros(3,1);

% coder.varsize('measures', [3*number_sensors 1]);

%% Pred
    [X_p,F,Gamma] = f_and_F_pred(X,AT);
    P_p = F*P*F' + Gamma*Q*Gamma';
    
    Y_info_p=inv(P_p);
    y_info_p=Y_info_p*X_p;
    %% Update
    % Detect which sensors we got and compute information and Matrix
    % information
    i_info=zeros(size(y_info_p));
    I_info=zeros(size(Y_info_p));
    for j=1:number_sensors
        if isequal(sensors(j,:),'gyro')
            idx=[ 3*(j-1)+1,3*j-1 , 3*j ];
            [h_gyro,H_gyro]=h_and_H_gyro(X_p,AT);
            R_gyro = sigmas(j)*eye(3);
            i_info = i_info + H_gyro'*inv(R_gyro)*(measures(idx)-h_gyro + H_gyro*X_p);
            I_info = I_info + H_gyro'*inv(R_gyro)*H_gyro;
        elseif isequal(sensors(j,:),'acce')
            idx=[ 3*(j-1)+1,3*j-1 , 3*j ];
            [h_accel,H_accel]=h_and_H_accel(X_p,AT);
            R_accel = sigmas(j)*eye(3);
            G_accel = Adaptive_term(measures(3*(j-1)+1:3*j));
            i_info = i_info + H_accel'*inv(G_accel*R_accel*G_accel')*(measures(idx)-h_accel +H_accel*X_p);
            I_info = I_info + H_accel'*inv(G_accel*R_accel*G_accel')*H_accel;
        elseif isequal(sensors(j,:),'magn')
            idx=[ 3*(j-1)+1,3*j-1 , 3*j ];
            [h_magne,H_magne]=h_and_H_magne(X_p,AT);
            R_magne = sigmas(j)*eye(3);
            i_info = i_info + H_magne'*inv(R_magne)*(measures(idx)-h_magne +H_magne*X_p);
            I_info = I_info + H_magne'*inv(R_magne)*H_magne;
        end
    end

    
    y_info= y_info_p + i_info;
    Y_info= Y_info_p + I_info;
    
    % Retrieving the phisical variables
    P=inv(Y_info);
    X=P*y_info;
   

    % Required normalization
    X(1:4)=X(1:4)./norm(X(1:4));
    eul= quatern2eul( X(1:4) );