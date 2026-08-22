% Kalman Filter Process %
close all
clear all

% Test Case Load
load('TC4_Rotate_Assy_Assist')

% Time length Set
[t_l,~] = size(SteerWhlAg_SteerWhlAgRaw_Ag_t0);

% Tuning parameter 
% Q : Process noise matrix
% R : Measurement Matrix

Q_el = [10  10 10 10];
Q = diag(Q_el);
Q = (250)*ones(4);

R_el = [0.5 0.5 0.5];
R = diag(R_el);

P_el = [0 0 0 0]
P0 = 0.001*eye(4);

w_tors = zeros(t_l,1);

% Baseline Setup
% x1 = theta_in
% x2 = w_in
% x3 = theta_out
% x4 = w_out

% Th_in  : Inlet Steering angle
% Th_out : Outlet Steering angle
Th_in= SteerWhlAg_SteerWhlAgRaw_Ag_t0;
Th_out = SteerWhlAg_SteerWhlAgRaw_Ag_t0 - SteerWhlTq_SteerWhlTq1Raw_Tq_t0(1:t_l)*(1480/(13*2.5));

for i = 1:t_l-3
    dt_2 = t0(i+2)-t0(i+1);
end

dt = mean(dt_2);

% calculate Angle speed

for i = 1:t_l-3
    w_tors(i) = (Th_in(i)-Th_out(i))/dt;
end
w_in = SteerWhlAg_SteerWhlAgSpd_SteerWhlAgSpd_t0;
w_out = SteerWhlAg_SteerWhlAgSpd_SteerWhlAgSpd_t0 - w_tors;

%% System Model
J = 1.25; %kg*m^2;
K = 2.5; %Nm;
B = 0;

% System input 
% T_in   : driver input torque
% T_load : system load torque 
T_in =10000;
T_load =-10000;

A = [0 1 0 0; -K/J -B/J K/J B/J; 0 0 0 1;K/J B/J -K/J -B/J];
Bu = (1/J)*[0 T_in 0 -T_load]';

% System Model Parameter

A_d = (eye(4)+dt*A);
B_du = dt*Bu;

% Make Initialzation
[x1_0,x2_0,x3_0,x4_0] = Ag_Init(Th_in(:),w_in(:),Th_out(:),w_out(:));

InitCmplFlg = 0;

% Pre-processing raw data to actual degree data (TAS Sensor)
% make measurement data from Raw Data
[A1_Deg,T1_Deg,Diff_Deg] = MakeBit_to_Deg(SteerWhlAg_A1RawData_Cnt_t0(:),SteerWhlAg_T1RawData_Cnt_t0(:));

% prediction of x vector
x_log = zeros(4,t_l);
P_log = zeros(4,4,t_l);
Kg_log = zeros(4,4,t_l);
inov_log = zeros(3,t_l);

for i = 1:t_l
    % Initialization
    z_k = [A1_Deg(i) T1_Deg(i) Diff_Deg(i)]';
    if InitCmplFlg == 0
        x_k = [x1_0 x2_0 x3_0 x4_0]';
        InitCmplFlg = InitCmplFlg + 1;
        P_k = P0;
    end
        % Prediction
        x_prdt = SysPrdt(x_k,A_d,B_du);
        P_prdt = ErrCovMat(P_k,A_d,Q);
        
        % Kalman gain Calculation
        Kg = KgainCalc(P_k,H_meas(x_k(1),x_k(3)),R);
        
        % Linear H
        x_k = xhatEstimn_Trigo(z_k,x_k,Kg,H_meas(x_k(1),x_k(3)));

        % using h function
        inov_log(:,i) = z_k-h(x_k);

        if abs(inov_log(1,i)) >250
        else
            x_k = xhatEstimn(z_k,x_k,Kg);
        end
        
        % Probability Update
        P_k = PEstimn(H_meas(x_k(1),x_k(3)),Kg,P_k);

        % logging prediction
        x_log(1,i) = x_k(1);
        x_log(2,i) = x_k(2);
        x_log(3,i) = x_k(3);
        x_log(4,i) = x_k(4);
        
end
