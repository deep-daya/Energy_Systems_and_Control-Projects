%% CE 295 - Energy Systems and Control
%   HW 2 : State Estimation in Oil Well Drilling
%   Deep Dayaramani, SID 3032083601
%   Prof. Moura

clear; close all;
fs = 15;    % Font Size for plots

%% Drill String Parameters

p.J_T =100 ;  % Table/top rotational inertia
p.J_B = 25 ;   % Bottom/bit rotational inertia
p.k =2 ;      % Spring constant
p.b = 5;      % Drag coefficient

%% Problem 2 - Observability Analysis

% State space matrices
A4 = [0 , 0 , 1 , 0;0 , 0 , 0 , 1 ;-p.k/p.J_T , p.k/p.J_T , -p.b/p.J_T , 0 ; p.k/p.J_B , -p.k/p.J_B , 0 , -p.b/p.J_B ];
B4 = [0,0;0,0; 1/p.J_T,0;0,-1/p.J_B];
C4 = [0,0,1,0];

% Compute observability Matrix for 4-state system and rank
O4 = [C4; C4*A4; C4*A4*A4; C4*A4*A4*A4];
disp('Rank of Observability Matrix for four-state system')
rank(O4)

% New A Matrix, for 3-state system
A =[0, 1,-1;-p.k/p.J_T,-p.b/p.J_T, 0 ; p.k/p.J_B, 0, -p.b/p.J_B,];
B =[0,0;1/p.J_T,0;0,-1/p.J_B];
C =[0,1,0];

% Observability Matrix for 3-state system and rank
O =[C; C*A; C*A*A];
disp('Rank of Observability Matrix for three-state system')
rank(O)

%% Problem 3 - Measurement Data
data = csvread('HW2_Data.csv');
t = data(:,1);      % t   : time vector [sec]
y_m = data(:,2);    % y_m : measured table velocity [radians/sec]
T = data(:,3);      % T   : table torque [N-m]
omega_B_true = data(:,4);    % \omega_B : true rotational speed of bit [radians/sec]

figure(1); clf;

subplot(2,1,1);
plot(t,T,'k');
legend('Table Torque T(t) vs time');
xlabel('time (s)');
ylabel('Table Torque T(t)- [N-m]');
% Plot table torque

subplot(2,1,2);
plot(t,y_m,'b');
legend('Measured Table Velocity vs time');
xlabel('time (s)');
ylabel('Measured Table Velocity [radians/sec]');
% Plot measured table velocity

%% Problem 4 - Luenberger Observer

% Eigenvalues of open-loop system
disp('Eigenvalues of 3-state system:')
eig(A)

% Desired poles of estimation error system
%   They should have negative real parts
%   Complex conjugate pairs
lam = [  -0.2502 + 0.8958i,  -0.2502 - 0.8958i,  -0.2497 + 0.0000i];

% Compute observer gain (See Remark 3.1 in Notes. Use "place" command)
L = place(A', C', lam)';

% State-space Matrices for Luenberger Observer
A_lobs = A-L*C ;
B_lobs =  [B(:,1), L];
C_lobs =  C;
sys_lobs = ss( A_lobs ,B_lobs  , C_lobs  ,0);

% Inputs to observer
u = [T ,y_m];

% Initial Conditions for Luenberger Observer
x_hat0 = [0 ;0 ;0 ];

% Simulate Response
[y,t,x_hat] = lsim(sys_lobs,u,t,x_hat0);

% Parse out states
theta_hat = x_hat(:,1);
omega_T_hat = x_hat(:,2);
omega_B_hat = x_hat(:,3);

% Plot Results
figure(2); clf;

subplot(2,1,1);
plot(t, omega_B_hat,'k',t, omega_B_true, 'b');
xlabel('time (s)');
ylabel('True Rotational Speed of Bit (radians/sec) ')
legend('Estimated Rotational Speed of Bit', 'True Rotational Speed of Bit');
% Plot true and estimated bit velocity
sum = 0;
for i = 1:length(omega_B_hat)
    sum = sum + (omega_B_true(i) - omega_B_hat(i))^2;
end
rmse = sqrt((1/length(t))*(sum));
subplot(2,1,2);
plot(t,omega_B_true- omega_B_hat,'b');
legend('Error between true and estimated bit velocity')
xlabel('time (s)')
ylabel({'$$\omega_{B}$$ - $$\hat{\omega_{B}}$$'}, 'Interpreter', 'Latex')
% Plot error between true and estimated bit velocity


%% Problem 5 - Kalman Filter
% Noise Covariances
W = [0,0,0;0,0,0;0,0,0.0805] ;   % You design this one.
N = 0.02 ;
Sig0 = eye(3);

% Input data
input_data = [t, T, y_m];

% Initial Condition
x_hat0 = [0 ;0  ;0  ];
states0 = [x_hat0; reshape(Sig0,[9 1])];

% Simulate Kalman Filter
%   This coding is a little complicated for novice Matlab users.
%   Try to reverse engineer what I've done here.
[tsim,states] = ode45(@(t,x) ode_kf(t,x,A,B(:,1),C,input_data,W,N),t,states0);

% Parse States
theta_hat = states(:,1);
omega_T_hat = states(:,2);
omega_B_hat = states(:,3);
Sig33 = states(:,end) ;   % Parse out the (3,3) element of Sigma only!
Sigma = reshape(states(end,4:end),[3 3]);
L_end = Sigma*C'*inv(N);
disp(eig(A-L_end*C));
% Compute the upper and lower bounds as described in Problem 5c.
omega_B_hat_upperbound = omega_B_hat + sqrt(Sig33);
omega_B_hat_lowerbound = omega_B_hat - sqrt(Sig33);

% Plot Results
figure(3); clf;

subplot(2,1,1);
plot(tsim, omega_B_true, 'b', tsim, omega_B_hat, 'g',...
    tsim, omega_B_hat - sqrt(Sig33), 'k-.', tsim, omega_B_hat + sqrt(Sig33), 'k-.');

xlabel('time (s)');
ylabel('Rotational Speed of Bit (radians/sec) ')
legend('Estimated Rotational Speed of Bit', 'True Rotational Speed of Bit', 'One standard deviation upper bound', 'One Standard Deviation Lower Bound');

%   Plot true and estimated bit velocity
%   Plot estimated bit velocity plus/minus one sigma

subplot(2,1,2);
plot(tsim, omega_B_true - omega_B_hat);
%   Plot error between true and estimated bit velocity
legend('Error between true and estimated bit velocity')
xlabel('time (s)')
ylabel({'$$\omega_{B}$$ - $$\hat{\omega_{B}}$$'}, 'Interpreter', 'Latex')
        

omega_B_tilde = omega_B_true - omega_B_hat;
RMSE = sqrt(mean(omega_B_tilde.^2));
fprintf(1,'Kalman Filter RMSE: %1.4f rad/s\n',RMSE);


%% Problem 6 - Extended Kalman Filter
% New parameters
p.k1 = 2;
p.k2 =5 ;
W = [0,0,0;0,0,0;0,0,0.0805] ;   % You design this one.
N = 0.02 ;
Sig0 = eye(3);

% Input data
input_data = [t, T, y_m];

% Initial Condition
x_hat0 = [0 ;0  ;0  ];
states0 = [x_hat0; reshape(Sig0,[9 1])];


% Simulate Kalman Filter
%   This coding is a little complicated for novice Matlab users.
%   Try to reverse engineer what I've done here.
[tsim,states] = ode45(@(t,x) ode_ekf(t,x,A,B(:,1),C,input_data,W,N,p.k2,p.J_T,p.J_B),t,states0);

% Parse States
theta_hat = states(:,1);
omega_T_hat = states(:,2);
omega_B_hat = states(:,3);
Sig33 = states(:,end) ;   % Parse out the (3,3) element of Sigma only!
Sigma = reshape(states(end,4:end),[3 3]);
L_end = Sigma*C'*inv(N);
disp(eig(A-L_end*C));
% Compute the upper and lower bounds as described in Problem 5c.
omega_B_hat_upperbound = omega_B_hat + sqrt(Sig33);
omega_B_hat_lowerbound = omega_B_hat - sqrt(Sig33);

% Plot Results
figure(4); clf;

subplot(2,1,1);
plot(tsim, omega_B_true, 'b', tsim, omega_B_hat, 'g',...
    tsim, omega_B_hat - sqrt(Sig33), 'k-.', tsim, omega_B_hat + sqrt(Sig33), 'k-.');

xlabel('time (s)');
ylabel('Rotational Speed of Bit (radians/sec) ')
legend('Estimated Rotational Speed of Bit', 'True Rotational Speed of Bit', 'One standard deviation upper bound', 'One Standard Deviation Lower Bound');

%   Plot true and estimated bit velocity
%   Plot estimated bit velocity plus/minus one sigma

subplot(2,1,2);
plot(tsim, omega_B_true - omega_B_hat);
%   Plot error between true and estimated bit velocity
legend('Error between true and estimated bit velocity')
xlabel('time (s)')
ylabel({'$$\omega_{B}$$ - $$\hat{\omega_{B}}$$'}, 'Interpreter', 'Latex')
        

omega_B_tilde = omega_B_true - omega_B_hat;
RMSE = sqrt(mean(omega_B_tilde.^2));
fprintf(1,'Extended Kalman Filter RMSE: %1.4f rad/s\n',RMSE);
% now you are on your own!
function [xdot] = ode_ekf(t,x,A,B,C,input_data,W,N,k2,JT,JB)

% Parse States
x_hat = x(1:3);
Sig = reshape(x(4:end)  ,[3 3]);   % Need to reshape Sigma from vector to matrix

% Parse and interpolate input signal data
%   This is a coding trick for time-varying input data.
it = input_data(:,1);
iT = input_data(:,2);
iy_m = input_data(:,3);

T = interp1(it,iT,t);
y_m = interp1(it,iy_m,t);

u= [T,(x_hat(1,:)).^3];
F = A + [0,0,0; -3*k2*(x_hat(1,:)^2)/JT,0,0;3*k2*(x_hat(1,:)^2)/JB,0,0];
H = C;
% Compute Kalman Gain (Look at Chapter 3, Section 4)
L = Sig*H'*inv(N);

% Kalman Filter equations
x_hat_dot = (A-L*C)*x_hat+ B*T+ L*y_m;

% Riccati Equation for Sigma
Sig_dot = Sig*F'+F*Sig+W-Sig*H'*inv(N)*H*Sig;

% Concatenate LHS
xdot = [x_hat_dot; reshape(Sig_dot, [9 1])];
end
function [xdot] = ode_kf(t,x,A,B,C,input_data,W,N)

% Parse States
x_hat = x(1:3);
Sig = reshape(x(4:end),[3 3]);   % Need to reshape Sigma from vector to matrix

% Parse and interpolate input signal data
%   This is a coding trick for time-varying input data.
it = input_data(:,1);
iT = input_data(:,2);
iy_m = input_data(:,3);

T = interp1(it,iT,t);
y_m = interp1(it,iy_m,t);
% Compute Kalman Gain (Look at Chapter 3, Section 4)
L = Sig*C'*inv(N);

% Kalman Filter equations
x_hat_dot = (A-L*C)*x_hat+ B*T+ L*y_m;

% Riccati Equation for Sigma
Sig_dot = Sig*A'+A*Sig+W-Sig*C'*inv(N)*C*Sig;

% Concatenate LHS
xdot = [x_hat_dot; reshape(Sig_dot, [9 1])];
end
