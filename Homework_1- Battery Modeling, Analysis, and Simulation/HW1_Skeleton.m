%% CE 295 - Energy Systems and Control
%   HW 1 : Battery Modeling, Analysis, and Control
%   Oski Bear, SID 18681868
%   Prof. Moura
%   Due Feb 6, 2015

% BEAR_OSKI_HW1.m

clear; close all;
fs = 15;    % Font Size for plots

%% Part(a): Model Parameters

%%% Enter model parameters here
% ECM parameters
Q = 3600; % [Coulombs]
R1 = 0.05; % [Ohms]
R2 =0.005 ; % [Ohms]
C = 500; % [Farads]

% OCV polynomial coefficients
p_0 = 3.4707;
p_1 = 1.6112;
p_2 = -2.6287; 
p_3 = 1.7175;

% Plot nonlinear OCV function
z_vec = linspace(0,1,25);
OCV = p_0 + p_1*z_vec + p_2*z_vec.^2 + p_3*z_vec.^3;

figure(1); clf;
plot(z_vec, OCV )
ylabel('OCV [volts]','FontSize',fs)
xlabel('SOC, z [-]','FontSize',fs)
set(gca,'FontSize',fs);

% Assemble (A,B) state-space matrices
A = [0 0; 0 -1/(C*R2)];
B = [1/Q; 1/C];

% Output states only (dummy variables, not used later)
C_dummy = eye(2);
D_dummy = 0;

% Create state-space model
ecm_sys = ss( A,B ,C_dummy , D_dummy );

%% Part(b): Simulate

% Create time vector
DeltaT = 1; % Time step size [sec]
t = 0:DeltaT:(10*60);   % Total Simulation time (min*sec/min)

% Input current signals
Current = zeros(size(t))';
Current(mod(t,40) < 20) = -5;   % 20sec 5A pulse discharge

% Initial conditions
z0 = 0.5;   % State-of-charge
V_c0 = 0;   % Capacitor voltage
x0 = [z0; V_c0];    % Vectorize initial conditions

% Simulate linear dynamics (Read documentation on lsim)
[ ~,T ,x ] = lsim( ecm_sys, Current, t   ,x0 );

% Parse out states
z = x(:,1);
V_c = x(:,2);

% Compute nonlinear output function [enter output function ]
V_nl =   V_c + Current*R1 + (p_0 + p_1*z + p_2*z.^2 + p_3*z.^3);                              ;

%%% Compute linearized output function
% Linearization Points
zeq =0.5;    % State-of-charge
V_ceq =0 ;   % Capacitor voltage
Ieq =0 ;     % Current

V_lin =  (p_0 + 0.5*p_1 + 0.25*p_2 + 0.125*p_3)+ V_c + Current*R1 + (p_1 + p_2 + 0.75*p_3)*(z-zeq);     ; % Linearized version of output function

%% Part(b): Plot results

figure(2); clf;

% Current
subplot(3,1,1);
plot(T, Current )
ylabel('Current (amps)', 'FontSize', fs)

% State-of-charge
subplot(3,1,2);
plot(T,z   )
ylabel('SOC, z[-]', 'FontSize', fs  )

% Nonlinear and Linear Voltage
subplot(3,1,3);
plot(T,V_nl, 'b', T, V_lin, 'k' )
ylabel('Voltages (volts)','FontSize',fs )
legend( 'V nonlinear', 'V linear')
xlabel('Time[min*sec/min]' );



