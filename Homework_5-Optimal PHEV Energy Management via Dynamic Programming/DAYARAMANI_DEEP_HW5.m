%% CE 295 - Energy Systems and Control
%   HW 5 : Optimal Energy Management of PHEV via Dynamic Programming
%   Deeo Dayaramani, SID 3032083601
%   Prof. Moura



clear; close all;
fs = 15;    % Font Size for plots

%% Parameters and Data

% Time step
Delta_t = 1;

% Fuel consumption in grams per unit energy
alph = 1e-4;     % [g/(s-W)]

Qcap = 5*3600;        % [A-s = Coulombs]
V_oc = 330;             % [volts]

% Limits on Batt Power, Eng Power, SOC
P_batt_max = 15e3; % [W]
P_eng_max = 35e3;  % [W]

SOC_min = 0.25;      % [-]
SOC_max = 0.9;      % [-]


% Plot Power Demand Data
M = csvread('UDDS_Pdem.csv',1,0);
t = M(:,1);
P_dem = M(:,2)*1e3;  % convert from kW to W
v_dc = M(:,3);

figure(1); clf;

subplot(2,1,1);
% plot speed
plot(t, v_dc, '-k')
xlabel('Time in seconds')
ylabel('Drive Cycle Speed (m/s)')
subplot(2,1,2)
% plot power demand
plot(t, P_dem, '-b')
ylabel('Power Demand in kW')
xlabel('Time in seconds')
% Plot Engine efficiency Curve
eff = [];
for i= 1:length(P_dem)
    eff(i) = eta_eng(P_dem(i));
end
figure(2); clf;
plot(P_dem, eff, '-k') % plot efficiency versus engine power, for total range of engine powers

%% Grid State and Preallocate
SOC_grid = (SOC_min:0.005:SOC_max)';

% Grid size
ns = length(SOC_grid);  % No. of states

% Planning horizon (time steps)
N = length(t);

% Preallocate Value Function (rows index state, columns index time)
V = inf*ones(ns,N+1);

% Preallocate Control (rows index state, columns index time)
u_star = zeros(ns,N);

%% Solve DP
tic;
% Boundary Condition of Value Function (Principle of Optimality)
V(:,N+1) = 0;
% Iterate backward in time
for k = N:-1:1
    % Iterate over SOC
    for idx = 1:ns
        bound1 = ((-SOC_max + SOC_grid(idx))*Qcap*V_oc/Delta_t);
        bound2 = -P_batt_max;
        bound3 = P_dem(k) - P_eng_max;
        bound4 = ((-SOC_min + SOC_grid(idx))*Qcap*V_oc/Delta_t);
        bound5 = P_batt_max;
        bound6 = P_dem(k);
        % Find dominant bounds
        lb = max([bound1 bound2 bound3]);
        ub = min([bound4 bound5 bound6]);           
        % Grid Battery Power between dominant bounds
        P_batt_grid = linspace(lb,ub,200)';
        % Compute engine power (vectorized for all P_batt_grid)
        P_eng = P_dem(k) - P_batt_grid;         
        % Cost-per-time-step (vectorized for all P_batt_grid)
        g_k = alph*Delta_t*P_eng./eta_eng(P_eng);        
        % Calculate next SOC (vectorized for all P_batt_grid)
        SOC_nxt = SOC_grid(idx)-Delta_t/Qcap/V_oc*P_batt_grid;        
        % Compute value function at nxt time step (need to interpolate)
        V_nxt = interp1(SOC_grid,V(:,k+1),SOC_nxt,'linear');
        % Value Function (Principle of Optimality)
        [V(idx, k), ind] = min(g_k+V_nxt);        
        % Save Optimal Control
        u_star(idx,k) = P_batt_grid(ind);
    end
end
solveTime = toc;
fprintf(1,'DP Solver Time %2.2f sec \n',solveTime);

%% Simulate Results

% Preallocate
SOC_sim = zeros(N,1);
P_batt_sim = zeros(N,1);
P_eng_sim = zeros(N,1);
J_sim = zeros(N,1);

% Initialize
SOC_0 = 0.75;
SOC_sim(1) = SOC_0;

% Simulate PHEV Dynamics
for k = 1:(N-1)
    
    % Use optimal battery power, for given SOC (need to interpolate)
    P_batt_sim(k) = interp1(SOC_grid, u_star(:,k), SOC_sim(k) );
    
    % Compute engine power
    P_eng_sim(k) = P_dem(k)-P_batt_sim(k);
    
    % Fuel Consumption
    J_sim(k) = alph*Delta_t*P_eng_sim(k)/eta_eng(P_eng_sim(k));
    
    % Time-step SOC dynamics
    SOC_sim(k+1) = SOC_sim(k) -Delta_t/Qcap/V_oc*P_batt_sim(k);
    
end

fprintf(1,'Total Fuel Consumption %2.2f kg \n',sum(J_sim)/1e3);

%% Plot Results
figure(3); clf;

subplot(4,1,1);
% UDDS speed versus time 
plot(t, v_dc, '-k');
xlabel('Time in seconds');
ylabel('UDDS Speed in m/s');
title('UDDS Speed vs time')
subplot(4,1,2);
% SOC versus time
plot(t, SOC_sim, '-b');

xlabel('Time in seconds');
ylabel('SOC of the Battery');
title('SOC of Battery vs time')
subplot(4,1,3);
% Accumulated fuel consumption [g] versus time
for i = 1:length(J_sim)
   J_new(i) = sum(J_sim(1:i)); 
end
plot(t, J_new,'-r');

xlabel('Time in seconds');
ylabel('Accumulated Fuel Consumption in g');
title('Accumulated Fuel Consumption vs Time')
subplot(4,1,4);
% Battery and engine power [kW] versus time
hold on
plot(t,P_batt_sim,'-b');
plot(t,P_eng_sim,'-k');
hold off

xlabel('Time in seconds');
ylabel('Power in kW');
legend('Battery Power', 'Engine Power');
title('Battery and Engine Power vs Time')
%%
if sum(SOC_sim>SOC_max | SOC_sim<SOC_min) ==0
        disp('SOC constraints satisfied');
end

if (P_batt_sim<=P_batt_max)
    if (P_batt_sim>=-P_batt_max)
        disp('P_batt constriants sarsfied');
    end
end

if (P_eng_sim<=P_eng_max)
    if (P_eng_sim>=0)
        disp('P_eng constriants sarsfied')
    end
end

%%
SOC_0 = 0.9:-0.15:0.3;
figure
subplot(2,1,1)
grid on;    hold on;
xlabel('time in seconds');  ylabel('Final SOC of Battery');
J=zeros(length(SOC_0),1);

for j=1:length(SOC_0)
    SOC_sim(1) = SOC_0(j);

    % Simulate PHEV Dynamics
    for k = 1:(N-1)
    
        % Use optimal battery power, for given SOC (need to interpolate)
        P_batt_sim(k) = interp1(SOC_grid,u_star(:,k),SOC_sim(k),'linear');
    
        % Compute engine power
        P_eng_sim(k) = P_dem(k)-P_batt_sim(k);
    
        % Fuel Consumption
        J_sim(k) = alph*Delta_t*(P_eng_sim(k)/eta_eng(P_eng_sim(k)));
    
        % Time-step SOC dynamics
        SOC_sim(k+1) = SOC_sim(k)-(Delta_t/(Qcap*V_oc))*P_batt_sim(k);
    end
    subplot(2,1,1)
    plot(t,SOC_sim,'DisplayName',['SOC_0=' num2str(SOC_0(j))]);
    J(j)=sum(J_sim/1000);
end
legend show

subplot(2,1,2)
plot(SOC_0,J,'-*');
grid on;    hold on;
xlabel({'$SOC_0$'},'interpreter','latex');  ylabel('TFC [kg]');
