%% CE 295 - Energy Systems and Control
%   HW 3 : Optimal Economic Dispatch in Distribution Feeders with Renewables
%   Deep Dayaramani, SID 3032083601
%   Prof. Moura


clear; close all;
fs = 15;    % Font Size for plots

%% 13 Node IEEE Test Feeder Parameters

%%% Node (aka Bus) Data
% l_j^P: Active power consumption [MW]
l_P = [ 0
        0.2
        0
        0.4
        0.17
        0.23
        1.155
        0
        0.17
        0.843
        0
        0.17
        0.128];

% l_j^Q: Reactive power consumption [MVAr]
l_Q = [ 0
    0.1160
         0
    0.2900
    0.1250
    0.1320
    0.6600
         0
    0.1510
    0.4620
         0
    0.0800
    0.0860  ];

% l_j^S: Apparent power consumption [MVA]
l_S = sqrt(l_P.^2 + l_Q.^2);

% s_j,max: Maximal generating power [MW]
s_max = [5
     0
     0
     3
     0
     0
     0
     0
     0
     3
     0
     0
     0   ];

% c_j: Marginal generation cost [USD/MW]
c = [100
     0
     0
   150
     0
     0
     0
     0
     0
    50
     0
     0
     0    ];

% V_min, V_max: Minimum and maximum nodal voltages [V]
%v_min = 0.95;
%v_max = 1.05;
v_min = 0.98;
v_max = 1.02;
%%% Edge (aka Line) Data
% r_ij: Resistance [p.u.]
r = [0	0.007547918	0	0	0	0	0	0	0	0	0	0	0
0	0	0.0041	0	0.007239685	0	0.007547918	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0.004343811	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0.003773959	0	0	0.004322245	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0.00434686	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0.004343157	0.01169764
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0   ];

% x_ij: Reactance [p.u.]
x = [0	0.022173236	0	0	0	0	0	0	0	0	0	0	0
0	0	0.0064	0	0.007336076	0	0.022173236	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0.004401645	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0.011086618	0	0	0.004433667	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0.002430473	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0.004402952	0.004490848
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0    ];

% I_max_ij: Maximal line current [p.u.]
I_max = [0	3.0441	0	0	0	0	0	0	0	0	0	0	0
0	0	1.4178	0	0.9591	0	3.0441	0	0	0	0	0	0
0	0	0	3.1275	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0.9591	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	3.0441	3.1275	0	0.9591	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	1.37193	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0.9591	1.2927
0	0	0	0	0	0	0	0	0	0	0	0	0
0	0	0	0	0	0	0	0	0	0	0	0	0    ];

% A_ij: Adjacency matrix; A_ij = 1 if i is parent of j
A = [0 1 0 0 0 0 0 0 0 0 0 0 0
0 0 1 0 1 0 1 0 0 0 0 0 0
0 0 0 1 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 1 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 1 1 0 1 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 1 1
0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0   ];


%%% Set Data (add +1 everywhere for Matlab indexing)
% \rho(j): Parent node of node j
rho = [ 1
     1
     2
     3
     2
     5
     2
     7
     7
     9
     7
    11
    11  ];


%% Problem 1

% Plot active and reactive power consumption
figure(1);
bar([0:12;0:12]',[l_P,l_Q]);
legend ('Active power consumption [MW]','Reactive power consumption [MVAr]')
xlabel('node')
ylabel('power consumption [MW]/[MVAr]')

%% Problem 2

% Assumptions:
%   - Disregard the entire network diagram
%   - Balance supply and demand, without any network considerations
%   - Goal is to minimize generation costs, given by c^T s

% Solve with CVX
cvx_begin
     variables q(13) s(13) p(13) % declare your optimization variables here
    minimize(c'*s) % objective function here
    subject to % constraints
        % Balance power generation with power consumption
        sum(l_S) == sum(s);
        % Loop over each node
        for jj = 1:13
            % Non-negative power generation
            p(jj)>=0;
            q(jj)>=0;
            % Compute apparent power from active & reactive power
            s(jj)>= norm([p(jj),q(jj)],2); 
        end
        % Apparent Power Limits        
        s<=s_max;
cvx_end

% Output Results
fprintf(1,'------------------- PROBLEM 2 --------------------\n');
fprintf(1,'--------------------------------------------------\n');
fprintf(1,'Minimum Generating Cost : %4.2f USD\n',cvx_optval);
fprintf(1,'\n');
fprintf(1,'Node 0 [Grid]  Gen Power : p_0 = %1.3f MW | q_0 = %1.3f MW | s_0 = %1.3f MW\n',p(1),q(1),s(1));
fprintf(1,'Node 3 [Gas]   Gen Power : p_3 = %1.3f MW | q_3 = %1.3f MW | s_3 = %1.3f MW\n',p(4),q(4),s(4));
fprintf(1,'Node 9 [Solar] Gen Power : p_9 = %1.3f MW | q_9 = %1.3f MW | s_9 = %1.3f MW\n',p(10),q(10),s(10));
fprintf(1,'\n');
fprintf(1,'Total active power   : %1.3f MW   consumed | %1.3f MW   generated\n',sum(l_P),sum(p));
fprintf(1,'Total reactive power : %1.3f MVAr consumed | %1.3f MVAr generated\n',sum(l_Q),sum(q));
fprintf(1,'Total apparent power : %1.3f MVA  consumed | %1.3f MVA  generated\n',sum(l_S),sum(s));



%% Problem 3

% Assumptions:
%   - Disregard L_ij, the squared magnitude of complex line current
%   - Disregard nodal voltage equation
%   - Disregard nodal voltage limits
%   - Disregard maximum line current
%   - Goal is to minimize generation costs, given by c^T s

% Solve with CVX
cvx_begin
    variables p(13) q(13) s(13) P(13,13) Q(13,13)
    dual variable mu_s
    minimize(  c'*s   )
    subject to
    
        % Boundary condition for power line flows
        P( 1 , 1 ) == 0;
        Q( 1 , 1 ) == 0;
        
        % Loop over each node
        for jj = 1:13
            p(jj)>=0;
            q(jj)>=0;
            % Parent node, i = \rho(j)
            i=rho(jj);
            % Line Power Flows
            P(i,jj) == (l_P(jj)-p(jj))+ A(jj,:)*P(jj,:)';
            Q(i,jj) == (l_Q(jj)-q(jj))+ A(jj,:)*Q(jj,:)';
            % Compute apparent power from active & reactive power
            s(jj) >=  norm([p(jj),q(jj)],2);
            
            
        end
        
        % Apparent Power Limits
        mu_s:s <= s_max
        
cvx_end

% Output Results
fprintf(1,'------------------- PROBLEM 3 --------------------\n');
fprintf(1,'--------------------------------------------------\n');
fprintf(1,'Minimum Generating Cost : %4.2f USD\n',cvx_optval);
fprintf(1,'\n');
fprintf(1,'Node 0 [Grid]  Gen Power : p_0 = %1.3f MW | q_0 = %1.3f MW | s_0 = %1.3f MW || mu_s0 = %3.0f USD/MW\n',p(1),q(1),s(1),mu_s(1));
fprintf(1,'Node 3 [Gas]   Gen Power : p_3 = %1.3f MW | q_3 = %1.3f MW | s_3 = %1.3f MW || mu_s3 = %3.0f USD/MW\n',p(4),q(4),s(4),mu_s(4));
fprintf(1,'Node 9 [Solar] Gen Power : p_9 = %1.3f MW | q_9 = %1.3f MW | s_9 = %1.3f MW || mu_s9 = %3.0f USD/MW\n',p(10),q(10),s(10),mu_s(10));
fprintf(1,'\n');
fprintf(1,'Total active power   : %1.3f MW   consumed | %1.3f MW   generated\n',sum(l_P),sum(p));
fprintf(1,'Total reactive power : %1.3f MVAr consumed | %1.3f MVAr generated\n',sum(l_Q),sum(q));
fprintf(1,'Total apparent power : %1.3f MVA  consumed | %1.3f MVA  generated\n',sum(l_S),sum(s));

%% Problem 4

% Assumptions:
%   - Add back all previously disregarded terms and constraints
%   - Relax squared line current equation into inequality
%   - Goal is to minimize generation costs, given by c^T s

% Solve with CVX
cvx_begin
    variables  q(13) s(13) Q(13,13) P(13,13)  L(13,13) V(13) p(13)
    dual variables  mu_ss{13} mu_s mu_q{13} mu_Lub{13} mu_p{13} mu_L{13}  mu_Vlb mu_Vub 
    minimize(c'*s   )
    subject to
    % Boundary condition for power line flows
        P( 1 , 1 ) == 0;
        Q( 1 , 1 ) == 0;
        % Boundary condition for squared line current
        L( 1 , 1 ) == 0;
        % Fix node 0 voltage to be 1 "per unit" (p.u.)
        V(1) == 1;
        % Loop over each node
        for jj = 1:13
            mu_p{jj}:p(jj)>=0;
            mu_q{jj}:q(jj)>=0;
            % Parent node, i = \rho(j)
            i=rho(jj);
            % Line Power Flows
            P(i,jj)==(l_P(jj)-p(jj))+ r(i,jj)*L(i,jj)+ A(jj,:)*P(jj,:)';
            Q(i,jj)==(l_Q(jj)-q(jj))+ x(i,jj)*L(i,jj)+ A(jj,:)*Q(jj,:)';
            % Nodal voltage
            V(jj)==V(i)+(r(i,jj)^2+x(i,jj)^2)*L(i,jj)-2*(r(i,jj)*P(i,jj)+x(i,jj)*Q(i,jj));
            % Squared current magnitude on lines
            mu_L{jj}:L(i,jj)>=quad_over_lin([P(i,jj);Q(i,jj)],V(jj));
            % Compute apparent power from active & reactive power
            mu_ss{jj}:s(jj)>= norm([p(jj),q(jj)],2);
            % Squared line current limits
            mu_Lub{jj}:L(i,jj)<=I_max(i,jj)^2;            
        end     
        % Nodal voltage limits
        mu_Vlb:v_min^2<=V;
        mu_Vub:V<=v_max^2;
        % Apparent Power Limits
        mu_s: s<=s_max; 
        
        
cvx_end

% Output Results
fprintf(1,'------------------- PROBLEM 4 --------------------\n');
fprintf(1,'--------------------------------------------------\n');
fprintf(1,'Minimum Generating Cost : %4.2f USD\n',cvx_optval);
fprintf(1,'\n');
fprintf(1,'Node 0 [Grid]  Gen Power : p_0 = %1.3f MW | q_0 = %1.3f MW | s_0 = %1.3f MW || mu_s0 = %3.0f USD/MW\n',p(1),q(1),s(1),mu_s(1));
fprintf(1,'Node 3 [Gas]   Gen Power : p_3 = %1.3f MW | q_3 = %1.3f MW | s_3 = %1.3f MW || mu_s3 = %3.0f USD/MW\n',p(4),q(4),s(4),mu_s(4));
fprintf(1,'Node 9 [Solar] Gen Power : p_9 = %1.3f MW | q_9 = %1.3f MW | s_9 = %1.3f MW || mu_s9 = %3.0f USD/MW\n',p(10),q(10),s(10),mu_s(10));
fprintf(1,'\n');
fprintf(1,'Total active power   : %1.3f MW   consumed | %1.3f MW   generated\n',sum(l_P),sum(p));
fprintf(1,'Total reactive power : %1.3f MVAr consumed | %1.3f MVAr generated\n',sum(l_Q),sum(q));
fprintf(1,'Total apparent power : %1.3f MVA  consumed | %1.3f MVA  generated\n',sum(l_S),sum(s));
fprintf(1,'\n');
for jj = 1:13
    fprintf(1,'Node %2.0f Voltage : %1.3f p.u.\n',jj,sqrt(V(jj)));
end


