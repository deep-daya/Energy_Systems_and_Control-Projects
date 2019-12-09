clear;
clc;
%% Load Data
tracks=csvread('VRD211.csv',1,1); %load data
xd=[tracks(:,7)';tracks(:,8)';tracks(:,4)'*0.514444]; %store [lon;lat;speed]
tf=size(xd,2)+1; %define time
xd=[xd;zeros(2,tf-1)]; %add new column for 

%% Plot Actual data
figure(1);
plot(xd(1,:)/1000,xd(2,:)/1000,'-o','DisplayName','Actual trajectory');
xlabel('x [km]');  ylabel('y [km]')
hold on

figure(2)
subplot(5,1,1)
plot(0:size(xd,2)-1,xd(1,:)/1000,'--','DisplayName','Actual')
hold on
xlabel('time [min]');    ylabel('x [km]');
grid on

subplot(5,1,2)
plot(0:size(xd,2)-1,xd(2,:)/1000,'--','DisplayName','Actual')
xlabel('time [min]');    ylabel('y [km]');
hold on 
grid on   

subplot(5,1,3)
plot(0:size(xd,2)-1,xd(3,:),'--','DisplayName','Actual')
xlabel('time [min]');    ylabel('v [m/s]');
hold on
grid on   

%% Drag Force Parameters:
Cd= 0.04;            %Drag force coefficient [-]
rho= 0.38;           %air density [kg/m3] 
eta= 0.01667*10^-3;  %Thrust-specific fuel consumption [kg/(N*s)]
S= 122.6;            %[m2]...Aircraft wing area

%% Star & End location coordinates
%ORD=[272.093, 41.9742]; %start coord
%SFO=[237.617, 37.6211]; %final coord

%coordinate refer to ORD [m]
%this is a rough conversion from the original WGS84 coordiantes 
ORD=[0, 0]; %start coord 
SFO=[-2895204.87, -491111.25]; %final coord  

%% Wind approximated funciton coefficient
a1=5.404039761756626e-12;
a2=-7.525095226657606e-09;
a3=-1.0097962636800737e-05;
a4=0.0018023200165759225;
a5=0.30541919780328985;
a6=1.0706181973188272e-12;
a7=8.130657646668963e-09;
a8=1.9566392595939457e-05;
a9=0.013598711210671936;
a10=0.3054111118706383;
a11=-4.4925592769414514e-13;
a12=1.3721355294688644e-12;
a13=-1.9705270758495876e-12;

b1=6.505580005194991e-12;
b2=-2.3582120860024345e-10;
b3=-2.0098332293991807e-06;
b4=-8.207601052061964e-06;
b5=6.216049027788961;
b6=-2.184365129705414e-12;
b7=-1.574583855440115e-08;
b8=-1.7909052661551478e-05;
b9=0.0358673394470828;
b10=6.216049027919514;

%% Simulation Process 
% Testing different size of time step to see impact on the result by
% reducing total time horizon by a samll number, defined by delta, in each 
% loop. One can also choose to change the number of time step (N).

% Also, Total time horizon (T) can be changed to test a wider time range. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Choose Time parameters
N=30;    %Total number of time step
T=4;     %Total time horizon [h]
delta=0.1;
%% Cost Matrics
Q=diag([10^3,10^3,0,0,0]);  %state cost
R=diag([0,0]);              %input cost

%% Initial/Final States
%x=[x; y; v; m; theta]      u=[T; yaw];
x0=[ORD'; 100; 78000; deg2rad(127)];   %initial states
xf=[SFO'; 0; 0; 0];                    %final states

%% State/Input Constraints
xlb=[-inf; -inf; 0; 37200; -inf]; %states lower bound
xub=[inf; inf; 250; 78000; inf];  %states upper bound
ulb=[0; -2.5*pi/180];             %input lower bound
uub=[120000*2; 2.5*pi/180];       %input upper bound

%% Main 
for ii=0:4 
    %% Simulation Parameters
    dT=(T-delta*ii)*3600/N;          %[s] time step     
    %% Non-linear discrete aircraft dynamic model
    %Aircraft dynamics
    %x=[x;y;v;m;theta]      u=[T;yaw];

    dyn=@(x,u) [x(1)+dT*(x(3)*cos(x(5))+a1*(x(2)^4/1000^4)+...
        a2*(x(2)^3/1000^3)+a3*(x(2)^2/1000^2)+a4*(x(2)/1000)+a5+...
        a6*(x(1)^4/1000^4)+a7*(x(1)^3/1000^3)+a8*(x(1)^2/1000^2)+...
        a9*(x(1)/1000)+a10+a11*(x(1)^3/1000^3)*(x(2)/1000)+...
        a12*(x(1)^2/1000^2)*(x(2)^2/1000^2)+a13*(x(1)/1000)*(x(2)^3/1000^3));
        
                x(2)+dT*(x(3)*sin(x(5))+b1*(x(2)^4/1000^4)+...
                b2*(x(2)^3/1000^3)+b3*(x(2)^2/1000^2)+b4*(x(2)/1000)+...
                b5+b6*(x(1)^4/1000^4)+b7*(x(1)^3/1000^3)+...
                b8*(x(1)^2/1000^2)+b9*(x(1)/1000)+b10);
                
                x(3)+dT*(2*u(1)-Cd*rho*S*(x(3)^2))/(2*x(4));
                
                x(4)-dT*eta*u(1);
                
                x(5)+dT*u(2)];

    %% Path Planning
    tic
    [xopt,uopt,flag]=CFTOC(dyn,Q,R,x0,xf,xlb,xub,ulb,uub,dT,N);
    toc

    %% Visualization
    %trajectory
    figure(1); hold on
    title(['Flight Trajectory N=' num2str(N)])
    plot(xopt(1,:)/1000,xopt(2,:)/1000,'-*','DisplayName',['Optimized trajectory, T=' num2str(T-delta*ii)]);
    hold on
    legend show
    grid on

    %states-longtitude
    figure(2); hold on
    subplot(5,1,1)
    plot(0:dT/60:N*dT/60,xopt(1,:)/1000,'DisplayName',['Optimized trajectory, T=' num2str(T-delta*ii)]);
    hold on
    xlabel('time [min]');    ylabel('x [km]');
    legend show
    grid on

    %states-latitude
    subplot(5,1,2)
    plot(0:dT/60:N*dT/60,xopt(2,:)/1000,'DisplayName',['Optimized trajectory, T=' num2str(T-delta*ii)]);
    xlabel('time [min]');    ylabel('y [km]');
    legend show
    hold on 
    grid on   

    %states-speed
    subplot(5,1,3)
    plot(0:dT/60:N*dT/60,xopt(3,:),'DisplayName',['Optimized trajectory, T=' num2str(T-delta*ii)]);
    xlabel('time [min]');    ylabel('v [m/s]');
    legend show
    hold on
    grid on    

    %states-mass
    subplot(5,1,4)
    plot(0:dT/60:N*dT/60,xopt(4,:)/1000,'DisplayName',['Optimized trajectory, T=' num2str(T-delta*ii)])
    xlabel('time [min]');    ylabel('m [ton]');
    legend show
    hold on
    grid on    

    %states-"heading angle"
    subplot(5,1,5)
    plot(0:dT/60:N*dT/60,rad2deg(xopt(5,:)),'DisplayName',['Optimized trajectory, T=' num2str(T-delta*ii)])
    xlabel('time [min]');    ylabel('theta [deg]');
    legend show
    hold on
    grid on    

    %input
    figure(3);
    subplot(2,1,1)
    plot(0:dT/60:N*dT/60-1,uopt(1,:)/1000,'DisplayName',['Optimized trajectory, T=' num2str(T-delta*ii)])
    xlabel('time [min]');    ylabel('T [kN]');
    legend show
    hold on;
    grid on  

    subplot(2,1,2)
    plot(0:dT/60:N*dT/60-1,uopt(2,:),'DisplayName',['Optimized trajectory, T=' num2str(T-delta*ii)])
    xlabel('time [min]');    ylabel('turning rate [rad/s]');
    legend show
    hold on
    grid on    

end





