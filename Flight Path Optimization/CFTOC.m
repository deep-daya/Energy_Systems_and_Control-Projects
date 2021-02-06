function [xopt,uopt,flag] = CFTOC(dyn,Q,R,x0,xf,xlb,xub,ulb,uub,dT,N)
% Using ipopt to solve a constrainted finite time optimal control problem
% for a non-linear system using discretized non-linear dynamic model

%% Initialize variables and cost
disp('initialize...')
%states:[x ;y ;v ;m ;theta]
%input:[T ;yaw]
x = sdpvar(5,N+1);
u = sdpvar(2,N);
%cost
cost=[];

%% Constraints & Cost function
%initial states constraint:
constraint = [x(:,1)==x0,x(1:3,end)==xf(1:3)];  %initial/final state

for k=1:N
    %states dynamics
    constraint = constraint + [x(:,k+1)==dyn(x(:,k),u(:,k))];
    
    %states constraints
    constraint = constraint + [xlb <= x(:,k)<= xub];
    constraint = constraint + [-0.6*dT <= (x(3,k+1)-x(3,k))<= 0.6*dT];
    
    %input constraints
    constraint = constraint + [ulb <= u(:,k)<= uub];
    %cost
    cost=cost+(x(:,k)-xf)'*Q*(x(:,k)-xf)+u(:,k)'*R*u(:,k);
end

%% slove optimization problem
disp('optimizing...')
%solver settings
options=sdpsettings('solver','ipopt','verbose',0,'showprogress',1);
%options.ipopt.check_derivatives_for_naninf='yes';
result=optimize(constraint,cost,options);

%check feasibility
flag=result.problem;
disp(yalmiperror(flag));
if flag==0 
    %solution found
    xopt=double(x);    uopt=double(u);
else
    %no feasible solution
    xopt=[];    uopt=[];
end
end





