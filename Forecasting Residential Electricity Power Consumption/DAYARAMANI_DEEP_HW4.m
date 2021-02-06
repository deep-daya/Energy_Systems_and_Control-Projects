%% Prob 1
Training_Data_1 = readtable('HW4_Train_Data.csv');
Training_Data_2 = table2array(Training_Data_1(:,4:end));

% part a)
average_kWh = sum(Training_Data_2) / size(Training_Data_2, 1);
bdx_index = 1:size(Training_Data_2, 2);
stand_kWh = std(Training_Data_2);
error_kWh = stand_kWh/sqrt(size(Training_Data_2, 1));

figure();
hold on
bar(bdx_index, average_kWh);
xlim([0.5 23.5]);
xticks(bdx_index);

errorbar(bdx_index, average_kWh, stand_kWh, 'color', 'r', 'LineStyle', 'none');
xlabel('Building Number', 'fontsize', 13 );
ylabel('Average Energy Consumption [kWh]', 'fontsize', 13);
legend('Average Energy Consumption [kWh]', 'error [kWh]');
%%
% part b)
% building 6 has an abnormally large variance
Training_Data_3 = [];  
neg_bldg = [];
for i = 1:size(Training_Data_2, 2)
    if all(Training_Data_2(:, i)>=0)
        Training_Data_3 = [Training_Data_3 Training_Data_2(:, i)];
    else
        neg_bldg = [neg_bldg i];  
    end
end
%%
% part c)
WoY = weeknum(datestr(table2array(Training_Data_1(:,3))));  %datetime. 
DoW = weekday(datestr(table2array(Training_Data_1(:,3))));  % day of week
HoD = hour(datestr(table2array(Training_Data_1(:,3))));  % hour of day
Training_Data_4D = [];
Training_Data_Temp = [WoY DoW HoD Training_Data_3];
%%
Training_Data_3_normalized = Training_Data_3./max(Training_Data_3);

% Re-organize energy data
% building 1
for i = 1:size(Training_Data_3, 2)
    for j = 1:size(WoY, 1)
        Training_Data_4D(i, WoY(j), DoW(j), HoD(j)+1)...
            = Training_Data_3_normalized(j, i);
    end
end
%%
% plot figures of kWh
days = [string('Sunday'),string('Monday'),string('Tuesday'),string('Wednesday'),string('Thursday'),string('Friday'),string('Saturday')];
avg_days =  [];
for k = 1:length(days)
    figure();
    hold on
    title(days(k));
    xlabel('Time of Day');
    ylabel('Normalized Hourly Energy');
    xlim([0 23]);
    TempMean = [];
    for i = 1:size(Training_Data_3, 2)
        for j = 1:max(WoY)
            plot(0:23, squeeze(Training_Data_4D(i,j,k,:))); 
            TempMean = [TempMean squeeze(Training_Data_4D(i,j,k,:))];
        end
    end
   avg_days = [avg_days, mean(TempMean')];
    plot(0:23, mean(TempMean'), '-k', 'linewidth', 4);
end

%%

%question 2
TestData1 = readtable('HW4_Test_Data.xlsx');
test_days = [];
for i= 1:24:168
    test_days = [test_days, TestData1.TestBldg(i:23+i)];
end
avg_days_2 = [];
for i= 1:24:168
    avg_days_2 = [avg_days_2, avg_days(i:i+23)'];
end
days = [string('Sunday'),string('Monday'),string('Tuesday'),string('Wednesday'),string('Thursday'),string('Friday'),string('Saturday')];
for k = 1:length(days)
    figure();
    hold on
    title(days(k));
    xlabel('Time of Day');
    ylabel('Normalized Hourly Energy');
    xlim([0 23]);
    plot(0:23, avg_days_2(:,k), '-b', 'linewidth', 4);
    plot(0:23, test_days(:,k), '-k', 'linewidth', 4);
    legend('Predicted data', 'Test data');
end
%%
%question 2b
list_MAE = [];
for day =1:7
    P_avxx = 0;
    for hour= 1:24
        P_avxx = P_avxx + abs(test_days(hour,day) - avg_days_2(hour,day));
    end
    list_MAE = [list_MAE,(P_avxx/24)];
end


for i = 1: length(list_MAE)
    fprintf(1,'The MAE of  (%1.3s) +  is about %1.3f',days(i),list_MAE(i));
    disp(' ')
end


%% 
%question 3, part a, b
P_arx =[];
for i = 1:22
P_arx = [P_arx; Training_Data_3_normalized(2:end,i)];
end
P_avg = [];
for p = 1:22
    P_avg = [ P_avg; (avg_days(2:end))'];
    for i = 1:50
        P_avg = [P_avg; avg_days'];

    end
end
L = 3;
Y = P_arx-P_avg;
Phi = [];
for i = 1:22
    d  = rotate(Training_Data_3(:,i),L);
    Phi = [Phi; d];
end
%part c
alpha_star = (inv(Phi'*Phi)*Phi')*Y;
fprintf(1,'alpha_1 is about %1.3f',alpha_star(1));
disp(' ');
fprintf(1,'alpha_2 is about %1.3f',alpha_star(2));
disp(' ');
fprintf(1,'alpha_3 is about %1.3f',alpha_star(3));
%part d
Phi_test = [];
d = rotate(TestData1.TestBldg(:),L);
Phi_test = [Phi_test; d];

Y_test= Phi_test*alpha_star + (avg_days(2:end))';
test_days_temp = [TestData1.TestBldg(1,:); Y_test];
test_days_avx = [];
for i= 1:24:168
    test_days_avx = [test_days_avx, test_days_temp(i:23+i)];
end
days = [string('Sunday'),string('Monday'),string('Tuesday'),string('Wednesday'),string('Thursday'),string('Friday'),string('Saturday')];
for k = 1:length(days)
    figure();
    hold on
    title(days(k));
    xlabel('Time of Day');
    ylabel('Normalized Hourly Energy');
    xlim([0 23]);
    plot(0:23, avg_days_2(:,k), '-b', 'linewidth', 4);
    plot(0:23, test_days(:,k), '-k', 'linewidth', 4);
    plot(0:23, test_days_avx(:,k), '-o', 'linewidth', 4);
    legend('Predicted data', 'Test data', 'Predicted AVX data');
end
list_MAE = [];
for day =1:7
    P_avxx = 0;
    for hour= 1:24
        P_avxx = P_avxx + abs(test_days(hour,day) - test_days_avx(hour,day));
    end
    list_MAE = [list_MAE,(P_avxx/24)];
end


for i = 1: length(list_MAE)
    fprintf(1,'The MAE of  (%1.3s) +  is about %1.3f',days(i),list_MAE(i));
    disp(' ')
end



%% question 4


gamma = 10^-5; %choose small enough gamma
w=[0;0;0]; %initialize 

%Train Model to Obtain w's
for t = 1:3 %perform 3 time steps of gradient descent  
    temp = 0;
     for i=1:22
         
         P_avxx = Training_Data_3_normalized(:,i);
         P_hat = [];
    for k = 4:8568

    P_hat(k-3) = avg_days_2(HoD(k)+1,DoW(k))';
 
    y_i=P_avxx(k) - P_hat(k-3);
    x_i=[P_avxx(k-1), P_avxx(k-2), P_avxx(k-3)]';
    dJdW = (y_i - tanh((w')*x_i))*(1-(tanh((w')*x_i))^2)*x_i;
temp=temp+dJdW;
end
     end
     w=w+gamma*temp;
end

fprintf(1,'w_1 is about %1.3f',w(1));
disp(' ');
fprintf(1,'w_2 is about %1.3f',w(2));
disp(' ');
fprintf(1,'w_3 is about %1.3f',w(3));
%Part d 
NN = [avg_days_2(1,1)'; avg_days_2(1,2)'; avg_days_2(1,3)'];
for k=4:168
    x = [NN(k-1); NN(k-2); NN(k-3)];
    PNN_in = tanh((w)'*x) + avg_days(k)';
    NN = [NN; PNN_in];
end
NN_plot = reshape(NN,[24,7]);


days = [string('Sunday'),string('Monday'),string('Tuesday'),string('Wednesday'),string('Thursday'),string('Friday'),string('Saturday')];
for k = 1:length(days)
    figure();
    hold on
    title(days(k));
    xlabel('Time of Day');
    ylabel('Normalized Hourly Energy');
    xlim([0 23]);
    plot(0:23, avg_days_2(:,k), '-b', 'linewidth', 4);
    plot(0:23, test_days(:,k), '-k', 'linewidth', 4);
    plot(0:23, test_days_avx(:,k), '-o', 'linewidth', 4);
      plot(0:23, NN_plot(:,k), '-o', 'linewidth', 4);
    legend('Predicted data', 'Test data', 'Predicted AVX data','NN_plot predicted');
end
list_MAE = [];
for day =1:7
    P_avxx = 0;
    for hour= 1:24
        P_avxx = P_avxx + abs(test_days(hour,day) - test_days_avx(hour,day));
    end
    list_MAE = [list_MAE,(P_avxx/24)];
end



for i = 1: length(list_MAE)
    fprintf(1,'The MAE of  (%1.3s) +  is about %1.3f',days(i),list_MAE(i));
    disp(' ')
end
