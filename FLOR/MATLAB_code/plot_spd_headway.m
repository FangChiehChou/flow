%% Plot vehicle Speed
figure()
for j = 1:1:length(veh_data)
    if(contains(veh_id(j),'IDM'))   %If this vehicle is IDM vehicle, then plot this way.
        plot(veh_data{j}.time,veh_data{j}.speed,'Color',[0, 0.4470, 0.7410])
    else
        plot(veh_data{j}.time,veh_data{j}.speed,'Color',[0.8500, 0.3250, 0.0980])
    end
    hold on
end
temp_line = 0:1:11;
plot(controller_active_time*ones(length(temp_line),1),temp_line,'LineWidth',2,'Color',[0 0 0]);   %%Line for indicating controller started
xlabel('Time[s]','FontSize',30)
ylabel('Speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
xlim([0 Sim_length])
filename = [hv,num2str(hv_num),'_',av,num2str(av_num),'.png'];
saveas(gcf,[Matlab_results_dir,filename]);
close all

%% Plot vehicle headways
figure()
for j = 1:1:length(veh_data)
    if(contains(veh_id(j),'IDM'))   %If this vehicle is IDM vehicle, then plot this way.
        plot(veh_data{j}.time,veh_data{j}.headway,'Color',[0, 0.4470, 0.7410])
    else
        plot(veh_data{j}.time,veh_data{j}.headway,'Color',[0.8500, 0.3250, 0.0980])
    end
    hold on
end
[max_gap, ~] =  max(Headway);
[max_gap, ~] = max(max_gap);
temp_line = 0:1:max_gap;
plot(controller_active_time*ones(length(temp_line),1),temp_line,'LineWidth',2,'Color',[0 0 0]); %%Line for indicating controller started
xlabel('Time[s]','FontSize',30)
ylabel('Headway[m]','FontSize',30)
set(gca,'FontSize',30)
ylim([0 max_gap])
xlim([0 Sim_length])
filename = [hv,num2str(hv_num),'_',av,num2str(av_num),'_Headway.png'];
saveas(gcf,[Matlab_results_dir,filename]);
close all   


%% Plot total headways
figure()
plot(standard_time_steps,total_headway)
xlabel('Time[s]','FontSize',30)
ylabel('Total Headway[m]','FontSize',30)
set(gca,'FontSize',30)
xlim([0 Sim_length])
filename = [hv,num2str(hv_num),'_',av,num2str(av_num),'_TotalHeadway.png'];
saveas(gcf,[Matlab_results_dir,filename]);
close all

%% Plot abs position of one vehicle
figure()
plot(veh_data{1}.time,veh_data{1}.pos,'Color',[0, 0.4470, 0.7410])    
xlabel('Time[s]','FontSize',30)
ylabel('RelPos[m]','FontSize',30)
set(gca,'FontSize',30)
xlim([0 Sim_length])
filename = [hv,num2str(hv_num),'_',av,num2str(av_num),'_RelPos.png'];
saveas(gcf,[Matlab_results_dir,filename]);
close all 