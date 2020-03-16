%%
clear all
close all 
clc

ring_len = 260;
edgestarts_tag  = {':bottom_0','bottom',':right_0','right',':top_0','top',':left_0','left'};
edgestarts_pos = [0,0,ring_len / 4,ring_len / 4,ring_len / 2,ring_len / 2,3 * ring_len / 4,3 * ring_len / 4];

% scenario  = {'IDM_BCM','IDM_FS','IDM_LAC','IDM_PI','IDM_PNS','OVM_BCM','OVM_FS','OVM_LAC','OVM_PI','OVM_PNS'};
% scenario  = {'IDM_AugmentedOVFTL','IDM_FuzzyNew','IDM_FuzzyOld','IDM_ModLyapunov1','IDM_ModLyapunov2'};

var = {'NOx','CO','CO2','Fuel','Velocity','STD Velocity','av std vel','hv std vel'};
mark ={'-x','-s','-o','-^','-v','-*'};

%%

av_case = {'AUG','BCM','FS','FUZN','FUZO','LACC','LinOpt','MLYAU1','MLYAU2','PI'};  %Aug collision/ BCM collision

hv = 'IDM';
FLOW_Sim_Results_path ='/home/lorr/flow/FLOR/';

Sim_AV_data=cell(length(av_case)*22,1);
AV_data_index = 0;

AV_stabilize_time = zeros(length(av_case),22);
AV_stabilize_speed = zeros(length(av_case),22);
AV_max_gap = zeros(length(av_case),22);

%%
for av_case_index = 1:1:length(av_case)
%Iterate through each AV type

    av = av_case{av_case_index};
    eval(['Sim_AV_data_',av,'=cell(22,1);'])

    for av_num = 1:1:22  %iterate through experiments of different number of AVs on the ring
        AV_data_index = AV_data_index+1;
        hv_num = 22-av_num;
        path_scenario = [FLOW_Sim_Results_path,'Sim1028_',hv,'_AVRider','_',av];  %%directory for simulation results

        Matlab_results_dir = [pwd,['/MATLABLORSim1029/',hv,'_',av,'/',]];   %%Making directory for saving figures from MATLAB
        mkdir(Matlab_results_dir)

        experiment_file = [num2str(hv_num),hv,'_',num2str(av_num),av];
        path_exp = [path_scenario,'/',experiment_file];
        files = dir(strcat(path_exp,'/*.csv'));

        veh_data = cell(22,1);  %totally 22 vehicles on the ring
        veh_id = [];  %Vehicle type in the simulation

        for file = files'   %check all the excel file in this expereiment and compute each vehicle's speed cycle
        
            t = readtable([path_exp,'/',file.name]);
 
            time_exist_flag = ~isempty(find(strcmp(t.Properties.VariableNames,'time')==1));   %==1 if this does exist, ==0 otherwise
            speed_exist_flag = ~isempty(find(strcmp(t.Properties.VariableNames,'speed')==1));
            id_exist_flag = ~isempty(find(strcmp(t.Properties.VariableNames,'id')==1));
            position_exist_flag = ~isempty(find(strcmp(t.Properties.VariableNames,'relative_position')==1));
            edge_id_exit_flag = ~isempty(find(strcmp(t.Properties.VariableNames,'edge_id')==1));

            if(~isempty(veh_id))  %if the vehicle type is not empty means that one of the excel has been vistied
            %there is no need to visit the other csv file
                continue;
            end
            if(time_exist_flag==0 || speed_exist_flag==0)  
                continue;
            end
            if(id_exist_flag==1)    %There exits a variable name 'id' in the table
                id = t.id;
                speed = t.speed;
                time = t.time;
                raw_pos = t.relative_position;
                edge_id = t.edge_id;
                for j = 1:1:length(id)    %iterate through each row in the table
                    temp_id = id{j};

                    if(isempty(veh_id))
                        veh_id = [veh_id,string(temp_id)];
                        veh_data{1}.time = [];
                        veh_data{1}.speed = [];
                        veh_data{1}.pos = [];
                        veh_data{1}.time = [veh_data{1}.time;time(j)];
                        veh_data{1}.speed = [veh_data{1}.speed;speed(j)];                          
                        index = find(strcmp(edgestarts_tag,edge_id(j))==1);
                        veh_data{1}.pos = [veh_data{1}.pos;raw_pos(j)+edgestarts_pos(index)];  
                        continue; 
                    end

                    if(isempty(find(strcmp(veh_id,temp_id)==1, 1))) %new id found from the data
                        veh_id = [veh_id,string(temp_id)];
                        veh_data{length(veh_id)}.time = [];
                        veh_data{length(veh_id)}.speed = [];
                        veh_data{length(veh_id)}.pos = [];
                        veh_data{length(veh_id)}.time = [veh_data{length(veh_id)}.time;time(j)];
                        veh_data{length(veh_id)}.speed = [veh_data{length(veh_id)}.speed;speed(j)];
                        edge_index = find(strcmp(edgestarts_tag,edge_id(j))==1);
                        veh_data{length(veh_id)}.pos = [veh_data{length(veh_id)}.pos;raw_pos(j)+edgestarts_pos(edge_index)]; 
                    else    
                        index = find(strcmp(veh_id,temp_id)==1); 
                        veh_data{index}.time = [veh_data{index}.time;time(j)];
                        veh_data{index}.speed = [veh_data{index}.speed;speed(j)];
                        edge_index = find(strcmp(edgestarts_tag,edge_id(j))==1);
                        veh_data{index}.pos = [veh_data{index}.pos;raw_pos(j)+edgestarts_pos(edge_index)]; 
                    end
                end
            end
        end
            

        
%% Compute the vehicle headways
        TimeCode = 0;
        position_vehs = zeros(length(veh_data),1);
        for j = 1:1:length(veh_data) %iterate thorugh each vehicle.
            [~,temp_time_index] = min(abs(veh_data{j}.time-TimeCode));
            position_vehs(j) = veh_data{j}.pos(temp_time_index(1));
        end    
        [~,sortIdx] = sort(position_vehs);

        for j = 1:1:length(sortIdx)
               
            if(j ==length(sortIdx))  %last vehicle in the string, its preceding vehicle is vehicle 1.
                veh_s_id = sortIdx(j);
                veh_t_id = sortIdx(1);
            else
                veh_s_id = sortIdx(j);
                veh_t_id = sortIdx(j+1);
            end
            tmep_len_t = length(veh_data{veh_t_id}.pos);
            temp_len_s = length(veh_data{veh_s_id}.pos);
            [temp_len,~] = min([tmep_len_t temp_len_s]);
            temp_headway = veh_data{veh_t_id}.pos(1:temp_len) - veh_data{veh_s_id}.pos(1:temp_len);
            temp_headway(temp_headway<0) = temp_headway(temp_headway<0) + 260.0;
            if(temp_len< temp_len_s)
                %current data length is longther than length_headway
                %pad null to the length of to the end
                veh_data{veh_s_id}.headway = nan*ones(temp_len_s,1);
                veh_data{veh_s_id}.headway(1:length(temp_headway)) = temp_headway;
            else
                veh_data{veh_s_id}.headway = temp_headway;
            end
        end 

%% Resample all the speed of vehicles and the headway of the vehicles.    
        standard_time_steps = 0:0.1:2000;
        Speed = [];
        Headway = [];
        for k = 1:1:22  %iterate throught each vehicles 
            spd_at_std_time = interp1(veh_data{k}.time,veh_data{k}.speed,standard_time_steps);
            Speed = [Speed;spd_at_std_time];
            headway_at_std_time = interp1(veh_data{k}.time,veh_data{k}.headway,standard_time_steps);
        %     vq = interp1(x,v,xq)
            Headway = [Headway;headway_at_std_time];
        end    

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
        plot(300*ones(length(temp_line),1),temp_line,'LineWidth',2,'Color',[0 0 0]);
        xlabel('Time[s]','FontSize',30)
        ylabel('Speed[m/s]','FontSize',30)
        set(gca,'FontSize',30)
        xlim([0 2000])
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
        plot(300*ones(length(temp_line),1),temp_line,'LineWidth',2,'Color',[0 0 0]);
        xlabel('Time[s]','FontSize',30)
        ylabel('Headway[m]','FontSize',30)
        set(gca,'FontSize',30)
        ylim([0 max_gap])
        xlim([0 2000])
        filename = [hv,num2str(hv_num),'_',av,num2str(av_num),'_Headway.png'];
        saveas(gcf,[Matlab_results_dir,filename]);
        close all   

%% Find average and standard deviation across vehicles at each time stamp and the maximum gap at the end. 
        mean_speed = mean(Speed,1);
        std_speed = std(Speed,0,1);
        I = find(std_speed <= 0.1 & standard_time_steps>=300, 1);
        if(isempty(I))  %if this scenario is not stabilized at all.
            stabilizing_time = nan;
            stable_speed = nan;
        else
            stabilizing_time = standard_time_steps(I)-300.0;
            stable_speed = mean(mean_speed(I:end));
        end
        [maximum_gap,I] = max(Headway(:,end));


%% Saving Processed Data of this experiment.
        av
        av_num
        Sim_AV_data{AV_data_index}.AVtype = av;
        Sim_AV_data{AV_data_index}.AVnum = av_num;
        Sim_AV_data{AV_data_index}.Std_TimeStamp = standard_time_steps;
        Sim_AV_data{AV_data_index}.Speed = Speed;
        Sim_AV_data{AV_data_index}.Headway = Headway;

        Sim_AV_data{AV_data_index}.AvgSpeed  = mean_speed;  % estimate the mean speed of all the vehicles at each time step
        Sim_AV_data{AV_data_index}.StdSpeed  = std_speed;  % Compute the standard deviation of the vehicle speeds at each time step.

        Sim_AV_data{AV_data_index}.veh_id = veh_id;
        Sim_AV_data{AV_data_index}.data = veh_data;  

        Sim_AV_data{AV_data_index}.stabilizing_time = stabilizing_time;
        Sim_AV_data{AV_data_index}.stable_speed = stable_speed;
        Sim_AV_data{AV_data_index}.maximum_gap = maximum_gap;

        eval(['Sim_AV_data_',av,'{',num2str(av_num),'} = Sim_AV_data{',num2str(AV_data_index),'};'])

        AV_stabilize_time(av_case_index,av_num) = stabilizing_time;  %zeros(length(av_case),22);
        AV_stabilize_speed(av_case_index,av_num)  = stable_speed;  
        AV_max_gap(av_case_index,av_num)  = maximum_gap; 
    end

    eval(['save(''temp1029_',av,'.mat'',''Sim_AV_data_',av,''');'])

end


%% Plot the stabilizing time /stabilizing speed /maximum gap of each scenario
mark ={'-x','-s','-o','-^','-v','-*','-p','-h','-d','-<','->'};
figure()
hold on
for av_case_index = 1:1:length(av_case)
    plot(1:1:22,AV_stabilize_time(av_case_index,:),mark{av_case_index},'LineWidth',2);
end
xlabel('NumAV','FontSize',30)
ylabel('Time to Stable[s]','FontSize',30)
set(gca,'FontSize',30)
legend('AUG','BCM','FS','FUZN','FUZO','LACC','LinOpt','MLYAU1','MLYAU2','PI') 
filename = ['StableTime.png'];
saveas(gcf,['/home/lorr/flow/FLOR/MATLAB_code/MATLABLORSim1029/',filename]);
close all   


figure()
hold on
for av_case_index = 1:1:length(av_case)
    plot(1:1:22,AV_max_gap(av_case_index,:),mark{av_case_index},'LineWidth',2);
end
xlabel('NumAV','FontSize',30)
ylabel('Max Gap[m]','FontSize',30)
set(gca,'FontSize',30)
legend('AUG','BCM','FS','FUZN','FUZO','LACC','LinOpt','MLYAU1','MLYAU2','PI') 
filename = ['MaxGap.png'];
saveas(gcf,['/home/lorr/flow/FLOR/MATLAB_code/MATLABLORSim1029/',filename]);
close all   


figure()
hold on
for av_case_index = 1:1:length(av_case)
    plot(1:1:22,AV_stabilize_speed(av_case_index,:),mark{av_case_index},'LineWidth',2);
end
xlabel('NumAV','FontSize',30)
ylabel('Stable Speed[m/s]','FontSize',30)
set(gca,'FontSize',30)
legend('AUG','BCM','FS','FUZN','FUZO','LACC','LinOpt','MLYAU1','MLYAU2','PI') 
filename = ['StableSpeed.png'];
saveas(gcf,['/home/lorr/flow/FLOR/MATLAB_code/MATLABLORSim1029/',filename]);
close all  


%% Save 
save('Temp_1029.mat','AV_stabilize_time','AV_stabilize_speed','AV_max_gap')


















