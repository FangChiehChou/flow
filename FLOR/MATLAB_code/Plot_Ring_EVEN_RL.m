%%
clear all
close all 
clc

%% Simulation configuration
Sim_length = 2000;
ring_len = 260.0;
controller_active_time = 300;
Sim_Runs = 10;
total_veh_on_ring = 22;
edgestarts_tag  = {':bottom_0','bottom',':right_0','right',':top_0','top',':left_0','left'};
edgestarts_pos = [0,0,ring_len / 4,ring_len / 4,ring_len / 2,ring_len / 2,3 * ring_len / 4,3 * ring_len / 4];

%%
var = {'NOx','CO','CO2','Fuel','Velocity','STD Velocity','av std vel','hv std vel'};
mark ={'-x','-s','-o','-^','-v','-*'};

%%

%av_case = {'AUG','BCM','FS','FUZN','FUZO','LACC','LinOpt','MLYAU1','MLYAU2','PI'};  %Aug collision/ BCM collision
av_case = {'RL'};
av_num_case = 2:1:11;
hv = 'IDM';
FLOW_Sim_Results_path ='/media/lorr/TOSHIBA EXT/lorr_sim_out/Even';

Sim_AV_data=cell(length(av_case)*length(av_num_case),1);
AV_data_index = 0;

AV_stabilize_time = zeros(length(av_case),length(av_num_case),Sim_Runs);
AV_stabilize_speed = zeros(length(av_case),length(av_num_case),Sim_Runs);
AV_max_gap = zeros(length(av_case),length(av_num_case),Sim_Runs);
AV_total_fuel = zeros(length(av_case),length(av_num_case),Sim_Runs);
AV_VMT = zeros(length(av_case),length(av_num_case),Sim_Runs);

%%
for av_case_index = 1:1:length(av_case)
%Iterate through each AV type

    av = av_case{av_case_index};
    eval(['Sim_AV_data_',av,'=cell(22,1);'])

    for av_num = 2:1:11  %iterate through experiments of different number of AVs on the ring
        disp([av,':',num2str(av_num)])
        
        AV_data_index = AV_data_index+1;
        hv_num = total_veh_on_ring-av_num;
        path_scenario = FLOW_Sim_Results_path;  %%directory for simulation results

        Matlab_results_dir = [FLOW_Sim_Results_path,['/LORProcResults/',hv,'_',av]];   %%Making directory for saving figures from MATLAB
        mkdir(Matlab_results_dir)

        experiment_file = [hv,num2str(hv_num),'_',av,num2str(av_num)];
        path_exp = [path_scenario,'/',experiment_file];
        files = dir(strcat(path_exp,'/*.csv'));

        index_run=0;
        
        for file = files'   %check all the excel file in this expereiment and compute each vehicle's speed cycle
        
            index_run = index_run+1
            veh_data = cell(total_veh_on_ring,1);  %totally 22 vehicles on the ring
            veh_id = [];  %Vehicle type in the simulation
            t = readtable([path_exp,'/',file.name]);
 
            time_exist_flag = ~isempty(find(strcmp(t.Properties.VariableNames,'time')==1));   %==1 if this does exist, ==0 otherwise
            speed_exist_flag = ~isempty(find(strcmp(t.Properties.VariableNames,'speed')==1));
            id_exist_flag = ~isempty(find(strcmp(t.Properties.VariableNames,'id')==1));
            position_exist_flag = ~isempty(find(strcmp(t.Properties.VariableNames,'relative_position')==1));
            edge_id_exist_flag = ~isempty(find(strcmp(t.Properties.VariableNames,'edge_id')==1));
            
            if(time_exist_flag==0 || speed_exist_flag==0)  
                continue;
            end
            if(id_exist_flag==1)    %There exits a variable name 'id' in the table
                %% data from the table
                id = t.id;
                speed = t.speed;
                time = t.time;
                raw_pos = t.relative_position;
                fuel =  t.fuel;   %%   ml/s
                edge_id = t.edge_id;
                
                for j = 1:1:length(id)    %iterate through each row in the table
                    temp_id = id{j};

                    if(isempty(veh_id))  %initialize the very first data read from the table
                        veh_id = [veh_id,string(temp_id)];
                        veh_data{1}.time = [];
                        veh_data{1}.speed = [];
                        veh_data{1}.pos = [];
                        veh_data{1}.fuel = [];

                        veh_data{1}.time = [veh_data{1}.time;time(j)];
                        veh_data{1}.speed = [veh_data{1}.speed;speed(j)];   
                        veh_data{1}.fuel = [veh_data{1}.fuel;fuel(j)];
                        edge_index = find(strcmp(edgestarts_tag,edge_id(j))==1);
                        veh_data{1}.pos = [veh_data{1}.pos;raw_pos(j)+edgestarts_pos(edge_index)];  
                        continue; 
                    end

                    if(isempty(find(strcmp(veh_id,temp_id)==1, 1))) %new ID found from the data
                        %initialize the space for saving the data we are interested in.
                        veh_id = [veh_id,string(temp_id)];
                        veh_data{length(veh_id)}.time = [];
                        veh_data{length(veh_id)}.speed = [];
                        veh_data{length(veh_id)}.pos = [];
                        veh_data{length(veh_id)}.fuel = [];

                        veh_data{length(veh_id)}.time = [veh_data{length(veh_id)}.time;time(j)];
                        veh_data{length(veh_id)}.speed = [veh_data{length(veh_id)}.speed;speed(j)];
                        edge_index = find(strcmp(edgestarts_tag,edge_id(j))==1);
                        veh_data{length(veh_id)}.pos = [veh_data{length(veh_id)}.pos;raw_pos(j)+edgestarts_pos(edge_index)]; 
                        veh_data{length(veh_id)}.fuel = [veh_data{length(veh_id)}.fuel;fuel(j)];
                    else    
                        index = find(strcmp(veh_id,temp_id)==1); 
                        veh_data{index}.time = [veh_data{index}.time;time(j)];
                        veh_data{index}.speed = [veh_data{index}.speed;speed(j)];
                        edge_index = find(strcmp(edgestarts_tag,edge_id(j))==1);
                        veh_data{index}.pos = [veh_data{index}.pos;raw_pos(j)+edgestarts_pos(edge_index)]; 
                        veh_data{index}.fuel = [veh_data{index}.fuel;fuel(j)];
                    end
                end

            end
            
            %% Compute the vehicle headways

            TimeCode = 0;
            Initial_position_vehs = zeros(length(veh_data),1);
            for j = 1:1:length(veh_data) %iterate thorugh each vehicle.
                [~,temp_time_index] = min(abs(veh_data{j}.time-TimeCode));
                Initial_position_vehs(j) = veh_data{j}.pos(temp_time_index(1));
            end    
            [~,sortIdx] = sort(Initial_position_vehs);

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
                temp_headway(temp_headway<0) = temp_headway(temp_headway<0) + ring_len;
                if(temp_len< temp_len_s)
                    %current data length is longther than length_headway
                    %pad null to the length of the data to the end
                    veh_data{veh_s_id}.headway = nan*ones(temp_len_s,1);
                    veh_data{veh_s_id}.headway(1:length(temp_headway)) = temp_headway;
                else
                    veh_data{veh_s_id}.headway = temp_headway;
                end
            end 

            %% Resample all the speed of vehicles and the headway of the vehicles.    
            standard_time_steps = 0:0.1:Sim_length;
            Speed = [];
            Headway = [];
            for k = 1:1:total_veh_on_ring  %iterate throught each vehicles 
                spd_at_std_time = interp1(veh_data{k}.time,veh_data{k}.speed,standard_time_steps);
                Speed = [Speed;spd_at_std_time];
                headway_at_std_time = interp1(veh_data{k}.time,veh_data{k}.headway,standard_time_steps); %   vq = interp1(x,v,xq)
                Headway = [Headway;headway_at_std_time];
            end
            
            %% get the total fuel conumption and vehicle VMT
            total_fuel = 0;
            total_VMT = 0;
            for i = 1:1:total_veh_on_ring
                vehicle_fuel_profile = veh_data{i}.fuel;
                vehicle_position = veh_data{i}.pos;

                for j = 1:1:length(vehicle_fuel_profile)
                    total_fuel = total_fuel + vehicle_fuel_profile(j);
                end

                vehicle_position_diff = vehicle_position(2:end) - vehicle_position(1:end-1);
                vehicle_position_reset = find(vehicle_position_diff<0);
                num_loops = length(vehicle_position_reset)-1;

                VMT = (ring_len-vehicle_position(1)) + ring_len*num_loops + vehicle_position(end);
                total_VMT = total_VMT + VMT;
            end            
            
        
            %% Find average and standard deviation across vehicles at each time stamp and the maximum gap at the end. 
            total_headway = sum(Headway,1);
            max_total_headway = max(total_headway);
            min_total_headway = min(total_headway);
            mean_speed = mean(Speed,1);
            std_speed = std(Speed,0,1);
            I = find(std_speed <= 0.1 & standard_time_steps>=controller_active_time, 1);

            if(isempty(I) || (max_total_headway-min_total_headway)>1 )  %if this scenario is not stabilized (or collision happens) at all.
                stabilizing_time = nan;
                stable_speed = nan;
                total_fuel = nan;
            else
                stabilizing_time = standard_time_steps(I)-controller_active_time;
                stable_speed = mean(mean_speed(I:end));
                total_fuel = total_fuel/1000; %%convert the unit ml -> l
            end
            [maximum_gap,I] = max(Headway(:,end));


            AV_stabilize_time(av_case_index,av_num,index_run) = stabilizing_time; 
            AV_stabilize_speed(av_case_index,av_num,index_run)  = stable_speed;  
            AV_max_gap(av_case_index,av_num,index_run)  = maximum_gap; 
            AV_total_fuel(av_case_index,av_num,index_run) = total_fuel;
            AV_VMT(av_case_index,av_num,index_run) = total_VMT;

        end

        %%Plot and save for the last run of each experiment
        plot_spd_headway;

    end
end


%% Saving Processed Data of this experiment.
%% Save 
save('RL_EVEN_results.mat','AV_stabilize_time','AV_stabilize_speed','AV_max_gap','AV_total_fuel','AV_VMT')



%%
% mark ={'-x','-s','-o','-^','-v','-*','-p','-h','-d','-<','->'};
% figure()
% hold on
% for av_case_index = 1:1:length(av_case)
%     plot(1:1:22,AV_stabilize_time(av_case_index,:),mark{av_case_index},'LineWidth',2);
% end
% xlabel('NumAV','FontSize',30)
% ylabel('Time to Stable[s]','FontSize',30)
% set(gca,'FontSize',30)
% legend('AUG','BCM','FS','FUZN','FUZO','LACC','LinOpt','MLYAU1','MLYAU2','PI') 
% filename = ['StableTime.png'];
% saveas(gcf,['/home/lorr/flow/FLOR/MATLAB_code/MATLABLORSim1029/',filename]);
% close all   
% 
% 
% figure()
% hold on
% for av_case_index = 1:1:length(av_case)
%     plot(1:1:22,AV_max_gap(av_case_index,:),mark{av_case_index},'LineWidth',2);
% end
% xlabel('NumAV','FontSize',30)
% ylabel('Max Gap[m]','FontSize',30)
% set(gca,'FontSize',30)
% legend('AUG','BCM','FS','FUZN','FUZO','LACC','LinOpt','MLYAU1','MLYAU2','PI') 
% filename = ['MaxGap.png'];
% saveas(gcf,['/home/lorr/flow/FLOR/MATLAB_code/MATLABLORSim1029/',filename]);
% close all   
% 
% 
% figure()
% hold on
% for av_case_index = 1:1:length(av_case)
%     plot(1:1:22,AV_stabilize_speed(av_case_index,:),mark{av_case_index},'LineWidth',2);
% end
% xlabel('NumAV','FontSize',30)
% ylabel('Stable Speed[m/s]','FontSize',30)
% set(gca,'FontSize',30)
% legend('AUG','BCM','FS','FUZN','FUZO','LACC','LinOpt','MLYAU1','MLYAU2','PI') 
% filename = ['StableSpeed.png'];
% saveas(gcf,['/home/lorr/flow/FLOR/MATLAB_code/MATLABLORSim1029/',filename]);
% close all





















