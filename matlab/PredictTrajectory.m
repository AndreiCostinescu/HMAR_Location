function PredictTrajectory( res_dir_name, out_file_name, print_flag )

%% Dictionary of actions

file_name = 'action_label.txt';
fileID = fopen(file_name);
data = textscan(fileID,'%s');
fclose(fileID);
DICT = containers.Map;
for i=1:length(data{1})
  DICT(data{1}{i}) = i;
end

DICT2 = DICT;
remove(DICT2,'MOVE');
remove(DICT2,'RELEASE');
DICT3 = containers.Map(values(DICT2),keys(DICT2));

%% Action sequence

file_name = 'label.txt';
fileID = fopen(file_name);
data = textscan(fileID,'%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s','Delimiter',',');
ALS = data{1};
for i=2:length(data)
    ALS = [ALS data{i}];
end
%% Confusion matrix


% dir_name = '../Result_Staticface';
% dir_name = '../Result_Moveface';
% dir_name = '../Result_ObjectState';

dir_name = res_dir_name;

PROB_ALL = cell(4,2); c=1;

% Experiments
DIR = dir(dir_name);
for i=1:length(DIR)
    
    PROB = cell(49,7);
    WINDOW = cell(49,7);
    
%     if (~strcmp(DIR(i).name,'SPG'))
%         continue;
%     end
    
    if (RootDirRemoval(DIR(i).name))
        continue;
    end
    
    % Subject left out in cross validation
    file_name = [dir_name '/' DIR(i).name];
    DIR_S = dir(file_name);
    for ii=1:length(DIR_S)

        if (RootDirRemoval(DIR_S(ii).name))
            continue;
        end
                
%         if (strcmp(DIR_S(ii).name,'1'))
%             continue;
%         end
%         
%         if (strcmp(DIR_S(ii).name,'2'))
%             continue;
%         end
%         
%         if (strcmp(DIR_S(ii).name,'3'))
%             continue;
%         end
        
        % # action sequence
        file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name];
        DIR_SAS = dir(file_name);
        for iii=1:length(DIR_SAS)
            
            if (RootDirRemoval(DIR_SAS(iii).name))
                continue;
            end
            
            if (strcmp(DIR_SAS(iii).name,'ParsedResult'))
                continue;
            end
            
            % Files of subject
            file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name '/' DIR_SAS(iii).name];
            DIR_F = dir(file_name);
            for iv=1:length(DIR_F)

                if (RootDirRemoval(DIR_F(iv).name) || strcmp(DIR_F(iv).name(1),'_'))
                    continue;
                end
                
%                 if (~strcmp(DIR_F(iv).name,'170419135613.txt'))
%                     continue;
%                 end
                
                file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name '/' DIR_SAS(iii).name '/' DIR_F(iv).name];
                fileID = fopen(file_name);
                data = textscan(fileID,'%s%s%f%f%f%f%f%f%f%f%f%f%f%f%f%f','Delimiter',',');
                fclose(fileID);
                
                % seq from label
                als = ALS(str2double(DIR_SAS(iii).name),2:end);  
                cc = length(als);
                for v=1:length(als)
                    if (isempty(als{v}))
                        cc = v-1;
                        break;
                    end
                end
                                
                seq_idx = zeros(100,1)-1;
                
                % seq from data
                als_tmp = cell(100,1); 
                als_tmp{1} = data{2}{1};
                c = 2;
                for v=2:length(data{2})
                    if(~strcmp(data{2}{v},data{2}{v-1}))
                        als_tmp{c} = data{2}{v};
                        seq_idx(c) = v;
                        c = c + 1;
                    end
                end   
                                
                for vv=1:2
                    c2 = 3;
                    seq_idx2 = zeros(100,1)-1;
                    seq_idx2(1) = seq_idx(1);
                    seq_idx2(2) = seq_idx(2);
                    als_tmp2 = cell(100,1);
                    als_tmp2{1} = als_tmp{1};
                    als_tmp2{2} = als_tmp{2};
                    if strcmp(file_name,'../Result/CUP/1/001/170419114241.txt')
                        nth=0;
                    end
                    for v=3:length(als_tmp(1:c-1))
                        seq_idx2(c2) = seq_idx(v);
                        als_tmp2{c2} = als_tmp{v};
                        c2 = c2 + 1;
                        if(strcmp(als_tmp{v},'RELEASE') && strcmp(als_tmp{v-2},'RELEASE'))
                            c2 = c2 - 1;
                            als_tmp2{c2} = [];
                            seq_idx(c2) = -1;
                            c2 = c2 - 1;
                            als_tmp2{c2} = [];
                            seq_idx(c2) = -1;
                        end
                    end  

                    c = 3;
                    seq_idx = zeros(100,1)-1;
                    seq_idx(1) = seq_idx2(1);
                    seq_idx(2) = seq_idx2(2);
                    als_tmp = cell(100,1);
                    als_tmp{1} = als_tmp2{1};
                    als_tmp{2} = als_tmp2{2};
                    if strcmp(file_name,'../Result/CUP/1/001/170419114241.txt')
                        nth=0;
                    end
                    for v=3:length(als_tmp2(1:c2-1))
                        seq_idx(c) = seq_idx2(v);
                        als_tmp{c} = als_tmp2{v};
                        c = c + 1;
                        if(~strcmp(als_tmp2{v},'RELEASE') && ~strcmp(als_tmp2{v-1},'RELEASE') && ~strcmp(als_tmp2{v},'MOVE') && strcmp(als_tmp2{v},als_tmp2{v-2}))
                            c = c - 1;
                            als_tmp{c} = [];
                            seq_idx(c) = -1;
                            c = c - 1;
                            als_tmp{c} = [];
                            seq_idx(c) = -1;
                        end
                    end  
                end
                
                if strcmp(file_name,'../Result/SPG/1/004/170419120357.txt')
                    nth=0;
                end
                              
                % compare between predict and label sequence
                for v=1:cc
                    if(~strcmp(als{v},als_tmp{v}))
                        disp(file_name);
                        disp([als_tmp(1:cc) als(1:cc)']);
                        break;
                    end
                end
                
                for v=2:cc-1
                    if(strcmp(als{v},'MOVE'))
                        for vv=1:7
                            PROB{(DICT(als{v-1})-1)*7+DICT(als{v+1}),vv} = ...
                                [ PROB{(DICT(als{v-1})-1)*7+DICT(als{v+1}),vv};
                                  {data{vv+2}(seq_idx(v):seq_idx(v+1)-1)} ];
                            WINDOW{(DICT(als{v-1})-1)*7+DICT(als{v+1}),vv} = ...
                                [ WINDOW{(DICT(als{v-1})-1)*7+DICT(als{v+1}),vv};
                                  {data{vv+2+7}(seq_idx(v):seq_idx(v+1)-1)} ];
                        end
                    end
                end                   
            end
        end
    end   
        
    PROB_ALL{c,1} = PROB;
    PROB_ALL{c,2} = DIR(i).name;
    c = c + 1;
      
    for ii=1:49

        if (isempty(PROB{ii}))
            continue;
        end

        figure(ii);
        
        c = 1;
        
        flag = ones(7,1);
        plot_handles = [];
        close_flag = 0;

        for iii = 1:7
            for iv = 1:length(PROB{ii,iii})

                PROB{ii,iii}{iv} = [linspace(0,100,length(PROB{ii,iii}{iv}))',PROB{ii,iii}{iv}];
                close_flag = close_flag + (sum(PROB{ii,iii}{iv}(:,2))==0);
                switch(iii)
                    case 1
                        if (mean(PROB{ii,iii}{iv}(:,2)==0)~=1)
                            h = plot(PROB{ii,iii}{iv}(:,1),PROB{ii,iii}{iv}(:,2),'color',[1,0.65,0]);
                            if (flag(iii))
                                plot_handles = [plot_handles h];
                                legend_name{c} = DICT3(iii);
                                c = c + 1;
                                flag(iii) = 0;
                            end
                            hold on;
                        end
                    case 2
                        if (mean(PROB{ii,iii}{iv}(:,2)==0)~=1)
                            h = plot(PROB{ii,iii}{iv}(:,1),PROB{ii,iii}{iv}(:,2),'m');
                            if (flag(iii))
                                plot_handles = [plot_handles h];
                                legend_name{c} = DICT3(iii);
                                c = c + 1;
                                flag(iii) = 0;
                            end
                            hold on;
                        end
                    case 3
                        if (mean(PROB{ii,iii}{iv}(:,2)==0)~=1)
                            h = plot(PROB{ii,iii}{iv}(:,1),PROB{ii,iii}{iv}(:,2),'c');
                            if (flag(iii))
                                plot_handles = [plot_handles h];
                                legend_name{c} = DICT3(iii);
                                c = c + 1;
                                flag(iii) = 0;
                            end
                            hold on;
                        end
                    case 4
                        if (mean(PROB{ii,iii}{iv}(:,2)==0)~=1)
                            h = plot(PROB{ii,iii}{iv}(:,1),PROB{ii,iii}{iv}(:,2),'r');
                            if (flag(iii))
                                plot_handles = [plot_handles h];
                                legend_name{c} = DICT3(iii);
                                c = c + 1;
                                flag(iii) = 0;
                            end
                            hold on;
                        end
                    case 5
                        if (mean(PROB{ii,iii}{iv}(:,2)==0)~=1)
                            h = plot(PROB{ii,iii}{iv}(:,1),PROB{ii,iii}{iv}(:,2),'g');
                            if (flag(iii))
                                plot_handles = [plot_handles h];
                                legend_name{c} = DICT3(iii);
                                c = c + 1;
                                flag(iii) = 0;
                            end
                            hold on;
                        end
                    case 6
                        if (mean(PROB{ii,iii}{iv}(:,2)==0)~=1)
                            h = plot(PROB{ii,iii}{iv}(:,1),PROB{ii,iii}{iv}(:,2),'b');
                            if (flag(iii))
                                plot_handles = [plot_handles h];
                                legend_name{c} = DICT3(iii);
                                c = c + 1;
                                flag(iii) = 0;
                            end
                            hold on;
                        end
                    case 7
                        if (mean(PROB{ii,iii}{iv}(:,2)==0)~=1)
                            h = plot(PROB{ii,iii}{iv}(:,1),PROB{ii,iii}{iv}(:,2),'k');
                            if (flag(iii))
                                plot_handles = [plot_handles h];
                                legend_name{c} = DICT3(iii);
                                c = c + 1;
                                flag(iii) = 0;
                            end
                            hold on;
                        end
                end
            end
        end
        
        axis([0,100,0,1]);
        xlabel('Trajectory [%]');
        ylabel('Confidence');
        
        hold off;

        if(close_flag/length(PROB{ii,iii})==6)
            close(figure(ii));
        else
            legend(plot_handles, legend_name');
%             title(['Trajectory Prediction ( ' DICT3(ceil(ii/7)) ' to ' DICT3(mod(ii-1,7)+1) ')']);
            title(['Trajectory Prediction (Goal : ' DICT3(mod(ii-1,7)+1) ')']);
            if(print_flag)
                print(figure(ii),[out_file_name '_' DIR(i).name '_' num2str(ii)],'-depsc');
            end;
        end
        

    end
    
    close all; 
    
end
end
