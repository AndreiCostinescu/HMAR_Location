clean;

dir_name = '../Result';
file_name = '';
% Experiments
DIR = dir(dir_name);
for i=1:length(DIR)
    
    PROB = cell(49,7);
    WINDOW = cell(49,7);
    
    if (~strcmp(DIR(i).name,'KNF'))
        continue;
    end
    
    if (strcmp(DIR(i).name,'.') || strcmp(DIR(i).name,'..'))
        continue;
    end
    
    % Subject left out in cross validation
    file_name = [dir_name '/' DIR(i).name];
    DIR_S = dir(file_name);
    for ii=1:length(DIR_S)

        if (strcmp(DIR_S(ii).name,'.') || strcmp(DIR_S(ii).name,'..'))
            continue;
        end
                
%         if (strcmp(DIR_S(ii).name,'2'))
%             continue;
%         end
%         
%         if (strcmp(DIR_S(ii).name,'1'))
%             continue;
%         end
%         
%         if (strcmp(DIR_S(ii).name,'0'))
%             continue;
%         end
        
        % # action sequence
        file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name];
        DIR_SAS = dir(file_name);
        for iii=1:length(DIR_SAS)
            
            if (strcmp(DIR_SAS(iii).name,'.') || strcmp(DIR_SAS(iii).name,'..'))
                continue;
            end
            
            % Files of subject
            file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name '/' DIR_SAS(iii).name];
            DIR_F = dir(file_name);
            for iv=1:length(DIR_F)

                if (strcmp(DIR_F(iv).name,'.') || strcmp(DIR_F(iv).name,'..'))
                    continue;
                end
                
                if (~strcmp(DIR_F(iv).name(1),'_'))
                    continue;
                end
                
                file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name '/' DIR_SAS(iii).name '/' DIR_F(iv).name];
                fileID = fopen(file_name);
                data = textscan(fileID,'%f%f%f%f%f%f%f%f%f%f','Delimiter',',');
                fclose(fileID);
                
%                1 tmp.push_back(i);
%                2 tmp.push_back(Graph_->GetActionState().grasp);
%                3 tmp.push_back(Graph_->GetActionState().label1);
%                4 tmp.push_back(Graph_->GetActionState().label2);
%                5 tmp.push_back(Graph_->GetActionState().mov);
%                6 tmp.push_back(Graph_->GetActionState().sur);
%                7 tmp.push_back(Graph_->GetActionState().sur_dist);
%                8 tmp.push_back(pva_avg[i][0].x);
%                9 tmp.push_back(pva_avg[i][0].y);
%               10 tmp.push_back(pva_avg[i][0].z);
                                
                plot(data{1},data{2},data{1},data{3},data{1},data{4},data{1},data{5},data{1},data{7});
%                 plot(   1:length(data{2}(data{3}+data{4}==8)),data{2}(data{3}+data{4}==8)./10, ...
%                         1:length(data{5}(data{3}+data{4}==8)),data{5}(data{3}+data{4}==8), ...
%                         1:length(data{7}(data{3}+data{4}==8)),data{7}(data{3}+data{4}==8));
%                 ylim([0 0.3]);
                hold off;
                
                end
        end
    end
end
