clean;

%% Dictionary of actions

file_name = 'action_label.txt';
fileID = fopen(file_name);
data = textscan(fileID,'%s');
fclose(fileID);
DICT = containers.Map;
for i=1:length(data{1})
  DICT(data{1}{i}) = i;
end
AL = zeros(length(data{1}),1);

%% Action sequence

file_name = 'label.txt';
fileID = fopen(file_name);
data = textscan(fileID,'%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s','Delimiter',',');
ALS = data{1};
for i=2:length(data)
    ALS = [ALS data{i}];
end
%% Confusion matrix

dir_name = '../recording';
file_name = '';

% Sub
DIR = dir(dir_name);
for i=1:length(DIR)
    
    if (strcmp(DIR(i).name,'.') || strcmp(DIR(i).name,'..'))
        continue;
    end
        
    if (length(DIR(i).name)<6)
        continue;
    end
        
    if (~DIR(i).isdir)
        continue;
    end
    
    % Exp
    file_name = [dir_name '/' DIR(i).name];
    DIR_S = dir(file_name);
    for ii=1:length(DIR_S)

        if (strcmp(DIR_S(ii).name,'.') || strcmp(DIR_S(ii).name,'..'))
            continue;
        end

        if (~DIR_S(ii).isdir)
            continue;
        end

        % Files of subject
        file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name];
        DIR_F = dir(file_name);
        for iv=1:length(DIR_F)

            if (strcmp(DIR_F(iv).name,'.') || strcmp(DIR_F(iv).name,'..'))
                continue;
            end
            
            if (strcmp(DIR_F(iv).name,'extra'))
                continue;
            end
            
            file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name '/' DIR_F(iv).name];
            fileID = fopen(file_name);
            data = textscan(fileID,'%f%f%f%f%f%f%f%f%s','Delimiter',',');
            fclose(fileID);

            als_tmp = cell(100,1); % seq from data
            als_tmp{1} = data{9}{1};
            c = 2;
            for v=2:length(data{9})
                if(~strcmp(data{9}{v},data{9}{v-1}))
                    als_tmp{c} = data{9}{v};
                    c = c + 1;
                end
            end   

            n = str2double(DIR_S(ii).name);
            als = ALS(n,2:end);  % seq from label
            cc = length(als);
            for v=1:length(als)
                if (isempty(als{v}))
                    cc = v-1;
                    break;
                end
            end
            for v=1:cc
                if(~strcmp(als{v},als_tmp{v}))
                    disp(file_name);
                    disp([als_tmp(1:cc) als(1:cc)']);
                    break;
                end
            end



%                 n = str2double(DIR_SAS(iii).name);
%                 als = ALS(n,2:end);  % seq from label
%                 cc = 1;
%                 for v=1:length(als)
%                     flag = false;
%                     tmp = als{v};
%                     if(~strcmp(tmp,als_tmp{cc}))
%                         while(1)                        
%                             if(strcmp(tmp,'RELEASE'))
%                                 tmp = als{v-1};
%                                 if(v>2 && strcmp(tmp,als_tmp{cc+1}))
%                                     AL(DICT(als_tmp{cc+1})) = AL(DICT(als_tmp{cc+1})) + 1;
%                                     cc = cc + 1;
%                                     break;
%                                 else
%                                     cc = cc + 1;
%                                 end
%                             else
%                                 if(v>2 && strcmp(als{v-2},als_tmp{cc}))
%                                     AL(DICT(als_tmp{cc})) = AL(DICT(als_tmp{cc})) + 1;
%                                     cc = cc + 1;
%                                     break;
%                                 else
%                                     cc = cc + 1;
%                                 end
%                             end
%                         end
%                     else
%                     	 cc = cc + 1;
%                     end
%                 end    
        end
    end   
end

