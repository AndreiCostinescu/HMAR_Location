clean;

dir_name = '../Result2';
file_name = '';

output = cell(1);
outputName = cell(1);
c = 1;
% Experiments
DIR = dir(dir_name);
for i=1:length(DIR)
    
    PROB = cell(49,7);
    WINDOW = cell(49,7);
    
%     if (~strcmp(DIR(i).name,'KNF'))
%         continue;
%     end
    
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
                
        if (strcmp(DIR_S(ii).name,'2'))
            continue;
        end
        
        if (strcmp(DIR_S(ii).name,'1'))
            continue;
        end
        
        if (strcmp(DIR_S(ii).name,'3'))
            continue;
        end
        
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
                
%                 tmp = data{1}(data{3}(100:end)+data{4}(100:end)==6);
                tmp1 = data{1}(data{3}==3);
                tmp2 = data{4}(data{3}==3);
                tmp = tmp1(tmp2==3);

                if(~isempty(tmp))
                    tmp = [(1:round(length(tmp)*0.05))+tmp(1)-(round(length(tmp)*0.05)+1) tmp' (1:round(length(tmp)*0.05))+tmp(end)];
                    output{c} = {   1:length(tmp), ...
                                    data{2}(tmp)', ...
                                    data{5}(tmp)', ...
                                    data{7}(tmp)'};
                    outputName{c} = DIR(i).name;
                    c = c+1;
                end
                
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
                                
%                 plot(data{1},data{2},data{1},data{3},data{1},data{4},data{1},data{5},data{1},data{7});
%                 plot(   1:length(data{2}(data{3}+data{4}==8)),data{2}(data{3}+data{4}==8)./10,'b', ...
%                         1:length(data{5}(data{3}+data{4}==8)),data{5}(data{3}+data{4}==8),'r', ...
%                         1:length(data{7}(data{3}+data{4}==8)),data{7}(data{3}+data{4}==8),'g');
%                 ylim([0 0.1]);
%                 hold off;
                
            end
        end
    end
end

c = 1;
for i=1:length(output)-1
    A = output{i}{2};
    A(A(:)>1)=1;
    figure(c)
    h = plot(linspace(-5,105,output{i}{1}(end)),A*0.1,'b',...
        linspace(-5,105,output{i}{1}(end)),output{i}{4},'r',...
        linspace(-5,105,output{i}{1}(end)),output{i}{3}*(10/30),'g');
    xlim([-5 105]);
    ylim([0 0.11]);
    if (~strcmp(outputName(i),outputName(i+1)))
        c = c + 1;
        legend('Contact_{Hand-Obj}','Distance_{Surface}','Speed');
        title([{'Surface Constrained Motion'};{'(Location Area: Table1)'}]);
        xlabel('Duration in location area [%]');
        ylabel('[Grasp] , [m] , [10e-1 m/s]');
        print(figure(ceil(i/3)),['Constraint/Surface_T1_' outputName{i}],'-depsc','-loose');
    end
        
    hold on;
end

legend(h, {'Contact_{Hand-Obj}','Distance_{Surface}','Speed'});
title([{'Surface Constrained Motion'};{'(Location Area: Table1)'}]);
xlabel('Duration in location area [%]');    
ylabel('[Grasp] , [m] , [10e-1 m/s]');
print(figure(c),['Constraint/Surface_T1_' outputName{i}],'-depsc','-loose');
hold off;
