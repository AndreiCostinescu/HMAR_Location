clean;

%% Dictionary of actions

file_name = 'action_label.txt';
fileID = fopen(file_name);
data = textscan(fileID,'%s');
fclose(fileID);
DICT = containers.Map;
AL{length(data{1}),1} = '';
for i=1:length(data{1})
  DICT(data{1}{i}) = i;
  AL{i,1} = data{1}{i};
end

%% Confusion matrix

conf_mat = zeros(DICT.Count);
res_t = zeros(DICT.Count,100000);
res_y = zeros(DICT.Count,100000);
count = 0;

dir_name = '../Result';
file_name = '';

% Experiments
DIR = dir(dir_name);
for i=1:length(DIR)
    
    if (strcmp(DIR(i).name,'.') || strcmp(DIR(i).name,'..'))
        continue;
    end
    
%     if (~strcmp(DIR(i).name,'CUP'))
%         continue;
%     end
    
    % Subject left out in cross validation
    file_name = [dir_name '/' DIR(i).name];
    DIR_S = dir(file_name);
    for ii=1:length(DIR_S)

        if (strcmp(DIR_S(ii).name,'.') || strcmp(DIR_S(ii).name,'..'))
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

                if (strcmp(DIR_F(iv).name,'.') || strcmp(DIR_F(iv).name,'..') || strcmp(DIR_F(iv).name(1),'_'))
                    continue;
                end

                file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name '/' DIR_SAS(iii).name '/' DIR_F(iv).name];
                fileID = fopen(file_name);
                data = textscan(fileID,['%s%s' repmat('%f', 1, 14)],'Delimiter',',');
                fclose(fileID);

                for iv=1:length(data{1})
                    conf_mat(DICT(data{1}{iv}),DICT(data{2}{iv})) = ...
                        conf_mat(DICT(data{1}{iv}),DICT(data{2}{iv})) + 1;
                end   

                for v=1:length(data{1})
                    res_t(DICT(data{1}{v}), count+v) = 1;
                    res_y(DICT(data{2}{v}), count+v) = 1;
                end 
                count = count + length(data{1});

            end
        end
    end

%     % col right PPV = TP/(TP+FP)
%     % row btm   TPR = TP/(TP+FN)
%     figure(1);
%     plotconfusion(res_t(:,1:count),res_y(:,1:count));
%     
%     %title,axes
%     set(findobj(gcf,'type','figure'),'position',[66 1 1855 1031]);
%     set(get(findobj(gcf,'type','axes'),'title'),'string',['Confusion Matrix for "' DIR(i).name '"']);
%     set(get(findobj(gcf,'type','axes'),'xlabel'),'string','Target');
%     set(get(findobj(gcf,'type','axes'),'ylabel'),'string','Predict');
%     set(get(findobj(gcf,'type','axes'),'title'),'fontsize',14);
%     set(get(findobj(gcf,'type','axes'),'xlabel'),'fontsize',14);
%     set(get(findobj(gcf,'type','axes'),'ylabel'),'fontsize',14);
%     set(findobj(gcf,'type','axes'),'xticklabel',[AL;' ']);
%     set(findobj(gcf,'type','axes'),'xticklabelrotation',90);
%     set(findobj(gcf,'type','axes'),'yticklabel',[AL;' ']);
%     %font
%     set(findobj(gcf,'type','text'),'fontsize',12);
%     %line
%     set(findobj(gcf,'type','line'),'linewidth',3);
%     set(findobj(gcf,'type','line'),'color',[30,30,30]./255);
%     %colors
%     blue  = [25  25  122 ]/255;
%     white = [255 255 255 ]/255;
%     green = [10  180 10  ]/255;
%     red   = [200 0   0   ]/255;
%     set(findobj(gcf,'color',[0,102,0]./255),'color',green)
%     set(findobj(gcf,'color',[102,0,0]./255),'color',red)
%     set(findobj(gcf,'facecolor',[120,230,180]./255),'facecolor',green)
%     set(findobj(gcf,'facecolor',[230,140,140]./255),'facecolor',red)
%     set(findobj(gcf,'facecolor',[0.5,0.5,0.5]),'facecolor',white)
%     set(findobj(gcf,'facecolor',[120,150,230]./255),'facecolor',blue)
% 
%     res_t = zeros(DICT.Count,100000);
%     res_y = zeros(DICT.Count,100000);
%     count = 0;
    
end

normA = conf_mat ./ max(conf_mat(:));
imshow(normA, 'InitialMagnification',10000);  % # you want your cells to be larger than single pixels
colormap(jet) % # to change the default grayscale colormap

% % col right PPV = TP/(TP+FP)
% % row btm   TPR = TP/(TP+FN)
% figure(1);
% fig = plotconfusion(res_t(:,1:count),res_y(:,1:count));
% 
% %title,axes
% set(findobj(gcf,'type','figure'),'position',[66 1 1855 1031]);
% set(get(findobj(gcf,'type','axes'),'title'),'string','Confusion Matrix');
% set(get(findobj(gcf,'type','axes'),'xlabel'),'string','Target');
% set(get(findobj(gcf,'type','axes'),'ylabel'),'string','Predict');
% set(get(findobj(gcf,'type','axes'),'title'),'fontsize',14);
% set(get(findobj(gcf,'type','axes'),'xlabel'),'fontsize',14);
% set(get(findobj(gcf,'type','axes'),'ylabel'),'fontsize',14);
% set(findobj(gcf,'type','axes'),'xticklabel',[AL;' ']);
% set(findobj(gcf,'type','axes'),'xticklabelrotation',90);
% set(findobj(gcf,'type','axes'),'yticklabel',[AL;' ']);
% %font
% set(findobj(gcf,'type','text'),'fontsize',12);
% %line
% set(findobj(gcf,'type','line'),'linewidth',3);
% set(findobj(gcf,'type','line'),'color',[30,30,30]./255);
% %colors
% blue  = [25  25  122 ]/255;
% white = [255 255 255 ]/255;
% green = [10  180 10  ]/255;
% red   = [200 0   0   ]/255;
% set(findobj(gcf,'color',[0,102,0]./255),'color',green)
% set(findobj(gcf,'color',[102,0,0]./255),'color',red)
% set(findobj(gcf,'facecolor',[120,230,180]./255),'facecolor',green)
% set(findobj(gcf,'facecolor',[230,140,140]./255),'facecolor',red)
% set(findobj(gcf,'facecolor',[0.5,0.5,0.5]),'facecolor',white)
% set(findobj(gcf,'facecolor',[120,150,230]./255),'facecolor',blue)
% 
% print(fig,'ConfusionMatrix','-depsc');
% 
% res_t = zeros(DICT.Count,100000);
% res_y = zeros(DICT.Count,100000);
% count = 0;

