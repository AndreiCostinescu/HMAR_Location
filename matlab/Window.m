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
DICT2 = DICT;
remove(DICT2,'MOVE');
remove(DICT2,'RELEASE');
remove(DICT2,'STOP');
DICT3 = containers.Map(values(DICT2),keys(DICT2));

%% Window

dir_name = '../Scene2';
file_name = '';



% Objects
DIR = dir(dir_name);
for i=1:length(DIR)

    if (strcmp(DIR(i).name,'.') || strcmp(DIR(i).name,'..'))
        continue;
    end
    
    if (~strcmp(DIR(i).name,'PT1'))
        continue;
    end
    
    X = cell(10,1);
    Y = cell(10,1);
    Y2 = cell(10,1);
    LA = cell(10,1);
    
    % Subjects
    file_name = [dir_name '/' DIR(i).name];
    DIR_S = dir(file_name);
    c = 1;
    nn = 0;
    for ii=1:length(DIR_S)

        if (strcmp(DIR_S(ii).name,'.') || strcmp(DIR_S(ii).name,'..'))
            continue;
        end
        
        file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name '/location_area.txt'];
        fileID = fopen(file_name);
        data = textscan(fileID,[repmat('%s', 1, 13)],'Delimiter',',');
        fclose(fileID);
        LA{c} = data{1};
        
        file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name '/window.txt'];
        fileID = fopen(file_name);
        data = textscan(fileID,[repmat('%s', 1, 100)],'Delimiter',',');
        fclose(fileID);
        
        nn = str2num(data{2}{1});
        
        X{c} = zeros((length(data{2})-3)/2,length(data));
        Y{c} = X{c}; 
        for iii=4:length(data{2})
            if(mod(iii,2)==0) 
                continue;
            end
            for iv=1:length(data)
                X{c}(ceil(iii/2)-2,iv) = iv;
                Y{c}(ceil(iii/2)-2,iv) = str2num(data{iv}{iii});
            end
        end
        
        file_name = [dir_name '/' DIR(i).name '/' DIR_S(ii).name '/window2.txt'];
        fileID = fopen(file_name);
        data = textscan(fileID,[repmat('%s', 1, 100)],'Delimiter',',');
        fclose(fileID);
        
        Y2{c} = X{c}; 
        for iii=4:length(data{2})
            if(mod(iii,2)==0) 
                continue;
            end
            for iv=1:length(data)
                Y2{c}(ceil(iii/2)-2,iv) = str2num(data{iv}{iii});
            end
        end     
        
        c = c + 1;
        
    end   
    
    for ii=1:length(X)
        if (isempty(X{ii}))
            continue;
        end
        figure; L = [];
        for iii = 1:size(X{ii},1)
            if (sum(Y{ii}(iii,:))==0)
                continue;
            end
            windowSize = 3; 
            b = (1/windowSize)*ones(1,windowSize);
            a = 1;
            y  = filter(b,a,Y {ii}(iii,:));
            y2 = filter(b,a,Y2{ii}(iii,:));
            y = smooth(Y{ii}(iii,:));
            y2 = smooth(Y2{ii}(iii,:));
%             plot(X{ii}(iii,:),y2./y); 
%             y2 = Y2{ii}(iii,:);
            plot(X{ii}(iii,:),y2); 
            L = [L ; iii];
            hold on;
        end
        
        LL = [];
        for iii=1:length(L)
            LL = [LL; {[LA{ii}{ceil(L(iii)/nn)} ' to ' LA{ii}{mod(L(iii)-1,nn)+1}]}];
        end
        legend(LL);
        axis([0 100 0 0.2]);
        hold off;
        title(['Subject ' num2str(ii)]);
        ylabel('Variation [m]');
        xlabel('Trajectory of sector-map [sector]');
        print(gcf,['Window_' DIR(i).name '_' num2str(ii)],'-depsc');
    end
    
%     for ii=1:length(X)
%         if (isempty(X{ii}))
%             continue;
%         end
%         figure; L = [];
%         for iii = 1:size(X{ii},1)
%             if (sum(Y2{ii}(iii,:))==0)
%                 continue;
%             end
%             plot(X{ii}(iii,:),Y2{ii}(iii,:)); 
%             L = [L ; iii];
%             hold on;
%         end
%         
%         LL = [];
%         for iii=1:length(L)
%             LL = [LL; {[LA{ii}{ceil(L(iii)/nn)} ' to ' LA{ii}{mod(L(iii)-1,nn)+1}]}];
%         end
%         legend(LL);
%         hold off;
%     end
    
%     close all;
    
    
    
end

