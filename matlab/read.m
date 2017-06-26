clean;

file_name = '/home/chen/Desktop/example.txt';
fileID = fopen(file_name);
data = textscan(fileID,'%f%f%f%f%f%f%f%f%f%f%f%f');
fclose(fileID);

all = [data{1} data{2} data{3}];

figure;
plot3(all(:,1),all(:,2),all(:,3),'o-');
grid on;
axis([0 1 -1 0 1 2]);

for i=1:size(data{1},1)
    rvec(i,:) = rotm2eul([data{4}(i) data{5}(i) data{6}(i); data{7}(i) data{8}(i) data{9}(i); data{10}(i) data{11}(i) data{12}(i)]);
    rvec(i,:) = rvec(i,:)*180/pi;
end

figure;
plot(1:length(rvec(:,1)),rvec(:,1),1:length(rvec(:,1)),rvec(:,2),1:length(rvec(:,1)),rvec(:,3));