% Open File
clc
clear all
close all

dxl_constant = 180/501932;
formatSpec = '%d %d %d %d %d %d %d %d %d %d %d %d %d';
sizeA = [13 inf];
fileID = fopen('Data_Log_fix.txt','r');
A = fscanf(fileID, formatSpec, sizeA);
data = A.';
%% Find the end of recording point
i = max(find(data(:,1)>0));

for j = 1:6
err(1:i,j) = (data(1:i, 1+j) - data(1:i, 7+j))*dxl_constant;
end
t = data(1:i,1).*0.001;
%% Plot the Data
figure (1)


h(1) = subplot(421);
plot(t,data(1:i,2)*dxl_constant,'k.','MarkerSize',0.5);
hold on;
plot(t,data(1:i,8)*dxl_constant,'.','MarkerSize',0.5);
axis([0 t(end) -90 90])
grid minor;

h(2) = subplot(422);
plot(t,data(1:i,3)*dxl_constant,'k.','MarkerSize',0.5);
hold on;
plot(t,data(1:i,9)*dxl_constant,'.','MarkerSize',0.5);
axis([0 t(end) -90 90])
grid minor;

h(3) = subplot(423);
plot(t,data(1:i,4)*dxl_constant,'k.','MarkerSize',0.5);
hold on;
plot(t,data(1:i,10)*dxl_constant,'.','MarkerSize',0.5);
axis([0 t(end) -90 90])
grid minor;

h(4) = subplot(424);
plot(t,data(1:i,5)*dxl_constant,'k.','MarkerSize',0.5);
hold on;
plot(t,data(1:i,11)*dxl_constant,'.','MarkerSize',0.5);
axis([0 t(end) -90 90])
grid minor;

h(5) = subplot(425);
plot(t,data(1:i,6)*dxl_constant,'k.','MarkerSize',0.5);
hold on;
plot(t,data(1:i,12)*dxl_constant,'.','MarkerSize',0.5);
grid minor;
axis([0 t(end) -90 90])

h(6) = subplot(426);
plot(t,data(1:i,7)*dxl_constant,'k.','MarkerSize',0.5);
hold on;
plot(t,data(1:i,13)*dxl_constant,'.','MarkerSize',0.5);
grid minor;
axis([0 t(end) -90 90])

h(7) = subplot(427);
pos = get(h,'Position');
new = mean(cellfun(@(v)v(1),pos(1:2)));
set(h(7),'Position',[new,pos{end}(2:end)]+[-0.2, 0, 0.4, 0])
for j = 1:6
plot(t,err(1:i,j));
hold on;
end

axis([0 t(end) -5 5])