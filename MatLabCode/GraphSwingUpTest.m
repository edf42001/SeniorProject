data = dlmread('SwingupTestData2ndMethod.txt', ' ');

t = 0:0.01:(size(data,1)-1) * 0.01;

figure
plotyy(t, data(:,1)+pi, t, data(:,2));
