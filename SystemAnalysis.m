dataFile = fopen('PendulumFreeSpinData.txt', 'r');
data = fscanf(dataFile, '%f')';

data += 180 * ones(size(data));
data *= pi/180;

t = [0:0.05:(length(data)-1)*0.05];

Fs = 20; #samples per second
dt = 1/Fs; #seconds per sample
X = fftshift(fft(data));
N = size(t,2);

dF = Fs/N; %hertz
f = -Fs/2:dF:Fs/2-dF;

#plot(f, abs(X)/N)
plot(t,data)

#frequency = 0.959 Hz
