clc;
clear all;
close all;

% Input values
n = input('Enter n value for N-bit PCM system: ');
n1 = input('Enter number of samples in period: ');

L = 2^n;
x = 0:2*pi/n1:4*pi;
s = 8 * sin(x);

% Plot the Analog Signal
subplot(3,1,1);
plot(s);
title('Analog Signal');
xlabel('Time --->');
ylabel('Amplitude --->');

% Sampling Process
subplot(3,1,2);
stem(s);
grid on;
title('Sampled Signal');
xlabel('Time --->');
ylabel('Amplitude --->');

% Quantization Process  
Vmax = 8;  
clc;  
clear all;  
close all;  

% Input values  
n = input('Enter n value for N-bit PCM system: ');  
n1 = input('Enter number of samples in period: '); 
L = 2^n;  
x = 0:2*pi/n1:4*pi;  
s = 8 * sin(x);  

% Plot the Analog Signal  
subplot(3,1,1);  
plot(s);  
title('Analog Signal');  
xlabel('Time --->');  
ylabel('Amplitude --->');  

% Sampling Process  
subplot(3,1,2);  
stem(s);  
grid on;  
title('Sampled Signal');  
xlabel('Time --->');  
ylabel('Amplitude --->');  

% Quantization Process  
Vmax = 8;  
Vmin = -Vmax;  
del = (Vmax - Vmin) / L;  
part = Vmin:del:Vmax;  
code = Vmin - (del/2):del:Vmax + (del/2);  

[ind, q] = quantiz(s, part, code);  

l1 = length(ind);  
l2 = length(q);  

for i = 1:l1  
    if ind(i) ~= 0  
        ind(i) = ind(i) - 1;  
    end  
end  

for i = 1:l2  
    if q(i) == Vmin - (del/2)  
        q(i) = Vmin + (del/2);  
    end  
end  

% Plot the Quantized Signal  
subplot(3,1,3);  
stem(q);  
grid on;  
title('Quantized Signal');  
xlabel('Time --->');  
ylabel('Amplitude --->');  

% Encoding Process  
Vmin = -Vmax;  
del = (Vmax - Vmin) / L;  
part = Vmin:del:Vmax;  
code = Vmin - (del/2):del:Vmax + (del/2);  

[ind, q] = quantiz(s, part, code);  

l1 = length(ind);  
l2 = length(q);  

for i = 1:l1  
    if ind(i) ~= 0  
        ind(i) = ind(i) - 1;  
    end  
end  

for i = 1:l2  
    if q(i) == Vmin - (del/2)  
        q(i) = Vmin + (del/2);  
    end  
end  
for i = 1:l2  
    if q(i) == Vmin - (del/2)  
        q(i) = Vmin + (del/2);  
    end  
end  

% Plot the Quantized Signal  
subplot(3,1,3);  
stem(q);  
grid on;  
title('Quantized Signal');  
xlabel('Time --->');  
ylabel('Amplitude --->');  

% Encoding Process  
figure;  
code = de2bi(ind, 'left-msb');  

coded = [];  
for i = 1:l1  
    for j = 1:n  
        coded = [coded, code(i, j)];  
    end  
end  

% Plot the Encoded Signal  
subplot(2,1,1);  
grid on;  
stairs(coded);  
axis([0 100 -2 3]);  
title('Encoded Signal');  
xlabel('Time --->');  
ylabel('Amplitude --->');  

% Demodulation of PCM Signal  
qunt = reshape(coded, n, length(coded) / n);  
index = bi2de(qunt', 'left-msb');  
q = del * index + Vmin + (del/2);  

% Plot the Demodulated Signal  
subplot(2,1,2);  
grid on;  
plot(q);  
title('Demodulated Signal');  
xlabel('Time --->');  
ylabel('Amplitude --->');
figure;  
code = de2bi(ind, 'left-msb');  

coded = [];  
for i = 1:l1  
    for j = 1:n  
        coded = [coded, code(i, j)];  
    end  
end  

% Plot the Encoded Signal  
subplot(2,1,1);  
grid on;  
stairs(coded);  
axis([0 100 -2 3]);  
title('Encoded Signal');  
xlabel('Time --->');  
ylabel('Amplitude --->');  
% Demodulation of PCM Signal  
qunt = reshape(coded, n, length(coded) / n);  
index = bi2de(qunt', 'left-msb');  
q = del * index + Vmin + (del/2);  

% Plot the Demodulated Signal  
subplot(2,1,2);  
grid on;  
plot(q);  
title('Demodulated Signal');  
xlabel('Time --->');  
ylabel('Amplitude --->');
