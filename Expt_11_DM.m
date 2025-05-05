clc;  
clear all;  
close all;  

predictor = [0 1];
step = 0.2;
partition = [0];codebook = [-1*step step];
t =[0:pi/20:2*pi];
x = 1.1*sin(2*pi*0.1*t);
encodedx = dpcmenco(x,codebook,partition,predictor);
decodedx = dpcmdeco(encodedx,codebook,predictor);
distor = sum((x-decodedx).^2)/length(x)
figure,subplot(2,2,1);
plot(t,x);
xlabel('time');
title('original signal');
subplot(2,2,2);
stairs(t,10*codebook(encodedx+1),'--');
xlabel('time');
title('DM output');
subplot(2,2,3);
plot(t,x);
hold;
stairs(t,decodedx);
grid;
xlabel('time');
title('received signal');
