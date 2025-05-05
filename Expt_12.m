clc;  
clear all;  
close all;  

Scrambler_input=[80 255 16 9 48 255 80 0 25 0 145]
s=20255;%initialization of scrambler circuit
rand_data=zeros(size(Scrambler_input));
for j=1:size(Scrambler_input,2);
    for i=1:8
        msb=bitxor(bitget(s,1),bitget(s,2));
        s=bitshift(s,-1);
        s=bitset(s,15,msb);
        t=bitxor(bitget(Scrambler_input(j),9-i),msb);
        rand_data(j)=bitset(rand_data(j),9-i,t);
    end
end
Scrambler_out=rand_data
