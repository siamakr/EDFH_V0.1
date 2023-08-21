clc; 
close;
clear all;

 P = './';
 S = dir(fullfile(P,'sr_0*')); 


 for k = 1:numel(S)
    F = fullfile(P,S(k).name);
    S(k).data = csvread(F);
    xangle = S(k).data(:,1);
    yangle = S(k).data(:,3);
    xsp = S(k).data(:,2);
    ysp = S(k).data(:,4);
    %%add more matricies for other datapoints
    figure(k), plot((1:numel(xangle)), xangle);
    hold on
    figure(k), plot((1:numel(yangle)), yangle);
    figure(k), plot((1:numel(xsp)), xsp);
    figure(k), plot((1:numel(ysp)), ysp);

end