clc; 
close;
clear vars;

 P = './';
 S = dir(fullfile(P,'sr_0*')); 


 for k = 1:numel(S)
    F = fullfile(P,S(k).name);
    S(k).data = csvread(F);
    xangle = S(k).data(:,1);
    yangle = S(k).data(:,3);
    xsp = S(k).data(:,2);
    ysp = S(k).data(:,4);
    altitude_sp = S(k).data(:,17);
    altitude = S(k).data(:,16);
    thrust = S(k).data(:,21);
    vz = S(k).data(:, 13);

    %%add more matricies for other datapoints
%     figure(k), plot((1:numel(xangle)), xangle);
%     hold on;
%     figure(k), plot((1:numel(yangle)), yangle);
%     figure(k), plot((1:numel(xsp)), xsp);
%     figure(k), plot((1:numel(ysp)), ysp);
    yyaxis left;
    figure(k), plot((1:numel(thrust)), thrust);

    hold on;
    yyaxis right;
    figure(k), plot((1:numel(altitude_sp)), altitude_sp);
    hold on;
    figure(k), plot((1:numel(altitude)), altitude);


    

 end

  yyaxis left;
 figure(10), plot((1:numel(vz)),vz);

 hold on;
 figure(10), plot(1:numel(thrust), (thrust./12) - 2.5);
 yyaxis right;