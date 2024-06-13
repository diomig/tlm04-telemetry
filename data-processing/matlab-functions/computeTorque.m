function computeTorque(controller_data)
twos2dec = @(x, b) x-(x>=2^(b-1))*2^b;
    
Torque = controller_data(controller_data(:,1)==14, 5);
Torque = twos2dec(Torque,16)*16;
timer = controller_data(controller_data(:,1)==14, 6)/1000;

hFig = figure( 603 ); set( hFig, 'Name', 'Torque','NumberTitle','off');
plot(timer, Torque, 'k');
title('Torque');
xlabel('Tempo[s]');
ylabel('Torque [N.m]');
end