    function computeMotorCurrents(controller_data)
twos2dec = @(x, b) x-(x>=2^(b-1))*2^b;
    
currents = controller_data(controller_data(:,1)==11,2:5);
currents = twos2dec(currents,16)*0.0625;
timer = controller_data(controller_data(:,1)==11, 6)/1000;

hFig = figure( 604 ); set( hFig, 'Name', 'BatVolt','NumberTitle','off');
clf;

tim = seconds(timer); tim.Format = 'mm:ss';

hold on;
plot(tim, currents);
hold off;

title('Correntes no Motor');
xlabel('Tempo[mm:ss]');
ylabel('Corrente [A]');
xlim([tim(1) tim(end)]); 

legend('Target Id', 'Target Iq', 'Id', 'Iq')
end