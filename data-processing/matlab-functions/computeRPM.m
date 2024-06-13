function computeRPM(controller_data)
twos2dec = @(x, b) x-(x>=2^(b-1))*2^b;

rpm = controller_data(controller_data(:,1)==15, 2);
rpm = twos2dec(rpm,16);
throttle = controller_data(controller_data(:,1)==15, 5)* 3.0518509e-3;
timer5 = controller_data(controller_data(:,1)==15, 6)/1000;

%timer5 = seconds(timer5); timer5.Format = 'mm:ss';

% figure('Name', 'rpm','NumberTitle','off');
hFig = figure( 301 ); set( hFig, 'Name', 'rpm','NumberTitle','off');
title('Velocidade em resposta ao acelerador');
hold on
xlabel('Tempo[s]');
yyaxis left; ylim([-20 6000]);
ylabel('RPM');
plot(timer5, rpm);
yyaxis right; ylim([-5 105]);
ylabel('Throttle [%]');
plot(timer5, throttle); 
xlim([timer5(1) timer5(end)]);
hold off
 
end