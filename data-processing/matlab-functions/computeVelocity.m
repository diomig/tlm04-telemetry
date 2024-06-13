function computeVelocity(controller_data, gps_data)

twos2dec = @(x, b) x-(x>=2^(b-1))*2^b;

conversion = 2*pi/60*0.188*0.3*3.6;

rpm = controller_data(controller_data(:,1)==15, 2);
rpm = twos2dec(rpm,16);
timer5 = controller_data(controller_data(:,1)==15, 6)/1000;

% figure('Name', 'rpm','NumberTitle','off');
hFig = figure( 501 ); set( hFig, 'Name', 'velocity','NumberTitle','off');
clf
title('Velocidade: Controlador VS GPS');
hold on
xlabel('Tempo [m:s]'); ylabel('Velocity [km/h]')
timerCont = seconds(timer5); timerCont.Format = 'mm:ss';
plot(timerCont, rpm*conversion, 'red');
timerGPS = seconds(gps_data.timer/1000); timerGPS.Format = 'mm:ss';
plot(timerGPS, gps_data.speed, 'green'); 

legend('Controlador','GPS');
hold off


end