function computeBatVolt(controller_data)
twos2dec = @(x, b) x-(x>=2^(b-1))*2^b;
    
batVolt = controller_data(controller_data(:,1)==13, 5);
batVolt = twos2dec(batVolt,16)*0.0625;
timer = controller_data(controller_data(:,1)==13, 6)/1000;

timer = seconds(timer); timer.Format = 'mm:ss'

hFig = figure( 602 ); set( hFig, 'Name', 'BatVolt','NumberTitle','off');
plot(timer, batVolt, 'g');
title('Tensão Total na Baterias');
xlabel('Tempo[mm:ss]');
ylabel('Tensão [V]');
xlim([timer(1) timer(end)]); ylim([0 130]);
 
end