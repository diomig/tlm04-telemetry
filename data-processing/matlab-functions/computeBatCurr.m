function computeBatCurr(controller_data)
    
batCurr = controller_data(controller_data(:,1)==14, 4);
batCurr = batCurr * (0.0625/8);
timer = controller_data(controller_data(:,1)==14, 6)/1000;

hFig = figure( 601 ); set( hFig, 'Name', 'BatCurr','NumberTitle','off');
plot(timer, batCurr);
title('Corrente nas Baterias');
xlabel('Tempo[s]');
ylabel('Corrente [A]');

xlim([timer(1) timer(end)]);
 
end