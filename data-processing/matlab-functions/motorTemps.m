function motorTemps(controllerData)

motorTemp = controllerData(controllerData(:,1)==13, 4);
timer3 = controllerData(controllerData(:,1)==13, 6)/1000;
timer3 = timer3-timer3(1);
hFig = figure( 401 ); set( hFig, 'Name', 'M&C Temp','NumberTitle','off');
title('Tempertaturas do Motor nas Diferentes saídas');
xlabel('Tempo [s]');
ylabel('Temperatura [ºC]');
% xlim([timer3(1) timer3(end)]);
% ALTERAR FORMATO DO TEMPO PARA mm:ss
timer3 = seconds(timer3); timer3.Format = 'mm:ss';

hold on
plot(timer3, motorTemp);
hold off

legend('1ª Saída', '2ª Saída', '3ª Saída', '4ª Saída');

end