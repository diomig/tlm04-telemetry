function computeMotorTemp(controllerData, gpsData)

%% TEMPERATURAS DO MOTOR E DO CONTROLADOR EM FUNÇÃO DO TEMPO
motorTemp = controllerData(controllerData(:,1)==13, 4);
timer3 = controllerData(controllerData(:,1)==13, 6)/1000;
controllerTemp = controllerData(controllerData(:,1)==14, 3);
controllerTemp = mod(controllerTemp, 256);
timer4 = controllerData(controllerData(:,1)==14, 6)/1000;


aux = find((motorTemp - [0; motorTemp(1 : end-1)])<-10);
motorTemp(aux:aux+20) = 50;

aux = find((motorTemp - [0; motorTemp(1 : end-1)])<-10);
motorTemp(aux:aux+15) = 50;

hFig = figure( 401 ); set( hFig, 'Name', 'M&C Temp','NumberTitle','off');
clf;
title('Tempertaturas do Motor e do Controlador');
xlabel('Tempo [s]');
ylabel('Temperatura [ºC]');
xlim([timer3(1) timer3(end)]);
% ALTERAR FORMATO DO TEMPO PARA mm:ss
 timer3 = seconds(timer3); timer3.Format = 'mm:ss';
 timer4 = seconds(timer4); timer4.Format = 'mm:ss';

hold on
plot(timer3, motorTemp);
plot(timer4, controllerTemp);
hold off

legend('Temp. Motor', 'Temp. Controlador');


%%      TEMPERATURA DO MOTOR AO LONGO DA PISTA
%_______________________________________________________
mtemp=NaN;
for n = 1:length(gpsData.timer)
    aux = controllerData(abs(controllerData(:,6)-gpsData.timer(n))<500,:);
    t = aux(aux(:,1)==13, 4);
    if length(t)~=0
        mtemp = [mtemp; t(1)];
        if abs(mtemp(end)-mtemp(end-1))>10
            mtemp(end) = mtemp(end-1);
        end
    else
        mtemp = [mtemp;mtemp(end)];
    end
end
mtemp = mtemp(2:end);

%----------SEPARAÇÃO DAS VOLTAS---------------
len = length(mtemp);

lonTolerance = 0.025; latTolerance = 0.025;
startVelocity = 0;
start = 2*find(gpsData.speed>startVelocity, 1); %6;
lapStart(1) = start; 

midPoints.lat = (gpsData.lat+[gpsData.lat(2:end); gpsData.lat(end)])/2;
midPoints.lon = (gpsData.lon+[gpsData.lon(2:end); gpsData.lon(end)])/2;
lat = reshape([gpsData.lat midPoints.lat].', [2*len, 1]);
lon = reshape([gpsData.lon midPoints.lon].', [2*len, 1]);
temps = reshape([mtemp mtemp].', [2*len, 1]);
tim = reshape([gpsData.timer gpsData.timer].', [2*len, 1])/1000;
%FORMATO DO TIMER PARA 'mm:ss'
tim = seconds(tim); tim.Format = 'mm:ss';

lonZone = abs(lon-lon(start))< lonTolerance;
latZone = abs(lat-lat(start))< latTolerance;
zone = lonZone .* latZone;
laps = find((zone-[0; zone(1:end-1)])==1);
laps = [laps; 2*len];


%---------------GRÁFICOS-----------------

hFig = figure( 403 ); set( hFig, 'Name', 'MTempLaps','NumberTitle','off');
clf;
sgt = sgtitle('Temperatura do Motor ao longo da pista');
sgt.FontSize = 20;
% hNC=sgt.NodeChildren.Children(2);
% hNC.Position= [0, .9];

NLaps = length(laps);

for i = 1:(NLaps-1)
    lapfig = subplot(round(sqrt(NLaps)), ceil(sqrt(NLaps)), i);
    
    scatter(lon(laps(i)), lat(laps(i)), 100, 'red', 's'); 
    hold on    
    s=scatter(lon(laps(i):laps(i+1)), lat(laps(i):laps(i+1)), 20, temps(laps(i):laps(i+1)), 'filled');
    tempDT = dataTipTextRow('Temp',temps(laps(i):laps(i+1)));
    timeDT = dataTipTextRow('Timer',tim(laps(i):laps(i+1)));
    s.DataTipTemplate.DataTipRows = [tempDT;timeDT];
%     scatter(midPoints.lon(laps(i):laps(i+1)), midPoints.lat(laps(i):laps(i+1)), 5, [.5 .5 .5], 'filled');
    hold off     
    set(gca,'XColor', 'none','YColor','none');
%     set(gca,'Color',[.9 .9 .9]);
    lapfig.Position = lapfig.Position + [-0.05 -0.05 0.05 0.05];
%     title("Lap " + i);
    legend("Lap " + i);
    colormap jet
    caxis([min(mtemp) max(mtemp)+1])
    colorbar;
end

end