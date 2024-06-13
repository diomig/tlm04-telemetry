function computeThrottle(controller_data, gps_data)
throttle=NaN;
bigGPS.lon = NaN; bigGPS.lat = NaN;
for k = 1:length(gps_data.timer)-1
    interval = ((controller_data(:,6)-gps_data.timer(k))>0).*((controller_data(:,6)-gps_data.timer(k))<900);
    aux = controller_data(interval==true,:);
    t = aux(aux(:,1)==15, 5) * 3.0518509e-3;
    new_value = mean(t);
    if length(t)~=0
        throttle = [throttle; t];
        n = (0:length(t)-1).';
        new_lon = gps_data.lon(k) + (gps_data.lon(k+1)-gps_data.lon(k))*n/length(t);
        new_lat = gps_data.lat(k) + (gps_data.lat(k+1)-gps_data.lat(k))*n/length(t);
        bigGPS.lon = [bigGPS.lon; new_lon];
        bigGPS.lat = [bigGPS.lat; new_lat];
%     else
%         throttle = [throttle;throttle(end)];
%         disp('AAAAAAAAAAAAAAAAAAAAA')
    end
end
% throttle = throttle(2:end);

hFig = figure( 351 ); set( hFig, 'Name', 'rpm','NumberTitle','off');

s = scatter(bigGPS.lon, bigGPS.lat, 40, throttle, 'filled'); 
title('Acelerador');
set(gca,'XColor', 'none','YColor','none');
cMap = interp1([0;1],[0.2 0.2 0.2; 0 1 0],linspace(0,1,256));
colormap(cMap);
colorbar;
xlim([min(gps_data.lon) max(gps_data.lon)]);
ylim([min(gps_data.lat) max(gps_data.lat)]);


%%
% ------------- SEPARAÇÃO POR VOLTAS---------------------------
lonTolerance = 0.01; latTolerance = 0.01;
startVelocity = 0;
start = 2*find(gps_data.speed>startVelocity, 1); %6;   <----- ???


lonZone = abs(bigGPS.lon-bigGPS.lon(start))< lonTolerance;
latZone = abs(bigGPS.lat-bigGPS.lat(start))< latTolerance;
zone = lonZone .* latZone;
laps = find((zone-[0; zone(1:end-1)])==1);
laps = [laps; length(throttle)];


hFig = figure( 352 ); set( hFig, 'Name', 'ThrottleLaps','NumberTitle','off');
clf;
sgt = sgtitle('---');
sgt.FontSize = 20;
% hNC=sgt.NodeChildren.Children(2);
% hNC.Position= [0, .9];

NLaps = length(laps);

for i = 1:(NLaps-1)
    lapfig = subplot(round(sqrt(NLaps)), ceil(sqrt(NLaps)), i);
    
    scatter(bigGPS.lon(laps(i)), bigGPS.lat(laps(i)), 100, 'red', 's'); 
    hold on    
    s=scatter(bigGPS.lon(laps(i):laps(i+1)), bigGPS.lat(laps(i):laps(i+1)), 20, throttle(laps(i):laps(i+1)), 'filled');
    throttleDT = dataTipTextRow('Temp',throttle(laps(i):laps(i+1)));
%     timeDT = dataTipTextRow('Timer',tim(laps(i):laps(i+1)));
    s.DataTipTemplate.DataTipRows = [throttleDT]; %;timeDT
%     scatter(midPoints.lon(laps(i):laps(i+1)), midPoints.lat(laps(i):laps(i+1)), 5, [.5 .5 .5], 'filled');
    hold off     
    set(gca,'XColor', 'none','YColor','none');
%     set(gca,'Color',[.9 .9 .9]);
    lapfig.Position = lapfig.Position + [-0.05 -0.05 0.05 0.05];
%     title("Lap " + i);
    legend("Lap " + i);
    colormap jet
    caxis([min(throttle) max(throttle)+1])
    colorbar;
end



end