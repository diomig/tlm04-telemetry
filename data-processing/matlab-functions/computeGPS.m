function [] = computeGPS(gps_data)
% lon = gps_data.lon(1:100);
% lat = gps_data.lat(1:100);
% hFig = figure( 101 ); set( hFig, 'Name', 'GPS');
% plot(lon, lat, '-.'); 
% % plot(gps_data.lon, gps_data.lat, '-.'); 


hFig = figure( 102 ); set( hFig, 'Name', 'GPScmap','NumberTitle','off');
s = scatter(gps_data.lon, gps_data.lat, 20, gps_data.speed, 'filled'); 
colormap jet
title('Velocidade ao longo da pista');
set(gca,'XColor', 'none','YColor','none');
colormap jet
colorbar;
xlim([min(gps_data.lon) max(gps_data.lon)]);
ylim([min(gps_data.lat) max(gps_data.lat)]);

tim = seconds(gps_data.timer/1000); tim.Format = 'mm:ss';

speedDT = dataTipTextRow('Vel',gps_data.speed);
timeDT = dataTipTextRow('Timer',tim);
    
s.DataTipTemplate.DataTipRows = [speedDT, timeDT];

race_speed = gps_data.speed(gps_data.speed>30);

annotation('textbox', [0.1 0.1 0.8 0], ...
'String', sprintf('V_{avg} = %.1f Km/h,     V_{max} = %.1f Km/h',mean(race_speed),  max(race_speed)), ...
'Color', [0 0.5 1], ...
'FontWeight', 'bold', ...
'EdgeColor', 'none')


%%     SEPARAÇÃO DAS VOLTAS

% lonTolerance = 0.01; latTolerance = 0.01;
% startVelocity = 40;
% start = find(gps_data.speed>startVelocity, 1); %6;
% lapStart(1) = start; 
% 
% lonZone = abs(gps_data.lon-gps_data.lon(start))< lonTolerance;
% latZone = abs(gps_data.lat-gps_data.lat(start))< latTolerance;
% zone = lonZone .* latZone;
% laps = find((zone-[0; zone(1:end-1)])==1);
% laps = [laps; length(gps_data.lon)];
% 
% hFig = figure( 103 ); set( hFig, 'Name', 'GPSlaps','NumberTitle','off');
% clf;
% N = length(laps);
% 
% for i = 1:(N-1)
%     lapfig = subplot(floor(sqrt(N)), ceil(sqrt(N)), i);
%     
%     scatter(gps_data.lon(laps(i)), gps_data.lat(laps(i)), 80, 'red', 's'); hold on    
%     scatter(gps_data.lon(laps(i):laps(i+1)), gps_data.lat(laps(i):laps(i+1)), 20, gps_data.speed(laps(i):laps(i+1)), 'filled');hold off     
%     set(gca,'XColor', 'none','YColor','none');
%     lapfig.Position = lapfig.Position + [-0.05 -0.05 0.05 0.05];
%     title("Lap " + i);
%     colormap cool
%     
%     caxis([0 max(gps_data.speed)])
%     colorbar;
% end
end
