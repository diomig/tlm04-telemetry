function [GPS_data, Battery_data, Controller_data] = decodeFile (file_name)

file_name = file_name;
fileID = fopen(file_name, 'r');
Battery_data = zeros(1, 10);
Controller_data = zeros(1,6);
GPS_data = struct('utc', 0, 'lat', 0, 'lon', 0, 'speed', 0, 'timer',0);
EastWest = '?';
timer = 0;
dados = "";
while true
    line = fgetl(fileID);
    if line == -1
        break;
    end
    if length(line)~=0
        dados = strsplit(""+line, ',');
        if (line(1) == '$' && length(dados)==15)
            new_gps.utc = str2double(dados(2));
            new_gps.lat = str2double(dados(3));
            new_gps.lon = str2double(dados(5));
            new_gps.speed = str2double(dados(13));
            new_gps.timer = str2double(dados(15));
            %GPS_data = [GPS_data; new_gps];
            if EastWest == '?'
                EastWest = dados(6);
            end
            if abs(new_gps.lon-908)<100
                GPS_data.utc = [GPS_data.utc; new_gps.utc];
                GPS_data.lat = [GPS_data.lat; new_gps.lat];
                GPS_data.lon = [GPS_data.lon; new_gps.lon];
                GPS_data.speed = [GPS_data.speed; new_gps.speed];
                GPS_data.timer = [GPS_data.timer; new_gps.timer];
            end
        elseif (line(1) == '5' && length(dados)==10)
            Battery_data = [Battery_data;str2num(line)];
            
        elseif (line(1) == '1' && length(dados) == 6)
            Controller_data = [Controller_data; str2num(line)];
        end
    end
end
fclose(fileID);

if EastWest =='W'
    GPS_data.lon = -GPS_data.lon;
end

GPS_data.utc = GPS_data.utc(2:end);
GPS_data.lat = GPS_data.lat(2:end);
GPS_data.lon = GPS_data.lon(2:end);
GPS_data.speed = GPS_data.speed(2:end);
GPS_data.timer = GPS_data.timer(2:end);

Battery_data = Battery_data(2:end,:);

Controller_data = Controller_data(2:end,:);
end