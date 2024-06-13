% clear;clc;
twos2dec = @(x, b) x-(x>=2^(b-1))*2^b;


%% FILE DECODING
file_name = 'KIRO 5.txt';

[gps_data, battery_data, controller_data] = decodeFile (file_name);

%%  PLOTS
%colormap da velocidade ao longo da pista
computeGPS(gps_data); 

%tensões do acumulador
computeBMS(battery_data);   

%RPMs com posição do acelerador
computeRPM(controller_data);    

%temperaturas do motor em função do tempo e ao longo da pista
computeMotorTemp(controller_data, gps_data); 

%velocidades de acordo com o Controlador e com o GPS
% computeVelocity(controller_data, gps_data); 

%valor acelerador ao longo da pista
computeThrottle(controller_data, gps_data);


computeBatCurr(controller_data);
computeBatVolt(controller_data);
computeMotorCurrents(controller_data);
computeMotorVoltages(controller_data);
computeTorque(controller_data);  %ALTERAR O FATOR DE SCALING
%% ToDo
% - correntes no motor
% - correntes na bateria
% - torque
% - suspensões


%% SOBREPOSIÇÃO DOS GRÁFICOS DE TEMPERATURA
%(abrir o workspace 'testes KIRO 31')

% motorTemps(controller1);
% motorTemps(controller2);
% motorTemps(controller3);
% motorTemps(controller4);