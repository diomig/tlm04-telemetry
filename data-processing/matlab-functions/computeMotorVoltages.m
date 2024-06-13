function computeMotorVoltages(controller_data)
twos2dec = @(x, b) x-(x>=2^(b-1))*2^b;
    
voltages = controller_data(controller_data(:,1)==12,2:3);
voltages = twos2dec(voltages,16)*0.0625;
timer = controller_data(controller_data(:,1)==12, 6)/1000;

hFig = figure( 605 ); set( hFig, 'Name', 'MotorVolt','NumberTitle','off');
clf;

hold on;
plot(timer, voltages);
hold off;

title('Tensões do Motor');
xlabel('Tempo[s]');
ylabel('Tensão [V]');
xlim([timer(1) timer(end)]); 

legend('Ud', 'Uq')
end