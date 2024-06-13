function computeBMS(battery_data)

%% TENSÕES DOS PACKS
Total = battery_data(battery_data(:,1)==50,2:6);
Total = Total/10;
time = battery_data(battery_data(:,1)==50,10);
time = time/1000;

hFig = figure( 201 ); set( hFig, 'Name', 'packVolt','NumberTitle','off');
hold on
for pack = 1:5
    plot(time/1000, Total(:,pack));
end
ylim([10 26]);
hold off
legend('Pack1','Pack2','Pack3','Pack4','Pack5');
title("Tensões dos Packs");
xlabel("Tempo [s]"); ylabel("Tensão [V]");



%% TENSÕES DAS CÉLULAS

psz = max(sum(battery_data(:,1)==(51:55)));

pack_tension = NaN(5,6,psz);
for pack = 1:5
    new = battery_data(battery_data(:,1)==(50+pack),2:7).';
    new_len = length(new);
    pack_tension(pack, :,1:new_len) = new;
end
pack_tension = (pack_tension/100+2).*(pack_tension~=0);

hFig = figure( 202 ); set( hFig, 'Name', 'cellVolt','NumberTitle','off');
clf;
sgt = sgtitle('Tensões das Células');
sgt.FontSize = 20;
for pack = 1:5
    subplot(3,2,pack);
    hold on
    for cell = 1:6
        plot_mat = squeeze(pack_tension(pack,cell,:));
        plot_sz = size(plot_mat);
        plot((1:plot_sz(1)),plot_mat);
    end
    xlim([0 plot_sz(1)-5]);
    ylim([0 4.2]);
    legend('Cell1','Cell2','Cell3','Cell4','Cell5', 'Cell6');
    title("Pack "+pack);
    ylabel("Tensão [V]");

    hold off
end

end

