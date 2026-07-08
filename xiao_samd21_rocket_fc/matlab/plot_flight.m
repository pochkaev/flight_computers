function data = plot_flight(csv_path)
if nargin < 1
    csv_path = fullfile('..', 'logs', 'sample_flight.csv');
end

data = readtable(csv_path);
data.t_s = data.ms ./ 1000.0;

figure('Name', 'Flight Telemetry', 'Color', 'w');
tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(data.t_s, data.baro_alt_m, 'LineWidth', 1.5);
hold on;
plot(data.t_s, data.gps_alt_m, 'LineWidth', 1.0);
grid on;
ylabel('Altitude (m)');
legend('Baro', 'GPS', 'Location', 'best');

nexttile;
plot(data.t_s, [data.ax_g data.ay_g data.az_g], 'LineWidth', 1.2);
grid on;
ylabel('Accel (g)');
legend('ax', 'ay', 'az', 'Location', 'best');

nexttile;
plot(data.t_s, [data.gx_dps data.gy_dps data.gz_dps], 'LineWidth', 1.2);
grid on;
ylabel('Gyro (dps)');
legend('gx', 'gy', 'gz', 'Location', 'best');

nexttile;
stairs(data.t_s, categorical(data.phase), 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Phase');
end
