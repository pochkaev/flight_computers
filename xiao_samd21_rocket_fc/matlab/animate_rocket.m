function animate_rocket(csv_path)
if nargin < 1
    csv_path = fullfile('..', 'logs', 'sample_flight.csv');
end

data = readtable(csv_path);
data.t_s = data.ms ./ 1000.0;

n = height(data);
roll = zeros(n, 1);
pitch = zeros(n, 1);
yaw = zeros(n, 1);
alpha = 0.98;

for i = 2:n
    dt = max(data.t_s(i) - data.t_s(i - 1), 1e-3);
    roll_gyro = roll(i - 1) + deg2rad(data.gx_dps(i)) * dt;
    pitch_gyro = pitch(i - 1) + deg2rad(data.gy_dps(i)) * dt;
    yaw(i) = yaw(i - 1) + deg2rad(data.gz_dps(i)) * dt;

    g = norm([data.ax_g(i), data.ay_g(i), data.az_g(i)]);
    if g < 1e-6
        roll(i) = roll_gyro;
        pitch(i) = pitch_gyro;
        continue;
    end

    ax = data.ax_g(i) / g;
    ay = data.ay_g(i) / g;
    az = data.az_g(i) / g;
    roll_acc = atan2(ay, az);
    pitch_acc = atan2(-ax, sqrt(ay^2 + az^2));
    roll(i) = alpha * roll_gyro + (1.0 - alpha) * roll_acc;
    pitch(i) = alpha * pitch_gyro + (1.0 - alpha) * pitch_acc;
end

base_points = [
    0.0 0.0 0.0;
    1.0 0.0 0.0;
    1.0 0.0 0.0;
    1.25 0.0 0.0;
    0.18 0.18 0.0;
    0.0  0.11 0.0;
    0.36 0.0  0.0;
    0.18 -0.18 0.0;
    0.0  -0.11 0.0;
    0.36 0.0   0.0;
];
segments = {
    [1 2], [3 4], [5 6 7 5], [8 9 10 8]
};

east_m = (data.lon_deg - data.lon_deg(1)) * 85000.0;
north_m = (data.lat_deg - data.lat_deg(1)) * 111000.0;
up_m = data.baro_alt_m;

figure('Name', 'Rocket Replay', 'Color', 'w');
ax = axes();
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'East (m)');
ylabel(ax, 'North (m)');
zlabel(ax, 'Altitude (m)');
view(ax, 3);
hold(ax, 'on');

trail = plot3(ax, east_m(1), north_m(1), up_m(1), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
rocket_lines = gobjects(numel(segments), 1);
for k = 1:numel(segments)
    rocket_lines(k) = plot3(ax, 0, 0, 0, 'LineWidth', 2);
end

for i = 1:n
    R = eul2rotm_local(yaw(i), pitch(i), roll(i));
    origin = [east_m(i); north_m(i); up_m(i)];
    pts = (R * base_points.')' + origin.';

    for k = 1:numel(segments)
        idx = segments{k};
        set(rocket_lines(k), 'XData', pts(idx, 1), 'YData', pts(idx, 2), 'ZData', pts(idx, 3));
    end

    set(trail, 'XData', east_m(1:i), 'YData', north_m(1:i), 'ZData', up_m(1:i));
    title(ax, sprintf('t = %.2f s   phase = %s', data.t_s(i), string(data.phase(i))));
    drawnow;
    pause(0.03);
end
end

function R = eul2rotm_local(yaw, pitch, roll)
cy = cos(yaw);  sy = sin(yaw);
cp = cos(pitch); sp = sin(pitch);
cr = cos(roll); sr = sin(roll);

Rz = [cy -sy 0; sy cy 0; 0 0 1];
Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
Rx = [1 0 0; 0 cr -sr; 0 sr cr];
R = Rz * Ry * Rx;
end
