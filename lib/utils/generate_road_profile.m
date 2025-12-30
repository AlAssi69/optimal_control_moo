function [road_ts, zr, zr_dot, time] = generate_road_profile(conf)
% GENERATE_ROAD_PROFILE - Creates the road input timeseries

time = 0:conf.dt:conf.T_end;
V = conf.car_speed_kmh * (1000/3600);
bump_duration = conf.bump_length / V;
end_time = conf.start_time + bump_duration;

zr = zeros(size(time));
zr_dot = zeros(size(time));

switch lower(conf.road_type)
    case 'parabolic'
        K = (4 * conf.bump_height) / (conf.bump_length^2);
        for i = 1:length(time)
            t = time(i);
            if t >= conf.start_time && t <= end_time
                t_loc = t - conf.start_time;
                x_loc = V * t_loc;
                % Parabolic Height
                zr(i) = K * x_loc * (conf.bump_length - x_loc);
                % Parabolic Velocity (Derivative)
                zr_dot(i) = V * K * (conf.bump_length - 2*x_loc);
            end
        end

    case 'haversine'
        omega = 2 * pi * V / conf.bump_length;
        for i = 1:length(time)
            t = time(i);
            if t >= conf.start_time && t <= end_time
                t_loc = t - conf.start_time;
                % Haversine Height
                zr(i) = 0.5 * conf.bump_height * (1 - cos(omega * t_loc));
                % Haversine Velocity
                zr_dot(i) = 0.5 * conf.bump_height * omega * sin(omega * t_loc);
            end
        end

    otherwise
        error('Unknown road profile type: %s', conf.road_type);
end

% Create Timeseries for Simulink
road_ts = timeseries(zr_dot', time);
end