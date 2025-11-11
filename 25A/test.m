clc,clear;

% 初始化参数
g = 9.8; 
v_d = 135;  % 无人机速度 m/s
t_deploy = 11.5;  % 无人机飞行时间 s
t_explode_delay = 1.8;  % 投掷后爆炸时间间隔 s
t_explode = t_deploy + t_explode_delay;  % 从出发到爆炸的时间
sink_speed = 3;  % 云团的下沉速度 m/s
cloud_radius = 10;  % 有效半径 m
effective_duration = 20;  %  爆炸后的有效时间 s

% t=0 的初始位置
drone_pos0 = [17800, 0, 1800];  % FY1
missile_pos0 = [20000, 0, 2000];  % M1
fake_target = [0, 0, 0];
true_target_array = [
    0, 207, 0;
    0, 193, 0;
    -7, 200, 0;
    7, 200, 0;
    0, 207, 10;
    0, 193, 10;
    -7, 200, 10;
    7, 200, 10
];

% 无人机的速度方向
dir_vec = [-cos(85*pi/180),cos(5*pi/180)];

drone_vel = v_d * dir_vec;  % [vx,vy]
drone_vel = [drone_vel, 0];  % vz=0

% t=t_deploy 时抛出烟雾弹，此时无人机的位置
drone_pos_deploy = drone_pos0 + drone_vel * t_deploy;

% 烟雾弹的初始速度
bomb_vel0 = drone_vel;

% 烟雾弹抛物线下落时的位置
t_fall = t_explode_delay;
bomb_pos_x = drone_pos_deploy(1) + bomb_vel0(1) * t_fall;
bomb_pos_y = drone_pos_deploy(2) + bomb_vel0(2) * t_fall;
bomb_pos_z = drone_pos_deploy(3) + bomb_vel0(3) * t_fall - 0.5 * g * t_fall^2;
bomb_explode_pos = [bomb_pos_x, bomb_pos_y, bomb_pos_z];

% 导弹朝向假目标
missile_dir = fake_target - missile_pos0;
missile_dir_norm = norm(missile_dir);
missile_speed = 300;  % m/s
missile_vel = missile_speed * (missile_dir / missile_dir_norm);  % 导弹的速度


dt = 0.0001;  % 时间步长 s
t_start = t_explode;
t_end = t_explode + effective_duration;
t_sim = t_start:dt:t_end;
shield_time = 0;


for t = t_sim
    intersection_cnt = 0;
    % 导弹在该时刻的位置
    missile_pos = missile_pos0 + missile_vel * t;
    
    % 云团下落，t时的位置
    cloud_center = [bomb_explode_pos(1), bomb_explode_pos(2), ...
                    bomb_explode_pos(3) - sink_speed * (t - t_explode)];
    for j = 1:8
        true_target_pos = true_target_array(j,:);
        if checkIntersection(missile_pos, true_target_pos, cloud_center)
            intersection_cnt = intersection_cnt + 1;
        end
    end 
    if intersection_cnt == 8
        shield_time = shield_time + dt;  % 可连续遮挡
    end
end

% 输出有效遮挡时间
fprintf('烟幕干扰弹对M1的有效遮蔽时长: %.2f s\n', shield_time);

