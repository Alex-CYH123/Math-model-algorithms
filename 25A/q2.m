clc,clear;

g = 9.8;  % 重力加速度
sink_speed = 3;  % 云团的下沉速度 m/s
cloud_radius = 10;  % 有效半径 m
effective_duration = 20;  %  爆炸后的有效时间 s

% t=0 的无人机和导弹的初始位置
drone_pos0 = [17800, 0, 1800];  % FY1
missile_pos0 = [20000, 0, 2000];  % M1
drone_speed = 140;  % 无人机速度 m/s

% 目标位置
fake_target = [0, 0, 0];
true_target_center = [0,200,5];

% 导弹朝向假目标
missile_dir = fake_target - missile_pos0;
missile_dir_norm = norm(missile_dir);
missile_speed = 300;  % m/s
missile_vel = missile_speed * (missile_dir / missile_dir_norm);  % 导弹的速度

% 导弹与真目标的向量
missile_dirto_true_target = true_target_center - missile_pos0;

% 导弹追上无人机的时间
missile_upto_drone_t = (drone_pos0(1) - missile_pos0(1)) / missile_vel(1);

% 遍历missile_upto_drone_t
dt = 0.1;
t_sim = 0:dt:missile_upto_drone_t;
shield_time_array = [];

for t = t_sim
    % 计算导弹在该时刻的位置
    missile_pos = missile_pos0 + missile_vel * t;
    intersection_point = find_intersection(missile_pos, true_target_center, ...
        drone_pos0(1), drone_pos0(2), drone_pos0(3), t);
    
    shield_time = cal_shield_time(t,missile_pos0,missile_vel,intersection_point);
    shield_time_array = [shield_time_array,shield_time];
    
end

% 输出最大遮挡时间
fprintf('烟幕干扰弹对M1的最大有效遮蔽时长: %.2f s\n', max(shield_time_array));
