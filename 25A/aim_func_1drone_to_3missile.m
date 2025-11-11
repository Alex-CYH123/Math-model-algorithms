function all_shield_time = aim_func_1drone_to_3missile(drone_speed, alpha, ...
    t_deploy1, t_explode_delay1,t_deploy2, t_explode_delay2,...
    t_deploy3, t_explode_delay3)

% 每一个无人机对三个导弹的遮挡情况
% 一个无人机有3个烟雾弹
% 有效遮挡时间为无人机对三个导弹的有效遮挡时间之和

g = 9.8;
drone_pos0 = [17800,0,1800];
missile1_pos0 = [20000, 0, 2000];  % M1
missile2_pos0 = [19000,600,2100];  % M2
missile3_pos0 = [18000,-600,1900]; % M3

effective_duration = 20;  %  爆炸后的有效时间 s

fake_target = [0,0,0];

missile_speed = 300;  % m/s

% 导弹1
missile1_dir = fake_target - missile1_pos0;
missile1_dir_norm = norm(missile1_dir);  % 导弹的飞行方向，指向假目标
missile1_vel = missile_speed * (missile1_dir / missile1_dir_norm);  % 导弹的速度

% 导弹2
missile2_dir = fake_target - missile2_pos0;
missile2_dir_norm = norm(missile2_dir);  % 导弹的飞行方向，指向假目标
missile2_vel = missile_speed * (missile2_dir / missile2_dir_norm);  % 导弹的速度

% 导弹3
missile3_dir = fake_target - missile3_pos0;
missile3_dir_norm = norm(missile3_dir);  % 导弹的飞行方向，指向假目标
missile3_vel = missile_speed * (missile3_dir / missile3_dir_norm);  % 导弹的速度

% 无人机
% =====需要修改方向======
drone_dir = [-sign(alpha) * cos(alpha), abs(sin(alpha))];
drone_vel = drone_speed * drone_dir;  % [vx,vy]
drone_vel = [drone_vel, 0];  % vz=0

% t=t_deploy 时抛出烟雾弹，此时无人机的位置
drone_pos_deploy1 = drone_pos0 + drone_vel * t_deploy1;
drone_pos_deploy2 = drone_pos0 + drone_vel * t_deploy2;
drone_pos_deploy3 = drone_pos0 + drone_vel * t_deploy3;

% 烟雾弹的初始速度
bomb_vel0 = drone_vel;

% 烟雾弹抛物线下落时的位置
% 烟雾弹1
t_fall1 = t_explode_delay1;
bomb_pos_x1 = drone_pos_deploy1(1) + bomb_vel0(1) * t_fall1;
bomb_pos_y1 = drone_pos_deploy1(2) + bomb_vel0(2) * t_fall1;
bomb_pos_z1 = drone_pos_deploy1(3) + bomb_vel0(3) * t_fall1 - 0.5 * g * t_fall1^2;
bomb_explode_pos1 = [bomb_pos_x1, bomb_pos_y1, bomb_pos_z1];
% 烟雾弹2
t_fall2 = t_explode_delay2;
bomb_pos_x2 = drone_pos_deploy2(1) + bomb_vel0(1) * t_fall2;
bomb_pos_y2 = drone_pos_deploy2(2) + bomb_vel0(2) * t_fall2;
bomb_pos_z2 = drone_pos_deploy2(3) + bomb_vel0(3) * t_fall2 - 0.5 * g * t_fall2^2;
bomb_explode_pos2 = [bomb_pos_x2, bomb_pos_y2, bomb_pos_z2];
% 烟雾弹3
t_fall3 = t_explode_delay3;
bomb_pos_x3 = drone_pos_deploy3(1) + bomb_vel0(1) * t_fall3;
bomb_pos_y3 = drone_pos_deploy3(2) + bomb_vel0(2) * t_fall3;
bomb_pos_z3 = drone_pos_deploy3(3) + bomb_vel0(3) * t_fall3 - 0.5 * g * t_fall3^2;
bomb_explode_pos3 = [bomb_pos_x3, bomb_pos_y3, bomb_pos_z3];



% 烟雾弹1
t_explode1 = t_deploy1 + t_explode_delay1;
t_start1 = t_explode1;
t_end1 = t_explode1 + effective_duration;



% 烟雾弹2
t_explode2 = t_deploy2 + t_explode_delay2;
t_start2 = t_explode2;
t_end2 = t_explode2 + effective_duration;



% 烟雾弹3
t_explode3 = t_deploy3 + t_explode_delay3;
t_start3 = t_explode3;
t_end3 = t_explode3 + effective_duration;

% 初始化--储存导弹的被遮挡有效时间，计算三个同时有效遮挡时使用
% shield_time1_array = [];
% 
% shield_time2_array = [];
% 
% shield_time3_array = [];

dt = 0.01;  % 时间步长 s
t_start = min([t_start1, t_start2, t_start3]);
t_end = max([t_end1,t_end2,t_end3]);
t_sim = t_start:dt:t_end;

% 第一个导弹的被遮挡时间
[shield_time1,~] = cal_3bomb_to_1missile(t_sim, missile1_pos0, missile1_vel, bomb_explode_pos1,bomb_explode_pos2,bomb_explode_pos3, ...
                                                t_start1,t_end1,t_start2,t_end2,t_start3,t_end3, t_explode1, t_explode2, t_explode3,dt);
[shield_time2,~] = cal_3bomb_to_1missile(t_sim, missile2_pos0, missile2_vel, bomb_explode_pos1,bomb_explode_pos2,bomb_explode_pos3, ...
                                                t_start1,t_end1,t_start2,t_end2,t_start3,t_end3, t_explode1, t_explode2, t_explode3,dt);
[shield_time3,~] = cal_3bomb_to_1missile(t_sim, missile3_pos0, missile3_vel, bomb_explode_pos1,bomb_explode_pos2,bomb_explode_pos3, ...
                                                t_start1,t_end1,t_start2,t_end2,t_start3,t_end3, t_explode1, t_explode2, t_explode3,dt);

all_shield_time = shield_time1 + shield_time2 +shield_time3;

end



