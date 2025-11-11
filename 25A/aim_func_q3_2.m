function [all_shield_time, shield_time1, shield_time2, shield_time3] = aim_func_q3_2(drone_speed, alpha, t_deploy1, t_explode_delay1, ...
                                        t_deploy2,t_explode_delay2, t_deploy3,t_explode_delay3)
% 目标函数
% 初始化参数
g = 9.8;
drone_pos0 = [17800, 0, 1800];  % FY1
missile_pos0 = [20000, 0, 2000];  % M1
effective_duration = 20;  %  爆炸后的有效时间 s
sink_speed = 3;  % 云团的下沉速度 m/s
fake_target = [0,0,0];
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

% 导弹
missile_dir = fake_target - missile_pos0;
missile_dir_norm = norm(missile_dir);  % 导弹的飞行方向，指向假目标
missile_speed = 300;  % m/s
missile_vel = missile_speed * (missile_dir / missile_dir_norm);  % 导弹的速度

% 无人机
if alpha > 0
    drone_dir = [-cos(alpha), sin(alpha)];  % xoy平面, 单位向量
elseif alpha < 0
    drone_dir = [cos(alpha), abs(sin(alpha))];
elseif alpha == 0
    drone_dir = [1, 0];
end

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
bomb_pos_y3 = drone_pos_deploy2(2) + bomb_vel0(2) * t_fall3;
bomb_pos_z3 = drone_pos_deploy2(3) + bomb_vel0(3) * t_fall3 - 0.5 * g * t_fall3^2;
bomb_explode_pos3 = [bomb_pos_x3, bomb_pos_y3, bomb_pos_z3];


% 烟雾弹1
dt = 0.01;  % 时间步长 s
t_explode1 = t_deploy1 + t_explode_delay1;
t_start1 = t_explode1;
t_end1 = t_explode1 + effective_duration;
t_sim1 = t_start1:dt:t_end1;
shield_time1 = 0;
shield_time1_array = [];  % 储存遮蔽时间

for t = t_sim1
    intersection_cnt = 0;
    % 导弹在该时刻的位置
    missile_pos = missile_pos0 + missile_vel * t;
    
    % 云团下落，t时的位置
    cloud_center = [bomb_explode_pos1(1), bomb_explode_pos1(2), ...
                    bomb_explode_pos1(3) - sink_speed * (t - t_explode1)];
    for j = 1:8
        true_target_pos = true_target_array(j,:);
        if checkIntersection(missile_pos, true_target_pos, cloud_center)
            intersection_cnt = intersection_cnt + 1;
        end
    end 
    if intersection_cnt == 8
        shield_time1 = shield_time1 + dt;  % 可连续遮挡
        shield_time1_array = [shield_time1_array,t];  % 储存遮蔽时间
    end
end

% 烟雾弹2 
t_explode2 = t_deploy2 + t_explode_delay2;
t_start2 = t_explode2;
t_end2 = t_explode2 + effective_duration;
t_sim2 = t_start2:dt:t_end2;
shield_time2 = 0;
shield_time2_array = [];  % 储存遮蔽时间

for t = t_sim2
    intersection_cnt = 0;
    % 导弹在该时刻的位置
    missile_pos = missile_pos0 + missile_vel * t;
    
    % 云团下落，t时的位置
    cloud_center = [bomb_explode_pos2(1), bomb_explode_pos2(2), ...
                    bomb_explode_pos2(3) - sink_speed * (t - t_explode2)];
    for j = 1:8
        true_target_pos = true_target_array(j,:);
        if checkIntersection(missile_pos, true_target_pos, cloud_center)
            intersection_cnt = intersection_cnt + 1;
        end
    end 
    if intersection_cnt == 8
        shield_time2 = shield_time2 + dt;  % 可连续遮挡
        shield_time2_array = [shield_time2_array,t];  % 储存遮蔽时间
    end
end

% 烟雾弹3
t_explode3 = t_deploy3 + t_explode_delay3;
t_start3 = t_explode3;
t_end3 = t_explode3 + effective_duration;
t_sim3 = t_start3:dt:t_end3;
shield_time3 = 0;
shield_time3_array = [];  % 储存遮蔽时间

for t = t_sim3
    intersection_cnt = 0;
    % 导弹在该时刻的位置
    missile_pos = missile_pos0 + missile_vel * t;
    
    % 云团下落，t时的位置
    cloud_center = [bomb_explode_pos3(1), bomb_explode_pos3(2), ...
                    bomb_explode_pos3(3) - sink_speed * (t - t_explode3)];
    for j = 1:8
        true_target_pos = true_target_array(j,:);
        if checkIntersection(missile_pos, true_target_pos, cloud_center)
            intersection_cnt = intersection_cnt + 1;
        end
    end 
    if intersection_cnt == 8
        shield_time3 = shield_time3 + dt;  % 可连续遮挡
        shield_time3_array = [shield_time3_array,t];  % 储存遮蔽时间
    end
end

% 除去重复的元素
grouped_values = find_diff_element(shield_time1_array, shield_time2_array, shield_time3_array, dt);
len = length(grouped_values);
grouped_values = sort(grouped_values);
if len == 0
    all_shield_time = 0;
elseif len == 1
    all_shield_time = dt;
else
    all_shield_time = grouped_values(end) - grouped_values(1);
end

figure; 

% 绘制四条线，并保存句柄
plot(shield_time1_array, ones(length(shield_time1_array)), 'r', 'LineWidth',2, 'DisplayName', 'Smoke Bomb1');
hold on;
plot(shield_time2_array, ones(length(shield_time2_array))+0.2, 'b', 'LineWidth',2, 'DisplayName', 'Smoke Bomb2');
plot(shield_time3_array, ones(length(shield_time3_array))+0.4, 'k', 'LineWidth',2, 'DisplayName', 'Smoke Bomb3');
plot(grouped_values, ones(length(grouped_values))+0.6, 'g','LineWidth',5, 'DisplayName', 'Effective Block Duration');
grid on;

% 添加图例 - 手动指定标签
% legend('Smoke Bomb1', 'Smoke Bomb2', 'Smoke Bomb3', 'Effective Block Duration', 'Location', 'best');

% 设置y轴标签
yticks([1, 1.2, 1.4, 1.6]);
yticklabels({'Smoke Bomb1', 'Smoke Bomb2', 'Smoke Bomb3', 'Effective Block Duration'});

end