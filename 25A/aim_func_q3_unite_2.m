function [all_shield_time, shield_time1, shield_time2, shield_time3] = aim_func_q3_unite_2(drone_speed, alpha, t_deploy1, t_explode_delay1, ...
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
    7, 200, 10;
    4.9497,204.9497,0;
    -4.9497,204.9497,0;
    4.9497,195.0503,0;
    -4.9497,195.0503,0;
    4.9497,204.9497,10;
    -4.9497,204.9497,10;
    4.9497,195.0503,10;
    -4.9497,195.0503,10;
];

% 导弹
missile_dir = fake_target - missile_pos0;
missile_dir_norm = norm(missile_dir);  % 导弹的飞行方向，指向假目标
missile_speed = 300;  % m/s
missile_vel = missile_speed * (missile_dir / missile_dir_norm);  % 导弹的速度

% 无人机
drone_dir = [-sign(alpha) * cos(alpha), abs(sin(alpha))];  % xoy平面, 单位向量

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
dt = 0.01;  % 时间步长 s

t_explode1 = t_deploy1 + t_explode_delay1;
t_start1 = t_explode1;
t_end1 = t_explode1 + effective_duration;

shield_time1 = 0;
shield_time1_array = [];  % 储存遮蔽时间

% 烟雾弹2 
t_explode2 = t_deploy2 + t_explode_delay2;
t_start2 = t_explode2;
t_end2 = t_explode2 + effective_duration;
shield_time2 = 0;
shield_time2_array = [];  % 储存遮蔽时间

% 烟雾弹3
t_explode3 = t_deploy3 + t_explode_delay3;
t_start3 = t_explode3;
t_end3 = t_explode3 + effective_duration;

shield_time3 = 0;
shield_time3_array = [];  % 储存遮蔽时间


t_start = min([t_start1, t_start2, t_start3]);
t_end = max([t_end1,t_end2,t_end3]);
t_sim = t_start:dt:t_end;

for t = t_sim
    intersection_cnt = 0;
    % 导弹在该时刻的位置
    missile_pos = missile_pos0 + missile_vel * t;
    % 云团下落，t时的位置
    cloud_center1 = [bomb_explode_pos1(1), bomb_explode_pos1(2), ...
                    bomb_explode_pos1(3) - sink_speed * (t - t_explode1)];
    cloud_center2 = [bomb_explode_pos2(1), bomb_explode_pos2(2), ...
                bomb_explode_pos2(3) - sink_speed * (t - t_explode2)];
    cloud_center3 = [bomb_explode_pos3(1), bomb_explode_pos3(2), ...
            bomb_explode_pos3(3) - sink_speed * (t - t_explode3)];

    % 判断是否有有效遮挡，是哪个烟雾弹发生遮挡
    
    flag1 = 0;
    flag2 = 0;
    flag3 = 0;

    for j = 1:16
        flag = 0; 
        true_target_pos = true_target_array(j,:);
        % 不能是选择分支结构
        if t >= t_start1 && t <= t_end1 && checkIntersection(missile_pos, true_target_pos, cloud_center1)
            flag = 1;
            flag1 = 1;
        end
        if t >= t_start2 && t <= t_end2 && checkIntersection(missile_pos, true_target_pos, cloud_center2)
            flag = 1;
            flag2 = 1;
        end
        if t >= t_start3 && t <= t_end3 && checkIntersection(missile_pos, true_target_pos, cloud_center3)
            flag = 1;
            flag3 = 1;
        end

        if flag ~= 0
            intersection_cnt = intersection_cnt + 1;
        end
    end

    if intersection_cnt == 16
        if flag1 == 1
            shield_time1 = shield_time1 + dt;  % 可连续遮挡
            shield_time1_array = [shield_time1_array,t];  % 储存遮蔽时间
        end
        if flag2 == 1
            shield_time2 = shield_time2 + dt;  % 可连续遮挡
            shield_time2_array = [shield_time2_array,t];  % 储存遮蔽时间
        end
        if flag3 == 1
            shield_time3 = shield_time3 + dt;  % 可连续遮挡
            shield_time3_array = [shield_time3_array,t];  % 储存遮蔽时间
        end

    end
end

% 除去重复的元素
all_values = unique([shield_time1_array(:); shield_time2_array(:); shield_time3_array(:)]);
all_shield_time = length(all_values) * dt;

figure; 
% 绘制四条线，并保存句柄
plot(shield_time1_array, ones(length(shield_time1_array)), 'r.', 'LineWidth',2, 'DisplayName', 'Smoke Bomb1');
hold on;
plot(shield_time2_array, ones(length(shield_time2_array))+0.2, 'b.', 'LineWidth',2, 'DisplayName', 'Smoke Bomb2');
plot(shield_time3_array, ones(length(shield_time3_array))+0.4, 'k.', 'LineWidth',2, 'DisplayName', 'Smoke Bomb3');
plot(all_values, ones(length(all_values))+0.6, 'g.','LineWidth',5, 'DisplayName', 'Effective Block Duration');
grid on;
% 设置y轴标签
yticks([1, 1.2, 1.4, 1.6]);
yticklabels({'Smoke Bomb1', 'Smoke Bomb2', 'Smoke Bomb3', 'Effective Block Duration'});

% 绘制从t_start1到t_end1区间内，的遮挡情况
draw(shield_time1_array, t_start1, t_end1, dt, 1);
draw(shield_time2_array, t_start2, t_end2, dt, 2);
draw(shield_time3_array, t_start3, t_end3, dt, 3);
% 绘制整个过程的遮挡情况
draw(all_values, t_start, t_end, dt, 0);


end