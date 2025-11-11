function all_shield_time = aim_func_q4_unite(drone1_speed, alpha1, t_deploy1, t_explode_delay1, ...
                                        drone2_speed, alpha2, t_deploy2,t_explode_delay2, ...
                                        drone3_speed, alpha3, t_deploy3,t_explode_delay3)

% 初始化参数
g = 9.8;
drone1_pos0 = [17800, 0, 1800];  % FY1
drone2_pos0 = [12000,1400,1400]; % FY2
drone3_pos0 = [6000,-3000,700];  % FY3

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

% 导弹速度
missile_dir = fake_target - missile_pos0;
missile_dir_norm = norm(missile_dir);  % 导弹的飞行方向，指向假目标
missile_speed = 300;  % m/s
missile_vel = missile_speed * (missile_dir / missile_dir_norm);  % 导弹的速度

% 无人机1
drone1_dir = [-sign(alpha1) * cos(alpha1), abs(sin(alpha1))];  % xoy平面, 单位向量
drone1_vel = drone1_speed * drone1_dir;  % [vx,vy]
drone1_vel = [drone1_vel, 0];  % vz=0

% 无人机2
drone2_dir = [-cos(alpha2), -sin(alpha2)];  % xoy平面, 单位向量 
drone2_vel = drone2_speed * drone2_dir;  % [vx,vy]
drone2_vel = [drone2_vel, 0];  % vz=0


% 无人机3
drone3_dir = [-sin(alpha3), cos(alpha3)];  % xoy平面, 单位向量 
drone3_vel = drone3_speed * drone3_dir;  % [vx,vy]
drone3_vel = [drone3_vel, 0];  % vz=0


% t=t_deploy 时抛出烟雾弹，此时无人机1，2，3的位置
drone1_pos_deploy = drone1_pos0 + drone1_vel * t_deploy1;
drone2_pos_deploy = drone2_pos0 + drone2_vel * t_deploy2;
drone3_pos_deploy = drone3_pos0 + drone3_vel * t_deploy3;

% 3个烟雾弹的初始速度
bomb1_vel0 = drone1_vel;
bomb2_vel0 = drone2_vel;
bomb3_vel0 = drone3_vel;

% 烟雾弹抛物线下落时的位置
% 无人机1
t_fall1 = t_explode_delay1;
bomb1_pos_x = drone1_pos_deploy(1) + bomb1_vel0(1) * t_fall1;
bomb1_pos_y = drone1_pos_deploy(2) + bomb1_vel0(2) * t_fall1;
bomb1_pos_z = drone1_pos_deploy(3) + bomb1_vel0(3) * t_fall1 - 0.5 * g * t_fall1^2;
bomb1_explode_pos = [bomb1_pos_x, bomb1_pos_y, bomb1_pos_z];
% 无人机2
t_fall2 = t_explode_delay2;
bomb2_pos_x = drone2_pos_deploy(1) + bomb2_vel0(1) * t_fall2;
bomb2_pos_y = drone2_pos_deploy(2) + bomb2_vel0(2) * t_fall2;
bomb2_pos_z = drone2_pos_deploy(3) + bomb2_vel0(3) * t_fall2 - 0.5 * g * t_fall2^2;
bomb2_explode_pos = [bomb2_pos_x, bomb2_pos_y, bomb2_pos_z];
% 无人机3
t_fall3 = t_explode_delay3;
bomb3_pos_x = drone3_pos_deploy(1) + bomb3_vel0(1) * t_fall3;
bomb3_pos_y = drone3_pos_deploy(2) + bomb3_vel0(2) * t_fall3;
bomb3_pos_z = drone3_pos_deploy(3) + bomb3_vel0(3) * t_fall3 - 0.5 * g * t_fall3^2;
bomb3_explode_pos = [bomb3_pos_x, bomb3_pos_y, bomb3_pos_z];

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
    cloud_center1 = [bomb1_explode_pos(1), bomb1_explode_pos(2), ...
                    bomb1_explode_pos(3) - sink_speed * (t - t_explode1)];
    cloud_center2 = [bomb2_explode_pos(1), bomb2_explode_pos(2), ...
                bomb2_explode_pos(3) - sink_speed * (t - t_explode2)];
    cloud_center3 = [bomb3_explode_pos(1), bomb3_explode_pos(2), ...
            bomb3_explode_pos(3) - sink_speed * (t - t_explode3)];
    
    % 判断是否有有效遮挡，是哪个无人机的烟雾弹发生遮挡 
    flag1 = 0;
    flag2 = 0;
    flag3 = 0;

    for j = 1:8
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

    if intersection_cnt == 8
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

end