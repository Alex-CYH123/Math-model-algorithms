function [total_shield_time, shield_time1, shield_time2, shield_time3] = cal_3bomb_to_1missile(t_sim, missile_pos0, missile_vel, bomb_explode_pos1,bomb_explode_pos2,bomb_explode_pos3, ...
                                                t_start1,t_end1,t_start2,t_end2,t_start3,t_end3, ...
                                                t_explode1, t_explode2, t_explode3, dt)
sink_speed = 3;
true_target_array = [
    0, 207, 0;
    0, 193, 0;
    7, 200, 0;
    0, 207, 10;
    0, 193, 10;
    -7, 200, 10;
    7, 200, 10
];

shield_time1 = 0;
shield_time2 = 0;
shield_time3 = 0;

shield_time1_array = [];
shield_time2_array = [];
shield_time3_array = [];

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

    for j = 1:7
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

    if intersection_cnt == 7
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
total_shield_time = length(all_values) * dt;  % 总的干扰时常

end