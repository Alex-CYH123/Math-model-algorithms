function shield_time = cal_shield_time(t_start,missile_pos0,missile_vel,intersection_point)
effective_duration = 20;  %  爆炸后的有效时间 s
dt = 0.01;
sink_speed = 3;  % 云团的下沉速度 m/s
t_sim = t_start:dt:t_start+effective_duration;
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
shield_time = 0;
for t = t_sim
    intersection_cnt = 0;
    % 导弹在该时刻的位置
    missile_pos = missile_pos0 + missile_vel * t;
    
    % 云团下落，t时的位置
    cloud_center = [intersection_point(1), intersection_point(2), ...
                    intersection_point(3) - sink_speed * (t - t_start)];
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
end
