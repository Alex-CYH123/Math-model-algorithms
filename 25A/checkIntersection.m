function intersects = checkIntersection(missile_pos, true_target, cloud_center)
    r = 10;  % 遮挡半径
    
    % 导弹指向真目标的向量
    dir_vec = true_target - missile_pos;
    dir_norm = norm(dir_vec);
    
    % 导弹指向云团中心
    vec_to_center = cloud_center - missile_pos;
    C = cross(dir_vec, vec_to_center);
    C_norm = norm(C);
    d = C_norm/dir_norm;
    if (d <= r && dot(dir_vec, vec_to_center) > 0) || norm(vec_to_center) < 10
        intersects = true;
    else
        intersects = false;
    end
end