function [drone_pos_deploy, bomb_explode_pos] = cal_excel_pos(drone_pos0, drone_dir, drone_speed, t_deploy, t_explode_delay)
% 初始化参数
g = 9.8;

% ====针对不同的无人机需要不同的方向向量=====

drone_vel = drone_speed * drone_dir;  % [vx,vy]
drone_vel = [drone_vel, 0];  % vz=0

% t=t_deploy 时抛出烟雾弹，此时无人机的位置
drone_pos_deploy = drone_pos0 + drone_vel * t_deploy;  % 抛出位置

% 烟雾弹的初始速度
bomb_vel0 = drone_vel;

% 烟雾弹抛物线下落时的位置
t_fall = t_explode_delay;
bomb_pos_x = drone_pos_deploy(1) + bomb_vel0(1) * t_fall;
bomb_pos_y = drone_pos_deploy(2) + bomb_vel0(2) * t_fall;
bomb_pos_z = drone_pos_deploy(3) + bomb_vel0(3) * t_fall - 0.5 * g * t_fall^2;
bomb_explode_pos = [bomb_pos_x, bomb_pos_y, bomb_pos_z];  % 起爆位置

end

