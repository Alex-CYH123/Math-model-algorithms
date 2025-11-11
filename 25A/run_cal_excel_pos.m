clc,clear;

% 无人机1
alpha = -0.0999;% ===修改参数===
drone_pos0 = [17800, 0, 1800];  % FY1% ===修改参数===

drone_dir = [-sign(alpha) * cos(alpha), abs(sin(alpha))];
% drone_dir = [-cos(alpha),-sin(alpha)];  % FY2
% drone_dir = [-sin(alpha), cos(alpha)];  % FY3


drone_speed = 139.9943;% ===修改参数===
t_deploy1 = 0;% ===修改参数===
t_explode_delay1 = 3.6738;% ===修改参数===
disp('====================第1个===============================');
[drone_pos_deploy1, bomb_explode_pos1] = cal_excel_pos(drone_pos0, drone_dir, drone_speed, t_deploy1, t_explode_delay1);
disp('====抛出位置====');
disp(drone_pos_deploy1);
disp('================');
disp('====起爆位置====');
disp(bomb_explode_pos1);
disp('================');

t_deploy2 = 3.3724;
t_explode_delay2 = 5.1290;
disp('====================第2个===============================');
[drone_pos_deploy2, bomb_explode_pos2] = cal_excel_pos(drone_pos0, drone_dir, drone_speed, t_deploy2, t_explode_delay2);
disp('====抛出位置====');
disp(drone_pos_deploy2);
disp('================');
disp('====起爆位置====');
disp(bomb_explode_pos2);
disp('================');

t_deploy3 = 5.4032;
t_explode_delay3 = 5.9771;
disp('====================第3个===============================');
[drone_pos_deploy3, bomb_explode_pos3] = cal_excel_pos(drone_pos0, drone_dir, drone_speed, t_deploy3, t_explode_delay3);
disp('====抛出位置====');
disp(drone_pos_deploy3);
disp('================');
disp('====起爆位置====');
disp(bomb_explode_pos3);
disp('================');
