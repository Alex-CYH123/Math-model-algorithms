clc,clear;
% 参数设置
w_max = 0.6;
w_min = 0.4;
N = 200;       % 粒子数
c1 = 0.5;
c2 = 2;
D = 8;         % 共需要8个变量
K = 100;       % 最大迭代次数
mutation_particles = 25;  % 每次变异时重置多个粒子

% ===修改====
% 无人机所需变量范围
drone_speed_bound = [100,140];  % 无人机速度范围
alpha_bound = [-pi/18,pi/18];  % 无人机飞行角度范围
t_deploy1_bound = [0,5];  % 无人机开始起飞到投掷第1枚烟雾弹
t_deploy2_bound = [0,5];  % 无人机开始起飞到投掷第2枚烟雾弹
t_deploy3_bound = [0,5];  % 无人机开始起飞到投掷第3枚烟雾弹
t_explode_delay1_bound = [0,7];  % 投掷后爆炸时间间隔范围1 s
t_explode_delay2_bound = [0,7];  % 投掷后爆炸时间间隔范围2 s
t_explode_delay3_bound = [0,7];  % 投掷后爆炸时间间隔范围3 s

missile1_pos0 = [20000, 0, 2000];  % M1
missile2_pos0 = [19000,600,2100];  % M2
missile3_pos0 = [18000,-600,1900]; % M3
fake_target = [0,0,0];

missile_speed = 300;

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

% 约束条件
% ====需要修改====
% 第1个烟雾弹
constraints11 = @(alpha,t_deploy1,drone_speed,t_explode_delay1)  abs(missile1_vel(1)) * (t_deploy1+t_explode_delay1) - 2200 ...
    - sign(alpha) * drone_speed * cos(alpha) * (t_deploy1+t_explode_delay1);
constraints12 = @(alpha,t_deploy1,drone_speed,t_explode_delay1)  abs(missile2_vel(1)) * (t_deploy1+t_explode_delay1) - 2100 ...
    - sign(alpha) * drone_speed * cos(alpha) * (t_deploy1+t_explode_delay1);
constraints13 = @(alpha,t_deploy1,drone_speed,t_explode_delay1)  abs(missile3_vel(1)) * (t_deploy1+t_explode_delay1) - 2000 ...
    - sign(alpha) * drone_speed * cos(alpha) * (t_deploy1+t_explode_delay1);


% 第2个烟雾弹
constraints21 = @(alpha,t_deploy2,drone_speed,t_explode_delay2)  abs(missile1_vel(1)) * (t_deploy2+t_explode_delay2) - 2200 ...
    - sign(alpha) * drone_speed * cos(alpha) * (t_deploy2+t_explode_delay2);
constraints22 = @(alpha,t_deploy2,drone_speed,t_explode_delay2)  abs(missile2_vel(1)) * (t_deploy2+t_explode_delay2) - 2100 ...
    - sign(alpha) * drone_speed * cos(alpha) * (t_deploy2+t_explode_delay2);
constraints23 = @(alpha,t_deploy2,drone_speed,t_explode_delay2)  abs(missile3_vel(1)) * (t_deploy2+t_explode_delay2) - 2000 ...
    - sign(alpha) * drone_speed * cos(alpha) * (t_deploy2+t_explode_delay2);

% 第3个烟雾弹
constraints31 = @(alpha,t_deploy3,drone_speed,t_explode_delay3)  abs(missile1_vel(1)) * (t_deploy3+t_explode_delay3) - 2200 ...
    - sign(alpha) * drone_speed * cos(alpha) * (t_deploy3+t_explode_delay3);
constraints32 = @(alpha,t_deploy3,drone_speed,t_explode_delay3)  abs(missile2_vel(1)) * (t_deploy3+t_explode_delay3) - 2100 ...
    - sign(alpha) * drone_speed * cos(alpha) * (t_deploy3+t_explode_delay3);
constraints33 = @(alpha,t_deploy3,drone_speed,t_explode_delay3)  abs(missile3_vel(1)) * (t_deploy3+t_explode_delay3) - 2000 ...
    - sign(alpha) * drone_speed * cos(alpha) * (t_deploy3+t_explode_delay3);

% 惩罚函数封装
penalty_factor = 1;  % 惩罚因子
fitness_func = @(drone_speed, alpha,t_deploy1, t_explode_delay1,t_deploy2, t_explode_delay2,t_deploy3, t_explode_delay3) ...
    aim_func_1drone_to_3missile(drone_speed, alpha, t_deploy1, t_explode_delay1,t_deploy2, t_explode_delay2,t_deploy3, t_explode_delay3)...
    - penalty_factor * max(0, constraints11(alpha,t_deploy1,drone_speed,t_explode_delay1)) ...
    - penalty_factor * max(0, constraints12(alpha,t_deploy1,drone_speed,t_explode_delay1)) ...
    - penalty_factor * max(0, constraints13(alpha,t_deploy1,drone_speed,t_explode_delay1)) ...
    - penalty_factor * max(0, constraints21(alpha,t_deploy2,drone_speed,t_explode_delay2)) ...
    - penalty_factor * max(0, constraints22(alpha,t_deploy2,drone_speed,t_explode_delay2)) ...
    - penalty_factor * max(0, constraints23(alpha,t_deploy2,drone_speed,t_explode_delay2)) ...
    - penalty_factor * max(0, constraints31(alpha,t_deploy3,drone_speed,t_explode_delay3)) ...
    - penalty_factor * max(0, constraints32(alpha,t_deploy3,drone_speed,t_explode_delay3)) ...
    - penalty_factor * max(0, constraints33(alpha,t_deploy3,drone_speed,t_explode_delay3));


% 粒子群速度限制
v_max = 1;
v_min = -1;

p_best = zeros(N, 1);     % 个体最优适应度
p_best_history = zeros(N, D);  % 储存个体最优值（参数）

g_best = zeros(K, 1);  % 储存每次迭代的群体最优适应度
g_best_history = zeros(K,D);  % 储存每次迭代的群体最优值（参数）

% 初始化
allind_x = zeros(N, D);  % 当前位置
allind_v = zeros(N, D);  % 当前速度

%% PSO迭代
for k = 1:K
    w = w_max - (w_max-w_min)*k/K; % 线性递减权重
    
    for i = 1:N
        if k == 1
            % 随机产生初值解，确保初始解满足约束条件
            allind_x(i,1) = drone_speed_bound(1) + (drone_speed_bound(2)-drone_speed_bound(1))*rand;
            allind_x(i,2) = alpha_bound(1) + (alpha_bound(2)-alpha_bound(1))*rand;
            allind_x(i,3) = t_deploy1_bound(1) + (t_deploy1_bound(2)-t_deploy1_bound(1))*rand;
            allind_x(i,4) = t_explode_delay1_bound(1) + (t_explode_delay1_bound(2)-t_explode_delay1_bound(1))*rand;
            
            % 确保 t_deploy2 >= t_deploy1 + 1
            min_t_deploy2 = max(t_deploy2_bound(1), allind_x(i,3) + 1);
            allind_x(i,5) = min_t_deploy2 + (t_deploy2_bound(2)-min_t_deploy2)*rand;
            
            allind_x(i,6) = t_explode_delay2_bound(1) + (t_explode_delay2_bound(2)-t_explode_delay2_bound(1))*rand;
            
            % 确保 t_deploy3 >= t_deploy2 + 1 (如果需要这个约束)
            min_t_deploy3 = max(t_deploy3_bound(1), allind_x(i,5) + 1);
            allind_x(i,7) = min_t_deploy3 + (t_deploy3_bound(2)-min_t_deploy3)*rand;
            
            allind_x(i,8) = t_explode_delay3_bound(1) + (t_explode_delay3_bound(2)-t_explode_delay3_bound(1))*rand;

            allind_v(i,:) = v_min + (v_max-v_min)*rand(1,D);
            
            % 储存该个体当前的最大适应值
            fit_val = fitness_func(allind_x(i,1), allind_x(i,2), allind_x(i,3), ...
                allind_x(i,4), allind_x(i,5), allind_x(i,6), allind_x(i,7), allind_x(i,8));
            p_best(i) = fit_val;
            p_best_history(i,:) = allind_x(i,:);

            % 初始化群体最优适应度
            [best_val, idx] = max(p_best);
            g_best(k) = best_val;  % 适应度
            g_best_history(k,:) = p_best_history(idx,:);
 
        elseif k > 1
    
            % 更新速度
            allind_v(i,:) = w*allind_v(i,:) + ...
                           c1*rand(1,D).*(p_best_history(i,:) - allind_x(i,:)) + ...
                           c2*rand(1,D).*(g_best_history(k-1,:) - allind_x(i,:));
            
            % 边界限制
            allind_v(i,:) = min(max(allind_v(i,:), v_min), v_max);
            
            % 更新位置
            allind_x(i,:) = allind_x(i,:) + allind_v(i,:);
            
            % 变量边界处理
            allind_x(i,1) = min(max(allind_x(i,1), drone_speed_bound(1)), drone_speed_bound(2));
            allind_x(i,2) = min(max(allind_x(i,2), alpha_bound(1)), alpha_bound(2));
            allind_x(i,3) = min(max(allind_x(i,3), t_deploy1_bound(1)), t_deploy1_bound(2));
            allind_x(i,4) = min(max(allind_x(i,4), t_explode_delay1_bound(1)), t_explode_delay1_bound(2));
            allind_x(i,5) = min(max(allind_x(i,5), t_deploy2_bound(1)), t_deploy2_bound(2));
            allind_x(i,6) = min(max(allind_x(i,6), t_explode_delay2_bound(1)), t_explode_delay2_bound(2));
            allind_x(i,7) = min(max(allind_x(i,7), t_deploy3_bound(1)), t_deploy3_bound(2));
            allind_x(i,8) = min(max(allind_x(i,8), t_explode_delay3_bound(1)), t_explode_delay3_bound(2));
            
            % 修复约束：确保 t_deploy2 >= t_deploy1 + 1
            if allind_x(i,5) < allind_x(i,3) + 1
                % 尝试调整 t_deploy2
                if allind_x(i,3) + 1 <= t_deploy2_bound(2)
                    allind_x(i,5) = allind_x(i,3) + 1;
                else
                    % 如果调整 t_deploy2 会超出上界，则调整 t_deploy1
                    allind_x(i,3) = allind_x(i,5) - 1;
                    % 确保 t_deploy1 不低于下界
                    if allind_x(i,3) < t_deploy1_bound(1)
                        allind_x(i,3) = t_deploy1_bound(1);
                        allind_x(i,5) = allind_x(i,3) + 1; % 再次调整 t_deploy2
                    end
                end
            end
            
            % 如果需要，也可以修复 t_deploy3 >= t_deploy2 + 1 的约束
            if allind_x(i,7) < allind_x(i,5) + 1
                % 类似的修复逻辑
                if allind_x(i,5) + 1 <= t_deploy3_bound(2)
                    allind_x(i,7) = allind_x(i,5) + 1;
                else
                    allind_x(i,5) = allind_x(i,7) - 1;
                    if allind_x(i,5) < t_deploy2_bound(1)
                        allind_x(i,5) = t_deploy2_bound(1);
                        allind_x(i,7) = allind_x(i,5) + 1;
                    end
                end
            end

            % 计算适应度（带罚函数）
            fit_val = fitness_func(allind_x(i,1), allind_x(i,2), allind_x(i,3), ...
                allind_x(i,4), allind_x(i,5), allind_x(i,6), allind_x(i,7), allind_x(i,8));
    
            % 更新个体最优
            if fit_val > p_best(i)
                p_best(i) = fit_val;
                p_best_history(i,:) = allind_x(i,:);
            end
        end
    end
        idxs = randperm(N, mutation_particles);  % 随机选择多个粒子
        for idx = idxs
            % 随机产生初值解，确保初始解满足约束条件
            allind_x(idx,1) = drone_speed_bound(1) + (drone_speed_bound(2)-drone_speed_bound(1))*rand;
            allind_x(idx,2) = alpha_bound(1) + (alpha_bound(2)-alpha_bound(1))*rand;
            allind_x(idx,3) = t_deploy1_bound(1) + (t_deploy1_bound(2)-t_deploy1_bound(1))*rand;
            allind_x(idx,4) = t_explode_delay1_bound(1) + (t_explode_delay1_bound(2)-t_explode_delay1_bound(1))*rand;
            
            % 确保 t_deploy2 >= t_deploy1 + 1
            min_t_deploy2 = max(t_deploy2_bound(1), allind_x(idx,3) + 1);
            allind_x(idx,5) = min_t_deploy2 + (t_deploy2_bound(2)-min_t_deploy2)*rand;
            
            allind_x(idx,6) = t_explode_delay2_bound(1) + (t_explode_delay2_bound(2)-t_explode_delay2_bound(1))*rand;
            
            % 确保 t_deploy3 >= t_deploy2 + 1 (如果需要这个约束)
            min_t_deploy3 = max(t_deploy3_bound(1), allind_x(idx,5) + 1);
            allind_x(idx,7) = min_t_deploy3 + (t_deploy3_bound(2)-min_t_deploy3)*rand;
            
            allind_x(idx,8) = t_explode_delay3_bound(1) + (t_explode_delay3_bound(2)-t_explode_delay3_bound(1))*rand;

            allind_v(idx,:) = v_min + (v_max-v_min)*rand(1,D);
            
            % 更新适应度
            fit_val = fitness_func(allind_x(idx,1), allind_x(idx,2), allind_x(idx,3), allind_x(idx,4), ...
                allind_x(idx,5), allind_x(idx,6), allind_x(idx,7), allind_x(idx,8));
            
            % 检查是否需要更新个体最优
            if fit_val > p_best(idx)
                p_best(idx) = fit_val;
                p_best_history(idx,:) = allind_x(idx,:);
            end  
        end

    % 更新全局最优
    [best_val, idx] = max(p_best);
    g_best(k) = best_val;  % 适应度
    g_best_history(k,:) = p_best_history(idx,:);

end


%% 绘制收敛曲线
figure;
plot(1:K, g_best, 'LineWidth',2);
xlabel('inter num'); ylabel('g best history');
title('PSO'); 
grid on;

[best_fit_val, idx] = max(g_best);
best_fit_var = g_best_history(idx,:);

best_drone_speed = best_fit_var(1);
best_alpha = best_fit_var(2);
best_t_deploy1 = best_fit_var(3);
best_t_explode_delay1 = best_fit_var(4);
best_t_deploy2 = best_fit_var(5);
best_t_explode_delay2 = best_fit_var(6);
best_t_deploy3 = best_fit_var(7);
best_t_explode_delay3 = best_fit_var(8);


[all_shield_time, shield_time1, shield_time2, shield_time3] = aim_func_1drone_to_3missile_2(best_drone_speed, best_alpha, ...
    best_t_deploy1, best_t_explode_delay1,best_t_deploy2, best_t_explode_delay2,...
    best_t_deploy3, best_t_explode_delay3);

fprintf('最优参数：\n');
fprintf('无人机速度 = %.4f m/s\n', best_drone_speed);
fprintf('飞行角度   = %.4f rad\n\n', best_alpha);
fprintf('烟雾弹1\n');
fprintf('飞行时间   = %.4f s\n', best_t_deploy1);
fprintf('爆炸延迟   = %.4f s\n', best_t_explode_delay1);

fprintf('烟雾弹2\n');
fprintf('飞行时间   = %.4f s\n', best_t_deploy2);
fprintf('爆炸延迟   = %.4f s\n', best_t_explode_delay2);

fprintf('烟雾弹3\n');
fprintf('飞行时间   = %.4f s\n', best_t_deploy3);
fprintf('爆炸延迟   = %.4f s\n', best_t_explode_delay3);

fprintf('导弹1被遮挡时间 = %.4f s\n', shield_time1);
fprintf('导弹2被遮挡时间 = %.4f s\n', shield_time2);
fprintf('导弹3被遮挡时间 = %.4f s\n', shield_time3);
fprintf('总遮挡时间 = %.4f s\n', shield_time);




