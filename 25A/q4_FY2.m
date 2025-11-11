clc,clear;

% 参数设置
w_max = 0.6;
w_min = 0.4;
N = 200;       % 粒子数
c1 = 2;
c2 = 1.2;
D = 4;         % 共需要4个变量
K = 300;       % 最大迭代次数
mutation_prob = 0.05;  
% 变量范围
drone_speed_bound = [70,140];  % 无人机速度范围
alpha_bound = [pi/4, pi];  % 无人机飞行角度范围
t_deploy_bound = [0,10];  % 无人机飞行时间范围
t_explode_delay_bound = [0,15];  % 投掷后爆炸时间间隔范围 s

missile_pos0 = [20000, 0, 2000];  % M1
fake_target = [0,0,0];


% 导弹
missile_dir = fake_target - missile_pos0;
missile_dir_norm = norm(missile_dir);  % 导弹的飞行方向，指向假目标
missile_speed = 300;  % m/s
missile_vel = missile_speed * (missile_dir / missile_dir_norm);  % 导弹的速度

% 约束条件
constraints1 = @(alpha,t_deploy,drone_speed,t_explode_delay)  abs(missile_vel(1)) * (t_deploy+t_explode_delay) - 8000 ...
    - drone_speed * cos(alpha) * (t_deploy+t_explode_delay);

penalty_factor = 1e-2;  % 惩罚因子
fitness_func = @(drone_speed, alpha, t_deploy, t_explode_delay) ...
    aim_func_q4_FY2(drone_speed, alpha, t_deploy, t_explode_delay)...
    - penalty_factor * max(0, constraints1(alpha,t_deploy,drone_speed,t_explode_delay));


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
            % 随机产生初值解
            allind_x(i,1) = drone_speed_bound(1) + (drone_speed_bound(2)-drone_speed_bound(1))*rand;
            allind_x(i,2) = alpha_bound(1) + (alpha_bound(2)-alpha_bound(1))*rand;
            allind_x(i,3) = t_deploy_bound(1) + (t_deploy_bound(2)-t_deploy_bound(1))*rand;
            allind_x(i,4) = t_explode_delay_bound(1) + (t_explode_delay_bound(2)-t_explode_delay_bound(1))*rand;
        
            allind_v(i,:) = v_min + (v_max-v_min)*rand(1,D);
            
            % 储存该个体当前的最大适应值
            fit_val = fitness_func(allind_x(i,1), allind_x(i,2), allind_x(i,3), allind_x(i,4));
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
                           c2*rand(1,D).*(g_best_history(k) - allind_x(i,:));
            
            % 边界限制
            allind_v(i,:) = min(max(allind_v(i,:), v_min), v_max);
            
            % 更新位置
            allind_x(i,:) = allind_x(i,:) + allind_v(i,:);
            
            % 变量边界处理
            allind_x(i,1) = min(max(allind_x(i,1), drone_speed_bound(1)), drone_speed_bound(2));
            allind_x(i,2) = min(max(allind_x(i,2), alpha_bound(1)), alpha_bound(2));
            allind_x(i,3) = min(max(allind_x(i,3), t_deploy_bound(1)), t_deploy_bound(2));
            allind_x(i,4) = min(max(allind_x(i,4), t_explode_delay_bound(1)), t_explode_delay_bound(2));
            % 计算适应度（带罚函数）
            fit_val = fitness_func(allind_x(i,1), allind_x(i,2), allind_x(i,3), allind_x(i,4));
    
            % 更新个体最优
            if fit_val > p_best(i)
                p_best(i) = fit_val;
                p_best_history(i,:) = allind_x(i,:);
            end
        end
    end
    % 随机变异：以mutation_prob概率重置随机粒子的位置和速度
    if rand() < mutation_prob
        idx = randi(N);  % 随机选择一个粒子
        % 随机产生值
        allind_x(idx,1) = drone_speed_bound(1) + (drone_speed_bound(2)-drone_speed_bound(1))*rand;
        allind_x(idx,2) = alpha_bound(1) + (alpha_bound(2)-alpha_bound(1))*rand;
        allind_x(idx,3) = t_deploy_bound(1) + (t_deploy_bound(2)-t_deploy_bound(1))*rand;
        allind_x(idx,4) = t_explode_delay_bound(1) + (t_explode_delay_bound(2)-t_explode_delay_bound(1))*rand;

        allind_v(idx,:) = v_min + (v_max-v_min)*rand(1,D);
        % 更新适应度
        fit_val = fitness_func(allind_x(i,1), allind_x(i,2), allind_x(i,3), allind_x(i,4));
        
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
best_t_deploy = best_fit_var(3);
best_t_explode_delay = best_fit_var(4);
shield_time = aim_func_q4_FY2(best_drone_speed, best_alpha, best_t_deploy, best_t_explode_delay);

fprintf('FY2最优参数：\n');
fprintf('无人机速度 = %.4f m/s\n', best_drone_speed);
fprintf('飞行角度   = %.4f rad\n', best_alpha);
fprintf('飞行时间   = %.4f s\n', best_t_deploy);
fprintf('爆炸延迟   = %.4f s\n', best_t_explode_delay);
fprintf('最大遮挡时间 = %.4f s\n', shield_time);


