function [optimal_alpha_bound, optimal_shield_time] = optimize_alpha_bound()
    % 初始区间
    low = -pi/2;
    high = pi/2;
    alpha_bound = [low, high];
    
    % 记录最佳结果
    best_alpha_bound = alpha_bound;
    best_shield_time = find_FY3_alpha(alpha_bound);
    
    % 显示初始状态
    fprintf('初始 alpha_bound: [%.6f, %.6f]\n', alpha_bound(1), alpha_bound(2));
    fprintf('初始 shield_time: %.6f\n', best_shield_time);
    fprintf('初始区间大小: %.6f\n\n', alpha_bound(2) - alpha_bound(1));
    
    % 二分查找迭代参数
    max_iterations = 10;  % 防止无限循环
    tol = 0.2;              % 收敛容差
    
    % 迭代过程
    for i = 1:max_iterations
        fprintf('=== 迭代 %d ===\n', i);
        fprintf('当前 alpha_bound: [%.6f, %.6f]\n', alpha_bound(1), alpha_bound(2));
        fprintf('当前区间大小: %.6f\n', alpha_bound(2) - alpha_bound(1));
        
        % 检查当前区间是否已经足够小
        if alpha_bound(2) - alpha_bound(1) < 1
            fprintf('区间大小已满足条件 (<1)，停止迭代\n');
            break;
        end
        
        % 计算中点
        mid = (alpha_bound(1) + alpha_bound(2)) / 2;
        fprintf('中点: %.6f\n', mid);
        
        % 测试左半区间
        left_alpha_bound = [alpha_bound(1), mid];
        left_shield_time = find_FY3_alpha(left_alpha_bound);
        fprintf('左半区间 [%.6f, %.6f] 的 shield_time: %.6f\n', ...
                left_alpha_bound(1), left_alpha_bound(2), left_shield_time);
        
        % 测试右半区间
        right_alpha_bound = [mid, alpha_bound(2)];
        right_shield_time = find_FY3_alpha(right_alpha_bound);
        fprintf('右半区间 [%.6f, %.6f] 的 shield_time: %.6f\n', ...
                right_alpha_bound(1), right_alpha_bound(2), right_shield_time);
        
        % 选择 shield_time 更大的区间
        if left_shield_time > right_shield_time
            alpha_bound = left_alpha_bound;
            current_shield_time = left_shield_time;
            fprintf('选择左半区间\n');
        else
            alpha_bound = right_alpha_bound;
            current_shield_time = right_shield_time;
            fprintf('选择右半区间\n');
        end
        
        % 更新最佳结果
        if current_shield_time > best_shield_time
            best_shield_time = current_shield_time;
            best_alpha_bound = alpha_bound;
            fprintf('更新最佳结果: shield_time = %.6f\n', best_shield_time);
        end
        
        % 显示当前最佳结果
        fprintf('当前最佳 alpha_bound: [%.6f, %.6f]\n', ...
                best_alpha_bound(1), best_alpha_bound(2));
        fprintf('当前最佳 shield_time: %.6f\n\n', best_shield_time);
        
        % 检查是否收敛
        if alpha_bound(2) - alpha_bound(1) < tol
            fprintf('区间大小已达到容差要求，停止迭代\n');
            break;
        end
        
        % 如果达到最大迭代次数
        if i == max_iterations
            fprintf('已达到最大迭代次数，停止迭代\n');
        end
    end
    
    optimal_alpha_bound = best_alpha_bound;
    optimal_shield_time = best_shield_time;
    
end

% 运行优化
[optimal_alpha_bound, optimal_shield_time] = optimize_alpha_bound();