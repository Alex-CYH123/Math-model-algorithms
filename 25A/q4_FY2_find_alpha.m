clc,clear;
function optimal_alpha = find_optimal_alpha(func)
    low = 0;
    high = pi;
    tolerance = 0.01;
    
    % 三分法迭代
    while (high - low) > tolerance
        mid1 = low + (high - low) / 3;
        mid2 = high - (high - low) / 3;
        
        % 计算两个中间点的函数值
        f_mid1 = func(mid1);
        f_mid2 = func(mid2);
        
        % 比较函数值，缩小搜索区间
        if f_mid1 < f_mid2
            low = mid1;
        else
            high = mid2;
        end
    end
    
    % 返回区间中点作为最优解
    optimal_alpha = (low + high) / 2;
end


% 寻找最优 alpha
optimal_alpha = find_optimal_alpha();
optimal_value = example_func(optimal_alpha);

fprintf('最优 alpha 值: %.4f rad\n', optimal_alpha);
fprintf('对应的 shield_time: %.4f\n', optimal_value);
