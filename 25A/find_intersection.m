function intersection = find_intersection(P1, P2, x1, y1, z1, t)
    x1_line = P1(1);
    y1_line = P1(2);
    z1_line = P1(3);
    
    x2_line = P2(1);
    y2_line = P2(2);
    z2_line = P2(3);
    
    % 定义目标函数和约束
    objective = @(k) 0;  
    
    % 等式约束：(x - x1)^2 + (y - y1)^2 = (140*t)^2
    nonlcon = @(k) nonlinear_constraints(k, x1_line, y1_line, z1_line, x2_line, y2_line, z2_line, x1, y1, t);
    
    % 不等式约束：z <= z1 已经包含在nonlcon中
    
    % 变量边界
    lb = 0;  % k 通常在 [0,1] 范围内
    ub = 1;
    
    % 求解
    k_initial_guess = 0.5;
    options = optimoptions('fmincon', 'Display', 'off');
    [k_sol, fval] = fmincon(objective, k_initial_guess, [], [], [], [], lb, ub, nonlcon, options);


    % 确保k在0和1之间
    if abs(fval) < 1e-6 && k_sol >= 0 && k_sol <= 1
        intersection = [x1_line + k_sol * (x2_line - x1_line), y1_line + k_sol * (y2_line - y1_line), z1_line + k_sol * (z2_line - z1_line)];
    else
        intersection = NaN(1, 3); % 无解
    end



    % 非线性约束函数
function [c, ceq] = nonlinear_constraints(k, x1_line, y1_line, z1_line, x2_line, y2_line, z2_line, x1, y1, t)
    % 计算线段上的点
    x = x1_line + k * (x2_line - x1_line);
    y = y1_line + k * (y2_line - y1_line);
    z = z1_line + k * (z2_line - z1_line);
    
    % 等式约束
    ceq = (x - x1)^2 + (y - y1)^2 - (140*t)^2;
    
    % 不等式约束：z <= z1
    c = z - z1;  % 注意：fmincon要求 c <= 0，所以 z - z1 <= 0 即 z <= z1
end

end
