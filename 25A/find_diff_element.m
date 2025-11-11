function grouped_values = find_diff_element(shield_time1_array, shield_time2_array, shield_time3_array, tolerance)

    % 输入数组全为空
    if isempty(shield_time1_array) && isempty(shield_time2_array) && isempty(shield_time3_array)
        grouped_values = [];
        return;
    end

    % 除去完全相同的元素
    all_values = unique([shield_time1_array(:); shield_time2_array(:); shield_time3_array(:)]);

    % 如果all_values为空，则返回0
    if isempty(all_values)
        grouped_values = [];
        return;
    end

    % 初始化代表值数组
    grouped_values = all_values(1); % 从第一个元素开始

    % 遍历所有值
    for i = 2:length(all_values)
        current_value = all_values(i);
        % 检查当前值是否与grouped_values中任何一个代表值的差在容差范围内
        is_in_tolerance = false;
        for j = 1:length(grouped_values)
            if abs(current_value - grouped_values(j)) < tolerance
                is_in_tolerance = true;
                break;
            end
        end
        % 如果当前值与所有代表值的差都大于容差，则将其加入代表值数组
        if ~is_in_tolerance
            grouped_values = [grouped_values, current_value];
        end
    end


end