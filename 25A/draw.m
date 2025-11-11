function draw(shield_time_array, t_start, t_end, dt, num)
t_sim = t_start:dt:t_end;
if isempty(shield_time_array)
    shielding_status = zeros(size(t_sim));
else
    
shielding_status = zeros(size(t_sim));

min_time = min(shield_time_array);
max_time = max(shield_time_array);

[~, min_idx] = min(abs(t_sim - min_time));
[~, max_idx] = min(abs(t_sim - max_time));

shielding_status(min_idx:max_idx) = 1;
end

% 绘制
figure;
plot(t_sim, shielding_status, 'b', 'LineWidth', 2);
xlabel('Time');
ylabel('Blocking Status');
axis equal;
if num ~= 0
    title(sprintf('Obstruction Situation of Smoke Bomb %d', num));
else
    title('The Entire Process of Obstruction');
end
ylim([-0.1 1.1]); 
grid on;

end
