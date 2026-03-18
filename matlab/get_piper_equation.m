%% Piper 准则下的解析解后三关节角推导脚本

clear; clc;

% 改进 DH 参数 [关节1 to 关节6]
% 注意要用 sym 类型以保持符号计算精度
% [ alpha_{i-1},  a_{i-1},  offset_i,  d_i]
params = [
    0,            0,        0,         0.15;
    sym(pi)/2,    0,        0,         0;
    0,            0.5,      sym(pi)/2, 0;
    -sym(pi)/2,   0.15,     0,         0.5;
    sym(pi)/2,    0,        sym(pi),   0;
    sym(pi)/2,    0,        0,         0;
    ];

% 定义符号变量（纯关节角，不含 offset）
syms q4 q5 q6 real

% 关节4~6的 DH 参数
alphas = params(4:6, 1);   % alpha_{i-1}
offsets = params(4:6, 3);  % offset_i

% 带 offset 的实际旋转角
theta4 = q4 + offsets(1);
theta5 = q5 + offsets(2);
theta6 = q6 + offsets(3); 

% 构建 Modified DH 旋转矩阵
% T = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
% 旋转部分: R = Rx(alpha) * Rz(theta)
getR = @(alpha, theta) [
    cos(theta),              -sin(theta),             0;
    sin(theta)*cos(alpha),    cos(theta)*cos(alpha), -sin(alpha);
    sin(theta)*sin(alpha),    cos(theta)*sin(alpha),  cos(alpha)
    ];

% 各关节旋转矩阵
R45 = simplify(getR(alphas(1), theta4));
R56 = simplify(getR(alphas(2), theta5));
R67 = simplify(getR(alphas(3), theta6));

% 总姿态矩阵 R_36 = R45 * R56 * R67
R_36 = simplify(R45 * R56 * R67);

% 打印矩阵，用于人工核对
disp('==============================================');
disp('       R_36 符号矩阵（带 offset 展开后）');
disp('==============================================');
disp(R_36);
disp(' ');

% 打印每个元素的简化表达式
disp('==============================================');
disp('       R_36 各元素表达式');
disp('==============================================');
R_expr = cell(3, 3);
for i = 1:3
    for j = 1:3
        expr = simplify(R_36(i, j));
        R_expr{i, j} = char(expr);
        fprintf('R(%d,%d) = %s\n', i, j, R_expr{i, j});
    end
end

% 自动分析矩阵结构
disp(' ');
disp('==============================================');
disp('       自动分析 R_36 结构');
disp('==============================================');

% 存储分析结果
single_var_elements = {};  % 只含单个变量的元素
two_var_elements = {};     % 只含两个变量的元素
three_var_elements = {};   % 含三个变量的元素

for i = 1:3
    for j = 1:3
        expr = R_36(i, j);
        vars = symvar(expr);
        var_names = {};
        for v = 1:length(vars)
            var_names{v} = char(vars(v));
        end
        
        if length(vars) == 1
            single_var_elements{end+1} = struct('row', i, 'col', j, 'expr', expr, 'vars', {var_names});
        elseif length(vars) == 2
            two_var_elements{end+1} = struct('row', i, 'col', j, 'expr', expr, 'vars', {var_names});
        elseif length(vars) == 3
            three_var_elements{end+1} = struct('row', i, 'col', j, 'expr', expr, 'vars', {var_names});
        end
    end
end

disp('只含单个变量的元素:');
if isempty(single_var_elements)
    disp('  （无）');
else
    for k = 1:length(single_var_elements)
        e = single_var_elements{k};
        fprintf('  R(%d,%d) = %s, 变量: %s\n', e.row, e.col, char(e.expr), strjoin(e.vars, ', '));
    end
end

disp('只含两个变量的元素:');
if isempty(two_var_elements)
    disp('  （无）');
else
    for k = 1:length(two_var_elements)
        e = two_var_elements{k};
        fprintf('  R(%d,%d) = %s, 变量: %s\n', e.row, e.col, char(e.expr), strjoin(e.vars, ', '));
    end
end

disp('含三个变量的元素:');
if isempty(three_var_elements)
    disp('  （无）');
else
    for k = 1:length(three_var_elements)
        e = three_var_elements{k};
        fprintf('  R(%d,%d) = %s, 变量: %s\n', e.row, e.col, char(e.expr), strjoin(e.vars, ', '));
    end
end

% 自动推导公式
disp(' ');
disp('==============================================');
disp('       自动推导 atan2 公式');
disp('==============================================');

% 定义目标变量
target_vars = {q4, q5, q6};
target_names = {'q4', 'q5', 'q6'};
formulas = cell(1, 3);       % 存储推导出的公式函数
formula_strs = cell(1, 3);   % 存储公式字符串

% 遍历每个目标变量，寻找最佳提取方式
for v = 1:3
    target = target_vars{v};
    target_name = target_names{v};
    
    fprintf('%s 推导:\n', target_name);
    
    found = false;
    
    % 策略1: 寻找只含该变量的元素（最简单情况）
    for k = 1:length(single_var_elements)
        e = single_var_elements{k};
        if isequal(symvar(e.expr), target)
            fprintf('  发现只含 %s 的元素: R(%d,%d) = %s\n', target_name, e.row, e.col, char(e.expr));
            
            % 分析表达式是 cos(q) 还是 sin(q)
            expr = e.expr;
            row = e.row;
            col = e.col;
            
            if isequal(simplify(expr - cos(target)), sym(0))
                fprintf('    R(%d,%d) = cos(%s)\n', row, col, target_name);
                % 需要找同行的两个元素来构造 sin(q)
                other_cols = setdiff(1:3, col);
                c1 = other_cols(1);
                c2 = other_cols(2);
                formulas{v} = @(R) atan2(sqrt(R(row, c1)^2 + R(row, c2)^2), R(row, col));
                formula_strs{v} = sprintf('%s = atan2(sqrt(r%d%d^2 + r%d%d^2), r%d%d)     %% R(%d,%d)=cos(%s)', ...
                    target_name, row, c1, row, c2, row, col, row, col, target_name);
                found = true;
            elseif isequal(simplify(expr - sin(target)), sym(0))
                fprintf('    R(%d,%d) = sin(%s)\n', row, col, target_name);
                % 需要找同行或同列的 cos(q)
                formulas{v} = @(R) asin(R(row, col));
                formula_strs{v} = sprintf('%s = asin(r%d%d)     %% R(%d,%d)=sin(%s)', ...
                    target_name, row, col, row, col, target_name);
                found = true;
            elseif isequal(simplify(expr + cos(target)), sym(0))
                fprintf('    R(%d,%d) = -cos(%s)\n', row, col, target_name);
                other_cols = setdiff(1:3, col);
                c1 = other_cols(1);
                c2 = other_cols(2);
                formulas{v} = @(R) atan2(sqrt(R(row, c1)^2 + R(row, c2)^2), -R(row, col));
                formula_strs{v} = sprintf('%s = atan2(sqrt(r%d%d^2 + r%d%d^2), -r%d%d)     %% R(%d,%d)=-cos(%s)', ...
                    target_name, row, c1, row, c2, row, col, row, col, target_name);
                found = true;
            elseif isequal(simplify(expr + sin(target)), sym(0))
                fprintf('    R(%d,%d) = -sin(%s)\n', row, col, target_name);
                formulas{v} = @(R) asin(-R(row, col));
                formula_strs{v} = sprintf('%s = asin(-r%d%d)     %% R(%d,%d)=-sin(%s)', ...
                    target_name, row, col, row, col, target_name);
                found = true;
            end
            
            if found
                break;
            end
        end
    end
    
    % 策略2: 从两变量元素对中推导
    if ~found
        fprintf('  未找到只含 %s 的元素，尝试从两变量元素推导...\n', target_name);
        
        % 寻找含有该变量的两变量元素
        sin_elem = [];
        cos_elem = [];
        
        for k = 1:length(two_var_elements)
            e = two_var_elements{k};
            expr = e.expr;
            
            if ~ismember(target, symvar(expr))
                continue;
            end
            
            % 检查是否含有 sin(target) 或 cos(target)
            if has(expr, sin(target))
                % 提取系数
                coeff = simplify(expr / sin(target));
                if ~has(coeff, target)  % 确保系数不含 target
                    sin_elem = struct('row', e.row, 'col', e.col, 'expr', expr, 'coeff', coeff);
                    fprintf('    找到 sin(%s) 元素: R(%d,%d) = %s\n', target_name, e.row, e.col, char(expr));
                end
            end
            
            if has(expr, cos(target))
                % 检查正系数
                coeff = simplify(expr / cos(target));
                if ~has(coeff, target)
                    cos_elem = struct('row', e.row, 'col', e.col, 'expr', expr, 'coeff', coeff, 'sign', 1);
                    fprintf('    找到 cos(%s) 元素: R(%d,%d) = %s\n', target_name, e.row, e.col, char(expr));
                end
                % 检查负系数
                coeff = simplify(expr / (-cos(target)));
                if ~has(coeff, target)
                    cos_elem = struct('row', e.row, 'col', e.col, 'expr', expr, 'coeff', coeff, 'sign', -1);
                    fprintf('    找到 -cos(%s) 元素: R(%d,%d) = %s\n', target_name, e.row, e.col, char(expr));
                end
            end
        end
        
        % 如果找到了配对的 sin 和 cos 元素
        if ~isempty(sin_elem) && ~isempty(cos_elem)
            % 检查系数是否相同（可以消去）
            if isequal(simplify(sin_elem.coeff - cos_elem.coeff), sym(0))
                if cos_elem.sign == -1
                    formulas{v} = @(R) atan2(R(sin_elem.row, sin_elem.col), -R(cos_elem.row, cos_elem.col));
                    formula_strs{v} = sprintf('%s = atan2(r%d%d, -r%d%d)     %% sin/(-cos) 配对', ...
                        target_name, sin_elem.row, sin_elem.col, cos_elem.row, cos_elem.col);
                else
                    formulas{v} = @(R) atan2(R(sin_elem.row, sin_elem.col), R(cos_elem.row, cos_elem.col));
                    formula_strs{v} = sprintf('%s = atan2(r%d%d, r%d%d)     %% sin/cos 配对', ...
                        target_name, sin_elem.row, sin_elem.col, cos_elem.row, cos_elem.col);
                end
                found = true;
            elseif isequal(simplify(sin_elem.coeff + cos_elem.coeff), sym(0))
                % 系数相反
                if cos_elem.sign == -1
                    formulas{v} = @(R) atan2(R(sin_elem.row, sin_elem.col), R(cos_elem.row, cos_elem.col));
                    formula_strs{v} = sprintf('%s = atan2(r%d%d, r%d%d)     %% sin/(-cos) 配对(反)', ...
                        target_name, sin_elem.row, sin_elem.col, cos_elem.row, cos_elem.col);
                else
                    formulas{v} = @(R) atan2(R(sin_elem.row, sin_elem.col), -R(cos_elem.row, cos_elem.col));
                    formula_strs{v} = sprintf('%s = atan2(r%d%d, -r%d%d)     %% sin/cos 配对(反)', ...
                        target_name, sin_elem.row, sin_elem.col, cos_elem.row, cos_elem.col);
                end
                found = true;
            end
        end
    end
    
    if ~found
        fprintf('  未能自动推导 %s 的公式\n', target_name);
    end
end

% 数值验证
disp(' ');
disp('==============================================');
disp('       公式验证');
disp('==============================================');

test_cases = [
    1.3816, 0.3768, 0;
    0.5, 0.3, 0.2;
    1.0, 0.5, 0.8;
    -0.8, 0.6, -0.3;
    0, 0.1, 0;
    pi/4, pi/6, pi/3;
];

fprintf('%-8s %-8s %-8s | %-8s %-8s %-8s | %-10s %-10s %-10s\n', ...
    'q4', 'q5', 'q6', 'q4_ext', 'q5_ext', 'q6_ext', 'err_q4', 'err_q5', 'err_q6');
fprintf('%s\n', repmat('-', 1, 90));

all_pass = true;
for tc = 1:size(test_cases, 1)
    q4_t = test_cases(tc, 1);
    q5_t = test_cases(tc, 2);
    q6_t = test_cases(tc, 3);
    
    % 计算 R_36 数值
    R = double(subs(R_36, [q4, q5, q6], [q4_t, q5_t, q6_t]));
    
    % 使用推导的公式提取角度
    q_ext = zeros(1, 3);
    for v = 1:3
        if ~isempty(formulas{v})
            q_ext(v) = formulas{v}(R);
        else
            q_ext(v) = NaN;
        end
    end
    
    % 计算误差
    q_actual = [q4_t, q5_t, q6_t];
    err = q_actual - q_ext;
    
    % 检查误差（考虑 2*pi 周期和多解）
    for i = 1:3
        if abs(err(i)) > 0.001 && abs(abs(err(i)) - 2*pi) > 0.001 && abs(abs(err(i)) - pi) > 0.001
            all_pass = false;
        end
    end
    
    fprintf('%-8.4f %-8.4f %-8.4f | %-8.4f %-8.4f %-8.4f | %-10.6f %-10.6f %-10.6f\n', ...
        q4_t, q5_t, q6_t, q_ext(1), q_ext(2), q_ext(3), err(1), err(2), err(3));
end

% 最终公式总结
disp(' ');
disp('==============================================');
disp('       最终公式总结');
disp('==============================================');
disp('给定 R_36 = R_03'' * R_end，提取后三轴关节角：');
for v = 1:3
    if ~isempty(formula_strs{v})
        fprintf('  %s\n', formula_strs{v});
    else
        fprintf('  %s: 未能自动推导公式\n', target_names{v});
    end
end
disp('其中 rij 表示 R_36 的第 i 行第 j 列元素。');

disp(' ');
disp('==============================================');
if all_pass
    disp('       所有测试用例通过验证！');
else
    disp('       部分测试用例未通过，请检查公式或考虑多解情况。');
end
disp('==============================================');