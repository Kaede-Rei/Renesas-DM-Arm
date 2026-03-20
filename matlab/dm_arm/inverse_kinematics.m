function main()
    % 初始化模型
    clc; clearvars; close all;
    
    % 相关参数
    dx = 0.244004;
    dy = 0.059971;
    phi = atan2(dy, dx);       % 倾斜角
    a3 = sqrt(dx^2 + dy^2);    % 公垂线长度 (0.25127)

    % MDH 参数: Link('alpha', alpha, 'a', a, 'd', d, 'offset', offset, 'modified')
    L1 = Link('alpha', 0,      'a', 0,        'd', 0.1,    'offset', pi/2,     'modified');
    L2 = Link('alpha', -pi/2,  'a', 0.02,     'd', 0,      'offset', pi,       'modified');
    L3 = Link('alpha', pi,     'a', 0.264,    'd', 0,      'offset', phi - pi, 'modified');
    L4 = Link('alpha', 0,      'a', a3,       'd', 0,      'offset', -phi,     'modified');
    L5 = Link('alpha', pi/2,   'a', 0.061868, 'd', 0,      'offset', pi/2,     'modified');
    L6 = Link('alpha', pi/2,   'a', 0,        'd', 0.19,   'offset', 0,        'modified');

    % 添加关节运动限位
    L1.qlim = [-2.0944, 2.0944];
    L2.qlim = [0, pi];
    L3.qlim = [0, 1.5*pi];
    L4.qlim = [-pi/2, pi/2];
    L5.qlim = [-pi/2, pi/2];
    L6.qlim = [-pi, pi];

    dm_arm = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'DM-Arm');
    dm_arm.tool = trotx(pi);

    figure;
    dm_arm.plot(zeros(1,6));
    dm_arm.teach;

    % 设置目标位姿
    q_target = [0.0, 0.0, 0.01, 0.0, 0.0, 0.0];
    T_target = dm_arm.fkine(q_target).T;
    T_target(1:3, 4) = [0.0; 0.2; 0.3];

    disp('-----------------------------------');
    disp('正在寻找 DM-Arm 的所有可能逆解...');
    
    % 调用多解数值搜索
    all_sols = get_multiple_ik_numerical(dm_arm, T_target);
    
    % 打印结果验证
    for i = 1:size(all_sols, 1)
        q_sol = all_sols(i, :);
        T_check = dm_arm.fkine(q_sol).T;
        err_pos = norm(T_target(1:3,4) - T_check(1:3,4));
        
        fprintf('-----------------------------------\n');
        fprintf('解 %d:\n', i);
        disp(q_sol);
        fprintf('位置残差: %.2e m\n', err_pos);
    end
end


% 基于多起点的数值逆解搜索
function unique_sols = get_multiple_ik_numerical(robot, T_target)
    unique_sols = [];
    
    % 生成初始猜测种子 (Seeds)
    % 这里模拟解析解的 8 种典型象限姿态：肩(前/后) x 肘(上/下) x 腕(翻/不翻)
    % 加上一些随机种子以防陷入局部极小值
    q_seeds = generate_heuristic_seeds(robot);
    
    options.max_iter = 200;
    options.tol_pos = 1e-5;
    options.tol_ori = 1e-4;
    
    for i = 1 : size(q_seeds, 1)
        q0 = q_seeds(i, :);
        
        % 调用阻尼最小二乘求解器
        [q_sol, iter, err] = numerical_ik(robot, T_target, q0, options);
        
        % 过滤：如果收敛成功
        if err < 1e-4
            % 过滤去重：检查是否与已找到的解重复
            if is_unique_solution(unique_sols, q_sol, 1e-3)
                unique_sols = [unique_sols; q_sol];
            end
        end
    end

    fprintf('迭代 %d 次，找到 %d 组唯一解！\n', iter, size(unique_sols, 1));
end

% 生成覆盖关节空间的启发式种子
function seeds = generate_heuristic_seeds(robot)
    seeds = [];
    
    % 提取限位
    qmin = [robot.links.qlim]; qmin = qmin(1:2:end);
    qmax = [robot.links.qlim]; qmax = qmax(2:2:end);
    
    % 构造 8 组典型的拓扑极限姿态
    shoulder = [0, pi];       % 肩部向前/向后
    elbow = [pi/4, 3*pi/4];   % 肘部上凸/下凹
    wrist = [pi/4, -pi/4];    % 腕部翻转
    
    for s = shoulder
        for e = elbow
            for w = wrist
                % 构造一个基础猜测，其他关节置 0
                q_guess = [s, e, 0, w, 0, 0];
                % 保证在限位内
                q_guess = max(qmin, min(qmax, q_guess));
                seeds = [seeds; q_guess];
            end
        end
    end
    
    % 补充 8 个随机种子，保证空间覆盖率 (蒙特卡洛思想)
    for i = 1:8
        q_rand = qmin + rand(1, 6) .* (qmax - qmin);
        seeds = [seeds; q_rand];
    end
end

% 检查新解是否与已有解重复
function is_unique = is_unique_solution(existing_sols, new_sol, tol)
    if isempty(existing_sols)
        is_unique = true;
        return;
    end
    
    is_unique = true;
    for i = 1:size(existing_sols, 1)
        % 计算关节角差异，考虑 2pi 环绕问题
        diff = existing_sols(i, :) - new_sol;
        diff = mod(diff + pi, 2*pi) - pi; 
        
        if norm(diff) < tol
            is_unique = false;
            break;
        end
    end
end

% 数值逆解：阻尼最小二乘
function [q, iter, final_err] = numerical_ik(robot, T_target, q0, options)

    % 默认参数
    if nargin < 4, options = struct(); end
    max_iter  = getopt(options, 'max_iter',  500);
    tol_pos   = getopt(options, 'tol_pos',   1e-6);   % 位置收敛阈值 [m]
    tol_ori   = getopt(options, 'tol_ori',   1e-5);   % 姿态收敛阈值 [rad]
    lambda    = getopt(options, 'lambda',    0.05);   % 阻尼系数
    step_max  = getopt(options, 'step_max',  0.1);    % 单步最大关节增量 [rad]

    q = q0(:)';   % 行向量
    n = length(q);
    R_target = T_target(1:3, 1:3);
    p_target = T_target(1:3, 4);

    for iter = 1:max_iter

        % 当前正解
        T_cur = robot.fkine(q).T;
        R_cur = T_cur(1:3, 1:3);
        p_cur = T_cur(1:3, 4);

        % 误差向量 e (6x1)
        e_pos = p_target - p_cur;                          % 位置误差 3x1

        R_err = R_target * R_cur';                         % 姿态误差旋转矩阵
        e_ori = rot2axisangle(R_err);                      % 轴角表示 3x1

        e = [e_pos; e_ori];

        % 收敛检测
        if norm(e_pos) < tol_pos && norm(e_ori) < tol_ori
            break;
        end

        % 雅可比矩阵
        J = robot.jacob0(q);                               % 6x6

        % 阻尼最小二乘求解关节增量
        % dq = (J'J + λ²I)¹ · J' · e
        A  = J' * J + lambda^2 * eye(n);
        dq = A \ (J' * e);                                 % 6x1

        % 步长限制（防止发散）
        dq_max = max(abs(dq));
        if dq_max > step_max
            dq = dq * (step_max / dq_max);
        end

        % 更新关节角
        q = q + dq';

        % 关节角归一化 + 限位夹紧
        for i = 1:n
            q(i) = normalize_angle(q(i));
            ql = robot.links(i).qlim;
            q(i) = max(ql(1), min(ql(2), q(i)));
        end
    end

    final_err = norm(e);
end

% 旋转矩阵 -> 轴角（用于姿态误差）
% 返回 ω = θ·n̂，模长为旋转角度，方向为旋转轴
function omega = rot2axisangle(R)
    % 利用反对称部分提取轴角（小角度近似下即 0.5 * [R32 - R23; R13 - R31; R21 - R12]）
    cos_theta = (trace(R) - 1) / 2;
    cos_theta = max(-1, min(1, cos_theta));   % 数值夹紧
    theta = acos(cos_theta);

    if abs(theta) < 1e-10
        omega = [0; 0; 0];
    elseif abs(theta - pi) < 1e-10
        % 180° 奇异处理
        [~, idx] = max(diag(R));
        n = (R(:, idx) + eye(3)*idx) / norm(R(:, idx) + eye(3)*idx);
        omega = theta * n;
    else
        skew = (R - R') / (2 * sin(theta));
        omega = theta * [skew(3,2); skew(1,3); skew(2,1)];
    end
end

% 角度归一化到 (-π, π]
function angle = normalize_angle(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

% struct 字段安全读取
function val = getopt(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end

main();