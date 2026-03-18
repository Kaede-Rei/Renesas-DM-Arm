function main()
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

    q_test = [0, 0, 0, 0, 0, 0];
    T_target = dm_arm.fkine(q_test).T;

    q0 = [0.1, 0.2, -0.1, 0.0, 0.1, 0.0];  % 初始猜测（移植时使用电机当前角度）
    [q_sol, iter, err] = numerical_ik(dm_arm, T_target, q0);

    fprintf('迭代次数: %d\n', iter);
    fprintf('最终误差: %.2e\n', err);
    disp('数值解关节角:'); disp(q_sol);

    T_check = dm_arm.fkine(q_sol).T;
    disp('位姿残差:'); disp(T_target - T_check);
end

%  数值逆解：阻尼最小二乘
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
        %    dq = (J'J + λ²I)⁻¹ · J' · e
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

%  旋转矩阵 → 轴角（用于姿态误差）
%  返回 ω = θ·n̂，模长为旋转角度，方向为旋转轴
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

%  角度归一化到 (-π, π]
function angle = normalize_angle(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

%  struct 字段安全读取
function val = getopt(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end

main();