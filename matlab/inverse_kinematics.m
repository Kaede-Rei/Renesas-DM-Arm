function main()
    % Piper 机械臂模型
    L0 = Link('alpha', 0,      'a', 0,     'offset', 0,        'd', 0.15,  'modified');
    L1 = Link('alpha', pi/2,   'a', 0,     'offset', 0,        'd', 0,     'modified');
    L2 = Link('alpha', 0,      'a', 0.5,   'offset', pi/2,     'd', 0,     'modified');
    L3 = Link('alpha', -pi/2,  'a', 0.15,  'offset', 0,        'd', 0.5,   'modified');
    L4 = Link('alpha', pi/2,   'a', 0,     'offset', pi,       'd', 0,     'modified');
    L5 = Link('alpha', pi/2,   'a', 0,     'offset', 0,        'd', 0,     'modified');

    piper_arm = SerialLink([L0 L1 L2 L3 L4 L5], 'name', 'piper');
    piper_arm.tool = transl(0, 0, 0.1);

    piper_arm.plot(zeros(1,6));
    piper_arm.teach;

    % 测试逆解
    % q_test = [0, 0.2512, -0.7536, 1.3816, 0.3768, 0];
    q_test = [-1.2, 1.0, -0.5, 0.93, 1.23, 0.22];
    T_test = piper_arm.fkine(q_test).T;
    
    all_solutions = get_inverse_kinematics_as(T_test, 0.1);
    % disp("找到的逆解数量："); disp(size(all_solutions, 1));
    fprintf('找到的逆解数量：%d\n', size(all_solutions, 1));

    for sol_idx = 1:size(all_solutions, 1)
        q_inverse = all_solutions(sol_idx, :);
        disp('------------------------------');
        disp("第 " + sol_idx + " 个解："); disp(q_inverse);
        T_inverse = piper_arm.fkine(q_inverse).T;

        disp("实际的关节角："); disp(q_test);
        disp("解析解计算的关节角："); disp(q_inverse);
        disp("关节角差异："); disp(q_test - q_inverse);
        disp("实际的末端位姿："); disp(T_test);
        disp("解析解计算的末端位姿："); disp(T_inverse);
        disp("末端位姿差异："); disp(T_test - T_inverse);
        disp("实际腕部中心位置："); disp(T_test(1:3,4) - T_test(1:3,3)*0.1);
        disp("解析解计算的腕部中心位置："); disp(T_inverse(1:3,4) - T_inverse(1:3,3)*0.1);
        disp(' ');
    end
end

% 计算齐次变换矩阵
function T = get_tf_matrix(alpha, a, theta, d)
    T = [
        cos(theta),             -sin(theta),            0,              a;
        sin(theta)*cos(alpha),  cos(theta)*cos(alpha),  -sin(alpha),   -sin(alpha)*d;
        sin(theta)*sin(alpha),  cos(theta)*sin(alpha),  cos(alpha),    cos(alpha)*d;
        0,                      0,                      0,              1
        ];
end

% 角度归一化
function angle = normalize_angle(angle)
    while angle > pi
        angle = angle - 2 * pi;
    end
    while angle <= -pi
        angle = angle + 2 * pi;
    end
end

% 解析解
function all_sols = get_inverse_kinematics_as(T_end, tool_d)
    % DH 参数
    % alpha,    a,    offset,       d
    params = [
        0,     0,      0,          0.15;
        pi/2,  0,      0,          0;
        0,     0.5,    pi/2,       0;
        -pi/2, 0.15,   0,          0.5;
        pi/2,  0,      pi,         0;
        pi/2,  0,      0,          0;
        ];

    % 初始化解集
    all_sols = [];

    % 提取末端位置和 Z 轴方向
    P_end = T_end(1:3, 4);
    Z_end = T_end(1:3, 3);

    % 计算腕部中心 P_c 和三角形边长
    P_c = P_end - Z_end * tool_d;
    x = P_c(1);
    y = P_c(2);
    z = P_c(3);

    d0 = params(1, 4);
    A = params(3, 2);
    B = sqrt(params(4, 2)^2 + params(4, 4)^2);
    C = sqrt(x^2 + y^2 + (z-d0)^2);

    % 检查目标位置是否可达
    if(abs((C^2 - A^2 - B^2) / (2 * A * B)) > 1)
        error('目标位置不可达');
    end

    % 肩关节两组解：向左 + 向右
    theta0_sol1 = atan2(y, x);
    theta0_sol2 = normalize_angle(atan2(y, x) + pi);
    theta0_sols = [theta0_sol1, theta0_sol2];

    for i = 1:2
        theta0 = theta0_sols(i);

        if i == 1
            r = sqrt(x^2 + y^2);   % 向左
        else
            r = -sqrt(x^2 + y^2);  % 向右
        end

        % 肘关节两组解：向上 + 向下
        theta2_cos_part = (C^2 - A^2 - B^2) / (2 * A * B);
        if abs(theta2_cos_part) > 1
            theta2_cos_part = sign(theta2_cos_part);
        end
        theta2_acos_part = acos(theta2_cos_part);
        theta2_sol1 = theta2_acos_part - atan2(params(4, 4), params(4, 2));
        theta2_sol2 = -theta2_acos_part - atan2(params(4, 4), params(4, 2));
        theta2_sols = [theta2_sol1, theta2_sol2];

        for j = 1:2
            theta2 = theta2_sols(j);

            % 计算 theta1
            if j == 1
                % 向上
                theta1 = atan2((z-d0), r) - acos((A^2 + C^2 - B^2) / (2 * A * C));
            else
                % 向下
                theta1 = atan2((z-d0), r) + acos((A^2 + C^2 - B^2) / (2 * A * C));
            end

            % 计算末端姿态相关的关节角 theta3, theta4, theta5
            T_02 = eye(4);
            for t_idx = 1:3
                alpha = params(t_idx, 1);
                a = params(t_idx, 2);
                theta = [theta0, theta1, theta2];
                d = params(t_idx, 4);

                Ti = get_tf_matrix(alpha, a, theta(t_idx), d);
                T_02 = T_02 * Ti;
            end

            R_02 = T_02(1:3, 1:3);
            R_end = T_end(1:3, 1:3);
            R_25 = R_02' * R_end;

            % 腕关节两个解：向内 + 向外
            theta4_sol1 = atan2(sqrt(R_25(2,1)^2 + R_25(2,2)^2), R_25(2,3));
            theta4_sol2 = -atan2(sqrt(R_25(2,1)^2 + R_25(2,2)^2), R_25(2,3));
            theta4_sols = [theta4_sol1, theta4_sol2];

            for k = 1:2
                theta4 = theta4_sols(k);

                % 计算 theta3 和 theta5
                if sin(theta4) > 0
                    theta3 = atan2(R_25(3,3), -R_25(1,3));
                    theta5 = atan2(R_25(2,2), -R_25(2,1));
                else
                    theta3 = atan2(-R_25(3,3), R_25(1,3));
                    theta5 = atan2(-R_25(2,2), R_25(2,1));
                end

                % 前三轴关节角减去偏移量，归一化所有关节角
                q = [theta0, theta1, theta2, theta3, theta4, theta5];

                for q_idx = 1:length(q)
                    if q_idx <= 3
                        q(q_idx) = q(q_idx) - params(q_idx, 3);
                    end
                    q(q_idx) = normalize_angle(q(q_idx));
                end

                all_sols = [all_sols; q];
            end
        end
    end
end

main();