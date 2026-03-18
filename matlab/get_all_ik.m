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
    q_test = [0, 0.2512, -0.7536, 1.3816, 0.3768, 0];
    T_test = piper_arm.fkine(q_test).T;
    
    % 获取所有可能的解
    all_solutions = get_all_inverse_kinematics(T_test, 0.1, piper_arm);
    
    disp("====== 逆运动学多解分析 ======");
    disp("原始关节角："); 
    disp(q_test);
    disp(" ");
    
    if ~isempty(all_solutions)
        disp(['找到 ' num2str(size(all_solutions, 1)) ' 组解：']);
        for i = 1:size(all_solutions, 1)
            disp(['解 ' num2str(i) '：']);
            disp(all_solutions(i, :));
            
            % 验证每组解
            T_verify = piper_arm.fkine(all_solutions(i, :)).T;
            pos_error = norm(T_verify(1:3, 4) - T_test(1:3, 4));
            rot_error = norm(T_verify(1:3, 1:3) - T_test(1:3, 1:3), 'fro');
            
            disp(['  位置误差: ' num2str(pos_error)]);
            disp(['  姿态误差: ' num2str(rot_error)]);
            disp(' ');
        end
        
        % 选择最接近原始关节角的解
        [best_sol, idx] = select_closest_solution(all_solutions, q_test);
        disp(['最优解（解 ' num2str(idx) '）：']);
        disp(best_sol);
    else
        disp('未找到有效解（目标可能不可达或处于奇异位形）');
    end
    
    % 测试奇异情况
    test_singularities(piper_arm);
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

% 获取所有逆运动学解
function all_solutions = get_all_inverse_kinematics(T_end, tool_d, robot)
    % DH 参数 (Modified DH)
    params = [
        0,     0,      0,          0.15;
        pi/2,  0,      0,          0;
        0,     0.5,    pi/2,       0;
        -pi/2, 0.15,   0,          0.5;
        pi/2,  0,      pi,         0;
        pi/2,  0,      0,          0;
        ];

    all_solutions = [];
    candidate_count = 0;
    
    % 提取末端位置和 Z 轴方向
    P_end = T_end(1:3, 4);
    Z_end = T_end(1:3, 3);

    % 计算腕部中心 P_c
    P_c = P_end - Z_end * tool_d;
    x = P_c(1);
    y = P_c(2);
    z = P_c(3);

    d0 = params(1, 4);
    A = params(3, 2);
    B = sqrt(params(4, 2)^2 + params(4, 4)^2);
    C = sqrt(x^2 + y^2 + (z-d0)^2);

    % 检查目标位置是否可达
    if abs((C^2 - A^2 - B^2) / (2 * A * B)) > 1
        warning('目标位置不可达');
        return;
    end
    
    disp("====== 调试信息：搜索所有候选解 ======");

    % 解1：theta0 的两个解（肩部左右旋转）
    theta0_sol1 = atan2(y, x);
    theta0_sol2 = normalize_angle(atan2(y, x) + pi);
    theta0_solutions = [theta0_sol1, theta0_sol2];
    
    for idx0 = 1:length(theta0_solutions)
        theta0 = normalize_angle(theta0_solutions(idx0));
        
        % theta0改变时，需要调整xy平面的投影距离计算
        % 对于theta0 + pi的解，实际上是从后面接近目标
        if idx0 == 2
            % 反向接近，使用负的xy投影
            r_xy = -sqrt(x^2 + y^2);
        else
            r_xy = sqrt(x^2 + y^2);
        end
        
        % 解2：theta2 的两个解（肘部向上/向下）
        cos_theta2_part = (C^2 - A^2 - B^2) / (2 * A * B);
        if abs(cos_theta2_part) > 1
            cos_theta2_part = sign(cos_theta2_part);
        end
        
        beta = atan2(params(4, 4), params(4, 2));
        acos_part = acos(cos_theta2_part);
        
        theta2_sol1 = acos_part - beta;
        theta2_sol2 = -acos_part - beta;
        
        theta2_solutions = [theta2_sol1, theta2_sol2];
        
        for idx2 = 1:length(theta2_solutions)
            theta2 = theta2_solutions(idx2);
            
            % 计算 theta1
            cos_angle = (A^2 + C^2 - B^2) / (2 * A * C);
            if abs(cos_angle) > 1
                cos_angle = sign(cos_angle);
            end
            
            % 根据肘部配置选择正确的theta1计算方式
            if idx2 == 1
                % 肘部向上：原始公式
                theta1 = atan2((z-d0), r_xy) - acos(cos_angle);
            else
                % 肘部向下：角度符号相反
                theta1 = atan2((z-d0), r_xy) + acos(cos_angle);
            end
            
            % 计算 T_02
            T_02 = eye(4);
            for i = 1:3
                alpha = params(i, 1);
                a = params(i, 2);
                theta = [theta0, theta1, theta2];
                d = params(i, 4);
                Ti = get_tf_matrix(alpha, a, theta(i), d);
                T_02 = T_02 * Ti;
            end

            R_02 = T_02(1:3, 1:3);
            R_end = T_end(1:3, 1:3);
            R_25 = R_02' * R_end;

            r_13 = R_25(1, 3);
            r_23 = R_25(2, 3);
            r_33 = R_25(3, 3);
            r_21 = R_25(2, 1);
            r_22 = R_25(2, 2);

            % 解3：theta4 的两个解
            theta4_sol1 = acos(max(-1, min(1, r_23)));
            theta4_sol2 = -theta4_sol1;
            
            theta4_solutions = [theta4_sol1, theta4_sol2];
            
            for idx4 = 1:length(theta4_solutions)
                theta4 = theta4_solutions(idx4);
                candidate_count = candidate_count + 1;
                
                % 检查腕部奇异
                if abs(sin(theta4)) < 1e-6
                    theta5 = 0;
                    if abs(theta4) < 1e-6
                        theta3 = atan2(R_25(1,2), R_25(1,1));
                    else
                        theta3 = atan2(-R_25(1,2), -R_25(1,1));
                    end
                else
                    % 根据theta4的符号选择正确的atan2
                    % 当sin(theta4) > 0时使用原公式
                    % 当sin(theta4) < 0时需要翻转符号
                    if sin(theta4) > 0
                        theta3 = atan2(r_33, -r_13);
                        theta5 = atan2(r_22, -r_21);
                    else
                        % theta4为负时，sin(theta4)为负
                        theta3 = atan2(-r_33, r_13);
                        theta5 = atan2(-r_22, r_21);
                    end
                end

                % 减去offset并归一化
                q = [theta0, theta1, theta2, theta3, theta4, theta5];
                
                for i = 1:length(q)
                    if i <= 3
                        q(i) = q(i) - params(i, 3);
                    end
                    q(i) = normalize_angle(q(i));
                end
                
                % 验证解的有效性
                T_check = robot.fkine(q).T;
                pos_err = norm(T_check(1:3, 4) - T_end(1:3, 4));
                rot_err = norm(T_check(1:3, 1:3) - T_end(1:3, 1:3), 'fro');
                
                fprintf('候选解 %d (theta0_idx=%d, theta2_idx=%d, theta4_idx=%d): 位置误差=%.6f, 姿态误差=%.6f\n', ...
                        candidate_count, idx0, idx2, idx4, pos_err, rot_err);

                if pos_err < 0.01 && rot_err < 0.1
                    all_solutions = [all_solutions; q];
                    fprintf('  ✓ 接受此解\n');
                else
                    fprintf('  ✗ 误差过大，拒绝\n');
                end
            end
        end
    end
    
    disp(" ");
    
    % 去除重复解
    if ~isempty(all_solutions)
        all_solutions = unique(round(all_solutions * 1e4) / 1e4, 'rows');
    end
end

% 选择最接近参考关节角的解
function [best_sol, best_idx] = select_closest_solution(all_solutions, q_ref)
    min_dist = inf;
    best_sol = [];
    best_idx = 1;
    
    for i = 1:size(all_solutions, 1)
        dist = norm(all_solutions(i, :) - q_ref);
        if dist < min_dist
            min_dist = dist;
            best_sol = all_solutions(i, :);
            best_idx = i;
        end
    end
end

% 测试奇异情况
function test_singularities(robot)
    disp(" ");
    disp("====== 奇异位形测试 ======");
    disp(" ");
    
    % 测试1：完全伸展（边界奇异）
    disp("1. 完全伸展位形：");
    q_stretch = [0, 0, 0, 0, 0, 0];
    T_stretch = robot.fkine(q_stretch).T;
    disp(['   末端位置: ' mat2str(T_stretch(1:3, 4)', 3)]);
    J = robot.jacob0(q_stretch);
    disp(['   雅可比条件数: ' num2str(cond(J), '%.2e')]);
    disp(['   可操作度: ' num2str(sqrt(abs(det(J*J'))), '%.2e')]);
    disp('   类型: 边界奇异 - 机械臂完全伸展，失去1个自由度');
    
    % 测试2：腕部奇异（theta4 = 0）
    disp(" ");
    disp("2. 腕部奇异（theta4 = 0）：");
    q_wrist = [0, pi/4, -pi/4, 0, 0, 0];
    T_wrist = robot.fkine(q_wrist).T;
    disp(['   末端位置: ' mat2str(T_wrist(1:3, 4)', 3)]);
    J = robot.jacob0(q_wrist);
    disp(['   雅可比条件数: ' num2str(cond(J), '%.2e')]);
    disp(['   可操作度: ' num2str(sqrt(abs(det(J*J'))), '%.2e')]);
    disp('   类型: 腕部奇异 - 关节4,5,6共线，theta3和theta5不唯一');
    
    % 测试3：肩部奇异
    disp(" ");
    disp("3. 肩部奇异（腕部中心在z轴上）：");
    q_shoulder = [0, pi/2, -pi/2, 0, pi/2, 0];
    T_shoulder = robot.fkine(q_shoulder).T;
    P_wrist = T_shoulder(1:3, 4) - T_shoulder(1:3, 3) * 0.1;
    disp(['   末端位置: ' mat2str(T_shoulder(1:3, 4)', 3)]);
    disp(['   腕部位置: ' mat2str(P_wrist', 3)]);
    J = robot.jacob0(q_shoulder);
    disp(['   雅可比条件数: ' num2str(cond(J), '%.2e')]);
    disp(['   可操作度: ' num2str(sqrt(abs(det(J*J'))), '%.2e')]);
    disp('   类型: 肩部奇异 - 腕部在基座z轴上，theta0失去约束');
    
    % 测试4：正常位形
    disp(" ");
    disp("4. 正常位形（对比）：");
    q_normal = [pi/6, pi/4, -pi/3, pi/4, pi/4, pi/6];
    T_normal = robot.fkine(q_normal).T;
    disp(['   末端位置: ' mat2str(T_normal(1:3, 4)', 3)]);
    J = robot.jacob0(q_normal);
    disp(['   雅可比条件数: ' num2str(cond(J), '%.2e')]);
    disp(['   可操作度: ' num2str(sqrt(abs(det(J*J'))), '%.2e')]);
    disp('   类型: 正常位形 - 远离奇异');
    
    disp(" ");
    disp("====== 奇异性判断标准 ======");
    disp("雅可比条件数: > 1e10 表示接近奇异");
    disp("可操作度: 接近0表示奇异，正常范围 0.01~0.1");
    disp(" ");
    disp("====== 多解情况总结 ======");
    disp("理论上6自由度机械臂最多有 2×2×2 = 8 组逆解：");
    disp("  1. theta0: 肩部左/右 (2个解)");
    disp("  2. theta2: 肘部向上/向下 (2个解)");
    disp("  3. theta4: 腕部翻转 (2个解)");
    disp("实际解的数量取决于关节限位和奇异位形");
end

main();