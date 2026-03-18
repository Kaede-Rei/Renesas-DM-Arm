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

    figure;
    dm_arm.plot(zeros(1,6));
    dm_arm.teach;

    q_test = [0.2, 1.2, 1.5, 0.5, -0.4, 0.2];

    T_toolbox = dm_arm.fkine(q_test);
    T_custom = get_forward_kinematics(q_test, dm_arm);
    [x, y, z] = get_end_coor(q_test, dm_arm);

    disp("工具箱结果："); disp(T_toolbox.T);
    disp("自定义函数结果："); disp(T_custom);
    disp("结果差异："); disp(T_toolbox.T - T_custom);
    disp("末端坐标：");
    fprintf("x: %.4f, y: %.4f, z: %.4f\n", x, y, z);
end

function T = get_tf_matrix(alpha, a, theta, d)
    T = [
        cos(theta),             -sin(theta),            0,              a;
        sin(theta)*cos(alpha),  cos(theta)*cos(alpha),  -sin(alpha),   -sin(alpha)*d;
        sin(theta)*sin(alpha),  cos(theta)*sin(alpha),  cos(alpha),    cos(alpha)*d;
        0,                      0,                      0,              1
        ];
end

function T_end = get_forward_kinematics(q, dm_arm)
    T_end = dm_arm.base.T;

    for i = 1 : dm_arm.n
        L = dm_arm.links(i);
        alpha = L.alpha;
        a = L.a;
        offset = L.offset;

        Ti = get_tf_matrix(alpha, a, q(i) + offset, L.d);
        T_end = T_end * Ti;
    end

    T_end = T_end * dm_arm.tool.T;
end

function [x, y, z] = get_end_coor(q, dm_arm)
    T = get_forward_kinematics(q, dm_arm);
    x = T(1, 4);
    y = T(2, 4);
    z = T(3, 4);
end

main();
