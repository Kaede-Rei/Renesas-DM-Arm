function main()
    L0 = Link('alpha', 0,      'a', 0,     'offset', 0,        'd', 0.15,  'modified');
    L1 = Link('alpha', pi/2,   'a', 0,     'offset', 0,        'd', 0,     'modified');
    L2 = Link('alpha', 0,      'a', 0.5,   'offset', 5*pi/6,   'd', 0,     'modified');
    L3 = Link('alpha', 0,      'a', 0.577, 'offset', pi/6,     'd', 0,     'modified');
    L4 = Link('alpha', pi/2,   'a', 0.1,   'offset', pi/2,     'd', 0,     'modified');
    L5 = Link('alpha', pi/2,   'a', 0,     'offset', 0,        'd', 0,     'modified');
    dm_arm = SerialLink([L0 L1 L2 L3 L4 L5], 'name', 'test');

    q_test = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];

    T_toolbox = dm_arm.fkine(q_test);
    T_custom = get_forward_kinematics(q_test);
    [x, y, z] = get_end_coor(q_test);

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

function T_end = get_forward_kinematics(q)
    params = [
        % alpha, a, offset, d
        0,     0,      0,          0.15;
        pi/2,  0,      0,          0;
        0,     0.5,    5*pi/6,     0;
        0,     0.577,  pi/6,       0;
        pi/2,  0.1,    pi/2,       0;
        pi/2,  0,      0,          0;
        ];

    T_end = eye(4);

    for i = 1:size(params, 1)
        alpha = params(i, 1);
        a = params(i, 2);
        offset = params(i, 3);
        d = params(i, 4);

        Ti = get_tf_matrix(alpha, a, q(i) + offset, d);

        T_end = T_end * Ti;
    end
end

function [x, y, z] = get_end_coor(q)
    T = get_forward_kinematics(q);
    x = T(1, 4);
    y = T(2, 4);
    z = T(3, 4);
end

main();
