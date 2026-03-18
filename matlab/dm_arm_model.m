% L0 = Link('alpha', 0,      'a', 0,     'offset', 0,        'd', 0,     'modified');
% L1 = Link('alpha', pi/2,   'a', 0,     'offset', 0,        'd', 0,     'modified');
% L2 = Link('alpha', 0,      'a', 0.5,   'offset', 5*pi/6,   'd', 0,     'modified');
% L3 = Link('alpha', 0,      'a', 0.577, 'offset', pi/6,     'd', 0,     'modified');
% L4 = Link('alpha', pi/2,   'a', 0.1,   'offset', pi/2,     'd', 0,     'modified');
% L5 = Link('alpha', pi/2,   'a', 0,     'offset', 0,        'd', 0,     'modified');

L0 = Link('alpha', 0,      'a', 0,     'offset', 0,        'd', 0.15,  'modified');
L1 = Link('alpha', pi/2,   'a', 0,     'offset', 0,        'd', 0,     'modified');
L2 = Link('alpha', 0,      'a', 0.5,   'offset', 5*pi/6,   'd', 0,     'modified');
L3 = Link('alpha', 0,      'a', 0.577, 'offset', pi/6,     'd', 0,     'modified');
L4 = Link('alpha', pi/2,   'a', 0.1,   'offset', pi/2,     'd', 0,     'modified');
L5 = Link('alpha', pi/2,   'a', 0,     'offset', 0,        'd', 0,     'modified');

dm_arm = SerialLink([L0 L1 L2 L3 L4 L5], 'name', 'test');

dm_arm.plot(zeros(1,6));

dm_arm.teach;

dm_arm.display();
