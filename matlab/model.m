L0 = Link('alpha', 0,      'a', 0,     'offset', -1.5708,        'd', 0,  'modified');
L1 = Link('alpha', -pi/2,   'a', 0,     'offset', -1.7939,        'd', 0,     'modified');
L2 = Link('alpha', 0,      'a', 0.28503,   'offset', 0,   'd', 0.25075,     'modified');
L3 = Link('alpha', pi/2,      'a', 0, 'offset', 0,     'd', 0,     'modified');
L4 = Link('alpha', -pi/2,   'a', 0.1,   'offset', 0,     'd', 0.091,     'modified');
L5 = Link('alpha', pi/2,   'a', 0,     'offset', 0,        'd', 0,     'modified');

arm = SerialLink([L0 L1 L2 L3 L4 L5], 'name', 'test');

arm.plot(zeros(1,6));

arm.teach;

arm.display();
