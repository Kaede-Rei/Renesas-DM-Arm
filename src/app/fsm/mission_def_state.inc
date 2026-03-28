// 业务状态定义

SX(init, NULL)
SX(active, NULL)

    SX(idle, &state_active)
    SX(plan_aim, &state_active)
    SX(moving, &state_active)
    SX(lasering, &state_active)
    SX(finish, &state_active)
    
SX(fault, NULL)
