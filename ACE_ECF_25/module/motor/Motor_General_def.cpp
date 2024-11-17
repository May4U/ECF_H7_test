#include "Motor_General_def.hpp"

void Motor::Motor_Controller_c::RefValChange(float ref_val)
{
    this->pid_ref = ref_val;
}

float Motor::Motor_Controller_c::GetRefVal(void)
{
    return this->pid_ref;
}