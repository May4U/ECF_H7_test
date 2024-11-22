#include "Motor_General_def.hpp"

namespace Motor
{

    void Motor_Controller_c::RefValChange(float ref_val)
    {
        this->pid_ref = ref_val;
    }

    float Motor_Controller_c::GetRefVal() const
    {
        return this->pid_ref;
    }

}// namespace Motor
