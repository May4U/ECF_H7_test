#ifndef __MATHS_H
#define __MATHS_H

#include <cstdint>

namespace UserMath
{

#ifndef PI
#define PI 3.14159265358979f
#endif

    //运动加速度限制斜坡函数
    typedef struct
    {
        float Input;     //当前取样值
        float Last_Input;//上次取样值
        float Output;    //输出值
        float acc_now;   //当前加速度
        float acc_limit; //需要限制的加速度
    } acceleration_control_type_t;

    /**
     * 绝对值
     */
    template<class T>
    T abs(T x)
    {
        return x > 0 ? x : -x;
    }

    /**
     * 数字限幅
     */
    template<class T, class S, class R>
    T clamp(T val, S min, R max)
    {
        if (val < min)
        {
            return min;
        }
        else if (val > max)
        {
            return max;
        }
        else
        {
            return val;
        }
    }

    /**
     * 最大值
     */
    template<class T>
    T max(T x, T y)
    {
        return x > y ? x : y;
    }

    /**
     * 弧度制格式化为-PI~PI
     */
    template<class T>
    T radFormat(T rad)
    {
        if (rad == 0)
        {
            return rad;
        }

        rad %= 2 * PI * rad / abs(rad);
        if (abs(rad) > PI)
        {
            rad -= 2 * PI * rad / abs(rad);
        }
        else
        {
            rad = 0;
        }

        return rad;
    }

/**输出的 y 的正负性, 由 x 的正负性质来决定, 可以用于减少 if else*/
#define anti_abs_value(x, y)                              \
    {                                                     \
        (x > 0) ? ((y > 0) ? y : -y) : ((y < 0) ? y : -y) \
    }

    int16_t max_abs(int16_t x, int16_t y);                                               //取最大值的绝对值
    float invSqrt(float x);                                                              //平方根倒数
    float float_min_distance(float target, float actual, float minValue, float maxValue);//寻最小值

    /* 循环限制 */
    int16_t loop_restriction_int16(int16_t num, int16_t limit_num);        // 16位循环限幅
    float loop_restriction_float(float num, float limit_num);              //浮点循环限幅
    float loop_fp32_constrain(float Input, float minValue, float maxValue);//循环限制（云台角度处理）
    float CircleIncreaseLimit(float current_val, float set_val, float limit);
    float cos_calculate(float angle);
    float sin_calculate(float angle);

    /* 斜坡函数 */
    void data_accelerated_control(float *input, float acc); //加速度限制斜坡函数

    /* 限幅滤波 */
    float limiting_filter(float new_value, float last_value, float delat_max);

    class acceleration_control_c
    {
    public:
        acceleration_control_type_t acceleration_control;

        /* 运动控制斜坡函数（加速度限制）（16位） */
        int16_t motion_acceleration_control(int16_t Input, int16_t Limit);//运动加速度限制
    };
}

#endif
