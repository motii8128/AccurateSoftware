#ifndef OMNI_UTILS_HPP_
#define OMNI_UTILS_HPP_

#include <math.h>
#include <array>

namespace omni_vel_ros2
{
    template<typename T>
    class OmniWheel
    {
        public:
        OmniWheel(T wheel_radius, T robot_radius, std::array<T, 3> wheel_rad, bool pwm_mode)
        {
            robot_radius_ = robot_radius;
            wheel_radius_ = wheel_radius;

            wheel_rad_ = wheel_rad;
            pwm_mode_ = pwm_mode;
        }

        std::array<T, 3> calculation(T x_vel, T y_vel, T rotation_vel, T now_yaw)
        {
            std::array<T, 3> rpm_array;

            T wheel_posture_0 = wheel_rad_[0] + now_yaw;
            T wheel_posture_1 = wheel_rad_[1] + now_yaw;
            T wheel_posture_2 = wheel_rad_[2] + now_yaw;

            T wheel_0_v = -1.0*std::sin(wheel_posture_0)*x_vel + std::cos(wheel_posture_0)*y_vel + robot_radius_ * rotation_vel;
            T wheel_1_v = -1.0*std::sin(wheel_posture_1)*x_vel + std::cos(wheel_posture_1)*y_vel + robot_radius_ * rotation_vel;
            T wheel_2_v = -1.0*std::sin(wheel_posture_2)*x_vel + std::cos(wheel_posture_2)*y_vel + robot_radius_ * rotation_vel;

            if(!pwm_mode_)
            {
                T wheel_0_omega = wheel_0_v / wheel_radius_;
                T wheel_1_omega = wheel_1_v / wheel_radius_;
                T wheel_2_omega = wheel_2_v / wheel_radius_;

                rpm_array[0] = (wheel_0_omega * 60) / (2 * M_PI);
                rpm_array[1] = (wheel_1_omega * 60) / (2 * M_PI);
                rpm_array[2] = (wheel_2_omega * 60) / (2 * M_PI);

                return rpm_array;
            }
            else
            {
                rpm_array[0] = wheel_0_v * 255;
                rpm_array[1] = wheel_1_v * 255;
                rpm_array[2] = wheel_2_v * 255;

                return rpm_array;
            }
        }

        private:
        T robot_radius_;
        T wheel_radius_;
        std::array<T, 3> wheel_rad_;
        bool pwm_mode_;
    };
}

#endif