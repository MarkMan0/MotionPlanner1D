#include "MotionPlanner.h"

#include <cmath>

#include <iostream>

//assume 2Mhz timer clock
mp::MotionPlanner::MotionPlanner(Timer &timer, Stepper &stepper, double s_per_tick) : mm_per_step_(stepper.get_mm_per_step()),
stepper_(stepper), s_per_tick_(s_per_tick), timer_(timer) {
}

void mp::MotionPlanner::set_block(const mp::Block &block) {
    block_ = block;
    ready_ = false;
    steps_to_take_ = static_cast<std::int64_t>(block_.target_position / mm_per_step_);
    if (block_.acceleration == 0) {
        accel_until_ = steps_to_take_;
        decel_after_ = 0;
        current_speed_ = block_.speed;
        return;
    }
    block_.acceleration *= bezier_scale_;
    double step_spd = block_.speed / mm_per_step_, step_accel = block_.acceleration / mm_per_step_;

    double t_accel = step_spd / step_accel;
    std::int64_t s_accel = static_cast<std::int64_t>(0.5 * step_accel * t_accel*t_accel),
        s_middle = steps_to_take_ - 2*s_accel;
    // steps_to_take is counted downwards, so accel_until is larger than decel_after

    if (s_middle < 0) {
        s_middle = 1; // need 1 step
        s_accel = steps_to_take_ / 2;
        block_.speed = std::sqrt(2*s_accel*mm_per_step_*block_.acceleration);
    }

    double h = 0.5;
    bezier_y_scale_ = block_.speed;
    bezier_x_scale_ = t_accel;

    accel_until_ = s_middle + s_accel;
    decel_after_ = s_accel;


    t_now_ = std::sqrt(4*mm_per_step_ / block_.acceleration);
    current_speed_ = 0;
}

double mp::MotionPlanner::eval_bezier(double time) const {
    double t = time / bezier_x_scale_;
    if (t > 1) {
        return bezier_y_scale_;
    }
    constexpr double h = 0.5;
    double y = std::pow(t, 5)*(6-20*h) + std::pow(t, 4)*(-15+50*h) + std::pow(t, 3)*(10-40*h) + std::pow(t, 2)*(10*h);
    double v = y*bezier_y_scale_;
    return y*bezier_y_scale_;
}



void mp::MotionPlanner::isr() {

    if (steps_to_take_) {

        // convert speed to numer of timer ticks
        // speed = change_in_pos / change_in_time
        // change_in_time = change_in_pos / speed
        // change_in_pos is 1 step = mm_per_step_

        if (steps_to_take_ > accel_until_) {
            // acceleration phase
            current_speed_ = eval_bezier(t_now_);
            current_speed_ = std::min(current_speed_, block_.speed);
        }
        else if (steps_to_take_ >= decel_after_) {
            // nominal speed phase
            t_now_ = 0;
            current_speed_ = block_.speed;
        }
        else {
            // deceleration phase
            current_speed_ = block_.speed - eval_bezier(t_now_);
        }
        current_speed_ = std::max(current_speed_, 1.0);
        const double dt = mm_per_step_ / current_speed_;
        const auto th = static_cast<Timer::counter_t>(dt / s_per_tick_);
        t_now_ += th * s_per_tick_;
        timer_.setThreshold(th);
        stepper_.apply_step();
        --steps_to_take_;
    }
    else {
        ready_ = true;
    }
}

bool mp::MotionPlanner::is_ready() const {
    return ready_;
}
