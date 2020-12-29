#include "MotionPlanner.h"


//assume 2Mhz timer clock
mp::MotionPlanner::MotionPlanner(Timer &timer, Stepper &stepper) : mm_per_step_(stepper.get_mm_per_step()), stepper_(stepper), s_per_tick_(1.0/2e6), timer_(timer) {
}

void mp::MotionPlanner::set_block(const mp::Block &block) {
    block_ = block;
    steps_to_take_ = static_cast<std::int64_t>(block_.target_position / mm_per_step_);
    ready_ = false;
}

void mp::MotionPlanner::isr() {

    if (steps_to_take_) {

        // convert speed to numer of timer ticks
        // speed = change_in_pos / change_in_time
        // change_in_time = change_in_pos / speed
        // change_in_pos is 1 step = mm_per_step_

        const double dt = mm_per_step_ / block_.speed;
        const auto th = static_cast<Timer::counter_t>(dt / s_per_tick_);
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
