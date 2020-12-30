#pragma once

#include "Stepper.h"
#include "Timer.h"
#include <stdint.h>

namespace mp {

    struct Block {
        double speed{0};
        double target_position{0};
        double acceleration{0};
    };

    class MotionPlanner {

    public:
        MotionPlanner(Timer &, Stepper &, double s_per_tick);
        void set_block(const Block &block);
        void isr();
        bool is_ready() const;

    private:
        Block block_;
        std::int64_t steps_to_take_{0};
        const double mm_per_step_{0};
        Stepper &stepper_;
        bool ready_{true};
        const double s_per_tick_{ };
        Timer &timer_;
        double current_speed_{0};
        std::int64_t accel_until_{0}, decel_after_{0};
        double t_now_{0};
    };
}

