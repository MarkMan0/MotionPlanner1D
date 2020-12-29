#pragma once

#include "Stepper.h"
#include "Timer.h"
#include <stdint.h>

namespace mp {

    struct Block {
        double speed{0};
        double target_position{0};
    };

    class MotionPlanner {

    public:
        MotionPlanner(Timer &, Stepper &);
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
    };
}

