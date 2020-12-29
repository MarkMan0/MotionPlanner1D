#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include "../MotionPlanner1D/MotionPlanner.h"
#include "../MotionPlanner1D/MotionPlanner.cpp"

namespace MotionPlannerTests {

    mp::MotionPlanner *planner_ptr = nullptr;

    void call_isr() {
        if (planner_ptr != nullptr) {
            planner_ptr->isr();
        }
        else {
            throw std::runtime_error("planner_ptr not set");
        }
    }

    TEST_CLASS(MotionPlannerTests) {
        TEST_METHOD(TestConstantSpeed) {
            const double dt = 1.0/2e6;  //assumed period for ticks

            Timer timer;
            timer.setPrescale(0);

            Stepper stepper(1.0/100.0);
            stepper.set_enabled();
            mp::MotionPlanner planner(timer, stepper);
            planner_ptr = &planner;

            timer.set_callback(call_isr);

            mp::Block block;
            block.target_position = 100;    //[mm]
            block.speed = 50;               //[mm/s]

            planner.set_block(block);
            unsigned long long tick_cnt = 0;
            timer.setThreshold(0);

            double last_pos = std::nan("");

            while (!planner.is_ready()) {
                timer.tick();
                ++tick_cnt;
            }
            Assert::AreEqual(block.target_position, stepper.get_position(), L"Stepper position not correct");
        }
    };
}
