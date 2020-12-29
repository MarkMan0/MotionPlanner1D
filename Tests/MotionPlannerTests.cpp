#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include "../MotionPlanner1D/MotionPlanner.h"
#include "../MotionPlanner1D/MotionPlanner.cpp"

static bool compare_doubles(double a, double b, double prec = 1e-5) {
    if (std::abs(a-b) < prec) {
        return true;
    }
    else {
        return false;
    }
}

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
            unsigned long long tick_cnt = 0, last_cnt = 0;
            timer.setThreshold(0);

            double last_pos = stepper.get_position();

            while (!planner.is_ready()) {
                timer.tick();
                ++tick_cnt;

                const auto pos = stepper.get_position();
                if (pos != last_pos) {
                    if (tick_cnt > 2) { // ignore first step, speed will be wrong
                        double speed = (pos - last_pos) / ((tick_cnt - last_cnt) * dt);
                        Assert::IsTrue(compare_doubles(block.speed, speed, 1e-6), L"Speed not as expected");
                    }
                    last_pos = pos;
                    last_cnt = tick_cnt;
                }
            }
            Assert::AreEqual(block.target_position, stepper.get_position(), L"Stepper position not correct");

            stepper.set_steps(0);
            block.target_position = 123;
            block.speed = 32;

            last_pos = stepper.get_position();
            tick_cnt = 0, last_cnt = 0;
            timer.reset();
            timer.setThreshold(0);
            planner.set_block(block);

            while (!planner.is_ready()) {
                timer.tick();
                ++tick_cnt;

                const auto pos = stepper.get_position();
                if (pos != last_pos) {
                    if (tick_cnt > 2) { // ignore first step, speed will be wrong
                        double speed = (pos - last_pos) / ((tick_cnt - last_cnt) * dt);
                        Assert::IsTrue(compare_doubles(block.speed, speed, 1e-6), L"Speed not as expected");
                    }
                    last_pos = pos;
                    last_cnt = tick_cnt;
                }
            }
            Assert::AreEqual(block.target_position, stepper.get_position(), L"Stepper position not correct");
        }
    };
}
