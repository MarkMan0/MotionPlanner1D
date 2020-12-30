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
public:
    const double dt{1.0/2e6};
    Timer timer;
    Stepper stepper;
    mp::MotionPlanner planner;
    mp::Block block;

    MotionPlannerTests() : stepper(1.0/100.0), planner(timer, stepper, dt) { }

    TEST_METHOD_INITIALIZE(init) {
        block.speed = 0;
        block.acceleration = 0;
        block.target_position = 0;
        timer.setPrescale(0);
        planner_ptr = &planner;
        timer.set_callback(call_isr);
        stepper.set_enabled();
        timer.reset();
        timer.setThreshold(0);
    }

    void run_planner() {
        double last_pos = stepper.get_position();
        unsigned long long tick_cnt{0}, last_cnt{0};
        while (!planner.is_ready()) {
            timer.tick();
            ++tick_cnt;

            const auto pos = stepper.get_position();
            if (pos != last_pos) {
                if (tick_cnt > 2) { // ignore first step, speed will be wrong
                    double speed = (pos - last_pos) / ((tick_cnt - last_cnt) * dt);
                    Assert::AreEqual(block.speed, speed, 1e-6, L"Speed not as expected");
                }
                last_pos = pos;
                last_cnt = tick_cnt;
            }
        }
        Assert::AreEqual(block.target_position, stepper.get_position(), L"Stepper position not correct");
    }

    TEST_METHOD(TestConstantSpeed1) {
        block.target_position = 100;    //[mm]
        block.speed = 50;               //[mm/s]
        planner.set_block(block);
        run_planner();
    }

    TEST_METHOD(TestConstantSpeed2) {
        block.target_position = 132;
        block.speed = 32;
        planner.set_block(block);
        run_planner();
    }

    TEST_METHOD(TestAcceleration) {
        block.target_position = 200;
        block.speed = 80;
        block.acceleration = 500;
        planner.set_block(block);
        double last_pos = stepper.get_position();
        unsigned long long tick_cnt{0}, last_cnt{0};
        while (!planner.is_ready()) {
            timer.tick();
            ++tick_cnt;

            const auto pos = stepper.get_position();
            if (pos != last_pos) {
                if (tick_cnt > 2) { // ignore first step, speed will be wrong
                    double speed = (pos - last_pos) / ((tick_cnt - last_cnt) * dt);
                    Assert::IsTrue(speed - 0.5 <= block.speed, L"Speed exceeds requested");
                }
                last_pos = pos;
                last_cnt = tick_cnt;
            }
        }
        Assert::AreEqual(block.target_position, stepper.get_position(), L"Stepper position not correct");
    }
    };
}
