#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include "../MotionPlanner1D/Stepper.h"
#include "../MotionPlanner1D/Stepper.cpp"

namespace MotionPlannerTests {

    TEST_CLASS(StepperTests) {
        TEST_METHOD(TestStepper) {
            Stepper stepper(1.0/100);

            Assert::AreEqual(0.0, stepper.get_position(), L"Position not 0 at start");
            Assert::AreEqual(static_cast<std::int64_t>(0), stepper.get_steps(), L"Steps not 0 at start");

            stepper.set_enabled(false);
            Assert::ExpectException<std::runtime_error>([&stepper] { stepper.apply_step(); }, L"Apply step with enabled_ false did not throw");

            stepper.set_enabled();
            for (int i = 0; i < 100; ++i) {
                stepper.apply_step();
            }

            Assert::AreEqual(1.0, stepper.get_position(), L"Position not correct");
            Assert::AreEqual(static_cast<std::int64_t>(100), stepper.get_steps(), L"Steps not correct");
        }
    };
}
