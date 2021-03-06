#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include "../MotionPlanner1D/Timer.h"
#include "../MotionPlanner1D/Timer.cpp"

namespace MotionPlannerTests {

    int val = 0;
    void incVal() {
        val++;
    }

    TEST_CLASS(TimerTests) {

        TEST_METHOD(TestCallback) {

            val = 0;
            Timer timer(incVal);

            timer.setPrescale(1);
            timer.setThreshold(5);

            timer.tick();
            Assert::AreEqual(0, val, L"Callback called on first tick()");

            timer.tick();
            Assert::AreEqual(static_cast<Timer::counter_t>(1), timer.getCounter(), L"Counter not 1 after two ticks");


            timer.reset();

            for (int i = 0; i < 9; ++i) {
                timer.tick();
                Assert::AreEqual(static_cast<Timer::counter_t>((i + 1) / 2), timer.getCounter(), L"Counter does not match expected");
            }
            Assert::AreEqual(0, val, L"val not 0");
            timer.tick();
            Assert::AreEqual(1, val, L"val not 1");
            Assert::AreEqual(static_cast<Timer::counter_t>(0), timer.getCounter(), L"Counter does not match expected");
        }

        TEST_METHOD(TestZeroPsc) {
            val = 0;
            Timer timer(incVal);

            timer.setPrescale(0);
            timer.setThreshold(5);
            timer.tick();
            timer.tick();
            timer.tick();
            timer.tick();
            Assert::AreEqual(0, val, L"val not 0");
            timer.tick();
            Assert::AreEqual(1, val, L"val not 0");
        }

        TEST_METHOD(TestDifferentPSC_TH) {
            val = 0;
            Timer timer(incVal);

            const int th = 4, psc = 4;
            timer.setPrescale(psc);
            timer.setThreshold(th);

            for (int i = 0; i < (th) * (psc+1) - 1; ++i) {
                timer.tick();
            }
            Assert::AreEqual(0, val, L"val not 0");
            timer.tick();
            Assert::AreEqual(1, val, L"val not 1");
            Assert::AreEqual(static_cast<Timer::counter_t>(0), timer.getCounter(), L"Counter does not match expected");
        }
    };

};
