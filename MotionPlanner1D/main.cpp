
#include "MotionPlanner.h"
#include "Timer.h"

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

mp::MotionPlanner *planner_ptr = nullptr;

void call_isr() {
    if (planner_ptr != nullptr) {
        planner_ptr->isr();
    }
    else {
        throw std::runtime_error("planner_ptr not set");
    }
}

int main(int argc, char **argv) {

    const double dt = 1.0/2e6;  //assumed period for ticks

    Timer timer;
    timer.setPrescale(0);

    Stepper stepper(1.0/100.0);
    stepper.set_enabled();
    mp::MotionPlanner planner(timer, stepper, dt);
    planner_ptr = &planner;

    timer.set_callback(call_isr);

    mp::Block block;
    block.target_position = 200;    //[mm]
    block.speed = 80;               //[mm/s]
    block.acceleration = 100;       //[mm/s/s]

    planner.set_block(block);
    unsigned long long tick_cnt = 0;
    timer.setThreshold(5);

    std::ofstream out_file("C:\\Users\\Mark\\Documents\\School\\2_ING\\DP\\output.txt");
    out_file << std::setprecision(10);

    double last_pos = NAN;

    while (!planner.is_ready()) {
        timer.tick();
        ++tick_cnt;
        const auto pos = stepper.get_position();
        if (pos != last_pos) {
            out_file << dt * tick_cnt << ", " << pos << ", " << planner.current_speed_ << "\n";
            last_pos = pos;
        }
    }

    out_file.close();
    std::cout << "Position: " << stepper.get_position() << std::endl;

    return 0;
}
