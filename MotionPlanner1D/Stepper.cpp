#include "Stepper.h"

#include <stdexcept>

Stepper::Stepper(double mm_per_step) : mm_per_step_(mm_per_step) {
}

void Stepper::apply_step() {
    if (enabled_) {
        if (dir_ >= 0) {
            ++steps_;
        }
        else {
            --steps_;
        }
    }
    else {
        throw std::runtime_error("apply_step called with enabled_ false");
    }
}

void Stepper::set_dir(int dir) {
    dir_ = dir;
}

void Stepper::set_enabled(bool ena) {
    enabled_ = ena;
}

bool Stepper::is_enabled() const {
    return enabled_;
}

double Stepper::get_position() const {
    return mm_per_step_ * steps_;
}

double Stepper::get_mm_per_step() const {
    return mm_per_step_;
}

std::int64_t Stepper::get_steps() const {
    return steps_;
}
