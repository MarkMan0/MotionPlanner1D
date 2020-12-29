#pragma once

#include <cstdint>

class Stepper {

public:
    Stepper(double mm_per_step);
    void apply_step();
    void set_dir(int);
    void set_enabled(bool ena = true);
    bool is_enabled() const;
    double get_position() const;
    double get_mm_per_step() const;
    std::int64_t get_steps() const;

private:
    double mm_per_step_{0};
    bool enabled_{false};
    int dir_{1};
    std::int64_t steps_{0};
};

