#pragma once

#include <Logger.hpp>
#include <chrono>
#include <cmath>
#include <copter.hpp>
#include <cstring>
#include <enum.hpp>
#include <iostream>
#include <ncurses.h>
#include <thread>

namespace EMIRO {
class GPS {
  public:
    GPS();

    void init(std::shared_ptr<EMIRO::Copter> copter,
              std::shared_ptr<EMIRO::Logger> logger);

    void lock_pos();

    void convert(LinearSpeed &linear_speed);

    ~GPS();

  private:
    std::shared_ptr<EMIRO::Copter> copter;
    std::shared_ptr<EMIRO::Logger> log;
    WayPoint start_point;
    float radians;
    float mul_x, mul_y;
    bool is_locked = false, is_init = false;
};
} // namespace EMIRO