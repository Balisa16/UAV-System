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

namespace EMIRO
{
  class GPS
  {
  public:
    static GPS &get_gps()
    {
      static GPS instance;
      return instance;
    }

    static void init();

    static void lock_pos();

    static void convert(LinearSpeed &linear_speed);

  private:
    GPS();
    ~GPS();
    void gps_init();

    void gps_lock_pos();

    void gps_convert(LinearSpeed &linear_speed) const;

    WayPoint start_point;
    float radians;
    float mul_x, mul_y;
    bool is_locked = false, is_init = false;
  };
}