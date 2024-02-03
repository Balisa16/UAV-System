#pragma once

#include <libnotify/notify.h>
#include <string>

class Notification {
  public:
    Notification();

    void show(std::string message, std::string title = "Copter",
              int timeout_ms = 3000);

    ~Notification();
};