#ifndef PIN_HEADER
#define PIN_HEADER

#include <JetsonGPIO.h>

namespace EMIRO {
class Pin {
  public:
    Pin();
    void init_pin(int *pin, bool isOut);
    void set_low(int *pinNum);
    void set_high(int *pinNum);
    ~Pin();
};
} // namespace EMIRO
#endif // PIN_HEADER
