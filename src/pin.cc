#include <jetson/pin.hpp>

namespace EMIRO {
void Pin::init_pin(int *pin, bool isOut) {
    GPIO::setmode(GPIO::BOARD);
    if (isOut)
        GPIO::setup((*pin), GPIO::OUT, GPIO::LOW);
    else
        GPIO::setup((*pin), GPIO::IN, GPIO::LOW);
}
void Pin::set_low(int *pinNum) { GPIO::output((*pinNum), GPIO::LOW); }
void Pin::set_high(int *pinNum) { GPIO::output((*pinNum), GPIO::HIGH); }
} // namespace EMIRO