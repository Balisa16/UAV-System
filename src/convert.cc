#include <convert.hpp>

namespace EMIRO {
Nav_Convert::Nav_Convert() {}

void Nav_Convert::init(float current_deg) {
    current_deg = 360.0f - current_deg;
    if (current_deg >= 360.0f)
        current_deg = (int)current_deg % 360;
    init_deg = current_deg;
    radians = (current_deg - 90) * M_PI / 180.0;
    printf("Lock copter degree : % .2f° or % .4f rad\n", init_deg, radians);
}

WayPoint Nav_Convert::convert(WayPoint need_pos) {
    double x = need_pos.x * cos(radians) - need_pos.y * sin(radians);
    double y = need_pos.x * sin(radians) + need_pos.y * cos(radians);
    need_pos.x = x;
    need_pos.y = y;
    need_pos.yaw = init_deg;
    return need_pos;
}

WayPoint2 Nav_Convert::convert(WayPoint2 need_pos) {
    double x = need_pos.x * cos(radians) - need_pos.y * sin(radians);
    double y = need_pos.x * sin(radians) + need_pos.y * cos(radians);
    need_pos.x = x;
    need_pos.y = y;
    need_pos.yaw = init_deg;
    return need_pos;
}

float Nav_Convert::get_degree() { return init_deg; }

Nav_Convert::~Nav_Convert() {}
} // namespace EMIRO