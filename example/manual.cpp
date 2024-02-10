#include <control.hpp>
#include <jsonio.hpp>

using namespace EMIRO;

int main(int argc, char **argv)
{
    Copter::init(argc, argv);

    Copter::takeoff(1);
    // ros::Duration(10).sleep();

    // Read JSON point
    JsonIO reader;
    reader = COPTER_DIR + "/docs/plan.json";
    std::vector<Target> target = reader.get_data_vector();

    // Set Speed limit
    Control::set_linear_speed_limit(2.f);

    int cnt = 4;
    for (Target &t : target)
    {
        if (!ros::ok())
        {
            Copter::Land();
            exit(EXIT_FAILURE);
        }
        cnt--;
        if (!cnt)
        {
            Copter::go_rtl(5.f);
            break;
        }
        std::cout << C_GREEN << S_BOLD << '[' << t.header << ']' << C_RESET
                  << '\n';
        Control::go(t.wp.x, t.wp.y, t.wp.z, t.wp.yaw, 0.05f, 5);
    }

    return 0;
}