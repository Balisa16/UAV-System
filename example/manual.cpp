#include <control.hpp>
#include <jsonio.hpp>

using namespace EMIRO;

int main(int argc, char **argv)
{
    Copter::init(argc, argv);

    if (!Copter::PreArmedCheck())
    {
        Copter::Land();
        exit(EXIT_FAILURE);
    }

    Copter::takeoff(1);
    // ros::Duration(10).sleep();

    // Read JSON point
    JsonIO reader;
    reader = COPTER_DIR + "/docs/plan.json";
    std::vector<Target> target = reader.get_data_vector();

    // Set Speed limit
    // Control::set_linear_speed_limit(2.f);
    PIDControl::get().set_rotation_speed(10.f);
    PIDControl::get().set_linear_tolerance(0.1f);
    PIDControl::get().set_rotation_tolerance(5.f);

    for (Target &t : target)
    {
        if (!ros::ok())
        {
            Copter::Land();
            exit(EXIT_FAILURE);
        }
        std::cout << C_GREEN << S_BOLD << '[' << t.header << ']' << C_RESET << '\n';

        PIDControl::get().set_target_point(t.wp);
        PIDControl::get().set_linear_speed(t.speed);
        PIDControl::get().go_wait();
        // Control::go(t.wp.x, t.wp.y, t.wp.z, t.wp.yaw, 0.05f, 5);
    }

    return 0;
}