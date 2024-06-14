#include <control.hpp>
#include <jsonio.hpp>

using namespace EMIRO;

int main(int argc, char **argv)
{
    Copter::init(argc, argv);

    // Copter::waitHDOP(1.0f, 5000);

    Copter::takeoff(1);
    // ros::Duration(10).sleep();

    // Read JSON point
    JsonIO reader;
    reader = COPTER_DIR + "/docs/full_no_sample.json";
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
        std::cout << '\n'
                  << C_GREEN << S_BOLD << '[' << t.header << ']' << C_RESET << '\n';

        if (t.header == "RTL")
        {
            Copter::go_rtl();
            ros::Duration(10).sleep();
            if (!Copter::set_mode(FlightMode::GUIDED))
                exit(EXIT_FAILURE);
            Copter::takeoff(1.f);
        }

        PIDControl::get().set_linear_speed(t.speed);
        PIDControl::get().set_target_point(t.wp);
        PIDControl::get().go_wait(true);
        ros::Duration(3).sleep();
    }

    return 0;
}