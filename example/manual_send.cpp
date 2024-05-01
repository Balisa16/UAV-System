#include <control.hpp>
#include <jsonio.hpp>
#include <tcpclient.hpp>

using namespace EMIRO;

int main(int argc, char **argv)
{

    Copter::init(argc, argv);
    std::cout << "Running\n";
    ros::Duration(5).sleep();

    // if (!Copter::PreArmedCheck())
    // {
    //     Copter::Land();
    //     exit(EXIT_FAILURE);
    // }

    // Copter::takeoff(1);
    // ros::Duration(10).sleep();

    // Read JSON point
    // JsonIO reader;
    // reader = COPTER_DIR + "/docs/plan.json";
    // reader.optimize_distance();
    // std::vector<Target> target = reader.get_data_vector();

    // Set Speed limit
    // PIDControl::get().set_rotation_speed(10.f);
    // PIDControl::get().set_linear_tolerance(0.2f);
    // PIDControl::get().set_rotation_tolerance(5.f);
    // Copter::set_yaw(YawMode::RELATIVE);

    int _cnt = 20;

    while (_cnt)
    {
        if (!ros::ok())
        {
            Copter::Land();
            exit(EXIT_FAILURE);
        }
        std::cout << "Counter : " << _cnt << '\n';
        // std::cout << C_GREEN << S_BOLD << '[' << t.header << ']' << C_RESET << '\n';

        // PIDControl::get().set_target_point(t.wp);
        // PIDControl::get().set_linear_speed(t.speed);
        // PIDControl::get().go_wait(true);

        Position pos;
        Quaternion quat;
        TCPClient client;
        client.connect();
        Copter::get_pose(&pos, &quat);
        Pose pose = {pos.x, pos.y, pos.z, quat.w, quat.x, quat.y, quat.z};

        client.send_pose(pose);
        client.read_response();
        client.close();

        ros::Duration(5).sleep();
        _cnt--;
        // ros::Duration(1).sleep();
    }

    return 0;
}