#include <control.hpp>
#include <jsonio.hpp>
#include <async_cam.hpp>

using namespace EMIRO;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_node");
    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();

    // Logger
    std::shared_ptr<Logger> logger = std::make_shared<Logger>();
    logger->init("Manual", FileType::CSV);
    logger->start(true);

    std::shared_ptr<Copter> copter = std::make_shared<Copter>();
    copter->init(nh, logger);
    std::shared_ptr<GPS> gps = std::make_shared<GPS>();
    gps->init(copter, logger);
    std::shared_ptr<Control> control = std::make_shared<Control>(copter, logger, gps);

    // Takeoff Copter
    copter->takeoff(1);
    ros::Duration(10).sleep();

    // Read JSON point
    JsonIO reader;
    reader = COPTER_DIR + "/copter/plan.json";
    std::vector<Target> target = reader.get_data();

    // Set Speed limit
    control->speed_limit = 0.5;
    for (Target &t : target)
    {
        if (!ros::ok())
        {
            copter->Land();
            ros::Duration(5).sleep();
            exit(EXIT_FAILURE);
        }
        std::cout << C_GREEN << S_BOLD << '[' << t.header << ']' << C_RESET << '\n';
        control->go(t.wp.x, t.wp.y, t.wp.z, t.wp.yaw, 0.05f, 5);
    }

    // Use Vision for Land
    AsyncCam cam(0, 640, 480);
    const int control_radius = 100;
    const int frame_w = cam.width, frame_h = cam.height;
    const int frame_w2 = frame_w / 2, frame_h2 = frame_h / 2;
    cam.set_range(cv::Scalar(255, 20, 255), cv::Scalar(0, 0, 0));
    cam.start();

    vector<Vec3f> circles;
    Mat frame;
    Vec3f selected_object;
    Keyboard kb;

    tpoint start_time = time_clock::now();
    int cnt_frame = 2;
    Point latest_center;
    while (ros::ok())
    {
        cam.getobject(circles, frame);
        if (!circles.empty())
        {
            selected_object = circles[0];

            // Select largest object
            if (circles.size() > 1)
            {
                int largest_radius = 0;
                for (auto &c : circles)
                {
                    Point p(c[0], c[1]);
                    int radius = cvRound(c[2]);

                    if (radius > largest_radius)
                    {
                        largest_radius = radius;
                        selected_object = c;
                    }
                }
            }

            Point center(cvRound(selected_object[0]), cvRound(selected_object[1]));
            int radius = cvRound(selected_object[2]);

            // Draw object outline
            cv::circle(frame, center, radius, Scalar(0, 0, 255), 3, 8, 0);

            center.x -= frame_w2;
            center.y -= frame_h2;
            AsyncCam::adjust_point(center, control_radius);

            if (std::sqrt(center.x * center.x + center.y * center.y) < 50)
                break;

            center.x = frame_w2 - center.x;
            center.y = frame_h2 - center.y;
            // center.x += frame_w2;
            // center.y += frame_h2;
            AsyncCam::point_buffer(center, 10);

            // Draw control points with invert direction
            cv::circle(frame, center, 9, Scalar(0, 255, 0), -1, 8, 0);
            latest_center = center;
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(time_clock::now() - start_time).count() > 1000)
        {
            float speed_x = 0.0f;
            if (std::abs(latest_center.x - frame_w2) > 50)
                speed_x = latest_center.x - frame_w2 > 0.0f ? 0.2 : -0.2;

            float speed_y = 0.0f;
            if (std::abs(latest_center.y - frame_h2) > 50)
                speed_y = latest_center.y - frame_h2 > 0.0f ? 0.2 : -0.2;

            LinearSpeed speed = {speed_x, speed_y, 0};
            gps->convert(speed);
            copter->set_vel(speed.linear_x, speed.linear_y, speed.linear_z, 0, 0, 0);

            start_time = time_clock::now();
            latest_center = Point(frame_w2, frame_h2);
        }

        cv::circle(frame, Point(frame_w2, frame_h2), control_radius, Scalar(0, 0, 255), 3, 8, 0);
        cv::putText(frame, "ESC to exit", Point(20, 20), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 2);
        cv::line(frame, Point(cam.width / 2, 0), Point(cam.width / 2, cam.height), Scalar(255, 0, 0), 2);
        cv::line(frame, Point(0, cam.height / 2), Point(cam.width, cam.height / 2), Scalar(255, 0, 0), 2);
        if (frame.size().width > 0)
            imshow("Result", frame);
        cv::waitKey(1);
        if (kb.get_key() == 27)
            break;
        cam.sync_fps();
    }
    cam.stop();
    copter->Land();
    ros::Duration(5).sleep();
    return 0;
}