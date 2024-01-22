#include <enum.hpp>
#include <async_cam.hpp>

using EMIRO::AsyncCam;

float sigmoid(float x, float k, float a, float b)
{
    // Calculate the expression: k / (1 + e^(a + bx))
    float exponent = a + b * x;
    float denominator = 1 + std::exp(exponent);
    float result = k / denominator;

    return result;
}

void point_buffer(Point point, const int &buffer_size)
{
    static vector<Point> buffer;
    buffer.push_back(point);

    if (buffer.size() > buffer_size)
        buffer.erase(buffer.begin());

    const float nat = 5.0f / (float)buffer.size();

    float Sn = 0.0f, _, _sig = 0.0f;
    point = {0, 0};
    for (int i = 0; i < buffer.size(); i++)
    {
        // Calculate sigmoid expression: k / (1 + e^(a + bx))
        _sig = 0.8f / (1 + std::exp(10 - 3 * nat * i)) + 0.2f;

        std::cout << _sig;
        if (i < buffer.size() - 1)
            std::cout << ",";
        point.x += (buffer[i].x * _sig);
        point.y += (buffer[i].y * _sig);
        Sn += _sig;
    }
    point.x /= Sn;
    point.y /= Sn;
}

static void rotate_point(Point &p, double &angle)
{
    double cos_theta = std::cos(angle);
    double sin_theta = std::sin(angle);

    // Rotation matrix
    double newX = p.x * cos_theta - p.y * sin_theta;
    double newY = p.x * sin_theta + p.y * cos_theta;

    p.x = newX;
    p.y = newY;
}

static void adjust_point(Point &p, int px_radius)
{
    float dist = std::sqrt(p.x * p.x + p.y * p.y);
    if (dist <= px_radius)
        return;

    float ratio = px_radius / dist;
    p.x *= ratio;
    p.y *= ratio;
}

int main()
{
    AsyncCam cam(0, 640, 480);
    cam.calibrate();
    cam.start();
    vector<Vec3f> circles;
    Mat frame;
    Vec3f selected_object;
    EMIRO::Keyboard kb;
    while (true)
    {
        cam.getobject(circles, frame);
        if (!circles.empty())
        {
            int frame_w = frame.size().width;
            int frame_h = frame.size().height;
            int frame_w2 = frame_w / 2;
            int frame_h2 = frame_h / 2;
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

            cv::Point center(cvRound(selected_object[0]), cvRound(selected_object[1]));
            int radius = cvRound(selected_object[2]);

            // circle center and outline
            cv::circle(frame, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

            // draw line from center to object
            // cv::line(frame, cv::Point(frame_w2, frame_h2), center, cv::Scalar(0, 255, 0), 3);

            // Draw cursor
            cv::circle(frame, Point(frame_w2, frame_h2), 50, cv::Scalar(0, 0, 255), 3, 8, 0);
            center.x -= frame_w2;
            center.y -= frame_h2;
            adjust_point(center, 50);
            center.x += frame_w2;
            center.y += frame_h2;
            point_buffer(center, 50);
            cv::circle(frame, center, 9, cv::Scalar(0, 255, 0), -1, 8, 0);
        }
        putText(frame, "ESC to exit", Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 2);
        cv::line(frame, cv::Point(cam.width / 2, 0), cv::Point(cam.width / 2, cam.height), cv::Scalar(255, 0, 0), 2);
        cv::line(frame, cv::Point(0, cam.height / 2), cv::Point(cam.width, cam.height / 2), cv::Scalar(255, 0, 0), 2);
        if (frame.size().width > 0)
            cv::imshow("Result", frame);
        cv::waitKey(1);
        if (kb.get_key() == 27)
            break;
        cam.sync_fps();
    }
    cam.stop();
    return 0;
}