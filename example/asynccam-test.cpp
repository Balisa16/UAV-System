#include <async_cam.hpp>

using EMIRO::AsyncCam;

int main()
{
    AsyncCam cam(0);
    cam.calibrate();
    // cam.start();
    // cam.stop();
    return 0;
}