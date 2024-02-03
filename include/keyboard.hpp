#ifndef KEYBOARD_INTERRUPT
#define KEYBOARD_INTERRUPT

#include <iostream>
#ifdef _WIN32
#include <windows.h>
#include <conio.h>
#elif __linux__
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#endif

struct Keyboard
{
private:
#ifdef __linux__
    int ch;
    int oldf;
    struct termios oldt, newt;
#endif

public:
    Keyboard()
    {
#ifdef __linux__
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
#endif
    }

    char get_key()
    {
#ifdef __linux__
        return getchar();
#elif _WIN32
        if (_kbhit())
            return _getch()
#endif
    }

    ~Keyboard()
    {
#ifdef __linux__
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
#endif
    }
};
#endif