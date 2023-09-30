#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>    
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <stdarg.h>

namespace EMIRO{
    enum class LogLevel {
        INFO,
        WARNING,
        ERROR
    };

    enum class FileType {
        TXT,
        CSV
    };

    class Logger
    {
    private:
        std::chrono::_V2::system_clock::time_point start_time, stop_time;
        std::string full_filename;
        uint64_t line_counter;
        LogLevel level;
        FileType type;
        std::ofstream writer;
        uint16_t info_msg, warn_msg, err_msg;
        bool combo_msg;
        void resume();
        std::string getLvl(LogLevel lvl = LogLevel::INFO);
        std::string cust_printf(const char *format, va_list args);
    public:
        Logger(std::string filename, FileType type = FileType::TXT)
        {
            init(filename, type);
        }
        Logger(){};

        /**
         * @brief Initialize filename and type of file
         * 
         * @param filename  Filename
         * @param type      Type of file CSV or TXT
         */
        void init(std::string filename, FileType type = FileType::TXT);

        /**
         * @brief Starting Logger System
         * 
         */
        void start();

        /**
         * @brief Write message into file without showing message into terminal
         * 
         * @param level     Message Level (ERROR, WARNING, INFO).
         * @param format    Message format
         * @param ...       Additional of message variable
         * 
         * @note Message format same as printf in C.
         */
        void write(LogLevel level, const char *format, ...);

        /**
         * @brief           Just showing message into terminal without write in logger file.
         * 
         * @param level     Message Level (ERROR, WARNING, INFO).
         * @param format    Message format
         * @param ...       Additional of message variable
         * 
         * @note Message format same as printf in C.
         */
        void show(LogLevel level, const char *format, ...);

        /**
         * @brief           Write message into logger file and show up message into terminal.
         * 
         * @param level     Message Level (ERROR, WARNING, INFO).
         * @param format    Message format
         * @param ...       Additional of message variable
         * 
         * @note Message format same as printf in C.
         */
        void write_show(LogLevel level, const char *format, ...);

        /**
         * @brief Finished Logger System.
         * 
         */
        void finish();
        
        ~Logger(){};
    };
}

#endif