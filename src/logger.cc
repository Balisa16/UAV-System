#include <Logger.hpp>

namespace EMIRO{
    Logger::Logger()
    {

    }

    Logger::Logger(std::string filename, FileType type)
    {
        status  == LoggerStatus::None;
        init(filename, type);
    }

    LoggerStatus Logger::get_status()
    {
        return status;
    }

    void Logger::resume()
    {
        std::time_t time = std::chrono::system_clock::to_time_t(start_time);
        char start_str[100];
        std::strftime(start_str, sizeof(start_str), "%Y-%m-%d %H:%M:%S", std::localtime(&time));

        time = std::chrono::system_clock::to_time_t(stop_time);
        char stop_str[100];
        std::strftime(stop_str, sizeof(stop_str), "%Y-%m-%d %H:%M:%S", std::localtime(&time));

        std::chrono::duration<double> elapsed_seconds = stop_time - start_time;

        std::cout << "\n\033[34m\033[1mFlight Resume :\033[0m\n\tStart Time\t: " << start_str << 
            "\n\tStop Time\t: " << stop_str << 
            "\n\tFlight Duration\t: " << std::to_string(elapsed_seconds.count()) << " seconds\nErrors\t\t: " << err_msg << 
            " message\nWarnings\t: " << warn_msg <<
            " message\nInformations\t: " << info_msg << " message\n\n";
    }

    void Logger::init(std::string filename, FileType type)
    {
        std::lock_guard<std::mutex> lg(mtx);
        if(status  == LoggerStatus::None)
        {
            struct passwd *pw = getpwuid(getuid());
            const char *home_char = pw->pw_dir;
            std::string home_dir = home_char;
            std::string flight_dir = "Flight Log";
            std::string temp_1 = home_dir + "/" + flight_dir;

            bool is_exist = boost::filesystem::exists(temp_1);

            if(!is_exist){
                std::cout << "Create folder " + flight_dir + " in " + home_dir << std::endl;
                temp_1 = "mkdir " + home_dir + "/'" + flight_dir +"'";
                system(temp_1.c_str());
            }

            // Set file type
            this->type = type;

            std::string file_ext = ".csv";
            if(type == FileType::TXT)
                file_ext = ".txt";

            std::string temp_filedir = home_dir + "/" + flight_dir +"/";
            
            temp_1 = temp_filedir + filename + file_ext;
            if(boost::filesystem::exists(temp_1.c_str()))
            {
                std::cout << "File : " << temp_1 << " already exist." << std::endl;
                int file_idx = 1;
                temp_1 = temp_filedir + filename + "-" + std::to_string(file_idx) + file_ext;
                while (boost::filesystem::exists(temp_1.c_str()))
                {
                    file_idx++;
                    temp_1 = temp_filedir + filename + "-" + std::to_string(file_idx) + file_ext;
                }
            }
            
            this->full_filename = temp_1;
            std::cout << "Log file stored in : " << temp_1 << std::endl;
            info_msg = 0;
            warn_msg = 0;
            err_msg = 0;
            combo_msg = false;
            status = LoggerStatus::Init;
        }
    }

    void Logger::start(bool reset_prev_counter)
    {
        std::lock_guard<std::mutex> lg(mtx);
        if(status  == LoggerStatus::Init || status  == LoggerStatus::Stop)
        {
            if(reset_prev_counter)
            {
                warn_msg = 0;
                info_msg = 0;
                err_msg = 0;
            }
            start_time = std::chrono::system_clock::now();
            writer.open(this->full_filename);
            if(type == FileType::CSV)
                writer << "Level,Message Id,Datetime (yyyy-mm-dd),Flight Time(s),Message\n";
            line_counter = 0;
            is_start = true;
            status = LoggerStatus::Run;
        }
        else if(status  == LoggerStatus::Run)
            std::cout << "\033[33mLogger is already activated.\033[30m\n";
        else if(status  == LoggerStatus::None)
            std::cout << "\033[31mLogger is not initialized.\033[30m\n";
        else
            std::cout << "\033[33mLogger in Waiting Mode.\033[30m\n";
    }

    std::string Logger::getLvl(LogLevel lvl, bool no_color)
    {
        std::string lvl_string = "";
        if(no_color)
            switch (lvl)
            {
            case LogLevel::INFO:
                lvl_string += "\033[32m[INFO ]\033[0m";
                break;
            case LogLevel::WARNING:
                lvl_string += "\033[33m[WARN ]\033[0m";
                break;
            case LogLevel::ERROR:
                lvl_string += "\033[31m[ERROR]\033[0m";
                break;
            
            default:
                break;
            }
        else
            switch (lvl)
            {
            case LogLevel::INFO:
                lvl_string += "INFO";
                break;
            case LogLevel::WARNING:
                lvl_string += "WARN";
                break;
            case LogLevel::ERROR:
                lvl_string += "ERROR";
                break;
            
            default:
                break;
            }
        return lvl_string;
    }

    std::string Logger::cust_printf(const char *format, va_list args)
    {
        char buffer[5000];
        vsprintf(buffer, format, args);
        return buffer;
    }

    void Logger::unavailable_msg()
    {
        std::string _msg = "\033[31m[Unavailabe]\033[0m Logger Status : ";
        switch(status)
        {
        case LoggerStatus::None:
            _msg += "None";
            break;
        case LoggerStatus::Init:
            _msg += "Init";
            break;
        case LoggerStatus::Run:
            _msg += "Run";
            break;
        case LoggerStatus::Wait:
            _msg += "Wait";
            break;
        case LoggerStatus::Stop:
            _msg += "Stop";
            break;
        default:
            _msg += "Unknown";
            break;
        }
        std::cout << _msg << '\n';
    }

    void Logger::write(LogLevel level, const char *format, ...)
    {
        // Lock mutex
        std::lock_guard<std::mutex> lg(mtx);

        if(status  == LoggerStatus::Run)
        {
            va_list args;
            va_start(args, format);
            std::string msg = cust_printf(format, args);
            va_end(args);

            line_counter++;
            std::string header = getLvl(level, true);
            std::time_t currentTime = std::time(nullptr);
            char timeString[100];
            std::string separator = "   ";
            if(type == FileType::CSV)
                separator = ',';
            header += separator;
            header += std::to_string(line_counter);
            header += separator;
            std::strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", std::localtime(&currentTime));
            header += timeString;
            header += separator;
            
            std::chrono::duration<double> t_elapsed = std::chrono::system_clock::now() - start_time;
            auto ms_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t_elapsed);
            header += std::to_string(ms_elapsed.count()/1000000.0f);

            header += separator + msg;
            writer << header << '\n';
            switch (level)
            {
            case LogLevel::INFO:
                info_msg++;
                break;
            case LogLevel::WARNING:
                warn_msg++;
                break;
            case LogLevel::ERROR:
                err_msg++;
                break;
            default:
                break;
            }
        }else
            unavailable_msg();
    }

    void Logger::show(LogLevel level, const char *format, ...)
    {
        // Lock mutex
        std::lock_guard<std::mutex> lg(mtx);

        if(status  == LoggerStatus::Run){
            va_list args;
            va_start(args, format);
            std::string msg = cust_printf(format, args);
            va_end(args);

            std::string header = getLvl(level);
            std::time_t currentTime = std::time(nullptr);
            char timeString[100];
            std::strftime(timeString, sizeof(timeString), "%H:%M:%S", std::localtime(&currentTime));
            header += ' ';
            header += timeString;
            header += ' ';

            std::chrono::duration<double> t_elapsed = std::chrono::system_clock::now() - start_time;
            auto ms_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t_elapsed);
            header += std::to_string(ms_elapsed.count()/1000000.0f);

            header += "s : " + msg;

            std::cout << header << '\n';

            if(!combo_msg)
                switch (level)
                {
                case LogLevel::INFO:
                    info_msg++;
                    break;
                case LogLevel::WARNING:
                    warn_msg++;
                    break;
                case LogLevel::ERROR:
                    err_msg++;
                    break;
                default:
                    break;
                }
        }else
            unavailable_msg();
    }

    void Logger::write_show(LogLevel level, const char *format, ...)
    {
        // Lock mutex
        std::lock_guard<std::mutex> lg(mtx);

        if(status  == LoggerStatus::Run)
        {
            combo_msg = true;

            va_list args;
            va_start(args, format);
            std::string msg = cust_printf(format, args);
            va_end(args);
            
            line_counter++;
            std::string header = getLvl(level);
            std::time_t currentTime = std::time(nullptr);
            char timeString[100];
            {
                std::string separator = "   ";
                if(type == FileType::CSV)
                    separator = ',';
                header += separator;
                header += std::to_string(line_counter);
                header += separator;
                std::strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", std::localtime(&currentTime));
                header += timeString;
                header += separator;
                
                std::chrono::duration<double> t_elapsed = std::chrono::system_clock::now() - start_time;
                auto ms_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t_elapsed);
                header += std::to_string(ms_elapsed.count()/1000000.0f);

                header += separator + msg;
                writer << header << '\n';
            }
            header = getLvl(level, true);
            {
                std::strftime(timeString, sizeof(timeString), "%H:%M:%S", std::localtime(&currentTime));
                header += ' ';
                header += timeString;
                header += ' ';

                std::chrono::duration<double> t_elapsed = std::chrono::system_clock::now() - start_time;
                auto ms_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t_elapsed);
                header += std::to_string(ms_elapsed.count()/1000000.0f);

                header += "s : " + msg;

                std::cout << header << '\n';
            }
            switch (level)
            {
            case LogLevel::INFO:
                info_msg++;
                break;
            case LogLevel::WARNING:
                warn_msg++;
                break;
            case LogLevel::ERROR:
                err_msg++;
                break;
            default:
                break;
            }
        }else
            unavailable_msg();
    }

    void Logger::finish()
    {
        std::lock_guard<std::mutex> lg(mtx);
        if(status  == LoggerStatus::Run)
        {
            stop_time = std::chrono::system_clock::now();
            writer.close();
            resume();
            status  == LoggerStatus::Stop;
        }else
            unavailable_msg();
    }

    Logger& wait(std::string wait_msg);

    Logger& wait_stop();

    Logger::~Logger()
    {
    }
}
