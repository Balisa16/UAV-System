#include <logger.hpp>

namespace EMIRO{
	void Logger::resume()
    {
        std::time_t time = std::chrono::system_clock::to_time_t(start_time);
        char start_str[100];
        std::strftime(start_str, sizeof(start_str), "%Y-%m-%d %H:%M:%S", std::localtime(&time));

        time = std::chrono::system_clock::to_time_t(stop_time);
        char stop_str[100];
        std::strftime(stop_str, sizeof(stop_str), "%Y-%m-%d %H:%M:%S", std::localtime(&time));

        std::chrono::duration<double> elapsed_seconds = stop_time - start_time;

        std::cout << "\n\n\033[34m\033[1mFlight Resume :\033[0m\n\tStart Time\t: " << start_str << 
            "\n\tStop Time\t: " << stop_str << 
            "\n\tFlight Duration\t: " << std::to_string(elapsed_seconds.count()) << " seconds\nErrors\t\t: " << err_msg << 
            " message\nWarnings\t: " << warn_msg <<
            " message\nInformations\t: " << info_msg << " message\n";
    }

    void Logger::init(std::string filename, FileType type)
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
    }

    void Logger::start()
    {
        start_time = std::chrono::system_clock::now();
        writer.open(this->full_filename);
        if(type == FileType::CSV)
            writer << "Level,Id,Datetime,Flight Time(s),Message\n";
        line_counter = 0;
    }

    std::string Logger::getLvl(LogLevel lvl)
    {
        std::string lvl_string = "";
        switch (lvl)
        {
        case LogLevel::INFO:
            lvl_string += "\033[32m[INFO ]\033[37m";
            break;
        case LogLevel::WARNING:
            lvl_string += "\033[33m[WARN ]\033[37m";
            break;
        case LogLevel::ERROR:
            lvl_string += "\033[31m[ERROR]\033[37m";
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
        va_end(args);
        return buffer;
    }


    void Logger::write(LogLevel level, const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        Logger logger;
        std::string msg = cust_printf(format, args);
        va_end(args);

        line_counter++;
        std::string header = getLvl(level);
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
        std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start_time;

        header += std::to_string(elapsed_seconds.count());
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
    }

    void Logger::show(LogLevel level, const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        Logger logger;
        std::string msg = cust_printf(format, args);
        va_end(args);

        std::string header = getLvl(level);
        std::time_t currentTime = std::time(nullptr);
        char timeString[100];
        std::strftime(timeString, sizeof(timeString), "%H:%M:%S", std::localtime(&currentTime));
        header += ' ';
        header += timeString;
        header += ' ';


        std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start_time;
        int int_duration = elapsed_seconds.count();
        header += std::to_string(int_duration);
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
    }

    void Logger::write_show(LogLevel level, const char *format, ...)
    {
        combo_msg = true;

        va_list args;
        va_start(args, format);
        Logger logger;
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
            std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start_time;

            header += std::to_string(elapsed_seconds.count());
            header += separator + msg;
            writer << header << '\n';
        }
        header = getLvl(level);
        {
            std::strftime(timeString, sizeof(timeString), "%H:%M:%S", std::localtime(&currentTime));
            header += ' ';
            header += timeString;
            header += ' ';


            std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start_time;
            int int_duration = elapsed_seconds.count();
            header += std::to_string(int_duration);
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
    }

    void Logger::finish()
    {
        stop_time = std::chrono::system_clock::now();
        writer.close();
        resume();
    }
}