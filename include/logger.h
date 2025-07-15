#ifndef LOGGER_H
#define LOGGER_H

#include <functional>
#include <cstdarg>
#include <string>

namespace hivepoker {

    class Logger {
    public:
        enum class LogLevel {
            DEBUG,
            INFO,
            WARNING,
            ERROR
        };

        using LogCallback = std::function<void(LogLevel, const std::string&)>;

        static void setLogCallback(LogCallback callback);
        static void setLogLevel(LogLevel level);
        static void log(LogLevel level, const char* format, ...);
        static std::string getLogLevelPrefix(LogLevel level);

    private:
        static LogCallback logCallback;
        static LogLevel globalLogLevel;
        static std::string formatString(const char* format, va_list args);
    };

} // namespace 

#endif // LOGGER_H
