#include "logger.h"
#include <cstdio>

namespace hivepoker {

    Logger::LogCallback Logger::logCallback = nullptr;
    Logger::LogLevel Logger::globalLogLevel = Logger::LogLevel::DEBUG;

    void Logger::setLogCallback(LogCallback callback) {
        logCallback = callback;
    }

    void Logger::setLogLevel(LogLevel level) {
        globalLogLevel = level;
    }

    void Logger::log(LogLevel level, const char* format, ...) {
        if (logCallback && level >= globalLogLevel) {
            va_list args;
            va_start(args, format);

            std::string message = formatString(format, args);

            va_end(args);

            logCallback(level, message);
        }
    }

    std::string Logger::formatString(const char* format, va_list args) {
        const int bufferSize = 256;
        char buffer[bufferSize];

        int result = std::vsnprintf(buffer, bufferSize, format, args);

        if (result >= 0 && result < bufferSize) {
            return std::string(buffer);
        } else {
            return "Error formatting string.";
        }
    }


    std::string Logger::getLogLevelPrefix(LogLevel level) {
        switch (level) {
            case LogLevel::DEBUG:
                return "[DEBUG] ";
            case LogLevel::INFO:
                return "[INFO] ";
            case LogLevel::WARNING:
                return "[WARNING] ";
            case LogLevel::ERROR:
                return "[ERROR] ";
            default:
                return "";
        }
    }

} // namespace 
