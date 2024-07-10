#ifndef LOGGER_HPP
#define LOGGER_HPP
class Logger {
    
public:
    Logger(const char* message);
    Logger(const char* message, unsigned long logInterval);


    void log(const char* message);
    void log(const char* message, unsigned long logInterval);
    void stop();

private:
    unsigned long lastLogTime;
    unsigned long logInterval;
    const char* message;
};
#endif