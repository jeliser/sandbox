#include <iostream>
#include <log4cplus/logger.h>
#include <log4cplus/loglevel.h>
#include <log4cplus/configurator.h>
#include <log4cplus/fileappender.h>

#define MYLOG1_INFO(logEvent) LOG4CPLUS_INFO (log4cplus::Logger::getInstance("my.logger1"), logEvent)
#define MYLOG2_INFO(logEvent) LOG4CPLUS_INFO (log4cplus::Logger::getInstance("my.logger2"), logEvent)

int main(int argc, char**argv)
{
    try
    {
        log4cplus::PropertyConfigurator::doConfigure("log.properties");
    }
    catch( ... )
    {
    std::cerr<<"Exception occured while opening log.properties\n";
    return -1;
    }
    MYLOG1_INFO("hello world!");
    MYLOG2_INFO("hello world!");
    return 0;
}
