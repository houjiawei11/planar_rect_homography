#ifndef PLANAR_RECT_LOGGING_UTIL_H
#define PLANAR_RECT_LOGGING_UTIL_H


#define CMAKE_BUILD_TYPE "@CMAKE_BUILD_TYPE@"

#include <iostream>

// make nice log message
#define LOG_INFO  std::cout << "[INFO, " <<basename(__FILE__)<< "] "
#define LOG_WARN  std::cout << "[WARN, " <<basename(__FILE__)<< "] "
#define LOG_DEBUG  if (CMAKE_BUILD_TYPE != "Release") std::cout << "[DEBUG, "<<basename(__FILE__)<<"] "
#define LOG_ERROR    std::cerr << "[ERROR, "<<basename(__FILE__)<<"] "

#endif //PLANAR_RECT_LOGGING_UTIL_H
