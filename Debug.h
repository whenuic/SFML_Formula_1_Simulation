#ifndef DEBUG_H
#define DEBUG_H

#include <iostream>
#include <sstream>
#include "Utils.h"

#define DEBUG 0

// LOG with Line info
#define LOG_L(msg) \
    std::cout << __FILE__ << "(" << __LINE__ << "): " << std::endl << msg << std::endl

// LOG only DEBUG == 1
#define LOG_D(msg) \
    if (DEBUG) std::cout << msg << std::endl;

// LOG a message
#define LOG(msg) \
    std::cout << msg << std::endl


#endif
