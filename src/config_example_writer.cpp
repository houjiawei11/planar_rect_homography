//
// Created by ernest on 2019/7/29.
//

#include "utils/settings.h"
#include "utils/logging_util.h"

int main(int argc, char * argv[]){
    if (argc != 2){
        LOG_ERROR << "[ERROR] Wrong Usage." << std::endl;
        return 1;
    }

    Settings settings;
    settings.toJSON(argv[1]);
    return 0;
}