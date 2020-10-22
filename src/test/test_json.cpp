//
// Created by ernest on 2019/7/28.
//

#include <nlohmann/json.hpp>

using nlohmann::json;

int main(int argc, const char * argv[]){
    json j;
    j["pi"] = 3.14;
    return 0;
}