#include "car_config.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <ros/package.h>

int main(int argc, char** argv)
{
    YAML::Node node;
    node["speed_to_erpm_gain"] = car_config::SPEED_TO_ERPM;
    node["speed_to_erpm_offset"] = 0;
    node["steering_angle_to_servo0_gain"] = 0.5;
    node["steering_angle_to_servo0_offset"] = 0.5;
    node["steering_angle_to_servo1_gain"] = 0.5;
    node["steering_angle_to_servo1_offset"] = 0.5;
    node["wheelbase"] = car_config::WHEELBASE;
    node["brake"] = 10;
    node["acceleration"] = 5;
    node["deceleration"] = 10;

    std::string package_path = ros::package::getPath("vesc_sim");
    std::ofstream fout(package_path + "/config/car_config.yaml");
    fout << node;
}
