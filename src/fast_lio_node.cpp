//
// Created by soulde on 2023/7/7.
//

#include "Fast_Lio.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, Fast_Lio::SigHandle);

    auto node = std::make_shared<Fast_Lio>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}