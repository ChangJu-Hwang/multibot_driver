#include "fake_driver/fake_odom.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto fake_odometry = std::make_shared<ISR_M2::FakeOdom>();
    
    rclcpp::spin(fake_odometry);
    rclcpp::shutdown();

    return 0;
}