#include "utils.hpp"

class imuPublisher : public rclcpp::Node
{
    public:
    imuPublisher() : Node("imuPublisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("imuData");
        timer_ = this->create_wall_timer(200ms, std::bind(&imuPublisher::callback, this));
    }
    filePath = "data/imuAndGPSdata.csv"

    private:
    void callback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = ;
        RCLCPP_INFO(this->get_logger(),"Publishing: '&s'", msg.data.c_str());
        publisher_->publish(msg);

    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

};

int main (int argc, char* argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imuPublisher>());
    rclcpp::shutdown;

    return 0;
}