#include "utils.hpp"
#include <cstring>
#include <fstream>
#include <gtsam/slam/dataset.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

class imuPublisher : public rclcpp::Node
{
    public:
    imuPublisher() : Node("imuPublisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("imuData");
        timer_ = this->create_wall_timer(200ms, std::bind(&imuPublisher::callback, this));
    }
    filePath = "data/imuAndGPSdata.csv";
    std::string data_file;
    boost::program_options::variables_map var_map = parseOptions(argc, argv);
    data_file = gtsam::findExampleDataFile(var_map["filePath"].as<string>());
    
    std::ifstream file();
    std::string value;
    
    

    

    private:
    void callback()
    {
        auto msg = std_msgs::msg::String();
        std::getline(file, value, ','); //get data from the file
        //int type = std::stoi(value.c_str());
        msg.data = value;
        RCLCPP_INFO(this->get_logger(),"Publishing: '&s'", msg.data.c_str());
        publisher_->publish(msg);

    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_ = 1;

};

int main (int argc, char* argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imuPublisher>());
    rclcpp::shutdown;

    return 0;
}