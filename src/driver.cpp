#include <iostream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

class HandleInputPublisher : public rclcpp::Node
{
public:
    HandleInputPublisher()
        : Node("g923_driver_node")
    {
        this->declare_parameter<std::string>("input_path", "");
        this->get_parameter("input_path", device_path_);

        handle_publisher_ = this->create_publisher<std_msgs::msg::Float32>("handle_position", 1);
        throttle_publisher_ = this->create_publisher<std_msgs::msg::Float32>("throttle_position", 1);
        brake_publisher_ = this->create_publisher<std_msgs::msg::Float32>("brake_position", 1);
        gear_publisher_ = this->create_publisher<std_msgs::msg::Int32>("gear_position", 1);
        
        handle_position_ = 0.0f;
        throttle_position_ = 0.0f;
        brake_position_ = 0.0f;
        gear_position_ = 0;
        
        open_device();
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  
            std::bind(&HandleInputPublisher::publish_latest_state, this)
        );
    }

    void open_device()
    {
        fd_ = open(device_path_.c_str(), O_RDONLY);
        if (fd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open device: %s", device_path_.c_str());
            rclcpp::shutdown();
        }
    }

    void read_loop()
    {
        struct input_event ev;
        while (rclcpp::ok())
        {
            if (read(fd_, &ev, sizeof(ev)) == sizeof(ev))
            {
                if (ev.type == EV_ABS)
                {
                    if (ev.code == ABS_X)
                    {
                        handle_position_ = normalize_handle(ev.value);
                    }
                    else if (ev.code == ABS_Y)
                    {
                        throttle_position_ = normalize_pedal(ev.value);
                    }
                    else if (ev.code == ABS_Z)
                    {
                        brake_position_ = normalize_pedal(ev.value);
                    }
                }
                else if (ev.type == EV_KEY)
                {
                    gear_position_ = determine_gear_position(ev.code, ev.value);
                }
            }
        }
    }

private:
    std::string device_path_;
    int fd_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr handle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gear_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    float handle_position_;
    float throttle_position_;
    float brake_position_;
    int gear_position_;

    float normalize_handle(int value, int max_value = 65535)
    {
        return (static_cast<float>(value) / max_value) * 2.0f - 1.0f;
    }

    float normalize_pedal(int value, int max_value = 255)
    {
        return 1.0f - (static_cast<float>(value) / max_value);
    }

    int determine_gear_position(int code, int value)
    {
        if (value == 1)
        {
            switch (code)
            {
                case 300: return 1;
                case 301: return 2;
                case 302: return 3;
                case 303: return 4;
                case 704: return 5;
                case 705: return 6;
                default: return 0;
            }
        }
        return gear_position_;
    }

    void publish_latest_state()
    {
        publish(handle_publisher_, handle_position_);
        publish(throttle_publisher_, throttle_position_);
        publish(brake_publisher_, brake_position_);
        publish(gear_publisher_, gear_position_);
    }

    void publish(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher, float value)
    {
        auto message = std_msgs::msg::Float32();
        message.data = value;
        publisher->publish(message);
    }

    void publish(rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher, int value)
    {
        auto message = std_msgs::msg::Int32();
        message.data = value;
        publisher->publish(message);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandleInputPublisher>();
    std::thread([&node]() { node->read_loop(); }).detach();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
