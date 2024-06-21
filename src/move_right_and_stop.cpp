#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <memory>

using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("service_move_right_and_stop") {

    srv_ = create_service<SetBool>(
        "move_right_and_stop", std::bind(&ServerNode::callback, this, _1, _2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<SetBool>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void callback(const std::shared_ptr<SetBool::Request> request,
                const std::shared_ptr<SetBool::Response> response) {
    auto message = geometry_msgs::msg::Twist();
    if (request->data) {
      message.linear.x = 0.2;
      message.angular.z = -0.2;
      response->message = "Robot moving in a CW circle";
    } else {
      message.linear.x = 0.0;
      message.angular.z = 0.0;
      response->message = "Robot stopped";
    }
    response->success = true;
    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}