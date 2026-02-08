#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>

class TwistToCarlaControl : public rclcpp::Node
{
public:
  TwistToCarlaControl() : Node("twist_to_carla_control_nav")
  {
    // 파라미터 선언    
    this->declare_parameter("max_steering_angle", 0.7);  // 최대 조향각 (라디안)
    this->declare_parameter("max_throttle", 1.0);
    this->declare_parameter("max_brake", 1.0);
    this->declare_parameter("wheelbase", 2.87);  // 차량 축거 (m)
    this->declare_parameter("max_speed", 1.5);  // 최대 속도 (m/s)
    
    // 파라미터 가져오기
    max_steering_ = this->get_parameter("max_steering_angle").as_double();
    max_throttle_ = this->get_parameter("max_throttle").as_double();
    max_brake_ = this->get_parameter("max_brake").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();

    // Subscriber 생성
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TwistToCarlaControl::twistCallback, this, std::placeholders::_1));

    // Publisher 생성
    control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
      "/carla/hero/vehicle_control_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "Twist to CARLA Control Node Started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /cmd_vel");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /carla/hero/vehicle_control_cmd");
  }

private:
  private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto control_msg = carla_msgs::msg::CarlaEgoVehicleControl();
    double desired_speed = msg->linear.x;

    // 1. 전진/후진 및 정지 로직 고도화
    // 부동소수점 오차 및 관성을 고려하여 0.05m/s 미만은 정지로 판단
    if (std::abs(desired_speed) < 0.2) {
      control_msg.throttle = 0.0;
      control_msg.brake = 1.0;      // 0.5에서 1.0(풀 브레이크)으로 상향
      control_msg.reverse = false;
    } 
    else if (desired_speed > 0.0) {
      control_msg.throttle = std::min(desired_speed / max_speed_, max_throttle_);
      control_msg.brake = 0.0;
      control_msg.reverse = false;
    } 
    else { // desired_speed < -0.05
      control_msg.throttle = std::min(std::abs(desired_speed) / max_speed_, max_throttle_);
      control_msg.brake = 0.0;
      control_msg.reverse = true;
    }

    // 2. 조향(Steering) 로직: 저속 시 급격한 조향 튀기 방지
    double angular_vel = msg->angular.z;
    if (std::abs(desired_speed) > 0.15) { // 기준을 0.01에서 0.1로 상향
      double turning_radius = desired_speed / angular_vel;
      double steering_angle = std::atan(wheelbase_ / turning_radius);
      control_msg.steer = -std::clamp(steering_angle / max_steering_, -1.0, 1.0);
    } else {
      // 아주 저속이거나 정지 시에는 조향각을 중립으로 하거나 이전 값 유지
      // 제자리 회전 시의 steering 튀기 방지를 위해 0.0으로 처리
      control_msg.steer = 0.0;
    }

    // 나머지 설정 유지
    control_msg.hand_brake = false;
    control_msg.manual_gear_shift = false;
    control_msg.gear = 0;

    control_pub_->publish(control_msg);

    // 디버그 출력 주기 조정 (20 -> 10: 더 자주 모니터링)
    static int count = 0;
    if (++count % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), 
        "Twist: v=%.2f, w=%.2f -> T: %.2f, B: %.2f, S: %.2f, R: %s",
        msg->linear.x, msg->angular.z, 
        control_msg.throttle, control_msg.brake, control_msg.steer,
        control_msg.reverse ? "ON" : "OFF");
    }
  }

  // Subscriber & Publisher
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;

  // 파라미터
  double max_steering_;
  double max_throttle_;
  double max_brake_;
  double wheelbase_;
  double max_speed_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistToCarlaControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}