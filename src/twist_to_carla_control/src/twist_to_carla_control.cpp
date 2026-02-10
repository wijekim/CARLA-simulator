#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>

class TwistToCarlaControl : public rclcpp::Node
{
public:
  TwistToCarlaControl() : Node("twist_to_carla_control")
  {
    // 파라미터 선언    
    this->declare_parameter("max_steering_angle", 0.3);  // 최대 조향각 (라디안)
    this->declare_parameter("max_throttle", 1.0);
    this->declare_parameter("max_brake", 1.0);
    this->declare_parameter("wheelbase", 2.87);  // 차량 축거 (m)
    this->declare_parameter("max_speed", 2.0);  // 최대 속도 (m/s)
    
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
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto control_msg = carla_msgs::msg::CarlaEgoVehicleControl();

    // Linear velocity (x) -> Throttle/Brake
    double desired_speed = msg->linear.x;
    
    if (desired_speed > 0.0) {
      // 전진: Throttle 계산
      control_msg.throttle = std::min(desired_speed / max_speed_, max_throttle_);
      control_msg.brake = 0.0;
    } else if (desired_speed < 0.0) {
      // 후진: Reverse와 Throttle 사용
      control_msg.throttle = std::min(std::abs(desired_speed) / max_speed_, max_throttle_);
      control_msg.brake = 0.0;
      control_msg.reverse = true;
    } else {
      // 정지: Brake 적용
      control_msg.throttle = 0.0;
      control_msg.brake = 1.0;
      control_msg.reverse = false;
    }

    // Angular velocity (z) -> Steering
    // Ackermann 조향 모델 사용
    double angular_vel = msg->angular.z;
    
    if (std::abs(desired_speed) > 0.01) {
      // 회전 반경 계산: R = v / ω
      double turning_radius = desired_speed / angular_vel;
      
      // Ackermann 조향각 계산: δ = atan(L / R)
      double steering_angle = std::atan(wheelbase_ / turning_radius);
      
      // 조향각 정규화 [-1, 1]
      control_msg.steer = -std::clamp(steering_angle / max_steering_, -1.0, 1.0);
    } else {
      // 제자리 회전
      control_msg.steer = (angular_vel > 0) ? 1.0 : -1.0;
    }

    // Hand brake는 기본적으로 사용 안 함
    control_msg.hand_brake = false;
    control_msg.manual_gear_shift = false;
    control_msg.gear = 0;

    // 메시지 발행
    control_pub_->publish(control_msg);

    // 디버그 정보 출력 (주기적으로)
    static int count = 0;
    if (++count % 20 == 0) {
      RCLCPP_INFO(this->get_logger(), 
        "Twist: linear=%.2f, angular=%.2f -> Control: throttle=%.2f, brake=%.2f, steer=%.2f",
        msg->linear.x, msg->angular.z, 
        control_msg.throttle, control_msg.brake, control_msg.steer);
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