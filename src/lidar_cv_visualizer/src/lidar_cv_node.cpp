#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>

#define WIDTH 600
#define HEIGHT 600
#define MAX_RANGE 40.0  // 시각화할 최대 거리 (미터)
#define SCALE ((WIDTH / 2.0) / MAX_RANGE)

class LidarCvNode : public rclcpp::Node {
public:
    LidarCvNode() : Node("lidar_cv_visualizer") {
        // 1. QoS 설정 (SensorDataQoS는 보통 Best Effort 방식)
        auto qos_profile = rclcpp::SensorDataQoS();

        // 2. Subscriber 생성
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_profile, std::bind(&LidarCvNode::scan_callback, this, std::placeholders::_1));

        // 3. OpenCV 창 및 비디오 저상 설정
        cv::namedWindow("Lidar_point", cv::WINDOW_AUTOSIZE);
        
        std::string filename = "lidar_record.mp4";
        int fcc = cv::VideoWriter::fourcc('m','p','4','v');
        video_writer_.open(filename, fcc, 20.0, cv::Size(WIDTH, HEIGHT));

        RCLCPP_INFO(this->get_logger(), "LiDAR OpenCV Node Started. Max Range: %.1fm", MAX_RANGE);
    }

    ~LidarCvNode() {
        if (video_writer_.isOpened()) video_writer_.release();
        cv::destroyAllWindows();
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 배경을 흰색으로 초기화
        cv::Mat img = cv::Mat(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
        
        int center_x = WIDTH / 2;
        int center_y = HEIGHT / 2;

        // 기준점(차량 위치) 그리기
        cv::circle(img, cv::Point(center_x, center_y), 5, cv::Scalar(255, 0, 0), -1);

        // LiDAR 데이터 처리 (단일 루프로 최적화)
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float r = msg->ranges[i];

            // 유효하지 않은 거리값 제외
            if (r < msg->range_min || r > msg->range_max || std::isinf(r) || std::isnan(r)) {
                continue;
            }

            float angle = msg->angle_min + msg->angle_increment * i;

            // 좌표 변환 (ROS 표준: x 앞, y 왼쪽 -> OpenCV: x 오른쪽, y 아래)
            // ROS x축(정면)은 OpenCV의 위쪽 방향이 되어야 함
            float x = r * std::cos(angle);
            float y = r * std::sin(angle);

            // 이미지 좌표계 매핑 (단위: 픽셀)
            int px = center_x - (int)(y * SCALE); // y를 왼쪽/오른쪽으로 반전
            int py = center_y - (int)(x * SCALE); // x를 위/아래로 반전

            // 화면 범위 내에 있는 경우만 그리기
            if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
                cv::circle(img, cv::Point(px, py), 2, cv::Scalar(0, 0, 255), -1);
            }
        }

        // 결과 화면 표시
        cv::imshow("Lidar_point", img);
        
        if (video_writer_.isOpened()) {
            video_writer_.write(img);
        }

        cv::waitKey(1); // GUI 갱신을 위해 필수
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    cv::VideoWriter video_writer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCvNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}