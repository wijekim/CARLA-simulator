#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"  
#include "cv_bridge/cv_bridge.hpp"     
#include "opencv2/opencv.hpp"
#include <memory>

class CameraVisualizer : public rclcpp::Node {
public:
    CameraVisualizer() : Node("camera") {
       
        auto qos_profile = rclcpp::QoS(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        // 토픽명 설정
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/carla/ego_vehicle/rgb/image", qos_profile,
            std::bind(&CameraVisualizer::callback, this, std::placeholders::_1));

        // OpenCV 창 생성
        cv::namedWindow("CARLA Camera Output", cv::WINDOW_NORMAL);
        cv::resizeWindow("CARLA Camera Output", 640, 360);

        // 비디오 저장 설정
        video_writer_.open("output_video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 20.0, cv::Size(800, 600));

        RCLCPP_INFO(this->get_logger(), "Camera Visualizer Node Started.");
    }

    ~CameraVisualizer() {
        if (video_writer_.isOpened()) video_writer_.release();
        cv::destroyAllWindows();
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // ROS Image 메시지를 OpenCV Mat으로 변환
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            if (frame.empty()) return;

            // 비디오  초기 설정 (첫 프레임 기준)
            if (!is_writer_init_) {
                video_writer_.open("output_video.mp4", cv::VideoWriter::fourcc('m','p','4','v'), 20.0, frame.size());
                is_writer_init_ = true;
            }

            // 영상 파일 기록
            if (video_writer_.isOpened()) {
                video_writer_.write(frame);
            }

            // 실시간 화면 출력
            cv::imshow("CARLA Camera Output", frame);
            cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter video_writer_;
    bool is_writer_init_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraVisualizer>());
    rclcpp::shutdown();
    return 0;
}