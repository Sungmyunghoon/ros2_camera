#include "rclcpp/rclcpp.hpp" // ROS2를 사용할 헤더 파일
#include "sensor_msgs/msg/compressed_image.hpp" // 
#include "opencv2/opencv.hpp" // opencv를 사용할 헤더 파일
#include <memory> 
#include <functional>
#include <iostream>
using std::placeholders::_1;
  
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR); // COLOR로 전송할 객체 선언
    cv::Mat gray_frame; // GRAY를 담을 객체
    cv::Mat binary_frame; // binary(이진화)를 담을 객체
    
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY); // COLOR를 GRAY로 변경하는 함수
    cv::threshold(gray_frame, binary_frame, 128, 255, cv::THRESH_BINARY); // COLOR를 binary로 변경할 함수

    cv::imshow("frame",frame); //화면에 COLOR 출력
    cv::imshow("gray_frame",gray_frame); // 화면에 GRAY 출력
    cv::imshow("binary_frame",binary_frame); // 화면에 binary 출력

    cv::waitKey(1);
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
}
int main(int argc, char* argv[]) // main 함수
{
    rclcpp::init(argc, argv); // ROS2 초기화
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl"); // 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //통신의 품질을 설정 ( best_effort()가 가장 좋은 품질로 됨 )
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn; // 이미지 메시지를 처리하기 위한 콜백 함수를 지정
    fn = std::bind(mysub_callback, node, _1); // bind로 콜백 함수에 전달할 인자를 설정
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",qos_profile,fn);// 구독자를 생성하는 역할
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

