#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <vector>
using std::placeholders::_1;
#define AREA 640*360/5

cv::Point2d past_point(320, 180), present_point(320, 180);

void mysub_callback(rclcpp::Node::SharedPtr node, 
    const sensor_msgs::msg::CompressedImage::SharedPtr msg,
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub)
{
    cv::Mat sgmt_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    //cv::Mat gray;
    //sgmt_img.copyTo(gray);
    cv::threshold(sgmt_img, sgmt_img, 160, 255, cv::THRESH_BINARY); //(4) 이진화
    cv::Mat labels, stats, centroids;
    int lable_cnt =  cv::connectedComponentsWithStats(sgmt_img, labels, stats, centroids); //(5) 레이블링
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);

    std_msgs::msg::Int32 error;
    int max_area = 0;
    cv::Mat floor;
    for(int i = 1; i < lable_cnt; i++) { //'0'은 배경
        double *p = centroids.ptr<double>(i);
        int *q = stats.ptr<int>(i);

        if((q[4] > 100) && (max_area < q[4])) { //면적이 100보다 크고, 최대 면적값보다 클 때
            max_area = q[4];
            //sgmt_img(cv::Rect(q[0],q[1],q[2],q[3])).copyTo(floor);
            floor = sgmt_img(cv::Rect(q[0],q[1],q[2],q[3]));
            //present_point = cv::Point2d(p[0], p[1]);
        }

        //if(q[4] < 100) continue;
        //rectangle(sgmt_img, cv::Rect(q[0],q[1],q[2],q[3]), cv::Scalar(255,0,0), 3);
    }
    //RCLCPP_INFO(node->get_logger(), "lable_cnt : %d", lable_cnt-1);
    

    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    //cv::RETR_CCOMP, cv::RETR_TREE  /  cv::CHAIN_APPROX_SIMPLE, cv::CHAIN_APPROX_NONE
    //cv::findContours(floor, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); 
    //cv::findContours(floor, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); 
    cv::findContours(sgmt_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); 
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);
    //for(int idx=0; idx >= 0; idx = hierarchy[idx][0]) {
    //    drawContours(floor, contours, idx, cv::Scalar(0,255,255), -1, cv::LINE_8, hierarchy);
    //}
    for(int i = 0; i < (int)contours.size(); i++) {
        if (cv::contourArea(contours[i]) > 300) // 면적 기준을 적절히 설정
            //drawContours(floor, contours, i, cv::Scalar(0, 255, 255), -1, cv::LINE_8);
            drawContours(sgmt_img, contours, i, cv::Scalar(255,255,255), -1, cv::LINE_8);
    }

    //cv::cvtColor(floor, floor, cv::COLOR_BGR2GRAY);
    //lable_cnt = cv::connectedComponentsWithStats(floor, labels, stats, centroids);
    ////RCLCPP_INFO(node->get_logger(), "lable_cnt : %d", lable_cnt-1);
    //present_point = cv::Point2d(centroids.at<double>(1, 0), centroids.at<double>(1, 1));
    //RCLCPP_INFO(node->get_logger(), "x : y || %lf : %lf", centroids.at<double>(1, 0), centroids.at<double>(1, 1));
    //cv::cvtColor(floor, floor, cv::COLOR_GRAY2BGR);

    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_BGR2GRAY);
    lable_cnt = cv::connectedComponentsWithStats(sgmt_img, labels, stats, centroids);
    ////RCLCPP_INFO(node->get_logger(), "lable_cnt : %d", lable_cnt-1);
    present_point = cv::Point2d(centroids.at<double>(1, 0), centroids.at<double>(1, 1));
    RCLCPP_INFO(node->get_logger(), "Point(x : y), %lf : %lf", centroids.at<double>(1, 0), centroids.at<double>(1, 1));
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);

    //벽과 가까워지면(바닥 영역이 sgmt_img.rows/5 보다 작아질 때) 조치 필요 -> 미완성 (+왼쪽, 오른쪽 영역 나눠서 처리해보기)
    /*if(max_area < AREA)
    {
        std::cout << "AREA : " << AREA << std::endl;
        std::cout << "max_area : " << max_area << std::endl;
        //present_point = past_point;
    }*/


    error.data = static_cast<int>(sgmt_img.cols/2 - present_point.x);
    pub->publish(error);

    cv::circle(sgmt_img, cv::Point(present_point.x, present_point.y), 5, cv::Scalar(0,0,255), -1);
    cv::imshow("Segmentation Image", sgmt_img);
    cv::imshow("Floor Image", floor);
    //cv::imshow("Gray Image", gray);
    cv::waitKey(1);

    past_point = present_point; //과거좌표 초기화
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("error_Node");

    auto pub = node-> create_publisher<std_msgs::msg::Int32>("error",
        rclcpp::QoS(rclcpp::KeepLast(10)));

    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1, pub);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("gray_segmentation_image/compressed",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}