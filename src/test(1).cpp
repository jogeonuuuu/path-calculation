#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <vector>
using std::placeholders::_1;

#define AREA_DIFFERENCE 16000
#define FLOOR_HEIGHT 360*3/4

cv::Point2d past_point(320, 180), present_point(320, 180);

void mysub_callback(rclcpp::Node::SharedPtr node, 
    const sensor_msgs::msg::CompressedImage::SharedPtr msg,
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub)
{
    // Step 1: 바닥 객체 검출
    cv::Mat sgmt_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    cv::threshold(sgmt_img, sgmt_img, 160, 255, cv::THRESH_BINARY); //이진화
    cv::Mat labels, stats, centroids;
    int lable_cnt =  cv::connectedComponentsWithStats(sgmt_img, labels, stats, centroids); //레이블링
    //cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);

    cv::Mat original;
    sgmt_img.copyTo(original);

    // Step 2(1): 이동할 바닥 객체(면적이 가장 큰 객체) 검출
    /*int max_area = 0;
    //int floor_max_height;
    for(int i = 1; i < lable_cnt; i++) { //'0'은 배경
        double *p = centroids.ptr<double>(i);
        int *q = stats.ptr<int>(i);

        if((q[4] > 100) && (max_area < q[4])) { //면적이 100보다 크고, 최대 면적값보다 클 때
            max_area = q[4];
            present_point = cv::Point2d(p[0], p[1]);
            //floor_max_height = q[1];
        }
        
        // Step 3-1: 레이블링 -> 갈 수 없는 바닥 객체 지우기
        //if(q[4] < 50*50)
        //    rectangle(sgmt_img, cv::Rect(q[0],q[1],q[2],q[3]), cv::Scalar(0,0,0), -1);
    }*/


    // Step 3-2(1): 외곽선 검출 -> 갈 수 없는 바닥 객체 지우기
    //cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point>> contours;
    //RETR_EXTERNAL, RETR_LIST || CHAIN_APPROX_NONE, CHAIN_APPROX_SIMPLE
    cv::findContours(sgmt_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //RETR_LIST 사용하면 X
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);
    for(int i = 0; i < (int)contours.size(); i++) {
        if (cv::contourArea(contours[i]) < 50*50) // 면적 기준을 적절히 설정 (contours[i]: 외곽선 개수)
            drawContours(sgmt_img, contours, i, cv::Scalar(0,0,0), -1);
    }
    // Step 3-2(2)
    /*cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    //RETR_CCOMP, RETR_TREE || CHAIN_APPROX_NONE, CHAIN_APPROX_SIMPLE
    cv::findContours(sgmt_img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);
    for(int i = 0; i < (int)contours.size(); i++) {
        // 부모 외곽선만 확인 (hierarchy[i][3]이 -1이면 부모 외곽선임)
        if (hierarchy[i][3] == -1 && cv::contourArea(contours[i]) < 50*50)
            drawContours(sgmt_img, contours, i, cv::Scalar(0,0,0), -1);
    }*/



    
    // Step 2(2): 이동할 바닥 객체(면적이 가장 큰 객체) 검출
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_BGR2GRAY);
    lable_cnt = cv::connectedComponentsWithStats(sgmt_img, labels, stats, centroids);
    //cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);
    int max_area = 0;
    int floor_max_height_py;
    int floor_max_height_distance;
    for(int i = 1; i < lable_cnt; i++) { //'0'은 배경
        double *p = centroids.ptr<double>(i);
        int *q = stats.ptr<int>(i);

        if((q[4] > 100) && (max_area < q[4])) { //면적이 100보다 크고, 최대 면적값보다 클 때
            max_area = q[4];
            present_point = cv::Point2d(p[0], p[1]);
            floor_max_height_py = q[1];
            floor_max_height_distance = q[3];
        }
    }
    std::cout << floor_max_height_py << " : " << floor_max_height_distance << std::endl;


    //바닥 영역 왼쪽/오른쪽 나누기 -> 이미 가장 큰 바닥 객체를 가지고 처리하기 때문에 필요 없을 거같음. -> X
    int y = floor_max_height_py - 1;
    cv::Mat floor_left = sgmt_img(cv::Rect(0, y, sgmt_img.cols/2, floor_max_height_distance));
    cv::Mat floor_right = sgmt_img(cv::Rect(sgmt_img.cols/2-1, y, sgmt_img.cols/2, floor_max_height_distance));
    cv::Mat left_centroids;
    cv::connectedComponentsWithStats(floor_left, labels, stats, left_centroids);
    int floor_left_area = stats.at<int>(1, 4);
    cv::Mat right_centroids;
    cv::connectedComponentsWithStats(floor_right, labels, stats, right_centroids);
    int floor_right_area = stats.at<int>(1, 4);
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);

    std::cout << "floor_left_area : floor_right_area || " << floor_left_area << " : " << floor_right_area << std::endl;
    if(floor_left_area + AREA_DIFFERENCE < floor_right_area)
    {
        present_point = cv::Point2d(floor_right.cols + (int)right_centroids.at<double>(1,0), 
            y + (int)right_centroids.at<double>(1,1));
        cv::circle(floor_right, cv::Point((int)right_centroids.at<double>(1,0), (int)right_centroids.at<double>(1,1)), 5, 0, -1);
    }
    else if(floor_left_area > floor_right_area + AREA_DIFFERENCE)
    {
        present_point = cv::Point2d((int)left_centroids.at<double>(1,0), 
            y + (int)left_centroids.at<double>(1,1));
        cv::circle(floor_right, cv::Point((int)left_centroids.at<double>(1,0), (int)left_centroids.at<double>(1,1)), 5, 0, -1);
    }

    std_msgs::msg::Int32 error;
    if(floor_max_height_py < FLOOR_HEIGHT)
        error.data = static_cast<int>(sgmt_img.cols/2 - present_point.x);
    else error.data = 180;
    pub->publish(error);


    cv::circle(sgmt_img, cv::Point(present_point.x, present_point.y), 5, cv::Scalar(0,0,255), -1);
    cv::imshow("original Image", original);
    cv::imshow("Segmentation Image", sgmt_img);
    cv::imshow("floor_left", floor_left);
    cv::imshow("floor_right", floor_right);
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
