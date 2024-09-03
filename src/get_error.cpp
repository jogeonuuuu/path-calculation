#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <vector>
using std::placeholders::_1;

#define AREA_DIFFERENCE 25000
#define FLOOR_HEIGHT 360*3/4

cv::Point2d present_point(320, 180);

void mysub_callback(rclcpp::Node::SharedPtr node, 
    const sensor_msgs::msg::CompressedImage::SharedPtr msg,
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub)
{
    //auto start = std::chrono::high_resolution_clock::now(); //count start(seconds)

    // Step 1: 바닥 객체 검출
    cv::Mat sgmt_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    cv::Mat original;
    sgmt_img.copyTo(original);

    cv::threshold(sgmt_img, sgmt_img, 230, 255, cv::THRESH_BINARY); // 이진화 ( || cv::THRESH_BINARY_INV )
    cv::Mat labels, stats, centroids;
    int lable_cnt = cv::connectedComponentsWithStats(sgmt_img, labels, stats, centroids); // 레이블링

    // Step 2-1(1): 외곽선 검출 -> 갈 수 없는 바닥 영역 지우기
    std::vector<std::vector<cv::Point>> contours;
    // RETR_EXTERNAL, RETR_LIST || CHAIN_APPROX_NONE, CHAIN_APPROX_SIMPLE
    cv::findContours(sgmt_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //RETR_LIST 사용하면 X
    //cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);
    for(int i = 0; i < (int)contours.size(); i++) {
        if (cv::contourArea(contours[i]) < 100*50) // 면적 기준을 적절히 설정 (contours[i]: 외곽선 개수)
            drawContours(sgmt_img, contours, i, 0, -1); //test cv::Scalar(0,255,0)
    }
    // Step 2-1(2)
    /*std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    //RETR_CCOMP, RETR_TREE || CHAIN_APPROX_NONE, CHAIN_APPROX_SIMPLE
    cv::findContours(sgmt_img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);
    for(int i = 0; i < (int)contours.size(); i++) {
        // 부모 외곽선만 확인 (hierarchy[i][3]이 -1이면 부모 외곽선임)
        if (hierarchy[i][3] == -1 && cv::contourArea(contours[i]) < 50*50)
            drawContours(sgmt_img, contours, i, cv::Scalar(0,0,0), -1);
    }*/


    // Step 3: 이동할 바닥 객체(면적이 가장 큰 객체) 검출
    //cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_BGR2GRAY);
    lable_cnt = cv::connectedComponentsWithStats(sgmt_img, labels, stats, centroids);
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
        
        // Step 2-2: 레이블링 -> 갈 수 없는 바닥 영역 지우기
        //if(q[4] < 50*50)
        //    rectangle(sgmt_img, cv::Rect(q[0],q[1],q[2],q[3]), cv::Scalar(0,0,0), -1);
    }


    // Step 4-1: 바닥 영역 왼쪽 나누기 + 가장 근접한 장애물과의 거리
    cv::Mat floor_left = sgmt_img(cv::Rect(0, floor_max_height_py, sgmt_img.cols/2, floor_max_height_distance)); //gray image
    cv::Mat left_centroids;
    lable_cnt = cv::connectedComponentsWithStats(floor_left, labels, stats, left_centroids);
    int floor_left_area = stats.at<int>(1, 4);

    cv::Point left_min_point(sgmt_img.cols/2, sgmt_img.rows-1);
    int left_point_x = 0;
    double left_min_distance = floor_max_height_py;
    if(lable_cnt > 1) {
        std::vector<std::vector<cv::Point>> left_contours;
        cv::findContours(~floor_left, left_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //RETR_LIST 사용하면 X
        
        for(int i = 0; i < (int)left_contours.size(); i++) {
            if(cv::contourArea(left_contours[i]) < 10*10) continue; // 잡음 제거
            for (int j = 0; j < (int)left_contours[i].size(); j++) {
                cv::Point pnt = left_contours[i][j];
                for(int k = floor_left.cols/2; k < floor_left.cols; k++) { // 중심이 아닌 x축(0.5~1.0)으로 가장 가까운 거리 찾기
                    double distance = sqrt(pow(pnt.x - k, 2) + pow(pnt.y - (floor_left.rows - 1), 2));
                    if (left_min_distance > distance) {
                        left_min_distance = distance;
                        left_min_point = cv::Point(pnt.x, pnt.y + floor_max_height_py);
                        left_point_x = k;
                    }
                }
            }
        }
    }

    // Step 4-2: 바닥 영역 오른쪽 나누기 + 가장 근접한 장애물과의 거리
    cv::Mat floor_right = sgmt_img(cv::Rect(sgmt_img.cols/2-1, floor_max_height_py, sgmt_img.cols/2, floor_max_height_distance)); //gray image
    cv::Mat right_centroids;
    lable_cnt = cv::connectedComponentsWithStats(floor_right, labels, stats, right_centroids);
    int floor_right_area = stats.at<int>(1, 4);

    cv::Point right_min_point(sgmt_img.cols/2, sgmt_img.rows-1);
    int right_point_x = 0;
    double right_min_distance = floor_max_height_py;
    if(lable_cnt > 1) {
        std::vector<std::vector<cv::Point>> right_contours;
        cv::findContours(~floor_right, right_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for(int i=0; i < (int)right_contours.size(); i++) {
            if(cv::contourArea(right_contours[i]) < 10*10) continue; // 잡음 제거
            for (int j = 0; j < (int)right_contours[i].size(); j++) {
                cv::Point pnt = right_contours[i][j];
                for(int k = 0; k < floor_right.cols/2; k++) { // 중심이 아닌 x축(0.0~0.5)으로 가장 가까운 거리 찾기
                    double distance = sqrt(pow(pnt.x - k, 2) + pow(pnt.y - (floor_right.rows - 1), 2));
                    if (right_min_distance > distance) {
                        right_min_distance = distance;
                        right_min_point = cv::Point(pnt.x + sgmt_img.cols/2, pnt.y + floor_max_height_py);
                        right_point_x = k;
                    }
                }
            }
        }
    }
    //std::cout << floor_left_area << " : " << floor_right_area << std::endl;


    // Step 5: 장애물과 일정 거리 이내로 근접해지면 급회전
    std_msgs::msg::Int32 error;
    bool ox = true;
    if((left_min_distance < 50) || (right_min_distance == 0)) {
        error.data = -180;
        ox = false;
    }
    // Step 6: 바닥의 왼쪽과 오른쪽 영역의 차이가 AREA_DIFFERENCE 보다 큰 영역의 무게중심으로 error값 저장
    if(ox) {
        if(((floor_left_area - floor_right_area) > AREA_DIFFERENCE)) {
            present_point = cv::Point2d((int)left_centroids.at<double>(1,0), 
                floor_max_height_py + (int)left_centroids.at<double>(1,1));
        }
        else if(((floor_right_area - floor_left_area) > AREA_DIFFERENCE)) {
            present_point = cv::Point2d(floor_right.cols + (int)right_centroids.at<double>(1,0), 
                floor_max_height_py + (int)right_centroids.at<double>(1,1));
        }


        // Step 7: error값 퍼블리싱 (벽과 너무 가까워지면 gain값이 커지도록 설계)
        if(floor_max_height_py < FLOOR_HEIGHT)
            error.data = static_cast<int>(sgmt_img.cols/2 - present_point.x);
        /*else if(floor_max_height_py > FLOOR_HEIGHT) { //바닥 영역이 더 넓은 방향으로 회전
            if(floor_left_area > floor_right_area) 
                error.data = 180;
            else if(floor_left_area < floor_right_area)
                error.data = -180;
        }*/
        else error.data = -180;
    }

    pub->publish(error);
    RCLCPP_INFO(node->get_logger(), "error: %d", error.data);


    //test
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);
    cv::circle(sgmt_img, present_point, 3, cv::Scalar(0,0,255), -1);
    cv::arrowedLine(sgmt_img, cv::Point(sgmt_img.cols/2 - 1, sgmt_img.rows - 1), present_point, cv::Scalar(0,0,255), 2);

    //cv::circle(sgmt_img, left_min_point, 3, cv::Scalar(0,255,0), -1);
    //cv::line(sgmt_img, cv::Point(left_point_x, sgmt_img.rows-1), left_min_point, cv::Scalar(0,255,0), 2);
    //cv::circle(sgmt_img, right_min_point, 3, cv::Scalar(255,0,0), -1);
    //cv::line(sgmt_img, cv::Point(right_point_x + sgmt_img.cols/2 - 1, sgmt_img.rows-1), right_min_point, cv::Scalar(255,0,0), 2);

    cv::imshow("gray Image", original);
    cv::imshow("Binary Image", sgmt_img);
    cv::waitKey(1);

    //auto end = std::chrono::high_resolution_clock::now(); //count stop
    //std::chrono::duration<double> inferenceTime = end - start;
    //std::cout << "에러값 구하는 시간: " << inferenceTime.count() << " seconds." << std::endl;
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
