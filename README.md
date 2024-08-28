## "deeplabv3(1).onnx"을 사용했을 때 바닥영역의 잡음 제거를 하기 위한 처리

### 외곽선 검출 함수(findContours)

![image](https://github.com/user-attachments/assets/4142e60c-faae-44dd-8d5c-c647150a91f2)
![image](https://github.com/user-attachments/assets/9296ce57-9348-4df0-8901-dee47222ad3f)

[출처](https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0)


### Parameter (RetrievalModes)
![image](https://github.com/user-attachments/assets/a8926abb-8e3f-4082-ba94-6856094da1aa)

[출처](https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html#ga819779b9857cc2f8601e6526a3a5bc71)

RETR_EXTERNAL : 객체 바깥쪽 외곽선만 검색. 계층 구조는 만들지 않음.

RETR_LIST : 객체 바깥쪽과 안쪽 외곽선을 모두 검색. 계층 구조는 만들지 않음.

RETR_CCOMP : 모든 외곽선을 검색하고 2단계 계층 구조를 구성.

RETR_TREE : 모든 외곽선을 검색하고 전체 계층 구조를 구성.

![image](https://github.com/user-attachments/assets/96a63d4b-0e51-4fc3-90e7-6ea7066a7544)

[출처](https://shpark98.github.io/devcourse-til/230427-3.-%EB%A0%88%EC%9D%B4%EB%B8%94%EB%A7%81%EA%B3%BC-%EC%99%B8%EA%B3%BD%EC%84%A0-%EA%B2%80%EC%B6%9C-%EB%B0%8F-OpenCV-%EC%99%B8%EA%B3%BD%EC%84%A0-%ED%95%A8%EC%88%98%EC%99%80-%EC%9D%91%EC%9A%A9/)


### Parameter (ContourApproximationModes)
![image](https://github.com/user-attachments/assets/d2f6459a-ec0c-4046-be2c-19fdd5ed6dd0)

[출처](https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html#ga4303f45752694956374734a03c54d5ff)


CHAIN_APPROX_NONE : 모든 외곽선 점들의 좌표를 저장.

CHAIN_APPROX_SIMPLE : 외곽선 중에서 수평선, 수직선, 대각선 성분은 끝점만 저장. (외곽선 점의 개수 줄이기)

CHAIN_APPROX_TC89_L1 & CHAIN_APPROX_TC89_KCOS : 점의 개수는 줄어들지만 외곽선 모양에 변화가 생김. (주의 필요)





## "deeplabv3(2).onnx"을 사용했을 때 알고리즘

![image](https://github.com/user-attachments/assets/b151d36b-e872-45b5-bed2-653805d59342)

![image](https://github.com/user-attachments/assets/fc45c4e7-debc-44de-a6fa-afbc0305e866)

![image](https://github.com/user-attachments/assets/13fca863-0885-4989-af50-fea98b72dbe3)


    // Step 3-2: 외곽선 검출 -> 갈 수 없는 바닥 객체 지우기
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point>> contours;
    //RETR_EXTERNAL, RETR_LIST || CHAIN_APPROX_NONE, CHAIN_APPROX_SIMPLE
    cv::findContours(sgmt_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::cvtColor(sgmt_img, sgmt_img, cv::COLOR_GRAY2BGR);
    for(int i = 0; i < (int)contours.size(); i++) {
        if (cv::contourArea(contours[i]) < 50*50) // 면적 기준을 적절히 설정 (contours[i]: 외곽선 개수)
            drawContours(sgmt_img, contours, i, cv::Scalar(0,255,0), -1);
    }

![image](https://github.com/user-attachments/assets/6de4527a-07d1-4b97-8cab-2c8d46835330)

![image](https://github.com/user-attachments/assets/4386984f-98cb-48d2-b3ca-32acae2137e9)


    // Step 2: 이동할 바닥 객체(면적이 가장 큰 객체) 검출
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
            present_point = cv::Point2d(p[0], p[1]);
        }
        
        // Step 3-1: 레이블링 -> 갈 수 없는 바닥 객체 지우기
        if(q[4] < 50*50)
            rectangle(sgmt_img, cv::Rect(q[0],q[1],q[2],q[3]), cv::Scalar(0,255,0), -1);
    }
