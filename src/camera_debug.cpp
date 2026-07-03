#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm> 
#include <vector> // std::vector와 std::min_element 사용을 위해 포함

bool traffic_light_30cm = false; //일정거리에 물체가 있는지 판단하는 플레그
bool red = false;  //빨간색 신호 감지 플레그              
bool green = false; //초록색 신호 감지 플레그              

void distance_callback(const sensor_msgs::Image::ConstPtr& msg) { //거리 판단 콜백함수
    cv_bridge::CvImageConstPtr cv_ptr; //ros 이미지 opencv형태로 참조하기 위한 포인터

    try {
        cv_ptr = cv_bridge::toCvShare(msg, msg->encoding); //encoding 타입을 그대로 사용해서 Ros 이미지를 open Cv Mat으로 공유(toCvShare로 복사 최소화)
    } catch (cv_bridge::Exception& e) { //변환 중 에러 생기면 함수 종료
        ROS_ERROR("%s", e.what()); 
        return;
    }

    cv::Mat depth_raw = cv_ptr->image; //opencv 형식의 원본 깊이 이미지를 가져옴
    cv::Mat depth_mm; //mm단위로 통일해서 사용할 mat 함수

    //인코딩 자동처리.
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) { //float형식 인코딩 단위는 미터.
        depth_mm = depth_raw * 1000.0f; //m->mm 변환
    } 
    else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) { //unsigned int 형식 단위는 mm
        depth_raw.convertTo(depth_mm, CV_32F); //계산 편의를 위해 float형식으로 변환
    }
    else {
        ROS_ERROR("Unsupported depth encoding: %s", msg->encoding.c_str()); //그 외 인코딩이면 종료.
        return;
    }

    int w = 100, h = 100; //ROI의 너비와 높이를 설정.
    int x = depth_mm.cols/2 - w/2; //이미지 중앙을 기준으로 ROI 최상단 x 좌표 계산.
    int y = depth_mm.rows/2 - h/2; //이미지 중앙을 기준으로 ROI 최상단 y 좌표 계산.

    cv::Rect roi(x, y, w, h); //Rect를 이용해 ROI영역 정의
    cv::Mat depth_roi = depth_mm(roi); //전체 depth 이미지에서 ROI 부분만 잘라낸 서브 Mat 생성.

    cv::Mat mask_valid = (depth_roi > 50) & (depth_roi < 1000); // 5~100cm만 활성화 하는 마스크
    cv::Mat nan_mask = (depth_roi != depth_roi); //유효하지 않은 픽셀을 찾기 위한 마스크
    mask_valid.setTo(0, nan_mask); //유효하지 않은 픽셀은 mask_vaild에서 0으로 설정.

    int valid = cv::countNonZero(mask_valid); //유효한 픽셀의 개수를 샘.

    if (valid < 10) { //너무 픽셀이 적으면 예외 처리 (영상처리 감도 설정.).
        traffic_light_30cm = false;
        ROS_INFO("No valid depth pixels.");
        return;
    }

    double min_mm; //최소값 저장할 변수
    cv::minMaxLoc(depth_roi, &min_mm, 0, 0, 0, mask_valid); //마스크로 필터링 된 ROI에서 최소/최댓값을 찾는 함수.

    double distance_cm = min_mm / 10.0; //mm단위를 cm단위로 변경.

    ROS_INFO("Distance: %.2f cm", distance_cm); //변환된 거리 로그에 출력.

    traffic_light_30cm = (distance_cm > 65 && distance_cm < 80); //거리가 65cm~80cm면 장애물 감지 플레그 on
}


void color_callback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr; //ros 이미지 opencv형태로 참조하기 위한 포인터
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //ROS 이미지 메시지를 opencv BGR8 이미지로 변환
    }
    catch (cv_bridge::Exception& e){ //변환 실패시 함수 종료
        return; 
    }

    cv::Mat image_hsv; //BGR -> HSV 색공간으로 변경.
    cv::cvtColor(cv_ptr->image, image_hsv, cv::COLOR_BGR2HSV); //BGR -> HSV 색공간으로 변경.
    const int PIXEL_THRESHOLD = 1000; //색 검출 시 픽셀 개수 기준

    cv::Mat red_mask_low, red_mask_high, red_mask; //RED 마스크는 2 구간으로 나누어서 검충
    cv::inRange(image_hsv, cv::Scalar(0,150,70), cv::Scalar(10,255,255), red_mask_low); //빨간색 (0~10)
    cv::inRange(image_hsv, cv::Scalar(170,150,70),cv::Scalar(180,255,255), red_mask_high); //빨간색 (170~180)
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); //두 red 마스크를 하나로 합치기.
    int red_count = cv::countNonZero(red_mask); //red 픽셀 개수 계산.

    cv::Mat green_mask;
    cv::inRange(image_hsv,cv::Scalar(35,40,40), cv::Scalar(85,255,255),green_mask); //  녹색은 35~85로
    int green_count = cv::countNonZero(green_mask); //픽셀 개수 계산

    red = false; //빨간색 플레그 초기화
    green = false; //초록색 플레그 초기화

    if(traffic_light_30cm == false){ //장애물 전 색상 감지는 x
    return;
    }
    const int max_count = std::max(red_count,green_count); //두 색중 더 큰 픽셀 개수 선택
    
    ROS_INFO_STREAM("Color Counts (30cm ON): RED=" << red_count << " | GREEN=" << green_count << " | Max=" << max_count); //빨간색 카운트 초록색 카운트 두 색중 가장 큰 픽셀 개수 터미널에 출력.

    if(max_count > PIXEL_THRESHOLD){ //특정 픽셀 개수 이상이면 색 판정
        if(max_count == red_count){ //만약 red가 max카운트면 red 플레그 on 아니면 green 플레그 on
            red = true;
        }
        else{
            green = true;
        }
    }
}

int main(int argc, char **argv){ 
    ros::init(argc, argv, "camera_debugger");
    ros::NodeHandle n; 
    
    ros::Subscriber sub_depth = n.subscribe("/camera/depth/image_raw", 10, distance_callback);
    ros::Subscriber sub_color = n.subscribe("/camera/rgb/image_raw", 10, color_callback);
    
    ros::Rate looprate(10);

    while(ros::ok()){
        ROS_INFO_STREAM("CURRENT STATUS: 30cm=" << (traffic_light_30cm ? "TRUE" : "FALSE") //현재 상태 출력.
                        << " | RED=" << (red ? "TRUE" : "FALSE") 
                        << " | GREEN=" << (green ? "TRUE" : "FALSE"));
        
        ros::spinOnce(); 
        looprate.sleep();
    }
    return 0;
}