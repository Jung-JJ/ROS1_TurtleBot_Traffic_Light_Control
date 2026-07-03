#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> //오도메트리 메시지 사용
#include <tf/transform_datatypes.h> //쿼터니언(속도를 4축)->일반 벡터(3축)으로 tf하기 위한 tf함수 부르기.
#include <cmath> //절댓값,파이값을 쓰기 위해.
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/Image.h> //카메라에서 들어오는 이미지 타입을 사용하기 위해.
#include <cv_bridge/cv_bridge.h> //Ros 이미지 msg를 OpenCv 이미지 변환을 하기 위해.
#include <sensor_msgs/image_encodings.h> //이미지 인코딩을 하기 위해서.
#include <opencv2/opencv.hpp> //OpenCv 사용하기 위해.
#include <opencv2/imgproc/imgproc.hpp> //이미지 처리 관련 함수를 쓰기 위해.
#include <algorithm> //max 알고리즘을 쓰기 위해.

double turtle_theta = 0.0; //터틀봇 초기 세타값 0.0
double theta_degree =0.0; //각도 초기값 0
double target_yaw = 0.0; //직진을 위한 타겟 yaw값은 0
bool bumper_state = false; //범퍼 상태 플레그 초기화
bool traffic_light_30cm = false; //장애물 감지 플레그 초기화
bool red = false; //빨간색 감지 플레그 초기화
bool green = false; //초록색 감지 플레그 초기화
bool first_traffic_light = false; //첫 번째 신호등 미션 완료 플레그 초기화
bool second_traffic_light = false; //두 번째 신호등 미션 완료 플레그 초기화
bool first_odom_impormation = false; //처음 오도메트리 정보를 받았는지 확인하는 flag. 초기값은 false.


double setrad(double a){ //각도 차이를 계산하기 위해 로봇의 각을 -파이~+파이로 고정. ex -180과 +180도는 x,y축으로 보면 같다고 볼 수 있는데 diff로 계산시 360도 차이 나있다고 나옴. 이를 보정하기 위한 함수.
    while (a > M_PI) a -= 2*M_PI; //받은 값이 pi를 넘어간다면 a-2pi로 값 보정.
    while (a < -M_PI) a += 2*M_PI; //받은 값이 -pi를 넘어간다면 a+2pi로 값 보정.
    return a;  
}

void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& msg){ //범퍼 상태에 따른 플레그 설정 callback함수
    if(msg->state == kobuki_msgs::BumperEvent::PRESSED){ //만약 범퍼가 눌렸으면 플레그 true 아니면 false
        bumper_state = true;
    }
    else{
        bumper_state = false;
    }
}


void rotate_angle(ros::Publisher& pub, double deg, ros::Rate& looprate)
{
    double target = setrad(turtle_theta + deg * M_PI/180.0); //회전할 각도를 라디안 변환.
    double w = 1; //각속도의 절댓값.
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0; //회전 중에는 선속도는 0이다.

    while(ros::ok()){
        ros::spinOnce();
        double diff = setrad(target - turtle_theta); //목표값과 현재 터틀봇의 각도를 뺌.
        if(fabs(diff) < 0.05) break; //그 값의 절댓값이 0.05보다 작으면 while문은 끝.
        cmd.angular.z = (diff > 0 ? w : -w); //차이에 따라서 각속도 해야해서 차이가 0보다 크면 w 아니면 -w
        pub.publish(cmd);
        looprate.sleep();
    }
    // 정지
    cmd.angular.z = 0.0;
    pub.publish(cmd);
}


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

    if(second_traffic_light == false){ //2번째 신호등 처리를 한 뒤에는 거리 감지 off
    traffic_light_30cm = (distance_cm > 65 && distance_cm < 80); //거리가 65cm~80cm면 장애물 감지 플레그 on
    }
    else{
        traffic_light_30cm = false; //아니면 false
    }
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


void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){ //오도메트리 값이 들어오면 실행되는 callback함수
    turtle_theta = tf::getYaw(msg->pose.pose.orientation); //오도메트리는 쿼터니언 값. 이걸 우리가 원하는 yaw값으로 tf해서 터틀봇 세타 변수에 저장. 
    first_odom_impormation = true; //이제 오도메트리 정보를 받았다는걸 알려줌.
}

int main(int agrc,char **argv){ 
    ros::init(agrc,argv,"s_curve_turtlebot");
    ros::NodeHandle n; //노드 핸들은 n
    ros::Subscriber sub_odom = n.subscribe("/odom",10,odom_callback); //오도메트리 값 받아오는 섭스크라이버 크기는 10 값을 받으면 oddom_callback함수 호출.
    ros::Subscriber sub_bumper = n.subscribe("/mobile_base/events/bumper",10,bumper_callback); //범퍼 이벤트 즉 범퍼 관련 노드에서 토픽이 오면 bumper_callback함수 호출.
    ros::Subscriber sub_depth = n.subscribe("/camera/depth/image_raw",10,distance_callback); //이미지 raw값 받아오면 distance_callback함수 호출.
    ros::Subscriber sub_color = n.subscribe("/camera/rgb/image_raw",10,color_callback); //이미지 rgb값 받아오면 color_callback함수 호출.
    ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",10); //각도를 geometry_msgs::Twist 타입으로 /cmd_vel_mux/input/navi에 메시지 pub. 크기는 10

    ros::Rate looprate(50); //50hz지연
    geometry_msgs::Twist msg; //msg의 타입은 geometry_msgs::Twist.

    while(ros::ok() && !first_odom_impormation){ //ros가 켜져있고 first_odom_impormation을 못 받았으면 무한 루프.(오도메이션 값을 받아야지 코드를 진행시키기 위해서)
        ros::spinOnce(); //Callback 함수 한 번 실행(큐에 쌓인 msg 처리)
        looprate.sleep(); //50hz지연.   
    }

    target_yaw = turtle_theta;
    const double v = 0.2; //선속도 값은 0.15m/s
    const double P_yaw = 1.5;//P값 튜닝 결국 1.5로 결정.
    while(ros::ok()){
        ros::spinOnce();
        ROS_INFO("flags: traffic30=%d, red=%d, green=%d, bumper=%d",traffic_light_30cm, red, green, bumper_state); //실시간 디버깅을 위한 로그 출력.(거리플레그, 빨간색 인식 값, 초록색 인식 값, 범퍼 상태)
        if(traffic_light_30cm == 1){ //만약 일정거리 앞에 장애물이 있으면 정지.
            msg.linear.x = 0.0;
            msg.angular.z= 0.0;
        }
        else{ //아니면 직진(피드백 컨트롤로 정확히 직진하도록 설계)
            msg.linear.x= v ;
            double yaw_e =setrad(target_yaw- turtle_theta);
            msg.angular.z= P_yaw*yaw_e;   
        }

        pub_twist.publish(msg); //위에 조건문에 따른 상태 Pub

        if(first_traffic_light == false && traffic_light_30cm == 1){ //첫번 째 신호등 처리를 하지 않았고, 앞에 장애물이 있으면 실행
            if(!(red == true || green == true)){ 
                looprate.sleep();
                continue; // 다음 루프에서 다시 확인
            }
            if(red == true){ //빨간색이 감지되면 왼쪽으로 90도.
                rotate_angle(pub_twist, +90, looprate);
            }   
            else if(green == true){ //초록색이 감지되면 오른쪽으로 90도
                rotate_angle(pub_twist, -90, looprate);
            }
            target_yaw = turtle_theta; //타겟 yaw값을 현재 터틀봇에 맞춤(회전했으니까 직진을 위해 폭표 업데이트)
            first_traffic_light = true; //첫 번째 신호등 처리 완료했다는 플레그 on
            traffic_light_30cm = false; //첫 번째 신호등 회전 처리 했으니 거리 플레그 off(안전장치)
            red = false; //red플레그도 초기화
            green = false; //green플레그 초기화
        }

        if(first_traffic_light == true && second_traffic_light == false && traffic_light_30cm == 1){ //첫 번째 신호등 처리 완료했고 두 번째 신호등 처리 안 했고 앞에 장애물이 있으면 실행
            if(!(red == true || green == true)){
                looprate.sleep();
                continue; // 다음 루프에서 다시 확인
            }
            if(red == true){ //빨간색이 감지되면 왼쪽으로 90도.
                rotate_angle(pub_twist, +90, looprate);
            }   
            else if (green== true){ //초록색이 감지되면 오른쪽으로 90도
                rotate_angle(pub_twist, -90, looprate);
            }
            target_yaw = turtle_theta; //타겟 yaw값을 현재 터틀봇에 맞춤(회전했으니까 직진을 위해 폭표 업데이트)
            second_traffic_light = true; //두 번째 신호등 처리 완료 플레그 
            traffic_light_30cm = false; //첫 번째 신호등 회전 처리 했으니 거리 플레그 off(안전장치)
            red = false; //red플레그도 초기화
            green = false; //green플레그 초기화
        }

        if(first_traffic_light == true && second_traffic_light == true && bumper_state == true){ //1,2 신호등 처리 완료했고 범퍼 상태가 부딪치면 (정지명령 실행.)
            msg.linear.x = 0.0; 
            msg.angular.z = 0.0;
            pub_twist.publish(msg);
            break;
        }
        looprate.sleep();
    }
    return 0;
}
