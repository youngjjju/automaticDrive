#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

class DriveNode : public rclcpp::Node
{
    public: 
        DriveNode(): Node("drive_node")
        {
            cam_.open(0);
            if(!cam.isOpened()){
                RCLCPP_ERROR(this->get_logger(), "카메라를 열 수 없습니다");
                return;
            }

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(33),
                std::bind(&DriveNode::process_frame, this)
            );
        }

    private:
        void process_frame()
        {
            cv::Mat frame, mask, edges;
            cam_ >> frame;
            if(frame.empty()){
                RCLCPP_WARN(this->get_logger(), "빈 프레임이 입력되었습니다.");
                return;
            }   
            
            cv::GaussianBlur(frame, frame, cv::Size(5,5),0); // 가우시안블러 (경계선을 선명하게 함, Size 조정 가능)
            
            cv::Scalar lower_white(220, 220, 220);
            cv::Scalar upper_white(255, 255,255);  
            cv::inRange(frame, lower_white, upper_white, mask);  // 흰색 선만 추출 (lower_white 값 조정 가능)

            cv::Canny(mask, edges, 100, 200); // Canny 엣지 검출 (임계값은 튜닝 가능)

            cv::imshow("카메라 상태", edges);

            if(cv::waitKey(1)==27){
                rclcpp::shutdown();
            }
        }

        cv::VideoCapture cam_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveNode>());
    rclcpp::shutdown();
    return 0;
}