#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

class DriveNode : public rclcpp::Node
{
    public: 
        DriveNode(): Node("drive_node")
        {
            cv::VideoCapture cam(0);
            if(!cam.isOpened()){
                RCLCPP_ERROR(this->get_logger(), "카메라를 열 수 없습니다");
            }

            cv::Mat frame;
            while(rclcpp::ok()){
                cam >> frame;
                cv::imshow("카메라 상태", frame);

                // 여기

                if(cv::waitKey(10) ==27) break;
            }
            cam.release();
            cv::destroyAllWindows();
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveNode>());
    rclcpp::shutdown();
    return 0;
}