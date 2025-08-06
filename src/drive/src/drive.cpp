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
            cv::Mat frame, mask, birdview;
            cam_ >> frame;
            if(frame.empty()){
                RCLCPP_WARN(this->get_logger(), "빈 프레임이 입력되었습니다.");
                return;
            }   
            
            cv::GaussianBlur(frame, frame, cv::Size(5,5),0); // 가우시안블러 (경계선을 선명하게 함, Size 조정 가능)
            
            cv::Scalar lower_white(220, 220, 220);
            cv::Scalar upper_white(255, 255,255);  
            cv::inRange(frame, lower_white, upper_white, mask);  // 흰색 선만 추출 (lower_white 값 조정 가능)

            std::vector<cv::Point2f> srcPts = {
                cv::Point2f(550, 460), // 좌상단
                cv::Point2f(730, 460), // 우상단
                cv::Point2f(1200, 700), // 우하단
                cv::Point2f(100, 700) // 좌하단
            };

            std::vector<cv::Point2f> dstPts = {
                cv::Point2f(0, 0),
                cv::Point2f(400, 0),
                cv::Point2f(400, 600),
                cv::Point2f(0, 600)
            };

            cv::Mat matrix_Trans = cv::getPerspectiveTransform(srcPts, dstPts);

            cv::warpPerspective(mask, birdview, matrix_Trans, mask.size()); // 버드아이뷰 변환 (srcPts/dstPts 조정 혹은 카메라 조정 혹은 해상도 조정)

            cv::imshow("상태", birdview);

            if(cv::waitKey(1)==27){
                rclcpp::shutdown();
            }
        }

        cv::Mat applyRoi(const cv::Mat& input, std::vector<cv::Point> polygon){
            int height = input.rows;
            int width = input.cols;
            
            cv::Mat mask = cv::Mat::zeros(input.size(), input.type());
            
            std::vector<std::vector<cv::Point>> polygons{polygon};
            cv::fillPoly(mask, polygons, cv::Scalar(255));

            cv::Mat masked;
            cv::bitwise_and(input, mask, masked);

            return masked;
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