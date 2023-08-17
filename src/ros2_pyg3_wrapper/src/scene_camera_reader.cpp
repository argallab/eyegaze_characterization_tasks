#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
// installing opencv https://docs.opencv.org/3.4/d7/d9f/tutorial_linux_install.html

#include "rclcpp/rclcpp.hpp"
#include "ros2_pyg3_common/msg/screen_corners_stamped.hpp"

using namespace std;

class SceneCameraReader : public rclcpp::Node
{
	public:
		SceneCameraReader(): Node("scene_camera_reader")
		{
			rclcpp::on_shutdown(std::bind(&SceneCameraReader::shutdown_hook, this));
			this->startup();
			
			publisher_ = this->create_publisher<ros2_pyg3_common::msg::ScreenCornersStamped>("/screen_corners", 10);
		
			corner_timer_ = this->create_wall_timer(2ms, 
													std::bind(&SceneCameraReader::corner_timer_callback, this), 
													corner_cb_grp_);

			video_timer_ = this->create_wall_timer(1ms, 
												   std::bind(&SceneCameraReader::video_timer_callback, this), 
												   video_cb_grp_);
		}
	
	private:
		void startup()
		{
			stream_name_ = "rtsp://192.168.8.110:8554/live/all";
			this->capture.open(stream_name_);

			/// DEBUGGING RTSP STREAM
			// if(!this->capture.isOpened())
			// {
			// 	cout << "Error opening video stream or file" << endl;
			// 	return;
			// }

			// while (1)
			// {
			// 	cv::Mat frame;
			// 	this->capture >> frame;
			// 	if (frame.empty())
			// 		break;

			// 	cv::imshow("Frame", frame);

			// 	char c=(char)cv::waitKey(25);
			// 	if(c==27)
			// 		break;
			// }

			this->corner_cb_grp_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			this->video_cb_grp_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		}

		void shutdown_hook()
		{
			RCLCPP_INFO(this->get_logger(), "Shutting down, closing glasses RTSP stream...");
			this->capture.release();
		}

		void video_timer_callback()
		{
			// clearing the video buffer as fast as possible
			this->capture.grab();
		}
		
		void corner_timer_callback()
		{
			cv::Mat frame;
			this->capture.retrieve(frame);

			if(!frame.empty())
			{
				std::vector<int> markerIds;
				std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
				cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
				cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
				cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

				// RCLCPP_INFO(this->get_logger(), "%ld markers detected", markerIds.size());
				if (markerIds.size() == 4)
				RCLCPP_INFO(this->get_logger(), "4 markers detected");
				{
					corner_pub.updatedonce = true;
				

					for (unsigned int i=0; i<markerIds.size(); i++)
					{
						int mId = markerIds[i];
						switch(mId)
						{
							case 0:
								corner_pub.topleft.x = markerCorners[i][0].x;
								corner_pub.topleft.y = markerCorners[i][0].y;
								break;
							case 1:
								corner_pub.topright.x = markerCorners[i][1].x;
								corner_pub.topright.y = markerCorners[i][1].y;
								break;
							case 2:
								corner_pub.bottomleft.x = markerCorners[i][3].x;
								corner_pub.bottomleft.y = markerCorners[i][3].y;
								break;
							case 3:
								corner_pub.bottomright.x = markerCorners[i][2].x;
								corner_pub.bottomright.y = markerCorners[i][2].y;
								break;
						}
					}
					
					corner_pub.header.stamp = this->get_clock()->now();
					publisher_->publish(corner_pub);

				}
			}
			else
			{
				;
				// RCLCPP_INFO(this->get_logger(), "Got empty camera frame...");
			}
			
		}

		rclcpp::CallbackGroup::SharedPtr corner_cb_grp_;
		rclcpp::CallbackGroup::SharedPtr video_cb_grp_;
		rclcpp::TimerBase::SharedPtr corner_timer_;
		rclcpp::TimerBase::SharedPtr video_timer_;
		rclcpp::Publisher<ros2_pyg3_common::msg::ScreenCornersStamped>::SharedPtr publisher_;
		string stream_name_;
		cv::VideoCapture capture;
		// cv::VideoCapture* captureptr;
		ros2_pyg3_common::msg::ScreenCornersStamped corner_pub;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SceneCameraReader>();
	
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}

