#include <opencv2/opencv.hpp>
#include <string>
#include <math.h>
#include "Logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <iostream>
#include <vector>
#include <time.h>

struct DetectResult
{
	int classId;
	float score;
	cv::Rect box;
};

class YOLOv5Detector
{
public:
	void load_model(std::string onnxpath, int iw, int ih, float threshold);
	void detect(cv::Mat &frame, std::vector<DetectResult> &result);

private:
	int input_w = 640;
	int input_h = 640;
	cv::dnn::Net net;
	int threshold_score = 0.25;
};

void YOLOv5Detector::load_model(std::string onnxpath, int iw, int ih, float threshold)
{
	this->input_w = iw;
	this->input_h = ih;
	this->threshold_score = threshold;
	this->net = cv::dnn::readNetFromONNX(onnxpath);
}

void YOLOv5Detector::detect(cv::Mat &frame, std::vector<DetectResult> &results)
{
	// 图象预处理 - 格式化操作
	int w = frame.cols;
	int h = frame.rows;
	int _max = std::max(h, w);
	cv::Mat image = cv::Mat::zeros(cv::Size(_max, _max), CV_8UC3);
	cv::Rect roi(0, 0, w, h);
	frame.copyTo(image(roi));

	float x_factor = image.cols / 640.0f;
	float y_factor = image.rows / 640.0f;

	// 推理
	cv::Mat blob = cv::dnn::blobFromImage(image, 1 / 255.0, cv::Size(this->input_w, this->input_h), cv::Scalar(0, 0, 0), true, false);
	this->net.setInput(blob);
	cv::Mat preds = this->net.forward();

	// 后处理, 1x25200x85
	// std::cout << "rows: " << preds.size[1] << " data: " << preds.size[2] << std::endl;
	cv::Mat det_output(preds.size[1], preds.size[2], CV_32F, preds.ptr<float>());
	std::vector<cv::Rect> boxes;
	std::vector<int> classIds;
	std::vector<float> confidences;
	std::vector<cv::Mat> car_images;
	for (int i = 0; i < det_output.rows; i++)
	{
		float confidence = det_output.at<float>(i, 4);
		if (confidence < 0.45)
		{
			continue;
		}
		// cv::Mat classes_scores = det_output.row(i).colRange(5, 85);

		cv::Point classIdPoint = cv::Point(0, 0);
		double score = confidence;
		// minMaxLoc(classes_scores, 0, &score, 0, &classIdPoint);

		if (score > this->threshold_score)
		{
			float cx = det_output.at<float>(i, 0);
			float cy = det_output.at<float>(i, 1);
			float ow = det_output.at<float>(i, 2);
			float oh = det_output.at<float>(i, 3);
			int x = static_cast<int>((cx - 0.5 * ow) * x_factor);
			int y = static_cast<int>((cy - 0.5 * oh) * y_factor);
			int width = static_cast<int>(ow * x_factor);
			int height = static_cast<int>(oh * y_factor);
			cv::Rect box;
			box.x = x;
			box.y = y;
			box.width = width;
			box.height = height;

			if (x > 0 && y > 0 && width > 0 && height > 0 && x + width <= frame.cols && y + height <= frame.rows)
			{
				cv::Mat car_image = frame(cv::Rect(x, y, width, height));
				// cv::imshow("car_images", car_image);
				// cv::waitKey(1);
				car_images.push_back(car_image);
			}

			boxes.push_back(box);
			classIds.push_back(classIdPoint.x);
			confidences.push_back(score);
		}
	}

	// NMS
	std::vector<int> indexes;
	cv::dnn::NMSBoxes(boxes, confidences, 0.25, 0.45, indexes);
	for (size_t i = 0; i < indexes.size(); i++)
	{
		DetectResult dr;
		int index = indexes[i];
		int idx = classIds[index];
		dr.box = boxes[index];
		dr.classId = idx;
		dr.score = confidences[index];
		cv::rectangle(frame, boxes[index], cv::Scalar(0, 0, 255), 2, 8);
		// cv::rectangle(frame, cv::Point(boxes[index].tl().x, boxes[index].tl().y - 20),
		// 			  cv::Point(boxes[index].br().x, boxes[index].tl().y), cv::Scalar(0, 0, 255), -1);
		results.push_back(dr);
	}

	std::ostringstream ss;
	std::vector<double> layersTimings;
	double freq = cv::getTickFrequency() / 1000.0;
	double time = net.getPerfProfile(layersTimings) / freq;
	ss << "FPS: " << float(int((1000 / time) * 10)) / 10 << " | time : " << time << " ms";
	putText(frame, ss.str(), cv::Point(20, 80), cv::FONT_HERSHEY_PLAIN, 5.0, cv::Scalar(255, 255, 0), 5, 8);
}

// [ROS2 数据收发类]>
class Points_publisher : public rclcpp::Node
{
public:
	Points_publisher(std::string name)
		: Node(name)
	{
		std::string topic = "detect_result";

		// [创建订阅]
		publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic, 10);
	}
	void publish(std_msgs::msg::Float32MultiArray message)
	{
		publisher_->publish(message);
	}

private:
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};
// [ROS2 数据收发类]<

int main(int argc, char **argv)
{
	Logger logger(Logger::file, Logger::debug, "./logs/yolo.log");

	rclcpp::init(argc, argv);
	//[创建对应节点的共享指针对象]
	auto publisher = std::make_shared<Points_publisher>("detect_result");
	//[运行节点，并检测退出信号]

	std_msgs::msg::Float32MultiArray message;
	std::string classNames[1] = {"Car"};

	std::shared_ptr<YOLOv5Detector> detector(new YOLOv5Detector());

	auto capture = cv::VideoCapture();

	detector->load_model("./resources/car_identfy.onnx", 640, 640, 0.25f);
	capture.open("./resources/2.mp4");

	if (!capture.isOpened())
	{
		logger.ERRORS("视频或摄像头打开失败");
	}

	cv::Mat frame;
	std::vector<DetectResult> results;
	std::vector<float> detect_result;

	cv::namedWindow("detect_window", 0);
	cv::resizeWindow("detect_window", cv::Size(960, 540));
	// cv::namedWindow("car_images", 0);
	// cv::resizeWindow("car_images", cv::Size(100, 100));
	while (true)
	{
		bool ret = capture.read(frame);
		if (!ret)
			break;
		detector->detect(frame, results);
		detect_result.clear();

		for (size_t i = 0; i < results.size(); i++)
		{
			detect_result.push_back(results[i].classId);
			detect_result.push_back(results[i].box.x);
			detect_result.push_back(results[i].box.y);
		}
		message.data = detect_result;
		publisher->publish(message);
		for (DetectResult dr : results)
		{
			std::ostringstream info;
			info << classNames[dr.classId] << " Conf:" << float(int(dr.score * 100)) / 100;
			cv::Rect box = dr.box;
			cv::putText(frame, info.str(), cv::Point(box.tl().x, box.tl().y - 10), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 3);
		}

		cv::imshow("detect_window", frame);
		char c = cv::waitKey(1);
		if (c == 27)
		{ // ESC 退出
			break;
		}
		// reset for next frame
		results.clear();
	}
	rclcpp::spin(publisher);
	rclcpp::shutdown();
}
