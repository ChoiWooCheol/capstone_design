#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector> // vector
#include <math.h> // abs
#include "constant_a.h" // 상수가 정의되어있는 헤더파일
#include "autodrive/autodrive.h" // msg 가 정의되어있는 함수.
#include <queue>

using namespace std;
using namespace cv;

Mat region_of_interest(Mat canny_image); // ROI 
Mat hough_lines(Mat ROI_image, Mat srcImage_COLOR); 
Mat weight_image(Mat hough_image, Mat srcImage); 

void draw_lines(Mat ROI_image, vector<Vec4i> lines, Mat srcImage_COLOR); 
void Draw_fitLine_L(Mat line_image, Vec4f output); 
void Draw_fitLine_R(Mat line_image, Vec4f output);
void Draw_fitLine_S(Mat line_image, Vec4f output);

void print_Line_and_point(Mat line_image, vector<Point2f> Pointxy, int L); 
void calc_x_y_point(Mat line_image, float vx, float vy, float x, float y, int L); 
void print_center_point(Mat line_image); 
void cal_car_radian(Mat line_image); 
void cal_one_line_degree_L(Mat line_image);
void cal_one_line_degree_R(Mat line_image);

double L_center_x, L_center_y = 0;
double R_center_x, R_center_y = 0;
double center_x, center_y;

deque<pair<double, double>> center_x_y_deque; 
deque<double> move_degree_deque; 
double L_standard_degree = 0;
double R_standard_degree = 0;
double move_degree = 0;
int L_standard_degree_count = 0; 
int R_standard_degree_count = 0; 
double L_sum = 0;
int L_sum_count = 0;
double R_sum = 0;
int R_sum_count = 0;
double radian = 0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "autodrive_pub"); //마스터 노드에 알릴 노드명 작성.
	ros::NodeHandle nh; // 통신을 위해 필요한 핸들러 NodeHandler nh
	ros::Publisher pub = nh.advertise<autodrive::autodrive>("opencv_msg",1000); // 큐사이즈 1000
	autodrive::autodrive msg;
	ros::Rate loop_rate(10); // 10hz로 발행한다. 0.1초 단위
	msg.goto_radian = 0;
	msg.goto_center_x = 0;
	msg.goto_center_y = 0;

	//VideoCapture inputVideo("soongsil_drive.avi"); // 동영상 출력할때 사용.
	VideoCapture inputVideo(1); //웹캠으로 출력할때 사용.
	Size size = Size((int)inputVideo.get(CAP_PROP_FRAME_WIDTH),(int)inputVideo.get(CAP_PROP_FRAME_HEIGHT));
	int fourcc = VideoWriter::fourcc('X','V','I','D');
	double fps_output = 24;
	bool isColor = true;
	cv::VideoWriter outputVideo("ouput3.avi", fourcc,fps_output,size, isColor);
	cv::VideoWriter outputVideo2("output4.avi", fourcc, fps_output, size, isColor);
	cv::Mat frame;
	Mat srcImage_GRAY;
	Mat srcImage_COLOR;
	Mat srcImage;
	Mat hough_image;
	Mat canny_image;
	Mat ROI_image;
	Mat result;
	int n = 1;
	int m = 1;

	int fps = (int)(inputVideo.get(CAP_PROP_FPS));
	int delay = Delay / fps;

	while(ros::ok()) // 만약 ctrl+c를 누르면 종료가 됨.
	{
		inputVideo >> frame; // frameÀ» ÀÐŸî¿ÂŽÙ.
		srcImage_COLOR = frame.clone(); // ¿øº»ÀÌ¹ÌÁö º¹»ç(ÄÃ·¯)
		cv::cvtColor(frame, srcImage_GRAY, COLOR_BGR2GRAY); //Èæ¹éÀž·Î º¯°æ
		Canny(srcImage_GRAY, canny_image, canny_threshold_1, canny_threshold_2, apertureSize); // edge °ËÃâ
		ROI_image = region_of_interest(canny_image); // ROI
		hough_image = hough_lines(ROI_image, srcImage_COLOR);
		result = weight_image(hough_image, srcImage_COLOR);
		outputVideo << result;
		outputVideo2 << frame;
		imshow("result", result);


		msg.goto_center_x = center_x; // 메세지 값 넣기
		msg.goto_center_y = center_y; // 메세지 값 넣기
		msg.goto_radian = radian;// 라디안 넣기

		//보내는 값 확인
		ROS_INFO("center x : %d", msg.goto_center_x);
		ROS_INFO("center y : %d", msg.goto_center_y);
		ROS_INFO("radian : %f", msg.goto_radian);

		pub.publish(msg);
		int ckey = waitKey(delay);
		if (ckey == ESC) break;
	}
	return 0;
}


Mat region_of_interest(Mat canny_image) // ¿øº»ÀÌ¹ÌÁöÀÇ ŽÙ°¢Çü ºÎºÐ ÃßÃâ
{
	int height = canny_image.size().height;
	int width = canny_image.size().width;
	Mat dst_image;
	Mat mask(canny_image.rows, canny_image.cols, CV_8UC1, Scalar(0));
	Point pts[6] = { Point(0,height),Point(0, height / 2), Point(width / 3, height / 4), Point(width / 3 * 2 , height / 4), Point(width, height / 2), Point(width,height) }; // ÁÖ°£
	//Point pts[4] = { Point(0,height), Point(width / 3 , height / 2), Point(width / 3 * 2, height / 2), Point(width , height) }; // Ÿß°£
	fillConvexPoly(mask, pts, 6, Scalar(255));
	bitwise_and(canny_image, mask, dst_image);
	return dst_image;
}

Mat hough_lines(Mat ROI_image, Mat srcImage_COLOR)
{
	Mat line_image(ROI_image.size(), CV_8UC3, Scalar(0, 0, 0));

	vector<Vec4i> lines;
	HoughLinesP(ROI_image, lines, rho, delta, hough_threshold, MinLineLength, MaxLineGap);
	draw_lines(line_image, lines, srcImage_COLOR);

	return line_image;
}

Mat weight_image(Mat hough_image, Mat srcImage)
{
	Mat outMat;
	addWeighted(srcImage, alpha, hough_image, beta, gamma, outMat);
	return outMat;
}

void print_Line_and_point(Mat line_image, vector<Point2f> Pointxy, int L)
{
	if (Pointxy.size() != 0)
	{
		Vec4f output;

		fitLine(Pointxy, output, DIST_L2, fit_param, fit_reps, fit_aeps); // vx, vy, x, y °ª output¿¡ ÀúÀå.
		if (L == Left) Draw_fitLine_L(line_image, output); //¿ÞÂÊ ŽëÇ¥Œ± ±×ž®±â
		if (L == Right) Draw_fitLine_R(line_image, output); //¿Àž¥ÂÊ ŽëÇ¥Œ± ±×ž®±â
		if (L == Stop) Draw_fitLine_S(line_image, output); // Á€ÁöŒ± ŽëÇ¥Œ± ±×ž®±â

		Point_<float> pt(output[2], output[3]); // ŽëÇ¥Á¡
		cv::circle(line_image, pt, radius, Scalar(0, 255, 0), line_thickness);
		if (L == Right) // ¿Àž¥ÂÊ
		{
			R_center_x = output[2];
			R_center_y = output[3];
		}

		else if (L == Left) // ¿ÞÂÊ
		{
			L_center_x = output[2];
			L_center_y = output[3];
		}
	}
}

void Draw_fitLine_L(Mat line_image, Vec4f output)
{
	float vx = output[0];
	float vy = output[1];
	float x = output[2];
	float y = output[3];

	int x1 = (int)x;
	int y1 = (int)y;
	int x2 = (int)(x1 - 200 * vx);
	int y2 = (int)(y1 - 200 * vy);

	Point_<int> pt1(x1, y1), pt2(x2, y2);
	cv::line(line_image, pt1, pt2, Scalar(255, 255, 255), line_thickness); // ŽëÇ¥ Â÷Œ±
	move_degree = atan2(pt2.x - pt1.x, pt2.y - pt1.y) * 180 / CV_PI;
	calc_x_y_point(line_image, vx, vy, x, y, Left);
}

void Draw_fitLine_R(Mat line_image, Vec4f output)
{
	float vx = output[0];
	float vy = output[1];
	float x = output[2];
	float y = output[3];

	int x1 = (int)x;
	int y1 = (int)y;
	int x2 = (int)(x1 + 200 * vx);
	int y2 = (int)(y1 + 200 * vy);

	Point_<int> pt1(x1, y1), pt2(x2, y2);
	cv::line(line_image, pt1, pt2, Scalar(255, 255, 255), line_thickness); // ŽëÇ¥ Â÷Œ±
	move_degree = atan2(pt2.x - pt1.x, pt2.y - pt1.y) * 180 / CV_PI;
	calc_x_y_point(line_image, vx, vy, x, y, Right);
}

void Draw_fitLine_S(Mat line_image, Vec4f output)
{
	int width = line_image.size().width;
	int height = line_image.size().height;

	float vx = output[0];
	float vy = output[1];
	float x = output[2];
	float y = output[3];

	int x1 = (int)x;
	int y1 = (int)y;
	int x2 = (int)(x1 + 200 * vx);
	int y2 = (int)(y1 + 200 * vy);

	Point_<int> pt1(x1, y1), pt2(x2, y2);
	if (y1 > height / 4 * 3)
	{
		ROS_INFO("------------Stop-----------\n");
	}
	cv::line(line_image, pt1, pt2, Scalar(255, 255, 255), line_thickness); // ŽëÇ¥ Â÷Œ±
}

void draw_lines(Mat line_image, vector<Vec4i> lines, Mat srcImage_COLOR)
{
	Vec4i params;

	vector<Point2f> L_Pointxy;
	vector<Point2f> R_Pointxy;
	vector<Point2f> S_Pointxy;

	int x1, y1, x2, y2;
	double slope_degree;
	int height = line_image.size().height;
	int width = line_image.size().width;

	for (int k = 0; k < lines.size(); k++) // Á÷Œ±¿¡ ŽëÇÑ µÎÁ¡ÀÌ ÀûÇôÀÖÀœ.
	{
		params = lines[k];
		x1 = params[0];
		y1 = params[1];
		x2 = params[2];
		y2 = params[3];
		Point_<int> pt1(x1, y1), pt2(x2, y2);
		slope_degree = atan2(y2 - y1, x2 - x1) * 180 / CV_PI;

		if (abs(slope_degree) < Max_slope_degree && abs(slope_degree) > Min_slope_degree && ((pt1.x <width/2 && pt2.x <width /2)||(pt1.x >width /2 && pt2.x >width /2))) //ŒöÆò±â¿ï±â , ŒöÁ÷±â¿ï±â
		{
			// Á÷Œ± ±Ù»ç µû·Î µû·Î
			if (slope_degree > inclination_standard)
			{
				cv::line(line_image, pt1, pt2, Scalar(255, 0, 0), line_thickness);
				R_Pointxy.push_back(pt1);
				R_Pointxy.push_back(pt2);

			}
			else if (slope_degree < inclination_standard)
			{
				cv::line(line_image, pt1, pt2, Scalar(255, 0, 0), line_thickness);
				L_Pointxy.push_back(pt1);
				L_Pointxy.push_back(pt2);
			}
		}
		else if (abs(slope_degree) < stop_degree && (pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y) >= 30000)
		{
			cv::line(line_image, pt1, pt2, Scalar(0, 255, 0), line_thickness);
			S_Pointxy.push_back(pt1);
			S_Pointxy.push_back(pt2);
		}
	}
	if (R_Pointxy.size() != 0 && L_Pointxy.size() != 0)
	{
		print_Line_and_point(line_image, R_Pointxy, Right); //¿Àž¥ÂÊ Â÷Œ± ±×ž®±â
		print_Line_and_point(line_image, L_Pointxy, Left);// ¿ÞÂÊ Â÷Œ± ±×ž®±â
		// print_Line_and_point ÇÔŒöžŠ ÅëÇØŒ­ center_x¿Í center_yÀÇ °ªÀÌ ÀÔ·ÂµÊ.
		print_center_point(line_image); // sensor ÆÀ¿¡ ÀüŒÛÇÒ ÁÂÇ¥ // ÁßŸÓ ÁÂÇ¥
	}
	else if (R_Pointxy.size() != 0 && L_Pointxy.size() == 0) // ¿Àž¥ÂÊ Â÷Œ±žž ºžÀÏ¶§
	{
		print_Line_and_point(line_image, R_Pointxy, Right); //¿Àž¥ÂÊ Â÷Œ± ±×ž®±â
		cal_one_line_degree_R(line_image);
	}
	else if (R_Pointxy.size() == 0 && L_Pointxy.size() != 0) // ¿ÞÂÊ Â÷Œ±žž ºžÀÏ¶§
	{
		print_Line_and_point(line_image, L_Pointxy, Left);// ¿ÞÂÊ Â÷Œ± ±×ž®±â
		cal_one_line_degree_L(line_image);
	}

	if (S_Pointxy.size() != 0)
		print_Line_and_point(line_image, S_Pointxy, Stop); // Á€ÁöŒ± ±×ž®±â
}

void calc_x_y_point(Mat line_image, float vx, float vy, float x, float y, int L)
{
	float inclination = vy / vx; // º€ÅÍžŠ ÀÌ¿ëÇÑ ±â¿ï±â
	int width = line_image.size().width;
	int height = line_image.size().height;
	int y_point = (int)(y - inclination * x); // y ÀýÆí

	int x_down;
	int y_down;
	int x_up;
	int y_up;

	if (L == Left) // ¿ÞÂÊ ÀýÆí Ãâ·Â
	{
		//ŸÆ·¡ Á¡ Ã£±â
		if (y_point > height) // Â÷Œ±ÀÌ ¿µ»ó ŸÆ·§žé¿¡Œ­ ²÷±æ¶§
		{
			x_down = (int)((height - y_point) / inclination);// ŸÆ·§žé°ú Â÷Œ±ÀÌ žž³ªŽÂ ÁÂÇ¥
			y_down = height;
		}
		else// Â÷Œ±ÀÌ ¿ÞÂÊ žé¿¡Œ­ ²÷±æ ¶§ 
		{
			x_down = 0;
			y_down = y_point;// ¿ÞÂÊ žé°ú Â÷Œ±ÀÌ žž³ªŽÂ ÁÂÇ¥
		}
		// À­Á¡ Ã£±â
		if ((height / 2 - y_point) / inclination > width) // ¿Àž¥ÂÊ žé°ú Â÷Œ±ÀÌ žž³ªŽÂ ÁÂÇ¥
		{
			x_up = width;
			y_up = (int)(inclination * width + y_point);
		}
		else // À§ÂÊ žé°ú Â÷Œ±ÀÌ žž³ªŽÂ ÁÂÇ¥
		{
			x_up = (int)((height / 2 - y_point) / inclination);
			y_up = height / 2;
		}
		Point pt1(x_down, y_down), pt2(x_up, y_up);
		cv::circle(line_image, pt1, 3, Scalar(0, 0, 255), 3);
		cv::circle(line_image, pt2, 3, Scalar(0, 0, 255), 3);
	}
	else if (L == Right) // ¿Àž¥ÂÊ ÀýÆí ±×ž®±â
	{
		// ŸÆ·¡Á¡ Ã£±â
		if (inclination * width + y_point > height) // Â÷Œ±ÀÌ ¿µ»ó ŸÆ·§žé¿¡Œ­ ²÷±æ¶§
		{
			x_down = (int)((height - y_point) / inclination); // ŸÆ·§žé°ú Â÷Œ±ÀÌ žž³ªŽÂ ÁÂÇ¥
			y_down = height;
		}
		else // Â÷Œ±ÀÌ ¿Àž¥ÂÊ žé¿¡Œ­ ²÷±æ ¶§ 
		{
			x_down = width;
			y_down = (int)(inclination * width + y_point); // ¿Àž¥ÂÊ žé°ú Â÷Œ±ÀÌ žž³ªŽÂ ÁÂÇ¥
		}

		// À­Á¡ Ã£±â
		if ((height / 2 - y_point) / inclination < 0) // ¿ÞÂÊ žé°ú Â÷Œ±ÀÌ žž³ªŽÂ ÁÂÇ¥
		{
			x_up = 0;
			y_up = y_point;
		}
		else
		{
			x_up = (int)((height / 2 - y_point) / inclination);
			y_up = height / 2;
		}
		Point pt3(x_down, y_down), pt4(x_up, y_up);
		cv::circle(line_image, pt3, 3, Scalar(0, 0, 255), 3);
		cv::circle(line_image, pt4, 3, Scalar(0, 0, 255), 3);
	}
}

void print_center_point(Mat line_image)
{
	int height = line_image.size().height;
	int width = line_image.size().width;

	center_x = (L_center_x + R_center_x) / 2;
	center_y = (L_center_y + R_center_y) / 2;

	pair<double, double> p;

	if (center_x_y_deque.size() < specimen)
	{
		p = make_pair(center_x, center_y);
		center_x_y_deque.push_front(p);
	}
	if (center_x_y_deque.size() == specimen)
	{
		double sum_x = 0;
		double sum_y = 0;

		for (int i = 0; i < specimen; i++)
		{
			sum_x += center_x_y_deque.at(i).first;
			sum_y += center_x_y_deque.at(i).second;
		}

		//Ç¥º»ÀÇ Æò±Õ
		center_x = (sum_x / specimen);
		center_y = (sum_y / specimen);

		center_x_y_deque.pop_back(); // ¿øŒÒ ÇÏ³ª »èÁŠ

		Point pt((int)center_x, (int)center_y);
		//°¡¿îµ¥ Á÷Œ±
		Point pt2(width / 2, height / 2);
		Point pt3(width / 2, height);

		cv::circle(line_image, pt, 5, Scalar(100, 100, 100), 3);
		cv::line(line_image, pt3, pt2, Scalar(200, 200, 200), 3); // °¡¿îµ¥ Á÷Œ±.

		cal_car_radian(line_image); // sensor ÆÀ¿¡ ÀüŒÛÇÒ °¢µµ
	}
	if (L_standard_degree_count == 0 && abs(center_x - (double)width / 2) < 0.2)
	{
		L_sum += move_degree;
		L_sum_count++;
		if (L_sum_count == 10)
		{
			L_standard_degree = L_sum / 10;
			L_standard_degree_count = 1;
		}
	}
	if (R_standard_degree_count == 0 && abs(center_x - (double)width / 2) < 0.2)
	{
		R_sum += move_degree;
		R_sum_count++;
		if (R_sum_count == 10)
		{
			R_standard_degree = (-1) * R_sum / 10;
			R_standard_degree_count = 1;
		}
	}

}

void cal_car_radian(Mat line_image)
{
	int height = line_image.size().height;
	int width = line_image.size().width;

	if (center_x >= width / 2) radian = atan2(center_x - width / 2, height - center_y) * 180 / CV_PI;
	else radian = atan2(width / 2 - center_x, height - center_y) * 180 / CV_PI * (-1);
}

void cal_one_line_degree_L(Mat line_image) // ±â¿ï±â°¡ ÇÏ³ªÀÏ¶§ ±â¿ï±â °è»ê.
{
	double degree = 0;
	double sum = 0;

	if (L_standard_degree < move_degree)
		degree = move_degree - L_standard_degree;
	else
		degree = (move_degree - L_standard_degree) * (-1);

	if (move_degree_deque.size() < one_line_specimen)
		move_degree_deque.push_front(degree);
	if (move_degree_deque.size() == one_line_specimen)
	{
		for (int i = 0; i < one_line_specimen; i++)
			sum += move_degree_deque.at(i);
		move_degree_deque.pop_back(); // ÇÏ³ª »èÁŠ
		radian = sum / one_line_specimen;
	}

}

void cal_one_line_degree_R(Mat line_image) // ±â¿ï±â°¡ ÇÏ³ªÀÏ ¶§ ±â¿ï±â °è»ê.
{
	double degree = 0;
	double sum = 0;

	if (R_standard_degree > move_degree)
		degree = (R_standard_degree - move_degree) * (-1);
	else
		degree = R_standard_degree - move_degree;

	if (move_degree_deque.size() < one_line_specimen)
		move_degree_deque.push_front(degree);
	if (move_degree_deque.size() == one_line_specimen)
	{
		for (int i = 0; i < one_line_specimen; i++)
			sum += move_degree_deque.at(i);
		move_degree_deque.pop_back(); // ÇÏ³ª »èÁŠ
		radian = sum / one_line_specimen;
	}
}