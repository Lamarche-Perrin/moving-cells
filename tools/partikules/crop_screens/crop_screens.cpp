// g++ crop_screens.cpp -o crop-screens `pkg-config --cflags --libs opencv`

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

double borderWidthRatio = 2.596 / 332.644;
double borderHeightRatio = 3.124 / 188.776;

int main (int argc, char *argv[])
{
	if (argc != 3) { return EXIT_FAILURE; }

	std::string inputFile = argv[1];
	std::string outputFile = argv[2];

	cv::Mat input = cv::imread (inputFile);
	cv::Size size = input.size();

	int borderWidth = round (size.width * borderWidthRatio / 2) * 2;
	int screenWidth = (size.width - borderWidth) / 2;

	int borderHeight = round (size.height * borderHeightRatio / 2) * 2;
	int screenHeight = (size.height - borderHeight) / 2;

	// std::cout << " image = " << size.width << " x " << size.height << std::endl;
	// std::cout << "border = " << borderWidth << " x " << borderHeight << std::endl;
	// std::cout << "screen = " << screenWidth << " x " << screenHeight << std::endl;
	
	cv::Mat output = cv::Mat (screenHeight * 2, screenWidth * 2, CV_8UC3, cv::Scalar (0, 0, 0));

	input (cv::Rect (0, 0, screenWidth, screenHeight)).copyTo (output (cv::Rect (0, 0, screenWidth, screenHeight)));
	input (cv::Rect (screenWidth + borderWidth, 0, screenWidth, screenHeight)).copyTo (output (cv::Rect (screenWidth, 0, screenWidth, screenHeight)));
	input (cv::Rect (0, screenHeight + borderHeight, screenWidth, screenHeight)).copyTo (output (cv::Rect (0, screenHeight, screenWidth, screenHeight)));
	input (cv::Rect (screenWidth + borderWidth, screenHeight + borderHeight, screenWidth, screenHeight)).copyTo (output (cv::Rect (screenWidth, screenHeight, screenWidth, screenHeight)));

	cv::imwrite (outputFile, output);

	return EXIT_SUCCESS;
}

