#include "QuadTree.h"
#include "opencv2/opencv.hpp"
#include <unistd.h>

QuadTree::QuadTree(cv::Mat src, int area, float threshold)
	:img(src), minArea(area), thr(threshold)
{
	dst = cv::Mat::zeros(img.size(), CV_8U);

	sImage = img.data;
	dImage = dst.data;

	root.x = 0;
	root.y = 0;
	root.width = img.cols;
	root.height = img.rows;

	createQuadTree(root);

	//cv::imshow("Test from QuadTree", img);
	//cv::waitKey(0);
}

QuadTree::~QuadTree() {
	//delete root.Children;
}

/* Calculates the average color of a rectangular region of a grid */
float QuadTree::average(int x, int y, int width, int height) {	
	int Sum = 0;
	
	// Adds the color values for each channel.
	for (int r = y; r < y + height; r++) {
		for (int c = x; c < x + width; c++) {
			int offset = r * img.cols + c;
			
			uchar intensity = sImage[offset]; 
            
			Sum += intensity;
		}
	}
    
    
	// number of pixels evaluated
	int area = width * height;

	return (float)Sum / (float)area;
}

/* Measures the amount of detail of a rectangular region of a grid */
float QuadTree::measureDetail(int x, int y, int width, int height) {
	float averageIntensity = average(x, y, width, height);
	
	int intensitySum = 0;

	// Calculate the distance between every pixel in the region
	// and the average color. The Manhattan distance is used, and
	// all the distances are added.
	for (int r = y; r < y + height; r++) {
		for (int c = x; c < x + width; c++) {
			int offset = r * img.cols + c;

			uchar intensity = sImage[offset];

			intensitySum += abs(averageIntensity - intensity);
		}
	}
	// Calculates the average distance, and returns the result.
	// We divide by three, because we are averaging over 3 channels.
	int area = width * height;
	return (float)intensitySum / (float)area;
}

void QuadTree::createQuadTree(Node node) {
    
	if (measureDetail(node.x, node.y, node.width, node.height) < thr || node.width * node.height <= minArea) {
		//float avgIntensity = average(node.x, node.y, node.width, node.height);
        //insert a keypoint
        
        bool flag = false;
        for (int r = node.y; r < node.y + node.height; r++) {
            for (int c = node.x; c < node.x + node.width; c++) {
                if(img.at<uchar>(r, c) == 255){
                    keypoints.push_back(cv::KeyPoint(c, r, 1)); 
                    flag = true; 
                    break;
                }
            }
            if (flag) break;
        }
        
		//coloring(avgBGR, node.x, node.y, node.width, node.height);
	}
	else {
		//TestShowImage(node.x, node.y, node.width, node.height);
		divideNode(node);
		createQuadTree(node.Children[0]);
		createQuadTree(node.Children[1]);
		createQuadTree(node.Children[2]);
		createQuadTree(node.Children[3]);
	}
}

void QuadTree::divideNode(Node& node) {
	// Node division
	node.Children = new Node[4];

	int x = node.x;
	int y = node.y;
	int width = node.width;
	int height = node.height;

	node.Children[0].x = x;
	node.Children[0].y = y;
	node.Children[0].width = width / 2;
	node.Children[0].height = height / 2;

	node.Children[1].x = x + width / 2;
	node.Children[1].y = y;
	node.Children[1].width = width - width / 2;
	node.Children[1].height = height / 2;

	node.Children[2].x = x;
	node.Children[2].y = y + height / 2;
	node.Children[2].width = width / 2;
	node.Children[2].height = height - height / 2;
	
	node.Children[3].x = x + width / 2;
	node.Children[3].y = y + height / 2;
	node.Children[3].width = width - width / 2;
	node.Children[3].height = height - height / 2;
}

// Coloring average bgr value in destination image.
void QuadTree::coloring(cv::Point3f avg, int x, int y, int width, int height) {
	uchar* dImage = dst.data;
	for (int r = y; r < y + height; r++) {
		for (int c = x; c < x + width; c++) {
			int offset = r * img.cols * 3 + c * 3;
			dImage[offset + 0] = avg.x;
			dImage[offset + 1] = avg.y;
			dImage[offset + 2] = avg.z;
		}
	}
}

void QuadTree::TestShowImage(int x, int y, int width, int height) {
	cv::imshow("Test", img(cv::Rect(x, y, width, height)));
	cv::waitKey(0);
}
cv::Mat QuadTree::getResultMat() {
	return dst;
}

vector<cv::KeyPoint> QuadTree::getKeyPoints(){
    return keypoints;
}