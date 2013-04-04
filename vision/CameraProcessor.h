#ifndef VISION_CAMERA_PROCESSOR_H_
#define VISION_CAMERA_PROCESSOR_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <utility>
#include <vector>

#include "opencv2/imgproc/imgproc.hpp"

// an over described geometric representation of a rectangle
class FullRect {
 public:
  FullRect();
  cv::Point2f ur; // upper right
  cv::Point2f ul; // upper left
  cv::Point2f br; // bottom right
  cv::Point2f bl; // bottom_left
  cv::Point2f centroid; //centroid
};

// All data needed once a target is found
class Target {
	public:
		Target(std::vector<cv::Point> new_contour,
			std::vector<cv::Point> new_raw_contour,
			FullRect new_rect, int new_weight, bool _is_90);
		void refineTarget();
		double getHeight(bool is_90);
		FullRect rect; // geometric representation of the target
		std::vector<cv::Point> this_contour; // opencv contour of target
		std::vector<cv::Point> raw_contour; // opencv contour of target
		double height; // top target to robot
		double weight; // confidence in this target
};

// main class for processing image data. All relavent data should be
// accessible through this structure.
class ProcessorData {
	public:
		ProcessorData(int width, int height, bool is_90_);
		~ProcessorData();
		void RGBtoHSV(uchar r, uchar g, uchar b,
			uchar *h, uchar *s, uchar *v);
		void threshold(uchar* buffer);
		void getContours();
		void filterToTargets();
		void clear();
	//protected:
		int img_width; // all images should be this width
		int img_height; // and this height
		bool is_90;
		int buffer_size; // width * height * 3
		IplImage *grey_image; // thresholded image
		cv::Mat *grey_mat; // Matrix representaion (just a header)
		std::vector<std::pair<std::vector<cv::Point>,
			std::vector<cv::Point> > > contour_pairs;
		//std::vector<std::vector<cv::Point> > contours; // filtered contours
		//yystd::vector<std::vector<cv::Point> > raw_contours; //original contours
		std::vector<cv::Vec4i> hierarchy; // ordering on contours
		cv::MemStorage g_storage; // opencv storage
		static const int HIST_SIZE = 20; // dimension of histogram
								 // ie number of scan lines
		static const double HIST_SIZE_F = 1.0/20.0; // step size
											// should be 1/HIST_SIZE
		double vert_hist[HIST_SIZE]; // desired vertical histogram
		double horz_hist[HIST_SIZE]; // desired horizontal histogram
		// defines the minimum dist for a match
		static const double HIST_MATCH = 1.9;
		double calcHistComponent(
				cv::Point2i start,
				cv::Point2i end,
				cv::Mat thresh_img);
		double checkHistogram(
				FullRect rect,
				cv::Mat thresh_img);
	public:
		int h1, s1, v1, h2, s2, v2; // HSV min and max
									// must be public for tuning
		IplImage * global_display;

		IplImage *src_header_image; // header for main image
		std::vector<Target> target_list; // list of found targets
};

#endif  // VISION_CAMERA_PROCESSOR_H_
