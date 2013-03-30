#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "../CameraProcessor.h"
#include "../SensorProcessor.h"

#include "opencv2/highgui/highgui.hpp"

const int num_names = 39;


static const bool USE_ROTATED = true;
static const int use_width = 320;
static const int use_height = 240;
const char * image_names[num_names] = {
			"NASA_bmp/img26_series_b_side_204_e65.jpg",
			"NASA_bmp/img19_series_b_side_110_e65.jpg",
			"NASA_bmp/img12_series_b_center_224_e65.jpg",
			"NASA_bmp/img23_series_b_side_101_e65.jpg",
			"NASA_bmp/img15_series_b_side_230_e65.jpg",
			"NASA_bmp/img10_series_b_center_203_e65.jpg",
			"NASA_bmp/img11_series_b_center_203_e65.jpg",
			"NASA_bmp/img13_series_b_center_260_e65.jpg",
			"NASA_bmp/img14_series_b_center_251_e65.jpg",
			"NASA_bmp/img16_series_b_side_196_e65.jpg",
			"NASA_bmp/img17_series_b_side_160_e65.jpg",
			"NASA_bmp/img18_series_b_side_140_e65.jpg",
			"NASA_bmp/img1_center_200_e65.jpg",
			"NASA_bmp/img20_series_b_side_114_e65.jpg",
			"NASA_bmp/img21_series_b_side_137_e65.jpg",
			"NASA_bmp/img22_center field_e10.jpg",
			"NASA_bmp/img22_dog Center Field_e10.jpg",
			"NASA_bmp/img22_series_b_side_150_e65.jpg",
			"NASA_bmp/img23_center field_e10.jpg",
			"NASA_bmp/img24_center field_e10.jpg",
			"NASA_bmp/img24_series_b_side_104_e65.jpg",
			"NASA_bmp/img25_series_b_side_195_e65.jpg",
			"NASA_bmp/img27_series_b_side_192_e65.jpg",
			"NASA_bmp/img28_series_b_side_192_e65.jpg",
			"NASA_bmp/img29_series_b_side_186_e65.jpg",
			"NASA_bmp/img2_center_207_e65.jpg",
			"NASA_bmp/img30_series_b_side_177_e65.jpg",
			"NASA_bmp/img31_series_b_side_176_e65.jpg",
			"NASA_bmp/img32_series_b_side_212_e65.jpg",
			"NASA_bmp/img33_series_b_side_251_e65.jpg",
			"NASA_bmp/img34_series_b_side_272_e65.jpg",
			"NASA_bmp/img35_series_b_side_23+219_e65.jpg",
			"NASA_bmp/img3_center_180_e65.jpg",
			"NASA_bmp/img4_series_b_center_106_e65.jpg",
			"NASA_bmp/img5_series_b_center_122_e65.jpg",
			"NASA_bmp/img6_series_b_center_145_e65.jpg",
			"NASA_bmp/img7_series_b_center_174_e65.jpg",
			"NASA_bmp/img8_series_b_center_196_e65.jpg",
			"NASA_bmp/img9_series_b_center_201_e65.jpg"};

const char	* WINDOW_NAME	= "Treshhold Window";
const char	* WINDOW_NAME2	= "Target Window";


static void onMouse( int event, int x, int y, int, void* userData ) {
	if( event != CV_EVENT_LBUTTONDOWN ) return;
	ProcessorData *proc = (ProcessorData *) userData;
	IplImage *image = proc->src_header_image;
	uchar b = *((uchar*) (image->imageData + y*image->widthStep + 3*x));
	uchar g = *((uchar*) (image->imageData + y*image->widthStep + 3*(x+1)));
	uchar r = *((uchar*) (image->imageData + y*image->widthStep + 3*(x+2)));
	
	uchar h=0;
	uchar s=0;
	uchar v=0;
	proc->RGBtoHSV(r, g, b, &h, &s, &v);

	
	*((uchar*) (image->imageData + y*image->widthStep + 3*x)) = 128;
	*((uchar*) (image->imageData + y*image->widthStep + 3*(x+1))) = 128;
	*((uchar*) (image->imageData + y*image->widthStep + 3*(x+2))) = 255;
	
	cv::Mat src(image);
	//cv::imshow("test", src);

	printf("got click (%d,%d)= <%d,%d,%d> -- [%d,%d,%d]\n",
			x, y, r, g, b, h, s, v);
}


int main( int argc, char *argv[] ){
	ProcessorData processor(use_width, use_height, USE_ROTATED);
	int img_cycle = 0;
	int thresh = 100;
	
	cvStartWindowThread();

	cvNamedWindow ("cnt", CV_WINDOW_AUTOSIZE);
	cvNamedWindow ("GLOBAL", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow ("Grey Img", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow ("test", CV_WINDOW_AUTOSIZE);
	cvNamedWindow (WINDOW_NAME2, CV_WINDOW_AUTOSIZE);
	cvNamedWindow (WINDOW_NAME, CV_WINDOW_AUTOSIZE);

	cvMoveWindow(WINDOW_NAME,0,0);
	cvMoveWindow("GLOBAL",325,0);
	cvMoveWindow(WINDOW_NAME2,650,0);
	//cvMoveWindow("Grey Img", 0, 275);
	//cvMoveWindow("test", 325, 275);
	cvMoveWindow("cnt",1100,100);
	//Creating the trackbars
	cvCreateTrackbar("H1","cnt",&processor.h1,360,0);
	cvCreateTrackbar("H2","cnt",&processor.h2,360,0);
	cvCreateTrackbar("S1","cnt",&processor.s1,255,0);
	cvCreateTrackbar("S2","cnt",&processor.s2,255,0);
	cvCreateTrackbar("V1","cnt",&processor.v1,255,0);
	cvCreateTrackbar("V2","cnt",&processor.v2,255,0);
	
	while (img_cycle >= 0) {
		processor.clear();
		printf("%d = %s\n", img_cycle, image_names[img_cycle]);
		processor.src_header_image = cvLoadImage(image_names[img_cycle]);
		cvCopy(processor.src_header_image, processor.global_display);

		cv::setMouseCallback( WINDOW_NAME2, onMouse,
				(void *)&processor );

		cv::Mat global_mat(processor.global_display);
		cv::Mat src_mat(processor.src_header_image);

		// These lines are the vision processing, the rest of main
		// is just fluff
		processor.threshold((uchar *)
				processor.src_header_image->imageData);
		processor.getContours();
		processor.filterToTargets();

		if(!processor.target_list.empty()){
			std::vector<std::vector<cv::Point> > target_contours;
			std::vector<std::vector<cv::Point> > best_contours;
			std::vector<std::vector<cv::Point> > raw_contours;
			std::vector<Target>::iterator target_it;
			Target *best_target = NULL;
			int i = 0;
			for(target_it = processor.target_list.begin();
					target_it != processor.target_list.end(); target_it++){
				target_contours.push_back(target_it->this_contour);
				raw_contours.push_back(target_it->raw_contour);
				printf("%d: h=%.1f, interp=%.1f, <x,y>=<%.1f,%.1f>\n",
						i++, target_it->height,
						interpolate(4, &pixel_to_dist[0], target_it->rect.centroid.x),
						target_it->rect.centroid.x, target_it->rect.centroid.y);
				if (best_target == NULL) {
					best_target = &*target_it;
				} else {
					if (target_it->height > best_target->height) {
						best_target = &*target_it;
					}
				/*	if (processor.is_90) {
						if (target_it->rect.centroid.x > best_target->rect.centroid.x) {
							best_target = &*target_it;
						}
					} else {
						if (target_it->rect.centroid.y < best_target->rect.centroid.y) {
							best_target = &*target_it;
						}
					}*/
				}
			}
			best_contours.push_back(best_target->this_contour);
			//drawContours(global_mat,target_contours,-1,color,CV_FILLED);
			cv::imshow(WINDOW_NAME, src_mat);
			//cv::imshow("Grey Img", *processor.grey_mat);
			cv::Scalar color(0,0,255);
			cv::drawContours( src_mat, target_contours, -1, color, CV_FILLED );
			cv::Scalar color2(128,0,255);
			cv::drawContours( src_mat, best_contours, -1, color2, CV_FILLED );
			cv::Scalar color3(0,255,0);
			cv::drawContours( src_mat, raw_contours, -1, color3, 1 );
		}
		//cv::Mat grey_mat(grey_image);
		//cv::imshow(WINDOW_NAME2, grey_mat);
		cv::imshow("GLOBAL", global_mat);
		cv::imshow(WINDOW_NAME2, src_mat);
		char key = cvWaitKey(3000);
		switch (key) {
			case ' ':
				img_cycle++;
				img_cycle = img_cycle % num_names;
				printf("%c %d= %s\n", key, img_cycle, image_names[img_cycle]);
				break;
			case 'g':
				thresh++;
				thresh = (thresh % 255);
				printf("+ thresh= %d\n", thresh);
				break;
			case 'G':
				thresh--;
				thresh = (thresh % 255);
				printf("- thresh= %d\n", thresh);
				break;
			case 'q':
				img_cycle = -1;
				break;
			default:
				break;
		}
		//redraw image cuz we drew all over it
	}

	cvDestroyWindow(WINDOW_NAME);
}

