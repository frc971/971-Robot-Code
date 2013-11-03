#include "vision/CameraProcessor.h"
#include "aos/common/logging/logging.h"

//#define LOCAL_DEBUG 1

// create a new target
Target::Target(std::vector<cv::Point> new_contour,
		std::vector<cv::Point> new_raw_contour,
		FullRect new_rect, int new_weight, bool _is_90) {
	this_contour = new_contour;
	raw_contour = new_raw_contour;
	rect = new_rect;
	weight = new_weight;
	height = getHeight(_is_90);
}

// calculate distance to target
double Target::getHeight(bool is_90) {
	// The 780.3296 is at full resolution 640x480, and we need
	// to scale back to 320x240
	//static const double cam_l = 780.3296 / 2.0; 
	////static const double cam_d = 20.78096;
	double height;
	if (is_90) {
		height = ((rect.ul.x - rect.ur.x) +
				(rect.bl.x - rect.br.x)) / 2.0;
	} else {
		height = ((rect.ur.y + rect.ul.y) -
				(rect.br.y + rect.bl.y)) / 2.0;
	}
	//return cam_l * 12.0 / height;
	return height;
}

void Target::refineTarget() {
	printf("okay refined\n");
}

FullRect::FullRect() {
	ur.x = -1;
	ur.y = -1;
	ul.x = -1;
	ul.y = -1;
	br.x = -1;
	br.y = -1;
	bl.x = -1;
	bl.y = -1;
}

// turns a contour into easier to access structure
FullRect calcFullRect(std::vector<cv::Point> *contour){
	FullRect rect;
	for(int i=0; i<4; i++){
		cv::Point2f pt = (*contour)[i];
		rect.centroid.x += pt.x;
		rect.centroid.y += pt.y;
	}
	rect.centroid.x /= 4;
	rect.centroid.y /= 4;
	for(int i=0; i<4; i++){
		cv::Point2f pt = (*contour)[i];
		if(pt.y > rect.centroid.y ){
			if(pt.x > rect.centroid.x){
				if (rect.ul.x < 0) {
					rect.ul = pt;
				} else {
					rect.ur = pt;
				}
			}else{
				if (rect.ur.x < 0) {
					rect.ur = pt;
				} else {
					rect.ul = pt;
				}
			}
			if (rect.ul.x > 0 && rect.ur.x > 0) {
				// both are set, so if we got it wrong correct it here
				if (rect.ul.x > rect.ur.x) {
					pt = rect.ur;
					rect.ur = rect.ul;
					rect.ul = pt;
				}
			}
		}else{
			if(pt.x > rect.centroid.x){
				if (rect.bl.x < 0) {
					rect.bl = pt;
				} else {
					rect.br = pt;
				}
			}else{
				if (rect.br.x < 0) {
					rect.br = pt;
				} else {
					rect.bl = pt;
				}
			}
			if (rect.bl.x > 0 && rect.br.x > 0) {
				// both are set, so if we got it wrong correct it here
				if (rect.bl.x > rect.br.x) {
					pt = rect.br;
					rect.br = rect.bl;
					rect.bl = pt;
				}
			}
		}
	}
	return rect;
}

// quickly remove targets that do not fit a very broad set of constraints
bool cullObvious(FullRect rect, double outside_size){
	// first check that could see a target this size
	// Calculated from dave's simulation, shloud be 850 and 72000 if filled
	if((outside_size < 500) || (outside_size > 90000)){
		return false;
	}
	// Targets on the edge are at best inaccurate.
	// In this case, we just want to point the right way,
	// so this is no longer a valid assumption.
	/*if(	rect.ur.x < 2 || rect.ur.y < 2 || rect.ur.x > 637 || rect.ur.y > 477 ||
		rect.ul.x < 2 || rect.ul.y < 2 || rect.ul.x > 637 || rect.ul.y > 477 ||
		rect.br.x < 2 || rect.br.y < 2 || rect.br.x > 637 || rect.br.y > 477 ||
		rect.bl.x < 2 || rect.bl.y < 2 || rect.bl.x > 637 || rect.bl.y > 477){
		return false;
	}*/
	// make sure the sides are close to the right ratio of a rect
	// some really odd shapes get confusing
	double ratio = norm(rect.ur-rect.ul)/norm(rect.br-rect.bl);
	if( ratio < .7 || ratio > 1.4 ) {
		return false;
	}
	ratio = norm(rect.ur-rect.br)/norm(rect.ul-rect.bl);
	if( ratio < .7 || ratio > 1.4 ) {
		return false;
	}

	return true;
}

// sum over values between these two points and normalize
// see Bresenham's Line Algorithm for the logic of moving
// over all the pixels between these two points.
double ProcessorData::calcHistComponent(
		cv::Point2i start,
		cv::Point2i end,
		cv::Mat thresh_img){
	int dx = abs(end.x - start.x);
	int dy = abs(end.y - start.y);
	int sx = (start.x < end.x) ? 1 : -1;
	int sy = (start.y < end.y) ? 1 : -1;
	int error = dx-dy;

	int total = 0;
	int value = 0;
	int total_error;
#if LOCAL_DEBUG
	IplImage gd = *global_display;
#endif
	IplImage ti = thresh_img;
	while(1){
		total++;
		
		uchar* ptr = (uchar*) (ti.imageData + start.y * ti.widthStep + start.x);
		if((int) *ptr) value++;
		// draw line checked
#if LOCAL_DEBUG
		uchar* ptr2 = (uchar*) (gd.imageData + start.y * gd.widthStep + start.x*3);
		*ptr2++ = 0;
		*ptr2++ = 255;
		*ptr2 = 0;
#endif
		if(start.x == end.x && start.y == end.y) break;
		total_error = 2 * error;
		if(total_error > -dy){
			error -=  dy;
			start.x += sx;
		}
		if(total_error < dx){
			error += dx;
			start.y += sy;
		}
	}
	return (double)value/(double)total;
}

// just a distance function
double chiSquared(int length, double* histA, double* histB){
	double sum = 0;
	for(int i=0; i<length;i++){
		double diff = *(histB+i) - *(histA+i);
		sum += (diff * diff) / *(histA+i);
	}
	return sum;
}
// euclidiean dist function
double L2_dist(int length, double* histA, double* histB){
	double sum = 0;
	for(int i=0; i<length;i++){
		double diff = *(histB+i) - *(histA+i);
		sum += (diff * diff);
	}
	return sqrt(sum);
}

// calc and compare the histograms to the desired
double ProcessorData::checkHistogram(FullRect rect, cv::Mat thresh_img){
	// found horiz histogram
	double hist_lr[HIST_SIZE];
	// step size along left edge
	cv::Point2f delta_left = (rect.ul - rect.bl)*HIST_SIZE_F;
	// step size along right edge
	cv::Point2f delta_right = (rect.ur - rect.br)*HIST_SIZE_F;
	// sum each left to right line for the histogram
	for(int i=0; i<HIST_SIZE; i++){
		hist_lr[i] = calcHistComponent(rect.bl+i*delta_left,
				rect.br+i*delta_right,thresh_img);
	}
	double check_vert = L2_dist(HIST_SIZE, vert_hist, hist_lr);
	// found vert histogram
	double hist_ub[HIST_SIZE];
	// step size along bottom edge
	cv::Point2f delta_bottom = (rect.bl - rect.br)*HIST_SIZE_F;
	// step size along top edge
	cv::Point2f delta_top = (rect.ul - rect.ur)*HIST_SIZE_F;
	// sum each top to bottom line for the histogram
	for(int i=0; i<HIST_SIZE; i++){
		hist_ub[i] = calcHistComponent(rect.ur+i*delta_top,
				rect.br+i*delta_bottom,thresh_img);
	}
	double check_horz = L2_dist(HIST_SIZE, horz_hist, hist_ub);

	// average the two distances
	double check = (check_vert + check_horz)/2.0;
	return check;
}

// return smallest
template<class T> inline T Min3(T x, T y, T z) {
  return y <= z ? (x <= y ? x : y)
                : (x <= z ? x : z);
}

// return largest
template<class T> inline T Max3(T x, T y, T z) {
  return y >= z ? (x >= y ? x : y)
                : (x >= z ? x : z);
}

// transforms the contour
void makeConvex(std::vector<cv::Point> *contour){
	std::vector<cv::Point2i> hull;
	convexHull(*contour, hull, false);
	*contour = hull;
}

// basic init
ProcessorData::ProcessorData(int width, int height, bool is_90_) {
	is_90 = is_90_;
	// series b images from nasa
	h1=79;  s1=53;   v1=82;
	h2=200; s2=255; v2=255;
	// For images from Jerry
	//h1=79;  s1=0;   v1=11;
	//h2=160; s2=255; v2=255;
	img_width = width;
	img_height = height;
	buffer_size = img_height * img_width * 3;
#if LOCAL_DEBUG
        global_display = cvCreateImage(cvSize(width, height),
                                       IPL_DEPTH_8U, 3);
#endif
	grey_image = cvCreateImage(cvSize(width, height),
			           IPL_DEPTH_8U, 1);
	grey_mat = new cv::Mat(grey_image);
	
	// calculate a desired histogram before we start
	int j = 0;
	for(double i=0; j<HIST_SIZE; i+=HIST_SIZE_F){
		if (is_90) {
			if(i < 2.0/12.0 || i > (1.0-2.0/12.0) ) horz_hist[j] = 1;
			else horz_hist[j] = 0.10;
			if(i < 2.0/24.0 || i > (1.0-2.0/24.0) ) vert_hist[j] = 1;
			else vert_hist[j] = 4.0/24.0;
		} else {
			if(i < 2.0/12.0 || i > (1.0-2.0/12.0) ) vert_hist[j] = 1;
			else vert_hist[j] = 0.10;
			if(i < 2.0/24.0 || i > (1.0-2.0/24.0) ) horz_hist[j] = 1;
			else horz_hist[j] = 4.0/24.0;
		}
		j++;
	}

        src_header_image = cvCreateImage(cvSize(width, height),
            IPL_DEPTH_8U, 3);
}

// throw stuff away
ProcessorData::~ProcessorData() {
	cvReleaseImage(&grey_image);
	cvReleaseImage(&src_header_image);
	delete(grey_mat);
}

// reset processor data freeing as little as possible.
void ProcessorData::clear() {
	target_list.clear();
	contour_pairs.clear();
	hierarchy.clear();
}


// r,g,b values are from 0 to 255
// h = [0,255], s = [0,255], v = [0,255]
//		if s == 0, then h = 0 (undefined)
void ProcessorData::RGBtoHSV(uchar r, uchar g, uchar b,
		uchar *h, uchar *s, uchar *v ) {
	uchar min, max, delta;
	min = Min3( r, g, b );
	max = Max3( r, g, b );
	*v = max;
	delta = max - min;
	if (max == 0 || delta == 0) {
		*s = 0;
		*h = 0;
		return;
	}
	*s = (255 * long(delta))/max;
	if (max == r) {
		*h = 0 + 43*(g - b)/(delta);
	} else if (max == g) {
		*h = 85 + 43*(b - r)/(delta);
	} else {
		*h = 171 + 43*(r - g)/(delta);
	}
}

// keep anything that is in our HVS wedge
// by far the longest running function in this code
void ProcessorData::threshold(uchar* buffer) {
#if LOCAL_DEBUG
  uchar * draw_buffer = (uchar *) global_display->imageData;
#endif
  for (int i = 0, j = 0; i < (buffer_size); i+= 3, j++) {
    uchar r = buffer[i + 2];
    uchar g = buffer[i + 1];
    uchar b = buffer[i + 0];
    uchar h, s, v;

    RGBtoHSV(r, g, b, &h, &s, &v);

    if (g > 128) {
#if LOCAL_DEBUG
      draw_buffer[i + 0] = 120;
      draw_buffer[i + 1] = 80;
      draw_buffer[i + 2] = 70;
#endif
      grey_image->imageData[j] = 255;
    } else if (h == 0 && s == 0 && v >= v1 && v <= v2) { 
      // Value thresholds invalid pixels.
#if LOCAL_DEBUG
      draw_buffer[i + 0] = 200;
      draw_buffer[i + 1] = 50;
      draw_buffer[i + 2] = 100;
#endif
      grey_image->imageData[j] = 255;
    } else if (h >= h1 && h <= h2 && v >= v1 &&
               v <= v2 && s >= s1 && s <= s2){
      // HSV Thresholded image.
#if LOCAL_DEBUG
      draw_buffer[i + 0] = 255;
      draw_buffer[i + 1] = 0;
      draw_buffer[i + 2] = 0;
#endif
      grey_image->imageData[j] = 255;
    } else {
      // Display the unmodified image.
#if LOCAL_DEBUG
      draw_buffer[i + 0] = buffer[i + 0];
      draw_buffer[i + 1] = buffer[i + 1];
      draw_buffer[i + 2] = buffer[i + 2];
#endif
      grey_image->imageData[j] = 0;
    }

  }

}

// run find contours and try to make them squares
void ProcessorData::getContours() {
	std::vector<std::vector<cv::Point> > raw_contours;
	//cv::findContours(*grey_mat, raw_contours, hierarchy, CV_RETR_LIST,
	//		CV_CHAIN_APPROX_SIMPLE);
	cv::findContours(*grey_mat, raw_contours, hierarchy, CV_RETR_EXTERNAL,
			CV_CHAIN_APPROX_SIMPLE);
#if LOCAL_DEBUG
	cv::Mat global_mat(global_display);
	cv::Scalar color(255,0,0);
	drawContours(global_mat,raw_contours,-1,color,1);

	std::vector<std::vector<cv::Point> > p_contours;
#endif
	if (!raw_contours.empty()) {
		std::vector<std::vector<cv::Point2i> >::iterator contour_it;
		for (contour_it=raw_contours.begin();
				contour_it != raw_contours.end();
				contour_it++) {
			// make the contours convex
			makeConvex(&(*contour_it));
			std::vector<cv::Point> contour;
			//then make them rectangle
			approxPolyDP(*contour_it, contour, 9, true);
			// stick the raw and processed contours together
			std::pair<std::vector<cv::Point>,
				std::vector<cv::Point> > a_pair(
						*contour_it, contour);
#if LOCAL_DEBUG
			p_contours.push_back(contour);
#endif
			// and put them in a list
			contour_pairs.push_back(a_pair);
		}
	}
#if LOCAL_DEBUG
	cv::Scalar color2(0,0,255);
	drawContours(global_mat,p_contours,-1,color2,CV_FILLED);
#endif
}

// filter the contours down to a list of targets
void ProcessorData::filterToTargets() {
  std::vector<std::pair<
    std::vector<cv::Point>,
    std::vector<cv::Point> > >::iterator contour_it;
  for(contour_it=contour_pairs.begin();
      contour_it != contour_pairs.end();
      contour_it++){
    double check = 0;
    std::vector<cv::Point> raw_contour = std::get<0>(*contour_it);
    std::vector<cv::Point> contour = std::get<1>(*contour_it);
    FullRect rect = calcFullRect(&contour);
    if(contour.size() == 4 &&
        cullObvious(rect, contourArea(contour)) &&
        (check = checkHistogram(rect, *grey_mat)) <= HIST_MATCH){
      // now we have a target, try to improve the square
#if LOCAL_DEBUG
      /*	printf("________\n");
                printf("\tcont= %d raw= %d\n",
                (int)contour.size(), (int)raw_contour.size());
                std::vector<cv::Point2i>::iterator point_it;
                for(point_it=raw_contour.begin();
                point_it != raw_contour.end(); point_it++){
                printf("(%d,%d)", point_it->x, point_it->y);
                }
                printf("\n");*/
#endif
      target_list.push_back(Target(contour,
            raw_contour, rect, check, is_90));
    }
    if (contour.size() == 4 && cullObvious(rect, contourArea(contour))) {
    	LOG(DEBUG, "check= %.2f\n", check);
    }
  }
}

