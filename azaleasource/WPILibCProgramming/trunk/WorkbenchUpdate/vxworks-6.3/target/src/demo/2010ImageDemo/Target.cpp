#include "nivision.h"
#include "Vision/MonoImage.h"
#include "Target.h"
#include <algorithm>
#include <math.h>

// These parameters set ellipse finding in the NI imaq (Image Aquisition) library.
// Refer to the CVI Function Reference PDF document installed with LabVIEW for
// additional information.
static EllipseDescriptor ellipseDescriptor = {
										3, 		// minMajorRadius
										200, 	// maxMajorRadius
										3, 		// minMinorRadius
										100		// maxMinorRadius
										};

static CurveOptions curveOptions = {	IMAQ_NORMAL_IMAGE,	// extractionMode
										40, 				// threshold
										IMAQ_NORMAL, 		// filterSize
										25, 				// minLength
										15, 				// rowStepSize 
										15, 				// columnStepSize
										10, 				// maxEndPointGap
										1,					// onlyClosed
										0					// subpixelAccuracy
										};
static ShapeDetectionOptions shapeOptions = {
										IMAQ_GEOMETRIC_MATCH_SHIFT_INVARIANT,	// mode
										NULL,			// angle ranges
										0,				// num angle ranges
										{75, 125},		// scale range
										500				// minMatchScore
};


double Target::GetHorizontalAngle()
{
	double x = m_xPos;
	x = x * 9.0 / m_xMax;
	x = atan2(x, 20.0);
	x = x * 180.0 / 3.14159;
	return x;
}

/**
 * Compare two targets.
 * Compare the score of two targets for the sort function in C++.
 * @param t1 the first Target
 * @param t2 the second Target
 * @returns (1, 0, or -1) for the scores of t1 > t2, t1 == t2, and t1 < t2
 */
int compareTargets(Target t1, Target t2)
{
	if (t1.m_score > t2.m_score) return 1;
	if (t1.m_score < t2.m_score) return -1;
	return 0;
}

/**
 * Find the best circular target in the image.
 * The best target is the one with two concentric circles, largest size, and best score
 * @param image The image to examine.
 * @returns A target object with the parameters filled in.
 */
vector<Target> Target::FindCircularTargets(HSLImage *image)
{
	wpi_assert(image != NULL);
	int width = image->GetWidth();
	int height = image->GetHeight();

	vector<Target> sortedTargets;

	// get the luminance plane only for the image to make the code
	// insensitive to lighting conditions.
	MonoImage  *luminancePlane = image->GetLuminancePlane();
	vector<EllipseMatch> *results = luminancePlane->DetectEllipses(&ellipseDescriptor, 
																	&curveOptions,
																	&shapeOptions,
																	NULL);
	delete luminancePlane;
	if (results->size() == 0)
	{
		return sortedTargets;
	}

	// create a list of targets corresponding to each ellipse found
	// in the image.
	for (unsigned i = 0; i < results->size(); i++)
	{
		Target target;
		EllipseMatch e = results->at(i);
		target.m_rawScore = e.score;
		target.m_score = (e.majorRadius * e.minorRadius)
							/ (1001 - e.score)
							/ (height * width) * 100;
		target.m_majorRadius = e.majorRadius / height;
		target.m_minorRadius = e.minorRadius / height;
		//always divide by height so that x and y are same units
		target.m_xPos = (2.0 * e.position.x - width) / height;
		target.m_yPos = (2.0 * e.position.y - height) / height;
		target.m_rotation = e.rotation;
		target.m_xMax = (double)width / height;
		target.m_bothFound = false;
		sortedTargets.push_back(target);
	}
	delete results;
	
	// sort the list of targets by score
	sort(sortedTargets.begin(), sortedTargets.end(), compareTargets);
	
	// go through each target found in descending score order and look
	// for another target whose center is contained inside of this target
	// Those concentric targets get a score which is the sum of both targets
	vector<Target> combinedTargets;
	while (sortedTargets.size() > 0)
	{
		vector<Target>::iterator iter = sortedTargets.begin();
		Target t1 = *iter++;
		for (; iter < sortedTargets.end(); iter++)
		{
			Target t2 = *iter;
			
			// check if the two are concentric
			if ((fabs(t1.m_xPos - t2.m_xPos) < min(t1.m_minorRadius, t2.m_minorRadius)) &&
					(fabs(t1.m_yPos - t2.m_yPos) < min(t1.m_majorRadius, t2.m_majorRadius)))
			{
				// create the information for the combined target
				// (the 2 concentric ellipses)
				t1.m_xPos = (t1.m_xPos + t2.m_xPos) / 2;
				t1.m_yPos = (t1.m_yPos + t2.m_yPos) / 2;
				t1.m_rawScore += t2.m_rawScore;
				t1.m_score = (t1.m_score + t2.m_score) * 2.0;  // add a 2x bonus for concentric
				t1.m_majorRadius = max(t1.m_majorRadius, t2.m_majorRadius);
				t1.m_minorRadius = max(t1.m_minorRadius, t2.m_minorRadius);
				t1.m_bothFound = true;
				sortedTargets.erase(iter); // loop needs to be an interator
				break;
			}
		}
		sortedTargets.erase(sortedTargets.begin());
//		if (t1.m_bothFound)
			combinedTargets.push_back(t1);
	}

	// sort the combined targets so the highest scoring one is first
	sort(combinedTargets.begin(), combinedTargets.end(), compareTargets);

	return combinedTargets;
}

/**
 * Print the target.
 * Print information about this target object.
 */
void Target::Print()
{
	printf("Target found: (%f, %f), major: %f, minor: %f\nrotation: %f, score: %f, both: %d\n\n",
						m_xPos, m_yPos,
						m_majorRadius, m_minorRadius,
						m_rotation, m_score, m_bothFound);
}
