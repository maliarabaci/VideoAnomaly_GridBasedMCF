/*
* \file MotionVectorExtractor.cpp
*
* \brief
*    Implementation of MotionVectorExtractor class.
*
* \author
*    Mehmet Ali Arabacı
*
* Created on: Nov 13, 2012
*/

#include "MotionVectorExtractor.h"

#include <cmath>

using namespace std;

static const double PI = 3.14159265358979323846;

MotionVectorExtractor::MotionVectorExtractor(){

}

MotionVectorExtractor::~MotionVectorExtractor(){

}

bool MotionVectorExtractor::Extract(const string& path_video, MotionVectorExtractor::eMotionMethod mMethod, int nFrameSkip)
{
	int nblock_size;
	long number_of_frames;
	CvSize frame_size, block, shift, maxRange, size;

	vector<vector<double> > vecMotionVelTemp, vecMotionAngleTemp;
	vector<vector<pair<int, int> > > vecMotionPosStartTemp, vecMotionPosEndTemp;
	vector<double> vecFrameMotionVel, vecFrameMotionAngle;
	vector<pair<int, int> > vecFrameMotionPosStart, vecFrameMotionPosEnd;
	vector<vector<stMotionVector> > vidMotionVectorsTemp;
	IplImage *imgCurrentFrame_Gray, *imgPreviousFrame_Gray, *imgFrame_Temp;

	nblock_size = 16;

	CvCapture * input_video = cvCaptureFromFile(path_video.c_str());
	if (input_video == NULL)
	{
		cerr << endl << "The path of the input video wrong or the file is corrupted!" << endl;
		return false;
	}

	frame_size.height = (int)cvGetCaptureProperty(input_video, CV_CAP_PROP_FRAME_HEIGHT);
	frame_size.width = (int)cvGetCaptureProperty(input_video, CV_CAP_PROP_FRAME_WIDTH);

	imgPreviousFrame_Gray = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	imgCurrentFrame_Gray = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);

	/* Go to the end of the AVI (ie: the fraction is "1") */
	cvSetCaptureProperty(input_video, CV_CAP_PROP_POS_AVI_RATIO, 1.);
	/* Now that we're at the end, read the AVI position in frames */
	number_of_frames = (int)cvGetCaptureProperty(input_video, CV_CAP_PROP_POS_FRAMES);
	//number_of_frames = 5;
	/* Return to the beginning */
	cvSetCaptureProperty(input_video, CV_CAP_PROP_POS_FRAMES, 0.);

	int current_frame = 1;
	cvSetCaptureProperty(input_video, CV_CAP_PROP_POS_FRAMES, current_frame);

	IplImage * frame = cvQueryFrame(input_video);
	if (frame == NULL) {
		/* Why did we get a NULL frame?  We shouldn't be at the end. */
		cerr << "The first video frame is empty!" << endl;
		return false;
	}

	cvCvtColor(frame, imgPreviousFrame_Gray, CV_BGR2GRAY);

	block = cvSize(nblock_size, nblock_size);
	shift = cvSize(nblock_size / 2, nblock_size / 2);
	maxRange = cvSize(nblock_size / 4, nblock_size / 4);
	size = cvGetSize(imgPreviousFrame_Gray);
	IplImage * velocityX = cvCreateImage(size, IPL_DEPTH_32F, 1),
		*velocityY = cvCreateImage(size, IPL_DEPTH_32F, 1);

	vec_nFrameID.clear();
	for (int current_frame = 2; current_frame <= number_of_frames; current_frame++) {

		//cout << current_frame << endl;
		cvSetCaptureProperty(input_video, CV_CAP_PROP_POS_FRAMES, current_frame);
		frame = cvQueryFrame(input_video);
		if (frame == NULL)
			break;

		cvCvtColor(frame, imgCurrentFrame_Gray, CV_BGR2GRAY);

		ExtractMotionVectors(imgCurrentFrame_Gray, imgPreviousFrame_Gray, mMethod, vecFrameMotionVel, vecFrameMotionAngle, vecFrameMotionPosStart, vecFrameMotionPosEnd);

		
		for (int i = 0; i < vecFrameMotionPosStart.size(); i++) {

			Point p1(vecFrameMotionPosStart[i].first, vecFrameMotionPosStart[i].second);
			Point p2(vecFrameMotionPosEnd[i].first, vecFrameMotionPosEnd[i].second);

			cvLine(frame, p1, p2, Scalar(110, 220, 0), 2, 8);
		}

		cvNamedWindow ("Current Frame", CV_WINDOW_AUTOSIZE);
		cvShowImage("Current Frame", frame);
		cvWaitKey(1);
		
		/*
		// Write frame to the files
		stringstream ss;
		ss << "./out_frames/585/" << current_frame << ".jpg" ;
		cvSaveImage(ss.str().c_str(), frame);
		*/

		// Estimated motion vectors
		vecMotionVelTemp.push_back(vecFrameMotionVel);
		vecMotionAngleTemp.push_back(vecFrameMotionAngle);
		vecMotionPosStartTemp.push_back(vecFrameMotionPosStart);
		vecMotionPosEndTemp.push_back(vecFrameMotionPosEnd);
		vec_nFrameID.push_back(current_frame);

		// Replace the previous frame with the current frame (swap pointers)
		imgFrame_Temp = imgPreviousFrame_Gray;
		imgPreviousFrame_Gray = imgCurrentFrame_Gray;
		imgCurrentFrame_Gray = imgFrame_Temp;
	}

	vecMotionVel = vecMotionVelTemp;
	vecMotionAngle = vecMotionAngleTemp;
	vecMotionPosStart = vecMotionPosStartTemp;
	vecMotionPosEnd = vecMotionPosEndTemp;

	cvReleaseImage(&velocityX);
	cvReleaseImage(&velocityY);
	cvReleaseImage(&imgCurrentFrame_Gray);
	cvReleaseImage(&imgPreviousFrame_Gray);

	return true;
}

void MotionVectorExtractor::GetFeature(vector<vector<double> >& vecoutMotionVel, vector<vector<double> >& vecoutMotionAngle, vector<vector<pair<int, int> > >& vecoutMotionPosStart, vector<vector<pair<int, int> > >& vecoutMotionPosEnd, vector<int>& vecoutnFrameID) const
{
	vecoutMotionVel = vecMotionVel;
	vecoutMotionAngle = vecMotionAngle;
	vecoutMotionPosStart = vecMotionPosStart;
	vecoutMotionPosEnd = vecMotionPosEnd;
	vecoutnFrameID = vec_nFrameID;
}

void MotionVectorExtractor::ExtractMotionVectors(const IplImage* curFrame, const IplImage* preFrame, MotionVectorExtractor::eMotionMethod mMethod, vector<double>& vecFrameMotionVel, vector<double>& vecFrameMotionAngle, vector<pair<int, int> >& vecFrameMotionPosStart, vector<pair<int, int> >& vecFrameMotionPosEnd)
{
	vector<stMotionVector> vecMotionVectors;
	vector<double> vecMotionVelTemp, vecMotionAngleTemp;
	vector<pair<int, int> > vecMotionPosStart, vecMotionPosEnd;

	switch (mMethod)
	{
	case MOTION_OPTICAL_FLOW_LC:
		OpticalFlow_Lucas_Kanade(curFrame, preFrame, vecMotionVectors);
		break;
	case MOTION_OPTICAL_FLOW_FARNEBACK:
		OpticalFlow_Farneback(curFrame, preFrame, vecMotionVectors);
		break;
	case MOTION_OPTICAL_FLOW_BM:
		OpticalFlow_BM(curFrame, preFrame, vecMotionVectors);
		break;
		break;
	default:
		OpticalFlow_Lucas_Kanade(curFrame, preFrame, vecMotionVectors);
	}

	for (int i = 0; i < vecMotionVectors.size(); i++) {

		vecMotionVelTemp.push_back(vecMotionVectors[i].m_dbSize);
		vecMotionAngleTemp.push_back(vecMotionVectors[i].m_nAngle);
		vecMotionPosStart.push_back(vecMotionVectors[i].m_startPoint);
		vecMotionPosEnd.push_back(vecMotionVectors[i].m_endPoint);
	}

	vecFrameMotionVel = vecMotionVelTemp;
	vecFrameMotionAngle = vecMotionAngleTemp;
	vecFrameMotionPosStart = vecMotionPosStart;
	vecFrameMotionPosEnd = vecMotionPosEnd;
	//stMotionVect = stMotionVectTemp;
}

void MotionVectorExtractor::OpticalFlow_Lucas_Kanade(const IplImage* curFrame, const IplImage* preFrame, vector<stMotionVector>& stMotionVect)
{
	IplImage *eig_image = NULL, *temp_image = NULL, *prePyramid = NULL, *curPyramid = NULL;
	map<pair<int, int>, pair<int, int> > motionVect;

	CvSize frame_size;
	const int MAX_NOF_FEATURES = 400;
	int number_of_features;

	frame_size.height = curFrame->height;
	frame_size.width = curFrame->width;

	// cvGoodFeaturesToTrack
	// Determines strong corners on an image
	/*
	* "cvPreFrameGray"		 Input image
	* "eig_image" 			 will be ignored
	* "temp_image"			 will be ignored
	* "preFrame_features"	 will hold the feature points
	* "number_of_features"	 maximum number of features
	* ".01" 				 specifies the minimum quality of the features (based on the eigenvalues).
	* ".1" 				 specifies the minimum Euclidean distance between features.
	*/
	AllocateOnDemand(&eig_image, frame_size, IPL_DEPTH_32F, 1);
	AllocateOnDemand(&temp_image, frame_size, IPL_DEPTH_32F, 1);

	//number_of_features = 400;

	//This array will contain the features found in previous frame
	CvPoint2D32f preFrame_features[MAX_NOF_FEATURES];

	cvGoodFeaturesToTrack(preFrame, eig_image, temp_image, preFrame_features, &number_of_features, .01, .1, NULL);

	// This array will contain the locations of the points from the previous frame in the current frame
	CvPoint2D32f curFrame_features[MAX_NOF_FEATURES];

	/* The i-th element of this array will be non-zero if and only if the i-th feature of
	* previous frame was found in current frame.
	*/
	char optical_flow_found_feature[MAX_NOF_FEATURES];

	/* The i-th element of this array is the error in the optical flow for the i-th feature
	* of previous frame as found in current frame.  If the i-th feature was not found.
	*/
	float optical_flow_feature_error[MAX_NOF_FEATURES];

	// This is the window size to use to avoid the aperture problem
	CvSize optical_flow_window = cvSize(3, 3);

	/* This termination criteria tells the algorithm to stop when it has either done 20 iterations or when
	* epsilon is better than .3.  You can play with these parameters for speed vs. accuracy.
	*/
	CvTermCriteria optical_flow_termination_criteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);

	/* This is some workspace for the algorithm.
	* (The algorithm actually carves the image into pyramids of different resolutions.)
	*/
	AllocateOnDemand(&prePyramid, frame_size, IPL_DEPTH_8U, 1);
	AllocateOnDemand(&curPyramid, frame_size, IPL_DEPTH_8U, 1);

	cvCalcOpticalFlowPyrLK(preFrame, curFrame, prePyramid, curPyramid, preFrame_features, curFrame_features, number_of_features, optical_flow_window, 5, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria, 0.9999);

	for (int i = 0; i<number_of_features; i++)
	{
		pair<int, int> startPoint = pair<int, int>(preFrame_features[i].x, preFrame_features[i].y);
		pair<int, int> endPoint = pair<int, int>(curFrame_features[i].x, curFrame_features[i].y);

		double dist;

		dist = sqrt(pow(startPoint.first - endPoint.first, 2) + pow(startPoint.second - endPoint.second, 2));
		if (dist > 2)
		{
			pair<pair<int, int>, pair<int, int>> v = pair<pair<int, int>, pair<int, int>>(startPoint, endPoint);
			motionVect.insert(v);
		}
	}

	// Convert map of motion vectors to motion vector struct
	ConvertMapToStruct(motionVect, stMotionVect);

	cvReleaseImage(&prePyramid);
	cvReleaseImage(&curPyramid);
	cvReleaseImage(&eig_image);
	cvReleaseImage(&temp_image);
}

void MotionVectorExtractor::OpticalFlow_Farneback(const IplImage* curFrame, const IplImage* preFrame, vector<stMotionVector>& stMotionVect)
{
	IplImage *cvFlowGray = NULL;
	map<pair<int, int>, pair<int, int> > motionVect;

	CvSize frame_size;

	frame_size.height = curFrame->height;
	frame_size.width = curFrame->width;


	AllocateOnDemand(&cvFlowGray, frame_size, IPL_DEPTH_32F, 2);

	cvCalcOpticalFlowFarneback(preFrame, curFrame, cvFlowGray, 0.5, 3, 25, 3, 7, 1.5, 0);

	for (int y = 0; y < cvFlowGray->height; y += 8)
	{
		const float* f = (const float*)(cvFlowGray->imageData + cvFlowGray->widthStep*y);

		for (int x = 0; x < cvFlowGray->width; x += 8)
		{
			pair<int, int> startPoint = pair<int, int>(x, y);
			pair<int, int> endPoint = pair<int, int>(x + f[2 * x], y + f[2 * x + 1]);

			double dist;

			dist = sqrt((startPoint.first - endPoint.first) ^ 2 + (startPoint.second - endPoint.second) ^ 2);
			if (dist > 1)
			{
				pair<pair<int, int>, pair<int, int> > v = pair<pair<int, int>, pair<int, int> >(startPoint, endPoint);
				motionVect.insert(v);
			}
		}
	}

	// Convert map of motion vectors to motion vector struct
	ConvertMapToStruct(motionVect, stMotionVect);

	cvReleaseImage(&cvFlowGray);
}

void MotionVectorExtractor::OpticalFlow_BM(const IplImage* curFrame, const IplImage* preFrame, vector<stMotionVector>& stMotionVect)
{
	IplImage *velx = NULL, *vely = NULL;
	map<pair<int, int>, pair<int, int> > motionVect;

	CvSize frame_size;

	frame_size.height = curFrame->height;
	frame_size.width = curFrame->width;

	CvSize winSize, shiftSize, maxRange;
	winSize.height = 16;
	winSize.width = 16;

	shiftSize.height = 10;
	shiftSize.width = 10;

	maxRange.height = 5;
	maxRange.width = 5;

	CvSize velSize =
	{
		(preFrame->width - winSize.width + shiftSize.width) / shiftSize.width,
		(preFrame->height - winSize.height + shiftSize.height) / shiftSize.height
	};

	AllocateOnDemand(&velx, velSize, IPL_DEPTH_32F, 1);
	AllocateOnDemand(&vely, velSize, IPL_DEPTH_32F, 1);

	cvCalcOpticalFlowBM(preFrame, curFrame, winSize, shiftSize, maxRange, 0, velx, vely);

	int step = shiftSize.width;

	for (int y = 8; y < preFrame->height - step; y += step)
	{
		const float* fx = (const float*)(velx->imageData + velx->widthStep*((y - 8) / step));
		const float* fy = (const float*)(vely->imageData + vely->widthStep*((y - 8) / step));

		for (int x = 8; x < preFrame->width - step; x += step)
		{
			pair<int, int> startPoint = pair<int, int>(x, y);
			pair<int, int> endPoint = pair<int, int>(x + fx[((x - 8) / step)], y + fy[((x - 8) / step)]);

			if ((endPoint.first >= 0) && (endPoint.second >= 0) && (endPoint.first < preFrame->width) && (endPoint.second < preFrame->height))
			{
				double dist;

				dist = sqrt(pow(startPoint.first - endPoint.first, 2) + pow(startPoint.second - endPoint.second, 2));
				if (dist > 1)
				{
					pair<pair<int, int>, pair<int, int> > v = pair<pair<int, int>, pair<int, int> >(startPoint, endPoint);
					motionVect.insert(v);
				}
			}
		}
	}

	// Convert map of motion vectors to motion vector struct
	ConvertMapToStruct(motionVect, stMotionVect);

	cvReleaseImage(&velx);
	cvReleaseImage(&vely);
}

void MotionVectorExtractor::ConvertMapToStruct(map<pair<int, int>, pair<int, int> >& mapMotionVect, vector<stMotionVector>& stMotionVect)
{
	int y_comp, x_comp;
	double tempAngle;
	map<pair<int, int>, pair<int, int> >::iterator pMapMotion;
	pMapMotion = mapMotionVect.begin();

	while (pMapMotion != mapMotionVect.end()) {
		stMotionVector s;

		x_comp = pMapMotion->second.first - pMapMotion->first.first;
		y_comp = pMapMotion->first.second - pMapMotion->second.second;

		s.m_startPoint = pMapMotion->first;
		s.m_endPoint = pMapMotion->second;

		tempAngle = atan2(y_comp, x_comp) * 180 / PI;
		if (tempAngle < 0)
			s.m_nAngle = tempAngle + 360;
		else
			s.m_nAngle = tempAngle;

		s.m_dbSize = sqrt(pow(y_comp, 2) + pow(x_comp, 2));

		stMotionVect.push_back(s);

		pMapMotion++;
	}
}

/*
void MotionVectorExtractor::ConvertCFrameToIplImage(const CFrame * frame, IplImage * im)
{
for ( int i = 0; i < im->height; ++i ){
for ( int j = 0; j < im->width; ++j ){
int p = im->nChannels*(i*im->width + j);
for ( int k = 0; k < im->nChannels; ++k ){
im->imageData[p+k] = frame->GetComponent(j,i,k);
}
}
}
}
*/

void MotionVectorExtractor::AllocateOnDemand(IplImage **img, CvSize size, int depth, int channels)
{
	if (*img != NULL)	return;

	*img = cvCreateImage(size, depth, channels);
	if (*img == NULL)
	{
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}