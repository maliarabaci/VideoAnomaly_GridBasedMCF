/*!
*************************************************************************************
* \file MotionVectorExtractor.h
*
* \brief
*    Motion vector extractor for the given video
*
* \author
*    - Mehmet Ali Arabacı             <mehmet.arabaci@uzay.tubitak.gov.tr>
*************************************************************************************
*/

#ifndef MotionVectorExtractor_H_
#define MotionVectorExtractor_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/gpu/gpu.hpp>

#include <utility>

using namespace cv;
using namespace std;

/**Returns motion vectors for the given video
*
* Motion vector extraction for the given video by using a couple
* methods(Optical Flow Lucas Canade, Optical Flow Farneback and Optical Flow BM)
*
* \class MotionVectorExtractor MotionVectorExtractor.h
*
*/
class MotionVectorExtractor
{
public:
	MotionVectorExtractor();
	virtual ~MotionVectorExtractor();

	// Motion vector method list
	enum eMotionMethod
	{
		MOTION_OPTICAL_FLOW_LC,
		MOTION_OPTICAL_FLOW_FARNEBACK,
		MOTION_OPTICAL_FLOW_BM,
	};

	// Motion vector struct
	struct stMotionVector {
		// Start point of the vector
		pair<int, int> m_startPoint;
		// End point
		pair<int, int> m_endPoint;
		// Angle of the vector according to the x-axis
		double m_nAngle;
		// Length of the vector
		double m_dbSize;
	};

	/// Extracts motion vector features for a given CVideo object within defined time interval.
	/*!
	* In this function frame skip value means that how frequently motion vectors will be extracted. For example, "0" means motion
	* vectors will be extracted from consequtive frames with no frame skip. "10" means motion vectors will be extracted from
	* consequtive frames, but 10 frames will be skipped for each step.
	*
	* \param	mMethod			Motion vector extraction method
	* \param	nFrameSkip		Frame skip value
	*/
	bool Extract(const string& path_video, eMotionMethod mMethod = MOTION_OPTICAL_FLOW_BM, int nFrameSkip = 0);

	/// Function to reach the extracted descriptor for CVideo input.
	/*!
	* \param output vector of motion vectors
	*/
	void GetFeature(vector<vector<double> > &vecoutMotionVel, vector<vector<double> > &vecoutMotionAngle, vector<vector<pair<int, int> > > &vecoutMotionPosStart, vector<vector<pair<int, int> > > &vecoutMotionPosEnd, vector<int>& vec_nFrameID) const;

private:

	// Holds motion vector method
	eMotionMethod motionMethod;

	// Video motion vector structs that holds start point, vector size etc.
	//vector<vector<stMotionVector> > vidMotionVectors;
	vector<vector<double> > vecMotionVel;
	vector<vector<double> > vecMotionAngle;
	vector<vector<pair<int, int> > > vecMotionPosStart;
	vector<vector<pair<int, int> > > vecMotionPosEnd;
	vector<int> vec_nFrameID;


	// Frame motion vector structs that holds start point, vector size etc.
	vector<stMotionVector> frameMotionVectors;

	/// Extracts motion vector features by using the current and the previous frame.
	/*!
	* \param  	curFrame  		Current frame
	* \param  	preFrame		Previous frame
	* \param  	mMethod			Motion extraction method
	* \param	stMotionVect	Vector of motion vectors
	*/
	void ExtractMotionVectors(const IplImage* curFrame, const IplImage* preFrame, MotionVectorExtractor::eMotionMethod mMethod, vector<double>& vecFrameMotionVel, vector<double>& vecFrameMotionAngle, vector<pair<int, int> >& vecFrameMotionPosStart, vector<pair<int, int> >& vecFrameMotionPosEnd);

	/// Extracts motion vector features with respect to Optical Flow Lucas Canade
	/*!
	* \param  curFrame  		Current frame
	* \param  preFrame			Previous frame
	*/
	void OpticalFlow_Lucas_Kanade(const IplImage* curFrame, const IplImage* preFrame, vector<stMotionVector>& stMotionVect);

	/// Extracts motion vector features with respect to Optical Flow Farneback
	/*!
	* \param  curFrame  		Current frame
	* \param  preFrame			Previous frame
	*/
	void OpticalFlow_Farneback(const IplImage* curFrame, const IplImage* preFrame, vector<stMotionVector>& stMotionVect);

	/// Extracts motion vector features with respect to Optical Flow Block Matching
	/*!
	* \param  curFrame  		Current frame
	* \param  preFrame			Previous frame
	*/
	void OpticalFlow_BM(const IplImage* curFrame, const IplImage* preFrame, vector<stMotionVector>& stMotionVect);

	/// Converts map of motion vector pairs to the vector of motion structs that contain start point, end point, angle and vector size
	/*!
	* \param  mapMotionVect	Map of motion vector pairs
	* \param  stMotionVect		Vector of motion vector structs
	*/
	void ConvertMapToStruct(map<pair<int, int>, pair<int, int> >& mapMotionVect, vector<stMotionVector>& stMotionVect);

	void AllocateOnDemand(IplImage **img, CvSize size, int depth, int channels);
};


#endif /* MOTIONVECTOREXTRACTOR_H_ */
