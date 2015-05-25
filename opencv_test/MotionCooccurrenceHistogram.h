/*!
*************************************************************************************
* \file MotionCooccurrenceHistogram.h
*
* \brief
*    Motion cooccurrence histogram calculator for the given video
*
* \author
*    - Mehmet Ali Arabacı             <mehmet.arabaci@uzay.tubitak.gov.tr>
*************************************************************************************
*/

#ifndef MOTIONCOOCCURRENCEHISTOGRAM_H_
#define MOTIONCOOCCURRENCEHISTOGRAM_H_

#include "MotionVectorExtractor.h"

#include <utility>
#include <vector>

/**Returns motion cooccurrence histograms for the given video
*
* Motion cooccurrence histogram extraction of motion angle or velocity for the given video.
*
* \class MotionCooccurrenceHistogram MotionCooccurrenceHistogram.h
*
*/
class MotionCooccurrenceHistogram
{
public:
	MotionCooccurrenceHistogram();
	//	MotionCooccurrenceHistogram( int angleQuantLevel, int velocityQuantLevel, int divider );
	virtual ~MotionCooccurrenceHistogram();

	enum eMotionCooccurrenceType
	{
		MOTION_COOCCURRENCE_ANGLE,
		MOTION_COOCCURRENCE_VELOCITY,
		MOTION_COOCCURRENCE_ANGLE_VELOCITY
	};

	/// Extracts motion cooccurrence histograms for a given CVideo object within defined time interval.
	/*!
	* In this function frame skip value means that how frequently MCF will be extracted. For example, "0" means
	* MCF will be extracted for consequtive frames wrt the given frame history value. On the other hand, "10" means
	* MCF will be extracted for the frames which are 10 frames apart between each other.
	*
	* \param  	video_path		Video file path
	* \param	cType			Motion co-occurrence histogram type ( angle, velocity or both )
	* \param	nFrameSkip		Frame skip value ( default=0 )
	* \param	nFrameHistory	Frame history value for temporal co-occurrence calculation ( default=0 )
	*/
	void Extract(const string& video_path, eMotionCooccurrenceType cType, int nFrameSkip = 0, int nFrameHistory = 0);

	/// Initialize MCF parameters to their default values
	/*!
	*
	*/
	void Initialize();

	/// Function to reach the extracted descriptor.
	/*!
	* \param	output	Vector of motion co-occurrence histograms
	*/
	void GetFeature(vector<vector<double> > &output, vector<int>& vec_noutFrameID) const;

	/// Function to reach the elapsed time for MCF extraction
	/*!
	* 
	*/
	double GetElapsedTime();

	/// Function to set the parameters
	/*!
	* Angle and velocity quantization levels, spatial window divider and minimum vector number for MCF extraction
	*
	* \param	angleQuantLevel		Angle quantization level
	* \param	velocityQuantLevel	Velocity quantization level
	* \param 	divider				Diagonal length divider to determine spatial window size
	* \param 	vectNumThreshold	Minimum number of the motion vectors for MCF extraction
	*/
	void SetParameters(int angleQuantLevel, int velocityQuantLevel, int divider, int vectNumThreshold);

private:

	// Diagonal length
	double m_dDiagLength;

	// Motion cooccurrence histogram for video analysis
	vector<vector<double> > m_vMCH_Video;


	vector<int> m_vFrameID;

	// Motion cooccurrence histogram for frame analysis
	vector<double> m_vMCH_Frame;

	// Position and quantized angle vector( size of the vector = frame history value )
	vector<map<pair<int, int>, int> > m_vPositionAngle;

	// Position and quantized velocity vector( size of the vector = frame history value )
	vector<map<pair<int, int>, int> > m_vPositionVelocity;

	// Angle quantization level
	unsigned int m_nAngleBinSize;

	// Velocity quantization level
	unsigned int m_nVelocityBinSize;

	// Window size
	unsigned int m_nWinSizeDivider;

	// Frame history size for cooccurrence calculation
	unsigned int m_nFrameHistory;

	//
	int m_nNofVectorThreshold;

	//
	double dElapsedTime;

	/// Motion angle cooccurrence histogram calculation
	/*!
	* \param  motionVectors	Given motion vectors
	* \param  nFrameSkip		Frame skip value
	* \param  motionAngleHist	Motion angle histogram
	*/
	void MotionAngleCooccurrence(vector<map<pair<int, int>, int> > &vPositionAngle, int nFrameSkip, vector<vector<double> > &vMotionAngleHist);

	/// Quantize the given motion angle to one of the specified 8-bins.
	//  Full rotation is equally divided into 8-bins from 0 to 7 respectively.
	/*!
	* \param  	angle	Given angle
	* \returns Motion angle histogram bin
	*/
	int QuantizeMotionAngle(double angle);

	/// Motion velocity cooccurrence histogram calculation
	/*!
	* \param  motionVectors	Given motion vectors
	* \param  nFrameSkip		Frame skip value
	* \param  motionAngleHist	Motion velocity histogram
	*/
	void MotionVelocityCooccurrence(vector<map<pair<int, int>, int> > &vPositionVelocity, int nFrameSkip, vector<vector<double> > &vMotionVelocityHist);

	/// Quantize the given motion velocity to one of the specified 8-bins.
	//
	/*!
	* \param  	vectSize	Motion vector size
	* \param	unitTime	Time value between two frames used for motion vector calculation
	* \returns Motion velocity histogram bin
	*/
	int QuantizeMotionVelocity(double  vectMag, const double unitTime);

	/// Calculate Euclidean distance between the given pixel pairs
	/*!
	* \param	p1	First pixel position
	* \param	p2	Second pixel position
	* \returns	Euclidean distance between two pixels
	*/
	double EuclideanDistanceBetweenPairs(const pair<int, int> p1, const pair<int, int> p2);

	/// Normalize hitogram
	/*!
	* \param	vHist		Histogram before normalization
	* \param	vHistNor	Normalized histogram
	*/
	void NormalizeHistogram(vector<vector<double> > &vHist, vector<vector<double> > &vHistNor);

};


#endif /* MOTIONCOOCCURRENCEHISTOGRAM_H_ */
