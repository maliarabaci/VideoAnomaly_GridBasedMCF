/*
* \file MotionCooccurrenceHistogram.cpp
*
* \brief
*    Implementation of MotionCooccurrenceHistogram class.
*
* \author
*    Mehmet Ali Arabacı
*
* Created on: Dec 29, 2012
*/

#include "MotionCooccurrenceHistogram.h"

#include <numeric> 
#include <algorithm>    // std::transform
#include <functional> 
#include <windows.h>
#include <fstream>

double PCFreq = 0.0;
__int64 CounterStart = 0;
const int BLOCKSIZE = 1024;

MotionCooccurrenceHistogram::MotionCooccurrenceHistogram()
{
	Initialize();
}

MotionCooccurrenceHistogram::~MotionCooccurrenceHistogram(){

}

void MotionCooccurrenceHistogram::Initialize()
{
	m_vPositionAngle.clear();
	m_vPositionVelocity.clear();

	m_nAngleBinSize = 8;
	m_nVelocityBinSize = 8;
	m_nWinSizeDivider = 7;
	m_nFrameHistory = 10;
	m_nNofVectorThreshold = 1;
}

void StartCounter()
{
	LARGE_INTEGER li;
	if (!QueryPerformanceFrequency(&li))
		std::cout << "QueryPerformanceFrequency failed!\n";

	PCFreq = double(li.QuadPart) / 1000.0;

	QueryPerformanceCounter(&li);
	CounterStart = li.QuadPart;
}

double GetCounter()
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart - CounterStart) / PCFreq;
}

void MotionCooccurrenceHistogram::Extract(const string& video_path, MotionCooccurrenceHistogram::eMotionCooccurrenceType cType, int nFrameSkip, int nFrameHistory)
{
	vector<vector<double> > vHistNor;
	vector<vector<double> > vecMotionVel, vecMotionAngle;
	vector<vector<pair<int, int> > > vecMotionPosStart, vecMotionPosEnd;
	vector<int> vec_nFrameID;

	// Frame history value
	m_nFrameHistory = nFrameHistory;

	// Define the length of diagonal
	m_dDiagLength = 400;

	MotionVectorExtractor m;
	// Extract motion vector with respect to the given parameters
	m.Extract(video_path, MotionVectorExtractor::MOTION_OPTICAL_FLOW_BM, 0);
	// Get motion vectors to calculate histograms
	m.GetFeature(vecMotionVel, vecMotionAngle, vecMotionPosStart, vecMotionPosEnd, vec_nFrameID);

	/*
	// Write motion vectors to the file
	ofstream outMotionVel("outMotionVel.txt", iostream::app);
	ofstream outMotionAngle("outMotionAngle.txt", iostream::app);
	for (unsigned int i = 0; i < vecMotionVel.size(); i++) {

		for (unsigned int j = 0; j < vecMotionVel[i].size(); j++) {
			
			outMotionVel << vecMotionVel[i][j] << endl;
			outMotionAngle << vecMotionAngle[i][j] << endl;
		}
	}
	outMotionVel.close();
	outMotionAngle.close();
	*/
	StartCounter();

	if (cType == MotionCooccurrenceHistogram::MOTION_COOCCURRENCE_ANGLE)
	{
		vector<map<pair<int, int>, int> > vPositionAngle; // Holds position and quantized angle bin for each frame
		vector<vector<double> > vHistTemp;
		int nAngleBin;

		for (unsigned int f = 0; f<vecMotionAngle.size(); f++)
		{
			map<pair<int, int>, int> mPositionAngleTemp;

			for (unsigned int m = 0; m<vecMotionAngle[f].size(); m++)
			{
				nAngleBin = QuantizeMotionAngle(vecMotionAngle[f][m]);
				mPositionAngleTemp.insert(pair<pair<int, int>, int>(vecMotionPosStart[f][m], nAngleBin));
			}

			vPositionAngle.push_back(mPositionAngleTemp);
		}
		MotionAngleCooccurrence(vPositionAngle, nFrameSkip, vHistTemp);
		NormalizeHistogram(vHistTemp, vHistNor);
	}
	else if (cType == MotionCooccurrenceHistogram::MOTION_COOCCURRENCE_VELOCITY)
	{
		vector<map<pair<int, int>, int> > vPositionVelocity; // Holds position and quantized angle bin for each frame
		vector<vector<double> > vHistTemp;
		int nVelocityBin;
		double unitTime;

		unitTime = 0.04;

		for (unsigned int f = 0; f<vecMotionVel.size(); f++)
		{
			map<pair<int, int>, int> mPositionVelocityTemp;

			for (unsigned int m = 0; m<vecMotionVel[f].size(); m++)
			{
				nVelocityBin = QuantizeMotionVelocity(vecMotionVel[f][m], unitTime);
				mPositionVelocityTemp.insert(pair<pair<int, int>, int>(vecMotionPosStart[f][m], nVelocityBin));
			}

			vPositionVelocity.push_back(mPositionVelocityTemp);
		}

		MotionVelocityCooccurrence(vPositionVelocity, nFrameSkip, vHistTemp);
		//vHistNor = vHistTemp;
		NormalizeHistogram(vHistTemp, vHistNor);
	}
	else if (cType == MotionCooccurrenceHistogram::MOTION_COOCCURRENCE_ANGLE_VELOCITY)
	{
		vector<vector<double> > vAngleHistTemp, vVelocityHistTemp;
		vector<vector<double> > vAngleHistNor, vVelocityHistNor;
		vector<map<pair<int, int>, int> > vPositionAngle, vPositionVelocity;
		int nAngleBin, nVelocityBin;
		double unitTime;

		unitTime = 0.04;

		// Quantize angle and velocity
		for (unsigned int f = 0; f<vecMotionAngle.size(); f++)
		{
			map<pair<int, int>, int> mPositionAngleTemp, mPositionVelocityTemp;

			for (unsigned int m = 0; m<vecMotionAngle[f].size(); m++)
			{
				nAngleBin = QuantizeMotionAngle(vecMotionAngle[f][m]);
				mPositionAngleTemp.insert(pair<pair<int, int>, int>(vecMotionPosStart[f][m], nAngleBin));

				nVelocityBin = QuantizeMotionVelocity(vecMotionVel[f][m], unitTime);
				mPositionVelocityTemp.insert(pair<pair<int, int>, int>(vecMotionPosStart[f][m], nVelocityBin));
			}

			vPositionAngle.push_back(mPositionAngleTemp);
			vPositionVelocity.push_back(mPositionVelocityTemp);
		}

		// Extract MCFs wrt position-angle and position-velocity
		MotionAngleCooccurrence(vPositionAngle, nFrameSkip, vAngleHistTemp);
		//vAngleHistNor = vAngleHistTemp;
		NormalizeHistogram(vAngleHistTemp, vAngleHistNor);

		MotionVelocityCooccurrence(vPositionVelocity, nFrameSkip, vVelocityHistTemp);
		//vVelocityHistNor = vVelocityHistTemp;
		NormalizeHistogram(vVelocityHistTemp, vVelocityHistNor);

		for (unsigned int j = 0; j<vAngleHistNor.size(); j++)
		{
			vector<double> vFeature;

			vFeature = vAngleHistNor[j];
			vFeature.insert(vFeature.end(), vVelocityHistNor[j].begin(), vVelocityHistNor[j].end());

			vHistNor.push_back(vFeature);
		}
	}

	m_vMCH_Video = vHistNor;
	m_vFrameID = vec_nFrameID;

	dElapsedTime = GetCounter();
}

void MotionCooccurrenceHistogram::GetFeature(vector<vector<double> > &output, vector<int>& vec_noutFrameID) const
{
	output = m_vMCH_Video;
	vec_noutFrameID = m_vFrameID;
}

void MotionCooccurrenceHistogram::MotionAngleCooccurrence(vector<map<pair<int, int>, int> > &vPositionAngle, int nFrameSkip, vector<vector<double> > &vMotionAngleHist)
{
	vector<vector<double> > vMotionAngleHistTemp;
	vector<double> vecFeature;
	double weightFactor;

	// Maximum distance for cooccurence histogram calculation
	//int maxDist = m_dDiagLength / m_nWinSizeDivider;
	int maxDist = 4;

	map<pair<int, int>, int>::iterator itPosAngleFirst, itPosAngleSecond;

	cout << "Extracting motion angle histogram" << endl;

	// Angle based motion cooccurrence histogram calculation
	//vec_nUpdatedFrameID.clear();
	for (unsigned int i = 0; i<vPositionAngle.size(); i = i + nFrameSkip + 1)
	{

		cout << i << endl;
		if (i < m_nFrameHistory) {

			vecFeature = vector<double>(m_nAngleBinSize*m_nAngleBinSize, 0);
		}
		else {

			vector<vector<double> > matAngleCooccurrence;
			vector<double> vecAngleRow(m_nAngleBinSize, 0);

			vecFeature.clear();

			for (unsigned int rindex = 0; rindex < m_nAngleBinSize; rindex++) {

				matAngleCooccurrence.push_back(vecAngleRow);
			}

			if (vPositionAngle[i].size() > m_nNofVectorThreshold)
			{
				for (int refFrameIndex = m_nFrameHistory; refFrameIndex >= 0; refFrameIndex--)
				{
					itPosAngleFirst = vPositionAngle[i - refFrameIndex].begin();
					while (itPosAngleFirst != vPositionAngle[i - refFrameIndex].end())
					{
						for (int searchFrameIndex = m_nFrameHistory; searchFrameIndex >= 0; searchFrameIndex--)
						{
							int frameDiff = abs(refFrameIndex - searchFrameIndex);

							itPosAngleSecond = vPositionAngle[i - searchFrameIndex].begin();

							while (itPosAngleSecond != vPositionAngle[i - searchFrameIndex].end())
							{
								if (itPosAngleFirst != itPosAngleSecond)
								{
									double motionDist = EuclideanDistanceBetweenPairs(itPosAngleFirst->first, itPosAngleSecond->first);

									//if (motionDist < maxDist)
									{
										// Use temporal weight factor for the motion vectors on different frames and
										// spatial weight factor for the motion vectors on the same frame
										//if (frameDiff != 0)
										//	weightFactor = (double)((m_nFrameHistory + 1) - frameDiff) / (m_nFrameHistory + 1);
										//else
										//	weightFactor = (double)((maxDist - motionDist) / maxDist);

										matAngleCooccurrence[itPosAngleFirst->second][itPosAngleSecond->second] += 1;
									}
								}
								itPosAngleSecond++;
							}
						}
						itPosAngleFirst++;
					}
				}
			}

			for (unsigned int rindex = 0; rindex < m_nAngleBinSize; rindex++) {

				vecFeature.insert(vecFeature.end(), matAngleCooccurrence[rindex].begin(), matAngleCooccurrence[rindex].end());
			}

			//vMotionAngleHistTemp.push_back(vecFeature);
			//vec_nUpdatedFrameID.push_back(vec_nFrameID[i]);
		}
		vMotionAngleHistTemp.push_back(vecFeature);
	}

	vMotionAngleHist = vMotionAngleHistTemp;
}

int MotionCooccurrenceHistogram::QuantizeMotionAngle(double angle)
{
	int directionBin, unitAngle;

	unitAngle = 360 / m_nAngleBinSize;

	directionBin = angle / unitAngle;

	//if ((angle < (unitAngle / 2)) || (angle >= (360 - (unitAngle / 2))))
	//	directionBin = 0;
	//else
	//	directionBin = (angle - (unitAngle / 2)) / unitAngle + 1;

	if (directionBin > m_nAngleBinSize - 1)
		directionBin = m_nAngleBinSize - 1;

	return directionBin;
}


void MotionCooccurrenceHistogram::MotionVelocityCooccurrence(vector<map<pair<int, int>, int> > &vPositionVelocity, int nFrameSkip, vector<vector<double> > &vMotionVelocityHist)
{
	double weightFactor;
	vector<vector<double> > vMotionVelocityHistTemp;

	int maxDist = m_dDiagLength / m_nWinSizeDivider;

	map<pair<int, int>, int>::iterator itPosVelocityFirst, itPosVelocitySecond;

	cout << "Extracting motion velocity histogram" << endl;
	// Velocity based motion cooccurrence histogram calculation
	
	for (unsigned int i = 0; i<vPositionVelocity.size(); i = i + nFrameSkip + 1)
	{
		vector<vector<double> > matMotionVelocity;
		vector<double> vecFeature;
		vector<double> vecMotionRow(m_nVelocityBinSize, 0);

		//cout << i << endl;

		if (i < m_nFrameHistory){

			vecFeature = vector<double>(m_nVelocityBinSize*m_nVelocityBinSize, 0);
		}
		else {
			for (unsigned int rindex = 0; rindex < m_nVelocityBinSize; rindex++) {

				matMotionVelocity.push_back(vecMotionRow);
			}

			if (vPositionVelocity[i].size() > m_nNofVectorThreshold)
			{
				for (int refFrameIndex = m_nFrameHistory; refFrameIndex >= 0; refFrameIndex--)
				{
					itPosVelocityFirst = vPositionVelocity[i - refFrameIndex].begin();
					while (itPosVelocityFirst != vPositionVelocity[i - refFrameIndex].end())
					{
						for (int searchFrameIndex = m_nFrameHistory; searchFrameIndex >= 0; searchFrameIndex--)
						{
							int frameDiff = abs(refFrameIndex - searchFrameIndex);

							itPosVelocitySecond = vPositionVelocity[i - searchFrameIndex].begin();

							while (itPosVelocitySecond != vPositionVelocity[i - searchFrameIndex].end())
							{
								if( itPosVelocityFirst != itPosVelocitySecond )
								{
									//double motionDist = EuclideanDistanceBetweenPairs(itPosVelocityFirst->first, itPosVelocitySecond->first);

									//if (motionDist < maxDist)
									{
										// Use temporal weight factor for the motion vectors on different frames and
										// spatial weight factor for the motion vectors on the same frame
										//if (frameDiff != 0)
										//	weightFactor = (double)((m_nFrameHistory + 1) - frameDiff) / (m_nFrameHistory + 1);
										//else
										//	weightFactor = (double)((maxDist - motionDist) / maxDist);

										matMotionVelocity[itPosVelocityFirst->second][itPosVelocitySecond->second] += 1;
									}
								}
								itPosVelocitySecond++;
							}
						}
						itPosVelocityFirst++;
					}
				}
			}

			for (unsigned int rindex = 0; rindex < m_nVelocityBinSize; rindex++) {

				vecFeature.insert(vecFeature.end(), matMotionVelocity[rindex].begin(), matMotionVelocity[rindex].end());
			}

			//vMotionVelocityHistTemp.push_back(vecFeature);
		}
		vMotionVelocityHistTemp.push_back(vecFeature);
	}

	vMotionVelocityHist = vMotionVelocityHistTemp;
}

int MotionCooccurrenceHistogram::QuantizeMotionVelocity(double vectMag, const double unitTime)
{
	int velocityBin;

	double velocity = vectMag / unitTime;

	if (velocity == 0)
	{
		velocityBin = 0;
	}
	else
	{
		double binSize = (9 * m_dDiagLength) / (19 * m_nVelocityBinSize);
		velocityBin = velocity / binSize;
		if (velocityBin > (m_nVelocityBinSize - 1))
			velocityBin = m_nVelocityBinSize - 1;
	}

	return velocityBin;
}

double MotionCooccurrenceHistogram::EuclideanDistanceBetweenPairs(const pair<int, int> p1, const pair<int, int> p2)
{
	return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
}

void MotionCooccurrenceHistogram::SetParameters(int angleQuantLevel, int velocityQuantLevel, int divider, int vectNumThreshold)
{
	m_nAngleBinSize = angleQuantLevel;
	m_nVelocityBinSize = velocityQuantLevel;
	m_nWinSizeDivider = divider;
	m_nNofVectorThreshold = vectNumThreshold;
}


void MotionCooccurrenceHistogram::NormalizeHistogram(vector<vector<double> > &vHist, vector<vector<double> > &vHistNor)
{
	vector<vector<double> > vHistNorTemp;
	double histSum = 0;

	for (unsigned int i = 0; i<vHist.size(); i++)
	{
		double cLower, cUpper, cScale;
		vector<double> vHistTemp = vHist[i];

		// Min max norm
		cLower = *min_element(vHistTemp.begin(), vHistTemp.end());
		cUpper = *max_element(vHistTemp.begin(), vHistTemp.end());
		cScale = cUpper - cLower; 

		vector<double> vec_min(vHistTemp.size(), cLower);
		vector<double> vec_scale(vHistTemp.size(), cScale);

		if (cScale != 0)
		{
			transform(vHistTemp.begin(), vHistTemp.end(), vec_min.begin(), vHistTemp.begin(), std::minus<double>());
			transform(vHistTemp.begin(), vHistTemp.end(), vec_scale.begin(), vHistTemp.begin(), std::divides<double>());
		}

		/*
		// L-2 Norm
		UStatistics stats( vHistTemp );
		double histSquareSum = stats.SquareSum();

		if( histSquareSum != 0 )
		{
		for( unsigned int j=0; j<vHistTemp.size(); j++ )
		{
		vHistTemp[j] = sqrt( vHistTemp[j]/histSquareSum );
		}
		}
		*/

		/*
		// Divide max normalization
		double nHistMaxValue;

		UExtrema<double> extremaHist( vHistTemp );
		nHistMaxValue = extremaHist.Max();

		if( nHistMaxValue != 0 )
		{
		vHistTemp[j] = vHistTemp[j]/nHistMaxValue;
		}
		*/

		vHistNorTemp.push_back(vHistTemp);
	}

	vHistNor = vHistNorTemp;
}

double MotionCooccurrenceHistogram::GetElapsedTime() {

	return dElapsedTime;
}