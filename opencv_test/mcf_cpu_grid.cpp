#include <string>
#include <iostream>
#include <fstream>
#include <windows.h>
#include <exception>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <functional> 

#include "MotionVectorExtractor.h"
#include "MotionCooccurrenceHistogram.h"

//using namespace cv;
using namespace std;

/*
double PCFreq = 0.0;
__int64 CounterStart = 0;
const int BLOCKSIZE = 1024;
*/

/// FUNCTION PROTOTYPES
// Tsime execution functions
//void StartCounter();
//double GetCounter();

enum eClassifier{

	kNN,
	SVM
};

bool EstimateMotionVectors(const string& pathVideo, vector<vector<double> >& vecMotionVel, vector<vector<double> >& vecMotionAngle, vector<vector<pair<int, int> > >& vecMotionPosStart, vector<vector<pair<int, int> > >& vecMotionPosEnd);
bool ExtractGridBasedMCF(const string& pathVideo, int nGridSize, vector<vector<vector<double> > >& vecMCFHist, vector<int>& vec_nFrameID);
void DivideTrainTestSet(vector<vector<vector<double> > >& vecMCFHist, vector<int>& vec_nFrameID, int nTrainSetPer, vector<vector<vector<double> > >& vecMCFHistTrain, vector<int>& vec_nFrameIDTrain, vector<vector<vector<double> > >& vecMCFHistTest, vector<int>& vec_nFrameIDTest) ;
void CreateMotionModel(vector<vector<vector<double> > >& vecMCFHistTrain, vector<vector<double> >& vecMotionModel);
double MeasureHistDistance(const vector<double>& vecMCFHistTest, const vector<double>& vecMotionModel) ; 
void MinMaxNormalization(vector<double>& vHist, vector<double>& vHistNor) ;
void UpdateMotionModel(vector<double>& vecMotionModel, vector<double>& vecMCFHistTest, double dLearnRate) ;

int main(int argc, const char* argv[]) {

	try {

		/* Read video list*/
		int nStartFrame, nEndFrame, nTrainSetPer, nGridSize;
		double dLearnRate ;
		vector<vector<double> > vec_dVideoHistDistance;
		string strVidPath, strVidName;
		vector<string> vec_strVidPath;
		vector<int> vec_nStartFrame, vec_nEndFrame, vec_nFrameID, vec_nFrameIDTrain, vec_nFrameIDTest;
		vector<vector<vector<double> > > vecMCFHist, vecMCFHistTrain, vecMCFHistTest;
		vector<vector<double> > vecMotionModel;

		ifstream inVideoList("video_list.txt");
		ifstream ingt("gt.txt");
		ofstream distanceFile("distanceFile.txt");

		// Read reference video list
		while (getline(inVideoList, strVidPath)) {

			vec_strVidPath.push_back(strVidPath);
			cout << strVidPath << endl;
		}

		// Read ground truth for anomaly 
		while (ingt >> strVidName >> nStartFrame >> nEndFrame) {

			vec_nStartFrame.push_back(nStartFrame);
			vec_nEndFrame.push_back(nEndFrame);
			cout << nStartFrame << "\t" << nEndFrame << endl;
		}

		/* For each video in the video list */
		nTrainSetPer = 10; // The percentage of the train set
		dLearnRate = 0.1;
		nGridSize = 4; 
		
		for (int nVidIndex = 0; nVidIndex < vec_strVidPath.size(); nVidIndex++) {

			/* Extract MCF histogram for all frames in the video */
			ExtractGridBasedMCF(vec_strVidPath[nVidIndex], nGridSize, vecMCFHist, vec_nFrameID);

			/* Create motion model from %x percent of the frames */
			DivideTrainTestSet(vecMCFHist, vec_nFrameID, nTrainSetPer, vecMCFHistTrain, vec_nFrameIDTrain, vecMCFHistTest, vec_nFrameIDTest );
			CreateMotionModel(vecMCFHistTrain, vecMotionModel);

			/* For each of the other frames */
			double dHistDistance;
			for (int nFrameIndex = 0; nFrameIndex < vec_nFrameIDTest.size(); nFrameIndex++) {

				vector<double> vec_dFrameDistance;
 				for (unsigned int gridindex = 0; gridindex < vecMCFHistTest.size(); gridindex++) {

					/* Measure the distance between two histograms */
					dHistDistance = MeasureHistDistance(vecMCFHistTest[gridindex][nFrameIndex], vecMotionModel[gridindex]);
					cout << "Grid index / Distance = " << gridindex << " / " << dHistDistance << endl;
					vec_dFrameDistance.push_back(dHistDistance);

					/* Update motion model */
					UpdateMotionModel(vecMotionModel[gridindex], vecMCFHistTest[gridindex][nFrameIndex], dLearnRate);
				}
				/* Check anomaly (soft threshold) */
				/* Maybe a counter for anomaly */

				vec_dVideoHistDistance.push_back(vec_dFrameDistance);
			}

			distanceFile << vec_strVidPath[nVidIndex] << endl;
			for (unsigned int nTestFrameIndex = 0; nTestFrameIndex < vec_dVideoHistDistance.size(); nTestFrameIndex++) {

				distanceFile << vec_nFrameIDTest[nTestFrameIndex] << "\t";

				for (unsigned int gridindex = 0; gridindex < vecMCFHistTest.size(); gridindex++) {

					distanceFile << vec_dVideoHistDistance[nTestFrameIndex][gridindex] << "\t";
				}
				distanceFile << endl;
			}
			/* For each of the other frames (end) */
		}
		/* For each video in the list (end) */

		// Close the opened files
		distanceFile.close();
		ingt.close();
		inVideoList.close();

		return 0;
	}
	catch (exception& e) {

		cout << e.what() << endl;
	}

	return 0;
}

void DivideTrainTestSet(vector<vector<vector<double> > >& vecMCFHist, vector<int>& vec_nFrameID, int nTrainSetPer, vector<vector<vector<double> > >& vecMCFHistTrain, vector<int>& vec_nFrameIDTrain, vector<vector<vector<double> > >& vecMCFHistTest, vector<int>& vec_nFrameIDTest) {

	int nFeatureSize, nTrainSize, nTestSize ;
	vector<int> vec_nFrameIDTrain_Temp, vec_nFrameIDTest_Temp;
	vector<int>::iterator itFrame;

	nFeatureSize = vec_nFrameID.size() ;
	nTrainSize = nFeatureSize*nTrainSetPer / 100;
	nTestSize = nFeatureSize - nTrainSize ;

	cout << "Train / Test Size = " << nTrainSize << " / " << nTestSize << endl;

	// Divide grid samples at first
	vecMCFHistTrain.clear();
	vecMCFHistTest.clear();
	for (unsigned int gridindex = 0; gridindex < vecMCFHist.size(); gridindex++) {

		vector<vector<double> > vecMCFHistTrain_Temp, vecMCFHistTest_Temp;
		vector<vector<double> >::iterator itMCF;

		cout << "Sample size = " << vecMCFHist[gridindex].size() << endl;

		itMCF = vecMCFHist[gridindex].begin();
		cout << "kontrol 1" << endl;
		vecMCFHistTrain_Temp.assign(itMCF, itMCF + nTrainSize);
		cout << "kontrol 2" << endl;
		vecMCFHistTest_Temp.assign(itMCF + nTrainSize + 1, vecMCFHist[gridindex].end());

		cout << "Grid index / Train / Test Size = " << gridindex << " / " << vecMCFHistTrain_Temp.size() << " / " << vecMCFHistTest_Temp.size() << endl;

		vecMCFHistTrain.push_back(vecMCFHistTrain_Temp);
		vecMCFHistTest.push_back(vecMCFHistTest_Temp);
	}

	// Divide frame indexes 
	itFrame = vec_nFrameID.begin();
	vec_nFrameIDTrain_Temp.assign(itFrame, itFrame + nTrainSize);
	vec_nFrameIDTest_Temp.assign(itFrame + nTrainSize + 1, vec_nFrameID.end());

	vec_nFrameIDTrain = vec_nFrameIDTrain_Temp;
	vec_nFrameIDTest = vec_nFrameIDTest_Temp;
}

void CreateMotionModel(vector<vector<vector<double> > >& vecMCFHistTrain, vector<vector<double> >& vecMotionModel) {

	vecMotionModel.clear();

	for (unsigned int gridindex = 0; gridindex < vecMCFHistTrain.size(); gridindex++) {

		vector<double> vecMotionNorModel_Temp;
		vector<double> vecMotionModel_Temp(vecMCFHistTrain[gridindex][0].size(), 0);

		for (unsigned int i = 0; i < vecMCFHistTrain[gridindex].size(); i++) {

			transform(vecMotionModel_Temp.begin(), vecMotionModel_Temp.end(), vecMCFHistTrain[gridindex][i].begin(), vecMotionModel_Temp.begin(), std::plus<double>());
		}

		// Min-max normalization
		MinMaxNormalization(vecMotionModel_Temp, vecMotionNorModel_Temp);
		vecMotionModel.push_back(vecMotionNorModel_Temp);
	}
}

void MinMaxNormalization(vector<double>& vHist, vector<double>& vHistNor) {

	double cLower, cUpper, cScale;

	vHistNor = vHist ;

	cLower = *min_element(vHist.begin(), vHist.end());
	cUpper = *max_element(vHist.begin(), vHist.end());
	cScale = cUpper - cLower;

	vector<double> vec_min(vHist.size(), cLower);
	vector<double> vec_scale(vHist.size(), cScale);

	if (cScale != 0)
	{
		transform(vHistNor.begin(), vHistNor.end(), vec_min.begin(), vHistNor.begin(), std::minus<double>());
		transform(vHistNor.begin(), vHistNor.end(), vec_scale.begin(), vHistNor.begin(), std::divides<double>());
	}
}

double MeasureHistDistance(const vector<double>& vecMCFHistTest, const vector<double>& vecMotionModel){

	double dDist=0;

	// Measure Euclidean distance
	for (unsigned int i = 0; i < vecMCFHistTest.size(); i++) {

		dDist += pow((vecMCFHistTest[i] - vecMotionModel[i]), 2) ;
	}
	dDist = sqrt(dDist) ;

	return dDist ;
}

void UpdateMotionModel(vector<double>& vecMotionModel, vector<double>& vecMCFHistTest, double dLearnRate) {

	vector<double> vecMotionModelNew ;
	vector<double> vec_dModelTemp(vecMotionModel.size(), 0);

	for (unsigned int i = 0; i < vecMotionModel.size(); i++) {

		vec_dModelTemp[i] = (1 - dLearnRate)*vecMotionModel[i] + dLearnRate*vecMCFHistTest[i];
	}

	MinMaxNormalization(vec_dModelTemp, vecMotionModelNew);

	vecMotionModel = vecMotionModelNew;
}

bool EstimateMotionVectors(const string& pathVideo, vector<vector<double> >& vecMotionVel, vector<vector<double> >& vecMotionAngle, vector<vector<pair<int, int> > >& vecMotionPosStart, vector<vector<pair<int, int> > >& vecMotionPosEnd, vector<int>& vec_nFrameID) {

	MotionVectorExtractor mvextract;
	MotionVectorExtractor::eMotionMethod mMethod = MotionVectorExtractor::MOTION_OPTICAL_FLOW_BM;

	mvextract.Extract(pathVideo, mMethod, 0);
	mvextract.GetFeature(vecMotionVel, vecMotionAngle, vecMotionPosStart, vecMotionPosEnd, vec_nFrameID);
	
	return true;
}


bool ExtractGridBasedMCF(const string& pathVideo, int nGridSize, vector<vector<vector<double> > >& vecMCFHist, vector<int>& vec_nFrameID) {

	int nFrameSkip, nFrameHistory, nrepeatTime;
	double dAverageElapsedTime;

	nFrameSkip = 0;
	nFrameHistory = 10;
	nrepeatTime = 1;

	MotionCooccurrenceHistogram mcfextract;
	MotionCooccurrenceHistogram::eMotionCooccurrenceType mHistType = MotionCooccurrenceHistogram::MOTION_COOCCURRENCE_ANGLE;

	dAverageElapsedTime = 0;
	for (int i = 0; i < nrepeatTime; i++) {

		double dElapsedTime;

		mcfextract.Extract(pathVideo, mHistType, nFrameSkip, nFrameHistory, nGridSize);
		mcfextract.GetFeature(vecMCFHist, vec_nFrameID);
		dElapsedTime = mcfextract.GetElapsedTime();
		//cout << "Elapsed Time (in milli seconds) = " << dElapsedTime << endl;

		dAverageElapsedTime += dElapsedTime;
	}
	dAverageElapsedTime /= nrepeatTime;

	cout << "Average Elapsed Time = " << dAverageElapsedTime << endl;

	return true;
}

/*
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
*/