#pragma once
//#include <io.h>
#include <iostream>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/ml.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
namespace lbp
{
	class LBP {
		/************************************************
		** Calculate LBP of grayscale image.
		** Arguments:
		**   -- int neighbour: bit digits of patterns;
		**   -- int cellsize: pixel size of cells;
		**   -- int radius: radius of extended LBP (circular);
		************************************************/
	private:
		int neighbours, radius, cellSize, numPattern;
		std::vector<uchar> RITable, RThash;
		void findRI();
		void getHopCount(std::vector<uchar>&);
		Mat RILBPHistogramW(const Mat& dst, int);
	public:
		//LBP();
		LBP(int, int, int);
		/*Extended LBP (circular)*/
		void ELBP(const Mat&, Mat &, bool padding = true);
		/*Uniform Pattern*/
		void ULBP(const Mat&, Mat &);
		/*Rotation-invariant*/
		void RILBP(const Mat&, Mat &);

		Mat RILBPHistogram(const Mat& dst);
		Mat RILBPHistogramT(const Mat& dst, int sub = 2);
		//Mat RILBPHistogramS(const Mat &dst, int, int);  // With sliding window. Not implemented;
		int getNumPattern()const;
	};

	class LBPSVM {
		/*****************************************
		** SVM classifier based on LBP histogram
		** Arguments that need to be specified:
		**   -- directories of datasets
		**   -- parameters of SVM classifier
		*****************************************/
	private:
		int csize, nblocks, radius;
		Ptr<ml::SVM> SVM;
		std::vector<const char*> dirs;
		std::vector<int> FN;
		char *ModelDir;
		void LoadSample(Mat&, std::vector<int>&, int);
		void randArrange01(std::vector<int>&);
		lbp::LBP extractor;
		Mat EigenMat;
	public:
		LBPSVM(int cs,
			int R,
			int sub = 1,
			std::vector<int> fileNums = { 551, 831, 172, 172 },
			const char* MD = "../models/svm/model",
			ml::SVM::Types type = ml::SVM::C_SVC,
			ml::SVM::KernelTypes kernel_type = ml::SVM::POLY,
			TermCriteria term_crit = TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6),
			double _c = 1.0,
			double coef0 = 0.1, //constant efficient
			double degree = 3.0,
			double gamma = 1.0,
			const char* TrainingPositive = "./lib81_1/smoke/",
			const char* TrainingNegative = "./lib81_1/non_smoke/",
			const char* TestingPositive = "./datasets/test/smoke/",
			const char* TestingNegative = "./datasets/test/non_smoke/");
		void Train();
		float Validate(const char* model = "./models/model.xml");  // model dir
		const char* getMD() const;
		int Predict(const char*, const char* model = "./models/model.xml");  // image filename and model filename
		float Predict(const Mat &);  // image
		std::vector<int> Predb(const Mat&);  // histogram of images
		std::vector<cv::Rect> PredictRec(const char*, const char* model = "./models/model.xml");
		void getEigen();
		~LBPSVM();
	};
}