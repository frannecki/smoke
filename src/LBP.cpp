#include "LBP.h"
namespace lbp{
    int LBP::getNumPattern()const { return numPattern; }

	//LBP::LBP() { neighbours = 8; radius = 0; cellSize = 10; } // default parameters
	LBP::LBP(int n = 8, int r = 0, int c = 10) {
		neighbours = n; radius = r;
		cellSize = c;
		findRI();
	}

	/*find the number of times of binary hopping*/
	void LBP::getHopCount(std::vector<uchar> &table) {
		uchar max = pow(2, neighbours) - 1;
		std::vector<uchar> str(neighbours);
		uchar hcount = 0, idx = 1, count;
		for (count = 0; count <= max; ++count) {
			uchar i = (uchar)count;
			for (int j = 0; j < neighbours; ++j) {
				str[j] = i / pow(2, neighbours - 1 - j);
				i -= str[j] * pow(2, neighbours - 1 - j);
			}
			hcount = 0;
			for (int j = 0; j < neighbours; ++j) {
				if (str[(j + 1) % neighbours] != str[j])
					hcount++;
			}
			if (hcount <= 2)
				table[i] = idx++;
			if (count == max)  break;
		}
	}

	/*find the minimal binary integer with the same rotation pattern*/
	void LBP::findRI() {
		RITable.assign(pow(2, neighbours), 0);
		for (uchar i = 0; i < RITable.size(); ++i) {
			/*Calculate the rotation invariant minimal pattern*/
			uchar min = i, k = i;
			for (int j = 0; j < neighbours - 1; ++j) {
				k = (k << 1 | k >> (neighbours - 1));
				if (k < min)
					min = k;
			}
			RITable[i] = min;
			if (i == RITable.size() - 1)  break;
		}
		numPattern = 0;
		RThash.assign(pow(2, neighbours), -1);
		for (int i = 0; i < RITable.size(); ++i) {
			if (RThash[RITable[i]] == 255) {
				RThash[RITable[i]] = numPattern++;
			}
		}
	}

	void LBP::ELBP(const Mat &src, Mat &dst, bool padding) {
		int div = radius;
		if (padding)  div = 0;
		dst = Mat(Size(src.cols - 2 * div, src.rows - 2 * div), CV_8UC1, Scalar(0));
		for (int n = 0; n < neighbours; n++) {
			// deviation of sample points
			float x = static_cast<float>(-radius * sin(2.0*CV_PI*n / static_cast<float>(neighbours)));
			float y = static_cast<float>(radius * cos(2.0*CV_PI*n / static_cast<float>(neighbours)));
			// floor and ceil
			int fx = static_cast<int>(floor(x));
			int fy = static_cast<int>(floor(y));
			int cx = static_cast<int>(ceil(x));
			int cy = static_cast<int>(ceil(y));
			// fractional part
			float ty = y - fy;
			float tx = x - fx;
			// interpolation weights
			float w1 = (1 - tx) * (1 - ty);
			float w2 = tx  * (1 - ty);
			float w3 = (1 - tx) *      ty;
			float w4 = tx  *      ty;

			for (int i = radius; i < src.rows - radius; i++)
			{
				for (int j = radius; j < src.cols - radius; j++)
				{
					// calculate interpolation
					float t = static_cast<float>(w1*src.at<uchar>(i + fy, j + fx) + w2*src.at<uchar>(i + fy, j + cx)
						+ w3*src.at<uchar>(i + cy, j + fx) + w4*src.at<uchar>(i + cy, j + cx));

					if ((t > src.at<uchar>(i, j)) || (std::abs(t - src.at<uchar>(i, j)) < std::numeric_limits<float>::epsilon()))
						dst.at<uchar>(i - div, j - div) += 1 << n;
				}
			}
		}
	}

	void LBP::RILBP(const Mat &src, Mat &dst) {
		ELBP(src, dst);
		for (int i = 0; i < dst.rows; ++i) {
			for (int j = 0; j < dst.cols; ++j) {
				dst.at<uchar>(i, j) = RITable[dst.at<uchar>(i, j)];
			}
		}
	}

	void LBP::ULBP(const Mat &src, Mat &dst) {
		ELBP(src, dst);
		std::vector<uchar> table(pow(2, neighbours), 0);
		getHopCount(table);
		for (int i = 0; i < dst.rows; ++i) {
			for (int j = 0; j < dst.cols; ++j) {
				dst.at<uchar>(i, j) = table[dst.at<uchar>(i, j)];
			}
		}
	}

	Mat LBP::RILBPHistogramW(const Mat& dst, int cs){
		int numCellX = dst.cols / cs;
		int numCellY = dst.rows / cs;
		Mat RILBPfeature(1, numPattern*numCellX*numCellY, CV_32FC1, Scalar(0.0));
		float *dt = RILBPfeature.ptr<float>(0);
		int idx = 0;
		Mat cell; // corresponding region of rotation-invariant LBP image.
		for (int y = 0; y < numCellY; ++y) {
			for (int x = 0; x < numCellX; ++x) {
				cell = dst(Rect(x*cs, y*cs, cs, cs));
				for (int i = 0; i < cs; ++i) {
					for (int j = 0; j < cs; ++j) {
						dt[idx + RThash[cell.at<uchar>(i, j)]] += 1.0;
					}
				}
				idx += numPattern;
			}
		}
		for (int j = 0; j < RILBPfeature.cols; ++j) {
			dt[j] /= static_cast<float>(cs*cs);
		}
		return RILBPfeature;
	}

	Mat LBP::RILBPHistogramT(const Mat& dst, int sub) {
		int r = dst.rows, c = dst.cols;
		Mat RILBPfeature;//(1, numPattern*numCellX*numCellY, CV_32FC1, Scalar(0.0));
		Mat cell, tmp;
		Mat cellFeature(1, 0, CV_32FC1);
		// corresponding region of rotation-invariant LBP image and its LBP feature vector.
		for (int y = 0; y < sub; ++y) {
			for (int x = 0; x < sub; ++x) {
				cell = dst(Rect(x*c/sub, y*r/sub, c/sub, r/sub));
				// cell = dst(Rect(x*cellSize/2, y*cellSize/2, cellSize/2, cellSize/2));
				cellFeature = RILBPHistogramW(cell, cellSize/sub);
				if(!RILBPfeature.cols)  RILBPfeature = cellFeature;
				else {
					hconcat(RILBPfeature, cellFeature, tmp);
					RILBPfeature = tmp;
				}
			}
		}
		return RILBPfeature;
	}


	Mat LBP::RILBPHistogram(const Mat &dst) {
		return RILBPHistogramW(dst, cellSize);
	}

    LBPSVM::~LBPSVM() {
		delete ModelDir;
		delete EigenMatDir;
	}

	const char* LBPSVM::getMD() const {return ModelDir;}

	LBPSVM::LBPSVM(int cs,	int R, const char* EM, const char* MD, int sub, std::vector<int> fileNums, ml::SVM::Types type, ml::SVM::KernelTypes kernel_type, 
               TermCriteria term_crit, double _c, double coef0, double degree, double gamma, 
               const char* TrainingPositive, const char* TrainingNegative, const char* TestingPositive, const char* TestingNegative)
    {
		assert(100 % cs == 0  &&  cs % sub == 0);
		csize = cs;
		nblocks = sub;
		radius = R;
		extractor = lbp::LBP(8, radius, csize);
		SVM = ml::SVM::create();
		SVM->setType(type);
		SVM->setKernel(kernel_type);
		SVM->setTermCriteria(term_crit);
		SVM->setC(_c);
		SVM->setCoef0(coef0);
		SVM->setDegree(degree);
		SVM->setGamma(gamma);
		FN = fileNums;
		dirs.push_back(TrainingPositive);
		dirs.push_back(TrainingNegative);
		dirs.push_back(TestingPositive);
		dirs.push_back(TestingNegative);

		//setting the directory for saving the support vectors and parameters.
		EigenMatDir = new char[100];
		strcpy(EigenMatDir, EM);
		ModelDir = new char[100];
		*ModelDir = '\0';
		char csizestr[10], radiusstr[10], nblocksstr[10];
		snprintf(csizestr, 10, "%d", csize);
		snprintf(radiusstr, 10, "%d", radius);
		snprintf(nblocksstr, 10, "%d", nblocks);
		strcat(ModelDir, MD);
		strcat(ModelDir, csizestr);
		strcat(ModelDir, "_");
		strcat(ModelDir, radiusstr);
		strcat(ModelDir, ".xml");
	}

	void LBPSVM::getEigen(){
/*
		Mat hist;
	    std::vector<int> vec;
	    LoadSample(hist, vec, 0);
	    int n_comp = 36;
	    PCA pca(hist, Mat(), PCA::DATA_AS_ROW, n_comp);
        EigenMat = pca.eigenvectors.clone();
*/
	    cv::FileStorage efile(EigenMatDir, cv::FileStorage::READ);
		efile["data"] >> EigenMat;
	}

	void LBPSVM::randArrange01(std::vector<int> &vec) {
		srand(time(NULL));
		int size = vec.size();
		for (int i = 0; i < size; ++i) {
			swap(vec[i], vec[rand() % size]);
		}
	}

	void LBPSVM::LoadSample(Mat &featureVectors, std::vector<int> &vec, int flg) {

		int i = 0, _flag = 2 * flg;
		for (; i < FN[_flag]; ++i)         vec.push_back(1);
		for (; i < FN[_flag] + FN[1 + _flag]; ++i)   vec.push_back(0);
		randArrange01(vec);
		
		//if(flg == 0)    extractor = lbp::LBP(8, nblocks, csize);
		//else    extractor = lbp::LBP(8, 1, csize);
		// reading images from files
		Mat img, dst;
		printf("Loading Samples...\n");
		clock_t t1 = clock();
		int countp = 0, countn = 0;
		char numstr[10], filePath[50];
		const char* suf = ".jpg";

		for (int j = 0; j < FN[_flag] + FN[1 + _flag]; ++j) {
			if (vec[j] == 1) {
				//_itoa_s(++countp, numstr, 10);
				snprintf(numstr, 10, "%d", ++countp);
				strcpy(filePath, dirs[_flag]);
			}
			else {
				//_itoa_s(++countn, numstr, 10);
				snprintf(numstr, 10, "%d", ++countn);
				strcpy(filePath, dirs[1 + _flag]);
			}
			strcat(filePath, numstr);
			strcat(filePath, suf);
			//printf("Loading file %s\n", filePath);
			img = imread(filePath, 0);
			assert(img.rows == img.cols);
			if (img.rows != 100)
				resize(img, img, Size(100, 100));
			extractor.RILBP(img, dst);
			if(flg == 0)  featureVectors.push_back(extractor.RILBPHistogram(dst));
			else  featureVectors.push_back(extractor.RILBPHistogramT(dst, nblocks));
		}
		clock_t t2 = clock();
		printf("Loading samples uses %.4f seconds\n", (t2 - t1) / 1000000.0);
	}

	void LBPSVM::Train() {
		printf("ModelDir: %s\n", ModelDir);
		Mat trainData, labels;
		std::vector<int> vec;
		LoadSample(trainData, vec, 0);
		trainData = trainData * EigenMat.t();
		labels = Mat(vec);
		printf("Training SVM classifier...\n");
		SVM->trainAuto(trainData, ml::ROW_SAMPLE, labels);
		printf("Training finished. Saving model...\n");
		SVM->save(ModelDir);
		printf("Model saved.\n");
	}

	float LBPSVM::Validate(const char* model) {
		Ptr<ml::SVM> SVC = ml::SVM::create();
		SVC = ml::SVM::load(model);
		Mat testData;
		std::vector<int> vec;
		LoadSample(testData, vec, 1);
		testData = testData * EigenMat.t();
		printf("Validating...\n");
		int r = testData.rows, c = testData.cols;
		int np = pow(100/csize, 2) * extractor.getNumPattern();
		int acc_count = 0, res = 0, pos;
		for (int i = 0; i < r; ++i) {
			res = 0;  pos = 0;
			for (int j = 0; j < c/np; ++j) {
				float response = SVC->predict(testData(Rect(j*np, i, np, 1)));
				if (static_cast<int>(response) == 1.0)
					res += 1;
			}
			if (res > (c/np)/4)  pos = 1;
			// Threshold: number of positive cells > number of cells/5
			if(pos == vec[i])
				acc_count++;
			//else printf("%d, %d->%d\n", res, vec[i], pos);
		}
		float acc = static_cast<float>(acc_count) / vec.size();
		printf("%dx%d. Predicting accuracy is %.4f\n", csize, nblocks, acc);
		return acc;
	}

	int LBPSVM::Predict(const char* fname, const char* model) {
		Mat img = imread(fname, 0), dst;
		assert(img.rows > 0  &&  img.rows == img.cols);
		if (img.rows != 100)
			resize(img, img, Size(100, 100));
		Ptr<ml::SVM> SVC = ml::SVM::create();
		SVC = ml::SVM::load(model);
		//extractor = lbp::LBP(8, 1, csize);
		extractor.RILBP(img, dst);
		Mat lbpf = extractor.RILBPHistogramT(dst, nblocks);
		int r = lbpf.rows, c = lbpf.cols;
		int np = pow(100 / csize, 2) * extractor.getNumPattern(), pos = 0;
		for (int j = 0; j < c / np; ++j) {
			Mat hist = lbpf(Rect(j*np, 0, np, 1)) * EigenMat.t();
			float response = SVC->predict(hist);
			if (response == 1.0)
				pos += 1;
		}
		if(pos > (c/np)/3)  return 1;
		else return 0;
	}

	std::vector<int> LBPSVM::Predb(const Mat& imgs){
        Ptr<ml::SVM> SVC = ml::SVM::create();
	    SVC = ml::SVM::load(ModelDir);
        int bsize = imgs.rows, col = imgs.cols;
        float response;
        std::vector<int> res;
        for(int i = 0; i < bsize; ++i){
			Mat hist = imgs(Rect(0, i, col, 1)) * EigenMat.t();
            response = SVC->predict(hist);
            if(response == 1.0)
                res.push_back(1);
            else  res.push_back(0);
        }
        return res;
    }

    float LBPSVM::Predict(const Mat& img){
	    assert(img.rows == img.cols);
    	if (img.rows != 100)
	    	resize(img, img, Size(100, 100));
	    Ptr<ml::SVM> SVC = ml::SVM::create();
    	SVC = ml::SVM::load(ModelDir);
    	LBP A(8, radius, csize);
		cv::Mat dst, hist_full;
		A.RILBP(img, dst);
		hist_full = A.RILBPHistogram(dst);
		Mat hist = hist_full * EigenMat.t();
		float response = SVC->predict(hist);
		//printf("%.2f\n", response);
		return response;
	}

	std::vector<cv::Rect> LBPSVM::PredictRec(const char* fname, const char* model) {
		Mat img = imread(fname, 0), dst;
		assert(img.rows == img.cols);
		if (img.rows != 100)
			resize(img, img, Size(100, 100));
		Ptr<ml::SVM> SVC = ml::SVM::create();
		SVC = ml::SVM::load(model);
		//extractor = lbp::LBP(8, 1, csize);
		extractor.RILBP(img, dst);
		Mat lbpf = extractor.RILBPHistogramT(dst, nblocks);
		int r = lbpf.rows, c = lbpf.cols;
		int np = pow(100 / csize, 2) * extractor.getNumPattern(), pos = 0;
		std::vector<cv::Rect> smkregions;
		for (int j = 0; j < c / np; ++j) {
			Mat hist = lbpf(Rect(j*np, 0, np, 1)) * EigenMat.t();
			float response = SVC->predict(hist);
			if (response == 1.0) {
				pos += 1;
				//cv::Point pt1((j%nblocks)*100/nblocks, 100*j/(nblocks*nblocks));
				//cv::Point pt2(pt1.x+100/nblocks, pt1.y+100/nblocks);
				smkregions.push_back(cv::Rect((j%nblocks)*100/nblocks, 
					(100/nblocks)*(j/nblocks), 100/nblocks, 100/nblocks));
			}
		}
		/*
		for (int i = 0; i < smkregions.size(); ++i) {
			cv::rectangle(img, smkregions[i], {255, 0, 0});
		}
		cv::imshow("detection image", img);
		cv::waitKey(0);
		cv::destroyAllWindows();*/
		return smkregions;
	}
}