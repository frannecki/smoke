#include <iostream>
#include "libfreenect.hpp"
#include <vector>
#include <cmath>
#include <pthread.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

class FreenectDeviceM : public Freenect::FreenectDevice {
	public:
		FreenectDeviceM(freenect_context *_ctx, int _index)
	 		: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
			m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
			m_new_depth_frame(false), m_depthMat(Size(640,480),CV_16UC1),
			m_rgbMat(Size(640,480), CV_8UC3, Scalar(0)){
			
			for( unsigned int i = 0 ; i < 2048 ; i++) {
				float v = i/2048.0;
				v = pow(v, 3)* 6;
				m_gamma[i] = v*6*256;
			}
            pthread_mutex_init(&rgbMutex, NULL);
            pthread_mutex_init(&depthMutex, NULL);
		}
		
		void VideoCallback(void* _rgb, uint32_t timestamp) {
			pthread_mutex_lock(&rgbMutex);
			uint8_t* rgb = static_cast<uint8_t*>(_rgb);
			m_rgbMat.data = rgb;
			m_new_rgb_frame = true;
			pthread_mutex_unlock(&rgbMutex);
		};

		void DepthCallback(void* _depth, uint32_t timestamp) {
			pthread_mutex_lock(&depthMutex);
			uint16_t* depth = static_cast<uint16_t*>(_depth);
			m_depthMat.data = (uchar*) depth;
			m_new_depth_frame = true;
			pthread_mutex_unlock(&depthMutex);
		}
		
		bool getVideo(Mat& output) {
			pthread_mutex_lock(&rgbMutex);
			if(m_new_rgb_frame) {
				cvtColor(m_rgbMat, output, CV_RGB2BGR);
				m_new_rgb_frame = false;
				pthread_mutex_unlock(&rgbMutex);
				return true;
			} else {
				pthread_mutex_unlock(&rgbMutex);
				return false;
			}
		}
		
		bool getDepth(Mat& output) {
				pthread_mutex_lock(&depthMutex);
				if(m_new_depth_frame) {
					m_depthMat.copyTo(output);
					m_new_depth_frame = false;
					pthread_mutex_unlock(&depthMutex);
					return true;
				} else {
					pthread_mutex_unlock(&depthMutex);
					return false;
				}
			}
	private:
		vector<uint8_t> m_buffer_depth;
		vector<uint8_t> m_buffer_rgb;
		vector<uint16_t> m_gamma;
		Mat m_depthMat;
		Mat m_rgbMat;
		pthread_mutex_t rgbMutex;
		pthread_mutex_t depthMutex;
		bool m_new_rgb_frame;  //rgbMat a new frame or not
		bool m_new_depth_frame;  //depthMat a new frame or not
};