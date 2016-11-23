#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <tchar.h>

#include <vector>
#include <array>

#include <wrl/client.h>

#include "Wininet.h"

#include <tchar.h>
#include <assert.h>
#include <string>
#include <string.h>
#include <sstream>
#include <locale.h>
#include <conio.h>
#include "NhConversion.h"
#include <random>

using namespace Microsoft::WRL;

typedef std::basic_string<TCHAR>		tstring;
typedef std::basic_stringstream<TCHAR>	tstringstream;

class Kinect
{
private:
	//フレーム数
	static int count;
	
	//チョキの長さ判定に使用
	static int lassoCount;

    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Coordinate Mapper
    ComPtr<ICoordinateMapper> coordinateMapper;

    // Reader
    ComPtr<IColorFrameReader> colorFrameReader;
    ComPtr<IBodyFrameReader> bodyFrameReader;
	ComPtr<IDepthFrameReader> depthFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;
    cv::Mat colorMat;

    // Body Buffer
    std::array<IBody*, BODY_COUNT> bodies;
    std::array<cv::Vec3b, BODY_COUNT> colors;

	//Depth Buffer
	std::vector<UINT16> depthBuffer;
	int depthWidth;
	int depthHeight;
	unsigned int depthBytesPerPixel;
	cv::Mat depthMat;

public:
    // Constructor
    Kinect();

    // Destructor
    ~Kinect();

    // Processing
    void run();

	void countInitialize(){ count = 0; }
	void nextCount(){ count++; }
	int getCount(){ return count; }

	//lassoCount
	void initializeLassoCount(){ lassoCount == 0; }
	void plus1ToLassoCount(){ lassoCount++; }
	bool isAcquireBodyPoint(){
		if (lassoCount > 10){
			return true;
		}
		else{
			return false;
		}
	}

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    inline void initializeSensor();

    // Initialize Color
    inline void initializeColor();

    // Initialize Body
    inline void initializeBody();

	//initialize Depth
	inline void initializeDepth();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Update Body
    inline void updateBody();

	//Update Depth
	inline void updateDepth();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Body
    inline void drawBody();

	//Draw Depth
	inline void drawDepth();

    // Draw Circle
    inline void drawEllipse( cv::Mat& image, const Joint& joint, const int radius, const cv::Vec3b& color, const int thickness = -1 );

    // Draw Hand State
    inline void drawHandState( cv::Mat& image, const Joint& joint, HandState handState, TrackingConfidence handConfidence );

    // Show Data
    void show();

    // Show Body
    inline void showBody();

	inline void showDepth();

	cv::Point convert_joint(Joint joint);

	//evaluate_angle
	float evaluate_angle(Joint c, Joint a, Joint b);

	//output_data
	void output_data();
};

bool HttpRequest(tstring strUserAgent, tstring strUrl, bool bIsHttpVerbGet, tstring strParameter, tstring& rstrResult);