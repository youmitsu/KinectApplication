#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>

#include <omp.h>

#include "stdlib.h"
#include <stdio.h>
#include <iostream>

#include <fstream>
#include <Wininet.h>
#include "NhConversion.h"
#include "json_builder.h"
#include <map>
#include <boost/any.hpp>

#pragma comment (lib, "Wininet.lib")

using namespace boost;

typedef std::basic_string<TCHAR>		tstring;
typedef std::basic_stringstream<TCHAR>	tstringstream;

#define PI 3.1415
#define URLBUFFER_SIZE		(4096)
#define	READBUFFER_SIZE		(4096)
#define FEATURE_SIZE 5
#define FRAME_SIZE 13
#define ISPOST false

// Constructor
Kinect::Kinect()
{
    // Initialize
    initialize();
}

// Destructor
Kinect::~Kinect()
{
    // Finalize
    finalize();
}

std::ofstream tracked_check_log("tracked_check.dat");

int Kinect::count;
int Kinect::lassoCount;
// Processing
void Kinect::run()
{	
	countInitialize();
	initializeLassoCount();
	tstring strUserAgent = _T("HttpRequestTest");
	tstring strUrl = _T("https://kinect-walking-api.herokuapp.com/index");
	bool bIsHttpVerbGet = false;
	const int proc_id = 2016120603;
	const int devise_id = 1;
	const int person = 2;
	tstring strResult;

	map<string, any> obj;
	obj["proc_id"] = proc_id;
	obj["devise_id"] = devise_id;
	obj["person"] = person;

	std::ofstream angle_logs[FEATURE_SIZE];
	angle_logs[0].open("angle_data_hip.dat");
	angle_logs[1].open("angle_data_left_knee.dat");
	angle_logs[2].open("angle_data_left_elbow.dat");
	angle_logs[3].open("angle_data_right_knee.dat");
	angle_logs[4].open("angle_data_right_elbow.dat");

	vector<any> featureHip;
	vector<any> featureLeftKnee;
	vector<any> featureLeftElbow;
	vector<any> featureRightKnee;
	vector<any> featureRightElbow;
	vector<vector<any>> features = { featureHip, featureLeftKnee, featureLeftElbow, featureRightKnee, featureRightElbow };
	double *sums;
	sums = (double *)calloc(FEATURE_SIZE, sizeof(double));

    // Main Loop
    while( true ){
		nextCount();
        // Update Data
        update();

		//角度データ出力
		if (isAcquireBodyPoint()){
			if (isFirstAcquire()){
				countInitialize();
			}
		//	std::cout << getCount() << std::endl;
			output_data(features, sums);
		}

        // Draw Data
        draw();

        // Show Data
        show();

        // Key Check
        const int key = cv::waitKey( 10 );
        if( key == VK_ESCAPE ){
            break;
        }
    }

	//平均値の算出～vectorの中身を平均値で引く
	
	for (int i = 0; i < FEATURE_SIZE; i++){ cout << sums[i] << endl; }
	double mean;
	int i = 0;
	for (auto itr = features.begin(); itr != features.end(); ++itr){
		vector<any> feature = *itr;
		mean = sums[i]/feature.size();
		for (auto itr2 = feature.begin(); itr2 != feature.end(); ++itr2){
			any val = *itr2;
			double cast_val = any_cast<double>(val)-mean;
			*itr2 = cast_val;
			angle_logs[i] << cast_val << endl;
		}
		i++;
	}

	obj["f1"] = features[0];
	obj["f2"] = features[1];
	obj["f3"] = features[2];
	obj["f4"] = features[3];
	obj["f5"] = features[4];
	string json = json_builder::toJson(obj);
	cout << json << endl;
	
	setlocale(LC_ALL, "Japanese");
	TCHAR* str = new TCHAR[10000000];
	_stprintf_s(str, 10000000, _T("%s"), json.c_str());
	tstring strParameter = str;
	if (ISPOST){
		if (!HttpRequest(strUserAgent, strUrl, bIsHttpVerbGet, strParameter, strResult))
		{
			std::cout << "failure" << std::endl;
		}
	}
	setlocale(LC_ALL, "Japanese");
	_tprintf(_T("%s"), strResult.c_str());
	tracked_check_log.close();
	for (int i = 0; i < FEATURE_SIZE; i++){
		angle_logs[i].close();
	}
}

// Initialize
void Kinect::initialize()
{
    cv::setUseOptimized( true );

    // Initialize Sensor
    initializeSensor();

	//initialize Depth
	initializeDepth();

    // Initialize Color
    initializeColor();

    // Initialize Body
    initializeBody();

    // Wait a Few Seconds until begins to Retrieve Data from Sensor ( about 2000-[ms] )
    std::this_thread::sleep_for( std::chrono::seconds( 2 ) );
}

// Initialize Sensor
inline void Kinect::initializeSensor()
{
    // Open Sensor
    ERROR_CHECK( GetDefaultKinectSensor( &kinect ) );

    ERROR_CHECK( kinect->Open() );

    // Check Open
    BOOLEAN isOpen = FALSE;
    ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
    if( !isOpen ){
        throw std::runtime_error( "failed IKinectSensor::get_IsOpen( &isOpen )" );
    }

    // Retrieve Coordinate Mapper
    ERROR_CHECK( kinect->get_CoordinateMapper( &coordinateMapper ) );
}

// Initialize Depth
inline void Kinect::initializeDepth()
{
	// Open Depth Reader
	ComPtr<IDepthFrameSource> depthFrameSource;
	ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
	ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

	// Retrieve Depth Description
	ComPtr<IFrameDescription> depthFrameDescription;
	ERROR_CHECK(depthFrameSource->get_FrameDescription(&depthFrameDescription));
	ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth)); // 512
	ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight)); // 424
	ERROR_CHECK(depthFrameDescription->get_BytesPerPixel(&depthBytesPerPixel)); // 2

	// Retrieve Depth Reliable Range
	UINT16 minReliableDistance;
	UINT16 maxReliableDistance;
	ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(&minReliableDistance)); // 500
	ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(&maxReliableDistance)); // 4500
	std::cout << "Depth Reliable Range : " << minReliableDistance << " - " << maxReliableDistance << std::endl;

	// Allocation Depth Buffer
	depthBuffer.resize(depthWidth * depthHeight);
}

// Initialize Color
inline void Kinect::initializeColor()
{
    // Open Color Reader
    ComPtr<IColorFrameSource> colorFrameSource;
    ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
    ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

    // Retrieve Color Description
    ComPtr<IFrameDescription> colorFrameDescription;
    ERROR_CHECK( colorFrameSource->CreateFrameDescription( ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
    ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) ); // 1920
    ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) ); // 1080
    ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) ); // 4

    // Allocation Color Buffer
    colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );
}

// Initialize Body
inline void Kinect::initializeBody()
{
    // Open Body Reader
    ComPtr<IBodyFrameSource> bodyFrameSource;
    ERROR_CHECK( kinect->get_BodyFrameSource( &bodyFrameSource ) );
    ERROR_CHECK( bodyFrameSource->OpenReader( &bodyFrameReader ) );

    // Initialize Body Buffer
    for( auto& body : bodies ){
        body = nullptr;
    }

    // Color Table for Visualization
    colors[0] = cv::Vec3b( 255,   0,   0 ); // Blue
    colors[1] = cv::Vec3b(   0, 255,   0 ); // Green
    colors[2] = cv::Vec3b(   0,   0, 255 ); // Red
    colors[3] = cv::Vec3b( 255, 255,   0 ); // Cyan
    colors[4] = cv::Vec3b( 255,   0, 255 ); // Magenta
    colors[5] = cv::Vec3b(   0, 255, 255 ); // Yellow
}

// Finalize
void Kinect::finalize()
{
    cv::destroyAllWindows();

    // Release Body Buffer
    for( auto& body : bodies ){
        SafeRelease( body );
    }

    // Close Sensor
    if( kinect != nullptr ){
        kinect->Close();
    }
}

// Update Data
void Kinect::update()
{
	//Update Depth
	updateDepth();

    // Update Color
    updateColor();

    // Update Body
    updateBody();
}

// Update Color
inline void Kinect::updateColor()
{
    // Retrieve Color Frame
    ComPtr<IColorFrame> colorFrame;
    const HRESULT ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Convert Format ( YUY2 -> BGRA )
    ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray( static_cast<UINT>( colorBuffer.size() ), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );
}

// Update Body
inline void Kinect::updateBody()
{
    // Retrieve Body Frame
    ComPtr<IBodyFrame> bodyFrame;
    const HRESULT ret = bodyFrameReader->AcquireLatestFrame( &bodyFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Release Previous Bodies
    for( auto& body : bodies ){
        SafeRelease( body );
    }

    // Retrieve Body Data
    ERROR_CHECK( bodyFrame->GetAndRefreshBodyData( static_cast<UINT>( bodies.size() ), &bodies[0] ) );
}

// Update Depth
inline void Kinect::updateDepth()
{
	// Retrieve Depth Frame
	ComPtr<IDepthFrame> depthFrame;
	const HRESULT ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
	if (FAILED(ret)){
		return;
	}

	// Retrieve Depth Data
	ERROR_CHECK(depthFrame->CopyFrameDataToArray(static_cast<UINT>(depthBuffer.size()), &depthBuffer[0]));
}

// Draw Data
void Kinect::draw()
{
	drawDepth();

    // Draw Color
    drawColor();

    // Draw Body
    drawBody();
}

// Draw Depth
inline void Kinect::drawDepth()
{
	// Retrieve Mapped Coordinates
	/*std::vector<DepthSpacePoint> depthSpace(colorWidth * colorHeight);
	ERROR_CHECK(coordinateMapper->MapColorFrameToDepthSpace(depthBuffer.size(), &depthBuffer[0], depthSpace.size(), &depthSpace[0]));

	// Mapping Depth to Color Resolution
	std::vector<UINT16> buffer(colorWidth * colorHeight);

	for (int colorY = 0; colorY < colorHeight; colorY++){
		for (int colorX = 0; colorX < colorWidth; colorX++){
			unsigned int colorIndex = colorY * colorWidth + colorX;
			int depthX = static_cast<int>(depthSpace[colorIndex].X + 0.5f);
			int depthY = static_cast<int>(depthSpace[colorIndex].Y + 0.5f);
			if ((0 <= depthX) && (depthX < depthWidth) && (0 <= depthY) && (depthY < depthHeight)){
				unsigned int depthIndex = depthY * depthWidth + depthX;
				buffer[colorIndex] = depthBuffer[depthIndex];
			}
		}
	}
	// Create cv::Mat from Depth Buffer*/
	depthMat = cv::Mat(depthHeight, depthWidth, CV_16UC1, &depthBuffer[0]);
}

// Draw Color
inline void Kinect::drawColor()
{
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
}

bool Kinect::isAllJointTracked(std::array<Joint, JointType::JointType_Count>& joints){
	std::vector<int> jointComposeAngle = { JointType::JointType_KneeLeft, JointType::JointType_HipLeft, JointType::JointType_AnkleLeft,
		JointType::JointType_KneeRight, JointType::JointType_HipRight, JointType::JointType_AnkleRight, JointType::JointType_SpineBase,
		JointType::JointType_ElbowLeft, JointType::JointType_ShoulderLeft, JointType::JointType_WristLeft,
		JointType::JointType_ElbowRight, JointType::JointType_ShoulderRight, JointType::JointType_WristRight };
	bool isTracked = true;
	tracked_check_log << "===============" << getCount() << "===============" << std::endl;
	for (int type = 0; type < JointType::JointType_Count; type++){
		const Joint joint = joints[type];
		tracked_check_log << "--jointType----" << joint.JointType << "-----" << std::endl;
		tracked_check_log << joint.TrackingState << std::endl;
		if (joint.TrackingState == TrackingState::TrackingState_NotTracked){
			isTracked = false;
			break;
		}
	/*	if (joint.TrackingState == TrackingState::TrackingState_Inferred){
			auto itr = std::find(jointComposeAngle.begin(), jointComposeAngle.end(), joint.JointType);
			if (itr == jointComposeAngle.end()){
				isTracked = false;
				break;
			}
		}*/
	}
	return isTracked;
}

void Kinect::output_data(std::vector<vector<any>>& features, double* sums)
{
	double angleLeftKnee, angleRightKnee, angleHip, angleLeftElbow, angleRightElbow;
	for (int index = 0; index < BODY_COUNT; index++){
		ComPtr<IBody> body = bodies[index];
		if (body == nullptr){
			continue;
		}

		BOOLEAN tracked = FALSE;
		ERROR_CHECK(body->get_IsTracked(&tracked));
		if (!tracked){
			continue;
		}

		std::array<Joint, JointType::JointType_Count> joints;
		ERROR_CHECK(body->GetJoints(static_cast<UINT>(joints.size()), &joints[0]));

		if (isAllJointTracked(joints)){
			angleLeftKnee = evaluate_angle(joints[JointType::JointType_KneeLeft], joints[JointType::JointType_SpineBase], joints[JointType::JointType_AnkleLeft]);
			angleRightKnee = evaluate_angle(joints[JointType::JointType_KneeRight], joints[JointType::JointType_SpineBase], joints[JointType::JointType_AnkleRight]);
			angleHip = evaluate_angle(joints[JointType::JointType_SpineBase], joints[JointType::JointType_KneeRight], joints[JointType::JointType_KneeLeft]);
			angleLeftElbow = evaluate_angle(joints[JointType::JointType_ElbowLeft], joints[JointType::JointType_ShoulderLeft], joints[JointType::JointType_WristLeft]);
			angleRightElbow = evaluate_angle(joints[JointType::JointType_ElbowRight], joints[JointType::JointType_ShoulderRight], joints[JointType::JointType_WristRight]);
		}
		else{
			angleLeftKnee = 0.0;
			angleRightKnee = 0.0;
			angleHip = 0.0;
			angleLeftElbow = 0.0;
			angleRightElbow = 0.0;
		}
		features[0].push_back(angleHip);
		features[1].push_back(angleLeftKnee);
		features[2].push_back(angleLeftElbow);
		features[3].push_back(angleRightKnee);
		features[4].push_back(angleRightElbow);

		//平均値計算用
		sums[0] += angleHip;
		sums[1] += angleLeftKnee;
		sums[2] += angleLeftElbow;
		sums[3] += angleRightKnee;
		sums[4] += angleRightElbow;
	}
}

// Draw Body
inline void Kinect::drawBody()
{
    // Draw Body Data to Color Data
    #pragma omp parallel for
    for( int index = 0; index < BODY_COUNT; index++ ){
        ComPtr<IBody> body = bodies[index];
        if( body == nullptr ){
            continue;
        }

        // Check Body Tracked
        BOOLEAN tracked = FALSE;
        ERROR_CHECK( body->get_IsTracked( &tracked ) );
        if( !tracked ){
            continue;
        }

        // Retrieve Joints
        std::array<Joint, JointType::JointType_Count> joints;
        ERROR_CHECK( body->GetJoints( static_cast<UINT>( joints.size() ), &joints[0] ) );

        #pragma omp parallel for
        for( int type = 0; type < JointType::JointType_Count; type++ ){
            // Check Joint Tracked
            const Joint joint = joints[type];
            if( joint.TrackingState == TrackingState::TrackingState_NotTracked ){
                continue;
            }

            // Draw Joint Position
            drawEllipse( colorMat, joint, 5, colors[index] );

            // Draw Left Hand State
            if( joint.JointType == JointType::JointType_HandLeft ){
                HandState handState;
                TrackingConfidence handConfidence;
                ERROR_CHECK( body->get_HandLeftState( &handState ) );
                ERROR_CHECK( body->get_HandLeftConfidence( &handConfidence ) );

                drawHandState( colorMat, joint, handState, handConfidence );
            }

            // Draw Right Hand State
            if( joint.JointType == JointType::JointType_HandRight ){
                HandState handState;
                TrackingConfidence handConfidence;
                ERROR_CHECK( body->get_HandRightState( &handState ) );
                ERROR_CHECK( body->get_HandRightConfidence( &handConfidence ) );

                drawHandState( colorMat, joint, handState, handConfidence );
				if (handState == HandState::HandState_Open){
					plus1ToLassoCount();
				}
            }
        }

        /*
        // Retrieve Joint Orientations
        std::array<JointOrientation, JointType::JointType_Count> orientations;
        ERROR_CHECK( body->GetJointOrientations( JointType::JointType_Count, &orientations[0] ) );
        */

        /*
        // Retrieve Amount of Body Lean
        PointF amount;
        ERROR_CHECK( body->get_Lean( &amount ) );
        */
    }
}

// Draw Ellipse
inline void Kinect::drawEllipse( cv::Mat& image, const Joint& joint, const int radius, const cv::Vec3b& color, const int thickness )
{
    if( image.empty() ){
        return;
    }

    // Convert Coordinate System and Draw Joint
    ColorSpacePoint colorSpacePoint;
    ERROR_CHECK( coordinateMapper->MapCameraPointToColorSpace( joint.Position, &colorSpacePoint ) );
    const int x = static_cast<int>( colorSpacePoint.X + 0.5f );
    const int y = static_cast<int>( colorSpacePoint.Y + 0.5f );
    if( ( 0 <= x ) && ( x < image.cols ) && ( 0 <= y ) && ( y < image.rows ) ){
        cv::circle( image, cv::Point( x, y ), radius, static_cast<cv::Scalar>( color ), thickness, cv::LINE_AA );
    }
}

// Draw Hand State
inline void Kinect::drawHandState( cv::Mat& image, const Joint& joint, HandState handState, TrackingConfidence handConfidence )
{
    if( image.empty() ){
        return;
    }

    // Check Tracking Confidence
    if( handConfidence != TrackingConfidence::TrackingConfidence_High ){
        return;
    }

    // Draw Hand State 
    const int radius = 75;
    const cv::Vec3b blue = cv::Vec3b( 128, 0, 0 ), green = cv::Vec3b( 0, 128, 0 ), red = cv::Vec3b( 0, 0, 128 );
    switch( handState ){
        // Open
        case HandState::HandState_Open:
            drawEllipse( image, joint, radius, green, 5 );
            break;
        // Close
        case HandState::HandState_Closed:
            drawEllipse( image, joint, radius, red, 5 );
            break;
        // Lasso
        case HandState::HandState_Lasso:
            drawEllipse( image, joint, radius, blue, 5 );
            break;
        default:
            break;
    }
}

// Show Data
void Kinect::show()
{
	// Show Depth
	showDepth();
    // Show Body
    showBody();
}

// Show Depth
inline void Kinect::showDepth()
{
	if (depthMat.empty()){
		return;
	}

	// Scaling ( 0-8000 -> 255-0 )
	cv::Mat scaleMat;
	depthMat.convertTo(scaleMat, CV_8U, -255.0 / 8000.0, 255.0);
	//cv::applyColorMap( scaleMat, scaleMat, cv::COLORMAP_BONE );

	// Show Image
	cv::imshow("Depth", scaleMat);
}

// Show Body
inline void Kinect::showBody()
{
    if( colorMat.empty() ){
        return;
    }

    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( colorMat, resizeMat, cv::Size(), scale, scale );

	if (isAcquireBodyPoint()){
		std::ostringstream oss;
		oss << getCount() << ".png";
		std::string filename = oss.str();
		try{
			imwrite(filename, resizeMat);
		}
		catch (std::runtime_error& ex){
			std::cout << "failure" << std::endl;
		}
		oss.str("");
	}
    // Show Image
    cv::imshow( "Body", resizeMat );
}

cv::Point Kinect::convert_joint(Joint joint){
	ColorSpacePoint colorSpacePoint;
//	std::cout << joint.Position.X << " " << joint.Position.Y << " " <<  joint.Position.Z << std::endl;
	ERROR_CHECK(coordinateMapper->MapCameraPointToColorSpace(joint.Position, &colorSpacePoint));
	const int x = static_cast<int>(colorSpacePoint.X + 0.5f);
	const int y = static_cast<int>(colorSpacePoint.Y + 0.5f);
	return cv::Point(x, y);
}

//3点を与えられたときに角度を求める
//c:角度の基準点、a,b:それ以外
float Kinect::evaluate_angle(Joint c, Joint a, Joint b)
{
	/*cv::Point pointC = convert_joint(c);
	cv::Point pointB = convert_joint(a);
	cv::Point pointA = convert_joint(b);*/
	cv::Point3f pointC = cv::Point3f(c.Position.X, c.Position.Y, c.Position.Z);
	cv::Point3f pointA = cv::Point3f(a.Position.X, a.Position.Y, a.Position.Z);
	cv::Point3f pointB = cv::Point3f(b.Position.X, b.Position.Y, b.Position.Z);
	int cx = pointC.x*1000;
	int cy = pointC.y*1000;
	int ax = pointA.x*1000;
	int ay = pointA.y*1000;
	int bx = pointB.x*1000;
	int by = pointB.y*1000;
	int ax_cx = ax - cx;
	int ay_cy = ay - cy;
	int bx_cx = bx - cx;
	int by_cy = by - cy;
	float cos = ((ax_cx*bx_cx) + (ay_cy*by_cy)) / ((sqrt((ax_cx*ax_cx) + (ay_cy*ay_cy))*sqrt((bx_cx*bx_cx) + (by_cy*by_cy))));
	float angle = acosf(cos);
	//if (angle > PI / 2){ angle = PI - angle; }
	return angle;
}

bool HttpRequest(tstring strUserAgent,
	tstring strUrl,
	bool bIsHttpVerbGet,
	tstring strParameter,
	tstring& rstrResult)
{
	// アウトプットの初期化
	rstrResult = tstring();

	// インプットのチェック
	if (0 == strUrl.length())
	{
		assert(!"URLが不正");
		return false;
	}

	// 変数
	HINTERNET			hInternetOpen = NULL;
	HINTERNET			hInternetConnect = NULL;
	HINTERNET			hInternetRequest = NULL;
	char*				pszOptional = NULL;
	URL_COMPONENTS		urlcomponents;
	tstring				strServer;
	tstring				strObject;
	INTERNET_PORT		nPort;
	tstring				strVerb;
	tstring				strHeaders;
	tstringstream		ssRead;

	// URL解析
	ZeroMemory(&urlcomponents, sizeof(URL_COMPONENTS));
	urlcomponents.dwStructSize = sizeof(URL_COMPONENTS);
	TCHAR szHostName[URLBUFFER_SIZE];
	TCHAR szUrlPath[URLBUFFER_SIZE];
	urlcomponents.lpszHostName = szHostName;
	urlcomponents.lpszUrlPath = szUrlPath;
	urlcomponents.dwHostNameLength = URLBUFFER_SIZE;
	urlcomponents.dwUrlPathLength = URLBUFFER_SIZE;
	if (!InternetCrackUrl(strUrl.c_str(),
		(DWORD)strUrl.length(),
		0,
		&urlcomponents))
	{	// URLの解析に失敗
		assert(!"URL解析に失敗");
		return false;
	}
	strServer = urlcomponents.lpszHostName;
	strObject = urlcomponents.lpszUrlPath;
	nPort = urlcomponents.nPort;

	// HTTPかHTTPSかそれ以外か
	DWORD dwFlags = 0;
	if (INTERNET_SCHEME_HTTP == urlcomponents.nScheme)
	{	// HTTP
		dwFlags = INTERNET_FLAG_RELOAD				// 要求されたファイル、オブジェクト、またはフォルダ一覧を、キャッシュからではなく、元のサーバーから強制的にダウンロードします。
			| INTERNET_FLAG_DONT_CACHE			// 返されたエンティティをキャシュへ追加しません。
			| INTERNET_FLAG_NO_AUTO_REDIRECT;	// HTTP だけで使用され、リダイレクトが HttpSendRequest で処理されないことを指定します。
	}
	else if (INTERNET_SCHEME_HTTPS == urlcomponents.nScheme)
	{	// HTTPS
		dwFlags = INTERNET_FLAG_RELOAD				// 要求されたファイル、オブジェクト、またはフォルダ一覧を、キャッシュからではなく、元のサーバーから強制的にダウンロードします。
			| INTERNET_FLAG_DONT_CACHE			// 返されたエンティティをキャシュへ追加しません。
			| INTERNET_FLAG_NO_AUTO_REDIRECT	// HTTP だけで使用され、リダイレクトが HttpSendRequest で処理されないことを指定します。
			| INTERNET_FLAG_SECURE						// 安全なトランザクションを使用します。これにより、SSL/PCT を使うように変換され、HTTP 要求だけで有効です。 
			| INTERNET_FLAG_IGNORE_CERT_DATE_INVALID	// INTERNET_FLAG_IGNORE_CERT_DATE_INVALID、INTERNET_FLAG_IGNORE_CERT_CN_INVALID
			| INTERNET_FLAG_IGNORE_CERT_CN_INVALID;		// は、証明書に関する警告を無視するフラグ
	}
	else
	{
		assert(!"HTTPでもHTTPSでもない");
		return false;
	}

	// GETかPOSTか
	if (bIsHttpVerbGet)
	{	// GET
		strVerb = _T("GET");
		strHeaders = _T("");
		if (0 != strParameter.length())
		{	// オブジェクトとパラメータを「?」で連結
			strObject += _T("?") + strParameter;
		}
	}
	else
	{	// POST
		strVerb = _T("POST");
		strHeaders = _T("Content-Type: application/x-www-form-urlencoded");
		if (0 != strParameter.length())
		{	// パラメータを、送信するオプションデータに変換する
			pszOptional = NhT2M(strParameter.c_str());	// char文字列に変換
		}
	}

	// WinInetの初期化
	hInternetOpen = InternetOpen(strUserAgent.c_str(),
		INTERNET_OPEN_TYPE_PRECONFIG,
		NULL, NULL, 0);
	if (NULL == hInternetOpen)
	{
		assert(!"WinInetの初期化に失敗");
		goto LABEL_ERROR;
	}

	// HTTP接続
	hInternetConnect = InternetConnect(hInternetOpen,
		strServer.c_str(),
		nPort,
		NULL,
		NULL,
		INTERNET_SERVICE_HTTP,
		0,
		0);
	if (NULL == hInternetConnect)
	{
		assert(!"HTTP接続に失敗");
		goto LABEL_ERROR;
	}

	// HTTP接続を開く
	hInternetRequest = HttpOpenRequest(hInternetConnect,
		strVerb.c_str(),
		strObject.c_str(),
		NULL,
		NULL,
		NULL,
		dwFlags,
		NULL);
	if (NULL == hInternetRequest)
	{
		assert(!"HTTP接続を開くに失敗");
		goto LABEL_ERROR;
	}

	// HTTP要求送信
	if (!HttpSendRequest(hInternetRequest,
		strHeaders.c_str(),
		(DWORD)strHeaders.length(),
		(LPVOID)((char*)pszOptional),
		pszOptional ? (DWORD)(strlen(pszOptional) * sizeof(char)) : 0))
	{
		assert(!"HTTP要求送信に失敗");
		goto LABEL_ERROR;
	}

	// HTTP要求に対応するステータスコードの取得
	DWORD dwStatusCode;
	DWORD dwLength = sizeof(DWORD);
	if (!HttpQueryInfo(hInternetRequest,
		HTTP_QUERY_STATUS_CODE | HTTP_QUERY_FLAG_NUMBER,
		&dwStatusCode,
		&dwLength,
		0))
	{
		assert(!"HTTP要求に対応するステータスコードの取得に失敗");
		goto LABEL_ERROR;
	}
	if (HTTP_STATUS_OK != dwStatusCode)
	{
		assert(!"ステータスコードがOKでない");
		goto LABEL_ERROR;
	}

	// HTTPファイル読み込み
	char szReadBuffer[READBUFFER_SIZE + 1];
	while (1)
	{
		DWORD dwRead = 0;
		if (!InternetReadFile(hInternetRequest, szReadBuffer, READBUFFER_SIZE, &dwRead))
		{
			assert(!"HTTPファイル読み込みに失敗");
			goto LABEL_ERROR;
		}
		if (0 == dwRead)
		{
			break;
		}
		szReadBuffer[dwRead] = '\0';	// 終端文字「\0」の付加
		size_t length = dwRead + 1;
		LPWSTR	pszWideChar = (LPWSTR)malloc(length * sizeof(WCHAR));
		MultiByteToWideChar(CP_UTF8,	// CODE PAGE: UTF-8
			0,
			szReadBuffer,
			-1,
			pszWideChar,
			(int)length);	// UTF-8文字列をANSI文字列に変換
		TCHAR* pszTchar = NhW2T(pszWideChar);	// WideChar文字列をTCHAR文字列に変換
		ssRead << pszTchar;	// ストリーム文字列に流し込む
		free(pszTchar);
		free(pszWideChar);
	}

	// ストリーム文字列を、出力文字列に変換
	rstrResult = ssRead.str().c_str();

	if (pszOptional){ free(pszOptional); }
	InternetCloseHandle(hInternetRequest);
	InternetCloseHandle(hInternetConnect);
	InternetCloseHandle(hInternetOpen);
	return true;

LABEL_ERROR:
	if (pszOptional){ free(pszOptional); }
	InternetCloseHandle(hInternetRequest);
	InternetCloseHandle(hInternetConnect);
	InternetCloseHandle(hInternetOpen);
	return false;
}