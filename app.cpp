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

#pragma comment (lib, "Wininet.lib")

#define PI 3.1415

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

std::ofstream angle_data_left_knee("angle_data_left_knee.dat");
std::ofstream angle_data_right_knee("angle_data_right_knee.dat");
std::ofstream angle_data_hip("angle_data_hip.dat");
std::ofstream angle_data_left_elbow("angle_data_left_elbow.dat");
std::ofstream angle_data_right_elbow("angle_data_right_elbow.dat");

int Kinect::count;
int Kinect::lassoCount;
// Processing
void Kinect::run()
{	
	countInitialize();
	initializeLassoCount();
    // Main Loop
    while( true ){
		nextCount();
        // Update Data
        update();

		//äpìxÉfÅ[É^èoóÕ
		if (isAcquireBodyPoint()){
			std::cout << getCount() << std::endl;
			output_data();
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
	angle_data_left_knee.close();
	angle_data_right_knee.close();
	angle_data_hip.close();
	angle_data_left_elbow.close();
	angle_data_right_elbow.close();
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

void Kinect::output_data()
{
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

		//ç∂ïG
		angle_data_left_knee << getCount() << " " << evaluate_angle(joints[JointType::JointType_KneeLeft], joints[JointType::JointType_HipLeft],joints[JointType::JointType_AnkleLeft]) << std::endl;
        //âEïG
		angle_data_right_knee << getCount() << " " << evaluate_angle(joints[JointType::JointType_KneeRight], joints[JointType::JointType_HipRight], joints[JointType::JointType_AnkleRight]) << std::endl;
        //å“
		angle_data_hip << getCount() << " " << evaluate_angle(joints[JointType::JointType_SpineBase], joints[JointType::JointType_KneeRight], joints[JointType::JointType_KneeLeft]) << std::endl;
		//ç∂ïI
		angle_data_left_elbow << getCount() << " " << evaluate_angle(joints[JointType::JointType_ElbowLeft], joints[JointType::JointType_ShoulderLeft], joints[JointType::JointType_WristLeft]) << std::endl;
		//âEïI
		angle_data_right_elbow << getCount() << " " << evaluate_angle(joints[JointType::JointType_ElbowRight], joints[JointType::JointType_ShoulderRight], joints[JointType::JointType_WristRight]) << std::endl;
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

//3ì_Çó^Ç¶ÇÁÇÍÇΩÇ∆Ç´Ç…äpìxÇãÅÇﬂÇÈ
//c:äpìxÇÃäÓèÄì_ÅAa,b:ÇªÇÍà»äO
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