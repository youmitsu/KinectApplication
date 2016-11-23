#define __APP__

#include <Windows.h>
#include <Kinect.h>

#include "app.h"

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
	try{
		Kinect kinect;
		kinect.run();
	}
	catch (std::exception& ex){
		std::cout << ex.what() << std::endl;
	}

	cout << "hogehoge" << endl;
	return 0;
}