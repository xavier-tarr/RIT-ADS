#include "stdafx.h"

#include "FlyCapture2.h"
#include "FlyCapture2Video.h"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace FlyCapture2;
using namespace std;

enum VideoType
{
	UNCOMPRESSED,
	MJPG,
	H264
};

void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion(&fc2Version);

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "."
            << fc2Version.minor << "." << fc2Version.type << "."
            << fc2Version.build;
    cout << version.str() << endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;
}

void PrintCameraInfo(CameraInfo *pCamInfo)
{
    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number - " << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl
         << endl;
}

void PrintError(Error error) { error.PrintErrorTrace(); }

void SaveVideoHelper(VideoType VideoType,
	std::vector<Image> &vecImages,
	std::string videoFileName,
	float frameRate)
{
	Error error;
	FlyCapture2Video video;

	// Set maximum video file size to 2GB.
	// A new video file is generated when 2GB
	// limit is reached. Setting maximum file
	// size to 0 indicates no limit.
	const unsigned int k_videoFileSize = 2048;

	video.SetMaximumFileSize(k_videoFileSize);

	// Open the video file for appending images
	switch (VideoType)
	{
	case UNCOMPRESSED:
	{
		AVIOption option;
		option.frameRate = frameRate;
		error = video.Open(videoFileName.c_str(), &option);
	}
	break;
	case MJPG:
	{
		MJPGOption option;
		option.frameRate = frameRate;
		option.quality = 75;
		error = video.Open(videoFileName.c_str(), &option);
	}
	break;
	case H264:
	{
		H264Option option;
		option.frameRate = frameRate;
		option.bitrate = 1000000;
		option.height = vecImages[0].GetRows();
		option.width = vecImages[0].GetCols();
		error = video.Open(videoFileName.c_str(), &option);
	}
	break;
	}

	if (error != PGRERROR_OK)
	{
		PrintError(error);
		return;
	}

	cout << endl;
	cout << "Appending " << vecImages.size()
		<< " images to video file: " << videoFileName.c_str() << endl;
	for (int imageCnt = 0; imageCnt < vecImages.size(); imageCnt++)
	{
		// Append the image to video file
		error = video.Append(&vecImages[imageCnt]);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			continue;
		}

		cout << "Appended image " << imageCnt << "..." << endl;
	}

	// Close the video file
	error = video.Close();
	if (error != PGRERROR_OK)
	{
		PrintError(error);
		return;
	}
}

int RunCamera(PGRGuid *guid, Camera *pCams, unsigned int numCams)
{
	const int k_numImages = 100;
	float frameRateToUse = 40.0f;
	Error error;

	//CameraInfo *camInfo = new CameraInfo[numCams];
	//PropertyInfo *propinfo = new PropertyInfo[numCams];

	for(int i = 0; i < numCams; i++){

		// Connect to a camera
		error = pCams[i].Connect(&guid[i]);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		CameraInfo camInfo;
		// Get the camera information
		error = pCams[i].GetCameraInfo(&camInfo);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		PrintCameraInfo(&camInfo);

		// Start capturing images
		cout << "Starting capture... " << endl;
		error = pCams[i].StartCapture();
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

	}

	std::vector<Image> vecImage1;
	vecImage1.resize(k_numImages);

	std::vector<Image> vecImage2;
	vecImage2.resize(k_numImages);

	// Grab images
	Image rawImage1;
	Image rawImage2;

        cout << "Starting Calibration Image Capture!!!" << endl;
	for (int imageCnt = 0; imageCnt < k_numImages; imageCnt++)
	{
		cout << "Press Enter to capture image " << imageCnt << endl;
                cin.ignore();

		error = pCams[0].RetrieveBuffer(&rawImage1);
		if (error != PGRERROR_OK)
		{
			cout << "Error grabbing image(1) " << imageCnt << endl;
			continue;
		}
		else
		{
			cout << "Grabbed image(1) " << imageCnt << endl;
		}

		vecImage1[imageCnt].DeepCopy(&rawImage1);

		error = pCams[1].RetrieveBuffer(&rawImage2);
		if (error != PGRERROR_OK)
		{
			cout << "Error grabbing image(2) " << imageCnt << endl;
			continue;
		}
		else
		{
			cout << "Grabbed image(2) " << imageCnt << endl;cout << "Error grabbing image(2) " << imageCnt << endl;
		}

		vecImage2[imageCnt].DeepCopy(&rawImage2);
	}

	for (int i = 0; i < numCams; i++){

		// Stop capturing images
		cout << "Stopping capture... " << endl;
		error = pCams[i].StopCapture();
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		// Check if the camera supports the FRAME_RATE property
		cout << "Detecting frame rate from camera... " << endl;

		PropertyInfo propInfo;
		propInfo.type = FRAME_RATE;
		error = pCams[i].GetPropertyInfo(&propInfo);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		if (propInfo.present == true)
		{
			// Get the frame rate
			Property prop;
			prop.type = FRAME_RATE;
			error = pCams[i].GetProperty(&prop);
			if (error != PGRERROR_OK)
			{
				PrintError(error);
			}
			else
			{
				// Set the frame rate.
				// Note that the actual recording frame rate may be slower,
				// depending on the bus speed and disk writing speed.
				frameRateToUse = prop.absValue;
			}
		}

		cout << "Using frame rate of " << fixed << setprecision(1) << frameRateToUse
			<< endl;
	}

	for(int i = 0; i < numCams; i++)
	{
		CameraInfo camInfo;
		// Get the camera information
		error = pCams[i].GetCameraInfo(&camInfo);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		ostringstream videoFileName;

		if (i == 0)
		{

			// Motion JPEG videos are always saved with avi containers with or without
			// extensions specified in the filepath
			videoFileName.str("");
			videoFileName.clear();
			videoFileName << "Calibration_cam1-" << camInfo.serialNumber;
			SaveVideoHelper(MJPG, vecImage1, videoFileName.str().c_str(), frameRateToUse);

		}else{

			// Motion JPEG videos are always saved with avi containers with or without
			// extensions specified in the filepath
			videoFileName.str("");
			videoFileName.clear();
			videoFileName << "Calibration_cam2-" << camInfo.serialNumber;
			SaveVideoHelper(MJPG, vecImage2, videoFileName.str().c_str(), frameRateToUse);

		}

	}

	for(int i = 0; i < numCams; i++)
	{
		// Disconnect the camera
		error = pCams[i].Disconnect();
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}
	}

	return 0;
}

int main(int /*argc*/, char ** /*argv*/)
{
    // Since this application saves images in the current folder
    // we must ensure that we have permission to write to this folder.
    // If we do not have permission, fail right away.
    FILE *tempFile = fopen("test.txt", "w+");
    if (tempFile == NULL)
     {
	cout << "Failed to create file in current folder.  Please check "
		"permissions."
		<< endl;
	return -1;
    }
    fclose(tempFile);
    remove("test.txt");

    PrintBuildInfo();

    Error error;

    //
    // Initialize BusManager and retrieve number of cameras detected
    //
    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    cout << "Number of cameras detected: " << numCameras << endl;

    //
    // Check to make sure at least two cameras are connected before
    // running example
    //
    if (numCameras < 2)
    {
        cout << "Insufficient number of cameras." << endl;
        cout << "Make sure at least two cameras are connected for example to "
                "run."
             << endl;
        cout << "Press Enter to exit." << endl;
        cin.ignore();
        return -1;
    }

    //
    // Initialize an array of cameras
    //
    // *** NOTES ***
    // The size of the array is equal to the number of cameras detected.
    // The array of cameras will be used for connecting, configuring,
    // and capturing images.
    //
    Camera *pCameras = new Camera[numCameras];
    
    PGRGuid *guid = new PGRGuid[numCameras];

    for(int i =0; i < numCameras; i++){
	    error = busMgr.GetCameraFromIndex(i, &guid[i]);
	    if (error != PGRERROR_OK)
	    {
		PrintError(error);
		return -1;
	    }
    }

    cout << "Running camera." << endl;
    RunCamera(guid, pCameras, numCameras);

    cout << "Done! Press Enter to exit..." << endl;
    delete[] pCameras;

    cin.ignore();

    return 0;

}

