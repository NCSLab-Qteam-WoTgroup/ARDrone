#include "ardrone/ardrone.h"
#include <fstream>
#include <time.h>
#include <iostream>
using namespace std;
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

typedef pair<int,int> Coord;

void pid(double& x,double& y,const Coord& oldCenter,const Coord& newCenter,int key,IplImage* image,const double z){
	//time derivative
	double dt;
	static clock_t test_clock, last_clock;

	test_clock = clock();
	dt = (test_clock-last_clock)/CLOCKS_PER_SEC;

	//pid
	double error_x = (oldCenter.first - newCenter.first )*z*0.15;
	double error_y = (oldCenter.second- newCenter.second)*z*0.15;

	static double old_error_x = error_x;
	static double old_error_y = error_y;

	static double accu_e_x = 0.0; // I Control
	static double accu_e_y = 0.0;

	double diff_e_x = (error_x - old_error_x)/dt;
	double diff_e_y = (error_y - old_error_y)/dt;

	old_error_x = error_x;
	old_error_y = error_y;

	accu_e_x = accu_e_x + error_x*dt;
	accu_e_y = accu_e_y + error_y*dt;

	enum MODE {
		P_CTRL = 1,
		I_CTRL,
		D_CTRL,
		Q_CTRL
	};

	static double pid_arr [4] = {0.007,0.001,0.0625,0.0};
	static MODE mode = P_CTRL;

	const double step = mode==I_CTRL?0.00001:0.0001;

	switch(key){
	case '1':
	case '2':
	case '3':
	case '4':
		mode = (MODE)(key-'0'); break;
	case '=':
		pid_arr[(int)mode-1] = pid_arr[(int)mode-1]+step;break;
	case '-':
		pid_arr[(int)mode-1] = pid_arr[(int)mode-1]-step;break;
	}
}

int main(int argc, char **argv)
{
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        printf("Failed to initialize.\n");
        return -1;
    }

	// Record data
	fstream file ;
	file.open("C:\\Users\\Austin\\Desktop\\ARDrone\\data\\navdata.txt",ios::out) ;

	// Get a image
    IplImage* image = ardrone.getImage();
	IplImage *display = cvCreateImage(cvSize(image->width,image->height*2), IPL_DEPTH_8U, image->nChannels);

	// Name of video
    char filename[256];
    SYSTEMTIME st;
    GetLocalTime(&st);
    sprintf(filename, "cam%d%02d%02d%02d%02d%02d.wmv", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

    // Create a video writer
    CvVideoWriter *video = cvCreateVideoWriter(filename, CV_FOURCC('D', 'I', 'V', 'X'), 15, cvGetSize(display));

	// Kalman filter
	CvKalman *kalman = cvCreateKalman(4, 2);//4 states, 2 measure

    // Initializing kalman filter matrix
    cvSetIdentity(kalman->measurement_matrix, cvRealScalar(1.0));	// set identity matrix      H
    cvSetIdentity(kalman->process_noise_cov, cvRealScalar(1e-1));	// set process noise = 1e-5 Q
    cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(0.1));// set RealScalar(0.1)      R
    cvSetIdentity(kalman->error_cov_post, cvRealScalar(1.0));		//						 P

    // Linear system  A:4*4
    kalman->DynamMatr[0]  = 1.0; kalman->DynamMatr[1]  = 0.0; kalman->DynamMatr[2]  = 0.0; kalman->DynamMatr[3]  = 0.0;
    kalman->DynamMatr[4]  = 0.0; kalman->DynamMatr[5]  = 1.0; kalman->DynamMatr[6]  = 0.0; kalman->DynamMatr[7]  = 0.0;
    kalman->DynamMatr[8]  = 0.0; kalman->DynamMatr[9]  = 0.0; kalman->DynamMatr[10] = 1.0; kalman->DynamMatr[11] = 0.0;
    kalman->DynamMatr[12] = 0.0; kalman->DynamMatr[13] = 0.0; kalman->DynamMatr[14] = 0.0; kalman->DynamMatr[15] = 1.0;

	// Thresholds
    int min = 173, max = 199;

    // Create a window
    cvNamedWindow("Trackbar");
    cvCreateTrackbar("max", "Trackbar", &max, 255);
    cvCreateTrackbar("min", "Trackbar", &min, 255);
    cvResizeWindow("Trackbar", 0, 0);

	int my=1000,mx=1000;
	int pmx=0,pmy=0;
	bool trackmode = false;
	int loop = 0 ;

	//PID
	Coord oldCenter(320,180),newCenter(320,180);

	// Time
	clock_t start,process;
	clock_t ctrl=0;


	// Battery
	printf("Battery = %d%%\n", ardrone.getBatteryPercentage());

    while (1) {

		start=clock();
		loop++;
		//system("cls") ;

		// Name of image
		char filename[256];
		SYSTEMTIME st;

		GetLocalTime(&st);
		sprintf(filename, "cam%d%02d%02d%02d%02d%02d.jpeg", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

        // Key input
        int key = cvWaitKey(1);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        image = ardrone.getImage();
		cvLine(image,cvPoint(319,174),cvPoint(319,184),CV_RGB(0,255,255),2);
		cvLine(image,cvPoint(314,179),cvPoint(324,179),CV_RGB(0,255,255),2);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode%4);

        // Take off / Landing
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                  ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == CV_VK_UP) vx =  1.0;
        if (key == CV_VK_DOWN) vx = -1.0;
        if (key == CV_VK_LEFT)		vy =  1.0;
        if (key == CV_VK_RIGHT)		vy = -1.0;
        if (key == 'd') vr =  1.0;
        if (key == 'a') vr = -1.0;
        if (key == 'w')		vz =  1.0;
        if (key == 's')		vz = -1.0;
        ardrone.move3D(vx, vy, vz, vr);

        // Binalized image
		IplImage *binalized = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
        CvScalar lower = cvScalar(min);
        CvScalar upper = cvScalar(max);
        cvInRangeS(image, lower, upper, binalized);

        // De-noising
        cvMorphologyEx(binalized, binalized, NULL, NULL, CV_MOP_CLOSE);

        // Detect contours
        CvSeq *contour = NULL, *maxContour = NULL;
        CvMemStorage *contourStorage = cvCreateMemStorage();
        cvFindContours(binalized, contourStorage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        // Find largest contour
        double max_area = 0.0;
        while (contour) {
            double area = fabs(cvContourArea(contour));
            if (area > max_area) {
                maxContour = contour;
                max_area = area;
            }
            contour = contour->h_next;
        }

        // Object detected
        if (maxContour) {
            // Draw a contour
            cvZero(binalized);
            cvDrawContours(binalized, maxContour, cvScalarAll(255), cvScalarAll(255), 0, CV_FILLED);

			// Calculate the moments ­±¿n­«¤ß
		    CvMoments moments;
		    cvMoments(binalized, &moments, 1);
		    my = (int)(moments.m01/moments.m00);
		    mx = (int)(moments.m10/moments.m00);

			// Measurements
	        float m[] = {mx, my};
	        CvMat measurement = cvMat(2, 1, CV_32FC1, m);

	        // Correct phase
			const CvMat *correction = cvKalmanCorrect(kalman, &measurement);
		}

		// Prediction phase
        const CvMat *prediction = cvKalmanPredict(kalman);

		// Draw element
		cvLine(image,cvPoint(prediction->data.fl[0],0),cvPoint(prediction->data.fl[0], 360),CV_RGB(0,0,255),2);
		cvLine(image,cvPoint(0,prediction->data.fl[1]),cvPoint(700,prediction->data.fl[1]),CV_RGB(0,0,255),2);

		// Mode
		if (key == 'v'){
			trackmode = !trackmode;
			file << "Marker" << endl;
		}

        if (key == 'o') ardrone.setAnimation(ARDRONE_ANIM_WAVE,5000);
		if (key == 's') file << "Record" << endl;

		// Controller
		if(trackmode){
			// Show tracking state
			printf("\nTracking mode\n");
			static CvFont font = cvFont(1.0);
            cvPutText(image, "TRACK", cvPoint(10, 20), &font, CV_RGB(255,0,0));


			// Strategy
			/*
			if(maxContour){
				// Error > 75%
				if( mx>559 || mx<79 || my>314 || my<44 ){
					vy = (double)-(prediction->data.fl[0]-319)/320;
					vx = (double)-(prediction->data.fl[1]-179)/180;
				}
				else	{
					vy = (double)-(prediction->data.fl[0]-319)/3200;
					vx = (double)-(prediction->data.fl[1]-179)/1800;
				}
			}*/
			if(maxContour){
				newCenter = Coord(prediction->data.fl[1],prediction->data.fl[0]);
				pid(vx,vy,oldCenter,newCenter,key,image,ardrone.getAltitude());
			}
			else{
				printf("//////// OUT OF REGION ////////\n");
				static CvFont font = cvFont(1.0);
				cvPutText(image, "OUT OF REGION", cvPoint(10, 50), &font, CV_RGB(255,0,0));

				if( ardrone.getAltitude() < 1.5 ){
					vz = 0.5;
				}
				else if (ardrone.getAltitude() > 1.5 && ardrone.getAltitude() < 1.8 && ctrl-clock() <= -4000){
					//ardrone.setAnimation(ARDRONE_ANIM_WAVE,2000);
					ctrl = clock();
				}
				else {
					vz = -0.5;
				}

			}

			ardrone.move3D(vx, vy, vz, vr);
			printf("mX = %d\n",mx);
			printf("mY = %d\n",my);
			printf("Altitude = %.4f\n",ardrone.getAltitude());
			printf("Velocity X = %.4f\n",vx);
			printf("Velocity Y = %.4f\n",vy);
			printf("Velocity Z = %.4f\n\n",vz);
		}

		// Display the image
		cvNamedWindow("Display");
		cvResizeWindow("Display", 640, 720);
		IplImage *mixbinalized = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, image->nChannels);
			// Mix the channel
		cvMerge(binalized,binalized,binalized,0,mixbinalized);
			// Push two images together
		cvSetImageROI(display,cvRect(0,0,mixbinalized->width,mixbinalized->height));
		cvCopy(mixbinalized,display);
		cvResetImageROI(display);
		cvSetImageROI(display,cvRect(0,binalized->height,image->width,image->height+binalized->height));
		cvCopy(image,display);
		cvResetImageROI(display);
		cvShowImage("Display", display);

		// Velocity
        double ardronevx, ardronevy, ardronevz;
        double velocity = ardrone.getVelocity(&ardronevx, &ardronevy, &ardronevz);
        double roll  = ardrone.getRoll();
        double pitch = ardrone.getPitch();
        double yaw   = ardrone.getYaw();

		// Save data
		process=clock()-start;
		file << loop << "\t" << double(process)/ CLOCKS_PER_SEC << "\t"					//Loop		Time
			<< vy << "\t" << vx << "\t" << vz << "\t" << vr << "\t"						//Velocity Input(y,x,z,r)
			<< ardronevy << "\t" << ardronevx << "\t" << ardronevz << "\t"					//Velocity Drone(y,x,z)
			<< roll*RAD_TO_DEG << "\t" << pitch*RAD_TO_DEG << "\t" << yaw*RAD_TO_DEG << "\t"	//Roll Pitch Yaw
			<< ardrone.getAltitude() << "\t"											//Altitude
			<< mx-319 << "\t" << my-179 << "\t"											//Error		image(x,y)
			<< mx << "\t" << my << endl;												//Element		image(x,y)

		// Catch picture
		if(key == 's')
			cvSaveImage(filename,display);

		// Write a frame
        cvWriteFrame(video, display);

		// Release the memories
		cvReleaseMemStorage(&contourStorage);
		cvReleaseImage(&mixbinalized);
		cvReleaseImage(&binalized);

    }//end while

	// Release the kalman filter
    cvReleaseKalman(&kalman);

	// Save the video
    cvReleaseVideoWriter(&video);

    // See you
    ardrone.close();

    return 0;
}
