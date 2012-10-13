#ifndef _MAIN_CPP_
#define _MAIN_CPP_
/*
 main.cpp
 Testing processing of ardrone_autonomy services
 Sarah Tang and Edward Francis Kelley V
 Senior thesis, 2012-2013
 */

/*
 * C++ INCLUDES
 */
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

/* 
 * ROS INCLUDES
 */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

/*
 * ARDRONE_AUTONOMY INCLUDES
 */
#include "ardrone_autonomy/Navdata.h"

/*
 * DEFINITIONS
 */
#define MY_MASK 0777 //for setting folder permissions when using mkdir
//path to log files, make sure trailing / is there
 //CHANGE TO GLOBAL VAR TODO
#define ROS_WORKSPACE "/home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterCode/bin/" 

/*
 * GLOBAL VARIABLES
 */
std::fstream navLog; //name of log file
long programStart; //time of program start

/*
 FUNCTION DECLARATIONS
 */
long myclock();
void navDataCB(const ardrone_autonomy::Navdata::ConstPtr& msg);

/*
 * FUNCTIONS
 */

/* 
 * Returns a time value.
 */
long myclock() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000000) + tv.tv_usec;
}

/* 
 * Callback function when navdata is updated. Logs navData in logfile. 
 */
void navDataCB(const ardrone_autonomy::Navdata::ConstPtr& msg) {
	//get elapsed time
   	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	//log all variables
	navLog << "time= " << currentTime << " droneTime= " << msg->tm << " batteryPrecent= " << msg->batteryPercent << " state= " << msg->state << " rot= " << msg->rotX << " " << msg->rotY << " " << msg->rotZ << " altd= " << msg->altd << " linearV= " << msg->vx << " " << msg->vy << " " << msg->vz << " " << " linearAccel " << msg->ax << " " << msg->ay << " " << msg->az << "\n";
}

/* 
 * Callback function when new image is recieved in front-facing camera. Stored as .jpgs in folder.  
 */
void imgCB(const sensor_msgs::ImageConstPtr& msg)
{
	//convert image from ros image to open CV image
	sensor_msgs::CvBridge bridge;
	IplImage* img;
  	try {
		img = bridge.imgMsgToCv(msg, "bgr8");
  	}
  	catch (sensor_msgs::CvBridgeException& e) {
    		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	} 

	//display image
	cvShowImage("view", img);

	//get elapsed time
	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	//save .jpg with timestamped name in ROS_WORKSPACE/[programStart]
	std::stringstream time;
	time << ROS_WORKSPACE << programStart << "/" << currentTime << ".jpg";
	const std::string tmp = time.str();
	const char* timeStamp = tmp.c_str();

	cvSaveImage(timeStamp, img);
}

int main(int argc, char** argv) {
	//INITIALIZE LOGGING
	//get current time
   	programStart = myclock();

	//make directory for images, named ROS_WORKSPACE/[programStart]
	std::stringstream imgDir;
	imgDir << ROS_WORKSPACE << programStart;
	const std::string tmp = imgDir.str();
	const char* imgDirectory = tmp.c_str();

  	int temp;
	temp = umask(0);
  	if ((temp = mkdir(imgDirectory, MY_MASK)) != 0) {
    		fprintf(stderr, "ERROR %d: unable to mkdir; %s\n", errno, strerror(errno));
  	}

	//make log file, named ROS_WORKSPACE/navLog_[programStart]
	std::stringstream navLogStr;
	navLogStr << ROS_WORKSPACE << "navLog" << programStart << ".txt";
	const std::string tmp2 = navLogStr.str();
	const char* navLogName = tmp2.c_str();

	navLog.open(navLogName, std::ios_base::out);

	//INITALIZE ROS NODES
	ros::init(argc, argv, "test_ardrone_autonomy");
	ros::NodeHandle n;

	//command take off, hover for two seconds, then land
	//not sure why this isn't working
	ROS_INFO("Taking off..");	
	ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
	std_msgs::Empty empty;
	takeoff.publish(empty);
	ros::spinOnce();

	ros::Duration(2).sleep(); //sleep for 2 seconds

	ROS_INFO("Landing..");
	ros::Publisher land = n.advertise<std_msgs::Empty>("/ardrone/land", 1000);
	land.publish(empty);
	ros::spinOnce();

	//subscribe to camera feeds
	image_transport::ImageTransport it(n);
	image_transport::Subscriber frontCam_sub = it.subscribe("ardrone/front/image_raw", 1, imgCB); //foward camera
	//downward camera - no images?
	//image_transport::Subscriber bottomCam_sub = it.subscribe("ardrone/bottom/image_raw", 1, imgCB); 

	//subscribe to navdata
	ros::Subscriber navdata_sub = n.subscribe("ardrone/navdata", 1000, navDataCB);

	//make openCV windows
	cvNamedWindow("view");
  	cvStartWindowThread();

	while (ros::ok()) {
		//process ros messages
		ros::spin();
	}
	
	//exit cleanly
	cvDestroyWindow("view");
	navLog.close();

	return 0;

}

//GUI stuff
static void hello( GtkWidget *widget,
                   gpointer   data )
{
    g_print ("Hello World\n");
}

static gboolean delete_event( GtkWidget *widget,
                              GdkEvent  *event,
                              gpointer   data )
{
    /* If you return FALSE in the "delete-event" signal handler,
     * GTK will emit the "destroy" signal. Returning TRUE means
     * you don't want the window to be destroyed.
     * This is useful for popping up 'are you sure you want to quit?'
     * type dialogs. */

    g_print ("delete event occurred\n");

    /* Change TRUE to FALSE and the main window will be destroyed with
     * a "delete-event". */

    return TRUE;
}

int main( int   argc,
          char *argv[] )
{
    /* GtkWidget is the storage type for widgets */
    GtkWidget *window;
    GtkWidget *button;
    
    /* This is called in all GTK applications. Arguments are parsed
     * from the command line and are returned to the application. */
    gtk_init (&argc, &argv);
    
    /* create a new window */
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    
    /* When the window is given the "delete-event" signal (this is given
     * by the window manager, usually by the "close" option, or on the
     * titlebar), we ask it to call the delete_event () function
     * as defined above. The data passed to the callback
     * function is NULL and is ignored in the callback function. */
    g_signal_connect (window, "delete-event",
		      G_CALLBACK (delete_event), NULL);
    
    /* Here we connect the "destroy" event to a signal handler.  
     * This event occurs when we call gtk_widget_destroy() on the window,
     * or if we return FALSE in the "delete-event" callback. */
    g_signal_connect (window, "destroy",
		      G_CALLBACK (destroy), NULL);
    
    /* Sets the border width of the window. */
    gtk_container_set_border_width (GTK_CONTAINER (window), 10);
    
    /* Creates a new button with the label "Hello World". */
    button = gtk_button_new_with_label ("Hello World");
    
    /* When the button receives the "clicked" signal, it will call the
     * function hello() passing it NULL as its argument.  The hello()
     * function is defined above. */
    g_signal_connect (button, "clicked",
		      G_CALLBACK (hello), NULL);
    
    /* This will cause the window to be destroyed by calling
     * gtk_widget_destroy(window) when "clicked".  Again, the destroy
     * signal could come from here, or the window manager. */
    g_signal_connect_swapped (button, "clicked",
			      G_CALLBACK (gtk_widget_destroy),
                              window);
    
    /* This packs the button into the window (a gtk container). */
    gtk_container_add (GTK_CONTAINER (window), button);
    
    /* The final step is to display this newly created widget. */
    gtk_widget_show (button);
    
    /* and the window */
    gtk_widget_show (window);
    
    /* All GTK applications must have a gtk_main(). Control ends here
     * and waits for an event to occur (like a key press or
     * mouse event). */
    gtk_main ();
    
    return 0;
}


#endif //_CUSTOMUSER_CPP_
