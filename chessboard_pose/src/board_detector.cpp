/*
 * board_detector.cpp
 *
 *  Created on: 8 Mar 2012
 *      Author: burbrcjc
 *
 *
 */


#include <cstdio>
#include <vector>
#include <ros/ros.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include <image_transport/image_transport.h>

#include <tf/transform_broadcaster.h>


#define sqr(a) ((a)*(a))

using namespace std;
using namespace ros;
using namespace boost;

class ChessBoardDetector {

public:
	struct ChessBoard {
		CvSize griddims; ///< number of squares
		vector<CvPoint3D32f> grid3d;
		vector<CvPoint2D32f> corners;
		double cellwidth;
		double cellheight;
		geometry_msgs::PoseStamped pose;
	} chessboard;

	sensor_msgs::CameraInfo _camInfoMsg;

	ros::Subscriber camInfoSubscriber;
	image_transport::Subscriber imageSubscriber;
	ros::Publisher posePublisher;


	int display, verbose;
	ros::Time lasttime;
	CvMat *intrinsic_matrix; // intrinsic matrices
	boost::mutex mutexcalib;
	IplImage* frame;

	bool publishTF;

	ros::NodeHandle _node_private;
	ros::NodeHandle _node_pub;

	tf::TransformBroadcaster trans_broad;
	tf::Transform transform;

	std::string name;

	/**
	 *  Constructor.
	 */
	ChessBoardDetector() :
			intrinsic_matrix(NULL), frame(NULL), publishTF(false), _node_private("~") {

		_node_private.param("display", display, 1);
		_node_private.param("verbose", verbose, 1);
		_node_private.param("publish_tf", publishTF, true);

//		_node_private.param("name", name, "/chessboard");
		name="chessboard";

		ROS_INFO("Creating chessboard detector.");
		int dimx, dimy;
		double fRectSize[2];

		if (!_node_private.getParam("grid_x_size", dimx)) {
			ROS_ERROR("Missing parameter: grid_x_size");
			return;
		}
		if (dimx < 3) {
			ROS_ERROR("Param: grid x size must be greater than 2");
			return;
		}

		if (!_node_private.getParam("grid_y_size", dimy)) {
			ROS_ERROR("Missing parameter: grid_y_size");
			return;
		}
		if (dimy < 3) {
			ROS_ERROR("Param: grid y size must be greater than 2");
			return;
		}

		if (!_node_private.getParam("rect_x_size", fRectSize[0])) {
			ROS_ERROR("Missing parameter: rect_x_size");
			return;
		}

		if (!_node_private.getParam("rect_y_size", fRectSize[1])) {
			ROS_ERROR("Missing parameter: rect_y_size");
			return;
		}

		ROS_INFO("Board: %d X %d, %f x %f;",dimx,dimy,fRectSize[0],fRectSize[1]);
		chessboard.griddims = cvSize(dimx, dimy);
		chessboard.cellwidth = fRectSize[0];
		chessboard.cellheight = fRectSize[0];

		chessboard.grid3d.resize(dimx * dimy);
		int j = 0;
		for (int y = 0; y < dimy; ++y)
			for (int x = 0; x < dimx; ++x)
				chessboard.grid3d[j++] = cvPoint3D32f(x * fRectSize[0],
						y * fRectSize[1], 0);

		if (display) {
			cvNamedWindow("Chess Board Detector", CV_WINDOW_AUTOSIZE);
		}

		posePublisher = _node_pub.advertise<geometry_msgs::PoseStamped>("/"+name+"_pose", 1);


		this->camInfoSubscriber = _node_pub.subscribe("camera_info", 1,
				&ChessBoardDetector::cameraInfoCallback, this);
		image_transport::ImageTransport it(_node_pub);
		this->imageSubscriber = it.subscribe("image_rect", 1,
				&ChessBoardDetector::imageCallback, this);



	}

	virtual ~ChessBoardDetector() {
		if (frame)
			cvReleaseImage(&frame);
		if (this->intrinsic_matrix)
			cvReleaseMat(&this->intrinsic_matrix);
		this->camInfoSubscriber.shutdown();
		this->imageSubscriber.shutdown();
	}

	// Camera info callback
	void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
		boost::mutex::scoped_lock lock(this->mutexcalib);

		this->_camInfoMsg = *msg;
	}

	// Image data callback
	void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
		boost::mutex::scoped_lock lock(this->mutexcalib);
		if (detectChessBoard(msg, this->_camInfoMsg)) {
			posePublisher.publish(chessboard.pose);
			if (verbose>0){
				ROS_INFO_STREAM("Chess board found, pose: " << chessboard.pose.pose);
			}
			if (publishTF) {
				transform.setOrigin( tf::Vector3(chessboard.pose.pose.position.x,
						chessboard.pose.pose.position.y,
						chessboard.pose.pose.position.z) );
				transform.setRotation( tf::Quaternion(chessboard.pose.pose.orientation.x,
						chessboard.pose.pose.orientation.y,
						chessboard.pose.pose.orientation.z,
						chessboard.pose.pose.orientation.w) );
				trans_broad.sendTransform(tf::StampedTransform(transform, ros::Time::now(), msg->header.frame_id, "/"+name));
			}
		}
	}

	/**
	 *  Detect the chess board in the camera image. Fills in chessboard member.
	 */
	bool detectChessBoard(const sensor_msgs::ImageConstPtr &imagemsg,
			const sensor_msgs::CameraInfo& camInfoMsg) {

		if (this->intrinsic_matrix == NULL)
			this->intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);



		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				this->intrinsic_matrix->data.fl[3 * i + j] = camInfoMsg.P[4 * i + j];

		cv_bridge::CvImageConstPtr image_grey = cv_bridge::toCvShare(imagemsg, "mono8");
//		if (!_cvbridge.fromImage(imagemsg, "mono8")) {
//			ROS_ERROR("failed to get image");
//			return false;
//		}

		IplImage pimggray = IplImage(image_grey->image);
		if (display) {
			// copy the raw image
			if (frame != NULL
					&& (frame->width != (int) imagemsg->width
							|| frame->height != (int) imagemsg->height)) {
				cvReleaseImage(&frame);
				frame = NULL;
			}

			if (frame == NULL)
				frame = cvCreateImage(cvSize(imagemsg->width, imagemsg->height),
						IPL_DEPTH_8U, 3);

			cvCvtColor(&pimggray, frame, CV_GRAY2RGB);
		}

		chessboard.corners.resize(200);
		int ncorners;
		int allfound = cvFindChessboardCorners(&pimggray, chessboard.griddims,
				&chessboard.corners[0], &ncorners, CV_CALIB_CB_ADAPTIVE_THRESH);
		chessboard.corners.resize(ncorners);

		if (display) {
			cvDrawChessboardCorners(frame,  chessboard.griddims, &chessboard.corners[0], ncorners, allfound);
			cvShowImage("Chess Board Detector", frame);
			cvWaitKey(10);
		}

		if (!allfound || ncorners != (int) chessboard.grid3d.size()){
			return false;
		}

		// remove any corners that are close to the border
		const int borderthresh = 30;

		for (int j = 0; j < ncorners; ++j) {
			int x = chessboard.corners[j].x;
			int y = chessboard.corners[j].y;
			if (x < borderthresh || x > pimggray.width - borderthresh
					|| y < borderthresh
					|| y > pimggray.height - borderthresh) {
				allfound = 0;
				return false;
			}
		}

		if (allfound) {
			cvFindCornerSubPix(&pimggray, &chessboard.corners[0],
					chessboard.corners.size(), cvSize(5, 5), cvSize(-1, -1),
					cvTermCriteria(CV_TERMCRIT_ITER, 20, 1e-2));

			chessboard.pose.pose = calculateTransformation(chessboard.corners, chessboard.grid3d, chessboard);

			chessboard.pose.header.stamp = imagemsg->header.stamp;
			chessboard.pose.header.frame_id = imagemsg->header.frame_id;

			return true;

		} else
			return false;

	}

	/**
	 *  Return the position of the chess board in the camera frame.
	 */
	geometry_msgs::Pose calculateTransformation(const vector<CvPoint2D32f> &imgpts, const vector<CvPoint3D32f> &objpts, ChessBoard &cb) {

		geometry_msgs::Pose pose;

		cv::Mat_<float> board_points(objpts.size(),3, const_cast<float*>(&(objpts[0].x)));
		cv::Mat_<float> image_points(imgpts.size(), 2, const_cast<float*>(&(imgpts[0].x)));

		cv::Mat_<double> R(3,1);
		cv::Mat_<double> T(3,1);

		float zero=0.0;
		cv::Mat_<float> distortion(4, 1, zero);

		cv::solvePnP(board_points, image_points, cv::Mat(this->intrinsic_matrix), distortion, R, T);

		pose.position.x=T(0);
		pose.position.y=T(1);
		pose.position.z=T(2);


		// Quaternion from angle-axis
		float angle;
		angle=-sqrt(R(0)*R(0) + R(1)*R(1) + R(2)*R(2))/2.0;
		R/=sqrt(R(0)*R(0) + R(1)*R(1) + R(2)*R(2));
		pose.orientation.w=cos( angle);
		pose.orientation.x=sin(angle)*R(0);
		pose.orientation.y=sin(angle)*R(1);
		pose.orientation.z=sin(angle)*R(2);

		return pose;
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "chessboard_pose");
	if (!ros::master::check())
		return 1;

	ChessBoardDetector cd;
	ros::spin();

	return 0;
}
