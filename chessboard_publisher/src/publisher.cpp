/*
 * pulisher.cpp
 *
 *  Created on: 26 Mar 2012
 *      Author: burbrcjc
 */


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class ChessBoardPublisher {
	ros::NodeHandle nh;
	ros::Publisher marker_pub;
	visualization_msgs::Marker board;

public:
	ChessBoardPublisher (ros::NodeHandle &node, std::string name, int squares_width, int squares_height, float sq_width, float sq_height) :
		nh(node){
		marker_pub = nh.advertise<visualization_msgs::Marker>(name, 1);

		board.header.frame_id="/"+name;
		board.type=visualization_msgs::Marker::CUBE_LIST;
//		board.set_points_size(squares_height*squares_width);
		board.scale.x=sq_width;
		board.scale.y=sq_height;
		board.scale.z=0.002; // 1cm thick squares
		board.ns=name;
		board.id = 0;
		board.action = visualization_msgs::Marker::ADD;
	    board.lifetime = ros::Duration();
	    board.pose.position.x = 0;
	    board.pose.position.y = 0;
	    board.pose.position.z = -0.001;
	    board.pose.orientation.x = 0.0;
	    board.pose.orientation.y = 0.0;
	    board.pose.orientation.z = 0.0;
	    board.pose.orientation.w = 1.0;
	    board.color.a=1.0;
	    board.color.r=1.0;
	    board.color.g=1.0;



		float c=0;
		for (int x=0;x <squares_width; x++) {
			float cc=c;
			for (int y=0;y<squares_height; y++) {
				geometry_msgs::Point pt;
				pt.x=x*sq_width - ((squares_width-1)*sq_width)/2.0 ;
				pt.y=y*sq_height - ((squares_height-1)*sq_height)/2.0;
				pt.z=0;
				board.points.push_back(pt);
				std_msgs::ColorRGBA colour;
				colour.a=1.0;
				colour.r=cc;
				colour.g=cc;
				colour.b=cc;
				board.colors.push_back(colour);
				cc=1-cc;
			}
			c=1-c; // change from black to white...
		}
	}

	void publishMarkers() {
	    board.header.stamp = ros::Time::now();
	    marker_pub.publish(board);

	}

	void spin(float pub_freq_hz=10){
		ros::Rate r(pub_freq_hz);
		while (ros::ok()) {
			publishMarkers();
		    r.sleep();
		}
	}
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "chessboard");
  ros::NodeHandle n;

  ros::NodeHandle n_private("~");

  std::string name;
  if (!n_private.getParam("name", name)) {
  	  ROS_ERROR("Missing parameter: board name");
  	  exit(1);
   }

  int dimx,dimy;

  if (!n_private.getParam("grid_x_size", dimx)) {
	  ROS_ERROR("Missing parameter: grid_x_size");
	  exit(1);
  }
  if (dimx < 3) {
	  ROS_ERROR("Param: grid x size must be greater than 2");
	  exit(1);
  }

  if (!n_private.getParam("grid_y_size", dimy)) {
	  ROS_ERROR("Missing parameter: grid_y_size");
	  exit(1);
  }
  if (dimy < 3) {
	  ROS_ERROR("Param: grid y size must be greater than 2");
	  exit(1);
  }


  double rect_size_x,rect_size_y;
  if (!n_private.getParam("rect_x_size", rect_size_x)) {
	  ROS_ERROR("Missing parameter: rect_x_size");
	  exit(1);
  }

  if (!n_private.getParam("rect_y_size", rect_size_y)) {
	  ROS_ERROR("Missing parameter: rect_y_size");
	  exit(1);
  }


  ChessBoardPublisher board(n,name,dimx, dimy,rect_size_x, rect_size_y);
  board.spin();

  return 0;
}
