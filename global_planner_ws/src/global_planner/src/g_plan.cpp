#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <grid_map_msgs/GridMap.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

class Robot
{
	private:
		ros::Subscriber CostMap;
		ros::Subscriber RobotPose;
		ros::Publisher CmdVelPub;

		std::string curr_state="stop", prev_state="stop";
		float length, width;

	public:
		Robot(ros::NodeHandle *nh)
		{
			CostMap = nh->subscribe("map", 1000, &Robot::mapCallback, this);
			RobotPose = nh->subscribe("pose", 1000, &Robot::checkObs, this);
			CmdVelPub = nh->advertise<geometry_msgs::Twist>("panthera_cmd", 10);
			length = nh->getParam("/robot_length", length);
			width = nh->getParam("/robot_width", width);
		}

		void mapCallback(const grid_map_msgs::GridMap& msg)
		{	
			double len_x = msg.info.length_x;
			double len_y = msg.info.length_y;
			double res = msg.info.resolution;
			auto data_pts = msg.data[0].data;

			double map_mat[(int)len_y][(int)len_x];

			int n = 0;
			for (int i=0; i<(int)len_x; i++)
			{	
				for (int j=0; j<(int)len_y; j++)
				{	
					map_mat[j][i] = data_pts[n];
					n++;
				}
			}
		}

		void checkObs(const geometry_msgs::Pose& msg)
		{
			tf::Quaternion q(msg.orientation.x,
					         msg.orientation.y,
					         msg.orientation.z,
					         msg.orientation.w);
		    tf::Matrix3x3 m(q);
		    double roll, pitch, yaw;
		    m.getRPY(roll, pitch, yaw);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "global_planner_node");
	ros::NodeHandle nh;

	Robot Panthera = Robot(&nh);
	ros::spin();
	return 0;
}

