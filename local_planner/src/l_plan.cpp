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
#include <local_planner/CmapClear.h>

/** PARAMS:
	- length of robot
	- width of robot
**/

class Robot
{
	private:
		ros::Subscriber CostMap;
		ros::Subscriber RobotPose;
		ros::Publisher CmdVelPub;
		ros::Publisher cmap_clear;

		// Footprint info
		float length, width;

		float safety_dist = 0.5;
		int buffer; // number of squares safety distance

		// Map info
		double len_x, len_y, res;
		int left_bound_x, right_bound_x;
		std::array<int,2> vert_bound, forward_bound_x;

	public:
		Robot(ros::NodeHandle *nh)
		{	
			CostMap = nh->subscribe("/map", 1000, &Robot::mapCallback, this);
			CmdVelPub = nh->advertise<geometry_msgs::Twist>("panthera_cmd",100);
			cmap_clear = nh->advertise<local_planner::CmapClear>("check_cmap",100);

			length = nh->getParam("/robot_length", length);
			width = nh->getParam("/robot_width", width);
		}

		void mapCallback(const nav_msgs::OccupancyGrid& msg)
		{	
			len_x = msg.info.width;
			len_y = msg.info.height;
			res = msg.info.resolution;
			auto data_pts = msg.data;

			double** map_mat;

			int n = 0;
			for (int i=0; i<(int)len_x; i++)
			{	
				for (int j=0; j<(int)len_y; j++)
				{	
					map_mat[j][i] = data_pts[n];
					n++;
				}
			}

			checkclear(map_mat);

		}

		void fpCoordinates()
		{
			float centre[1][2];
			centre[0][0] = len_x/2;
			centre[0][1] = len_y/2;

			float horz_dist = width/2;
			float vert_dist = length/2;

			int horz_pix = (int)ceil(horz_dist/res);
			int vert_pix = (int)ceil(vert_dist/res);

			left_bound_x = (int)round(len_x/2) - horz_pix;
			right_bound_x = (int)ceil(len_x/2) - horz_pix;
			vert_bound = {(int)round(len_y/2) - vert_pix, (int)ceil(len_y/2) + vert_pix};

			forward_bound_x = {left_bound_x, right_bound_x};

			buffer = (int)ceil(safety_dist/res);
		}
		
		void checkclear(double** costmap)
		{	
			bool left_clear=true, right_clear=true, up_clear=true;
			// right clear
			for (int i = right_bound_x; i <= right_bound_x + buffer; i++)
			{	
				if (right_clear == false)
				{
					break;
				}
				for (int j = vert_bound[0] - buffer; j <= vert_bound[1] + buffer; j++)
				{
					if (costmap[i][j] > 0)
					{
						right_clear = false;
						break;
					}
				}
			}

			// up clear
			for (int i = left_bound_x - buffer; i <= right_bound_x + buffer; i++)
			{	
				if (up_clear == false)
				{
					break;
				}
				for (int j = vert_bound[1]; j <= vert_bound[1] + buffer; j++)
				{
					if (costmap[i][j] > 0)
					{
						up_clear = false;
					}
				}
			}

			// left clear
			for (int i = left_bound_x; i >= left_bound_x + buffer; i--)
			{	
				if (left_clear == false)
				{
					break;
				}
				for (int j = vert_bound[1]; j <= vert_bound[1] + buffer; j++)
				{
					if (costmap[i][j] > 0)
					{
						left_clear = false;
						break;
					}
				}
			}
			local_planner::CmapClear bools;
			bools.right = right_clear;
			bools.left = left_clear;
			bools.up = up_clear;
			cmap_clear.publish(bools);
			
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_planner_node");
	ros::NodeHandle nh;

	Robot Panthera = Robot(&nh);
	ros::spin();
	return 0;
}