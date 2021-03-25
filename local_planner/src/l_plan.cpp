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
		int left_bound, right_bound;
		std::array<int,2> vert_bound, forward_bound;

		std::vector<std::vector<int>> map_mat;

		local_planner::CmapClear bools;

	public:
		Robot(ros::NodeHandle *nh)
		{	
			CostMap = nh->subscribe("/semantics/costmap_generator/occupancy_grid", 1000, &Robot::mapCallback, this);
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
			fpCoordinates();
			int n = 0;
			for (int i=0; i<(int)len_x; i++)
			{	
				std::vector<int> v = {};
				for (int j=0; j<(int)len_y; j++)
				{	
					v.push_back(data_pts[n]);
					n++;
				}
				map_mat.push_back(v);
				//std::cout << n << std::endl;
			}
			std::cout << "checking clear" << std::endl;
			checkclear();

		}

		void fpCoordinates()
		{
			float centre[1][2];
			centre[0][0] = len_x/2;
			centre[0][1] = len_y/2;

			float horz_dist = width/2; // x axis
			float vert_dist = length/2; // y axis

			int horz_pix = (int)ceil(horz_dist/res);
			int vert_pix = (int)ceil(vert_dist/res);

			left_bound = (int)ceil(len_x/2) - vert_pix;
			right_bound = (int)round(len_x/2) + vert_pix;
			vert_bound = {(int)round(len_x/2) - vert_pix, (int)ceil(len_x/2) + vert_pix};

			forward_bound = {left_bound, right_bound};

			buffer = (int)ceil(safety_dist/res);
			printf("footprinted\n");
			std::cout << left_bound << std::endl;
			std::cout << right_bound << std::endl;
		}

		void checkclear()
		{	
			bool left_clear=true, right_clear=true, up_clear=true;
			// right clear
			printf("checking1\n");
			for (int i = right_bound; i <= right_bound + buffer; i++)
			{	
				if (right_clear == false)
				{
					break;
				}
				for (int j = vert_bound[0] - buffer; j <= vert_bound[1] + buffer; j++)
				{	
					if (map_mat[j][i] > 0)
					{
						right_clear = false;
						break;
					}
				}
			}
			// up clear
			printf("checking2\n");
			for (int i = left_bound - buffer; i <= right_bound + buffer; i++)
			{	
				if (up_clear == false)
				{
					break;
				}
				for (int j = vert_bound[1]; j <= vert_bound[1] + buffer; j++)
				{
					if (map_mat[j][i] > 0)
					{
						up_clear = false;
					}
				}
			}

			// left clear
			printf("checking3\n");
			for (int i = left_bound - buffer; i <= left_bound; i++)
			{	
				//std::cout << i << std::endl;
				if (left_clear == false)
				{
					break;
				}
				for (int j = vert_bound[0]; j <= vert_bound[1] + buffer; j++)
				{
					if (map_mat[j][i] > 0)
					{
						left_clear = false;
						break;
					}
				}
			}
			printf("checked\n");
			auto* bl = &bools;
			bl->right = right_clear;
			bl->left = left_clear;
			bl->up = up_clear;
			cmap_clear.publish(*bl);
			std::cout << *bl << std::endl;
			
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