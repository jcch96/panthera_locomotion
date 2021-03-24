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

/** PARAMS:
	- length of robot
	- width of robot
	- costmap_topic
**/

class Robot
{
	private:
		ros::Subscriber CostMap;
		ros::Subscriber RobotPose;
		ros::Publisher CmdVelPub;

		std::string costmap_topic;

		int curr_state, prev_state;

		geometry_msgs:: Twist cmd;

		// Footprint info
		float length, width;
		//float front_left, front_right, back_left, back_right;

		float safety_dist = 0.5;
		int buffer; // number of squares safety distance

		// Map info
		double len_x, len_y, res;
		int left_bound_x, right_bound_x;
		std::array<int,2> vert_bound, forward_bound_x;

		// robot speeds
		double vx=0.1, vy=0.1, wz=0.5;

	public:
		Robot(ros::NodeHandle *nh)
		{	
			costmap_topic = nh->getParam("/costmap_topic", costmap_topic);
			CostMap = nh->subscribe(costmap_topic, 1000, &Robot::mapCallback, this);
			CmdVelPub = nh->advertise<geometry_msgs::Twist>("panthera_cmd",100);
			length = nh->getParam("/robot_length", length);
			width = nh->getParam("/robot_width", width);
			//front_left = nh->getParam("/footprint/front_left", front_left);
			//front_right = nh->getParam("/footprint/front_right", front_right);
			//back_left = nh->getParam("/footprint/back_left", back_left);
			//back_right = nh->getParam("/footprint/back_right", back_right);
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

			bool isClear;
			if (curr_state == 1)
			{
				isClear = checkclear(curr_state, map_mat);
			}
			else if (curr_state == 2)
			{
				isClear = checkclear(curr_state, map_mat);
			}
			else if (curr_state == 3)
			{
				isClear = checkclear(curr_state, map_mat);
			}

			if (isClear == false)
			{
				stop();
				ros::Duration(2).sleep();
				if (curr_state == 1)
				{
					prev_state = curr_state;
					curr_state = 2;
					up();
				}

				else if (curr_state == 3)
				{
					prev_state = curr_state;
					curr_state = 2;
					up();
				}

				else if (curr_state == 2)
				{
					if (prev_state == 1)
					{
						prev_state = curr_state;
						curr_state = 3;
						left();
					}
					else if (prev_state == 3)
					{
						prev_state = curr_state;
						curr_state = 1;
						right();
					}
				}
			}

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
		
		bool checkclear(int state, double** costmap)
		{
			// moving right
			if(state == 1)
			{
				for (int i = right_bound_x; i <= right_bound_x + buffer; i++)
				{
					for (int j = vert_bound[0] - buffer; j <= vert_bound[1] + buffer; j++)
					{
						if (costmap[i][j] > 0)
						{
							return false;
						}
					}
				}
			}

			// moving up
			else if(state == 2)
			{
				for (int i = left_bound_x - buffer; i <= right_bound_x + buffer; i++)
				{
					for (int j = vert_bound[1]; j <= vert_bound[1] + buffer; j++)
					{
						if (costmap[i][j] > 0)
						{
							return false;
						}
					}
				}
			}

			// moving left
			else if(state == 3)
			{
				for (int i = left_bound_x; i >= left_bound_x + buffer; i--)
				{
					for (int j = vert_bound[1]; j <= vert_bound[1] + buffer; j++)
					{
						if (costmap[i][j] > 0)
						{
							return false;
						}
					}
				}
			}

			return true;
		}

		// Twist commands
		void stop()
		{
			geometry_msgs::Twist* ts = &cmd;
			ts->linear.x = 0;
			ts->linear.y = 0;
			ts->angular.z = 0;
			CmdVelPub.publish(*ts);
		}

		void right()
		{
			geometry_msgs::Twist* ts = &cmd;
			ts->linear.x = 0;
			ts->linear.y = -vy;
			ts->angular.z = 0;
			CmdVelPub.publish(*ts);
		}

		void up()
		{
			geometry_msgs::Twist* ts = &cmd;
			ts->linear.x = vx;
			ts->linear.y = 0;
			ts->angular.z = 0;
			CmdVelPub.publish(*ts);
		}

		void left()
		{
			geometry_msgs::Twist* ts = &cmd;
			ts->linear.x = 0;
			ts->linear.y = vy;
			ts->angular.z = 0;
			CmdVelPub.publish(*ts);
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