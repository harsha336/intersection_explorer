#include <math.h>

#include <iostream>
#include <string.h>
#include <fstream>
#include <sys/stat.h>
#include <ctime>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
//#include "laser_geometry/laser_geometry.h"
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <filters/filter_chain.h>
#include <intersection_explorer/IPlanner.h>

#define ERR_MSG_FILE_HANDLING -1004
#define ERR_MSG_TIMEOUT -1005
#define ERR_MSG_UNKNOWN -1006


namespace intersection_explorer
{
class Explorer
{
	//constants
	float MAX_RANGE = 1.5;
	enum dir{CW, CCW};
	enum pathType{STAR=1, ROTATE};

	// update flags
	bool service_req_;
	bool scan_updated_;
	bool odom_updated_;
	bool robot_moving_;
	float max_move_dist_;
	ros::NodeHandle nh_private_, nh_;
	tf::TransformListener tf_;

	//message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
	ros::Subscriber laser_sub_;
	ros::Subscriber odom_sub_;

	ros::Publisher vel_pub_;
	ros::ServiceServer service_;
	std::string in_file_, out_file_;

	std::string odom_, scan_topic_, base_link_;

	geometry_msgs::Pose cur_pose_;
	
	int img_count_;
	std::string inter_type_;
	std::string req_file_;

	float angular_speed_, linear_speed_;

	tf::MessageFilter<sensor_msgs::LaserScan> *laser_notifier_;
	//filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;

    	public: Explorer();
	public: ~Explorer();
	protected: void scanCallback ( const sensor_msgs::LaserScan::ConstPtr scan_in);
	protected: void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	protected: bool handleRequest(intersection_explorer::IPlanner::Request &req,
				intersection_explorer::IPlanner::Response &resp);
	protected: void clearVelocityMessage(geometry_msgs::Twist &vel_msg);
	protected: bool moveToInitPose(geometry_msgs::Pose goal);
	protected: float moveStraight(float dist);
	protected: void rotate(int direction, float angle);
	protected: int createAndWaitStatus(std::string &r);
	protected: float computeDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
			{
				float dist;
				dist = sqrt(pow((a.position.y - b.position.y), 2)+
					    pow((a.position.x - b.position.x), 2));
				return dist;
			}
	protected: float getYawAngle(geometry_msgs::Pose msg)
			{
				tf::Quaternion q(msg.orientation.x,
						 msg.orientation.y,
						 msg.orientation.z,
						 msg.orientation.w);
				tf::Matrix3x3 m(q);
				double roll, pitch, yaw;
				m.getRPY(roll, pitch, yaw);
				return yaw;
			}
	protected: float getAngleToGoal(geometry_msgs::Pose a, geometry_msgs::Pose b)
			{
				float dy = a.position.y - b.position.y;
				float dx = a.position.x - b.position.x;
				float ang = atan(dy/dx);
				return ang;
			}
	protected: geometry_msgs::PoseStamped convertToStamped(geometry_msgs::Pose a)
			{
				geometry_msgs::PoseStamped ret;
				ret.pose = a;
				ret.header.seq = 0;
				ret.header.stamp = ros::Time::now();
				ret.header.frame_id = odom_;
				return ret;
			}

};
static const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;

// Floating-point modulo
// The result (the remainder) has same sign as the divisor.
// Similar to matlab's mod(); Not similar to fmod() -   Mod(-3,4)= 1   fmod(-3,4)= -3
template<typename T>
T Mod(T x, T y)
{
    static_assert(!std::numeric_limits<T>::is_exact , "Mod: floating-point type expected");

    if (0. == y)
        return x;

    double m= x - y * floor(x/y);

    // handle boundary cases resulted from floating-point cut off:
    if (y > 0)              // modulo range: [0..y)
    {
        if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
            return 0;
        if (m<0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14 
        }
    }
    else                    // modulo range: (y..0]
    {
        if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
            return 0;

        if (m>0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14 
        }
    }
    return m;
}

// wrap [rad] angle to [-PI..PI)
inline double WrapPosNegPI(double fAng)
{
    return Mod(fAng + _PI, _TWO_PI) - _PI;
}

// wrap [rad] angle to [0..TWO_PI)
inline double WrapTwoPI(double fAng)
{
    return Mod(fAng, _TWO_PI);
}

// wrap [deg] angle to [-180..180)
inline double WrapPosNeg180(double fAng)
{
    return Mod(fAng + 180., 360.) - 180.;
}

// wrap [deg] angle to [0..360)
inline double Wrap360(double fAng)
{
    return Mod(fAng ,360.);
}
}
