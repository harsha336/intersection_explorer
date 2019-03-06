#include <intersection_explorer/explorer.hh>

intersection_explorer::Explorer::Explorer() :
    nh_private_("~"),
    service_req_(false),
    scan_updated_(false),
    odom_updated_(false),
    tf_(),
    filter_chain_("sensor_msgs::LaserScan")
{
	ROS_INFO("Explorer::Explorer:: Created the explorer node!");

	nh_private_.param("odom_frame", odom_, std::string("odom"));
	nh_private_.param("scan_topic", scan_topic_, std::string("scan_filtered"));

	//laser_sub_.subscribe(nh_, scan_topic_, 10);
	//laser_notifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, base_link_.c_str(), 10);
	//laser_notifier_->registerCallback(
	//	boost::bind(&intersection_explorer::Explorer::scanCallback, this, _1));
	//filter_chain_.configure("scan_filter_chain");
	//ros::Subscriber sub = nh_.subscribe(odom_, 1000, &Explorer::odomCallback, this);
}

void intersection_explorer::Explorer::scanCallback ( const sensor_msgs::LaserScan::ConstPtr scan_in)
{
	if( !service_req_ )
	{
		scan_updated_ = false;
		return;
	}
	sensor_msgs::LaserScan scan;
	//filter_chain_.update(*scan_in, scan);
	if(scan.ranges.empty()) return;

	float delta_theta = scan.angle_increment;
	unsigned int zero_theta = (scan.angle_max - scan.angle_min)/delta_theta;

	unsigned int iterations = (30.0 / 180.0 *M_PI) / delta_theta;
	unsigned int i = zero_theta - (iterations/2);
	unsigned int max_i = i + iterations;

	float min_space = MAX_RANGE;
	for( ; i < max_i ; i++)
	{
		if( scan.ranges[i] < min_space )
			min_space = scan.ranges[i];
	}
	max_move_dist_ = min_space;
	scan_updated_ = true;

	ROS_DEBUG_STREAM( "Explorer::scanCallback: Max moving space is: " <<
				max_move_dist_ );
}

void intersection_explorer::Explorer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if( !service_req_ )
	{
		odom_updated_ = false;
		return;
	}

	cur_pose_ = msg->pose.pose;
	odom_updated_ = false;

	ROS_DEBUG( "Explorer::odomCallback: Current pose: (%f, %f):(%f)",
			cur_pose_.position.x, cur_pose_.position.y, 
			cur_pose_.orientation.z);
}

intersection_explorer::Explorer::~Explorer()
{
}
