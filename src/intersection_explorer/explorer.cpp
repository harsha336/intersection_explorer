#include <intersection_explorer/explorer.hh>

intersection_explorer::Explorer::Explorer() :
    nh_private_("~"),
    service_req_(false),
    scan_updated_(false),
    odom_updated_(false),
    robot_moving_(false),
    img_count_(0),
    tf_()
    //filter_chain_("sensor_msgs::LaserScan")
{
	ROS_INFO("Explorer::Explorer:: Created the explorer node!");

	nh_private_.param("odom_frame", odom_, std::string("/RosAria/pose"));
	nh_private_.param("scan_topic", scan_topic_, std::string("scan"));
	nh_private_.param("base_link_frame", base_link_, std::string("base_link"));

	angular_speed_ = 0.1;
	linear_speed_ = 0.5;

	//laser_sub_.subscribe(nh_, scan_topic_, 10);
	//laser_notifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, base_link_.c_str(), 10);
	//laser_notifier_->registerCallback( boost::bind(&intersection_explorer::Explorer::scanCallback, this, _1) );
	
	//filter_chain_.configure("scan_filter_chain");
	laser_sub_ = nh_.subscribe(scan_topic_, 1000, &Explorer::scanCallback, this);
	odom_sub_ = nh_.subscribe(odom_, 1000, &Explorer::odomCallback, this);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	service_ = nh_.advertiseService("planner", &Explorer::handleRequest, this);

	req_file_ = "/tmp/mat_request";
	out_file_ = "/tmp/mat_response";
}

void intersection_explorer::Explorer::scanCallback ( const sensor_msgs::LaserScan::ConstPtr scan)
{
	if( !service_req_ )
	{
		scan_updated_ = false;
		return;
	}
	//sensor_msgs::LaserScan scan;
	//filter_chain_.update(*scan_in, scan);
	if(scan->ranges.empty()) return;

	float delta_theta = scan->angle_increment;
	unsigned int zero_theta = scan->ranges.size() / 2;

	unsigned int iterations = (90.0 / 180.0 *M_PI) / delta_theta;
	unsigned int i = zero_theta - (iterations/2);
	unsigned int max_i = i + iterations;
	ROS_DEBUG("Zero theta: %d, iterations: %d, max_i: %d", zero_theta, iterations,
			max_i);

	float min_space = MAX_RANGE;
	for( ; i < max_i ; i++)
	{
		if( scan->ranges[i] < min_space )
		{
			min_space = scan->ranges[i];
			ROS_DEBUG("Setting space to: %f", min_space);
		}
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
	odom_updated_ = true;

	ROS_DEBUG( "Explorer::odomCallback: Current pose: (%f, %f):(%f)",
			cur_pose_.position.x, cur_pose_.position.y, 
			cur_pose_.orientation.z);
}

bool intersection_explorer::Explorer::handleRequest(intersection_explorer::IPlanner::Request &req, intersection_explorer::IPlanner::Response &resp)
{
	inter_type_ = req.name;
    	service_req_ = true;
	ROS_INFO("intersection_explorer::Explorer::handleRequest: Entered");
    	ros::spinOnce();
    	while( !odom_updated_ || !scan_updated_ )
    	{
    		//ROS_INFO("Pose should have updated! scan: [%d] Odom: [%d]", 
				//scan_updated_, odom_updated_);
		// Fill resp with error code
		//service_req_ = false;
		//return false;
		ros::spinOnce();
    	}
	ROS_INFO("intersection_explorer::Explorer::handleRequest: Done");

	geometry_msgs::Pose init_pose = cur_pose_;
    	/*geometry_msgs::PoseStamped init_pose;
	init_pose.pose = cur_pose_;
	init_pose.header.seq = 0;
	init_pose.header.stamp = ros::Time::now();
	init_pose.header.frame_id = odom_;*/
    	if( req.shape == STAR )
    	{
	    	rotate(CW, M_PI/2, false);
	    	//rotate(CCW, M_PI/2);
		ROS_INFO("==========");
		float trav_dist;
	    	for(unsigned int i = 0;i< 6;i++)
	    	{
			trav_dist = moveStraight(max_move_dist_);
			rotate(CW, M_PI/2, false);
			rotate(CW, M_PI/2, false);
			moveStraight(trav_dist);
			//moveToInitPose(init_pose);
			rotate(CW, M_PI/2, false);
			rotate(CW, M_PI/4, false);
			//moveStraight(max_move_dist_);
			//moveToInitPose(init_pose);
			ROS_INFO("===========");
	    	}
		trav_dist = moveStraight(max_move_dist_);
		rotate(CW, M_PI/2, false);
		rotate(CW, M_PI/2, false);
		moveStraight(trav_dist);

		ROS_INFO_STREAM("Images taken are : " << img_count_);
		ROS_INFO("Requesting label of intersection");
		std::string ret;
		resp.ret_code = createAndWaitStatus(ret);
		resp.label = ret;
		/*if(createAndWaitStatus(ret) < )
		{
			resp.ret_code = 1;
			resp.label = ret;
			return true;
		}
		else
		{
			resp.ret_code = -2;
			resp.label = ret;
		}*/
		bool rc = (resp.ret_code > 0) ? true : false;
		img_count_ = 0;
		return rc;
    	}
	else if(req.shape == ROTATE)
	{
		rotate(CW, M_PI/2, true);
		rotate(CW, M_PI/2, true);
		rotate(CW, M_PI/2, true);
		rotate(CW, M_PI/2, true);
                ROS_INFO_STREAM("Images taken are : " << img_count_);
                ROS_INFO("Requesting label of intersection");
                std::string ret;
                resp.ret_code = createAndWaitStatus(ret);
                resp.label = ret;
                bool rc = (resp.ret_code > 0) ? true : false;
                img_count_ = 0;
                return rc;
	}

	service_req_ = false;
	resp.ret_code = 1;
	return true;
}

int intersection_explorer::Explorer::createAndWaitStatus(std::string &r)
{
	ROS_INFO("intersection_explorer::Explorer::createAndWaitStatue: Entered!");

	// Create request file
	struct stat buf1;
	if(stat(req_file_.c_str(), &buf1) == 0)
	{
		ROS_ERROR("Improper request handling! Cleanup /tmp files!");
		return ERR_MSG_FILE_HANDLING;
	}

	std::ofstream rfile;
	rfile.open(req_file_.c_str());
	rfile << inter_type_;
	rfile.close();

	// Wait for reply for label
	struct stat buf;
	clock_t start = clock();
	std::string slash = "\\|/-";
        int count = 0;
        unsigned int microseconds = 1000000;
        while(stat(out_file_.c_str(), &buf) != 0)
        {
                clock_t cur = clock();
                double elapsed = double(cur - start) / CLOCKS_PER_SEC;
                std::string print_log = "Processing..." + slash[count%4];
                if(elapsed > 6000)
                {
                        ROS_INFO("Timeout!");
                        return ERR_MSG_TIMEOUT;
                }
                std::cout << print_log << "\r";
                usleep(microseconds);
                count++;
        }
        //ROS_INFO_STREAM("\nProcessed!\n");
        std::ifstream f(out_file_.c_str());
        std::string line, room;
	int it;
        if(f.is_open())
        {
                getline(f, line);
		it = std::stoi(line);
		getline(f, line);
		room = line;
                ROS_INFO("TYPE: %d, ROOM: %s",it, room.c_str());
                r = room;
                f.close();
                //remove(out_file_.c_str());
                //remove(in_file_.c_str());
                system("/home/ucmrobotics/inter_det_ws/src/intersection_explorer/scripts/delete_files");
                return it;
        }
        //ROS_ERROR("Unable to open response file!");
        return ERR_MSG_UNKNOWN;
}

void intersection_explorer::Explorer::rotate(int direction, float angle, bool take_pic)
{
	ROS_INFO("intersection_explorer::Explorer::rotate: Direction: %d, Angle: %f",
			direction, angle);
	geometry_msgs::Twist vel_msg;
	float goal_ang;
	if(direction == CW)
		goal_ang = getYawAngle(cur_pose_) - angle;
	else
		goal_ang = getYawAngle(cur_pose_) + angle;
	goal_ang = intersection_explorer::WrapPosNegPI(goal_ang);
	ROS_INFO("Current angle: [%f], Goal angle: [%f]", getYawAngle(cur_pose_), goal_ang);
	double t0 = ros::Time::now().toSec();
	float cur_ang = getYawAngle(cur_pose_);
	float interval = angle / 20;
	float pic_time = getYawAngle(cur_pose_);
	int count = 0;
	bool flag = true;
	while(flag)
	{
		float yaw = getYawAngle(cur_pose_);
		//float test_yaw = intersection_explorer::WrapPosNegPI(yaw);
		//ROS_INFO("Angle is: %f", yaw);
		if( direction == CW )
		{
			vel_msg.angular.z = -std::abs(angular_speed_);
			if(std::abs(yaw - goal_ang) > 0.01)
				vel_pub_.publish(vel_msg);
			else
				flag = false;
		}
		else
		{
			vel_msg.angular.z = std::abs(angular_speed_);
			if(std::abs(yaw - goal_ang) > 0.01)
				vel_pub_.publish(vel_msg);
			else
				flag = false;
		}
		double t1 = ros::Time::now().toSec();
		//cur_ang = angular_speed_ * ( t1 - t0 );
		ros::spinOnce();
		
		if(std::abs(yaw - pic_time) < 0.01 && take_pic)
		{
			ROS_DEBUG(" Taking picture : %d", count);
			img_count_++;
			vel_msg.angular.z = 0;
			vel_pub_.publish(vel_msg);
			std::string command = "/home/ucmrobotics/inter_det_ws/src/intersection_explorer/scripts/capture_image " + inter_type_ + " " + std::to_string(img_count_);
                        system(command.c_str());
			pic_time = intersection_explorer::WrapPosNegPI(pic_time -
									interval);
		}
		//ROS_INFO("Current: [%f], Goal: [%f]", cur_ang, goal_ang);
	}
	vel_msg.angular.z = 0;
	vel_pub_.publish(vel_msg);
}
float intersection_explorer::Explorer::moveStraight(float dist)
{
	ROS_INFO("intersection_explorer::Explorer::moveStraight: %f", dist);
	robot_moving_ = true;
	geometry_msgs::Twist vel_msg;
	clearVelocityMessage(vel_msg);
	geometry_msgs::Pose goal_pose = cur_pose_;
	goal_pose.position.x += dist;
	ROS_INFO("Current pose: (%f,%f), Goal pose: (%f, %f)", cur_pose_.position.x,
			cur_pose_.position.y, goal_pose.position.x,
			goal_pose.position.y);
	geometry_msgs::Pose start_pose = cur_pose_;
	float move_dist = computeDistance(start_pose, cur_pose_);
	float interval = dist/4;
	float pic_time = 0;
	ROS_DEBUG_STREAM("Goal distance: " << move_dist);

	while( move_dist < (dist - 0.2) )
	{
		vel_msg.linear.x = std::abs(linear_speed_);
		vel_pub_.publish(vel_msg);
		ros::spinOnce();
		move_dist = computeDistance(start_pose, cur_pose_);
		ROS_DEBUG_STREAM("Goal distance: " << move_dist);
		if(move_dist > pic_time)
		{
			// Stop robot and take image
			vel_msg.linear.x = 0;
			vel_pub_.publish(vel_msg);
			img_count_++;
			std::string command = "/home/ucmrobotics/inter_det_ws/src/intersection_explorer/scripts/capture_image " + inter_type_ + " " + std::to_string(img_count_);
			system(command.c_str());
			pic_time += interval;
		}
		if((max_move_dist_ - 0.75) < 0)
			break;
	}

	robot_moving_ = false;
	vel_msg.linear.x = 0;
	vel_pub_.publish(vel_msg);
	ROS_INFO("intersection_explorer::Explorer::moveStraight*******: Finished, Images taken: %d", img_count_);
	return move_dist;
}

bool intersection_explorer::Explorer::moveToInitPose(geometry_msgs::Pose goal)
{
	ROS_INFO("intersection_explorer::Explorer::moveToInitPose: Current pose: (%f, %f),(%f)", cur_pose_.position.x, cur_pose_.position.y, getYawAngle(cur_pose_));
	ROS_INFO("intersection_explorer::Explorer::moveToInitPose: Goal: (%f,%f),(%f)",goal.position.x ,goal.position.y, getYawAngle(goal));
	geometry_msgs::PoseStamped gpose, cpose, gpose_bl, cpose_bl;
	gpose = convertToStamped(goal);
	cpose = convertToStamped(cur_pose_);
	try
	{
		tf_.waitForTransform(base_link_, odom_, gpose.header.stamp,
			ros::Duration(3.0));
		tf_.transformPose(base_link_, gpose, gpose_bl);
		tf_.transformPose(base_link_, cpose, cpose_bl);
		ROS_INFO("Goal in base link: (%f,%f),(%f)", gpose_bl.pose.position.x,
				gpose_bl.pose.position.y, 
				getYawAngle(gpose_bl.pose));
		ROS_INFO("Curr in base link: (%f,%f),(%f)", cpose_bl.pose.position.x,
				cpose_bl.pose.position.y,
				getYawAngle(cpose_bl.pose));
		//rotate(CW, M_PI/2);
		//rotate(CW, M_PI/2);
		float to_rot = getAngleToGoal(cpose_bl.pose, gpose_bl.pose);
		float mov_dist = computeDistance(gpose_bl.pose, cpose_bl.pose);
		ROS_INFO("To rotate: %f, to move straight: %f", to_rot, mov_dist);
		//rotate(CW, M_PI/2);
		//rotate(CW, to_rot);
		moveStraight(mov_dist);
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR(ex.what());
	}
	/*float to_rot = getAngleToGoal(goal, cur_pose_);
	float mov_dist = computeDistance(cur_pose_, goal);
	ROS_INFO("To rotate: %f, to move straight: %f", to_rot, mov_dist);
	rotate(CCW, to_rot);
	moveStraight(mov_dist);*/
	return true;
}

void intersection_explorer::Explorer::clearVelocityMessage(geometry_msgs::Twist &vel_msg)
{
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
}

intersection_explorer::Explorer::~Explorer()
{
}
