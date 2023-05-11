#include <five_dof/robot_arm_robot_hardware_interface.h>

robot_arm::robot_arm(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 20;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_arduino",10);
	// client = nh_.serviceClient<arm_package::Floats_array>("/read_joint_state");
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &robot_arm::update, this);
}

robot_arm::~robot_arm() {
}

void robot_arm::init() {
    
    num_joints_ = 5;

	joint_names_[0]="base_joint_1";	
	joint_names_[1]="shoulder_joint_2";
	joint_names_[2]="elbow_joint_3";
    joint_names_[3]="wrist_joint_4";	
	joint_names_[4]="roll_wrist_joint_5";

    for (int i = 0; i < num_joints_; ++i) {

         // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        
        position_joint_interface_.registerHandle(jointPositionHandle);
   
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    joint_position_command_[0]=0;
    joint_position_command_[1]=0;
    joint_position_command_[2]=0;
    joint_position_command_[3]=0;
    joint_position_command_[4]=0;
}

void robot_arm::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void robot_arm::read() {

	joint_read.request.req=1.0;
	
	if(client.call(joint_read))
	{
	    
		joint_position_[0]=angles::from_degrees(90-joint_read.response.res[0]);
		joint_position_[1]=angles::from_degrees(joint_read.response.res[1]-90);
		joint_position_[2]=angles::from_degrees(joint_read.response.res[2]-90);
        joint_position_[3]=angles::from_degrees(joint_read.response.res[3]-90);
		joint_position_[4]=angles::from_degrees(joint_read.response.res[4]-90);
		
		// ROS_INFO("Receiving  j1: %.2f, j2: %.2f, j3: %.2f",joint_read.response.res[0],joint_read.response.res[1], joint_read.response.res[2]);
		
	}	
    else
    {
    	joint_position_[0]=0;
        joint_position_[1]=0;
        joint_position_[2]=0;
        joint_position_[3]=0;
        joint_position_[4]=0;
        // ROS_INFO("Service not found ");
    }
        

}

void robot_arm::write(ros::Duration elapsed_time) {
    
	joints_pub.data.clear();
	joints_pub.data.push_back(90-(angles::to_degrees(joint_position_command_[0])));
	joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[1])));
	joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[2])));
    joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[3])));
	joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[4])));
	// ROS_INFO("Publishing j1: %.2f, j2: %.2f, j3: %.2f",joints_pub.data[0],joints_pub.data[1],joints_pub.data[2]);
	pub.publish(joints_pub);	
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_hardware_interface_node");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(2); 
    ros::MultiThreadedSpinner spinner(2);// 2 threads for controller service and for the Service client used to get the feedback from ardiuno
    //spinner.start();
    robot_arm ROBOT(nh);
    spinner.spin();
    //ros::spin();
    return 0;
}