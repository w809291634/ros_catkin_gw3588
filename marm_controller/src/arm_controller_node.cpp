#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
using namespace std;

class JointTrajectoryActionServer
{
public:
JointTrajectoryActionServer(std::string name):
		as_(nh_, name, false), action_name_(name)
{
	// register callback for goal
	as_.registerGoalCallback(boost::bind(&JointTrajectoryActionServer::goalCallback, this));
	as_.start();
}
~JointTrajectoryActionServer(void){}

// when a trajectory command comes, this function will be called.
void goalCallback()
{
	control_msgs::FollowJointTrajectoryGoal::_trajectory_type trajectory;
	trajectory_msgs::JointTrajectory::_points_type::iterator iter;
	trajectory = as_.acceptNewGoal()->trajectory;
	int i;
	for (i=0; i<trajectory.joint_names.size(); i++) {
		cout<<trajectory.joint_names[i]<<":"<<endl;
	}
	i = 0;
	for(iter= trajectory.points.begin(); iter!=trajectory.points.end(); iter++){
		cout<<"points "<<i++<<":"<<endl;
		cout<<"    pos:";
		for (int j=0; j<iter->positions.size(); j++) {
			cout<<iter->positions[0]<<",";
		}
		cout<<"time: "<<iter->time_from_start<<endl;
	}
    // do something
	// when finished, return result
	as_.setSucceeded(result_);
}
protected:
ros::NodeHandle nh_;
actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::Result result_;
std::string action_name_;
};

int main(int argc, char** argv)
{
	ros::init(argc,argv, "arm_controller");
	JointTrajectoryActionServer srv("arm_controller/follow_joint_trajectory");
	ros::spin();
	return 0;
}

