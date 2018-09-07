#include <simulator.h>

using namespace std;

Simulator::Simulator(ros::NodeHandle nh_)
{
    joy_sub = nh_.subscribe("/joy", 1, &Simulator::joy_cb, this);
    pub_joint_state = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model = robot_model_loader.getModel();
    // ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
}

Simulator::~Simulator()
{
    joy_sub.shutdown();
}

void Simulator::publish_joint_state()
{
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("arm");
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    std::vector<double> joints;
    geometry_msgs::Pose pose_msg;
    sensor_msgs::JointState joint_state_msg;
    kinematic_state->setToRandomPositions(joint_model_group);
    kinematic_state->copyJointGroupPositions(joint_model_group, joints);
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        joint_state_msg.name.push_back(joint_names[i].c_str());
        joint_state_msg.position.push_back(joints[i]);
    }
    pub_joint_state.publish(joint_state_msg);
}

void Simulator::spin()
{
    ros::Rate r(0.5);
    while (ros::ok())
    {
        publish_joint_state();
        ros::spinOnce();
        r.sleep();
    }
}

main(int argc, char *argv[])
{
    ros::init(argc, argv, "simulator");
    ros::NodeHandle nh_("~");
    Simulator Sim(nh_);
    Sim.spin();
    return EXIT_SUCCESS;
}