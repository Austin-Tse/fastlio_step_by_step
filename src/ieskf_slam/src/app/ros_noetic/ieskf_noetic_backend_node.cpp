#include "wrapper/ros_noetic/ieskf_backend_noetic_wrapper.h"


int main(int argc,char *argv[])
{
    ros::init(argc,argv,"ieskf_backend_running_node");
    ros::NodeHandle nh;
    std::shared_ptr<ROSNoetic::IESKFBackEndWrapper>backend_ptr = std::make_shared<ROSNoetic::IESKFBackEndWrapper>(nh);
    return 0;
}