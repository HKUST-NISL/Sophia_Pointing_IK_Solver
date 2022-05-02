#include <SophiaModelHandler.h>
#include <pwd.h>

int main(int argc, char** argv)
{
	//Initialize ros node
    ros::init(argc, argv, "yifan_vital_sign_node");
  	ros::NodeHandle nh;
  	ros::AsyncSpinner spinner(4);//spinner thread specification
	spinner.start();

    //Get verbose level
	int VERBOSE_LEVEL;
	nh.param<int>("verbosity", VERBOSE_LEVEL,1);

	//Initialze the solver class
	SophiaModelHandler sophia_model_handler(nh, VERBOSE_LEVEL);

	//Test the pointing capability
	sophia_model_handler.testPointing();


    ros::shutdown();
  	return 0;
}