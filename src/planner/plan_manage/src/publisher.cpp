#include <ros/ros.h>
#include <lbfgs/Optdata.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  ros::NodeHandle nh("~");

  ros::ServiceClient Optdata_client = nh.serviceClient<lbfgs::Optdata>("/lbfgs/Optdata");
  lbfgs::Optdata data;
  data.request.final_cost =73;
  while(ros::ok()){
    Optdata_client.call(data);
  }


  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

