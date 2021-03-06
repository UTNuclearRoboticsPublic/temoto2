#include "ros/ros.h"
#include "ros/package.h"
#include "TTP/task_descriptor_processor.h"

// main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "algorithm_info_tests");
  ros::NodeHandle n;
  std::string base_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/test/";

  /*
   * Create a dummy task descriptor
   */

  std::string action = "track";
  TTP::Subjects subjects;

  TTP::Subject sub_0("what", "apple");
  TTP::Subject sub_1("what", "orange");

  sub_1.addData("topic", std::string("no_data"));
  sub_1.addData("number", 12.4);

  subjects.push_back(sub_0);
  subjects.push_back(sub_1);

  TTP::TaskInterface interface_0;
  interface_0.id_ = 0;
  interface_0.type_ = "synchronous";
  interface_0.input_subjects_ = subjects;
  interface_0.output_subjects_ = subjects;

  TTP::TaskInterface interface_1;
  interface_1.id_ = 1;
  interface_1.type_ = "synchronous";
  interface_1.input_subjects_ = subjects;

  std::vector<TTP::TaskInterface> interfaces;
  interfaces.push_back(interface_0);
  interfaces.push_back(interface_1);

  TTP::TaskDescriptor task_descriptor(action, interfaces);
  task_descriptor.setTaskPackageName("Name of the action");

  std::cout << task_descriptor << std::endl;

  try
  {
    TTP::saveAIDescriptor(task_descriptor, base_path + "tst_descriptor.xml");
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
