/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Task that visualizes the robot.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "output_manager/output_manager_interface.h"
//#include "robot_manager/robot_manager_interface.h"

class TaskRobotViz : public TTP::BaseTask
{
public:
  TaskRobotViz()// : rmi_(this)
  {
    TASK_INFO("Task Robot constructed");
  }

  bool startTask(TTP::TaskInterface task_interface)
  {
    task_alias_ = task_interface.alias_;
    input_subjects = task_interface.input_subjects_;

    // initialize output manager
    omi_.initialize(this);

    switch (task_interface.id_)
    {
      case 0:
        showRobot();
        break;

      case 1:
        showNavigation();
        break;

      case 2:
        showManipulation();
        break;

      case 3:
        showFootprint();
        break;
    }
    return true;
  }

  void showRobot()
  {
    std::string prefix = common::generateLogPrefix("", this->getPackageName(), __func__);
    try
    {
      // Show the ur5 robot model
      std::set<std::string> viz_options {"robot_model"};
      omi_.showRobot("ur5", viz_options);
    }
    catch (error::ErrorStackUtil& e)
    {
      e.forward(prefix);
      error_handler_.append(e);
    }
  }

  void showNavigation()
  {
    std::string prefix = common::generateLogPrefix("", this->getPackageName(), __func__);
    try
    {
      // Show the ur5 robot model
      std::set<std::string> viz_options {"navigation"};
      omi_.showRobot(viz_options);
    }
    catch (error::ErrorStackUtil& e)
    {
      e.forward(prefix);
      error_handler_.append(e);
    }
  }

  void showManipulation()
  {
    std::string prefix = common::generateLogPrefix("", this->getPackageName(), __func__);
    try
    {
      // Show the ur5 robot model
      std::set<std::string> viz_options {"manipulation"};
      omi_.showRobot(viz_options);
    }
    catch (error::ErrorStackUtil& e)
    {
      e.forward(prefix);
      error_handler_.append(e);
    }
  }

  void showFootprint()
  {
    std::string prefix = common::generateLogPrefix("", this->getPackageName(), __func__);
    try
    {
      // Show the ur5 robot model
      std::set<std::string> viz_options {"footprint"};
      omi_.showRobot(viz_options);
    }
    catch (error::ErrorStackUtil& e)
    {
      e.forward(prefix);
      error_handler_.append(e);
    }
  }


  std::vector<TTP::Subject> getSolution()
  {
    return output_subjects;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * *
   * Inherited methods that have to be implemented / END
   * * * * * * * * * * * * * * * * * * * * * * * * */

  ~TaskRobotViz()
  {
    // in order to not upset moveit, we have to unload output manager interface first!!!
    TASK_INFO("[TaskRobotViz::~TaskRobotViz] TaskRobotViz destructed");
  }

private:
  // Create interfaces for accessing temoto devices
  output_manager::OutputManagerInterface<TaskRobotViz> omi_;
  std::string task_alias_;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskRobotViz, TTP::BaseTask);
