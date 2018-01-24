/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Task that loads the robot using robot manager interface.
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
#include "robot_manager/robot_manager_interface.h"

class TaskRobot : public TTP::BaseTask
{
public:
  TaskRobot()
  {
    TASK_INFO("Task Robot constructed");
  }

  void startTask(TTP::TaskInterface task_interface)
  {

    task_alias_ = task_interface.alias_;
    input_subjects = task_interface.input_subjects_;

    // initialize robot manager interface
    rmi_.initialize(this);

    switch (task_interface.id_)
    {
      case 0:
        loadRobot();
        break;

      case 1:
        if (task_interface.alias_ == "make")
        {
            plan();
        }
        else
        if (task_interface.alias_ == "execut")
        {
            execute();
        }
        break;

      case 2:
        execute();
        break;

      case 3:
        setTargetToHand();
        break;

      case 4:
        setTargetToHand();
        break;
    }
  }

  void loadRobot()
  {
    // Load the ur5 robot
    //rmi_.loadRobot("ur5");
    //rmi_.loadRobot("vaultbot");
    rmi_.loadRobot();

    // DO NOT EXIT THIS FUNCTION
    //ros::waitForShutdown();
  }

  void plan()
  {
    TEMOTO_DEBUG("PLANNING");
    // geometry_msgs::PoseStamped target_pose;
    //rmi_.plan(target_pose);
    rmi_.plan(); //plan with default planning group using default target
    TEMOTO_DEBUG("DONE PLANNING");
  }

  void execute()
  {
    TEMOTO_DEBUG("EXECUTING");
    geometry_msgs::Pose pose;
    rmi_.execute();
    TEMOTO_DEBUG("DONE EXECUTING");
  }

  void setTargetToHand()
  {
    TEMOTO_DEBUG("SET TARGET TO HAND");
    TTP::Subject where_0_in = TTP::getSubjectByType("where", input_subjects);
    std::string where_object = where_0_in.words_[0];
    where_object = (where_object=="") ? where_object : "right hand";
    rmi_.setTarget(where_object);
    TEMOTO_DEBUG("DONE SET TARGET TO HAND");
  }

  std::vector<TTP::Subject> getSolution()
  {
    return output_subjects;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * *
   * Inherited methods that have to be implemented / END
   * * * * * * * * * * * * * * * * * * * * * * * * */

  ~TaskRobot()
  {
    // in order to not upset moveit, we have to unload output manager interface first!!!
    TASK_INFO("[TaskRobot::~TaskRobot] TaskRobot destructed");
  }

private:
  // Create interfaces for accessing temoto managers
  robot_manager::RobotManagerInterface<TaskRobot> rmi_;

  std::string task_alias_;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskRobot, TTP::BaseTask);
