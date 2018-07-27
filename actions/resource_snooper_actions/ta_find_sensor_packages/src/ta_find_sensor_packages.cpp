
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *
 *  The basis of this file has been automatically generated
 *  by the TeMoto action package generator. Modify this file
 *  as you wish but please note:
 *
 *    WE HIGHLIY RECOMMEND TO REFER TO THE TeMoto ACTION
 *    IMPLEMENTATION TUTORIAL IF YOU ARE UNFAMILIAR WITH
 *    THE PROCESS OF CREATING CUSTOM TeMoto ACTION PACKAGES
 *    
 *  because there are plenty of components that should not be
 *  modified or which do not make sence at the first glance.
 *
 *  See TeMoto documentation & tutorials at: 
 *    https://utnuclearroboticspublic.github.io/temoto2
 *
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

/* REQUIRED BY TEMOTO */
#include "TTP/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes

#include "sensor_manager/sensor_info.h"
#include "sensor_manager/sensor_info_database.h"

#include <boost/filesystem/operations.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>


/* 
 * ACTION IMPLEMENTATION of TaFindSensorPackages 
 */
class TaFindSensorPackages : public TTP::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaFindSensorPackages()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaFindSensorPackages constructed");
}
    
/* REQUIRED BY TEMOTO */
void startTask(TTP::TaskInterface task_interface)
{
  input_subjects = task_interface.input_subjects_;
  switch (task_interface.id_)
  {
        
    // Interface 0
    case 0:
      startInterface_0();
      break;

  }
}

/* REQUIRED BY TEMOTO */
std::vector<TTP::Subject> getSolution()
{
  return output_subjects;
}

~TaFindSensorPackages()
{
  TEMOTO_INFO("TaFindSensorPackages destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:
    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];
  std::string  what_0_data_0_in = boost::any_cast<std::string>(what_0_in.data_[0].value);
  sensor_manager::SensorInfoDatabase*     what_0_data_1_in = boost::any_cast<sensor_manager::SensorInfoDatabase*>(what_0_in.data_[1].value);

  // Get the catkin workspace src directory path
  std::string catkin_ws_src_dir = what_0_data_0_in;

  // Look for new packages every 10 seconds
  while(stop_task_ == false)
  {
    TEMOTO_INFO_STREAM("Snooping the catkin workspace at: " << catkin_ws_src_dir);

    // Find all sensor descriptor file paths
    std::vector<std::string> sensor_desc_file_paths = findSensorDescFiles(catkin_ws_src_dir);

    // Read in the sensor descriptors
    std::vector<sensor_manager::SensorInfo> sensor_infos;
    for (const std::string& desc_file_path : sensor_desc_file_paths)
    {
      sensor_manager::SensorInfo sensor_info;

      try
      {
        std::vector<sensor_manager::SensorInfo> sensor_infos_current = getSensorInfo(desc_file_path);
        // sensor_infos.push_back(sensor_info);
      }
      catch(...)
      {
        // TODO: implement a proper catch block
      }
    }

    // Sleep for 10 seconds
    ros::Duration(10).sleep();
  }
}

/**
 * @brief findSensorDescFiles
 * @param path
 * @return
 */
std::vector<std::string> findSensorDescFiles(const std::string& path) const
{
//  boost::filesystem::path current_dir (base_path);
//  boost::filesystem::directory_iterator end_itr;
//  std::vector <TaskDescriptor> tasks_found;

//  try
//  {
//    // Start looking the files inside current directory
//    for ( boost::filesystem::directory_iterator itr( current_dir ); itr != end_itr; ++itr )
//    {

//      // if its a directory and depth limit is not there yet, go inside it
//      if ( boost::filesystem::is_directory(*itr) && (search_depth > 0) )
//      {
//        std::vector<TaskDescriptor> sub_tasks_found = findTaskFilesys( task_to_find, *itr, (search_depth - 1) );

//        // Append the subtasks if not empty
//        if ( !sub_tasks_found.empty() )
//        {
//          tasks_found.insert(std::end(tasks_found), std::begin(sub_tasks_found), std::end(sub_tasks_found));
//        }
//      }

//      // if its a file and matches the desc file name, process the file
//      else if ( boost::filesystem::is_regular_file(*itr) &&
//                ((*itr).path().filename() == description_file_) )
//      {
//        try
//        {
//          /*
//           * Create a description processor object and Get TaskDescriptor
//           * I THINK THIS SHOULD NOT BE CREATED EVERY SINGLE TIME
//           */
//          boost::filesystem::path hackdir ((*itr)); //HACKATON
//          TaskDescriptorProcessor tdp(hackdir.parent_path().string(), *this);
//          tasks_found.push_back(tdp.getTaskDescriptor());
//        }

//        catch(error::ErrorStack& error_stack)
//        {
//          FORWARD_ERROR(error_stack);
//        }
//      }
//    }
//    return std::move(tasks_found);
//  }
//  catch (std::exception& e)
//  {
//    // Rethrow the exception
//    throw CREATE_ERROR(error::Code::FIND_TASK_FAIL, e.what());
//  }

//  catch(...)
//  {
//    // Rethrow the exception
//    throw CREATE_ERROR(error::Code::UNHANDLED_EXCEPTION, "Received an unhandled exception");
//  }

  return std::vector<std::string>();
}

/**
 * @brief getSensorInfo
 * @param desc_file_path
 * @return
 */
std::vector<sensor_manager::SensorInfo> getSensorInfo(const std::string& desc_file_path) const
{
  std::ifstream in( desc_file_path );
  YAML::Node config = YAML::Load(in);
  std::vector<sensor_manager::SensorInfo> sensors;

  // Parse any sensor related information
  if (config["Sensors"])
  {
    sensors = parseSensors(config);
    for (auto& s : sensors)
    {
      TEMOTO_DEBUG("Got sensor: '%s'.", s.getName().c_str());
    }
  }
  else
  {
    TEMOTO_WARN("Failed to read '%s'. Verify that the file exists and the sequence of sensors "
                "is listed under 'Sensors' node.", desc_file_path.c_str());
  }

  return std::move(sensors);
}

/**
 * @brief parseSensors
 * @param config
 * @return
 */
std::vector<sensor_manager::SensorInfo> parseSensors(const YAML::Node& config) const
{
  std::vector<sensor_manager::SensorInfo> sensors;

  if (!config.IsMap())
  {
    TEMOTO_WARN("Unable to parse 'Sensors' key from config.");
    return sensors;
  }

  YAML::Node sensors_node = config["Sensors"];
  if (!sensors_node.IsSequence())
  {
    TEMOTO_WARN("The given config does not contain sequence of sensors.");
    return sensors;
  }

  TEMOTO_DEBUG("Parsing %lu sensors.", sensors_node.size());

  // go over each sensor node in the sequence
  for (YAML::const_iterator node_it = sensors_node.begin(); node_it != sensors_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_WARN("Unable to parse the sensor. Parameters in YAML have to be specified in "
                   "key-value pairs.");
      continue;
    }

    try
    {
      sensor_manager::SensorInfo sensor = node_it->as<sensor_manager::SensorInfo>();
      if (std::count_if( sensors.begin()
                       , sensors.end()
                       , [&](const sensor_manager::SensorInfo& s)
                         {
                            return s == sensor;
                         }) == 0)
      {
        // OK, this is unique pointer, add it to the sensors vector.
        sensors.emplace_back(sensor);
        //TEMOTO_DEBUG_STREAM("####### PARSED SENSOR: #######\n" << sensors.back()->toString());
      }
      else
      {
        TEMOTO_WARN("Ignoring duplicate of sensor '%s'.", sensor.getName().c_str());
      }
    }
    catch (YAML::TypedBadConversion<sensor_manager::SensorInfo> e)
    {
      TEMOTO_WARN("Failed to parse sensor_manager::SensorInfo from config.");
      continue;
    }
  }
  return std::move(sensors);
}

}; // TaFindSensorPackages class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaFindSensorPackages, TTP::BaseTask);
