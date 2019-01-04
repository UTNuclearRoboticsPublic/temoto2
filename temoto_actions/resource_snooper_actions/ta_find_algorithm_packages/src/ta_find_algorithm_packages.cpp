
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

#include "algorithm_manager/algorithm_info.h"
#include "algorithm_manager/algorithm_info_registry.h"

#include <boost/filesystem/operations.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>


/* 
 * ACTION IMPLEMENTATION of TaFindAlgorithmPackages 
 */
class TaFindAlgorithmPackages : public TTP::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaFindAlgorithmPackages()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaFindAlgorithmPackages constructed");
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

~TaFindAlgorithmPackages()
{
  TEMOTO_INFO("TaFindAlgorithmPackages destructed");
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
  algorithm_manager::AlgorithmInfoRegistry*     what_0_data_1_in = boost::any_cast<algorithm_manager::AlgorithmInfoRegistry*>(what_0_in.data_[1].value);

  // Get the catkin workspace src directory path
  std::string catkin_ws_src_dir = what_0_data_0_in;
  algorithm_manager::AlgorithmInfoRegistry* aid = what_0_data_1_in;

  // Look for new packages every 10 seconds
  while(stop_task_ == false)
  {
    TEMOTO_DEBUG_STREAM("Snooping the catkin workspace at: " << catkin_ws_src_dir);

    // Find all algorithm descriptor file paths
    boost::filesystem::path base_path (catkin_ws_src_dir);
    std::vector<std::string> algorithm_desc_file_paths = findAlgorithmDescFiles(base_path);

    // Read in the algorithm descriptors
    std::vector<algorithm_manager::AlgorithmInfo> algorithm_infos;
    for (const std::string& desc_file_path : algorithm_desc_file_paths)
    {
      try
      {
        std::vector<algorithm_manager::AlgorithmInfo> algorithm_infos_current = getAlgorithmInfo(desc_file_path);
        algorithm_infos.insert( algorithm_infos.end()
                              , algorithm_infos_current.begin()
                              , algorithm_infos_current.end());
      }
      catch(...)
      {
        // TODO: implement a proper catch block
      }
    }

    TEMOTO_DEBUG_STREAM("got " << algorithm_infos.size() << " algorithms");

    for (auto ai : algorithm_infos)
    {
      if (aid->addLocalAlgorithm(ai))
      {
        TEMOTO_INFO("Added a new algorithm");
      }
      else
      {
        TEMOTO_DEBUG("This algorithm already exists in the SID");
      }
    }

    // Sleep for 10 seconds
    ros::Duration(10).sleep();
  }
}

/**
 * @brief findAlgorithmDescFiles
 * @param path
 * @return
 */
std::vector<std::string> findAlgorithmDescFiles( const boost::filesystem::path& base_path, int search_depth = 1)
{
  boost::filesystem::directory_iterator end_itr;
  std::vector <std::string> desc_file_paths;

  try
  {
    // Start looking the files inside current directory
    for (boost::filesystem::directory_iterator itr( base_path ); itr != end_itr; ++itr)
    {
      // if its a directory and depth limit is not there yet, go inside it
      if (boost::filesystem::is_directory(*itr) &&
          checkIgnoreDirs(itr->path().filename().string()) &&
          (search_depth > 0))
      {
         std::vector <std::string> sub_desc_file_paths = findAlgorithmDescFiles( *itr, search_depth - 1 );

        // Append the subtasks if not empty
        if (!sub_desc_file_paths.empty())
        {
          desc_file_paths.insert( std::end(desc_file_paths)
                                , std::begin(sub_desc_file_paths)
                                , std::end(sub_desc_file_paths));
        }
      }

      // if its a file and matches the desc file name, process the file
      else if ( boost::filesystem::is_regular_file(*itr) &&
                (itr->path().filename() == description_file_) )
      {
        desc_file_paths.push_back(itr->path().string());
      }


    }
    return desc_file_paths;
  }
  catch (std::exception& e)
  {
    // Rethrow the exception
    throw CREATE_ERROR(temoto_core::error::Code::FIND_TASK_FAIL, e.what());
  }

  catch(...)
  {
    // Rethrow the exception
    throw CREATE_ERROR(temoto_core::error::Code::UNHANDLED_EXCEPTION, "Received an unhandled exception");
  }
}

/**
 * @brief getAlgorithmInfo
 * @param desc_file_path
 * @return
 */
std::vector<algorithm_manager::AlgorithmInfo> getAlgorithmInfo(const std::string& desc_file_path) const
{
  std::ifstream in( desc_file_path );
  YAML::Node config = YAML::Load(in);
  std::vector<algorithm_manager::AlgorithmInfo> algorithms;

  // Parse any algorithm related information
  if (config["Algorithms"])
  {
    algorithms = parseAlgorithms(config);
    for (auto& s : algorithms)
    {
      TEMOTO_DEBUG("Got algorithm: '%s'.", s.getName().c_str());
    }
  }
  else
  {
    TEMOTO_WARN("Failed to read '%s'. Verify that the file exists and the sequence of algorithms "
                "is listed under 'Algorithms' node.", desc_file_path.c_str());
  }

  return std::move(algorithms);
}

/**
 * @brief parseAlgorithms
 * @param config
 * @return
 */
std::vector<algorithm_manager::AlgorithmInfo> parseAlgorithms(const YAML::Node& config) const
{
  std::vector<algorithm_manager::AlgorithmInfo> algorithms;

  if (!config.IsMap())
  {
    TEMOTO_WARN("Unable to parse 'Algorithms' key from config.");
    return algorithms;
  }

  YAML::Node algorithms_node = config["Algorithms"];
  if (!algorithms_node.IsSequence())
  {
    TEMOTO_WARN("The given config does not contain sequence of algorithms.");
    return algorithms;
  }

  TEMOTO_DEBUG("Parsing %lu algorithms.", algorithms_node.size());

  // go over each algorithm node in the sequence
  for (YAML::const_iterator node_it = algorithms_node.begin(); node_it != algorithms_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_WARN("Unable to parse the algorithm. Parameters in YAML have to be specified in "
                   "key-value pairs.");
      continue;
    }

    try
    {
      algorithm_manager::AlgorithmInfo algorithm = node_it->as<algorithm_manager::AlgorithmInfo>();
      if (std::count_if( algorithms.begin()
                       , algorithms.end()
                       , [&](const algorithm_manager::AlgorithmInfo& s)
                         {
                            return s == algorithm;
                         }) == 0)
      {
        // OK, this is unique pointer, add it to the algorithms vector.
        algorithms.emplace_back(algorithm);
        //TEMOTO_DEBUG_STREAM("####### PARSED SENSOR: #######\n" << algorithms.back()->toString());
      }
      else
      {
        TEMOTO_WARN("Ignoring duplicate of algorithm '%s'.", algorithm.getName().c_str());
      }
    }
    catch (YAML::TypedBadConversion<algorithm_manager::AlgorithmInfo> e)
    {
      TEMOTO_WARN("Failed to parse algorithm_manager::AlgorithmInfo from config.");
      continue;
    }
  }
  return std::move(algorithms);
}

/**
 * @brief Checks if the given directory should be ignored or not
 * @param dir
 * @return
 */
bool checkIgnoreDirs( std::string dir)
{
  for (const std::string& ignore_dir : ignore_dirs_)
  {
    if (dir == ignore_dir)
    {
      return false;
    }
  }

  return true;
}

/// Directories that can be ignored
std::vector<std::string> ignore_dirs_{"src", "launch", "config", "build", "description"};

/// Name of the algorithm description file
std::string description_file_= "algorithm_description.yaml";

}; // TaFindAlgorithmPackages class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaFindAlgorithmPackages, TTP::BaseTask);
