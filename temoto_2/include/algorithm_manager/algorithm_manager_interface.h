#ifndef ALGORITHM_MANAGER_INTERFACE_H
#define ALGORITHM_MANAGER_INTERFACE_H

#include "common/base_subsystem.h"
#include "common/topic_container.h"

#include "TTP/base_task/base_task.h"
#include "algorithm_manager/algorithm_manager_services.h"
#include "rmp/resource_manager.h"
#include <memory>   // unique_ptr

/**
 * @brief The AlgorithmTopicsReq class
 */
class AlgorithmTopicsReq : public TopicContainer
{
  /* DELIBERATELY EMPTY */
};

/**
 * @brief The AlgorithmTopicsRes class
 */
class AlgorithmTopicsRes : public TopicContainer
{
  /* DELIBERATELY EMPTY */
};


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *                        ALGORITHM MANAGER INTERFACE
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

template <class OwnerTask>
class AlgorithmManagerInterface : BaseSubsystem
{
public:
  /**
   * @brief AlgorithmManagerInterface
   */
  AlgorithmManagerInterface()
  {
    class_name_ = __func__;
  }

  void initialize(TTP::BaseTask* task)
  {
    initializeBase(task);
    log_group_ = "interfaces." + task->getPackageName();
    name_ = task->getName() + "/algorithm_manager_interface";

    resource_manager_ = std::unique_ptr<rmp::ResourceManager<AlgorithmManagerInterface>>(new rmp::ResourceManager<AlgorithmManagerInterface>(name_, this));
    resource_manager_->registerStatusCb(&AlgorithmManagerInterface::statusInfoCb);
  }

  /**
   * @brief startAlgorithm
   * @param algorithm_type
   * @return
   */
  AlgorithmTopicsRes startAlgorithm(const std::string& algorithm_type)
  {
    try
    {
      validateInterface();
    }
    catch (error::ErrorStack& e)
    {
      FORWARD_ERROR(error_stack);
    }

    return startAlgorithm(algorithm_type, "", "", AlgorithmTopicsReq());
  }

  /**
   * @brief startAlgorithm
   * @param algorithm_type
   * @param topics
   * @return
   */
  AlgorithmTopicsRes startAlgorithm(const std::string& algorithm_type
                                  , const AlgorithmTopicsReq& topics)
  {
    try
    {
      validateInterface();
    }
    catch (error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }

    return startAlgorithm(algorithm_type, "", "", topics);
  }

  /**
   * @brief startAlgorithm
   * @param algorithm_type
   * @param package_name
   * @param ros_program_name
   * @param topics
   * @return
   */
  AlgorithmTopicsRes startAlgorithm(const std::string& algorithm_type
                                  , const std::string& package_name
                                  , const std::string& ros_program_name
                                  , const AlgorithmTopicsReq& topics)
  {
    try
    {
      validateInterface();
    }
    catch (error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }

    return startAlgorithm(algorithm_type, "", "", topics);

    // Fill out the "StartAlgorithmRequest" request
    temoto_2::LoadAlgorithm srv_msg;
    srv_msg.request.algorithm_type = algorithm_type;
    srv_msg.request.package_name = package_name;
    srv_msg.request.executable = ros_program_name;
    srv_msg.request.input_topics = topics.inputTopicsAsKeyValues();
    srv_msg.request.output_topics = topics.outputTopicsAsKeyValues();

    try
    {
      // Call the server
      if (!resource_manager_->template call<temoto_2::LoadAlgorithm>(algorithm_manager::srv_name::MANAGER
                                                                   , algorithm_manager::srv_name::SERVER
                                                                   , srv_msg))

      allocated_algorithms_.push_back(srv_msg);
      AlgorithmTopicsRes responded_topics;
      responded_topics.setInputTopicsByKeyValue( srv_msg.response.input_topics );
      responded_topics.setOutputTopicsByKeyValue( srv_msg.response.output_topics );

      return responded_topics;
    }
    catch (error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
  }

  /**
   * @brief stopAlgorithm
   * @param algorithm_type
   * @param package_name
   * @param ros_program_name
   */
//  void stopAlgorithm(std::string algorithm_type
//                   , std::string package_name
//                   , std::string ros_program_name)
//  {
//    // Name of the method, used for making debugging a bit simpler
//    std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);
//    validateInterface(prefix);

//    // Find all instances where request part matches with the function's input argumens
//    // and unload each resource
//    temoto_2::LoadAlgorithm::Request req;
//    req.algorithm_type = algorithm_type;
//    req.package_name = package_name;
//    req.executable = ros_program_name;

//    auto cur_algorithm_it = allocated_algorithms_.begin();
//    while (cur_algorithm_it != allocated_algorithms_.end())
//    {
//      // The == operator used in the lambda function is defined in
//      // algorithm manager services header
//      auto found_algorithm_it = std::find_if(cur_algorithm_it
//                                           , allocated_algorithms_.end()
//                                           , [&](const temoto_2::LoadAlgorithm& srv_msg) -> bool
//                                             {
//                                               return srv_msg.request == req;
//                                             });

//      if (found_algorithm_it != allocated_algorithms_.end())
//      {
//        // Unload the algorithm
//        resource_manager_->unloadClientResource(found_algorithm_it->response.rmp.resource_id);
//        cur_algorithm_it = found_algorithm_it;
//      }
//      else if (cur_algorithm_it == allocated_algorithms_.begin())
//      {
//        throw error::ErrorStackUtil(
//            taskErr::RESOURCE_UNLOAD_FAIL, error::Subsystem::TASK, error::Urgency::MEDIUM,
//            prefix + " Unable to unload resource that is not loaded.", ros::Time::now());
//      }
//    }
//  }

  /**
   * @brief statusInfoCb
   * @param srv
   */
  void statusInfoCb(temoto_2::ResourceStatus& srv)
  {
    try
    {
      validateInterface();
    }
    catch (error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }

    TEMOTO_DEBUG("Received status information.");
    TEMOTO_DEBUG_STREAM(srv.request);

    // If the resource has falied then reload it since there is a chance that
    // algorithm manager will provide better algorithm this time
    if (srv.request.status_code == rmp::status_codes::FAILED)
    {
      TEMOTO_WARN("detected an algorithm failure. Unloading and trying again.");

      // Find the algorithm from the list of allocated algorithms
      auto algorithm_itr = std::find_if(allocated_algorithms_.begin()
                                , allocated_algorithms_.end()
                                , [&](const temoto_2::LoadAlgorithm& algorithm) -> bool
                                  {
                                    return algorithm.response.rmp.resource_id == srv.request.resource_id;
                                  });

      // If the algorithm was found then ...
      if (algorithm_itr != allocated_algorithms_.end())
      {
        // ... unload it and ...
        TEMOTO_DEBUG_STREAM(prefix << "Unloading");
        resource_manager_->unloadClientResource(algorithm_itr->response.rmp.resource_id);

        // ... copy the output/input topics from the response into the output/input topics
        // of the request (since the user still wants to receive the data on the same topics) ...
        algorithm_itr->request.output_topics = algorithm_itr->response.output_topics;
        algorithm_itr->request.input_topics = algorithm_itr->response.input_topics;

        // ... and load an alternative algorithm. This call automatically
        // updates the response in allocated algorithms vector
        TEMOTO_DEBUG_STREAM(prefix << "Trying to load an alternative algorithm");

        try
        {
          resource_manager_->template call<temoto_2::LoadAlgorithm>(algorithm_manager::srv_name::MANAGER
                                                                  , algorithm_manager::srv_name::SERVER
                                                                  , *algorithm_itr);
        }
        catch (error::ErrorStack& error_stack)
        {
          throw CREATE_ERROR(error_stack);
        }
      }
      else
      {
        // If the algorithm was not found then this is certaintly an interesting situation
        // TODO: Create and throw an error message
      }

      // Forward and append the error stack
      SEND_ERROR(srv.request.error_stack);
    }
  }

  const std::string& getName() const
  {
    return name_;
  }

private:

  std::string name_;
  std::vector<temoto_2::LoadAlgorithm> allocated_algorithms_;
  std::unique_ptr<rmp::ResourceManager<AlgorithmManagerInterface>> resource_manager_;


  /**
   * @brief validateInterface
   */
  void validateInterface()
  {
    if(!resource_manager_)
    {
      throw CREATE_ERROR(error::Code::NOT_INITIALIZED, "Interface is not initalized.");
    }
  }
};

#endif
