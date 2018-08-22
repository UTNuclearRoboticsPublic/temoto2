#include "algorithm_manager/algorithm_info_registry.h"
#include <algorithm>

namespace algorithm_manager
{

AlgorithmInfoRegistry::AlgorithmInfoRegistry(){}

bool AlgorithmInfoRegistry::addLocalAlgorithm(const AlgorithmInfo& si)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  // Check if there is no such algorithm
  AlgorithmInfo si_ret;
  if (!findAlgorithm(si, local_algorithms_, si_ret))
  {
    local_algorithms_.push_back(si);
    return true;
  }

  // Return false if such algorithm already exists
  return false;
}

bool AlgorithmInfoRegistry::addRemoteAlgorithm(const AlgorithmInfo &si)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  // Check if there is no such algorithm
  AlgorithmInfo si_ret;
  if (!findAlgorithm(si, remote_algorithms_, si_ret))
  {
    remote_algorithms_.push_back(si);
    return true;
  }

  // Return false if such algorithm already exists
  return false;
}

bool AlgorithmInfoRegistry::updateLocalAlgorithm(const AlgorithmInfo &si, bool advertised)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  const auto it = std::find_if( local_algorithms_.begin()
                              , local_algorithms_.end()
                              , [&](const AlgorithmInfo& ls)
                              {
                                return ls == si;
                              });

  // Update the local algorithm if its found
  if (it != local_algorithms_.end())
  {
    *it = si;
    it->setAdvertised( advertised );
    return true;
  }

  // Return false if no such algorithm was found
  return false;
}

bool AlgorithmInfoRegistry::updateRemoteAlgorithm(const AlgorithmInfo &si, bool advertised)
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  const auto it = std::find_if( remote_algorithms_.begin()
                              , remote_algorithms_.end()
                              , [&](const AlgorithmInfo& rs)
                              {
                                return rs == si;
                              });

  // Update the local algorithm if its found
  if (it != remote_algorithms_.end())
  {
    *it = si;
    it->setAdvertised( advertised );
    return true;
  }

  // Return false if no such algorithm was found
  return false;
}

bool AlgorithmInfoRegistry::findLocalAlgorithm( temoto_2::LoadAlgorithm::Request& req
                                        , AlgorithmInfo& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findAlgorithm(req, local_algorithms_, si_ret);
}

bool AlgorithmInfoRegistry::findLocalAlgorithm( const AlgorithmInfo &si, AlgorithmInfo& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findAlgorithm(si, local_algorithms_, si_ret);
}

bool AlgorithmInfoRegistry::findLocalAlgorithm( const AlgorithmInfo &si ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  AlgorithmInfo si_ret;
  return findAlgorithm(si, local_algorithms_, si_ret);
}

bool AlgorithmInfoRegistry::findRemoteAlgorithm( temoto_2::LoadAlgorithm::Request& req
                                         , AlgorithmInfo& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findAlgorithm(req, remote_algorithms_, si_ret);
}

bool AlgorithmInfoRegistry::findRemoteAlgorithm( const AlgorithmInfo &si, AlgorithmInfo& si_ret ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  return findAlgorithm(si, remote_algorithms_, si_ret);
}

bool AlgorithmInfoRegistry::findRemoteAlgorithm( const AlgorithmInfo &si ) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> guard(read_write_mutex);

  AlgorithmInfo si_ret;
  return findAlgorithm(si, remote_algorithms_, si_ret);
}

bool AlgorithmInfoRegistry::findAlgorithm( temoto_2::LoadAlgorithm::Request& req
                                   , const std::vector<AlgorithmInfo>& algorithms
                                   , AlgorithmInfo& si_ret ) const
{
  // Local list of devices that follow the requirements
  std::vector<AlgorithmInfo> candidates;

  // Find the devices that follow the "type" criteria
  auto it = std::copy_if(algorithms.begin()
                       , algorithms.end()
                       , std::back_inserter(candidates)
                       , [&](const AlgorithmInfo& s)
                         {
                           return s.getType() == req.algorithm_type;
                         });

  // The requested type of algorithm is not available
  if (candidates.empty())
  {
    return false;
  }

  // If package_name is specified, remove all non-matching candidates
  auto it_end = candidates.end();
  if (req.package_name != "")
  {
    it_end = std::remove_if(candidates.begin(), candidates.end(),
                            [&](AlgorithmInfo s)
                            {
                              return s.getPackageName() != req.package_name;
                            });
  }

  // If executable is specified, remove all non-matching candidates
  if (req.executable != "")
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](AlgorithmInfo s)
                            {
                              return s.getExecutable() != req.executable;
                            });
  }

  // If output topics are specified ...
  if (!req.output_topics.empty())
  {
    it_end = std::remove_if(candidates.begin(), it_end,
                            [&](AlgorithmInfo s)
                            {
                              if (s.getOutputTopics().size() < req.output_topics.size())
                                return true;

                              // Make a copy of the input topics
                              std::vector<StringPair> output_topics_copy = s.getOutputTopics();

                              // Start looking for the requested topic types
                              for (auto& topic : req.output_topics)
                              {
                                bool found = false;
                                for (auto it=output_topics_copy.begin(); it != output_topics_copy.end(); it++)
                                {
                                  // If the topic was found then remove it from the copy list
                                  if (topic.key == it->first)
                                  {
                                    found = true;
                                    output_topics_copy.erase(it);
                                    break;
                                  }
                                }

                                // If this topic type was not found then return with false
                                if (!found)
                                {
                                  return true;
                                }
                              }

                              return false;
                            });
  }

  // Sort remaining candidates based on their reliability.
  std::sort( candidates.begin()
           , it_end
           , [](AlgorithmInfo& s1, AlgorithmInfo& s2)
             {
               return s1.getReliability() > s2.getReliability();
             });

  if (candidates.begin() == it_end)
  {
    // Algorithm with the requested criteria was not found.
    return false;
  }

  // Return the first algorithm of the requested type.
  si_ret = candidates.front();
  return true;
}

bool AlgorithmInfoRegistry::findAlgorithm( const AlgorithmInfo &si
                                   , const std::vector<AlgorithmInfo>& algorithms
                                   , AlgorithmInfo& si_ret ) const
{

  const auto it = std::find_if( algorithms.begin()
                              , algorithms.end()
                              , [&](const AlgorithmInfo& rs)
                              {
                                return rs == si;
                              });


  if (it == algorithms.end())
  {
    return false;
  }
  else
  {
    si_ret = *it;
    return true;
  }
}

const std::vector<AlgorithmInfo>& AlgorithmInfoRegistry::getLocalAlgorithms() const
{
  return local_algorithms_;
}

const std::vector<AlgorithmInfo>& AlgorithmInfoRegistry::getRemoteAlgorithms() const
{
  return remote_algorithms_;
}

} // algorithm_manager namespace
