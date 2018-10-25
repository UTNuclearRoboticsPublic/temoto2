#ifndef ALGORITHM_INFO_DATABASE_H
#define ALGORITHM_INFO_DATABASE_H

#include "algorithm_manager/algorithm_info.h"
#include "temoto_2/LoadAlgorithm.h"

#include <mutex>

namespace algorithm_manager
{

/**
 * @brief Class that maintains and handles the algorithm info objects
 */
class AlgorithmInfoRegistry
{
public:

  struct AlgorithmInfoPtrs
  {
    std::vector<AlgorithmInfoPtr>& algorithms;
  };

  AlgorithmInfoRegistry();

  bool findLocalAlgorithms( temoto_2::LoadAlgorithm::Request& req, std::vector<AlgorithmInfo>& ai_ret ) const;

  bool findLocalAlgorithm( const AlgorithmInfo& ai, AlgorithmInfo& ai_ret ) const;

  bool findLocalAlgorithm( const AlgorithmInfo& ai ) const;

  bool findRemoteAlgorithms( temoto_2::LoadAlgorithm::Request& req, std::vector<AlgorithmInfo>& ai_ret ) const;

  bool findRemoteAlgorithm( const AlgorithmInfo& ai, AlgorithmInfo& ai_ret ) const;

  bool findRemoteAlgorithm( const AlgorithmInfo& ai ) const;

  bool addLocalAlgorithm( const AlgorithmInfo& ai );

  bool addRemoteAlgorithm( const AlgorithmInfo& ai );

  bool updateLocalAlgorithm(const AlgorithmInfo& ai, bool advertised = false);

  bool updateRemoteAlgorithm(const AlgorithmInfo& ai, bool advertised = false);

  const std::vector<AlgorithmInfo>& getLocalAlgorithms() const;

  const std::vector<AlgorithmInfo>& getRemoteAlgorithms() const;

private:

  /**
   * @brief findAlgorithm
   * @param req
   * @param algorithms
   * @return
   */
  bool findAlgorithms( temoto_2::LoadAlgorithm::Request& req
                     , const std::vector<AlgorithmInfo>& algorithms
                     , std::vector<AlgorithmInfo>& ai_ret ) const;

  bool findAlgorithm( const AlgorithmInfo& ai
                    , const std::vector<AlgorithmInfo>& algorithms
                    , AlgorithmInfo& ai_ret ) const;

  /// List of all locally defined algorithms.
  std::vector<AlgorithmInfo> local_algorithms_;

  /// List of all algorithms in remote managers.
  std::vector<AlgorithmInfo> remote_algorithms_;

  /// Mutex for protecting algorithm info vectors from data races
  mutable std::mutex read_write_mutex;
};

} // algorithm_manager namespace

#endif
