/**
 * @file branch_finder.h
 * @author Robert valner
 */

#ifndef META_PARSER_BRANCH_FINDER_H_
#define META_PARSER_BRANCH_FINDER_H_

#include <memory>
#include <vector>
#include <unordered_map>

#include "meta/parser/trees/visitors/visitor.h"
#include "meta/parser/sr_parser.h"
#include "TTP/task_descriptor.h"
#include "TTP/language_processors/nlp_tools/number_operations.h"

namespace meta
{
namespace parser
{

/**
 * This is a visitor that finds and extracts a branch in a parse tree.
 */
class branch_finder : public const_visitor<void>
{
public:

  /**
   * @brief branch_finder
   * @param num_str_map
   */
  branch_finder(std::shared_ptr<TTP::nummap> num_str_map);

  void operator()(const leaf_node&) override;
  void operator()(const internal_node&) override;

  /**
   * @brief Returns the phrases found by visitor
   * @return
   */
  std::vector<TTP::TaskDescriptor> getTaskDescs();

  std::string getAddressable();

private:

  // Pointer to the string-to-number lookup table
  std::shared_ptr<TTP::nummap> num_str_map_;

  // The storage for the task descriptors found so far
  std::vector<TTP::TaskDescriptor> task_descs_;

  // String that contains info about who/what was addressed
  std::string addressable_ = "";

  // Creates a numeric type subject
  TTP::Subject parseToNumericSubject(parser::parse_tree& tree);

  // TODO: THIS IS A HACK FOR MAKING STOPPING TASKS POSSIBLE
  unsigned int verb_count_ = 0;
  bool next_verb_is_stopped_ = false;
};


}
}

#endif
