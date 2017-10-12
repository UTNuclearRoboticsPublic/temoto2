/**
 * @file branch_finder.h
 * @author Robert valner
 *
 * All files in META are released under the MIT license. For more details,
 * consult the file LICENSE in the root of the project.
 */

#ifndef META_PARSER_BRANCH_FINDER_H_
#define META_PARSER_BRANCH_FINDER_H_

#include <memory>
#include <vector>

#include "meta/parser/trees/visitors/visitor.h"
#include "meta/parser/sr_parser.h"
#include "TTP/task_descriptor.h"

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

    void operator()(const leaf_node&) override;
    void operator()(const internal_node&) override;

    /**
     * @brief Returns the phrases found by visitor
     * @return
     */
    std::vector<TTP::TaskDescriptor> getTaskDescs();

  private:
    /// The storage for the task descriptors found so far
    std::vector<TTP::TaskDescriptor> task_descs_;
};


}
}

#endif
