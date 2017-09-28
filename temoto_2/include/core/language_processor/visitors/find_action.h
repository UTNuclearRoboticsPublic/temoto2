/**
 * @file branch_finder.h
 * @author Robert valner
 *
 * All files in META are released under the MIT license. For more details,
 * consult the file LICENSE in the root of the project.
 */

#ifndef META_PARSER_ACTION_FINDER_H_
#define META_PARSER_ACTION_FINDER_H_

#include <memory>
#include <vector>

#include "meta/parser/trees/visitors/visitor.h"
#include "meta/parser/sr_parser.h"

namespace meta
{
namespace parser
{

/**
 * This is a visitor that finds and extracts a branch in a parse tree.
 */
class find_action : public const_visitor<void>
{
  public:

    void operator()(const leaf_node&) override;
    void operator()(const internal_node&) override;

    /**
     * @brief Returns the branches found by visitor
     * @return
     */
    std::string getAction();

  private:
    /// The storage for the parse trees found so far
    std::string action_;

    bool action_found = false;

    std::vector<std::string> branching_categories = {"VP", "SINV", "S"};
    std::vector<std::string> verb_categories = {"VB", "VBD", "VBG", "VBN", "VBP", "VBZ"};
};


}
}

#endif
