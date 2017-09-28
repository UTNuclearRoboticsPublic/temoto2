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

    std::vector<parser::parse_tree> parse_trees;

    void operator()(const leaf_node&) override;
    void operator()(const internal_node&) override;

    /**
     * Returns the leaf nodes found by the visitor. This should be called
     * to extract the leaves after the visitor has been run. The leaves are
     * moved into the return value, so the visitor will be left empty by
     * this operation.
     */
    std::vector<std::unique_ptr<leaf_node>> leaves();

  private:
    /// The storage for the leaf nodes found so far
    std::vector<std::unique_ptr<leaf_node>> leaves_;

    // Cathegory of the previous node
    class_label parent_category_;

};


}
}

#endif
