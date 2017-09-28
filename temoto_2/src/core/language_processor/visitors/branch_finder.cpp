/**
 * @file leaf_node_finder.cpp
 * @author Chase Geigle
 */

#include "meta/parser/trees/internal_node.h"
#include "meta/parser/trees/leaf_node.h"
#include "core/language_processor/visitors/branch_finder.h"
#include "meta/util/shim.h"
#include <iostream>
#include <algorithm>

std::vector<std::string> branching_categories = {"VP", "SINV", "S"};
std::vector<std::string> phrase_categories = {"VB", "NP", "PP"};

namespace meta
{
namespace parser
{

// TODO DESC
bool checkValidity( class_label current_category, class_label child_category )
{
    // Check if it is a valid branching category
    if ( std::find(branching_categories.begin(),
                   branching_categories.end(),
                   current_category.id_) == branching_categories.end() )
    {
        return false;
    }

    // Check if the childs category is not another branching category
    if ( std::find(branching_categories.begin(),
                   branching_categories.end(),
                   child_category.id_) != branching_categories.end())
    {
        return false;
    }

    return true;
}

void branch_finder::operator()(const leaf_node& ln)
{
    leaves_.push_back(make_unique<leaf_node>(ln));
}

void branch_finder::operator()(const internal_node& in)
{
    // Save the current parent category and replace it with the category of this node
    //class_label prev_parent_category = this->parent_category_;
    //parent_category_ = in.category();

    if( checkValidity( in.category(), in.child(0)->category() ))
    {
        parse_trees.emplace_back(in.clone());
    }

    in.each_child([&](const node* n)
                  {
                      n->accept(*this);
                  });

    // Restore the contents of the parent_category_
    //parent_category_ = prev_parent_category;
}

std::vector<std::unique_ptr<leaf_node>> branch_finder::leaves()
{
    return std::move(leaves_);
}
}
}
