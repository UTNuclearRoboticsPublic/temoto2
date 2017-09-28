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

std::vector<std::string> verb_categories = {"VP", "SINV", "S"};
std::vector<std::string> noun_categories = {"NP"};
std::vector<std::string> prep_categories = {"PP"};

namespace meta
{
namespace parser
{

// TODO DESC
bool checkIfPhrase( class_label current_category,
                    class_label child_category,
                    std::vector<std::string>& categories )
{
    // Check if it is a valid branching category
    if ( std::find(categories.begin(),
                   categories.end(),
                   current_category.id_) == categories.end() )
    {
        return false;
    }

    // Check if the childs category is not another branching category
    if ( std::find(categories.begin(),
                   categories.end(),
                   child_category.id_) != categories.end())
    {
        return false;
    }

    return true;
}

void branch_finder::operator()(const leaf_node& ln)
{
    /*
     * Do something with leaf nodes if necessary
     */
}

void branch_finder::operator()(const internal_node& in)
{
    // Check if it is a verb phrase node
    if( checkIfPhrase( in.category(), in.child(0)->category(), verb_categories) )
    {
        Branch br;
        br.verb_phrases_.emplace_back(in.child(0)->clone());
        branches_.push_back(br);
    }

    // Check if noun phrase
    else
    if ( checkIfPhrase( in.category(), in.child(0)->category(), noun_categories) )
    {
        branches_.back().noun_phrases_.emplace_back(in.clone());
    }

    // Check if preposition phrase
    else
    if ( checkIfPhrase( in.category(), in.child(0)->category(), prep_categories) )
    {
        branches_.back().prep_phrases_.emplace_back(in.clone());
    }

    // Dive in
    in.each_child([&](const node* n)
                  {
                      n->accept(*this);
                  });
}

std::vector<Branch> branch_finder::getBranches()
{
    return std::move(branches_);
}
}
}
