/**
 * @file branch_finder.cpp
 * @author Robert Valner
 */

#include "meta/parser/trees/internal_node.h"
#include "meta/parser/trees/leaf_node.h"
#include "core/language_processor/visitors/branch_finder.h"
#include "meta/parser/trees/visitors/leaf_node_finder.h"
#include "meta/util/shim.h"
#include <iostream>
#include <sstream>
#include <algorithm>

std::vector<std::string> verb_categories = {"VP", "SINV", "S", "SQ"};
std::vector<std::string> noun_categories = {"NP"};
std::vector<std::string> prep_categories = {"PP"};
std::vector<std::string> advb_categories = {"ADVP"};

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

// TODO DESC
std::string nodesAsString (parser::parse_tree& tree)
{
    // Visit the tree and get the leaf nodes
    parser::leaf_node_finder lnf_;
    tree.visit(lnf_);
    std::vector<std::unique_ptr<parser::leaf_node>> leaves = lnf_.leaves();

    std::stringstream ss;

    // Put the leaves into the stringstream
    for(auto& leaf : leaves)
    {
        ss << *leaf->word() << " ";
    }

    return ss.str();
}

void branch_finder::operator()(const leaf_node& ln)
{
    /*
     * Do something with leaf nodes if necessary
     */
}

void branch_finder::operator()(const internal_node& in)
{
    // stupid little variable for indicating wether all the words under a branch
    // belong together or not
    bool dive = true;

    // Check if it is a verb phrase node
    if( checkIfPhrase( in.category(), in.child(0)->category(), verb_categories) )
    {
        TTP::IODescriptor task_desc;
        parser::parse_tree tree( in.child(0)->clone() );

        task_desc.setAction( nodesAsString(tree) );
        task_descs_.push_back( task_desc );
    }

    // Check if noun phrase
    else
    if ( checkIfPhrase( in.category(), in.child(0)->category(), noun_categories) )
    {
        parser::parse_tree tree( in.clone() );
        task_descs_.back().addWhat( nodesAsString(tree) );
    }


    // Check if adverb phrase
    else
    if ( checkIfPhrase( in.category(), in.child(0)->category(), advb_categories) )
    {
        parser::parse_tree tree( in.clone() );
        task_descs_.back().addWhereAdv( nodesAsString(tree) );
    }

    // Check if preposition phrase
    else
    if ( checkIfPhrase( in.category(), in.child(0)->category(), prep_categories) )
    {
        parser::parse_tree tree( in.clone() );
        task_descs_.back().addWhere( nodesAsString(tree) );

        dive = false;
    }

    if(dive)
    {
        // Dive in
        in.each_child([&](const node* n)
                      {
                          n->accept(*this);
                      });
    }
}

std::vector<TTP::IODescriptor> branch_finder::getTaskDescs()
{
    return std::move(task_descs_);
}
}
}
