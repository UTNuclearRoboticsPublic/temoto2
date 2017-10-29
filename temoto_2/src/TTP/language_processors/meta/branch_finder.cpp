/**
 * @file branch_finder.cpp
 * @author Robert Valner
 */

#include "meta/parser/trees/internal_node.h"
#include "meta/parser/trees/leaf_node.h"
#include "TTP/language_processors/meta/branch_finder.h"
#include "meta/parser/trees/visitors/leaf_node_finder.h"
#include "meta/analyzers/filters/porter2_stemmer.h"
#include "meta/util/shim.h"
#include <iostream>
#include <sstream>
#include <algorithm>

std::vector<std::string> verb_phrases = {"VP", "SINV", "S", "SQ"};
std::vector<std::string> noun_phrases = {"NP"};
std::vector<std::string> prep_phrases = {"PP"};
std::vector<std::string> advb_phrases = {"ADVP"};

std::vector<std::string> verb_categories = {"VB", "VBD", "VBG", "VBN", "VBP", "VBZ"};

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
    if (std::find(categories.begin(), categories.end(), current_category.id_) == categories.end())
    {
        return false;
    }

    // Check if the childs category is not another branching category
    if (std::find(categories.begin(), categories.end(), child_category.id_) != categories.end())
    {
        return false;
    }

    return true;
}

// TODO DESC
bool checkIfContains(class_label category, std::vector<std::string>& categories)
{
    if (std::find(categories.begin(), categories.end(), category.id_) != categories.end())
    {
        return true;
    }

    return false;
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
        // If the category of the leaf is "Determiner", then skip this word ("the", "a", "an")
        if(leaf->category().id_ == "DT")
        {
            continue;
        }

        // If the category of the leaf is "subordinating conjunction", then skip this word ("in", "on")
        // TODO: Actually this is relevant information but at the moment im not using it
        if(leaf->category().id_ == "IN")
        {
            continue;
        }

        ss << *leaf->word();
        if (&leaf != &leaves.back())
        {
            ss << " ";
        }
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
    if( checkIfPhrase( in.category(), in.child(0)->category(), verb_phrases) &&
        checkIfContains(in.child(0)->category(), verb_categories))
    {
        parser::parse_tree tree( in.child(0)->clone() );

        // Stem the verb
        std::string action = nodesAsString(tree);
        analyzers::filters::porter2::stem(action);

        verb_count_++;

        if (next_verb_is_stopped_ == true)
        {
            task_descs_.back().getFirstInputSubjects().emplace_back("action", action);
            next_verb_is_stopped_ = false;
        }
        else
        {
            if (action == "stop" && verb_count_ == 1)
            {
                next_verb_is_stopped_ = true;
            }

            // Create a task descriptor
            TTP::TaskDescriptor task_desc( action );
            task_descs_.push_back( task_desc );
        }
    }

    // Check if noun phrase
    else
    if ( checkIfPhrase( in.category(), in.child(0)->category(), noun_phrases) )
    {
        parser::parse_tree tree( in.clone() );

        if (task_descs_.empty())
        {
            addressable_ = nodesAsString(tree);
        }
        else
        {
            task_descs_.back().getFirstInputSubjects().emplace_back("what", nodesAsString(tree));
        }
    }


    // Check if adverb phrase
    else
    if ( checkIfPhrase( in.category(), in.child(0)->category(), advb_phrases) )
    {
        if (!task_descs_.empty())
        {
            parser::parse_tree tree( in.clone() );
            task_descs_.back().getFirstInputSubjects().emplace_back("where", nodesAsString(tree));
        }
    }

    // Check if preposition phrase
    else
    if ( checkIfPhrase( in.category(), in.child(0)->category(), prep_phrases) )
    {
        if (!task_descs_.empty())
        {
            parser::parse_tree tree( in.clone() );
            task_descs_.back().getFirstInputSubjects().emplace_back("where", nodesAsString(tree));

            dive = false;
        }
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

std::vector<TTP::TaskDescriptor> branch_finder::getTaskDescs()
{
    return std::move(task_descs_);
}

std::string branch_finder::getAddressable()
{
    return addressable_;
}
}
}
