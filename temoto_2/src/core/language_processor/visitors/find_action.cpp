/**
 * @file leaf_node_finder.cpp
 * @author Chase Geigle
 */

#include "meta/parser/trees/internal_node.h"
#include "meta/parser/trees/leaf_node.h"
#include "core/language_processor/visitors/find_action.h"
#include "meta/util/shim.h"
#include <iostream>
#include <algorithm>

namespace meta
{
namespace parser
{

// TODO DESC
bool checkCategory( class_label current_category, std::vector<std::string>& categories )
{
    if ( std::find(categories.begin(),
                   categories.end(),
                   current_category.id_) == categories.end() )
    {
        return false;
    }
    else
    {
        return true;
    }
}

void find_action::operator()(const leaf_node& ln)
{
    if( checkCategory(ln.category(), this->verb_categories) )
    {
       action_ = *(ln.word());
       action_found = true;
    }
}

void find_action::operator()(const internal_node& in)
{
    if( checkCategory(in.category(), this->branching_categories) && action_found == false)
    {
        in.each_child([&](const node* n)
                      {
                          n->accept(*this);
                      });
    }
}

std::string find_action::getAction()
{
    return action_;
}
}
}
