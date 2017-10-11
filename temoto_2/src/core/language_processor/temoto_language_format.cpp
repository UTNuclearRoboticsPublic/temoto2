#include "core/language_processor/temoto_language_format.h"
#include <iostream>

namespace TLF
{

void TaskDescriptor::setAction( Action action )
{
    action_ = action;
}

void TaskDescriptor::addWhat( std::string word )
{
    What what;
    what.word = word;
    whats_.push_back(what);
}

void TaskDescriptor::addWhere( std::string word )
{
    Where where;
    where.word = word;
    wheres_.push_back(where);
}

void TaskDescriptor::addWhereAdv( std::string word )
{
    WhereAdv where_adv;
    where_adv.word = word;
    where_advs_.push_back(where_adv);
}

Action TaskDescriptor::getAction() const
{
    return action_;
}

const std::vector<What>& TaskDescriptor::getWhats() const
{
    return whats_;
}

const std::vector<Where>& TaskDescriptor::getWheres() const
{
    return wheres_;
}

const std::vector<Where>& TaskDescriptor::getWhereAdvs() const
{
    return where_advs_;
}

std::ostream& operator<<( std::ostream& stream, const TLF::TaskDescriptor& td)
{
    // Print out the action
    if (td.getAction() != "")
    {
        stream << "Action: " << td.getAction() << std::endl;
    }

    // Print out the whats
    if (!td.getWhats().empty())
    {
        stream << "What: ";
        for (auto& what : td.getWhats())
        {
            stream << what.word << ", ";
        }
        stream << std::endl;
    }

    // Print out the wheres (prepositions)
    if (!td.getWheres().empty())
    {
        stream << "Where (prep/kus): ";
        for (auto& where : td.getWheres())
        {
            stream << where.word << ", ";
        }
        stream << std::endl;
    }

    // Print out the wheres (adverbs)
    if (!td.getWhereAdvs().empty())
    {
        stream << "Where (adv/kuhu): ";
        for (auto& where : td.getWhereAdvs())
        {
            stream << where.word << ", ";
        }
        stream << std::endl;
    }

    return stream;
}

}
