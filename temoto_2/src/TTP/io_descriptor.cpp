#include "TTP/io_descriptor.h"
#include <iostream>

namespace TTP
{

/*
 * SETTERS
 */

// What
void IODescriptor::addWhat( What what )
{
    whats_.push_back(what);
}

void IODescriptor::addWhat( std::string word )
{
    What what;
    what.words.push_back(word);
    whats_.push_back(what);
}

// Where
void IODescriptor::addWhere( Where where )
{
    wheres_.push_back(where);
}

void IODescriptor::addWhere( std::string word )
{
    Where where;
    where.words.push_back(word);
    wheres_.push_back(where);
}

void IODescriptor::addWhereAdv( std::string word )
{
    WhereAdv where_adv;
    where_adv.words.push_back(word);
    where_advs_.push_back(where_adv);
}

// Numeric
void IODescriptor::addNumeric( Numeric numeric )
{
    numerics_.push_back(numeric);
}

void IODescriptor::addNumeric( std::string word )
{
    Numeric numeric;
    numeric.words.push_back(word);
    numerics_.push_back(numeric);
}


/*
 * GETTERS
 */

const std::vector<What>& IODescriptor::getWhats() const
{
    return whats_;
}

const std::vector<Where>& IODescriptor::getWheres() const
{
    return wheres_;
}

const std::vector<Where>& IODescriptor::getWhereAdvs() const
{
    return where_advs_;
}

std::ostream& operator<<( std::ostream& stream, const TTP::IODescriptor& td)
{
    // Print out the whats
    if (!td.getWhats().empty())
    {
        stream << "  What: ";
        for (auto& what : td.getWhats())
        {
            stream << what;
        }
        stream << std::endl;
    }

    // Print out the wheres (prepositions)
    if (!td.getWheres().empty())
    {
        stream << "  Where (prep/kus): ";
        for (auto& where : td.getWheres())
        {
            stream << where;
        }
        stream << std::endl;
    }

    // Print out the wheres (adverbs)
    if (!td.getWhereAdvs().empty())
    {
        stream << "  Where (adv/kuhu): ";
        for (auto& where : td.getWhereAdvs())
        {
            stream << where;
        }
        stream << std::endl;
    }

    return stream;
}

std::ostream& operator<<( std::ostream& stream, const Subject& subject)
{
    stream << "[";
    for (auto& word : subject.words)
    {
        stream << word;
        if (&word != &subject.words.back())
        {
            stream << ", ";
        }
    }

    stream << "]";
    if (!subject.data.empty())
    {
        stream << " + data {";
        for (auto& data : subject.data)
        {
            stream << data;
            if (&data != &subject.data.back())
            {
                stream << ", ";
            }
        }
        stream << "}";
    }

    stream << "\n        ";

    return stream;
}

std::ostream& operator<<( std::ostream& stream, const Data& data)
{
    stream << "[" << data.type;
    if (!data.value.empty())
    {
        try
        {
            if (data.type == "string")
            {
                stream << " : " << boost::any_cast<std::string>(data.value);
            }

            if (data.type == "topic")
            {
                stream << " : " << boost::any_cast<std::string>(data.value);
            }

            if (data.type == "number")
            {
                stream << " : " << boost::any_cast<double>(data.value);
            }
        }
        catch (boost::bad_any_cast& e)
        {
            stream << " : " << e.what();
        }
    }
    stream << "]";

    return stream;
}

}
