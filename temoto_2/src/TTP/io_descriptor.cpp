#include "TTP/io_descriptor.h"
#include <iostream>

namespace TTP
{

Subject::Subject(std::string type, std::string word)
    : type_(type)
{
    words_.push_back(word);
}

void Subject::markIncomplete()
{
    is_complete_ = false;
}

void Subject::markComplete()
{
    is_complete_ = true;
}

// Subjects streaming operator
std::ostream& operator<<( std::ostream& stream, const std::vector<Subject>& subjects)
{

    // Print out the whats
    if (!subjects.empty())
    {
        for (const auto& subject : subjects)
        {
            stream << "| |_ " << subject;
        }
        stream << "|" << std::endl;
    }

    return stream;
}

// Subject streaming operator
std::ostream& operator<<( std::ostream& stream, const Subject& subject)
{
    // Print out the type
    stream << subject.type_ << ": ";

    // Print out if the subject is complete or not
    if (subject.is_complete_)
    {
        stream << " Complete";
    }
    else
    {
        stream << " Incomplete";
    }

    stream << " [";

    // Print out the word (candidate words)
    for (auto& word : subject.words_)
    {
        stream << word;
        if (&word != &subject.words_.back())
        {
            stream << ", ";
        }
    }

    stream << "]";

    // Print out the data
    if (!subject.data_.empty())
    {
        stream << " + data {";
        for (auto& data : subject.data_)
        {
            stream << data;
            if (&data != &subject.data_.back())
            {
                stream << ", ";
            }
        }
        stream << "}";
    }

    stream << std::endl;

    return stream;
}

// Data streaming operator
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
