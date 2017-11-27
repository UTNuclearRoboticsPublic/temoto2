#ifndef IO_DESCRIPTOR_H
#define IO_DESCRIPTOR_H

#include <boost/any.hpp>
#include <string>
#include <vector>

/**
 * Temoto Tasking Protocol
 */
namespace TTP
{

// Forward declare the Subject class and create a Subjects typedef
class Subject;
typedef std::vector<Subject> Subjects;

Subject getSubjectByType(const std::string& type, Subjects& subjects);

/**
 * @brief The Data struct
 */
struct Data
{
    std::string type = "";
    boost::any value;

    Data(){}

    Data(std::string type_in, boost::any value_in) : type(type_in), value(value_in){}
};

std::ostream& operator<<( std::ostream& stream, const Data& data);

bool operator==(const Data& d1, const Data& d2);

bool operator==(const std::vector<Data>& dv1, const std::vector<Data>& dv2);

// Valid datatypes
const std::vector<std::string> valid_datatypes = {"topic",
                                                  "number",
                                                  "pointer",
                                                  "other",
                                                  "string"};

// Valid subjects
const std::vector<std::string> valid_subjects = {"what",
                                                 "where",
                                                 "numeric",
                                                 "action"};

/**
 * @brief The Subject struct
 */
class Subject
{
public:

    std::string type_;
    std::string pos_tag_;
    std::vector<std::string> words_;
    std::vector<Data> data_;
    bool is_complete_ = false;

    Subject() = default;

    Subject(std::string type, std::string word);

    friend bool operator==(const std::vector<Subject>& subs_1, const std::vector<Subject>& subs_2);

    void addData(std::string datatype, float data);
    void addData(std::string datatype, std::string data);

    void markIncomplete();
    void markComplete();

private:

    std::string class_name_ = "Subject";
};

std::ostream& operator<<( std::ostream& stream, const Subject& subject);

std::ostream& operator<<( std::ostream& stream, const std::vector<Subject>& subjects);

}
#endif
