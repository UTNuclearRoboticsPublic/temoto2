#ifndef TEMOTO_LANGUAGE_FORMAT_H
#define TEMOTO_LANGUAGE_FORMAT_H

#include <boost/any.hpp>
#include <string>
#include <vector>

/**
 * Temoto Language Format
 */
namespace TLF
{

/**
 * @brief The Data struct
 */
struct Data
{
    std::string type = "";
    boost::any payload;
};

// Valid datatypes
const std::vector<std::string> valid_datatypes = {"topic",
                                                  "number",
                                                  "pointer",
                                                  "other"};

/**
 * @brief The Subject struct
 */
struct Subject
{
    std::string word;
    Data data;
};

typedef std::string Action;
typedef Subject What;
typedef Subject Where;
typedef Subject WhereAdv;
typedef Subject Numerical;

/**
 * @brief The TaskDescriptor struct
 */
class TaskDescriptor
{
public:

    void setAction( Action action );

    void addWhat( std::string word );

    void addWhere( std::string word );

    void addWhereAdv( std::string word );

    Action getAction() const;

    const std::vector<What>& getWhats() const;

    const std::vector<Where>& getWheres() const;

    const std::vector<WhereAdv>& getWhereAdvs() const;


private:

    Action action_;
    std::vector<What> whats_;
    std::vector<Where> wheres_;
    std::vector<WhereAdv> where_advs_;
    std::vector<Numerical> numerics_;
};

std::ostream& operator<<( std::ostream& stream, const TaskDescriptor& td);
}
#endif
