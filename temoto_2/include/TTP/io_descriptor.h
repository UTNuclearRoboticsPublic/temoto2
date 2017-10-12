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

typedef Subject What;
typedef Subject Where;
typedef Subject WhereAdv;
typedef Subject Numerical;

/**
 * @brief The IODescriptor struct
 */
class IODescriptor
{
public:

    IODescriptor(){}

    void addWhat( std::string word );

    void addWhere( std::string word );

    void addWhereAdv( std::string word );

    const std::vector<What>& getWhats() const;

    const std::vector<Where>& getWheres() const;

    const std::vector<WhereAdv>& getWhereAdvs() const;


private:

    std::vector<What> whats_;
    std::vector<Where> wheres_;
    std::vector<WhereAdv> where_advs_;
    std::vector<Numerical> numerics_;
};

std::ostream& operator<<( std::ostream& stream, const IODescriptor& td);
}
#endif
