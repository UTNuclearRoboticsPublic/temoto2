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
    boost::any value;
};

std::ostream& operator<<( std::ostream& stream, const Data& data);

// Valid datatypes
const std::vector<std::string> valid_datatypes = {"topic",
                                                  "number",
                                                  "pointer",
                                                  "other",
                                                  "string"};

/**
 * @brief The Subject struct
 */
struct Subject
{
    std::vector<std::string> words;
    std::string pos_tag;
    std::vector<Data> data;
};

std::ostream& operator<<( std::ostream& stream, const Subject& subject);

typedef Subject What;
typedef Subject Where;
typedef Subject WhereAdv;
typedef Subject Numeric;

/**
 * @brief The IODescriptor struct
 */
class IODescriptor
{
public:

    IODescriptor(){}

    /*
     * SETTERS
     */

    void addWhat( std::string word );

    void addWhat( What what );


    void addWhere( std::string word );

    void addWhere( Where where );


    void addWhereAdv( std::string word );


    void addNumeric( std::string word );

    void addNumeric( Numeric numeric );


    /*
     * GETTERS
     */

    const std::vector<What>& getWhats() const;

    const std::vector<Where>& getWheres() const;

    const std::vector<WhereAdv>& getWhereAdvs() const;

    const std::vector<Numeric>& getNumerics() const;


private:

    std::vector<What> whats_;
    std::vector<Where> wheres_;
    std::vector<WhereAdv> where_advs_;
    std::vector<Numeric> numerics_;
};

std::ostream& operator<<( std::ostream& stream, const IODescriptor& td);
}
#endif
