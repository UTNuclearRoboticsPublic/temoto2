#ifndef LANGUAGE_PROCESSOR_H
#define LANGUAGE_PROCESSOR_H

#include "core/task_handler/common.h"
#include <sstream>
#include <map>
#include <string>
#include <vector>
#include <boost/any.hpp>
#include <iostream>
#include <ctype.h>
#include <utility>


class LanguageProcessor
{

public:
    /**
     * @brief processText
     * @param my_text
     * @return
     */
    TaskList processText (std::string my_text);

private:

    // A vector containting known commands and the arguments that these accept
    std::map<std::string, std::vector<std::string>> * taskToArgBook;

    //std::map<std::string, std::vector<std::string>> * taskToArgBook = { {"add", {"int", "int"}},
    //                                                             {"turn", {"string"}}
    //                                                           };

    /**
     * @brief parseString
     * @param in_str
     * @param delimiter
     * @return
     */
    std::vector<std::string> parseString (std::string in_str, char delimiter);

    /**
     * @brief extractArguments
     * @param in_strs
     * @param arg_types
     * @return
     */
    std::vector<boost::any> extractArguments (std::vector<std::string> in_strs, std::vector<std::string> arg_types);

    /**
     * @brief lookForInt
     * @param returnInt
     * @param in_strs
     * @return
     */
    int lookForInt (int * returnInt, std::vector<std::string> * in_strs);

    /**
     * @brief lookForStr
     * @param returnStr
     * @param in_strs
     * @return
     */
    int lookForStr (std::string * returnStr, std::vector<std::string> * in_strs);
};

#endif
