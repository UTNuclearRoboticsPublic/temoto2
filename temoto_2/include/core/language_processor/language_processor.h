#ifndef LANGUAGE_PROCESSOR_H
#define LANGUAGE_PROCESSOR_H

#include "core/common.h"
#include <sstream>
#include <boost/any.hpp>
#include <iostream>
#include <ctype.h>


class LanguageProcessor
{

public:
    /**
     * @brief processText
     * @param my_text
     * @return
     */
    TaskList processText (std::string my_text);

    /**
     * @brief setTasksIndeed
     * @return
     */
    bool setTasksIndexed (std::vector <TaskInfo> tasksIndexed);

private:

    /**
     * @brief tasksIndexed_
     */
    std::vector <TaskInfo> tasksIndexed_;

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
     * @param args_list
     * @param extractedArguments
     * @param ignore_next_match
     * @return
     */
    bool extractArguments (std::vector<std::string>& in_strs,
                           ParamList args_list,
                           std::vector<boost::any>& extractedArguments,
                           bool& ignore_next_match);

    /**
     * @brief lookForInt
     * @param returnInt
     * @param in_strs
     * @return
     */
    int lookForInt (int * returnInt, std::vector<std::string> * in_strs, std::vector<std::string> restrictions);

    /**
     * @brief lookForStr
     * @param returnStr
     * @param in_strs
     * @return
     */
    int lookForStr (std::string * returnStr, std::vector<std::string> * in_strs, std::vector<std::string> restrictions);

    /**
     * @brief lookForTask
     * @param taskName
     * @param in_strs
     * @param restrictions
     * @return
     */
    bool lookForTask (std::string * taskName, std::vector<std::string> * in_strs, std::vector<std::string> restrictions);

    /**
     * @brief checkRestrictions
     * @param inputWord
     * @param restrictions
     * @return
     */
    bool checkRestrictions (std::string inputWord, std::vector<std::string> restrictions);

    /**
     * @brief checkTask
     * @param taskName
     * @return
     */
    bool checkTask (std::string taskName);
};

#endif
