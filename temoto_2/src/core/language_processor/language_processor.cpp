#include "core/language_processor/language_processor.h"

/* * * * * * * * *
 * SET TASKSINDEXED
 * * * * * * * * */

bool LanguageProcessor::setTasksIndexed (std::vector <TaskInfo> tasksIndexed)
{
    this->tasksIndexed_ = tasksIndexed;
    return true;
}


/* * * * * * * * *
 *  PROCESS TEXT
 * * * * * * * * */

TaskList LanguageProcessor::processText (std::string my_text)
{
    // Create empty tasklist
    TaskList taskList;

    // A bool that says if a detected task should be ignored or not
    bool ignore_next_match = false;

    // Extract the sentences
    std::vector <std::string> sentences = this->parseString(my_text, '.');

    std::cout << "--------------------------------------------------------" << std::endl;
    std::cout << "found " << sentences.size() << " sentences\n" << std::endl;

    // Analyze the sentences
    for (std::string sentence : sentences)
    {
        // Extract words, delimited by whitespace
        std::vector <std::string> words = this->parseString(sentence, ' ');
        std::vector <std::string> tempWords;

        std::cout << "sentence to analyze: '" << sentence << "'" << std::endl;

        // Loop over the words
        for (int i=0; i<words.size(); i++)
        {
            // Loop over known commands
            for (auto& command : this->tasksIndexed_)
            {
                // Check if the command matches
                if ( words[i].compare(command.getName()) == 0 )
                {
                    // Check if this match should be ignored or not
                    if (!ignore_next_match)
                    {
                        std::cout << " + : " << words[i] << ": known, ";

                        std::cout << "extracting the arguments" << std::endl;

                        // Remove the strings before the command, including command itself
                        tempWords = words;
                        tempWords.erase(tempWords.begin(), tempWords.begin() + i+1);

                        // Get the arguments in a format of vector<boost::any>
                        std::vector<boost::any> arguments;

                        if ( extractArguments(tempWords,
                                              command.getArgs(),
                                              arguments,
                                              ignore_next_match) )
                        {
                            taskList.push_back ( std::pair<TaskInfo, std::vector<boost::any>>(command, arguments) );
                        }
                        break;
                    }
                    else
                    {
                        ignore_next_match = false;
                        break;
                    }
                }
            }
        }
        std::cout << std::endl;
    }

    std::cout << "--------------------------------------------------------" << std::endl;
    return taskList;
}


/* * * * * * * * * * *
 *  EXTRACT ARGUMENTS
 * * * * * * * * * * */

bool LanguageProcessor::extractArguments (std::vector<std::string>& in_strs,
                                          ParamList args_list,
                                          std::vector<boost::any>& extractedArguments,
                                          bool& ignore_next_match)
{
    // A variable that stores information about if returning an empty list of arguments is permitted
    bool return_empty = false;

    std::cout << " This task accepts " << args_list.size() << " different sets of arguments" << std::endl;

    // Check which set of arguments is the most suitable
    for (auto& args : args_list)
    {
        std::vector<boost::any> argumentsLocal;

        // Make a copy of the input string, since it is going to be hacked and slashed
        //std::vector<std::string> in_strs_copy = in_strs;

        unsigned int nrOfArgs = args.size();
        std::cout << "    * searching for " << nrOfArgs << " arguments of type: ";

        // Print out the args
        for (auto& arg : args)
        {
            std::cout << "'" << arg.first << "', ";
        }
        std::cout << std::endl;

        // Start checking the arguments
        for (auto& arg : args)
        {
            // Search for void
            if (arg.first.compare("void") == 0)
            {
                return_empty = true;
            }

            // Search for other tasks
            else if (arg.first.compare("task") == 0)
            {
                std::string argTask;
                if (lookForTask(&argTask, &in_strs, arg.second))
                {
                    std::cout << "      - got task: " << argTask << std::endl;

                    // Push the argument to the argument vector
                    argumentsLocal.push_back( boost::any_cast<std::string>(argTask) );
                    ignore_next_match = true;
                }
            }

            // Search for numbers
            else if (arg.first.compare("number") == 0)
            {
                int argInt;
                if (lookForInt(&argInt, &in_strs, arg.second))
                {
                    std::cout << "      - got number: " << argInt << std::endl;

                    // Push the argument to the argument vector
                    argumentsLocal.push_back( boost::any_cast<int>(argInt) );
                }
            }

            // Search for strings
            else if (arg.first.compare("string") == 0)
            {
                std::string argStr;
                if (lookForStr(&argStr, &in_strs, arg.second))
                {
                    std::cout << "      - got string: " << argStr << std::endl;

                    // Push the argument to the argument vector
                    argumentsLocal.push_back( boost::any_cast<std::string>(argStr) );
                }
            }
        }

        // Now if the arguments are extracted, check if the number of extracted args
        // matches with the number of required ones
        if (argumentsLocal.size() == args.size())
        {
            // If it does, then the one that has the best match, "wins". For example say one arg list
            // is specified by 3 args and other by 5, and lets say that both got a complete match, then
            // the one which is specified with more arguments (5) "wins"
            if (argumentsLocal.size() > extractedArguments.size())
            {
                extractedArguments = argumentsLocal;
            }
        }
    }

    // Check if returning an empty list of arguments is legal or not
    if (return_empty)
        return true;
    else
        return ( (extractedArguments.empty()) ? false : true);
}


/* * * * * * * * *
 *  PARSE STRING
 * * * * * * * * */

std::vector<std::string> LanguageProcessor::parseString (std::string in_str, char delimiter)
{
    // Create a vector for storing sentences
    std::vector <std::string> strings;

    // Create a stringstream and temporary string
    std::stringstream iss(in_str);
    std::string s;

    // TEMPORARY HACK THING for getting strings separated by whitespace
    if (delimiter == ' ')
    {
        for( ; iss >> s; )
        {
            strings.push_back(s);
        }
        return strings;
    }

    // Extract the sentences
    while ( getline(iss, s, delimiter) )
    {
        strings.push_back(s);
    }

    return strings;
}


/* * * * * * * * *
 *  LOOK FOR INT
 * * * * * * * * */

int LanguageProcessor::lookForInt (int * returnInt, std::vector<std::string> * in_strs, std::vector<std::string> restrictions)
{
    // See if the restrictions apply
    bool restrictionsApply = false;
    if (!restrictions.empty())
    {
        restrictionsApply = true;
    }

    // Loop over strings
    for (int i=0; i<in_strs->size(); i++)
    {
        // First check if the restrictions apply
        if (restrictionsApply)
        {
            // If the word matches with the restriction then ...
            if (checkRestrictions(in_strs->at(i), restrictions))
            {
                *returnInt = atoi ( in_strs->at(i).c_str() );
                in_strs->erase( in_strs->begin() + i );

                return 1;
            }
        }

        // Check if the first character is a digit
        else if (isdigit( in_strs->at(i)[0] ))
        {
            // Convert to int and remove the str
            *returnInt = atoi ( in_strs->at(i).c_str() );
            in_strs->erase( in_strs->begin() + i );

            return 1;
        }
    }
    return 0;
}


/* * * * * * * * *
 *  LOOK FOR STR
 * * * * * * * * */

int LanguageProcessor::lookForStr (std::string * returnStr, std::vector<std::string> * in_strs, std::vector<std::string> restrictions)
{
    // See if the restrictions apply
    bool restrictionsApply = false;
    if (!restrictions.empty())
    {
        restrictionsApply = true;
    }

    for (int i=0; i<in_strs->size(); i++)
    {
        // First check if the restrictions apply
        if (restrictionsApply)
        {
            // If the word matches with the restriction then ...
            if (checkRestrictions(in_strs->at(i), restrictions))
            {
                // Convert to int and remove the str
                *returnStr = in_strs->at(i);
                in_strs->erase( in_strs->begin() + i );

                return 1;
            }
        }

        // Check if the first character is NOT a digit
        else if (!isdigit( in_strs->at(i)[0] ))
        {
            // Fill the return and cut the input words
            *returnStr = in_strs->at(i);
            in_strs->erase( in_strs->begin() + i );

            return 1;
        }
    }
    return 0;
}

/* * * * * * * * *
 *  LOOK FOR TASK
 * * * * * * * * */

bool LanguageProcessor::lookForTask (std::string * taskName, std::vector<std::string> * in_strs, std::vector<std::string> restrictions)
{
    // See if the restrictions apply
    bool restrictionsApply = false;
    if (!restrictions.empty())
    {
        restrictionsApply = true;
    }

    // How many words are looked through before returning false
    int max_words = 2;
    int words_to_check = (in_strs->size() < max_words) ? in_strs->size() : max_words;


    for (int i=0; i<words_to_check; i++)
    {
        // First check if the restrictions apply
        if (restrictionsApply)
        {
            // If the word matches with the restriction then ...
            if (checkRestrictions(in_strs->at(i), restrictions))
            {
                // Check if this is even a known task
                if (checkTask(in_strs->at(i)))
                {
                    // Fill the return and cut the input words
                    *taskName = in_strs->at(i);
                    in_strs->erase( in_strs->begin() + i );
                    return true;
                }
            }
        }

        // Check if the first character is NOT a digit
        else if (!isdigit( in_strs->at(i)[0] ))
        {
            // Check if this is even a known task
            if (checkTask(in_strs->at(i)))
            {
                // Fill the return and cut the input words
                *taskName = in_strs->at(i);
                in_strs->erase( in_strs->begin() + i );
                return true;
            }
        }
    }
    return false;
}


/* * * * * * * * *
 *  CHECK THE RESTRICTIONS
 * * * * * * * * */

bool LanguageProcessor::checkRestrictions (std::string inputWord, std::vector<std::string> restrictions)
{
    // Loop through the restrictions list
    for (auto& restriction : restrictions)
    {
        // If the word matches with the restriction entry
        if (restriction.compare(inputWord) == 0)
        {
            return true;
        }
    }

    return false;
}


/* * * * * * * * *
 *  CHECK THE RESTRICTIONS
 * * * * * * * * */

bool LanguageProcessor::checkTask (std::string taskName)
{
    for (auto& task : tasksIndexed_)
    {
        if (task.getName().compare(taskName) == 0)
            return true;
    }

    return false;
}
