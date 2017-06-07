#include "temoto_2/language_processor/language_processor.h"
//BROKEN
LanguageProcessor::setTaskToArgBook( &(this->taskToArgBook_) )


/* * * * * * * * *
 *  PROCESS TEXT
 * * * * * * * * */
//BROKEN
TaskList LanguageProcessor::processText (std::string my_text)
{
    // Create empty tasklist
    TaskList taskList;

    // Extract the sentences
    std::vector <std::string> sentences = this->parseString(my_text, '.');

    std::cout << "--------------------------------------------------------" << std::endl;
    std::cout << "found " << sentences.size() << " sentences\n" << std::endl;

    // Analyze the sentences
    for(std::string sentence : sentences)
    {
        // Extract words, delimited by whitespace
        std::vector <std::string> words = this->parseString(sentence, ' ');
        std::vector <std::string> tempWords;

        std::cout << "sentence to analyze: '" << sentence << "'" << std::endl;

        for(int i=0; i<words.size(); i++)
        {
            // Check it it is a known command
            for (auto& command : commands)
            {
                if ( words[i].compare(command.first) == 0)
                {
                    std::cout << "  + : " << words[i] << ": known, ";

                    // Check if it takes arguments
                    if ( !command.second.empty() )
                    {
                        std::cout << "arguments needed" << std::endl;

                        // Remove the strings before the command, including command itself
                        tempWords = words;
                        tempWords.erase(tempWords.begin(), tempWords.begin() + i+1);

                        // Get the arguments
                        std::vector<boost::any> arguments = this->extractArguments(tempWords, command.second);
                        taskList.push_back ( std::pair<std::string, std::vector<boost::any>>(command.first, arguments) );

                    }
                    else
                    {
                        std::cout << "arguments NOT needed" << std::endl;
                    }

                    break;
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

std::vector<boost::any> LanguageProcessor::extractArguments (std::vector<std::string> in_strs, std::vector<std::string> arg_types)
{
    // Create a vector for storing arguments
    std::vector<boost::any> arguments;

    unsigned int nrOfArgs = arg_types.size();
    std::cout << "    searching for " << nrOfArgs << " arguments of type: ";

    // Print out the args
    for (std::string arg_type : arg_types)
    {
        std::cout << "'" << arg_type << "', ";
    }
    std::cout << std::endl;

    std::cout << "    input string size: " << in_strs.size() << std::endl;

    // Start searching for the arguments
    for (std::string arg_type : arg_types)
    {
        // Search for integers
        if (arg_type.compare("int") == 0)
        {
            int argInt;
            if (lookForInt(&argInt, &in_strs))
            {
                std::cout << "    got int: " << argInt << std::endl;
                std::cout << "    remaining string size: " << in_strs.size() << std::endl;

                // Push the argument to the argument vector
                arguments.push_back( boost::any_cast<int>(argInt) );
            }
        }

        // Search for strings
        if (arg_type.compare("string") == 0)
        {
            std::string argStr;
            if (lookForStr(&argStr, &in_strs))
            {
                std::cout << "    got string: " << argStr << std::endl;
                std::cout << "    remaining string size: " << in_strs.size() << std::endl;

                // Push the argument to the argument vector
                arguments.push_back( boost::any_cast<std::string>(argStr) );
            }
        }
    }

    return arguments;
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

int LanguageProcessor::lookForInt (int * returnInt, std::vector<std::string> * in_strs)
{
    // Create a vector where to store the integers

    for(int i=0; i<in_strs->size(); i++)
    {
        // Check if the first character is a digit
        if (isdigit( in_strs->at(i)[0] ))
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

int LanguageProcessor::lookForStr (std::string * returnStr, std::vector<std::string> * in_strs)
{
    // Create a vector where to store the integers

    for(int i=0; i<in_strs->size(); i++)
    {
        // Check if the first character is NOT a digit
        if (!isdigit( in_strs->at(i)[0] ))
        {
            // Convert to int and remove the str
            *returnStr = in_strs->at(i);
            in_strs->erase( in_strs->begin() + i );

            return 1;
        }
    }
    return 0;
}


