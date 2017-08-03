/* * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *     TODO: * Start using ros cpp naming conventions
 *           * (?) include starting time variable
 *           * Include a "signature" variable that i2ndicates
 *             a unique combination of the name and i/o variables
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "core/task_handler/task_info.h"

std::string TaskInfo::getPath() const
{
    return this->path_;
}

std::string TaskInfo::getLibPath() const
{
    return this->libPath_;
}

std::string TaskInfo::getName() const
{
    return this->name_;
}

std::string TaskInfo::getClassName() const
{
    return this->class_name_;
}

std::string TaskInfo::getPackageName() const
{
    return this->packageName_;
}

ParamList TaskInfo::getArgs() const
{
    return this->args_;
}

ParamList TaskInfo::getReturn() const
{
    return this->return_;
}

std::ostream& operator<<(std::ostream& out, const TaskInfo& t)
{
    out << std::endl;
    out << "------------------------------------------------" << std::endl;
    out << "* name: " << t.getName() << std::endl;
    out << "* package name: " << t.getPackageName() << std::endl;
    out << "* path: " << t.getPath() << std::endl;
    out << "* lib path: " << t.getLibPath() << std::endl;

    // Put the input arguments into the stream
    out << "* input arguments: " << std::endl;

    for( auto & args : t.getArgs() )
    {
        out << "    (";

        for( unsigned int i=0; i<args.size(); i++)
        {
            out << args[i].first;

            if( args[i].first.compare("string") == 0)
            {
                out << " [accepts: ";
                for (int j=0; j<args[i].second.size(); j++)
                {
                    out << args[i].second[j];
                    if (j != (args[i].second.size() - 1))
                        out << ",";
                }
                out << "]";
            }

            if( i == args.size() - 1 )
            {
                out << ")" << std::endl;
            }

            else
            {
                out << ", ";
            }
        }
    }

    // Put the return types into the stream
    out << "* return values: " << std::endl;

    for( auto & rets : t.getReturn() )
    {
        out << "    (";

        for( unsigned int i=0; i<rets.size(); i++)
        {
            out << rets[i].first;

            if( i == rets.size() - 1 )
            {
                out << ")" << std::endl;
            }

            else
            {
                out << ", ";
            }
        }
    }
    out << "------------------------------------------------" << std::endl;

    return out ;
}
