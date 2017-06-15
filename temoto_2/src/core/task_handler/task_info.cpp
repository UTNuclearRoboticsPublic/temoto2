#include "core/task_handler/task_info.h"

std::string TaskInfo::getPath() const
{
    return this->path_;
}

std::string TaskInfo::getName() const
{
    return this->name_;
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
    out << "* path: " << t.getPath() << std::endl;

    // Put the input arguments into the stream
    out << "* input arguments: " << std::endl;

    for( auto & args : t.getArgs() )
    {
        out << "    (";

        for( unsigned int i=0; i<args.size(); i++)
        {
            out << args[i];

            if( args[i].compare("string") == 0)
            {
                out << " [accepts: " << args[i+1] << "]";
                i++;
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
            out << rets[i];

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
