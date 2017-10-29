#include "TTP/task_tree_node.h"
#include <utility>
#include <iostream>

namespace TTP
{

TaskTreeNode::TaskTreeNode(){}

TaskTreeNode::TaskTreeNode( Action action )
{
    // Create a task descriptor
    TaskDescriptor task_descriptor( action );

    // Assign it to the local task descriptor
    task_descriptor_ = std::move( task_descriptor );
}

TaskTreeNode::TaskTreeNode( Action action, std::vector<Subject>& input_subjects )
{
    // Create a task descriptor
    TaskDescriptor task_descriptor( action, input_subjects );

    // Assign it to the local task descriptor
    task_descriptor_ = std::move( task_descriptor );
}

TaskTreeNode::TaskTreeNode( TaskDescriptor task_descriptor ) : task_descriptor_( task_descriptor ){}

void TaskTreeNode::addChildNode( TaskTreeNode child )
{
    child_nodes_.push_back(std::move(child));
}


TaskTreeNode* TaskTreeNode::lastChild()
{
    return &(child_nodes_.back());
}

TaskDescriptor& TaskTreeNode::getTaskDescriptor()
{
    return task_descriptor_;
}

std::vector<TaskTreeNode>& TaskTreeNode::getChildren()
{
    return child_nodes_;
}


std::ostream& operator<<( std::ostream& stream, const TaskTreeNode& ttn)
{
    stream << ttn.task_descriptor_.getAction();

    // If the child nodes are not empty, then print them out
    if (!ttn.child_nodes_.empty())
    {
        stream << "->(";
        for (auto& child : ttn.child_nodes_)
        {
            stream << child;
            if (&child != &ttn.child_nodes_.back())
            {
                stream << ", ";
            }
        }
        stream << ")";
    }

    return stream;
}

}// END of TPP namespace
