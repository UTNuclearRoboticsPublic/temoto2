#include "TTP/task_tree.h"
#include <utility>
#include <iostream>

namespace TTP
{

/*
 * --------------- Task Tree Node implementations ---------------
 */
TaskTreeNode::TaskTreeNode(){}

TaskTreeNode::TaskTreeNode( Action action )
{
    // Create a task descriptor
    TaskDescriptor task_descriptor( action );

    // Assign it to the local task descriptor
    task_descriptor_ = std::move( task_descriptor );
}

TaskTreeNode::TaskTreeNode( Action action, IODescriptor io_descriptor )
{
    // Create a task descriptor
    TaskDescriptor task_descriptor( action, io_descriptor );

    // Assign it to the local task descriptor
    task_descriptor_ = std::move( task_descriptor );
}

TaskTreeNode::TaskTreeNode( TaskDescriptor task_descriptor ) : task_descriptor_( task_descriptor ){}

void TaskTreeNode::addChildNode( TaskTreeNode child )
{
    child_nodes_.push_back(child);
}

TaskTreeNode* TaskTreeNode::lastChild()
{
    return &(child_nodes_.back());
}

std::ostream& operator<<( std::ostream& stream, const TaskTreeNode& ttn)
{
    // If it is a root node, then print the root label
    if (ttn.task_descriptor_.getAction() == "ROOT")
    {
        stream << "ROOT";
    }
    else
    {
        stream << ttn.task_descriptor_.getAction();
    }

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

/*
 * --------------- Task Tree implementations ---------------
 */
TaskTree::TaskTree( TaskTreeNode root_node )
{
    this->root_node_ = root_node;
}

std::ostream& operator<<( std::ostream& stream, const TaskTree& tt)
{
    stream << tt.root_node_ << std::endl;
    return stream;
}

/*
 * --------------- Task Tree Builder implementations ---------------
 */
TaskTree TaskTreeBuilder::build( std::vector<TaskDescriptor>& input_task_descs )
{
    try
    {
        // Make sure "input_task_descs" is not empty
        if (input_task_descs.empty())
        {
            // THROW A TEMOTO ERROR HERE
            throw;
        }

        /*
         * Create the first (root) node. The root node just points to the first tasks
         */
        TaskTreeNode root_node("ROOT");

        /*
         * Create two intermediate nodes. These are used during tree building process
         * for pointing at active parent and previous node.
         */
        TaskTreeNode* active_parent_node = &root_node;
        TaskTreeNode* previous_node = &root_node;

        // Start building the tree
        for (auto& task_desc : input_task_descs)
        {
            /*
             * Check if this node is depending on the output results of a previous node.
             * This is evaluated based on keywords, that indicate if a node is independent or not
             */
            if (checkIfDependent(task_desc))
            {
                // Create a node, add it as a child of previous node
                TaskTreeNode node(task_desc);
                previous_node->addChildNode(node);
                active_parent_node = previous_node->lastChild();
                previous_node = active_parent_node;

            }
            else
            {
                // Add this node as a child of the active_node
                TaskTreeNode node(task_desc);
                active_parent_node->addChildNode(node);
                previous_node = active_parent_node->lastChild();
            }
        }

        return TaskTree(std::move(root_node));
    }
    catch(...)
    {
        // CATCH TEMOTO ERRORS HERE
    }
}

bool TaskTreeBuilder::checkIfDependent(TaskDescriptor& task_desc) const
{
    bool isDependent = false;

    const IODescriptor input_desc = task_desc.getFirstInputDescriptor();

    // Check the "whats"
    const std::vector<What>& whats = input_desc.getWhats();
    for (auto& what : whats)
    {
        if (what.words[0] == "it")
        {
            isDependent = true;
            break;
        }
    }

    return isDependent;
}


}
