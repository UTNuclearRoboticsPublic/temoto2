#include "TTP/task_tree.h"

namespace TTP
{

/*
 * Task Tree Node implementations
 */
TaskTreeNode::TaskTreeNode( IODescriptor io_descriptor )
{
    // Create a task descriptor
    TaskDescriptor task_descriptor( io_descriptor );

    // Assign it to the local task descriptor
    task_descriptor_ = std::move( task_descriptor );
}

TaskTreeNode::TaskTreeNode( TaskDescriptor task_descriptor ) : task_descriptor_( task_descriptor ){}

void TaskTreeNode::addChildNode( std::unique_ptr<TaskTreeNode> child )
{
    child_nodes.push_back( std::move( child ) );
}

/*
 * Task Tree implementations
 */
TaskTree::TaskTree( std::unique_ptr<TaskTreeNode> root_node )
{
    this->root_node_ = std::move( root_node );
}

/*
 * Task Tree Builder implementations
 */
TaskTree TaskTreeBuilder::build( std::vector<TaskDescriptor>& input_task_descs )
{
    try
    {
        // Make sure "input_task_descs" is not empty
        if ( input_task_descs.empty() )
        {
            // THROW A TEMOTO ERROR HERE
        }

        /*
         * Create the first (root) node. It is assumed, that the root node is going
         * to be created from the first description in the input_task_descs
         */
        TaskTreeNode root_node( input_task_descs[0] );

        TaskTreeNode& active_node = root_node;

        for ( ; ; )
        {
            if ( checkIfChild(  ) )
            {
                // Add this node as a child of the active_node
            }
        }
    }
    catch(...)
    {
        // CATCH TEMOTO ERRORS HERE
    }
}

}
