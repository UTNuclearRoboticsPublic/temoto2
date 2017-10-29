#include "TTP/task_tree.h"
#include <utility>
#include <iostream>

namespace TTP
{

/*
 * --------------- Task Tree implementations ---------------
 */
TaskTree::TaskTree( TaskTreeNode root_node )
{
    this->root_node_ = std::move(root_node);
}

std::ostream& operator<<( std::ostream& stream, const TaskTree& tt)
{
    stream << tt.root_node_ << std::endl;
    return stream;
}

TaskTreeNode& TaskTree::getRootNode()
{
    return root_node_;
}

void TaskTree::printTaskDescriptors(TaskTreeNode& node)
{
    std::cout << node.getTaskDescriptor();
    for (auto& child : node.getChildren())
    {
        printTaskDescriptors(child);
    }
}

/*
 * --------------- Task Tree Builder implementations ---------------
 */
TaskTree TaskTreeBuilder::build( std::vector<TaskDescriptor>& input_task_descs )
{
    /*
     * Create the first (root) node. The root node just points to the first tasks
     */
    TaskTreeNode root_node("ROOT");

    try
    {
        // Make sure "input_task_descs" is not empty
        if (input_task_descs.empty())
        {
            // THROW A TEMOTO ERROR HERE
            throw;
        }

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
                previous_node->addChildNode(std::move(node));
                active_parent_node = previous_node->lastChild();
                previous_node = active_parent_node;

            }
            else
            {
                // Add this node as a child of the active_node
                TaskTreeNode node(task_desc);
                active_parent_node->addChildNode(std::move(node));
                previous_node = active_parent_node->lastChild();
            }
        }
    }
    catch(...)
    {
        // CATCH TEMOTO ERRORS HERE
    }

    return TaskTree(std::move(root_node));
}

bool TaskTreeBuilder::checkIfDependent(TaskDescriptor& task_descriptor)
{
    bool isDependent = false;

    // Check the "whats"
    std::vector<Subject>& subjects = task_descriptor.getFirstInputSubjects();
    for (auto& subject : subjects)
    {
        // Look for "what" types
        if (subject.type_ == "what")
        {
            if (subject.words_[0] == "it")
            {
                /*
                 * Mark this "what" phrase as incomplete. This indicates that
                 * this phase has to be filled with additional information during
                 * runtime. Also create a copy of this subject into node's
                 * "incomplete_subjects" set
                 */
                subject.markIncomplete();
                task_descriptor.addIncompleteSubject(subject);

                isDependent = true;
            }
            else
            {
                subject.markComplete();
            }
        }

        // Look for "where" types
        else if (subject.type_ == "where")
        {
            if (subject.words_[0] == "there")
            {
                /*
                 * Mark this "where" phrase as incomplete. This indicates that
                 * this phase has to be filled with additional information during
                 * runtime. Also create a copy of this subject into node's
                 * "incomplete_subjects" set
                 */
                subject.markIncomplete();
                task_descriptor.addIncompleteSubject(subject);

                isDependent = true;
            }
            else
            {
                subject.markComplete();
            }
        }

        // TODO: Look for other types
    }

    return isDependent;
}


}
