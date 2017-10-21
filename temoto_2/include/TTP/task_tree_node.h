#ifndef TASK_TREE_NODE_H
#define TASK_TREE_NODE_H

#include "TTP/task_descriptor.h"

namespace TTP
{

// Forward declaration of the task manager
class TaskManager;

/**
 * @brief The TaskTreeNode class
 */
class TaskTreeNode
{
    friend TaskManager;

public:

    /**
     * @brief A default constructor that is used for creating root nodes only
     */
    TaskTreeNode();

    TaskTreeNode( Action action );

    TaskTreeNode( Action action, std::vector<Subject>& input_subjects );

    TaskTreeNode( TaskDescriptor task_descriptor );

    friend std::ostream& operator<<( std::ostream& stream, const TaskTreeNode& ttn);

    void addChildNode( TaskTreeNode child );


    TaskTreeNode* lastChild();

    TaskDescriptor& getTaskDescriptor();

    std::vector<TaskTreeNode>& getChildren();

private:

    TaskDescriptor task_descriptor_;

    std::vector<TaskTreeNode> child_nodes_;
};

}// END of TPP namespace

#endif
