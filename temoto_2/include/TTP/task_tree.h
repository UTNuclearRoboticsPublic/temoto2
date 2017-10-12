#ifndef TASK_TREE_H
#define TASK_TREE_H

#include "TTP/task_descriptor.h"

namespace TTP
{

/**
 * @brief The TaskTreeNode class
 */
class TaskTreeNode
{
public:

    /**
     * @brief A default constructor that is used for creating root nodes only
     */
    TaskTreeNode();

    TaskTreeNode( Action action );

    TaskTreeNode( Action action, IODescriptor io_descriptor );

    TaskTreeNode( TaskDescriptor task_descriptor );

    friend std::ostream& operator<<( std::ostream& stream, const TaskTreeNode& ttn);

    void addChildNode( TaskTreeNode child );

    TaskTreeNode* lastChild();

private:

    TaskDescriptor task_descriptor_;

    std::vector<TaskTreeNode> child_nodes_;
};

/**
 * @brief The TaskTree class
 */
class TaskTree
{
public:
    TaskTree( TaskTreeNode root_node);

    friend std::ostream& operator<<( std::ostream& stream, const TaskTree& tt);

private:
    TaskTreeNode root_node_;
};

/**
 * @brief The TaskTreeBuilder class
 */
class TaskTreeBuilder
{
public:

    //TaskTree build( std::vector<IODescriptor> input_descs );

    TaskTreeBuilder(){}

    TaskTree build( std::vector<TaskDescriptor>& input_task_descs );

private:

    bool checkIfDependent(TaskDescriptor& task_desc) const;
};
}
#endif
