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

    TaskTreeNode( IODescriptor io_descriptor );

    TaskTreeNode( TaskDescriptor task_descriptor );

    void addChildNode( std::unique_ptr<TaskTreeNode> child );

private:

    TaskDescriptor task_descriptor_;

    std::vector<std::unique_ptr<TaskTreeNode>> child_nodes_;
};

/**
 * @brief The TaskTree class
 */
class TaskTree
{
public:
    TaskTree( std::unique_ptr<TaskTreeNode> root_node);

private:
    std::unique_ptr<TaskTreeNode> root_node_;
};

/**
 * @brief The TaskTreeBuilder class
 */
class TaskTreeBuilder
{
public:

    TaskTree build( std::vector<IODescriptor> input_descs );

    TaskTree build( std::vector<TaskDescriptor>& input_task_descs );
};
}
#endif
