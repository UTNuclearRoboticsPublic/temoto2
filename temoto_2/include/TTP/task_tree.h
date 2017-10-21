#ifndef TASK_TREE_H
#define TASK_TREE_H

#include "TTP/task_tree_node.h"

namespace TTP
{

/**
 * @brief The TaskTree class
 */
class TaskTree
{
public:

    TaskTree() = default;

    /**
     * @brief TaskTree
     * @param root_node
     */
    TaskTree( TaskTreeNode root_node);

    friend std::ostream& operator<<( std::ostream& stream, const TaskTree& tt);

    TaskTreeNode& getRootNode();

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

    /**
     * @brief TaskTreeBuilder
     */
    TaskTreeBuilder(){}

    /**
     * @brief build
     * @param input_task_descs
     * @return
     */
    TaskTree build( std::vector<TaskDescriptor>& input_task_descs );

private:

    bool checkIfDependent(TaskDescriptor& task_descriptor);
};
}
#endif
