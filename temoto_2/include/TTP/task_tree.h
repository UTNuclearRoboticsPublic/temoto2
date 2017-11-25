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

    void printTaskDescriptors(TaskTreeNode& node);

private:

    TaskTreeNode root_node_;
};

/**
 * Semantic Frame Tree Builder namespace.
 * This namespace contains a collection of functions for building a
 * semantic frame tree. The logic about how to detect dependencies
 * between SFs is contained within SFT Builder.
 */
namespace SFTBuilder
{
    TaskTree build( std::vector<TaskDescriptor>& input_task_descs );
    bool checkIfDependent(TaskDescriptor& task_descriptor);
}
}
#endif
