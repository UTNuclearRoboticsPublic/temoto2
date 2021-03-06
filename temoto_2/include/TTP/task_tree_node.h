#ifndef TASK_TREE_NODE_H
#define TASK_TREE_NODE_H

#include "TTP/task_descriptor.h"
#include "tbb/flow_graph.h"
#include <boost/shared_ptr.hpp>

namespace TTP
{

// Forward declaration of the task manager
class TaskManager;

// Forward declaration of base task
class BaseTask;

/**
 * @brief The TaskTreeNode class
 */
class TaskTreeNode
{
    friend TaskManager;

public:

    // TODO: MAKE PRIVATE!!!!!!!!!!
    std::unique_ptr< tbb::flow::broadcast_node<Subjects> > root_fgn_;

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

    boost::shared_ptr<BaseTask> task_pointer_;

    // TODO: This is a hack and it should not exist. But currently this is necessary
    // because flow graph internally updates the task descriptors and the updated
    // task descriptors are needed for stopping the tasks correctly
    boost::shared_ptr<TaskDescriptor> task_descriptor_ptr_;

    std::unique_ptr< tbb::flow::function_node<Subjects, Subjects> > task_fgn_;

    std::vector<TaskTreeNode> child_nodes_;
};

}// END of TPP namespace

#endif
