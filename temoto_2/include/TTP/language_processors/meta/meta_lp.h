#ifndef META_LP_H
#define META_LP_H

#include "TTP/language_processors/meta/branch_finder.h"
#include "TTP/task_tree.h"

#include "meta/parser/sr_parser.h"
#include "meta/sequence/perceptron.h"

namespace TTP
{

class MetaLP
{
public:

    /// Default constructor
    MetaLP() = default;

    /**
     * @brief Constructor
     * @param Base path to language model files
     */
    MetaLP(std::string language_models_dir);

    /**
     * @brief Looks for any potential tasks from input text and
     * sets them in hierarchical order,i.e., task tree
     * @param input_text
     * @return
     */
    TaskTree processText(std::string input_text);

private:

    const std::string class_name_ = "MetaLP";

    meta::sequence::perceptron tagger_;

    meta::parser::sr_parser parser_;

    TaskTreeBuilder task_tree_builder_;

    TaskTree task_tree_;
};

}// END of TTP namespace

#endif
