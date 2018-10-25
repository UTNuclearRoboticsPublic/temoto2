#ifndef META_LP_H
#define META_LP_H

#include "TTP/language_processors/meta/branch_finder.h"
#include "TTP/task_tree.h"

#include "meta/parser/sr_parser.h"
#include "meta/sequence/perceptron.h"
#include "common/base_subsystem.h"

namespace TTP
{

class MetaLP : BaseSubsystem
{
public:

  /**
   * @brief MetaLP
   * @param language_models_dir
   * @param b
   * @param wake_word
   */
  MetaLP( std::string language_models_dir
        , BaseSubsystem& b
        , std::string wake_word = "");

  /**
   * @brief Looks for any potential tasks from input text and
   * sets them in hierarchical order,i.e., task tree
   * @param input_text
   * @return
   */
  TaskTree processText(std::string input_text);

private:

  bool checkIfWakeWord(std::string word);

  meta::sequence::perceptron tagger_;

  meta::parser::sr_parser parser_;

  TaskTree task_tree_;

  std::shared_ptr<nummap> str_int_map_;

  std::string wake_word_;

  std::vector<std::string> wake_words_ = {"everybody"};
};

}// END of TTP namespace

#endif
