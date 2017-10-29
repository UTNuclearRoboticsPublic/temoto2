#include "common/tools.h"
#include "TTP/language_processors/meta/meta_lp.h"
#include "TTP/language_processors/meta/branch_finder.h"
#include "TTP/TTP_errors.h"

#include "meta/analyzers/tokenizers/icu_tokenizer.h"
#include "meta/sequence/sequence.h"
#include "meta/parser/trees/visitors/leaf_node_finder.h"
#include "meta/parser/trees/internal_node.h"
#include "meta/parser/trees/leaf_node.h"

namespace TTP
{

MetaLP::MetaLP(std::string language_models_dir)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", class_name_, __func__);

    // load POS-tagging model
    std::cout << prefix << " Loading tagging model ... " << std::flush;
    tagger_ = meta::sequence::perceptron(language_models_dir + "perceptron-tagger");
    std::cout << "done\n";

    // load parser model
    std::cout << prefix << " Loading parser model ... " << std::flush;
    parser_ = meta::parser::sr_parser(language_models_dir + "parser");
    std::cout << "done\n";
}

TaskTree MetaLP::processText(std::string input_text)
{
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = common::generateLogPrefix("", class_name_, __func__);

    // If the input string is empty then cause panic
    if (input_text.empty())
    {
        // Throw error
        error::ErrorStackUtil error_stack_util( TTPErr::NLP_INV_ARG,
                                                error::Subsystem::CORE,
                                                error::Urgency::LOW,
                                                prefix + " Received an empty string",
                                                ros::Time::now() );
        throw error_stack_util;
    }

    meta::sequence::sequence seq;

    std::unique_ptr<meta::analyzers::token_stream> stream = meta::make_unique<meta::analyzers::tokenizers::icu_tokenizer>();
    stream->set_content(std::move(input_text));

    while (*stream)
    {
        auto token = stream->next();
        if (token == "<s>")
        {
            seq = {};
        }
        else if (token == "</s>")
        {
            tagger_.tag(seq);
            meta::parser::parse_tree p_tree = parser_.parse(seq);

            // Create a parse tree branch finder visitor
            meta::parser::branch_finder bf;

            /*
             * Extract the potential tasks from the parse tree. Tasks are
             * returned as a vector of task descriptors
             */
            p_tree.visit(bf);
            std::vector<TTP::TaskDescriptor> task_descs = bf.getTaskDescs();

            // If potential tasks were found then ...
            if (task_descs.size() > 0)
            {
                std::cout << "Found " << task_descs.size() << " potential tasks addressed to: " << bf.getAddressable() << std::endl;

                // Build a task tree
                task_tree_ = task_tree_builder_.build(task_descs);

                // Print out the tasks after being parsed
                for( auto& task_descriptor : task_descs )
                {
                    std::cout << task_descriptor << std::endl;
                }
            }
            else
            {
                std::cout << prefix << "No tasks were found\n";
                // Throw error
                error::ErrorStackUtil error_stack_util( TTPErr::NLP_BAD_INPUT,
                                                        error::Subsystem::CORE,
                                                        error::Urgency::LOW,
                                                        prefix + " Could not make any sense of input text",
                                                        ros::Time::now() );
                throw error_stack_util;
            }
        }
        else
        {
            seq.add_symbol(meta::sequence::symbol_t{token});
        }
    }

    return std::move(task_tree_);
}

}// END of TTP namespace
