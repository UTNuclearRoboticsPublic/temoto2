#include "ros/ros.h"
#include "vector"

#include "meta/analyzers/tokenizers/icu_tokenizer.h"
#include "meta/classify/confusion_matrix.h"
#include "cpptoml.h"
#include "meta/logging/logger.h"
#include "meta/sequence/crf/crf.h"
#include "meta/sequence/crf/tagger.h"
#include "meta/sequence/io/ptb_parser.h"
#include "meta/sequence/sequence.h"
#include "meta/sequence/crf/tagger.h"

#include "meta/parser/sr_parser.h"
#include "meta/sequence/perceptron.h"

#include "meta/parser/trees/visitors/head_finder.h"
#include "meta/parser/trees/visitors/leaf_node_finder.h"
#include "meta/parser/trees/internal_node.h"
#include "meta/parser/trees/leaf_node.h"

#include "meta/parser/trees/visitors/multi_transformer.h"
#include "meta/parser/io/ptb_reader.h"

#include "core/language_processor/visitors/branch_finder.h"
#include "core/language_processor/visitors/find_action.h"

using namespace meta;

void treePrinter( std::string type, std::vector<parser::parse_tree>& trees )
{
    if ( !trees.empty() )
    {
        // Create a parse tree visitor
        parser::leaf_node_finder lnf;

        std::cout << type << ": ";

        for(auto& tree : trees)
        {
            tree.visit(lnf);
            std::vector<std::unique_ptr<parser::leaf_node>> leaves = lnf.leaves();

            // Print out the leaves
            for(auto& leaf : leaves)
            {
                std::cout << *leaf->word() << " ";
            }
            std::cout << ", ";
        }

        std::cout << std::endl;
    }
}

parser::parse_tree tree(std::string input)
{
    std::stringstream in_ss{input};
    auto in_trees = parser::io::extract_trees(in_ss);
    return std::move(in_trees.front());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pos_tagger");
    ros::NodeHandle nh;
/*
    // load the model
    meta::sequence::crf crf{"/home/robert/repos_sdks/meta/models/crf"};

    // create a tagger
    auto tagger = crf.make_tagger();

    // load the sequence analyzer (for feature generation)
    auto analyzer = meta::sequence::default_pos_analyzer();
    analyzer.load("/home/robert/repos_sdks/meta/models/crf");
*/

    ////////
    std::cout << "Loading tagging model" << std::endl;
    // load POS-tagging model
    sequence::perceptron tagger{"/home/robert/repos_sdks/meta/models/perceptron-tagger"};

    std::cout << "Loading parser model" << std::endl;
    // load parser model
    parser::sr_parser parser{"/home/robert/repos_sdks/meta/models/parser"};

    // Create a parse tree visitor
    parser::leaf_node_finder lnf;

    ////////

    std::string line;

    while (ros::ok())
    {
        meta::sequence::sequence seq;

        std::cout << " > ";
        std::getline(std::cin, line);

        if (line.empty())
            break;

        std::unique_ptr<analyzers::token_stream> stream = make_unique<analyzers::tokenizers::icu_tokenizer>();
        stream->set_content(std::move(line));

        std::ostream stream_tree(nullptr); // useless ostream (badbit set)
        std::stringbuf str;
        stream_tree.rdbuf(&str);

/*
        while (*stream)
        {
            auto token = stream->next();
            if (token == " " || token == "<s>" || token == "</s>")
                continue;
            seq.add_observation( {sequence::symbol_t{token}, sequence::tag_t{"[UNK]"}} );
        }
*/

        while (*stream)
        {
            auto token = stream->next();
            if (token == "<s>")
            {
                seq = {};
            }
            else if (token == "</s>")
            {
                tagger.tag(seq);
                parser::parse_tree p_tree = parser.parse(seq);

                // Create a parse tree branch finder visitor
                parser::branch_finder bf;

                p_tree.visit(bf);
                std::vector<parser::Branch> branches = bf.getBranches();

                std::cout << "nr of branches found: " << branches.size() << std::endl;

                for( auto& branch : branches )
                {
                    /*
                    parser::find_action fa;
                    branch.visit(fa);
                    std::cout << fa.getAction() << std::endl;
                    */

                    treePrinter( "Action", branch.verb_phrases_ );
                    treePrinter( "What", branch.noun_phrases_ );
                    treePrinter( "Where", branch.prep_phrases_ );

                    std::cout << std::endl;
                }

            }
            else
            {
                seq.add_symbol(sequence::symbol_t{token});
            }
        }

        ros::Duration(0.5).sleep();
    }

    return 0;
}
