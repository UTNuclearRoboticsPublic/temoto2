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

class TaskVerbal
{
public:
    TaskVerbal(std::string verb,
               std::vector< meta::sequence::observation > observations)
        : verb_(verb),
          observations_(observations)
    {}

    TaskVerbal(std::string verb)
        : verb_(verb)
    {}

    std::string getVerb()
    {
        return verb_;
    }

    std::vector< meta::sequence::observation > getObservations()
    {
        return observations_;
    }

private:
    std::string verb_;
    std::vector< meta::sequence::observation > observations_;
};

typedef std::vector<TaskVerbal> TasksRaw;

std::ostream& operator<<(std::ostream& out, const TasksRaw& t)
{
    // Start printing out the verbs and according sequences
    for( auto task_verbal : t)
    {
        out << "\nPotential task: '" << task_verbal.getVerb() << "'\n";

        if( !task_verbal.getObservations().empty() )
        {
            out << "Potential args: ";

            for( auto& obs : task_verbal.getObservations() )
            {
                out << obs.symbol() << " ";
            }
        }

        out << std::endl;
    }

    return out;
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
                    if ( !branch.verb_phrases_.empty() )
                    {
                        std::cout << "Action:" <<  branch.verb_phrases_[0] << std::endl;
                    }

                    if ( !branch.noun_phrases_.empty() )
                    {
                        std::cout << "What:" <<  branch.noun_phrases_[0] << std::endl;
                    }

                    if ( !branch.prep_phrases_.empty() )
                    {
                        std::cout << "Where:" <<  branch.prep_phrases_[0] << std::endl;
                    }

                    std::cout << std::endl;
                }

            }
            else
            {
                seq.add_symbol(sequence::symbol_t{token});
            }
        }
/*
        // tag a sequence
        const auto& ana = analyzer; // access the analyzer via const ref
                                    // so that no new feature ids are generated
        ana.analyze(seq);
        tagger.tag(seq);

        // Vector for storing the base form verbs
        std::vector< std::pair<std::string, int> > verbs;
*/
/*
        // print the tagged sequence
        for( unsigned int i=0; i<seq.size(); i++ )
        {
            if( static_cast<std::string>(analyzer.tag(seq[i].label())) == "VB" )
            {
                verbs.push_back( {seq[i].symbol(), i} );
            }

            std::cout << seq[i].symbol() << "(" << analyzer.tag(seq[i].label()) << ") ";
        }
        std::cout << "\n";
*/
     /*
        // print the tagged sequence
        for( unsigned int i=0; i<seq.size(); i++ )
        {
            std::cout << seq[i].symbol() << " (" << seq[i].label() << ")" << std::endl;
        }
        std::cout << "\n";
     */
/*
        // Divide the intitial sequence by verbs
        TasksRaw tasks_raw;

        // First, check if any verbs were found
        if( verbs.empty() )
        {
            std::cout << "No verbs were found\n\n";
            continue;
        }

        // Check if the sequnece contained more than just this verb
        if(verbs.size() > 1)
        {
            for(unsigned int i=0; i<verbs.size()-1; i++)
            {
                // Are there any other descriptive words between two verbs?
                if( (verbs[i+1].second - verbs[i].second) > 1)
                {
                    // Get the indices in the main sequence
                    auto first = seq.begin() + verbs[i].second + 1;  // +1 for excluding the verb itself
                    auto last = seq.begin() + verbs[i+1].second;

                    // Extract the subsequence and push it into the sequences
                    std::vector< meta::sequence::observation > sub_seq(first, last);
                    tasks_raw.emplace_back( verbs[i].first, sub_seq );
                }
                else
                {
                    tasks_raw.emplace_back( verbs[i].first );
                }
            }
        }
        // And now add the last verb (or the first if its the only one)
        if( seq.size() - verbs.back().second > 1)
        {
            auto first = seq.begin() + verbs.back().second + 1;  // +1 for excluding the verb itself
            auto last = seq.end();

            // Extract the subsequence and push it into the sequences
            std::vector< meta::sequence::observation > sub_seq(first, last);
            tasks_raw.emplace_back( verbs.back().first, sub_seq );
        }
        else
        {
            tasks_raw.emplace_back( verbs.back().first );
        }

        // Print out the extracted verbs and potential arguments
        std::cout << tasks_raw << std::endl;
*/
        ros::Duration(0.5).sleep();
    }

    return 0;
}
