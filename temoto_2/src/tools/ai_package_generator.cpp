#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"

#include "TTP/task_descriptor_processor.h"
#include "temoto_action_assistant/semantic_frame_yaml.h"
#include "common/semantic_frame_legacy.h"
#include "file_template_parser/file_template_parser.h"
#include <boost/filesystem.hpp>

const std::string ACTION_DESCRIPTOR_TOPIC = "action_descriptor";

std::map<std::string, std::string> data_map = {
  {"number", "float"},
  {"string", "std::string"},
  {"topic", "std::string"},
  {"pointer", "<YOUR DATATYPE HERE>"}};

std::map<std::string, std::string> data_padding_map = {
  {"number", "       "},
  {"string", " "},
  {"topic", " "},
  {"pointer", "    "}};

// "Declaring" the templates
tp::TemplateContainer t_cmakelists;
tp::TemplateContainer t_packagexml;

tp::TemplateContainer t_header;
tp::TemplateContainer t_start_task;
tp::TemplateContainer t_case;
tp::TemplateContainer t_get_solution;
tp::TemplateContainer t_start_interface;

tp::TemplateContainer t_subject_in;
tp::TemplateContainer t_subject_in_data_in;
tp::TemplateContainer t_subject_in_word_out;
tp::TemplateContainer t_subject_in_data_out;

tp::TemplateContainer t_subject_out;
tp::TemplateContainer t_subject_out_data_out;

tp::TemplateContainer t_footer;

// ******************************************************************************************
// Generate Package function
// ******************************************************************************************
void generatePackage(temoto_action_assistant::ActionDescriptor& action_descriptor)
{
  // Convert the action descriptor to legacy format
  TTP::TaskDescriptor ai_descriptor = toLegacyTaskDescriptor(action_descriptor);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   *                    CREATE AI PACKAGE DIRECTORY STRUCTURE
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  // Get the name of the package
  const std::string ai_package_name = ai_descriptor.getTaskPackageName();
  const std::string ai_dst_path = ai_descriptor.getLibPath() + "/" + ai_package_name + "/";

  // Create a package directory
  boost::filesystem::create_directories(ai_dst_path + "/src");

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   *                           GENERATE THE CONTENT
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


  /*
   * Extract the necessary datastructures from the ai_descriptor
   */
  const std::vector<TTP::TaskInterface>& ai_interfaces = ai_descriptor.getInterfaces();
  const TTP::Action& ai_lexical_unit = ai_descriptor.getAction();
  std::string ai_class_name = ai_descriptor.getTaskClassName();

  /*
   * Generate descriptor.xml
   */
  TTP::saveAIDescriptor(ai_descriptor, ai_dst_path + "descriptor.xml");

  /*
   * Generate CMakeLists.txt
   */
  t_cmakelists.setArgument("ai_name", ai_package_name);
  t_cmakelists.processAndSaveTemplate(ai_dst_path, "CmakeLists");

  /*
   * Generate package.xml
   */
  t_packagexml.setArgument("ai_name", ai_package_name);
  t_packagexml.processAndSaveTemplate(ai_dst_path, "package");

  /*
   * Generate the action implementation c++ source file
   */

  // String for maintaining the generated content
  std::string generated_content_cpp;

  // Insert the header
  t_header.setArgument("ai_class_name", ai_class_name);
  generated_content_cpp += t_header.processTemplate();

  // Generate the start task cases
  std::string generated_cases;
  unsigned int nr_of_interfaces = ai_descriptor.getInterfaces().size();
  for (unsigned int i=0; i<nr_of_interfaces; i++)
  {
    t_case.setArgument("interface_nr", std::to_string(i));
    generated_cases += t_case.processTemplate();
  }

  // Insert the "start task" method
  t_start_task.setArgument("cases", generated_cases);
  generated_content_cpp += t_start_task.processTemplate();

  // Insert the "get solution" method
  generated_content_cpp += t_get_solution.processTemplate();

  /*
   * Generate "start interface" methods
   */
  unsigned int i = 0;
  for (auto ai_interface = ai_interfaces.begin();
       ai_interface != ai_interfaces.end();
       ai_interface++, i++)
  {
    /*
     * Loop through input subjects
     */
    std::string generated_input_subjects;
    unsigned int sub_idx = 0;
    generated_input_subjects += "/* EXTRACTION OF INPUT SUBJECTS */";

    for (auto ai_subject = ai_interface->input_subjects_.begin();
         ai_subject != ai_interface->input_subjects_.end();
         ai_subject++, sub_idx++)
    {
      std::string generated_input_subject;
      t_subject_in.setArgument("type", ai_subject->type_);
      t_subject_in.setArgument("number", std::to_string(sub_idx));

      generated_input_subject += t_subject_in.processTemplate();

      /*
       * extract the data
       */
      unsigned int data_idx = 0;

      for (auto ai_data = ai_subject->data_.begin();
           ai_data != ai_subject->data_.end();
           ai_data++, data_idx++)
      {
        t_subject_in_data_in.setArgument("subject_type", ai_subject->type_);
        t_subject_in_data_in.setArgument("subject_number", std::to_string(sub_idx));
        t_subject_in_data_in.setArgument("data_type", data_map.at(ai_data->type));
        t_subject_in_data_in.setArgument("data_number", std::to_string(data_idx));
        t_subject_in_data_in.setArgument("data_padding", data_padding_map.at(ai_data->type));

        generated_input_subject += t_subject_in_data_in.processTemplate();
      }

      generated_input_subjects += generated_input_subject;
    }

    /*
     * Loop through output subjects
     */
    if (!ai_interface->output_subjects_.empty())
    {
      generated_input_subjects += "\n  /* DECLARATION OF OUTPUT SUBJECTS */";
    }

    std::string generated_output_subjects;
    sub_idx = 0;

    for (auto ai_subject = ai_interface->output_subjects_.begin();
         ai_subject != ai_interface->output_subjects_.end();
         ai_subject++, sub_idx++)
    {

      // Insert the declaration of output argumens
      t_subject_in_word_out.setArgument("subject_type", ai_subject->type_);
      t_subject_in_word_out.setArgument("subject_number", std::to_string(sub_idx));
      generated_input_subjects += t_subject_in_word_out.processTemplate();

      /*
       * extract the data
       */
      std::string generated_output_data;
      unsigned int data_idx = 0;

      for (auto ai_data = ai_subject->data_.begin();
           ai_data != ai_subject->data_.end();
           ai_data++, data_idx++)
      {
        t_subject_in_data_out.setArgument("subject_type", ai_subject->type_);
        t_subject_in_data_out.setArgument("subject_number", std::to_string(sub_idx));
        t_subject_in_data_out.setArgument("data_type", data_map.at(ai_data->type));
        t_subject_in_data_out.setArgument("data_number", std::to_string(data_idx));
        t_subject_in_data_out.setArgument("data_padding", data_padding_map.at(ai_data->type));

        t_subject_out_data_out.setArgument("subject_type", ai_subject->type_);
        t_subject_out_data_out.setArgument("subject_number", std::to_string(sub_idx));
        t_subject_out_data_out.setArgument("data_type", data_map.at(ai_data->type));
        t_subject_out_data_out.setArgument("data_number", std::to_string(data_idx));
        t_subject_out_data_out.setArgument("data_literal", ai_data->type);

        generated_input_subjects += t_subject_in_data_out.processTemplate();
        generated_output_data += t_subject_out_data_out.processTemplate();
      }

      // Insert the assignment of output arguments
      t_subject_out.setArgument("subject_type", ai_subject->type_);
      t_subject_out.setArgument("subject_number", std::to_string(sub_idx));
      t_subject_out.setArgument("data", generated_output_data);

      generated_output_subjects += t_subject_out.processTemplate();
    }

    // Insert the input and output subjects to the start interface template
    t_start_interface.setArgument("interface_nr", std::to_string(i));
    t_start_interface.setArgument("subjects_in", generated_input_subjects);
    t_start_interface.setArgument("subjects_out", generated_output_subjects);

    generated_content_cpp += t_start_interface.processTemplate();
  }

  // Insert the footer
  t_footer.setArgument("ai_class_name", ai_class_name);
  generated_content_cpp += t_footer.processTemplate();

  /*
   * Save the generated c++ content
   */
  tp::saveStrToFile(generated_content_cpp, ai_dst_path + "/src", ai_package_name, ".cpp");
}

// ******************************************************************************************
// Generate Package callback
// ******************************************************************************************
void generatePackageCb(const std_msgs::String& ad_yaml_str)
{

  // Convert the yaml string to an action descriptor
  temoto_action_assistant::ActionDescriptor action_descriptor =
    YAML::Load(ad_yaml_str.data).as<temoto_action_assistant::ActionDescriptor>();

  YAML::Node tst = YAML::Node(action_descriptor);
  std::cout << tst << std::endl;

  generatePackage(action_descriptor);
}

// ******************************************************************************************
// Main
// ******************************************************************************************
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ai_package_generator");
  ros::NodeHandle n;
  std::string base_template_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/test/";

  // Subscriber that generates the action implementation package
  ros::Subscriber ad_subscriber = n.subscribe(ACTION_DESCRIPTOR_TOPIC, 10, generatePackageCb);


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   *                           IMPORT THE TEMPLATES
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  // Import the CMakeLists template
  t_cmakelists           = tp::TemplateContainer(base_template_path + "file_templates/temoto_ai_cmakelists.xml");

  // Import the package.xml template
  t_packagexml           = tp::TemplateContainer(base_template_path + "file_templates/temoto_ai_packagexml.xml");

  // Import the action implementation c++ code templates
  t_header               = tp::TemplateContainer(base_template_path + "file_templates/ai_header.xml");
  t_start_task           = tp::TemplateContainer(base_template_path + "file_templates/ai_start_task.xml");
  t_case                 = tp::TemplateContainer(base_template_path + "file_templates/ai_start_task_case.xml");
  t_get_solution         = tp::TemplateContainer(base_template_path + "file_templates/ai_get_solution.xml");
  t_start_interface      = tp::TemplateContainer(base_template_path + "file_templates/ai_start_interface.xml");

  t_subject_in           = tp::TemplateContainer(base_template_path + "file_templates/ai_subject_in.xml");
  t_subject_in_data_in   = tp::TemplateContainer(base_template_path + "file_templates/ai_subject_in_data_in.xml");
  t_subject_in_word_out  = tp::TemplateContainer(base_template_path + "file_templates/ai_subject_in_word_out.xml");
  t_subject_in_data_out  = tp::TemplateContainer(base_template_path + "file_templates/ai_subject_in_data_out.xml");

  t_subject_out          = tp::TemplateContainer(base_template_path + "file_templates/ai_subject_out.xml");
  t_subject_out_data_out = tp::TemplateContainer(base_template_path + "file_templates/ai_subject_out_data_out.xml");

  t_footer               = tp::TemplateContainer(base_template_path + "file_templates/ai_footer.xml");

  // Spin
  ros::spin();

  return 0;
}
