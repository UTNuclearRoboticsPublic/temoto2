#include "ros/ros.h"
#include "ros/package.h"
#include "TTP/task_descriptor_processor.h"
#include "file_template_parser/file_template_parser.h"

std::map<std::string, std::string> data_map = {
  {"number", "float"},
  {"string", "std::string"},
  {"topic", "std::string"},
  {"pointer", "<YOUR DATATYPE HERE>"}};

// main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ai_package_generator");
  ros::NodeHandle n;
  std::string base_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/test/";



  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  std::string ai_name = "TestAI";
  /*
   * Create a dummy task descriptor
   */

  std::string action = "track";
  TTP::Subjects subjects;

  TTP::Subject sub_0("what", "apple");
  TTP::Subject sub_1("where", "table");

  sub_1.addData("topic", std::string("no_data"));
  sub_1.addData("number", 12.4);

  subjects.push_back(sub_0);
  subjects.push_back(sub_1);

  TTP::TaskInterface interface_0;
  interface_0.id_ = 0;
  interface_0.type_ = "synchronous";
  interface_0.input_subjects_ = subjects;
  interface_0.output_subjects_ = subjects;

  TTP::TaskInterface interface_1;
  interface_1.id_ = 1;
  interface_1.type_ = "synchronous";
  interface_1.input_subjects_ = subjects;

  std::vector<TTP::TaskInterface> interfaces;
  interfaces.push_back(interface_0);
  interfaces.push_back(interface_1);

  TTP::TaskDescriptor ai_descriptor(action, interfaces);
  ai_descriptor.setTaskPackageName("Name of the action");

  std::cout << ai_descriptor << std::endl;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



  const std::vector<TTP::TaskInterface>& ai_interfaces = ai_descriptor.getInterfaces();
  const TTP::Action& ai_lexical_unit = ai_descriptor.getAction();
  const std::string ai_package_name = ai_descriptor.getTaskPackageName();

  // String for maintaining the generated content
  std::string generated_content;

  /*
   * Import the templates
   */
  tp::TemplateContainer t_header(base_path + "file_templates/ai_header.xml");
  tp::TemplateContainer t_start_task(base_path + "file_templates/ai_start_task.xml");
  tp::TemplateContainer t_case(base_path + "file_templates/ai_start_task_case.xml");
  tp::TemplateContainer t_get_solution(base_path + "file_templates/ai_get_solution.xml");
  tp::TemplateContainer t_start_interface(base_path + "file_templates/ai_start_interface.xml");
  tp::TemplateContainer t_subject_in(base_path + "file_templates/ai_subject_in.xml");
  tp::TemplateContainer t_subject_in_data(base_path + "file_templates/ai_subject_in_data.xml");
  tp::TemplateContainer t_footer(base_path + "file_templates/ai_footer.xml");

  /*
   * Modify the templates according to the task descriptor
   */

  // Insert the header
  t_header.setArgument("ai_class_name", ai_name);
  generated_content += t_header.processTemplate();

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
  generated_content += t_start_task.processTemplate();

  // Insert the "get solution" method
  generated_content += t_get_solution.processTemplate();

  // Generate "start interface" methods
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
    generated_input_subjects += "/* EXTRACTING INPUT SUBJECTS */";

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
        t_subject_in_data.setArgument("subject_type", ai_subject->type_);
        t_subject_in_data.setArgument("subject_number", std::to_string(sub_idx));
        t_subject_in_data.setArgument("data_type", data_map.at(ai_data->type));
        t_subject_in_data.setArgument("data_number", std::to_string(data_idx));

        generated_input_subject += t_subject_in_data.processTemplate();
      }

      generated_input_subjects += generated_input_subject;
    }

    /*
     * Loop through output subjects
     */
    std::string generated_output_subjects;
    sub_idx = 0;

    for (auto ai_subject = ai_interface->output_subjects_.begin();
         ai_subject != ai_interface->output_subjects_.end();
         ai_subject++, sub_idx++)
    {
      std::string generated_output_subject;

      /* extract the data */

      generated_output_subjects += generated_output_subject;
    }

    t_start_interface.setArgument("interface_nr", std::to_string(i));
    t_start_interface.setArgument("subjects_in", generated_input_subjects);
    t_start_interface.setArgument("subjects_out", generated_output_subjects);

    generated_content += t_start_interface.processTemplate();
  }

  // Insert the footer
  t_footer.setArgument("ai_class_name", ai_name);
  generated_content += t_footer.processTemplate();

  /*
   * Save the generated content
   */
  tp::saveStrToFile(generated_content, base_path, "ai_test", ".cpp");

}
