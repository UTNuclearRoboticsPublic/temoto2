/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "generate_package_widget.h"
#include "temoto_action_assistant/semantic_frame_yaml.h"
#include "std_msgs/String.h"

#include <QVBoxLayout>
#include <QFileDialog>
#include <QFormLayout>

#include <boost/algorithm/string.hpp>
#include <iostream>

namespace temoto_action_assistant
{

// ******************************************************************************************
// Constructor
// ******************************************************************************************
GeneratePackageWidget::GeneratePackageWidget(QWidget* parent, ActionDescriptorPtr action_descriptor)
: SetupScreenWidget(parent),
  action_descriptor_(action_descriptor)
{
  // Layout for "add/remove selected" buttons
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Add header
  HeaderWidget* header = new HeaderWidget("Generate Action Package",
                                          "Generate Action Package", this);
  layout->addWidget(header);
  layout->addSpacerItem(new QSpacerItem(1,50, QSizePolicy::Expanding, QSizePolicy::Fixed));

  /*
   * Create a form layout for the package name and path editors
   */
  QFormLayout* form_layout = new QFormLayout();

  // Add the package name field
  package_name_field_ = new QLineEdit(this);
  form_layout->addRow("Action Package Name:", package_name_field_);
  connect(package_name_field_, SIGNAL(textEdited(const QString)), this, SLOT(modifyPackageName(const QString)));

  // Add the package path field
  package_path_field_ = new QLineEdit(this);
  form_layout->addRow("Action Package Path:", package_path_field_);
  connect(package_path_field_, SIGNAL(returnPressed()), this, SLOT(modifyRootDir()));

  // action package path selection button
  btn_root_dir_ = new QPushButton("&or Select the Path", this);
  btn_root_dir_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_root_dir_->setMaximumWidth(200);
  connect(btn_root_dir_, SIGNAL(clicked()), this, SLOT(setRootDir()));
  form_layout->addWidget(btn_root_dir_);
  form_layout->setAlignment(btn_root_dir_, Qt::AlignRight);

  layout->addLayout(form_layout);
  layout->addSpacerItem(new QSpacerItem(1,50, QSizePolicy::Expanding, QSizePolicy::Fixed));

  /*
   * Generate package Button
   */
  btn_generate_package_ = new QPushButton("&Generate", this);
  btn_generate_package_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  connect(btn_generate_package_, SIGNAL(clicked()), this, SLOT(generatePackage()));
  layout->addWidget(btn_generate_package_);
  layout->setAlignment(btn_generate_package_, Qt::AlignCenter);

  layout->addStretch();
  this->setLayout(layout);

  // Initialize the publisher
  action_descriptor_publisher_ = nh_.advertise<std_msgs::String>( ACTION_DESCRIPTOR_TOPIC, 1 );

}

// ******************************************************************************************
//
// ******************************************************************************************
void GeneratePackageWidget::modifyPackageName(const QString &text)
{
  /*
   * Remove whitespaces and change to lower case
   */
  std::string path = text.toStdString();
  boost::algorithm::to_lower(path);
  boost::replace_all(path, " ", "_");

  /*
   * Remove all non alphanumeric elements except "_"
   */
  std::string path_alnum;
  for(char& c : path)
  {
    if (std::isalnum(c) || std::string(1, c)=="_")
    {
      path_alnum += c;
    }
  }

  /*
   * Remove repetitive "_" characters
   */
  std::string before = path_alnum;
  std::string after = path_alnum;
  do
  {
    before = after;
    boost::replace_all(after, "__", "_");
  }
  while (before != after);
  path_alnum = after;

  /*
   * Add the "ai_" prefix
   */
  if (!boost::contains(path_alnum, "ai_"))
  {
    path_alnum = "ai_" + path_alnum;
  }

  /*
   * Create action class name
   */
  std::string action_class_name;
  std::vector<std::string> tokens;
  boost::split(tokens, path_alnum, boost::is_any_of("_"));

  for (std::string token : tokens)
  {
    token[0] = std::toupper(token[0]);
    action_class_name += token;
  }

  /*
   * Modify the action descriptor
   */
  action_descriptor_->action_pkg_name_ = path_alnum;
  action_descriptor_->action_class_name_ = action_class_name;

  package_name_field_->setText(QString::fromStdString(path_alnum));
}

// ******************************************************************************************
//
// ******************************************************************************************
void GeneratePackageWidget::modifyRootDir()
{
  action_descriptor_->action_pkg_path_ = package_path_field_->text().toStdString();
}

// ******************************************************************************************
//
// ******************************************************************************************
void GeneratePackageWidget::setRootDir()
{
  QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                             "~/",
                                             QFileDialog::ShowDirsOnly
                                             | QFileDialog::DontResolveSymlinks);

  package_path_field_->setText(dir);
  action_descriptor_->action_pkg_path_ = dir.toStdString();
}

// ******************************************************************************************
//
// ******************************************************************************************
void GeneratePackageWidget::generatePackage()
{
  // TODO: Make sure that a name was given for the package
  // TODO: Make sure that there isnt any other ros packages with the same name

  // Convert the action descriptor to a yaml node
  YAML::Node action_descriptor_node = YAML::Node(*action_descriptor_);
  //std::cout << action_descriptor_node << std::endl;

  // Dump the action descriptor data to a std_msgs::String message
  std_msgs::String action_descriptor_msg;
  action_descriptor_msg.data = YAML::Dump(action_descriptor_node);

  // Publish the message
  action_descriptor_publisher_.publish(action_descriptor_msg);
}

} // temoto action assistant namespace
