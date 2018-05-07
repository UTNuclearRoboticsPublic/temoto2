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

/* Author: Dave Coleman */

// Qt
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QString>
#include <QApplication>
#include <QFont>
#include <QFileDialog>
#include <QTextEdit>
// ROS
#include <ros/ros.h>
#include <ros/package.h>  // for getting file path for loadng images
// SA
#include "header_widget.h"  // title and instructions
#include "start_screen_widget.h"
// C
#include <fstream>  // for reading in urdf
#include <streambuf>
// Boost
#include <boost/algorithm/string.hpp>  // for trimming whitespace from user input
#include <boost/filesystem.hpp>        // for reading folders/files
#include <boost/algorithm/string.hpp>  // for string find and replace in paths
// MoveIt
#include <moveit/rdf_loader/rdf_loader.h>

namespace temoto_action_assistant
{
// Boost file system
namespace fs = boost::filesystem;

// ******************************************************************************************
// Start screen user interface for MoveIt Configuration Assistant
// ******************************************************************************************
StartScreenWidget::StartScreenWidget(QWidget* parent, temoto_action_assistant::ActionDescriptorPtr action_descriptor)
  : SetupScreenWidget(parent),
    action_descriptor_(action_descriptor)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Horizontal layout splitter
  QHBoxLayout* hlayout = new QHBoxLayout();
  // Left side of screen
  QVBoxLayout* left_layout = new QVBoxLayout();
  // Right side of screen
  QVBoxLayout* right_layout = new QVBoxLayout();


  logo_image_ = new QImage();
  logo_image_label_ = new QLabel(this);
  std::string image_path = "./resources/temoto_logo.png";

  if (chdir(action_descriptor_->action_pkg_path_.c_str()) != 0)
  {
    ROS_ERROR("FAILED TO CHANGE PACKAGE TO temoto_action_assistant");
  }

  if (logo_image_->load(image_path.c_str()))
  {
    logo_image_label_->setPixmap(QPixmap::fromImage(logo_image_->scaledToHeight(50)));
    logo_image_label_->setMinimumWidth(400);
  }
  else
  {
    ROS_ERROR_STREAM("FAILED TO LOAD " << image_path);
  }

  layout->addWidget(logo_image_label_);
  layout->setAlignment(logo_image_label_, Qt::AlignLeft | Qt::AlignTop);

  // Top Label Area ---------------------------------------------------
  HeaderWidget* header = new HeaderWidget(
      "Action Assistant", "Welcome to the TeMoto Action Assistant! These tools will assist you in creating a "
                                "TeMoto action package. \n UNDER DEVELOPMENT",
      this);
  layout->addWidget(header);

  // Select Mode Area -------------------------------------------------
  select_mode_ = new SelectModeWidget(this);
  connect(select_mode_->btn_new_, SIGNAL(clicked()), this, SLOT(showNewOptions()));
  connect(select_mode_->btn_exist_, SIGNAL(clicked()), this, SLOT(showExistingOptions()));
  left_layout->addWidget(select_mode_);

  // Path Box Area ----------------------------------------------------

  // Stack Path Dialog
  stack_path_ =
      new LoadPathArgsWidget("Load MoveIt Configuration Package Path",
                             "Specify the package name or path of an existing MoveIt configuration package to be "
                             "edited for your robot. Example package name: <i>pr2_moveit_config</i>",
                             "xacro arguments", this, true);  // directory
  stack_path_->hide();                                        // user needs to select option before this is shown
  stack_path_->setArgs("--inorder ");
  connect(stack_path_, SIGNAL(pathChanged(QString)), this, SLOT(onPackagePathChanged(QString)));
  left_layout->addWidget(stack_path_);

  // URDF File Dialog
  urdf_file_ = new LoadPathArgsWidget("Load a URDF or COLLADA Robot Model",
                                      "Specify the location of an existing Universal Robot Description Format or "
                                      "COLLADA file for "
                                      "your robot. The robot model will be loaded to the parameter server for you.",
                                      "xacro arguments", this, false, true);  // no directory, load only
  urdf_file_->hide();  // user needs to select option before this is shown
  urdf_file_->setArgs("--inorder ");
  connect(urdf_file_, SIGNAL(pathChanged(QString)), this, SLOT(onUrdfPathChanged(QString)));
  left_layout->addWidget(urdf_file_);

  // Load settings box ---------------------------------------------
  QHBoxLayout* load_files_layout = new QHBoxLayout();

  progress_bar_ = new QProgressBar(this);
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  progress_bar_->hide();
  load_files_layout->addWidget(progress_bar_);

  btn_load_ = new QPushButton("&Load Files", this);
  btn_load_->setMinimumWidth(180);
  btn_load_->setMinimumHeight(40);
  btn_load_->hide();
  load_files_layout->addWidget(btn_load_);
  load_files_layout->setAlignment(btn_load_, Qt::AlignRight);
  connect(btn_load_, SIGNAL(clicked()), this, SLOT(loadFilesClick()));

  // Next step instructions
  next_label_ = new QLabel(this);
  QFont next_label_font(QFont().defaultFamily(), 11, QFont::Bold);
  next_label_->setFont(next_label_font);
  // next_label_->setWordWrap(true);
  next_label_->setText("Success! Use the left navigation pane to continue.");
  //  next_label_->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  next_label_->hide();  // only show once the files have been loaded.

  // Final Layout Setup ---------------------------------------------
  // Alignment
  layout->setAlignment(Qt::AlignTop);
  hlayout->setAlignment(Qt::AlignTop);
  left_layout->setAlignment(Qt::AlignTop);
  right_layout->setAlignment(Qt::AlignTop);

  // Stretch
  left_layout->setSpacing(20);

  // Attach Layouts
  hlayout->addLayout(left_layout);
  hlayout->addLayout(right_layout);
  layout->addLayout(hlayout);

  // Vertical Spacer
  QWidget* vspacer = new QWidget(this);
  vspacer->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  layout->addWidget(vspacer);

  // Attach bottom layout
  layout->addWidget(next_label_);
  layout->setAlignment(next_label_, Qt::AlignRight);
  layout->addLayout(load_files_layout);

  this->setLayout(layout);
  //  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

// ******************************************************************************************
// Destructor
// ******************************************************************************************
StartScreenWidget::~StartScreenWidget()
{
  delete right_image_;  // does not have a parent passed to it
  delete logo_image_;
}

// ******************************************************************************************
// Show options for creating a new configuration package
// ******************************************************************************************
void StartScreenWidget::showNewOptions()
{
//  // Do GUI stuff
//  select_mode_->btn_exist_->setChecked(false);
//  select_mode_->btn_new_->setChecked(true);
//  urdf_file_->show();
//  stack_path_->hide();
//  btn_load_->show();

//  // Remember choice
//  create_new_package_ = true;

  /*
   * TODO: create new action package
   */
  Q_EMIT readyToProgress();
}

// ******************************************************************************************
// Show options for editing an existing configuration package
// ******************************************************************************************
void StartScreenWidget::showExistingOptions()
{
  // Do GUI stuff
  select_mode_->btn_exist_->setChecked(true);
  select_mode_->btn_new_->setChecked(false);
  urdf_file_->hide();
  stack_path_->show();
  btn_load_->show();

  // Remember choice
  create_new_package_ = false;
}

// ******************************************************************************************
// Load files to parameter server - CLICK
// ******************************************************************************************
void StartScreenWidget::loadFilesClick()
{
  // Disable start screen GUI components from being changed
  urdf_file_->setDisabled(true);
  // srdf_file_->setDisabled(true);
  stack_path_->setDisabled(true);
  select_mode_->setDisabled(true);
  btn_load_->setDisabled(true);
  progress_bar_->show();

  bool result;

  // Decide if this is a new config package, or loading an old one
  if (create_new_package_)
  {
    result = loadNewFiles();
  }
  else
  {
    result = loadExistingFiles();
  }

  // Check if there was a failure loading files
  if (!result)
  {
    // Renable components
    urdf_file_->setDisabled(false);
    // srdf_file_->setDisabled(false);
    stack_path_->setDisabled(false);
    select_mode_->setDisabled(false);
    btn_load_->setDisabled(false);
    progress_bar_->hide();
  }
  else
  {
    // Hide the logo image so that other screens can resize the rviz thing properly
    right_image_label_->hide();
    logo_image_label_->hide();
  }
}

void StartScreenWidget::onPackagePathChanged(const QString& path)
{
//  if (!loadPackageSettings(false))
//    return;
//  // set xacro args from loaded settings
//  stack_path_->setArgs(QString::fromStdString(config_data_->xacro_args_));
}

void StartScreenWidget::onUrdfPathChanged(const QString& path)
{
//  urdf_file_->setArgsEnabled(rdf_loader::RDFLoader::isXacroFile(path.toStdString()));
}

bool StartScreenWidget::loadPackageSettings(bool show_warnings)
{
//  // Get the package path
//  std::string package_path_input = stack_path_->getPath();
//  // Check that input is provided
//  if (package_path_input.empty())
//  {
//    if (show_warnings)
//      QMessageBox::warning(this, "Error Loading Files", "Please specify a configuration package path to load.");
//    return false;
//  }

//  // check that the folder exists
//  if (!config_data_->setPackagePath(package_path_input))
//  {
//    if (show_warnings)
//      QMessageBox::critical(this, "Error Loading Files", "The specified path is not a directory or is not accessable");
//    return false;
//  }

//  std::string setup_assistant_path;

//  // Check if the old package is a setup assistant package. If it is not, quit
//  if (!config_data_->getSetupAssistantYAMLPath(setup_assistant_path))
//  {
//    if (show_warnings)
//      QMessageBox::warning(
//          this, "Incorrect Directory/Package",
//          QString("The chosen package location exists but was not created using TeMoto Action Assistant. "
//                  "If this is a mistake, provide the missing file: ")
//              .append(setup_assistant_path.c_str()));
//    return false;
//  }

//  // Get setup assistant data
//  if (!config_data_->inputSetupAssistantYAML(setup_assistant_path))
//  {
//    if (show_warnings)
//      QMessageBox::warning(this, "Setup Assistant File Error",
//                           QString("Unable to correctly parse the setup assistant configuration file: ")
//                               .append(setup_assistant_path.c_str()));
//    return false;
//  }
  return true;
}

// ******************************************************************************************
// Load exisiting package files
// ******************************************************************************************
bool StartScreenWidget::loadExistingFiles()
{
//  // Progress Indicator
//  progress_bar_->setValue(10);
//  QApplication::processEvents();

//  if (!loadPackageSettings(true))
//    return false;

//  // Progress Indicator
//  progress_bar_->setValue(30);
//  QApplication::processEvents();

//  // Get the URDF path using the loaded .setup_assistant data and check it
//  if (!createFullURDFPath())
//    return false;  // error occured

//  // use xacro args from GUI
//  config_data_->xacro_args_ = stack_path_->getArgs().toStdString();

//  // Load the URDF
//  if (!loadURDFFile(config_data_->urdf_path_, config_data_->xacro_args_))
//    return false;  // error occured

//  // Get the SRDF path using the loaded .setup_assistant data and check it
//  if (!createFullSRDFPath(config_data_->config_pkg_path_))
//    return false;  // error occured

//  // Progress Indicator
//  progress_bar_->setValue(50);
//  QApplication::processEvents();

//  // Load the SRDF
//  if (!loadSRDFFile(config_data_->srdf_path_))
//    return false;  // error occured

//  // Progress Indicator
//  progress_bar_->setValue(60);
//  QApplication::processEvents();

//  // Load the allowed collision matrix
//  config_data_->loadAllowedCollisionMatrix();

//  // Load kinematics yaml file if available --------------------------------------------------
//  fs::path kinematics_yaml_path = config_data_->config_pkg_path_;
//  kinematics_yaml_path /= "config/kinematics.yaml";

//  if (!config_data_->inputKinematicsYAML(kinematics_yaml_path.make_preferred().native().c_str()))
//  {
//    QMessageBox::warning(this, "No Kinematic YAML File",
//                         QString("Failed to parse kinematics yaml file. This file is not critical but any previous "
//                                 "kinematic solver settings have been lost. To re-populate this file edit each "
//                                 "existing planning group and choose a solver, then save each change. \n\nFile error "
//                                 "at location ")
//                             .append(kinematics_yaml_path.make_preferred().native().c_str()));
//  }

//  // DONE LOADING --------------------------------------------------------------------------

//  // Call a function that enables navigation
//  Q_EMIT readyToProgress();

//  // Progress Indicator
//  progress_bar_->setValue(70);
//  QApplication::processEvents();

//  // Progress Indicator
//  progress_bar_->setValue(100);
//  QApplication::processEvents();

//  next_label_->show();  // only show once the files have been loaded

  ROS_INFO("Loading Setup Assistant Complete");
  return true;  // success!
}

// ******************************************************************************************
// Load chosen files for creating new package
// ******************************************************************************************
bool StartScreenWidget::loadNewFiles()
{
//  // Get URDF file path
//  config_data_->urdf_path_ = urdf_file_->getPath();

//  // Check that box is filled out
//  if (config_data_->urdf_path_.empty())
//  {
//    QMessageBox::warning(this, "Error Loading Files", "No robot model file specefied");
//    return false;
//  }

//  // Check that this file exits
//  if (!fs::is_regular_file(config_data_->urdf_path_))
//  {
//    QMessageBox::warning(this, "Error Loading Files",
//                         QString("Unable to locate the URDF file: ").append(config_data_->urdf_path_.c_str()));
//    return false;
//  }

//  // Attempt to get the ROS package from the path
//  if (!extractPackageNameFromPath())
//  {
//    return false;  // An error occurred
//  }

//  // Progress Indicator
//  progress_bar_->setValue(20);
//  QApplication::processEvents();

//  // use xacro args from GUI
//  config_data_->xacro_args_ = urdf_file_->getArgs().toStdString();

//  // Load the URDF to the parameter server and check that it is correct format
//  if (!loadURDFFile(config_data_->urdf_path_, config_data_->xacro_args_))
//    return false;  // error occurred

//  // Progress Indicator
//  progress_bar_->setValue(50);
//  QApplication::processEvents();

//  // Create blank SRDF file
//  const std::string blank_srdf = "<?xml version='1.0'?><robot name='" + config_data_->urdf_model_->getName() + "'></"
//                                                                                                               "robot>";

//  // Load a blank SRDF file to the parameter server
//  if (!setSRDFFile(blank_srdf))
//  {
//    QMessageBox::warning(this, "Error Loading Files", "Failure loading blank SRDF file.");
//    return false;
//  }

//  // Progress Indicator
//  progress_bar_->setValue(60);
//  QApplication::processEvents();

  // DONE LOADING --------------------------------------------------------------------------

  // Call a function that enables navigation
  Q_EMIT readyToProgress();

//  // Progress Indicator
//  progress_bar_->setValue(70);
//  QApplication::processEvents();

//  // Progress Indicator
//  progress_bar_->setValue(100);
//  QApplication::processEvents();

  next_label_->show();  // only show once the files have been loaded

  ROS_INFO("Loading Setup Assistant Complete");
  return true;  // success!
}


// ******************************************************************************************
// ******************************************************************************************
// Class for selecting which mode
// ******************************************************************************************
// ******************************************************************************************

// ******************************************************************************************
// Create the widget
// ******************************************************************************************
SelectModeWidget::SelectModeWidget(QWidget* parent) : QFrame(parent)
{
  // Set frame graphics
  setFrameShape(QFrame::StyledPanel);
  setFrameShadow(QFrame::Raised);
  setLineWidth(1);
  setMidLineWidth(0);

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Horizontal layout splitter
  QHBoxLayout* hlayout = new QHBoxLayout();

  // Widget Title
  QLabel* widget_title = new QLabel(this);
  widget_title->setText("Choose mode:");
  QFont widget_title_font(QFont().defaultFamily(), 12, QFont::Bold);
  widget_title->setFont(widget_title_font);
  layout->addWidget(widget_title);
  layout->setAlignment(widget_title, Qt::AlignTop);

  // Widget Instructions
  //QTextEdit* widget_instructions = new QTextEdit(this);
  //widget_instructions->setText("");
  //widget_instructions->setWordWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
  // widget_instructions->setMinimumWidth(1);
  //layout->addWidget(widget_instructions);
  //layout->setAlignment(widget_instructions, Qt::AlignTop);

  // New Button
  btn_new_ = new QPushButton(this);
  btn_new_->setText("Create &New TeMoto\nAction Package");
  hlayout->addWidget(btn_new_);

  // Exist Button
  btn_exist_ = new QPushButton(this);
  btn_exist_->setText("&Edit Existing TeMoto\nAction Package");
  btn_exist_->setCheckable(true);
  btn_exist_->setEnabled(false); // TODO: Create package modification functionality
  hlayout->addWidget(btn_exist_);

  // Add horizontal layer to verticle layer
  layout->addLayout(hlayout);
  setLayout(layout);
  btn_new_->setCheckable(true);
}

}  // namespace
