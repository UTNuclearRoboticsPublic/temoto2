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

// SA
#include "sf_editor_widget.h"
// Qt
#include <QFormLayout>
#include <QMessageBox>
#include <QTreeWidgetItem>

namespace temoto_action_assistant
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
SFEditorWidget::SFEditorWidget(QWidget* parent, temoto_action_assistant::MoveItConfigDataPtr config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();
  QHBoxLayout* layout_e_t = new QHBoxLayout();

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   *                            Create content for the top layer
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  // Add header
  HeaderWidget* header = new HeaderWidget("Semantic Frame Editor", "Edit semantic frames", this);
  layout->addWidget(header);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   *                           Create content for the edit screen
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  edit_screen_content_ = new QStackedLayout();
  layout_e_t->addLayout(edit_screen_content_);

  /*
   * Subject editor page
   */
  sew_ = new SubjectEditWidget(parent);
  edit_screen_content_->addWidget(sew_);

  /*
   * Subjects editor page
   */
  // Dummy widget that is going to contain the subjects_editor_layout
  QWidget* subjects_editor_widget = new QWidget();
  QVBoxLayout* subjects_editor_layout = new QVBoxLayout();
  subjects_editor_widget->setLayout(subjects_editor_layout);

  /* DO STUFF */

  edit_screen_content_->addWidget(subjects_editor_widget);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   *                           Create content for the interfaces tree
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  interfaces_tree_widget_ = createContentsWidget();
  layout_e_t->addWidget(interfaces_tree_widget_);

  /*
   * Add the edtior/tree layout to the top layout
   */
  layout->addLayout(layout_e_t);
  layout->setAlignment(Qt::AlignTop);
  this->setLayout(layout);

  /* TEST */

  // Create an action descriptor
  Subject subject_0, subject_1;
  subject_0.type_ = Subject::WHAT;
  subject_1.type_ = Subject::WHERE;
  subject_0.words_.push_back("dog");
  subject_1.words_.push_back("table");

  DataInstance data_0_sub_0;
  data_0_sub_0.type_ = DataInstance::TOPIC;
  subject_0.data_.push_back(data_0_sub_0);

  DataInstance data_1_sub_0;
  data_1_sub_0.type_ = DataInstance::POINTER;
  subject_0.data_.push_back(data_1_sub_0);

  Interface interface_0;
  interface_0.id_ = 0;
  interface_0.input_subjects_.push_back(subject_0);
  interface_0.input_subjects_.push_back(subject_1);

  action_descriptor_.interfaces_.push_back(interface_0);
  // Action descriptor end

  populateInterfacesTree();
  interfaces_tree_->expandAll();
}

// ******************************************************************************************
// Create the main tree view widget
// ******************************************************************************************
QWidget* SFEditorWidget::createContentsWidget()
{
  // Main widget
  QWidget* content_widget = new QWidget(this);

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Tree Box ----------------------------------------------------------------------

  interfaces_tree_ = new QTreeWidget(this);
  interfaces_tree_->setHeaderLabel("Interfaces");
  connect(interfaces_tree_, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(editSelected()));
  //connect(interfaces_tree_, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(previewSelected()));
  layout->addWidget(interfaces_tree_);

  // Bottom Controls -------------------------------------------------------------

  QHBoxLayout* controls_layout = new QHBoxLayout();

  // Expand/Contract controls
  QLabel* expand_controls = new QLabel(this);
  expand_controls->setText("<a href='expand'>Expand All</a> <a href='contract'>Collapse All</a>");
  connect(expand_controls, SIGNAL(linkActivated(const QString)), this, SLOT(alterTree(const QString)));
  controls_layout->addWidget(expand_controls);

  // Spacer
  QWidget* spacer = new QWidget(this);
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  controls_layout->addWidget(spacer);

  // Delete Selected Button
  btn_delete_ = new QPushButton("&Delete Selected", this);
  btn_delete_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_delete_->setMaximumWidth(300);
  connect(btn_delete_, SIGNAL(clicked()), this, SLOT(deleteGroup()));
  controls_layout->addWidget(btn_delete_);
  controls_layout->setAlignment(btn_delete_, Qt::AlignRight);

  //  Edit Selected Button
  btn_edit_ = new QPushButton("&Edit Selected", this);
  btn_edit_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_edit_->setMaximumWidth(300);
  btn_edit_->hide();  // show once we know if there are existing groups
  connect(btn_edit_, SIGNAL(clicked()), this, SLOT(editSelected()));
  controls_layout->addWidget(btn_edit_);
  controls_layout->setAlignment(btn_edit_, Qt::AlignRight);

  // Add Group Button
  QPushButton* btn_add = new QPushButton("&Add Interface", this);
  btn_add->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_add->setMaximumWidth(300);
  connect(btn_add, SIGNAL(clicked()), this, SLOT(addInterface()));
  controls_layout->addWidget(btn_add);
  controls_layout->setAlignment(btn_add, Qt::AlignRight);

  // Add Controls to layout
  layout->addLayout(controls_layout);

  // Set layout
  content_widget->setLayout(layout);

  return content_widget;
}

// ******************************************************************************************
// Create a new interface
// ******************************************************************************************
void SFEditorWidget::addInterface()
{
//  adding_new_interface_ = true;

//  // Load the data
//  loadGroupScreen(NULL);  // NULL indicates this is a new group, not an existing one

//  // Switch to screen
//  changeScreen(5);
}

// ******************************************************************************************
// Edit whatever element is selected in the tree view
// ******************************************************************************************
void SFEditorWidget::editSelected()
{
  active_tree_item_ = interfaces_tree_->currentItem();

  //std::cout << *(item->data(0, Qt::UserRole).value<InterfaceTreeData>().name_) << std::endl;

  // Check that something was actually selected
  if (active_tree_item_ == NULL)
  {
    return;
  }

  // Get the user custom properties of the currently selected row
  active_tree_element_ = active_tree_item_->data(0, Qt::UserRole).value<InterfaceTreeData>();

  switch(active_tree_element_.type_)
  {
    case (InterfaceTreeData::INTERFACE):
    {
      std::cout << "interface" << std::endl;
      interfaces_tree_->clear();
    }
    break;

    case (InterfaceTreeData::INPUT):
    {
      std::cout << "input" << std::endl;
      edit_screen_content_->setCurrentIndex(1);
    }
    break;

    case (InterfaceTreeData::OUTPUT):
    {
      std::cout << "output" << std::endl;
      edit_screen_content_->setCurrentIndex(1);
    }
    break;

    case (InterfaceTreeData::SUBJECT):
    {
      // Set up the subject editor
      sew_->focusGiven(active_tree_item_);

      // Show subject editor
      edit_screen_content_->setCurrentIndex(0);
    }
    break;

    default:
    {
      QMessageBox::critical(this, "Error Loading", "An internal error has occured while loading.");
    }
  }
}

// ******************************************************************************************
//
// ******************************************************************************************
void SFEditorWidget::focusGiven()
{

}

// ******************************************************************************************
//
// ******************************************************************************************
void SFEditorWidget::selectionUpdated()
{

}

// ******************************************************************************************
//
// ******************************************************************************************
void SFEditorWidget::populateInterfacesTree()
{
  // Define the tree fonts. TODO: Move to header
  const QFont top_level_font(QFont().defaultFamily(), 11, QFont::Bold);
  const QFont type_font(QFont().defaultFamily(), 11, QFont::Normal, QFont::StyleItalic);

  // Loop over interfaces
  for (Interface& interface_sf : action_descriptor_.interfaces_)
  {
    std::string interface_id_str =  std::string("Interface ") + std::to_string(interface_sf.id_);
    QString interface_id_qstr = QString::fromStdString(interface_id_str);

    /*
     * Create interface element for the tree
     */
    QTreeWidgetItem* interface_item = new QTreeWidgetItem(interfaces_tree_);
    interface_item->setText(0, interface_id_qstr);
    interface_item->setFont(0, top_level_font);
    InterfaceTreeData interface_tdata = InterfaceTreeData(
                                  InterfaceTreeData::INTERFACE,
                                  boost::any_cast<Interface*>(&interface_sf));
    interface_item->setData(0, Qt::UserRole, QVariant::fromValue(interface_tdata));
    interfaces_tree_->addTopLevelItem(interface_item);

    /*
     * Create input element for the tree
     */

    // First check if the interface contains input subjects
    if (interface_sf.input_subjects_.empty())
    {
      // TODO: throw an errror
      return;
    }

    QTreeWidgetItem* input_subjects_item = new QTreeWidgetItem(interface_item);
    input_subjects_item->setText(0, "Input");
    input_subjects_item->setFont(0, type_font);
    InterfaceTreeData input_subjects_tdata = InterfaceTreeData(
                                  InterfaceTreeData::INPUT,
                                  boost::any_cast<Subjects*>(&interface_sf.input_subjects_));
    input_subjects_item->setData(0, Qt::UserRole, QVariant::fromValue(input_subjects_tdata));
    interface_item->addChild(input_subjects_item);

    /*
     * Add the subjects of the interface
     */
    for (Subject& subject_sf : interface_sf.input_subjects_)
    {
      QTreeWidgetItem* subject_item = new QTreeWidgetItem(input_subjects_item);
      QString text = QString::fromStdString(subject_sf.getTypeStr() + ": " + subject_sf.words_[0]);
      subject_item->setText(0, text);
      subject_item->setFont(0, type_font);
      InterfaceTreeData subject_tdata = InterfaceTreeData(
                                    InterfaceTreeData::SUBJECT,
                                    boost::any_cast<Subject*>(&subject_sf));
      subject_item->setData(0, Qt::UserRole, QVariant::fromValue(subject_tdata));
      input_subjects_item->addChild(subject_item);

      /*
       * Add the data instance elements
       */
      for (DataInstance& data_instance_sf : subject_sf.data_)
      {
        QTreeWidgetItem* data_instance_item = new QTreeWidgetItem(subject_item);
        QString text_data_instance = QString::fromStdString(data_instance_sf.getTypeStr());
        data_instance_item->setText(0, text_data_instance);
        data_instance_item->setFont(0, type_font);
        InterfaceTreeData data_instance_tdata = InterfaceTreeData(
                                      InterfaceTreeData::DATA,
                                      boost::any_cast<DataInstance*>(&data_instance_sf));
        data_instance_item->setData(0, Qt::UserRole, QVariant::fromValue(data_instance_tdata));
        subject_item->addChild(data_instance_item);
      }

    }

  }
}

} // temoto_action_assistant namespace