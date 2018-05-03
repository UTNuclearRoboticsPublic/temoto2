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
  QVBoxLayout* layout_editor = new QVBoxLayout();
  layout_e_t->addLayout(layout_editor);

  // Top Header Area ------------------------------------------------

  HeaderWidget* header = new HeaderWidget("Semantic Frame Editor", "Edit semantic frames", this);
  layout->addWidget(header);
  layout->addLayout(layout_e_t);

  // Simple form -------------------------------------------
  QFormLayout* form_layout = new QFormLayout();
  form_layout->setContentsMargins(0, 15, 0, 15);

  // Lexical unit input
  lexical_unit_field_ = new QLineEdit(this);
  lexical_unit_field_->setMaximumWidth(400);
  form_layout->addRow("Lexical Unit:", lexical_unit_field_);
  layout_editor->addLayout(form_layout);

  // Create the interfaces tree widget
  interfaces_tree_widget_ = createContentsWidget();
  layout_e_t->addWidget(interfaces_tree_widget_);

  // Finish Layout --------------------------------------------------
  layout->setAlignment(Qt::AlignTop);
  this->setLayout(layout);

  /* TEST */

  // Create an action descriptor
  Subject subject_0, subject_1;
  subject_0.type_ = Subject::WHAT;
  subject_1.type_ = Subject::WHERE;
  subject_0.words_.push_back("dog");
  subject_1.words_.push_back("table");

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
  QTreeWidgetItem* item = interfaces_tree_->currentItem();

  //std::cout << *(item->data(0, Qt::UserRole).value<InterfaceTreeData>().name_) << std::endl;

  // Check that something was actually selected
  if (item == NULL)
  {
    return;
  }

  //adding_new_group_ = false;

  // Get the user custom properties of the currently selected row
  InterfaceTreeData tree_elem = item->data(0, Qt::UserRole).value<InterfaceTreeData>();

  switch(tree_elem.type_)
  {
    case (ElementType::INTERFACE):
    {
      std::cout << "interface" << std::endl;
    }
    break;

    case (ElementType::INPUT):
    {
      std::cout << "input" << std::endl;
    }
    break;

    case (ElementType::OUTPUT):
    {
      std::cout << "output" << std::endl;
    }
    break;

    case (ElementType::SUBJECT):
    {
      Subject* subject = boost::any_cast<Subject*>(tree_elem.payload_);
      std::cout << subject->words_[0] << std::endl;

      // Load subjects editor
    }
    break;

    default:
    {
      QMessageBox::critical(this, "Error Loading", "An internal error has occured while loading.");
    }
  }

//  if (interface.type_ == JOINT)
//  {
////    // Load the data
////    loadJointsScreen(interface.group_);

////    // Switch to screen
////    changeScreen(1);  // 1 is index of joints
//  }
//  else if (interface.type_ == LINK)
//  {
////    // Load the data
////    loadLinksScreen(interface.group_);

////    // Switch to screen
////    changeScreen(2);
//  }
//  else if (interface.type_ == CHAIN)
//  {
////    // Load the data
////    loadChainScreen(interface.group_);

////    // Switch to screen
////    changeScreen(3);
//  }
//  else if (interface.type_ == SUBGROUP)
//  {
////    // Load the data
////    loadSubgroupsScreen(interface.group_);

////    // Switch to screen
////    changeScreen(4);
//  }
//  else if (interface.type_ == GROUP)
//  {
////    // Load the data
////    loadGroupScreen(interface.group_);

////    // Switch to screen
////    changeScreen(5);
//  }
//  else
//  {
//    QMessageBox::critical(this, "Error Loading", "An internal error has occured while loading.");
//  }
}

// ******************************************************************************************
// Load the popup screen with correct data for groups
// ******************************************************************************************
//void PlanningGroupsWidget::loadGroupScreen(srdf::Model::Group* this_group)
//{
//  // Load the avail kin solvers. This function only runs once
//  group_edit_widget_->loadKinematicPlannersComboBox();

//  if (this_group == NULL)  // this is a new screen
//  {
//    current_edit_group_.clear();  // provide a blank group name
//    group_edit_widget_->title_->setText("Create New Planning Group");
//    group_edit_widget_->btn_delete_->hide();
//    group_edit_widget_->new_buttons_widget_->show();  // helps user choose next step
//    group_edit_widget_->btn_save_->hide();            // this is only for edit mode
//  }
//  else  // load the group name into the widget
//  {
//    current_edit_group_ = this_group->name_;
//    group_edit_widget_->title_->setText(
//        QString("Edit Planning Group '").append(current_edit_group_.c_str()).append("'"));
//    group_edit_widget_->btn_delete_->show();
//    group_edit_widget_->new_buttons_widget_->hide();  // not necessary for existing groups
//    group_edit_widget_->btn_save_->show();            // this is only for edit mode
//  }

//  // Set the data in the edit box
//  group_edit_widget_->setSelected(current_edit_group_);

//  // Remember what is currently being edited so we can later save changes
//  current_edit_element_ = GROUP;
//}

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
                                  ElementType::INTERFACE,
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
                                  ElementType::INPUT,
                                  boost::any_cast<Subjects*>(&interface_sf.input_subjects_));
    input_subjects_item->setData(0, Qt::UserRole, QVariant::fromValue(input_subjects_tdata));
    interface_item->addChild(input_subjects_item);

    /*
     * Add the subjects of the interface
     */
    for (Subject& subject_sf : interface_sf.input_subjects_)
    {
      QTreeWidgetItem* subject_item = new QTreeWidgetItem(input_subjects_item);
      subject_item->setText(0, QString::fromStdString(subject_sf.getTypeStr()));
      subject_item->setFont(0, type_font);
      InterfaceTreeData subject_tdata = InterfaceTreeData(
                                    ElementType::SUBJECT,
                                    boost::any_cast<Subject*>(&subject_sf));
      subject_item->setData(0, Qt::UserRole, QVariant::fromValue(subject_tdata));
      input_subjects_item->addChild(subject_item);
    }

  }


//  std::string* test_name = new std::string("test_interface_0");
//  QTreeWidgetItem* interface = new QTreeWidgetItem(interfaces_tree_);
//  interface->setText(0, "Interface 0");
//  interface->setFont(0, top_level_font);
//  interface->setData(0, Qt::UserRole, QVariant::fromValue(InterfaceTreeData(test_name, ElementType::INTERFACE)));
//  interfaces_tree_->addTopLevelItem(interface);

//  std::string* test_name_1 = new std::string("test_interface_1");
//  QTreeWidgetItem* interface_1 = new QTreeWidgetItem(interfaces_tree_);
//  interface_1->setText(0, "Interface 1");
//  interface_1->setFont(0, top_level_font);
//  interface_1->setData(0, Qt::UserRole, QVariant::fromValue(InterfaceTreeData(test_name_1, ElementType::INTERFACE)));
//  interfaces_tree_->addTopLevelItem(interface_1);
}

}  // namespace

// ******************************************************************************************
// ******************************************************************************************
// CLASS
// ******************************************************************************************
// ******************************************************************************************

InterfaceTreeData::InterfaceTreeData(const temoto_action_assistant::ElementType type, boost::any payload)
  : type_(type),
    payload_(payload)
{}
