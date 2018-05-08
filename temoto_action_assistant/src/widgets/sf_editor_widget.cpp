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
#include <iostream>

namespace temoto_action_assistant
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
SFEditorWidget::SFEditorWidget(QWidget* parent, ActionDescriptorPtr action_descriptor)
  : SetupScreenWidget(parent),
    action_descriptor_(action_descriptor),
    top_level_font_(QFont(QFont().defaultFamily(), 11, QFont::Bold)),
    io_font_(QFont(QFont().defaultFamily(), 11, QFont::StyleNormal)),
    type_font_(QFont(QFont().defaultFamily(), 11, QFont::Normal, QFont::StyleItalic))

{
  // TODO: add a description element to the widget

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();
  QHBoxLayout* layout_e_t = new QHBoxLayout();

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   *                                Create content for the top layer
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  // Add header
  HeaderWidget* header = new HeaderWidget("Semantic Frame Editor", "Edit semantic frames", this);
  layout->addWidget(header);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   *                               Create content for the edit screen
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  QVBoxLayout* edit_screen_top_layout = new QVBoxLayout();
  edit_screen_content_ = new QStackedLayout();
  layout_e_t->addLayout(edit_screen_top_layout);
  edit_screen_top_layout->addLayout(edit_screen_content_);

  /*
   * Dummy editor page
   */
  QWidget* dummy_widget = new QWidget();
  edit_screen_content_->addWidget(dummy_widget);

  /*
   * Interface editor page
   */
  iew_ = new InterfaceEditWidget(parent);
  edit_screen_content_->addWidget(iew_);

  /*
   * Subjects editor page: TODO
   */
  QWidget* subjects_editor_widget = new QWidget();
  QVBoxLayout* subjects_editor_layout = new QVBoxLayout();
  subjects_editor_widget->setLayout(subjects_editor_layout);

  // DO STUFF //

  edit_screen_content_->addWidget(subjects_editor_widget);

  /*
   * Subject editor page
   */
  sew_ = new SubjectEditWidget(parent);
  edit_screen_content_->addWidget(sew_);

  /*
   * Data editor page
   */
  diew_ = new DataInstanceEditWidget(parent);
  edit_screen_content_->addWidget(diew_);

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

  // Layout for "add/remove selected" buttons
  QHBoxLayout* add_delete_btn_layout = new QHBoxLayout(this);

  // Add to Selected Button
  btn_add_ = new QPushButton("&Add to Selected", this);
  btn_add_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  //btn_add_->setMaximumWidth(400);
  btn_add_->setDisabled(true);
  connect(btn_add_, SIGNAL(clicked()), this, SLOT(addToActiveTreeElement()));
  add_delete_btn_layout->addWidget(btn_add_);
  add_delete_btn_layout->setAlignment(btn_add_, Qt::AlignCenter);

  // Delete Selected Button
  btn_delete_ = new QPushButton("&Delete Selected", this);
  btn_delete_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  //btn_delete_->setMaximumWidth(400);
  btn_delete_->setDisabled(true);
  connect(btn_delete_, SIGNAL(clicked()), this, SLOT(removeActiveTreeElement()));
  add_delete_btn_layout->addWidget(btn_delete_);
  add_delete_btn_layout->setAlignment(btn_delete_, Qt::AlignCenter);

  layout->addLayout(add_delete_btn_layout);

  // Tree Box ----------------------------------------------------------------------

  interfaces_tree_ = new QTreeWidget(this);
  interfaces_tree_->setHeaderLabel("Interfaces");
  connect(interfaces_tree_, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(editSelected()));
  //connect(interfaces_tree_, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(previewSelected()));
  layout->addWidget(interfaces_tree_);

  // Bottom Controls -------------------------------------------------------------

  QHBoxLayout* controls_layout = new QHBoxLayout();

  // Expand/Contract controls
  QLabel* expand_controls = new QLabel(this);
  expand_controls->setText("<a href='expand'>Expand All</a> <a href='contract'>Collapse All</a>");
  //connect(expand_controls, SIGNAL(linkActivated(const QString)), this, SLOT(alterTree(const QString)));
  controls_layout->addWidget(expand_controls);

  // Spacer
  QWidget* spacer = new QWidget(this);
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  controls_layout->addWidget(spacer);

  // Add interface button
  btn_add_interface_ = new QPushButton("&Add Interface", this);
  btn_add_interface_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  connect(btn_add_interface_, SIGNAL(clicked()), this, SLOT(addInterface()));
  controls_layout->addWidget(btn_add_interface_);
  controls_layout->setAlignment(btn_add_interface_, Qt::AlignRight);

  // Add Controls to layout
  layout->addLayout(controls_layout);

  // Set layout
  content_widget->setLayout(layout);

  return content_widget;
}

// ******************************************************************************************
// Edit whatever element is selected in the tree view
// ******************************************************************************************
void SFEditorWidget::editSelected()
{
  active_tree_item_ = interfaces_tree_->currentItem();

  // Check that something was actually selected
  if (active_tree_item_ == NULL)
  {
    return;
  }

  // Enable the add/delete buttons
  btn_add_->setDisabled(false);
  btn_delete_->setDisabled(false);

  // Get the user custom properties of the currently selected row
  active_tree_element_ = active_tree_item_->data(0, Qt::UserRole).value<InterfaceTreeData>();

  switch(active_tree_element_.type_)
  {
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Edit the interface
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::INTERFACE):
    {
      iew_->focusGiven(active_tree_item_);
      edit_screen_content_->setCurrentIndex(1);
    }
    break;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Edit the input of the interface
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::INPUT):
    {
      // Disable the delete button since input is a required component of the interface
      btn_delete_->setDisabled(true);
      edit_screen_content_->setCurrentIndex(2);
    }
    break;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Edit the output of the interface
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::OUTPUT):
    {
      edit_screen_content_->setCurrentIndex(2);
    }
    break;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Edit subject
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::SUBJECT):
    {
      // Set up the subject editor
      sew_->focusGiven(active_tree_item_);

      // Show subject editor
      edit_screen_content_->setCurrentIndex(3);
    }
    break;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Edit data
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::DATA):
    {
      // Set up the data instance editor
      diew_->focusGiven(active_tree_item_);

      // Disable the add button since there is nothing to add
      btn_add_->setDisabled(true);

      // Show the data instance editor
      edit_screen_content_->setCurrentIndex(4);
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
void SFEditorWidget::populateInterfacesTree()
{
  // Loop over interfaces
  uint32_t index = 0; // TODO: Remove this indexing hack - embed it into the loop header
  for (Interface& interface_sf : action_descriptor_->interfaces_)
  {
    std::string interface_id_str =  std::string("Interface ") + std::to_string(index) +
                                    ": " + interface_sf.getTypeStr();
    QString interface_id_qstr = QString::fromStdString(interface_id_str);

    // Set the index
    interface_sf.id_ = index;

    /*
     * Create interface item for the tree
     */
    QTreeWidgetItem* interface_item = new QTreeWidgetItem(interfaces_tree_);
    interface_item->setText(0, interface_id_qstr);
    interface_item->setFont(0, top_level_font_);
    InterfaceTreeData interface_tdata = InterfaceTreeData(
                                  InterfaceTreeData::INTERFACE,
                                  boost::any_cast<Interface*>(&interface_sf));
    interface_item->setData(0, Qt::UserRole, QVariant::fromValue(interface_tdata));
    interfaces_tree_->addTopLevelItem(interface_item);

    /*
     * Create input item for the interface item
     */
    if (interface_sf.input_subjects_.empty())
    {
      // Interface is required to have input subjects
      // TODO: throw an errror
      return;
    }

    // TODO: Remove this indexing hack - embed it into the loop header
    index++;

    QTreeWidgetItem* input_subjects_item = new QTreeWidgetItem(interface_item);
    input_subjects_item->setText(0, "Input");
    input_subjects_item->setFont(0, io_font_);
    InterfaceTreeData input_subjects_tdata = InterfaceTreeData(
                                  InterfaceTreeData::INPUT,
                                  boost::any_cast<Subjects*>(&interface_sf.input_subjects_));
    input_subjects_item->setData(0, Qt::UserRole, QVariant::fromValue(input_subjects_tdata));
    interface_item->addChild(input_subjects_item);

    // Add the subjects of the interface
    populateSubjects(input_subjects_item, interface_sf.input_subjects_);

    /*
     * Create output item for the interface item
     */
    if (interface_sf.output_subjects_.empty())
    {
      // Interface is not required to have output subjects
      continue;
    }

    QTreeWidgetItem* output_subjects_item = new QTreeWidgetItem(interface_item);
    output_subjects_item->setText(0, "Output");
    output_subjects_item->setFont(0, io_font_);
    InterfaceTreeData output_subjects_tdata = InterfaceTreeData(
                                  InterfaceTreeData::OUTPUT,
                                  boost::any_cast<Subjects*>(&interface_sf.output_subjects_));
    output_subjects_item->setData(0, Qt::UserRole, QVariant::fromValue(output_subjects_tdata));
    interface_item->addChild(output_subjects_item);

    // Add the subjects of the interface
    populateSubjects(output_subjects_item, interface_sf.output_subjects_);
  }
}

// ******************************************************************************************
//
// ******************************************************************************************
void SFEditorWidget::populateSubjects(QTreeWidgetItem* parent_item, Subjects& subjects)
{
  for (Subject& subject_sf : subjects)
  {
    QTreeWidgetItem* subject_item = new QTreeWidgetItem(parent_item);
    QString text = QString::fromStdString(subject_sf.getTypeStr() + ": " + subject_sf.words_[0]);
    subject_item->setText(0, text);
    subject_item->setFont(0, type_font_);
    InterfaceTreeData subject_tdata = InterfaceTreeData(
                                  InterfaceTreeData::SUBJECT,
                                  boost::any_cast<Subject*>(&subject_sf));
    subject_item->setData(0, Qt::UserRole, QVariant::fromValue(subject_tdata));
    parent_item->addChild(subject_item);

    /*
     * Add the data instance elements
     */
    for (DataInstance& data_instance_sf : subject_sf.data_)
    {
      QTreeWidgetItem* data_instance_item = new QTreeWidgetItem(subject_item);
      QString text_data_instance = QString::fromStdString(data_instance_sf.getTypeStr());
      data_instance_item->setText(0, text_data_instance);
      data_instance_item->setFont(0, type_font_);
      InterfaceTreeData data_instance_tdata = InterfaceTreeData(
                                    InterfaceTreeData::DATA,
                                    boost::any_cast<DataInstance*>(&data_instance_sf));
      data_instance_item->setData(0, Qt::UserRole, QVariant::fromValue(data_instance_tdata));
      subject_item->addChild(data_instance_item);
    }
  }
}

// ******************************************************************************************
//
// ******************************************************************************************
void SFEditorWidget::removeActiveTreeElement()
{
  // Get the active tree item
  active_tree_item_ = interfaces_tree_->currentItem();
  active_tree_element_ = active_tree_item_->data(0, Qt::UserRole).value<InterfaceTreeData>();

  QTreeWidgetItem* parent_item;
  InterfaceTreeData parent_element;

  // Check if the active item has a parent
  if (active_tree_item_->parent())
  {
    // Get the parent of the active tree item
    parent_item = active_tree_item_->parent();
    parent_element = parent_item->data(0, Qt::UserRole).value<InterfaceTreeData>();
  }

  /*
   * Check which tree element has to be removed
   */
  switch(active_tree_element_.type_)
  {
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Remove interface
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::INTERFACE):
    {
      Interface* interface = boost::any_cast<Interface*>(active_tree_element_.payload_);

      // Loop through the interfaces and find which interface must be erased
      uint32_t index = 0;
      for (Interface& interface_sf : action_descriptor_->interfaces_)
      {
        if (&interface_sf == interface)
        {
          break;
        }
        index++;
      }

      // Check if the element was found
      if (index >= action_descriptor_->interfaces_.size())
      {
        // The element was not found
        // TODO: throw error
        std::cout << "element not found" << std::endl;
        return;
      }

      // Remove the element
      action_descriptor_->interfaces_.erase(action_descriptor_->interfaces_.begin() + index);
    }
    break;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Remove input subjects. Not.
     * If this part of the code is executed, then there is a bug in the code, since
     * the btn_delete should be disabled if the interface input is highlighted
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::INPUT):
    {
      QMessageBox::information(this, "", "An input of the interface cannot be removed.");
    }

    break;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Remove output subjects.
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::OUTPUT):
    {
      Interface* interface = boost::any_cast<Interface*>(parent_element.payload_);
      interface->output_subjects_.clear();
    }
    break;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Remove a subject.
     * The procedure is the same for the input and output
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::SUBJECT):
    {
      Subjects* subjects = boost::any_cast<Subjects*>(parent_element.payload_);
      Subject* subject = boost::any_cast<Subject*>(active_tree_element_.payload_);

      // Find the index where the subject resides inside the subjects vector
      uint32_t index = 0;
      for (auto& subject_tmp : *subjects)
      {
        if (&subject_tmp == subject)
        {
          break;
        }
        index++;
      }

      // Check if the element was found
      if (index >= subjects->size())
      {
        // The element was not found
        // TODO: throw error
        std::cout << "element not found" << std::endl;
        return;
      }

      // Remove the element
      (*subjects).erase(subjects->begin() + index);
    }
    break;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Remove data.
     * The data vecctor is contained by the parent subject
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::DATA):
    {
      Subject* subject = boost::any_cast<Subject*>(parent_element.payload_);
      std::vector<DataInstance>* data = &(subject->data_);
      DataInstance* data_instance = boost::any_cast<DataInstance*>(active_tree_element_.payload_);

      // Find the index where the subject resides inside the subjects vector
      uint32_t index = 0;
      for (auto& data_tmp : *data)
      {
        if (&data_tmp == data_instance)
        {
          break;
        }
        index++;
      }

      // Check if the element was found
      if (index >= data->size())
      {
        // The element was not found
        // TODO: throw error
        std::cout << "element not found" << std::endl;
        return;
      }

      // Remove the element
      (*data).erase(data->begin() + index);
    }
    break;

    default:
    {
      QMessageBox::critical(this, "Error Loading", "An internal error has occured while loading.");
    }
  }

  /*
   * Refresh the tree
   */
  edit_screen_content_->setCurrentIndex(0);
  refreshTree();
}

// ******************************************************************************************
//
// ******************************************************************************************
void SFEditorWidget::addToActiveTreeElement()
{
  // Get the active tree item
  active_tree_item_ = interfaces_tree_->currentItem();
  active_tree_element_ = active_tree_item_->data(0, Qt::UserRole).value<InterfaceTreeData>();

  switch(active_tree_element_.type_)
  {
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Add output to the interface.
     * Input is there by default and cannot be removed
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::INTERFACE):
    {
      Interface* interface = boost::any_cast<Interface*>(active_tree_element_.payload_);

      // Check if this interface already contains output subjects
      if (!interface->output_subjects_.empty())
      {
        QMessageBox::information(this, "", "This interface already contains an output.");
        return;
      }

      // Add a blank subject to the output interface
      Subject new_subject;
      new_subject.type_ = Subject::WHAT;
      new_subject.words_.push_back("<modify me please>");
      interface->output_subjects_.push_back(new_subject);
    }
    break;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Add new subject element to selected input/output
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::INPUT):
    case (InterfaceTreeData::OUTPUT):
    {
      Subjects* subjects = boost::any_cast<Subjects*>(active_tree_element_.payload_);
      Subject new_subject;
      new_subject.type_ = Subject::WHAT;
      new_subject.words_.push_back("<modify me please>");
      subjects->push_back(new_subject);
    }
    break;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Add new data element to selected subject
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    case (InterfaceTreeData::SUBJECT):
    {
      Subject* subject = boost::any_cast<Subject*>(active_tree_element_.payload_);
      DataInstance new_data_instance;
      new_data_instance.type_ = DataInstance::STRING;
      subject->data_.push_back(new_data_instance);
    }
    break;

    default:
    {
      QMessageBox::critical(this, "Error Loading", "An internal error has occured while loading.");
    }
  }

  // Refresh the tree
  refreshTree();
}

// ******************************************************************************************
//
// ******************************************************************************************
void SFEditorWidget::addInterface()
{
  // Add a new interface with one uninitialized subject
  Subject new_subject;
  new_subject.type_ = Subject::WHAT;
  new_subject.words_.push_back("<modify me please>");

  Interface new_interface;
  new_interface.type_ = Interface::SYNCHRONOUS;
  new_interface.input_subjects_.push_back(new_subject);

  action_descriptor_->interfaces_.push_back(new_interface);
  refreshTree();
}

// ******************************************************************************************
//
// ******************************************************************************************
void SFEditorWidget::refreshTree()
{
  interfaces_tree_->clear();      // Clear the tree
  populateInterfacesTree();       // Create new contents for the tree
  interfaces_tree_->expandAll();  // Expand the tree. TODO: the user might not like it

  // Disable the add/delete buttons, since there is no active tree element after refresh
  btn_add_->setDisabled(true);
  btn_delete_->setDisabled(true);
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
} // temoto_action_assistant namespace
