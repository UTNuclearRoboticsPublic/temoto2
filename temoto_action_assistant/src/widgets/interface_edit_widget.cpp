#include "interface_edit_widget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>

#include <iostream>

namespace temoto_action_assistant
{

// ******************************************************************************************
// Constructor
// ******************************************************************************************
InterfaceEditWidget::InterfaceEditWidget(QWidget *parent)
: QWidget(parent)
{
  // TODO: add a description element to the widget

  QVBoxLayout* interface_editor_layout = new QVBoxLayout();
  // TODO: align it to the top

  QFormLayout* interface_form_layout = new QFormLayout();
  interface_form_layout->setContentsMargins(0, 15, 0, 15);
  interface_editor_layout->addLayout(interface_form_layout);

  /*
   * Create subject type editing combobox
   */
  interface_type_field_ = new QComboBox(this);
  interface_type_field_->setMaximumWidth(400);
  interface_form_layout->addRow("Type:", interface_type_field_);

  connect(interface_type_field_, SIGNAL(highlighted(QString)), this, SLOT(modifyType(QString)));

  // Add items to the combo box
  for (auto interface_type : Interface().type_to_str)
  {
    interface_type_field_->addItem(interface_type.second.c_str());
  }

  this->setLayout(interface_editor_layout);
}

// ******************************************************************************************
//
// ******************************************************************************************
void InterfaceEditWidget::modifyType(const QString &text)
{
  Interface* interface = boost::any_cast<Interface*>(tree_data_.payload_);
  interface->setTypeByStr(text.toStdString());

  std::string interface_str =  std::string("Interface ") + std::to_string(interface->id_) + ": ";
  QString item_text = QString::fromStdString(interface_str) + text;
  tree_item_ptr_->setText(0, item_text);
}

// ******************************************************************************************
//
// ******************************************************************************************
void InterfaceEditWidget::focusGiven(QTreeWidgetItem* tree_item_ptr)
{
  // Set the tree item pointer to active element
  tree_item_ptr_ = tree_item_ptr;
  tree_data_ = tree_item_ptr_->data(0, Qt::UserRole).value<InterfaceTreeData>();
  Interface* interface = boost::any_cast<Interface*>(tree_data_.payload_);

  /*
   * Update the type field
   */
  int index = interface_type_field_->findText(interface->getTypeStr().c_str());
  if (index == -1)
  {
    // TODO: Throw an error
    return;
  }

  interface_type_field_->setCurrentIndex(index);
}


} // temoto_action_assistant namespace
