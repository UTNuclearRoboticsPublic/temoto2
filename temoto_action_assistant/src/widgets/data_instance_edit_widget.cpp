#include "data_instance_edit_widget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>

namespace temoto_action_assistant
{

// ******************************************************************************************
// Constructor
// ******************************************************************************************
DataInstanceEditWidget::DataInstanceEditWidget(QWidget *parent)
: QWidget(parent)
{
  // TODO: add a description element to the widget

  QVBoxLayout* data_instance_editor_layout = new QVBoxLayout();
  // TODO: align it to the top

  QFormLayout* data_form_layout = new QFormLayout();
  data_form_layout->setContentsMargins(0, 15, 0, 15);
  data_instance_editor_layout->addLayout(data_form_layout);

  /*
   * Create subject type editing combobox
   */
  data_type_field_ = new QComboBox(this);
  data_type_field_->setMaximumWidth(400);
  data_form_layout->addRow("Type:", data_type_field_);

  connect(data_type_field_, SIGNAL(highlighted(QString)), this, SLOT(modifyType(QString)));

  // Add items to the combo box
  for (auto data_type : DataInstance().type_to_str)
  {
    data_type_field_->addItem(data_type.second.c_str());
  }

  this->setLayout(data_instance_editor_layout);
}

// ******************************************************************************************
//
// ******************************************************************************************
void DataInstanceEditWidget::modifyType(const QString &text)
{
  DataInstance* data_instance = boost::any_cast<DataInstance*>(tree_data_.payload_);
  data_instance->setTypeByStr(text.toStdString());

  QString item_text = QString::fromStdString(data_instance->getTypeStr());
  tree_item_ptr_->setText(0, item_text);
}

// ******************************************************************************************
//
// ******************************************************************************************
void DataInstanceEditWidget::focusGiven(QTreeWidgetItem* tree_item_ptr)
{
  // Set the tree item pointer to active element
  tree_item_ptr_ = tree_item_ptr;
  tree_data_ = tree_item_ptr_->data(0, Qt::UserRole).value<InterfaceTreeData>();
  DataInstance* data_instance = boost::any_cast<DataInstance*>(tree_data_.payload_);

  /*
   * Update the type field
   */
  int index = data_type_field_->findText(data_instance->getTypeStr().c_str());
  if (index == -1)
  {
    // TODO: Throw an error
    return;
  }

  data_type_field_->setCurrentIndex(index);
}


} // temoto_action_assistant namespace
