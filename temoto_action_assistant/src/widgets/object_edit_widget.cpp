#include "object_edit_widget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>

namespace temoto_action_assistant
{

// ******************************************************************************************
// Constructor
// ******************************************************************************************
ObjectEditWidget::ObjectEditWidget(QWidget *parent)
: QWidget(parent)
{
  // TODO: add a description element to the widget

  QVBoxLayout* object_editor_layout = new QVBoxLayout();
  // TODO: align it to the top

  QFormLayout* object_form_layout = new QFormLayout();
  object_form_layout->setContentsMargins(0, 15, 0, 15);
  object_editor_layout->addLayout(object_form_layout);

  /*
   * Create object type editing combobox
   */
  object_type_field_ = new QComboBox(this);
  object_type_field_->setMaximumWidth(400);
  object_form_layout->addRow("Type:", object_type_field_);

  connect(object_type_field_, SIGNAL(highlighted(QString)),
          this, SLOT(modifyType(QString)));

  // Add items to the combo box
  for (auto object_type : Object().type_to_str)
  {
    object_type_field_->addItem(object_type.second.c_str());
  }

  /*
   * Create the word editing field
   */
  object_word_field_ = new QLineEdit(this);
  object_word_field_->setMaximumWidth(400);
  object_form_layout->addRow("Word:", object_word_field_);

  connect(object_word_field_, SIGNAL(returnPressed()), this, SLOT(modifyWord()));

  this->setLayout(object_editor_layout);
}

// ******************************************************************************************
//
// ******************************************************************************************
void ObjectEditWidget::modifyWord()
{
  Object* object = boost::any_cast<Object*>(tree_data_.payload_);
  object->words_[0] = object_word_field_->text().toStdString();

  QString text = QString::fromStdString(object->getTypeStr() + ": " + object->words_[0]);
  tree_item_ptr_->setText(0, text);
}

// ******************************************************************************************
//
// ******************************************************************************************
void ObjectEditWidget::modifyType(const QString &text)
{
  Object* object = boost::any_cast<Object*>(tree_data_.payload_);
  object->setTypeByStr(text.toStdString());

  QString item_text = QString::fromStdString(object->getTypeStr() + ": " + object->words_[0]);
  tree_item_ptr_->setText(0, item_text);
}

// ******************************************************************************************
//
// ******************************************************************************************
void ObjectEditWidget::focusGiven(QTreeWidgetItem* tree_item_ptr)
{
  // Set the tree item pointer to active element
  tree_item_ptr_ = tree_item_ptr;
  tree_data_ = tree_item_ptr_->data(0, Qt::UserRole).value<InterfaceTreeData>();
  Object* object = boost::any_cast<Object*>(tree_data_.payload_);

  /*
   * Update the word field
   */
  object_word_field_->setText(QString::fromStdString(object->words_[0]));

  /*
   * Update the type field
   */
  int index = object_type_field_->findText(object->getTypeStr().c_str());
  if (index == -1)
  {
    // TODO: Throw an error
    return;
  }

  object_type_field_->setCurrentIndex(index);
}


} // temoto_action_assistant namespace
