#include "subject_edit_widget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>

namespace temoto_action_assistant
{

// ******************************************************************************************
// Constructor
// ******************************************************************************************
SubjectEditWidget::SubjectEditWidget(QWidget *parent)
: QWidget(parent)
{
  // TODO: add a description element to the widget

  QVBoxLayout* subject_editor_layout = new QVBoxLayout();
  // TODO: align it to the top

  QFormLayout* subject_form_layout = new QFormLayout();
  subject_form_layout->setContentsMargins(0, 15, 0, 15);
  subject_editor_layout->addLayout(subject_form_layout);

  /*
   * Create subject type editing combobox
   */
  subject_type_field_ = new QComboBox(this);
  subject_type_field_->setMaximumWidth(400);
  subject_form_layout->addRow("Type:", subject_type_field_);

  connect(subject_type_field_, SIGNAL(highlighted(QString)),
          this, SLOT(modifyType(QString)));

  // Add items to the combo box
  for (auto subject_type : Subject().type_to_str)
  {
    subject_type_field_->addItem(subject_type.second.c_str());
  }

  /*
   * Create the word editing field
   */
  subject_word_field_ = new QLineEdit(this);
  subject_word_field_->setMaximumWidth(400);
  subject_form_layout->addRow("Word:", subject_word_field_);

  connect(subject_word_field_, SIGNAL(returnPressed()), this, SLOT(modifyWord()));

  this->setLayout(subject_editor_layout);
}

// ******************************************************************************************
//
// ******************************************************************************************
void SubjectEditWidget::modifyWord()
{
  Subject* subject = boost::any_cast<Subject*>(tree_data_.payload_);
  subject->words_[0] = subject_word_field_->text().toStdString();

  QString text = QString::fromStdString(subject->getTypeStr() + ": " + subject->words_[0]);
  tree_item_ptr_->setText(0, text);
}

// ******************************************************************************************
//
// ******************************************************************************************
void SubjectEditWidget::modifyType(const QString &text)
{
  Subject* subject = boost::any_cast<Subject*>(tree_data_.payload_);
  subject->setTypeByStr(text.toStdString());

  QString item_text = QString::fromStdString(subject->getTypeStr() + ": " + subject->words_[0]);
  tree_item_ptr_->setText(0, item_text);
}

// ******************************************************************************************
//
// ******************************************************************************************
void SubjectEditWidget::focusGiven(QTreeWidgetItem* tree_item_ptr)
{
  // Set the tree item pointer to active element
  tree_item_ptr_ = tree_item_ptr;
  tree_data_ = tree_item_ptr_->data(0, Qt::UserRole).value<InterfaceTreeData>();
  Subject* subject = boost::any_cast<Subject*>(tree_data_.payload_);

  /*
   * Update the word field
   */
  subject_word_field_->setText(QString::fromStdString(subject->words_[0]));

  /*
   * Update the type field
   */
  int index = subject_type_field_->findText(subject->getTypeStr().c_str());
  if (index == -1)
  {
    // TODO: Throw an error
    return;
  }

  subject_type_field_->setCurrentIndex(index);
}


} // temoto_action_assistant namespace
