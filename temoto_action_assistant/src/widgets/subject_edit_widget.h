#ifndef TEMOTO_ACTION_ASSISTANT_SUBJECT_EDIT_WIDGET
#define TEMOTO_ACTION_ASSISTANT_SUBJECT_EDIT_WIDGET

#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QTreeWidget>

#ifndef Q_MOC_RUN
#endif

#include "interface_tree_data.h"
#include "temoto_action_assistant/semantic_frame.h"

namespace temoto_action_assistant
{
class SubjectEditWidget : public QWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Constructor
  SubjectEditWidget(QWidget* parent);

  /// Focus given
  void focusGiven(QTreeWidgetItem* tree_item_ptr);

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  QLabel* title_;  // specify the title from the parent widget
  QLineEdit* subject_word_field_;
  QComboBox* subject_type_field_;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Modifies the word variable
  void modifyWord();

  /// Modify the type variable
  void modifyType(const QString &text);

Q_SIGNALS:

  // ******************************************************************************************
  // Emitted Signals
  // ******************************************************************************************

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Points to the active element of the interface tree
  InterfaceTreeData tree_data_;
  QTreeWidgetItem* tree_item_ptr_;


  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************


};
}

#endif
