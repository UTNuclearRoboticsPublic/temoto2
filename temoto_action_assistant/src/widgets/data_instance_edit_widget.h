#ifndef TEMOTO_ACTION_ASSISTANT_DATA_INSTANCE_EDIT_WIDGET
#define TEMOTO_ACTION_ASSISTANT_DATA_INSTANCE_EDIT_WIDGET

#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QTreeWidget>

#ifndef Q_MOC_RUN
#endif

#include "interface_tree_data.h"
#include "temoto_action_assistant/semantic_frame.h"

namespace temoto_action_assistant
{
class DataInstanceEditWidget : public QWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Constructor
  DataInstanceEditWidget(QWidget* parent);

  /// Focus given
  void focusGiven(QTreeWidgetItem* tree_item_ptr);

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  QLabel* title_;  // specify the title from the parent widget
  QLineEdit* subject_word_field_;
  QComboBox* subject_type_field_;
  QPushButton* btn_delete_;      // this button is hidden for new groups
  QPushButton* btn_save_;        // this button is hidden for new groups
  QWidget* new_buttons_widget_;  // for showing/hiding the new group buttons

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
