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

#ifndef TEMOTO_ACTION_ASSISTANT_SEMANTIC_FRAME_EDIT_WIDGET
#define TEMOTO_ACTION_ASSISTANT_SEMANTIC_FRAME_EDIT_WIDGET

// Qt
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStackedLayout>
#include <QString>
#include <QTreeWidget>

// SA
#ifndef Q_MOC_RUN
#endif

#include "header_widget.h"
#include "setup_screen_widget.h"  // a base class for screens in the setup assistant
#include "interface_edit_widget.h"
#include "subject_edit_widget.h"
#include "data_instance_edit_widget.h"
#include "boost/any.hpp"
#include "temoto_action_assistant/semantic_frame.h"
#include "interface_tree_data.h"

namespace temoto_action_assistant
{

class SFEditorWidget : public SetupScreenWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  SFEditorWidget(QWidget* parent, temoto_action_assistant::ActionDescriptorPtr action_descriptor);

  /// Recieved when this widget is chosen from the navigation menu
  virtual void focusGiven();

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************
  void selectionUpdated();

  /// Edit whatever element is selected in the tree view
  void editSelected();

  /// Modifies the lexical unit string
  void modifyLexicalUnit();

  /// Remove the active tree element
  void removeActiveTreeElement();

  /// Add a child element to the active tree element
  void addToActiveTreeElement();

  /// Add a new interface
  void addInterface();

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Fonts
  const QFont top_level_font_;
  const QFont io_font_;
  const QFont type_font_;

  /// Contains all the configuration data for the semantic frame
  temoto_action_assistant::ActionDescriptorPtr action_descriptor_;

  /// Variables for maintaining editing information
  InterfaceTreeData active_tree_element_;
  QTreeWidgetItem* active_tree_item_;

  /// Widgets for editing the contents of the interfaces tree
  InterfaceEditWidget* iew_;
  SubjectEditWidget* sew_;
  DataInstanceEditWidget* diew_;

  /// Main table for holding groups
  QTreeWidget* interfaces_tree_;
  QWidget* interfaces_tree_widget_;
  QPushButton* btn_add_;
  QPushButton* btn_delete_;
  QPushButton* btn_add_interface_;

  QStackedLayout* edit_screen_content_;
  QLineEdit* lexical_unit_field_;



  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /// Builds the main screen list widget
  QWidget* createContentsWidget();

  /// Populates the interfaces tree
  void populateInterfacesTree();

  /// Populate subjects
  void populateSubjects(QTreeWidgetItem* parent_item, Subjects& subjects);

  /// Removes the data that interfaces tree element points to
  void removeData(InterfaceTreeData &parent, InterfaceTreeData &child);

  /// Builds a new tree based on updated action descriptor
  void refreshTree();

};

}  // namespace

#endif
