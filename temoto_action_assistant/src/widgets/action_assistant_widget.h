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

#ifndef TEMOTO_ACTION_ASSISTANT_ACTION_ASSISTANT_WIDGET
#define TEMOTO_ACTION_ASSISTANT_ACTION_ASSISTANT_WIDGET

// Qt
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>
#include <QApplication>
#include <QObject>
#include <QEvent>
#include <QListWidget>
#include <QTimer>
#include <QSplitter>
#include <QStringList>
// Setup Asst
#include "navigation_widget.h"
#include "start_screen_widget.h"
#include "sf_editor_widget.h"
#include "generate_package_widget.h"

#ifndef Q_MOC_RUN

// Other
#include <ros/ros.h>
#include <boost/program_options.hpp>  // for parsing input arguments
#include <boost/thread/mutex.hpp>
#endif

namespace temoto_action_assistant
{
class ActionAssistantWidget : public QWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /**
   * Construct the setup assistant widget, the primary window for this application
   * @param parent - used by Qt for destructing all elements
   * @return
   */
  ActionAssistantWidget(QWidget* parent, boost::program_options::variables_map args);

  /**
   * Deconstructor
   *
   */
  ~ActionAssistantWidget();

  /**
   * Changes viewable screen
   * @param index screen index to switch to
   */

  void moveToScreen(const int index);

  /**
   * Qt close event function for reminding user to save
   * @param event A Qt paramenter
   */
  void closeEvent(QCloseEvent* event);

  /**
   * Qt error handling function
   *
   * @param rec
   * @param ev
   * @return bool
   */
  virtual bool notify(QObject* rec, QEvent* ev);

  /**
   * Show/hide the Rviz right panel
   * @param show bool - whether to show
   */
  // void showRviz( bool show );

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

private Q_SLOTS:
  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /**
   * Event for changing screens by user clicking
   * @param index screen id
   */
  void navigationClicked(const QModelIndex& index);

  /**
   * Event for spinning the ros node
   */
  void updateTimer();

  /**
   * Call a function that enables navigation and goes to screen 2
   */
  void progressPastStartScreen();

  /**
   * Change the widget modal state based on subwidgets state
   *
   * @param isModal if true disable left navigation
   */
  void setModalMode(bool isModal);

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************
  QList<QString> nav_name_list_;
  NavigationWidget* navs_view_;

  QWidget* middle_frame_;
  QSplitter* splitter_;
  QStackedLayout* main_content_;
  int current_index_;
  boost::mutex change_screen_lock_;

  // Screen Widgets
  StartScreenWidget* ssw_;
  SFEditorWidget* sfew_;
  GeneratePackageWidget* gpw_;

  /// Contains all the configuration data
  ActionDescriptorPtr action_descriptor_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************
};
}

#endif
