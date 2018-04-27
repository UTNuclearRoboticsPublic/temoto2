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

  // Top Header Area ------------------------------------------------

  HeaderWidget* header = new HeaderWidget("Semantic Frame Editor", "Edit semantic frames", this);
  layout->addWidget(header);

  // Simple form -------------------------------------------
  QFormLayout* form_layout = new QFormLayout();
  form_layout->setContentsMargins(0, 15, 0, 15);

  // Lexical unit input
  lexical_unit_field_ = new QLineEdit(this);
  lexical_unit_field_->setMaximumWidth(400);
  form_layout->addRow("Lexical Unit:", lexical_unit_field_);

  layout->addLayout(form_layout);
  layout->setAlignment(Qt::AlignTop);


  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
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


}  // namespace
