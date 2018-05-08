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

#include "generate_package_widget.h"
#include "temoto_action_assistant/semantic_frame_yaml.h"
#include <iostream>
#include <QVBoxLayout>

namespace temoto_action_assistant
{

GeneratePackageWidget::GeneratePackageWidget(QWidget* parent, ActionDescriptorPtr action_descriptor)
: SetupScreenWidget(parent),
  action_descriptor_(action_descriptor)
{
  // Layout for "add/remove selected" buttons
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Add to Selected Button
  btn_generate_package_ = new QPushButton("&Generate", this);
  btn_generate_package_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  //btn_add_->setMaximumWidth(400);
  connect(btn_generate_package_, SIGNAL(clicked()), this, SLOT(generatePackage()));
  layout->addWidget(btn_generate_package_);
  layout->setAlignment(btn_generate_package_, Qt::AlignCenter);

  this->setLayout(layout);
}

void GeneratePackageWidget::generatePackage()
{
  YAML::Node action_descriptor_node = YAML::Node(*action_descriptor_);

  std::cout << action_descriptor_node << std::endl;

  ActionDescriptor new_action_descriptor = action_descriptor_node.as<ActionDescriptor>();

  std::cout << "-----------------*-*-******************" << std::endl;

  YAML::Node new_action_descriptor_node = YAML::Node(new_action_descriptor);

  std::cout << new_action_descriptor_node << std::endl;


}

} // temoto action assistant namespace
