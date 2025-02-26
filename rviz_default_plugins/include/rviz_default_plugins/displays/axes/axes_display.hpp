// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__AXES__AXES_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__AXES__AXES_DISPLAY_HPP_

#include <memory>

#include "rviz_common/display.hpp"

namespace rviz_rendering
{
class Axes;
}

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class TfFrameProperty;
}
}

namespace rviz_default_plugins
{
namespace displays
{

class AxesDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  AxesDisplay();

  ~AxesDisplay() override;

  void onInitialize() override;

  // Overrides from Display
  void update(float dt, float ros_dt) override;

protected:
  void onEnable() override;

  void onDisable() override;

private Q_SLOTS:
  void updateShape();

private:
  std::shared_ptr<rviz_rendering::Axes> axes_;

  rviz_common::properties::FloatProperty * length_property_;
  rviz_common::properties::FloatProperty * radius_property_;
  rviz_common::properties::TfFrameProperty * frame_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__AXES__AXES_DISPLAY_HPP_
