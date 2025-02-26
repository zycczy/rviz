// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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


#ifndef RVIZ_COMMON__PROPERTIES__INT_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__INT_PROPERTY_HPP_

#include <QString>

#include "rviz_common/properties/property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

/// Property specialized to provide max/min enforcement for integers.
class RVIZ_COMMON_PUBLIC IntProperty : public Property
{
  Q_OBJECT

public:
  /// Constructor.
  /**
   * \param name The name of this property.
   *   Appears in the left column of a PropertyTreeWidget.
   * \param default_value The initial value to store in the property.
   *   Appears in the right column of a PropertyTreeWidget.
   * \param description Text describing the property.
   *   Is shown in the "help" area of a PropertyTreeWithHelp widget.
   * \param parent The parent Property, or NULL if there is no parent at this
   *   time.
   * \param changed_slot This should be a Qt slot specification,
   *   generated by Qt's @c SLOT() macro.
   *   It should be a slot on the receiver object, or if receiver is not
   *   specified, it should be a slot on the parent.
   * \param receiver If receiver is non-NULL, the changed() signal is
   *   connected to the changed_slot on the receiver object.
   */
  explicit IntProperty(
    const QString & name = QString(),
    int default_value = 0,
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0,
    int min_value = INT_MIN,
    int max_value = INT_MAX);

  /// Set the new value for this property.
  /**
   * Returns true if the new value is different from the old value, else false.
   *
   * If the new value is different from the old value, this emits
   * aboutToChange() before changing the value and changed() after.
   *
   * Overridden from Property::setValue() to enforce minimum and maximum.
   */
  bool setValue(const QVariant & new_value) override;

  /// Return the internal property value as an integer.
  /**
   * If a non-integer value was stored in this property, this will return 0.
   */
  virtual int getInt() const;

  /// Set the minimum value to be enforced.
  void setMin(int min);

  /// Set the minimum value enforced.
  int getMin();

  /// Set the maximum value to be enforced.
  void setMax(int max);

  /// Get the maximum value to be enforced.
  int getMax();

  /// Called when the editor is created.
  /**
   * Overridden to create a QSpinBox with the min and max set and with a
   * signal/slot connection to setInt(), so the Property value updates every
   * time the value changes, not just when "return" is pressed.
   */
  QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option) override;

public Q_SLOTS:
  /// Set the value of this property to the given integer.
  /**
   * This just calls setValue(), which is where the min/max are enforced.
   */
  void setInt(int new_value);

private:
  int min_;
  int max_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__INT_PROPERTY_HPP_
