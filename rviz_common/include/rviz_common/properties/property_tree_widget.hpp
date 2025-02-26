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


#ifndef RVIZ_COMMON__PROPERTIES__PROPERTY_TREE_WIDGET_HPP_
#define RVIZ_COMMON__PROPERTIES__PROPERTY_TREE_WIDGET_HPP_

#include <QTreeView>
#include <QString>

#include "./property_tree_model.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

class Property;
class SplitterHandle;

class RVIZ_COMMON_PUBLIC PropertyTreeWidget : public QTreeView
{
  Q_OBJECT

public:
  explicit PropertyTreeWidget(QWidget * parent = 0);

// disable the overloaded virtual warning, since this is intentional
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif
  /// Set the data model this widget should view.
  void setModel(PropertyTreeModel * model);
#ifdef __clang__
# pragma clang diagnostic pop
#endif

  PropertyTreeModel * getModel() const {return model_;}

  /// Return the list of objects of a given type which are currently selected.
  template<class Type>
  QList<Type *> getSelectedObjects()
  {
    QModelIndexList indexes = selectedIndexes();
    int num_selected = indexes.size();

    QList<Type *> objects_out;

    for (int i = 0; i < num_selected; i++) {
      if (indexes[i].column() == 0) {
        Property * prop = model_->getProp(indexes[i]);
        if (prop != model_->getRoot() ) {
          // TODO(greimela) Figure out why qobject_cast does not work here
          auto obj = dynamic_cast<Type *>(prop);
          if (obj) {
            objects_out.push_back(obj);
          }
        }
      }
    }
    return objects_out;
  }

  /// Write state to the given Config.
  void save(Config config) const;

  /// Read state from the given Config.
  void load(const Config & config);

protected:
  /// Called whenever current item changes.
  /**
   * Calls QTreeView implementation then emits currentPropertyChanged().
   */
  void currentChanged(const QModelIndex & current, const QModelIndex & previous) override;

  /// Called whenever selection changes.
  /**
   * Calls QTreeView implementation then emits selectionHasChanged().
   */
  void selectionChanged(
    const QItemSelection & selected, const QItemSelection & deselected) override;

protected Q_SLOTS:
  virtual void propertyHiddenChanged(const Property * property);

Q_SIGNALS:
  void currentPropertyChanged(const Property * new_current_property);
  void selectionHasChanged();

private:
  /// Recursively write full names of properties which are expanded in this view to the Config.
  void saveExpandedEntries(
    Config config, const QModelIndex & parent_index,
    const QString & prefix) const;

  /// Recursively expand entries whose full names appear in expanded_full_names.
  void expandEntries(
    const QSet<QString> & expanded_full_names,
    const QModelIndex & parent_index,
    const QString & prefix);

  PropertyTreeModel * model_;
  SplitterHandle * splitter_handle_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__PROPERTY_TREE_WIDGET_HPP_
