#include "cloud_viewer/scene/SceneGraph.h"

#include <algorithm>

namespace cloud_viewer
{

SceneObjectId SceneGraph::add(SceneObjectId parentId, SceneObject object)
{
    object.id = nextId_++;
    object.parentId = parentId;
    objects_.push_back(std::move(object));
    return objects_.back().id;
}

void SceneGraph::remove(SceneObjectId id)
{
    // 收集待删除节点（自身 + 所有后代）。
    std::vector<SceneObjectId> toRemove{id};
    for (std::size_t i = 0; i < toRemove.size(); ++i)
    {
        for (const auto &object : objects_)
        {
            if (object.parentId == toRemove[i])
            {
                toRemove.push_back(object.id);
            }
        }
    }

    objects_.erase(
        std::remove_if(objects_.begin(), objects_.end(),
                       [&](const SceneObject &object)
                       {
                           return std::find(toRemove.begin(), toRemove.end(), object.id) !=
                                  toRemove.end();
                       }),
        objects_.end());
}

void SceneGraph::setVisible(SceneObjectId id, bool visible)
{
    if (auto *object = find(id))
    {
        object->visible = visible;
    }
}

bool SceneGraph::effectiveVisible(SceneObjectId id) const
{
    const SceneObject *object = find(id);
    while (object)
    {
        if (!object->visible)
        {
            return false;
        }
        if (object->parentId == 0)
        {
            break;
        }
        object = find(object->parentId);
    }
    return true;
}

const SceneObject *SceneGraph::find(SceneObjectId id) const
{
    for (const auto &object : objects_)
    {
        if (object.id == id)
        {
            return &object;
        }
    }
    return nullptr;
}

SceneObject *SceneGraph::find(SceneObjectId id)
{
    for (auto &object : objects_)
    {
        if (object.id == id)
        {
            return &object;
        }
    }
    return nullptr;
}

std::vector<SceneObjectId> SceneGraph::visibleClouds() const
{
    std::vector<SceneObjectId> ids;
    for (const auto &object : objects_)
    {
        if (object.kind == SceneObjectKind::Cloud && object.renderable &&
            effectiveVisible(object.id))
        {
            ids.push_back(object.id);
        }
    }
    return ids;
}

void SceneGraph::clear()
{
    objects_.clear();
    nextId_ = 1;
}

} // namespace cloud_viewer
