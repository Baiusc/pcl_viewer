/**
 * @brief 轻量场景实体树
 *
 * 参考 CloudCompare 的 ccHObject 层次与可见性规则：
 *  - 子对象最终可见性 = 自身可见 && 所有父节点可见。
 *  - 删除父节点递归删除子节点。
 *  - Actor ID 使用稳定实体 ID，不用文件名。
 */
#pragma once

#include <string>
#include <vector>

#include "cloud_viewer/core/CloudFrame.h"
#include "cloud_viewer/render/CloudRenderBridge.h"

namespace cloud_viewer
{

enum class SceneObjectKind
{
    Root = 0,
    File,
    Frame,
    Cloud
};

struct SceneObject
{
    SceneObjectId id = 0;
    SceneObjectId parentId = 0;
    std::string name;
    SceneObjectKind kind = SceneObjectKind::Cloud;
    bool visible = true;

    FrameHandle frame;
    RenderableCloud::Ptr renderable;
};

class SceneGraph
{
public:
    SceneObjectId add(SceneObjectId parentId, SceneObject object);

    void remove(SceneObjectId id);

    void setVisible(SceneObjectId id, bool visible);

    bool effectiveVisible(SceneObjectId id) const;

    const SceneObject *find(SceneObjectId id) const;
    SceneObject *find(SceneObjectId id);

    const std::vector<SceneObject> &objects() const { return objects_; }

    std::vector<SceneObjectId> visibleClouds() const;

    void clear();

private:
    std::vector<SceneObject> objects_;
    SceneObjectId nextId_ = 1;
};

} // namespace cloud_viewer
