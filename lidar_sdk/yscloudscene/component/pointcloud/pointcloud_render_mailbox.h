#pragma once

#include "pointcloud_render_frame.h"

#include <array>
#include <mutex>
#include <utility>

#include <QObject>

namespace ys_scene {

// Workers only touch this mailbox, never the widget or its OpenGL context.
class PointcloudRenderMailbox : public QObject
{
    Q_OBJECT

public:
    using PendingFrames = std::array<PointcloudRenderFramePtr, 4>;

    void Publish(PointcloudRenderFramePtr frame)
    {
        if (!frame || !frame->IsValid() || frame->sequence == 0) {
            return;
        }
        bool notify = false;
        PointcloudRenderFramePtr retired;
        {
            const std::scoped_lock lock(mutex_);
            const auto index = all_mirrors_ ? static_cast<std::size_t>(frame->frame_id % 4) : 0;
            if (closed_ || frozen_ || frame->sequence <= latest_sequences_[index]) {
                return;
            }
            latest_sequences_[index] = frame->sequence;
            retired = std::exchange(pending_[index], std::move(frame));
            notify = !notification_pending_;
            notification_pending_ = true;
        }
        if (notify) {
            emit FramesAvailable();
        }
    }

    [[nodiscard]] PendingFrames Take()
    {
        const std::scoped_lock lock(mutex_);
        notification_pending_ = false;
        return std::exchange(pending_, PendingFrames{});
    }

    void SetAllMirrors(bool enabled)
    {
        PendingFrames retired;
        const std::scoped_lock lock(mutex_);
        all_mirrors_ = enabled;
        retired = std::exchange(pending_, PendingFrames{});
        latest_sequences_.fill(NextPointcloudRenderSequence());
    }

    void SetFrozen(bool frozen)
    {
        PendingFrames retired;
        const std::scoped_lock lock(mutex_);
        frozen_ = frozen;
        retired = std::exchange(pending_, PendingFrames{});
        latest_sequences_.fill(NextPointcloudRenderSequence());
    }

    void Close()
    {
        PendingFrames retired;
        const std::scoped_lock lock(mutex_);
        closed_ = true;
        retired = std::exchange(pending_, PendingFrames{});
    }

signals:
    void FramesAvailable();

private:
    std::mutex mutex_;
    PendingFrames pending_{};
    std::array<std::uint64_t, 4> latest_sequences_{};
    bool all_mirrors_{false};
    bool frozen_{false};
    bool closed_{false};
    bool notification_pending_{false};
};

} // namespace ys_scene
