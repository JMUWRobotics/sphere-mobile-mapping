#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <typeinfo>
#include <cstdint>

#include <ros/time.h>

namespace lio_gf_nodelet_manager
{
    // Data container passed by value between lio and gf nodelet.
    // Holds immutable snapshot payload and metadata.
    struct SharedIKDTreeHandle
    {
        // Type-erased pointer to immutable snapshot payload.
        std::shared_ptr<void> snapshot_payload;
        uint64_t version = 0;
        uint64_t point_count = 0;
        ros::Time stamp;
        std::string frame_id;
        std::string payload_type; // typeid(PayloadT).name(), for checking
    };

    class SharedIKDTree
    {
    public:
        // Returns the single process-wide instance.
        // Both nodelets call this to reach the same tree
        static SharedIKDTree &instance() // returns global
        {
            static SharedIKDTree tree;
            return tree;
        }

        template <typename PayloadT>

        // Called by the owning nodelet to register/update a snapshot payload.
        // Converts shared_ptr<PayloadT> to shared_ptr<void>, preserving the ref-count,
        // then stores it together with metadata and a monotonically increasing version.
        // Locks meta_mutex_ so a concurrent snapshot() sees a consistent handle.
        void publish(const std::shared_ptr<PayloadT> &payload_ptr, const ros::Time &stamp, const std::string &frame_id, uint64_t point_count = 0)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            handle_.snapshot_payload = payload_ptr;
            ++handle_.version;
            handle_.point_count = point_count;
            handle_.stamp = stamp;
            handle_.frame_id = frame_id;
            handle_.payload_type = typeid(PayloadT).name();
        }

        // Called by consumer nodelets to obtain a copy of the current handle.
        // Copying shared_ptr<void> increments the ref-count, so the snapshot stays
        // alive even if the owning nodelet updates to newer versions.
        // Locks meta_mutex_ to ensure the returned handle is self-consistent.
        SharedIKDTreeHandle snapshot() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return handle_;
        }

        template <typename PayloadT>
        // Casts the type-erased snapshot_payload back to shared_ptr<PayloadT>.
        // Safe only when PayloadT matches the type originally passed to publish().
        // no runtime check performed. Use handle.payload_type to verify if needed
        static std::shared_ptr<PayloadT> castPayload(const SharedIKDTreeHandle &handle)
        {
            return std::static_pointer_cast<PayloadT>(handle.snapshot_payload);
        }

    private:
        SharedIKDTree() = default;

        mutable std::mutex mutex_; // guards handle_ struct only, not tree contents
        SharedIKDTreeHandle handle_;
    };

    // Backward-compatible alias used by existing code paths.
    using SharedIKDTreeStore = SharedIKDTree;
} // namespace lio_gf_nodelet_manager
