#pragma once

/*
FATP_META:
  meta_version: 1
  component: TelemetryLog
  file_role: public_header
  path: include/drone/TelemetryLog.h
  namespace: drone
  layer: Domain
  summary: Rolling telemetry event log connecting to DroneEventHub signals.
  api_stability: in_work
  related:
    tests:
      - components/TelemetryLog/tests/test_TelemetryLog.cpp
  hygiene:
    pragma_once: true
    include_guard: false
    defines_total: 0
    defines_unprefixed: 0
    undefs_total: 0
    includes_windows_h: false
*/

/**
 * @file TelemetryLog.h
 * @brief Rolling telemetry log that connects to DroneEventHub signals.
 *
 * @details
 * TelemetryLog subscribes to a DroneEventHub at construction and records every
 * subsystem change, vehicle state transition, and safety alert into an
 * in-memory circular log. The log can be queried for recent entries.
 *
 * Because fat_p::CircularBuffer is an SPSC (single-producer, single-consumer)
 * ring buffer without a forEach API, TelemetryLog maintains a parallel
 * std::deque<TelemetryEntry> bounded to kMaxEntries, which supports
 * random-access snapshot. This matches the single-threaded usage model of this
 * application (all emits and reads occur on the control thread).
 *
 * @note Thread-safety: NOT thread-safe. All operations must occur on the
 *       single control thread that fires DroneEventHub signals.
 */

#include "DroneEvents.h"
#include "Signal.h"

#include <chrono>
#include <deque>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace drone
{

// ============================================================================
// TelemetryEntry
// ============================================================================

/// @brief Categories of telemetry events.
enum class EventCategory
{
    SubsystemEnabled,
    SubsystemDisabled,
    SubsystemError,
    StateTransition,
    TransitionRejected,
    SafetyAlert,
    Info
};

/**
 * @brief A single timestamped telemetry log entry.
 */
struct TelemetryEntry
{
    std::chrono::steady_clock::time_point timestamp;
    EventCategory category;
    std::string subject;  ///< Subsystem name, state name, or similar subject
    std::string detail;   ///< Human-readable detail string
};

// ============================================================================
// TelemetryLog
// ============================================================================

/**
 * @brief Rolling telemetry log connected to a DroneEventHub.
 *
 * Subscribes to the hub's signals at construction and appends entries
 * to a bounded deque. ScopedConnections ensure automatic disconnection
 * when TelemetryLog is destroyed.
 *
 * @tparam MaxEntries Maximum number of entries retained. Oldest entries are
 *                    evicted when this limit is reached.
 */
template <std::size_t MaxEntries = 512>
class TelemetryLog
{
public:
    static constexpr std::size_t kMaxEntries = MaxEntries;

    /**
     * @brief Constructs the telemetry log and wires it to the event hub.
     *
     * @param hub DroneEventHub to subscribe to. Must outlive this log.
     */
    explicit TelemetryLog(drone::events::DroneEventHub& hub)
    {
        // Wire signals. ScopedConnections disconnect automatically on destruction.
        mConnections.push_back(
            hub.onSubsystemChanged.connect(
                [this](std::string_view name, bool enabled)
                {
                    append(enabled ? EventCategory::SubsystemEnabled : EventCategory::SubsystemDisabled,
                           std::string(name),
                           enabled ? "enabled" : "disabled");
                }));

        mConnections.push_back(
            hub.onSubsystemError.connect(
                [this](std::string_view name, std::string_view reason)
                {
                    append(EventCategory::SubsystemError, std::string(name), std::string(reason));
                }));

        mConnections.push_back(
            hub.onVehicleStateChanged.connect(
                [this](std::string_view from, std::string_view to)
                {
                    std::string detail = from.empty()
                        ? std::string("initial -> ") + std::string(to)
                        : std::string(from) + " -> " + std::string(to);
                    append(EventCategory::StateTransition, std::string(to), std::move(detail));
                }));

        mConnections.push_back(
            hub.onTransitionRejected.connect(
                [this](std::string_view command, std::string_view reason)
                {
                    append(EventCategory::TransitionRejected,
                           std::string(command),
                           std::string(reason));
                }));

        mConnections.push_back(
            hub.onSafetyAlert.connect(
                [this](std::string_view alert)
                {
                    append(EventCategory::SafetyAlert, std::string(alert), {});
                }));
    }

    // Non-copyable; ScopedConnections are move-only
    TelemetryLog(const TelemetryLog&) = delete;
    TelemetryLog& operator=(const TelemetryLog&) = delete;

    // -------------------------------------------------------------------------
    // Query API
    // -------------------------------------------------------------------------

    /**
     * @brief Returns the number of entries currently in the log.
     */
    [[nodiscard]] std::size_t size() const noexcept
    {
        return mEntries.size();
    }

    /**
     * @brief Returns true if the log contains no entries.
     */
    [[nodiscard]] bool empty() const noexcept
    {
        return mEntries.empty();
    }

    /**
     * @brief Returns all entries as a vector (oldest first).
     */
    [[nodiscard]] const std::deque<TelemetryEntry>& all() const noexcept
    {
        return mEntries;
    }

    /**
     * @brief Returns the most recent N entries (oldest first within the result).
     *
     * @param n Number of entries to return. Clamped to size().
     */
    [[nodiscard]] std::vector<TelemetryEntry> recent(std::size_t n) const
    {
        n = std::min(n, mEntries.size());
        return {mEntries.end() - static_cast<std::ptrdiff_t>(n), mEntries.end()};
    }

    /**
     * @brief Formats the last N entries as human-readable lines.
     *
     * Each line: [+ms] CATEGORY subject: detail
     *
     * @param n Number of entries (clamped to size()).
     */
    [[nodiscard]] std::string formatTail(std::size_t n) const
    {
        auto entries = recent(n);
        if (entries.empty())
        {
            return "(no telemetry entries)\n";
        }

        const auto& first = entries.front();
        std::ostringstream oss;
        for (const auto& e : entries)
        {
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                e.timestamp - first.timestamp).count();

            oss << "[+" << ms << "ms] "
                << categoryName(e.category) << " "
                << e.subject;
            if (!e.detail.empty())
            {
                oss << ": " << e.detail;
            }
            oss << "\n";
        }
        return oss.str();
    }

    /**
     * @brief Manually appends an informational message to the log.
     */
    void logInfo(std::string subject, std::string detail)
    {
        append(EventCategory::Info, std::move(subject), std::move(detail));
    }

    /**
     * @brief Clears all log entries.
     */
    void clear() noexcept
    {
        mEntries.clear();
    }

    /**
     * @brief Returns the display string for an EventCategory.
     */
    [[nodiscard]] static std::string_view categoryName(EventCategory c) noexcept
    {
        switch (c)
        {
            case EventCategory::SubsystemEnabled:  return "ENABLED";
            case EventCategory::SubsystemDisabled: return "DISABLED";
            case EventCategory::SubsystemError:    return "ERROR";
            case EventCategory::StateTransition:   return "STATE";
            case EventCategory::TransitionRejected:return "REJECTED";
            case EventCategory::SafetyAlert:       return "SAFETY";
            case EventCategory::Info:              return "INFO";
        }
        return "UNKNOWN";
    }

private:
    std::deque<TelemetryEntry>              mEntries;
    std::vector<fat_p::ScopedConnection>    mConnections;

    void append(EventCategory category, std::string subject, std::string detail)
    {
        if (mEntries.size() >= kMaxEntries)
        {
            mEntries.pop_front(); // evict oldest
        }
        mEntries.push_back({std::chrono::steady_clock::now(),
                            category,
                            std::move(subject),
                            std::move(detail)});
    }
};

} // namespace drone
