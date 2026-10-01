#ifndef MISSION_SCHEDULER_H
#define MISSION_SCHEDULER_H

#include <cstdint>
#include <map>
#include <queue>
#include <optional>
#include <string>
#include <memory>
#include <algorithm>
#include <functional>
#include <chrono>

/**
 * @brief Mission priority queue comparator. Higher priority missions come first.
 */
struct MissionComparator {
    bool operator()(const std::pair<uint64_t, int>& a, const std::pair<uint64_t, int>& b) const {
        // Higher priority values = higher priority (5=Critical is max, 1=VeryLow is min)
        // So we want LARGER values to come FIRST (higher priority)
        return a.second < b.second;  // Reverse: larger values have higher priority
    }
};

/**
 * @brief Control type: USER (user-initiated) vs AUTONOMOUS (system-initiated)
 */
enum class ControlType {
    USER,
    AUTONOMOUS
};

/**
 * @brief Information about a mission in the scheduler
 */
struct MissionInfo {
    uint64_t mission_id;
    int priority;           // 1-5, where 5=Critical (highest) and 1=VeryLow (lowest)
    ControlType control_type;       // USER or AUTONOMOUS
    std::string status;     // "pending", "running", "stopped", "completed"
    
    MissionInfo() : mission_id(0), priority(3), control_type(ControlType::AUTONOMOUS), status("pending") {}
    
    MissionInfo(uint64_t id, int pr, ControlType t, const std::string& st = "pending")
        : mission_id(id), priority(pr), control_type(t), status(st) {}
};

/**
 * @brief Execution events that SpecificWorker can handle
 */
enum class ExecutionEvent {
    MISSION_SELECTED,      // A new mission was selected for activation
    MISSION_ACTIVATED,     // Mission activated, waiting for affordance
    MISSION_RUNNING,       // Mission is running, need to monitor
    MISSION_COMPLETED,     // Mission marked complete, cleanup needed
    FALLBACK_CREATED,      // Fallback follow_person mission created
    HANDSHAKE_TIMEOUT      // Handshake with episodic_memory timed out
};

/**
 * @brief Event callback data
 */
struct ExecutionEventData {
    ExecutionEvent event;
    uint64_t mission_id;
    std::string mission_type;
};

/**
 * @brief MissionScheduler manages mission priorities and scheduling logic integrated with autopilot
 * 
 * Features:
 * - Priority-based mission queue (5=Critical/highest, 1=VeryLow/lowest)
 * - Support for USER and AUTONOMOUS missions
 * - Preemption logic: USER missions can preempt AUTONOMOUS missions of lower priority
 * - Built-in orchestration: state machine drives mission lifecycle internally
 * - Callback mechanism for SpecificWorker to handle DSR operations
 * 
 * Usage:
 * 1. Register callbacks for events
 * 2. Call executionStep() from compute()
 * 3. Scheduler handles all state transitions internally
 */
class MissionScheduler {
public:
    /**
     * @brief Internal execution state machine (not exposed to users)
     */
    enum class InternalState {
        IDLE,                   // No mission active, waiting for next
        SELECTING_NEXT,         // Searching for next pending mission
        WAITING_AFFORDANCE,     // Waiting for affordance/handshake
        RUNNING,                // Mission actively running
        COMPLETED               // Mission finished, about to cleanup
    };

private:
    std::map<uint64_t, MissionInfo> missions;  // mission_id -> MissionInfo
    uint64_t current_mission_id = 0;           // Currently executing mission
    
    // === INTERNAL STATE MACHINE  ===
    InternalState internal_state = InternalState::IDLE;
    uint64_t active_mission_id = 0;            // Mission in current execution cycle
    std::string active_mission_semantic_type = "";      // Mission type for callbacks
    int idle_cycles_count = 0;                 // Cycles with no pending missions
    static constexpr int MAX_IDLE_CYCLES = 20;  // 20 cycles * 100ms = 2 seconds
    
    // Handshake tracking
    uint64_t handshake_mission_id = 0;
    int handshake_timeout_cycles = 0;
    static constexpr int HANDSHAKE_TIMEOUT_CYCLES = 150;  // Sync with SpecificWorker.h (1.5s timeout)
    int handshake_retry_count = 0;             // Track retry attempts (max 3)
    static constexpr int MAX_HANDSHAKE_RETRIES = 3;
    
    bool was_affordance_ready = false;         // Track affordance state across cycles
    bool disable_requested = false;            // Preserve external disable requests across callbacks
    
    // === CALLBACKS (for SpecificWorker to implement DSR operations) ===
    std::function<void(const ExecutionEventData&)> event_callback;
    
public:
    MissionScheduler() = default;
    ~MissionScheduler() = default;
    
    // === SETUP AND CONFIGURATION ===
    
    /**
     * @brief Register an event callback for execution events
     * @param callback Function to call when execution events occur
     */
    void setEventCallback(std::function<void(const ExecutionEventData&)> callback) {
        event_callback = callback;
    }
    
    /**
     * @brief Enable or disable autopilot orchestration
     * @param enabled If true, scheduler manages mission lifecycle automatically
     */
    void setAutopilotEnabled(bool enabled) {
        if (enabled && internal_state == InternalState::IDLE) {
            // Initialize state machine on enabling
            disable_requested = false;
            internal_state = InternalState::SELECTING_NEXT;
        } else if (!enabled) {
            // Reset on disabling
            disable_requested = true;
            internal_state = InternalState::IDLE;
            active_mission_id = 0;
            active_mission_semantic_type.clear();
            idle_cycles_count = 0;
            handshake_mission_id = 0;
            handshake_timeout_cycles = 0;
            handshake_retry_count = 0;
            was_affordance_ready = false;
        }
    }
    
    bool isAutopilotEnabled() const {
        return internal_state != InternalState::IDLE || active_mission_id != 0;
    }
    
    // === MISSION MANAGEMENT (same as before) ===
    
    /**
     * @brief Add or update a mission in the scheduler
     * @param mission_id Unique mission identifier
     * @param priority Priority level (1-5, where 5=Critical is highest)
     * @param type Mission type (USER or AUTONOMOUS)
     */
    void addMission(uint64_t mission_id, int priority, ControlType control_type) {
        // Clamp priority to [1, 5]
        priority = std::max(1, std::min(5, priority));
        missions[mission_id] = MissionInfo(mission_id, priority, control_type);
    }
    
    /**
     * @brief Update the status of a mission
     * @param mission_id Mission to update
     * @param status New status ("pending", "running", "stopped", "completed")
     */
    void updateMissionStatus(uint64_t mission_id, const std::string& status) {
        auto it = missions.find(mission_id);
        if (it != missions.end()) {
            it->second.status = status;
        }
    }
    
    /**
     * @brief Update the priority of a mission
     * @param mission_id Mission to update
     * @param priority New priority level (1-5, where 5=Critical is highest)
     */
    void updateMissionPriority(uint64_t mission_id, int priority) {
        auto it = missions.find(mission_id);
        if (it != missions.end()) {
            priority = std::max(1, std::min(5, priority));
            it->second.priority = priority;
        }
    }
    
    /**
     * @brief Set the currently executing mission
     * @param mission_id Mission that is now running
     */
    void setCurrentMission(uint64_t mission_id) {
        current_mission_id = mission_id;
    }
    
    /**
     * @brief Get the currently executing mission
     * @return Mission ID of currently executing mission, or 0 if none
     */
    uint64_t getCurrentMission() const {
        return current_mission_id;
    }
    
    /**
     * @brief Check if a mission should preempt the currently running mission
     * @param mission_id Candidate mission to check
     * @return true if this mission should preempt the current one
     * 
     * Priority rules (higher number = higher priority):
     * - USER missions preempt ANY mission (USER or AUTONOMOUS) if higher priority (larger number)
     * - AUTONOMOUS missions preempt AUTONOMOUS if higher priority
     * - AUTONOMOUS missions do NOT preempt USER missions
     */
    bool shouldPreempt(uint64_t mission_id) const {
        auto it_candidate = missions.find(mission_id);
        auto it_current = missions.find(current_mission_id);
        
        if (it_candidate == missions.end() || it_current == missions.end()) return false;
        
        const auto& candidate = it_candidate->second;
        const auto& current = it_current->second;
        
        // USER missions preempt AUTONOMOUS missions if higher priority (larger number)
        if (candidate.control_type == ControlType::USER && current.control_type == ControlType::AUTONOMOUS) return candidate.priority > current.priority;
        
        // USER missions preempt other USER missions if higher priority (larger number)
        if (candidate.control_type == ControlType::USER && current.control_type == ControlType::USER) return candidate.priority > current.priority;
        
        // AUTONOMOUS missions preempt other AUTONOMOUS if higher priority (larger number)
        if (candidate.control_type == ControlType::AUTONOMOUS && current.control_type == ControlType::AUTONOMOUS) return candidate.priority > current.priority;
        
        // AUTONOMOUS missions do NOT preempt USER missions
        return false;
    }
    
    /**
     * @brief Get the next mission to run based on priorities
     * @return Optional mission_id of the highest priority pending/stopped mission, nullopt if none
     */
    std::optional<uint64_t> selectNextMission() const {
        uint64_t best_mission_id = 0;
        int best_priority = 0;  // Start at minimum priority (will find highest)
        
        for (const auto& [id, info] : missions) {
            // Look for PENDING or STOPPED missions (not RUNNING or COMPLETED)
            if ((info.status == "pending" || info.status == "stopped") && 
                info.priority > best_priority) { 
                best_mission_id = id;
                best_priority = info.priority;
            }
        }
        
        if (best_mission_id != 0) {
            return best_mission_id;
        }
        return std::nullopt;
    }
    
    /**
     * @brief Remove a mission from the scheduler
     * @param mission_id Mission to remove
     */
    void removeMission(uint64_t mission_id) {
        missions.erase(mission_id);
        if (current_mission_id == mission_id) {
            current_mission_id = 0;
        }
    }
    
    /**
     * @brief Get mission info
     * @param mission_id Mission to query
     * @return Optional MissionInfo if mission exists
     */
    std::optional<MissionInfo> getMissionInfo(uint64_t mission_id) const {
        auto it = missions.find(mission_id);
        if (it != missions.end()) {
            return it->second;
        }
        return std::nullopt;
    }
    
    /**
     * @brief Get total number of missions in scheduler
     */
    size_t getTotalMissions() const {
        return missions.size();
    }
    
    /**
     * @brief Get number of pending missions (waiting to run)
     */
    size_t getPendingMissionCount() const {
        size_t count = 0;
        for (const auto& [_, info] : missions) {
            if (info.status == "pending") {
                count++;
            }
        }
        return count;
    }
    
    /**
     * @brief Result of attempting to activate a mission
     * Contains information about preemption that occurred
     */
    struct ActivationResult {
        bool success = false;
        bool preempted_existing = false;
        uint64_t preempted_mission_id = 0;
        std::string previous_status;
    };
    
    /**
     * @brief Activate a mission, handling preemption automatically
     * 
     * This method:
     * 1. Checks if a mission currently running should be preempted
     * 2. Updates its own internal state
     * 3. Returns result with preemption info for SpecificWorker to handle
     * 
     * @param mission_id Mission to activate
     * @return ActivationResult with preemption details
     */
    ActivationResult activateMission(uint64_t mission_id) {
        ActivationResult result;
        
        auto it_candidate = missions.find(mission_id);
        if (it_candidate == missions.end()) {
            return result;  // Mission not found
        }
        
        result.success = true;
        
        // Check if current mission should be preempted
        if (current_mission_id != 0 && shouldPreempt(mission_id)) {
            auto it_current = missions.find(current_mission_id);
            if (it_current != missions.end()) {
                result.preempted_existing = true;
                result.preempted_mission_id = current_mission_id;
                result.previous_status = it_current->second.status;
                
                // Update preempted mission status
                missions[current_mission_id].status = "stopped";
            }
        }
        
        // Activate new mission
        missions[mission_id].status = "running";
        current_mission_id = mission_id;
        
        return result;
    }
    
    /**
     * @brief Stop a mission gracefully
     * 
     * @param mission_id Mission to stop
     * @return true if mission was found and stopped
     */
    bool stopMission(uint64_t mission_id) {
        auto it = missions.find(mission_id);
        if (it == missions.end()) {
            return false;
        }
        
        it->second.status = "stopped";
        
        if (current_mission_id == mission_id) {
            current_mission_id = 0;
            active_mission_id = 0;
            active_mission_semantic_type.clear();
            handshake_mission_id = 0;
            handshake_timeout_cycles = 0;
            handshake_retry_count = 0;
            was_affordance_ready = false;
            internal_state = InternalState::SELECTING_NEXT;
        }
        
        return true;
    }
    
    /**
     * @brief Complete a mission (mark as done)
     * 
     * @param mission_id Mission to complete
     * @return true if mission was found and completed
     */
    bool completeMission(uint64_t mission_id) {
        auto it = missions.find(mission_id);
        if (it == missions.end()) {
            std::cerr << "[SCHEDULER_ERROR] completeMission called but mission_id=" << mission_id << " not found!" << std::endl;
            return false;
        }
        

        it->second.status = "completed";
        
        if (current_mission_id == mission_id) {
            current_mission_id = 0;
        }
        
        // If this is the mission in handshake, reset handshake state
        if (mission_id == handshake_mission_id) {
            handshake_mission_id = 0;
            handshake_timeout_cycles = 0;
            handshake_retry_count = 0;
        }
        
        // If this mission is currently active, transition to COMPLETED state
        if (mission_id == active_mission_id) {
            internal_state = InternalState::COMPLETED;
        }
        
        return true;
    }
    
    // === ORCHESTRATION ENGINE (Core refactored autopilot logic) ===
    
    /**
     * @brief Main execution step - run this from compute() loop
     * 
     * This orchestrates the entire mission lifecycle. Call it every compute cycle when autopilot is enabled.
     * Events are fired via callback for SpecificWorker to handle DSR operations.
     */
    void executionStep() {
        // Step 0: Check handshake timeout
        if (handshake_mission_id != 0) {
            handshake_timeout_cycles++;
            if (handshake_timeout_cycles == 1) {
                // Log on first cycle for this handshake
            }
            if (handshake_timeout_cycles > HANDSHAKE_TIMEOUT_CYCLES) {
                handshake_retry_count++;
                std::cout << "[SCHEDULER] Handshake timeout attempt " << handshake_retry_count << "/" << MAX_HANDSHAKE_RETRIES << " for mission: " << handshake_mission_id << std::endl;
                
                if (handshake_retry_count < MAX_HANDSHAKE_RETRIES) {
                    // Retry: reset cycle counter and try again
                    std::cout << "[SCHEDULER] Retrying handshake..." << std::endl;
                    handshake_timeout_cycles = 0;
                } else {
                    // All retries failed - remove mission completely and go back to selecting
                    std::cout << "[SCHEDULER] Handshake failed after " << MAX_HANDSHAKE_RETRIES << " attempts. Removing mission from scheduler." << std::endl;
                    removeMission(handshake_mission_id);
                    if (event_callback) event_callback({ ExecutionEvent::HANDSHAKE_TIMEOUT, handshake_mission_id, active_mission_semantic_type });
                    handshake_mission_id = 0;
                    handshake_retry_count = 0;
                    handshake_timeout_cycles = 0;
                    // Return to SELECTING_NEXT to find another mission or go idle
                    internal_state = InternalState::SELECTING_NEXT;
                }
            }
        }
        
        switch (internal_state) {
            case InternalState::IDLE:
                // Nothing to do
                break;
                
            case InternalState::SELECTING_NEXT:
                executeSelectingNext();
                break;
                
            case InternalState::WAITING_AFFORDANCE:
                executeWaitingAffordance();
                break;
                
            case InternalState::RUNNING:
                executeRunning();
                break;
                
            case InternalState::COMPLETED:
                executeCompleted();
                break;
        }
    }
    
    /**
     * @brief Check if mission has affordance ready (should be called from SpecificWorker)
     * @param mission_id The mission to check
     * @return true if affordance is ready
     */
    void setAffordanceReady(uint64_t mission_id, bool ready) {
        if (mission_id == handshake_mission_id) {
            was_affordance_ready = ready;
            if (ready) {
                handshake_mission_id = 0;  // Handshake complete
                handshake_timeout_cycles = 0;
                handshake_retry_count = 0;  // Reset retry counter on success
            }
        }
    }
    
    /**
     * @brief Signal that mission should be marked complete (can be called from UI or monitors)
     * @param mission_id Mission to complete
     */
    void requestMissionCompletion(uint64_t mission_id) {
        if (mission_id == active_mission_id) {
            completeMission(mission_id);
            internal_state = InternalState::COMPLETED;
        }
    }
    
    /**
     * @brief Get currently active mission ID
     */
    uint64_t getActiveMissionId() const {
        return active_mission_id;
    }
    
    /**
     * @brief Get currently active mission type
     */
    std::string getActiveMissionSemanticType() const {
        return active_mission_semantic_type;
    }
    
    /**
     * @brief Get current internal execution state (for debugging)
     */
    InternalState getInternalState() const {
        return internal_state;
    }
    
    /**
     * @brief Check if currently waiting for EM handshake
     * @return true if waiting for handshake (SpecificWorker should poll check_affordance_and_complete_handshake)
     */
    bool isWaitingForHandshake() const {
        return handshake_mission_id != 0;
    }
    
private:
    // === INTERNAL STATE MACHINE METHODS (executed by executionStep) ===
    
    void executeSelectingNext() {
        // Check for pending missions
        auto next_opt = selectNextMission();
        
        if (!next_opt.has_value()) {
            // No pending missions - increment idle counter
            idle_cycles_count++;
            
            // Only log every 10 cycles to avoid spam
            if (idle_cycles_count % 10 == 0) {

            }
            
            if (idle_cycles_count > MAX_IDLE_CYCLES) {
                // No automatic fallback in the one-shot autopilot flow.
                // Keep waiting so a later mission insertion can be picked up.
                idle_cycles_count = 0;
            }
            return;
        }
        
        // Found a pending mission

        active_mission_id = next_opt.value();
        idle_cycles_count = 0;
        
        // Get mission type (callback should provide this info)
        auto info_opt = getMissionInfo(active_mission_id);
        if (!info_opt.has_value()) {
            active_mission_id = 0;
            return;
        }
        
        // For now, assume mission_type comes from external source
        // This will be set when mission is added or retrieved from model
        
        // Signal that mission was selected
        if (event_callback) {
            event_callback({ ExecutionEvent::MISSION_SELECTED, active_mission_id, active_mission_semantic_type });
        }
        
        // Activate the mission
        auto result = activateMission(active_mission_id);
        if (!result.success) {
            active_mission_id = 0;
            return;
        }
        
        // Signal activation and transition to waiting for affordance
        if (event_callback) {
            event_callback({ ExecutionEvent::MISSION_ACTIVATED, active_mission_id, active_mission_semantic_type });
        }
        
        handshake_mission_id = active_mission_id;
        handshake_timeout_cycles = 0;
        was_affordance_ready = false;
        
        internal_state = InternalState::WAITING_AFFORDANCE;
    }
    
    void executeWaitingAffordance() {
        // Check if affordance became ready (set via setAffordanceReady())
        if (was_affordance_ready || handshake_mission_id == 0) {
            // Handshake complete, transition to RUNNING
            was_affordance_ready = false;
            internal_state = InternalState::RUNNING;
            
            if (event_callback) {
                event_callback({ ExecutionEvent::MISSION_RUNNING, active_mission_id, active_mission_semantic_type });
            }
        }
    }
    
    void executeRunning() {
        // Mission is actively running
        // SpecificWorker should call monitor methods from here
        // No changes to state in this method - state changes via external requests (completion)
    }
    
    void executeCompleted() {
        // Mission completed, perform cleanup
        if (event_callback) {
            event_callback({ ExecutionEvent::MISSION_COMPLETED, active_mission_id, active_mission_semantic_type });
        }
        
        active_mission_id = 0;
        active_mission_semantic_type = "";
        was_affordance_ready = false;
        
        // Transition to selecting next mission unless an external disable was requested
        internal_state = disable_requested ? InternalState::IDLE : InternalState::SELECTING_NEXT;
        disable_requested = false;
    }
};

#endif
