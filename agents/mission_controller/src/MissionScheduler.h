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
 * @brief Mission type: USER (user-initiated) vs AUTONOMOUS (system-initiated)
 */
enum class MissionType {
    USER,
    AUTONOMOUS
};

/**
 * @brief Information about a mission in the scheduler
 */
struct MissionInfo {
    uint64_t mission_id;
    int priority;           // 1-5, where 5=Critical (highest) and 1=VeryLow (lowest)
    MissionType type;       // USER or AUTONOMOUS
    std::string status;     // "pending", "running", "stopped", "completed"
    
    MissionInfo() : mission_id(0), priority(3), type(MissionType::AUTONOMOUS), status("pending") {}
    
    MissionInfo(uint64_t id, int pr, MissionType t, const std::string& st = "pending")
        : mission_id(id), priority(pr), type(t), status(st) {}
};

/**
 * @brief MissionScheduler manages mission priorities and scheduling logic
 * 
 * Features:
 * - Priority-based mission queue (5=Critical/highest, 1=VeryLow/lowest)
 * - Support for USER and AUTONOMOUS missions
 * - Preemption logic: USER missions can preempt AUTONOMOUS missions of lower priority
 * - Automatic selection of next mission to run
 */
class MissionScheduler {
private:
    std::map<uint64_t, MissionInfo> missions;  // mission_id -> MissionInfo
    uint64_t current_mission_id = 0;           // Currently executing mission
    
public:
    MissionScheduler() = default;
    ~MissionScheduler() = default;
    
    /**
     * @brief Add or update a mission in the scheduler
     * @param mission_id Unique mission identifier
     * @param priority Priority level (1-5, where 5=Critical is highest)
     * @param type Mission type (USER or AUTONOMOUS)
     */
    void addMission(uint64_t mission_id, int priority, MissionType type) {
        // Clamp priority to [1, 5]
        priority = std::max(1, std::min(5, priority));
        missions[mission_id] = MissionInfo(mission_id, priority, type);
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
        
        if (it_candidate == missions.end() || it_current == missions.end()) {
            return false;
        }
        
        const auto& candidate = it_candidate->second;
        const auto& current = it_current->second;
        
        // USER missions preempt AUTONOMOUS missions if higher priority (larger number)
        if (candidate.type == MissionType::USER && current.type == MissionType::AUTONOMOUS) {
            return candidate.priority > current.priority;
        }
        
        // USER missions preempt other USER missions if higher priority (larger number)
        if (candidate.type == MissionType::USER && current.type == MissionType::USER) {
            return candidate.priority > current.priority;
        }
        
        // AUTONOMOUS missions preempt other AUTONOMOUS if higher priority (larger number)
        if (candidate.type == MissionType::AUTONOMOUS && current.type == MissionType::AUTONOMOUS) {
            return candidate.priority > current.priority;
        }
        
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
        
        for (const auto& [mission_id, info] : missions) {
            // Look for PENDING or STOPPED missions (not RUNNING or COMPLETED)
            if ((info.status == "pending" || info.status == "stopped") && 
                info.priority > best_priority) { 
                best_mission_id = mission_id;
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
            return false;
        }
        
        it->second.status = "completed";
        
        if (current_mission_id == mission_id) {
            current_mission_id = 0;
        }
        
        return true;
    }
};

#endif
