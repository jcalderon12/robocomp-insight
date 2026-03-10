#include "dsr_episodic_api.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace DSR {

    // ============================================================
    // Constructor / Status
    // ============================================================

    EpisodicMemoryAPI::EpisodicMemoryAPI(const std::string &filepath)
        : filepath_(filepath) {
        ready_ = index_file_();
        if (!ready_)
        std::cerr << "[EpisodicMemoryAPI] Failed to index file: " << filepath << "\n";
    }

    bool EpisodicMemoryAPI::is_ready() const { return ready_; }

    const std::string &EpisodicMemoryAPI::get_filepath() const { return filepath_; }

    // ============================================================
    // Keyframe queries
    // ============================================================

    size_t EpisodicMemoryAPI::get_keyframe_count() const { return keyframe_meta_.size(); }

    std::vector<uint64_t> EpisodicMemoryAPI::get_keyframe_timestamps() const {
        std::vector<uint64_t> ts;
        ts.reserve(keyframe_meta_.size());
        for (const auto &m : keyframe_meta_)
            ts.push_back(m.timestamp);
        return ts;
    }

    std::optional<uint64_t>
    EpisodicMemoryAPI::get_keyframe_timestamp(size_t idx) const {
        if (idx >= keyframe_meta_.size())
            return std::nullopt;
        return keyframe_meta_[idx].timestamp;
    }

    std::optional<DSRData> EpisodicMemoryAPI::get_keyframe(size_t idx) {
        if (idx >= keyframe_meta_.size())
            return std::nullopt;
        const DSREvent *ev = get_event_(keyframe_meta_[idx]);
        if (!ev)
            return std::nullopt;
        return to_dsr_data_(*ev);
    }

    std::optional<DSRData>
    EpisodicMemoryAPI::get_keyframe_at_time(uint64_t timestamp) {
        if (keyframe_meta_.empty())
            return std::nullopt;
        return get_keyframe(find_keyframe_index_at_time_(timestamp));
    }

    std::optional<size_t>
    EpisodicMemoryAPI::get_keyframe_index_at_time(uint64_t timestamp) {
        if (keyframe_meta_.empty())
            return std::nullopt;
        return find_keyframe_index_at_time_(timestamp);
    }

    std::optional<size_t>
    EpisodicMemoryAPI::get_event_index_at_timestamp(uint64_t timestamp) {
        if (all_events_meta_.empty())
            return std::nullopt;
        auto it = std::lower_bound(
            all_events_meta_.begin(), all_events_meta_.end(), timestamp,
            [](const EventMeta &meta, uint64_t ts) { return meta.timestamp < ts; }
        );
        if (it == all_events_meta_.end() || it->timestamp != timestamp)
            return std::nullopt;
        return std::distance(all_events_meta_.begin(), it);
    }

    std::optional<DSRData> EpisodicMemoryAPI::get_event_at_timestamp(uint64_t timestamp) {
        // Binary search for exact timestamp match
        auto it = std::lower_bound(
            all_events_meta_.begin(), all_events_meta_.end(), timestamp,
            [](const EventMeta &meta, uint64_t ts) { return meta.timestamp < ts; }
        );

        if (it == all_events_meta_.end() || it->timestamp != timestamp)
            return std::nullopt;

        const DSREvent *ev = get_event_(*it);
        if (!ev)
            return std::nullopt;
        return to_dsr_data_(*ev);
    }

    // ============================================================
    // Local-change queries
    // ============================================================

    size_t EpisodicMemoryAPI::get_local_changes_count(size_t keyframe_idx) const {
        auto it = local_changes_meta_.find(keyframe_idx);
        return (it != local_changes_meta_.end()) ? it->second.size() : 0;
    }

    std::vector<DSRData> EpisodicMemoryAPI::get_local_changes(size_t keyframe_idx) {
        std::vector<DSRData> result;
        auto it = local_changes_meta_.find(keyframe_idx);
        if (it == local_changes_meta_.end())
            return result;

        result.reserve(it->second.size());
        for (const auto &meta : it->second) {
            const DSREvent *ev = get_event_(meta);
            if (ev)
            result.push_back(to_dsr_data_(*ev));
        }
        return result;
    }

    std::optional<DSRData> EpisodicMemoryAPI::get_local_change(size_t keyframe_idx, size_t change_idx) {
        auto it = local_changes_meta_.find(keyframe_idx);
        if (it == local_changes_meta_.end() || change_idx >= it->second.size())
            return std::nullopt;
        const DSREvent *ev = get_event_(it->second[change_idx]);
        if (!ev)
            return std::nullopt;
        return to_dsr_data_(*ev);
    }

    // ============================================================
    // Time-range queries
    // ============================================================

    std::vector<DSRData> EpisodicMemoryAPI::get_events_between(uint64_t t_start, uint64_t t_end) {
        std::vector<DSRData> result;
        if (all_events_meta_.empty())
            return result;

        size_t start_idx = find_first_event_by_time_(t_start);
        size_t end_idx = find_last_event_by_time_(t_end);

        if (start_idx >= all_events_meta_.size())
            return result;

        for (size_t i = start_idx; i <= end_idx && i < all_events_meta_.size(); ++i) {
            const auto &meta = all_events_meta_[i];
            const DSREvent *ev = get_event_(meta);
            if (ev)
                result.push_back(to_dsr_data_(*ev));
        }
        return result;
    }

    std::vector<DSRData> EpisodicMemoryAPI::get_changes_between(uint64_t t_start, uint64_t t_end) {
        std::vector<DSRData> result;
        if (all_events_meta_.empty())
            return result;

        size_t start_idx = find_first_event_by_time_(t_start);
        size_t end_idx = find_last_event_by_time_(t_end);

        if (start_idx >= all_events_meta_.size())
            return result;

        for (size_t i = start_idx; i <= end_idx && i < all_events_meta_.size(); ++i) {
            const auto &meta = all_events_meta_[i];
            if (meta.modification_type == DSRSpecialChars::K)
                continue;
            const DSREvent *ev = get_event_(meta);
            if (ev)
                result.push_back(to_dsr_data_(*ev));
        }
        return result;
    }

    // ============================================================
    // Node-centric queries
    // ============================================================

    std::vector<DSRData> EpisodicMemoryAPI::get_node_history(uint64_t node_id) {
        std::vector<DSRData> result;
        for (const auto &meta : all_events_meta_) {
            const std::string &t = meta.modification_type;
            if (t != DSRSpecialChars::MN && t != DSRSpecialChars::MNA &&
                t != DSRSpecialChars::DN)
            continue;
            const DSREvent *ev = get_event_(meta);
            if (!ev)
            continue;
            if (ev->node_id.has_value() && ev->node_id.value() == node_id)
            result.push_back(to_dsr_data_(*ev));
        }
        return result;
    }

    std::vector<DSRData>
    EpisodicMemoryAPI::get_node_history_by_name(const std::string &node_name) {
        // Fast lookup using pre-built index (built during index_file_)
        auto it = node_name_index_.find(node_name);
        if (it == node_name_index_.end())
            return {};

        // Collect all events for the IDs associated with this name
        std::vector<DSRData> result;
        for (uint64_t id : it->second) {
            auto partial = get_node_history(id);
            result.insert(result.end(), partial.begin(), partial.end());
        }

        // Re-sort by timestamp to maintain order
        std::sort(result.begin(), result.end(),
                    [](const DSRData &a, const DSRData &b) {
                    return a.timestamp < b.timestamp;
                    });
        return result;
    }

    // ============================================================
    // Edge-centric queries
    // ============================================================

    std::vector<DSRData>
    EpisodicMemoryAPI::get_edge_history(uint64_t from_id, uint64_t to_id, const std::string &edge_type) {
        std::vector<DSRData> result;
        for (const auto &meta : all_events_meta_) {
            const std::string &t = meta.modification_type;
            if (t != DSRSpecialChars::ME && t != DSRSpecialChars::MEA &&
                t != DSRSpecialChars::DE)
            continue;
            const DSREvent *ev = get_event_(meta);
            if (!ev)
            continue;
            if (ev->edge_from_id.value_or(0) == from_id &&
                ev->edge_to_id.value_or(0) == to_id &&
                ev->type.value_or("") == edge_type) {
            result.push_back(to_dsr_data_(*ev));
            }
        }
        return result;
    }

    std::vector<DSRData> EpisodicMemoryAPI::get_node_changes_between(uint64_t node_id,
                                                                       uint64_t t_start,
                                                                       uint64_t t_end) {
        std::vector<DSRData> result;
        if (all_events_meta_.empty())
            return result;

        size_t start_idx = find_first_event_by_time_(t_start);
        size_t end_idx = find_last_event_by_time_(t_end);

        if (start_idx >= all_events_meta_.size())
            return result;

        for (size_t i = start_idx; i <= end_idx && i < all_events_meta_.size(); ++i) {
            const auto &meta = all_events_meta_[i];
            const std::string &t = meta.modification_type;

            // Only include local changes (not keyframes) that reference this node
            if (t != DSRSpecialChars::MN && t != DSRSpecialChars::MNA &&
                t != DSRSpecialChars::DN)
                continue;

            const DSREvent *ev = get_event_(meta);
            if (!ev || !ev->node_id.has_value() || ev->node_id.value() != node_id)
                continue;

            result.push_back(to_dsr_data_(*ev));
        }
        return result;
    }

    // ============================================================
    // Type-filtering query
    // ============================================================

    std::vector<DSRData>
    EpisodicMemoryAPI::get_events_by_type(const std::string &mod_type) {
        std::vector<DSRData> result;
        for (const auto &meta : all_events_meta_) {
            if (meta.modification_type != mod_type)
            continue;
            const DSREvent *ev = get_event_(meta);
            if (ev)
            result.push_back(to_dsr_data_(*ev));
        }
        return result;
    }

    // ============================================================
    // Private helpers
    // ============================================================

    bool EpisodicMemoryAPI::index_file_() {
        std::ifstream file(filepath_);
        if (!file.is_open()) { std::cerr << "[EpisodicMemoryAPI] ERROR: Cannot open file: " << filepath_ << "\n"; return false; }

        keyframe_meta_.clear();
        local_changes_meta_.clear();
        all_events_meta_.clear();
        node_name_index_.clear();

        std::string line;
        size_t current_kf = 0;
        size_t line_count = 0;

        while (std::getline(file, line)) {
            if (line.empty()) continue;
            line_count++;

            std::streampos end_pos = file.tellg();
            std::streampos pos = (end_pos == std::streampos(-1)) 
                    ? static_cast<std::streampos>(0) : end_pos - static_cast<std::streamoff>(line.size() + 1);

            auto [ts, type] = parse_header_(line);
            EventMeta meta{ts, pos, line.size(), type, current_kf};

            if (type == DSRSpecialChars::K) {
                keyframe_meta_.push_back(meta);
                current_kf = keyframe_meta_.size() - 1;
                meta.keyframe_index = current_kf;
            } else { 
                local_changes_meta_[current_kf].push_back(meta);
                
                // Build name index for MN events (node modifications)
                if (type == DSRSpecialChars::MN) {
                    const DSREvent *ev = get_event_(meta);
                    if (ev && ev->node_name.has_value() && ev->node_id.has_value()) {
                        const auto &name = ev->node_name.value();
                        const auto id = ev->node_id.value();
                        auto &ids = node_name_index_[name];
                        if (ids.empty() || ids.back() != id) {
                            ids.push_back(id);
                        }
                    }
                }
            }
            all_events_meta_.push_back(meta);
        }

        if (keyframe_meta_.empty()) {
            std::cerr << "[EpisodicMemoryAPI] ERROR: No keyframes found in file. File may be empty or malformed.\n";
            return false;
        }

        std::cout << "[EpisodicMemoryAPI] Indexed successfully: "
                    << keyframe_meta_.size() << " keyframes, "
                    << (all_events_meta_.size() - keyframe_meta_.size()) << " local changes, "
                    << line_count << " total lines, "
                    << node_name_index_.size() << " unique node names from "
                    << filepath_ << "\n";
        return true;
    }

    std::string EpisodicMemoryAPI::read_line_at_(std::streampos pos) const {
        std::ifstream file(filepath_);
        if (!file.is_open())
            return {};
        file.seekg(pos);
        std::string line;
        std::getline(file, line);
        return line;
    }

    const DSREvent *EpisodicMemoryAPI::get_event_(const EventMeta &meta) {
        // Cache hit
        auto it = cache_.find(meta.timestamp);
        if (it != cache_.end()) {
            touch_lru_(meta.timestamp);
            return it->second.get();
        }

        // Decode from file
        std::string line = read_line_at_(meta.file_position);
        if (line.empty()) {
            std::cerr << "[EpisodicMemoryAPI] Failed to read line at position "
                    << meta.file_position << "\n";
            return nullptr;
        }

        auto decoded = DSRDecoder::decode(line);
        if (!decoded) {
            std::cerr << "[EpisodicMemoryAPI] Failed to decode event at ts="
                    << meta.timestamp << "\n";
            return nullptr;
        }

        if (cache_.size() >= CACHE_SIZE)
            evict_lru_();

        DSREvent *ptr = decoded.get();
        cache_[meta.timestamp] = std::move(decoded);
        cache_order_.push_back(meta.timestamp);
        return ptr;
    }

    void EpisodicMemoryAPI::evict_lru_() {
        if (cache_order_.empty())
            return;
        cache_.erase(cache_order_.front());
        cache_order_.pop_front();
    }

    void EpisodicMemoryAPI::touch_lru_(uint64_t timestamp) {
        auto it = std::find(cache_order_.begin(), cache_order_.end(), timestamp);
        if (it != cache_order_.end())
            cache_order_.erase(it);
        cache_order_.push_back(timestamp);
        }

        DSRData EpisodicMemoryAPI::to_dsr_data_(const DSREvent &ev) {
        DSRData d;
        d.timestamp = ev.timestamp;
        d.modification_type = ev.modification_type;
        d.node_id = ev.node_id;
        d.edge_from_id = ev.edge_from_id;
        d.edge_to_id = ev.edge_to_id;
        d.type = ev.type;
        d.node_name = ev.node_name;
        d.attributes = ev.attributes;
        d.nodes = ev.nodes;
        d.edges = ev.edges;
        return d;
    }

    std::pair<uint64_t, std::string> EpisodicMemoryAPI::parse_header_(const std::string &line) {
        size_t first = line.find(DSRSpecialChars::SLOT);
        size_t second = line.find(DSRSpecialChars::SLOT, first + 1);

        if (first == std::string::npos || second == std::string::npos)
            throw std::runtime_error("[EpisodicMemoryAPI] Invalid line format: " +
                                    line.substr(0, 80));

        uint64_t ts = std::stoull(line.substr(0, first));
        std::string type = line.substr(first + 1, second - first - 1);
        return {ts, type};
    }

    size_t EpisodicMemoryAPI::find_keyframe_index_at_time_(uint64_t ts) const {
        // Binary search: last keyframe with timestamp <= ts.
        size_t lo = 0, hi = keyframe_meta_.size();
        while (lo < hi) {
            size_t mid = lo + (hi - lo) / 2;
            if (keyframe_meta_[mid].timestamp <= ts)
            lo = mid + 1;
            else
            hi = mid;
        }
        return (lo > 0) ? lo - 1 : 0;
    }

    size_t EpisodicMemoryAPI::find_first_event_by_time_(uint64_t t_start) const {
        // Binary search: first event with timestamp >= t_start
        auto it = std::lower_bound(
            all_events_meta_.begin(), all_events_meta_.end(), t_start,
            [](const EventMeta &meta, uint64_t ts) { return meta.timestamp < ts; }
        );
        return std::distance(all_events_meta_.begin(), it);
    }

    size_t EpisodicMemoryAPI::find_last_event_by_time_(uint64_t t_end) const {
        // Binary search: last event with timestamp <= t_end
        auto it = std::upper_bound(
            all_events_meta_.begin(), all_events_meta_.end(), t_end,
            [](uint64_t ts, const EventMeta &meta) { return ts < meta.timestamp; }
        );
        // upper_bound returns iterator to first element > t_end, so we go back
        return (it == all_events_meta_.begin()) ? 0 : std::distance(all_events_meta_.begin(), it) - 1;
    }

} // namespace DSR
