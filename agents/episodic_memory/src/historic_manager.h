#ifndef DSRDECODER_H
#define DSRDECODER_H

#include "DSRDecoder.h"
#include "DSRTypeTrait.h"
#include <dsr/api/dsr_api.h>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <thread>
#include <atomic>
#include <deque>
#include <mutex>

/**
 * @brief Event meta data from a keyframe or modification
 */
struct EventMetaData {
    uint64_t timestamp;               // timestamp
    std::streampos file_position;     // position on file
    size_t line_lenght;               // line lenght
    std::string modification_type;    // K - MN, MNA, ME, MEA, DN, DE
    size_t keyframe_index;            // keyframe it belongs (if modification other than K)
};


/**
 * @brief Historic manager. Indexes the content of a DSR Event, stores data in a LRU cache and displays the graph.
 */
class HistoricManager {
public:
    HistoricManager(std::shared_ptr<DSR::DSRGraph> graph, size_t max_cache_size = 50) 
        : G(graph), max_cache_size(max_cache_size), preloading(false), current_keyframe_idx(0) {}

    ~HistoricManager() { if (preload_thread.joinable()) preload_thread.join(); }

    /**
     * @brief Indexes an event file, reading timestamps, event types and file positions.
     */
    bool index_file(const std::string &filepath) {
        this->filepath = filepath; 
        std::ifstream file(filepath);
        if (!file.is_open()) { std::cerr << __FUNCTION__ << " - Cannot open file " << filepath << std::endl; return false; }

        keyframe_metadata.clear();
        local_changes_metadata.clear();
        all_events_metadata.clear();

        std::string line;
        size_t current_keyframe = 0;
        size_t event_idx = 0;

        while (std::getline(file, line)) {
            if (line.empty()) continue;

            std::streampos pos = file.tellg();
            pos -= static_cast<std::streamoff>(line.size() + 1);

            auto [timestamp, type] = quick_parse_header(line);
            EventMetaData meta{timestamp, pos, line.size(), type, current_keyframe};

            if (type == DSRSpecialChars::K) {
                keyframe_metadata.push_back(meta);
                current_keyframe = keyframe_metadata.size() - 1;
                meta.keyframe_index = current_keyframe;
            } else {
                local_changes_metadata[current_keyframe].push_back(meta);
            }

            all_events_metadata.push_back(meta);
            event_idx++;
        }

        std::cout << __FUNCTION__ << "[HistoricManager] Indexed " << keyframe_metadata.size()
                                  << " keyframes with " << all_events_metadata.size()
                                  << " total events in " << filepath << std::endl;

        return true;
    }

private:
    std::shared_ptr<DSR::DSRGraph> G;
    std::string filepath;

    // Metadata
    std::vector<EventMetaData> keyframe_metadata;
    std::map<size_t, std::vector<EventMetaData>> local_changes_metadata;
    std::vector<EventMetaData> all_events_metadata;

    // LRU cache
    std::deque<uint64_t> cache_order; 
    size_t max_cache_size;
    std::mutex cache_mutex;

    // Pre-loading
    std::thread preload_thread;
    std::atomic<bool> preloading;
    size_t current_keyframe_idx;

    /**
     * @brief Header parser
     */
    std::pair<uint64_t, std::string> quick_parse_header(const std::string &line) {
        size_t first_hash = line.find(DSRSpecialChars::SLOT);
        size_t second_hash = line.find(DSRSpecialChars::SLOT, first_hash + 1);

        if (first_hash == std::string::npos || second_hash == std::string::npos)            
            throw std::runtime_error("Invalid line format");
        
        uint64_t ts = std::stoull(line.substr(0, first_hash));
        std::string type = line.substr(first_hash + 1, second_hash - first_hash - 1);
        
        return {ts, type};
    }

    /**
     * @brief Read a specific line from file
     */
    std::string read_line_at(std::streampos pos) {
        std::ifstream file(filepath);
        file.seekg(pos);
        std::string line;
        std::getline(file, line);
        return line;
    }

    /**
     * @brief Get a decoded event, from cache if posible
     */
    DSREvent* get_event_by_metadata(const EventMetaData &meta) {
        std::lock_guard<std::mutex>(cache_mutex);

        // Cache search
        auto it = event_cache.find(meta.timestamp);
        if (it != event_cache.end()) {
            update_lru(meta.timestamp);
            return it->second.get();
        }

        // Not in cache, decode from file
        std::string line = read_line_at(meta.file_position);
        auto decoded = DSRDecoder::decode(line);
        if (!decoded) { std::cerr << __FUNCTION__ << " - [HistoricManager] Failed to decoded event at timestamp " << meta.timestamp << std::endl; return nullptr; }

        // Add to cache
        DSREvent* ptr = decoded.get();
        event_cache[meta.timestamp] = std::move(decoded);
        cache_order.push_back(meta.timestamp);

        // Check cache size limits
        while (cache_order.size() > max_cache_size) {
            uint64_t oldest = cache_order.front();
            cache_order.pop_front();
            event_cache.erase(oldest);
        }

        return ptr;
    }

    /**
     * @brief Updates LRU order when cache is accessed
     */
    void update_lru(uint64_t timestamp) {
        // 
        auto it = std::find(cache_order.begin(), cache_order.end(), timestamp);
        if (it != cache_order.end())
            cache_order.erase(it);
        cache_order.push_back(timestamp);
    }

    /**
     * @brief 
     */
    void reconstruct_graph_from_keyframe(const DSREvent &keyframe) {
        if (keyframe.modification_type != DSRSpecialChars::K) { std::cerr << __FUNCTION__ << " - [HistoricManager] Event is not a keyframe." << std:.endl; return; }
        std::cout << __FUNCTION__ << " - [HistoricManager] Reconstructing graph from keyframe with " << keyframe.nodes.size() << " nodes and " << keyframe.edges.size() << " edges." << std::endl;
        // Clear graph
        clear_graph();
        // Insert nodes
        for (const auto &node : keyframe.nodes) {
            try {
                G->insert_node(node);
            } catch (const std::exception &e) { std::cerr << __FUNCTION__ << " - [HistoricManager] Error inserting node " << node.id() << ": " << e.what() << std::endl; }
        }
        // Insert edges
        for (const auto &edge : keyframe.edges) {
            try { 
                G->insert_or_assign_edge(edge);
            } catch(const std::exception &e) { std::cerr << __FUNCTION__ << " - [HistoricManager] Error inserting edge " << edge.from() << "->" << edge.to() << ": " << e.what() << std::endl; }
        }

        std::cout << __FUNCTION__ << " - [HistoricManager] Graph reconstructed successfully." << std::endl;
    }

    /**
     * @brief Apply one modification by its event type
     */
    void apply_modification_to_graph(const DSREvent &mod) {
        try {
            if (mod.modification_type == DSRSpecialChars::MN)
                apply_modify_node(mod);
            else if (mod.modification_type == DSRSpecialChars::MNA)
                apply_modify_node_attrs(mod);
            else if (mod.modification_type == DSRSpecialChars::ME)
                apply_modify_edge(mod);
            else if (mod.modification_type == DSRSpecialChars::MEA)
                apply_modify_edge_attrs(mod);
            else if (mod.modification_type == DSRSpecialChars::DN)
                apply_delete_node(mod);
            else if (mod.modification_type == DSRSpecialChars::DE)
                apply_delete_edge(mod);
        } catch(const std::exception &e) {
            std::cerr << __FUNCTION__ << " - []" << e.what() << '\n';
        }  
    }

    /**
     * @brief Apply MN - Modify Node
     */
    void apply_modify_node(const DSREvent &mod) {
        if (!mod.node_id.has_value() || !mod.type.has_value() || !mod.node_name.has_value()) { std::cerr << __FUNCTION__ << " - [HistoricManager] MN: Missing required fields" << std::endl; return; }
        
        // Check if node already exists
        auto node_optional = G->get_node(*mod.node_name);
        if (!node_optional.has_value()) {
            DSR::Node node;
            node.id(*mod.node_id);
            node.type(*mod.type);
            node.name(*mod.node_name);
            G->insert_node(node);
        } else { 
            auto node = node_optional.value();
            if (!(node.id == *mod.node_id && node.name == *mod.node_name && node.type == *mod.type)) {
                node.id(*mod.node_id);
                node.type(*mod.type);
                node.name(*mod.node_name);
            }
        }
    }

    /**
     * @brief Apply MNE - Modify Node Attributes
     */
    void apply_modify_node_attrs(const DSREvent &mod) {
        if (!mod.node_id.has_value()) { std::cerr << __FUNCTION__ << " - [HistoricManager] MNA: Missing required fields" << std::endl; return; }

        // Check if node exists
        auto node_optional = G->get_node(*mod.node_id);
        if (!node_optional.has_value()) { std::cerr << __FUNCTION__ << " - [HistoricManager] MNA: Node " << *mod.node_id << " not found" << std::endl; return; }

        for(const auto &[name, attr] : mod.attributes)
            if (name != DSRAttributeNames::ID)
                G->add_or_modify_attrib_local(node_optional.value(), name, attr);
        
        G->update_node(node_optional.value());
    }

    /**
     * @brief Apply ME - Modify Edge
     */
    void apply_modify_edge(const DSREvent &mod) {
        if (!mod.edge_from_id.has_value() || !mod.edge_to_id.has_value() || !mod.type.has_value()) { std::cerr << __FUNCTION__ << " - [HistoricManager] ME: Missing required fields" << std::endl; return; }
        
        // Check if edge already exists
        auto edge_optional = G->get_edge(*mod.edge_from_id, *mod.edge_to_id, *mod.type);
        if (!node_optional.has_value()) {
            DSR::Edge edge;
            edge.from(*mod.edge_from_id);
            edge.to(*mod.edge_to_id);
            edge.type(*mod.type);
            G->insert_or_assign_edge(edge);
        } else { 
            auto edge = edge_optional.value();
            if (!(edge.from == *mod.edge_from_id && edge.to == *mod.edge_to_id && edge.type == *mod.type)) {
                edge.from(*mod.edge_from_id);
                edge.to(*mod.edge_to_id);
                edge.type(*mod.type);
            }
        }
    }

    /**
     * @brief Apply MEA - Modify Edge Attributes
     */
    void apply_modify_edge_attrs(const DSREvent &mod) {
        if (!mod.edge_from_id.has_value() || !mod.edge_to_id.has_value() || !mod.type.has_value()) { std::cerr << __FUNCTION__ << " - [HistoricManager] MEA: Missing required fields" << std::endl; return; }

        // Check if edge exists
        auto edge_optional = G->get_edge(*mod.edge_from_id, *mod.edge_to_id, *mod.type);
        if (!edge_optional.has_value()) { std::cerr << __FUNCTION__ << " - [HistoricManager] MEA: Edge not found" << std::endl; return; }

        for(const auto &[name, attr] : mod.attributes)
            if (name != DSRAttributeNames::IDF &&
                name != DSRAttributeNames::IDT &&
                name != DSRAttributeNames::TYPE)
                G->add_or_modify_attrib_local(edge_optional.value(), name, attr);
        
        G->insert_or_assign_edge(edge_optional.value());
    }

    /**
     * @brief Apply DN - Delete Node
     */
    void apply_delete_node(const DSREvent &mod) {
        // Check if node exists
        if (!mod.node_id.has_value()) return;
        G->delete_node(*mod.node_id);
    }

    /**
     * @brief Apply DE - Delete Edge
     */
    void apply_delete_edge(const DSREvent &mod) {
        if (!mod.edge_from_id.has_value() || !mod.edge_to_id.has_value() || !mod.type.has_value()) return;
        G->delete_edge(*mod.edge_from_id, *mod.edge_to_id, *mod.type)
    }

    /**
     * @brief Apply every local change from a keyframe
     */
    void apply_all_local_changes(size_t keyframe_idx) {
        auto it = local_changes_metadata.find(keyframe_idx);
        if (it == local_changes_metadata.end()) return;

        for (const auto &meta : it->second) {
            DSREvent* change = get_event_by_metadata(meta);
            if (change) apply_modification_to_graph(*change);
        }
    }

    /**
     * @brief Clean every node and edge from graph
     */
    void clear_graph() {
        auto nodes = G->get_nodes();
        for (const auto &node : nodes)
            G->delete_node(node.id());
    }

    /**
     * @brief Initiates the background pre-load of keyframes around
     */
    void start_preload(size_t center_idx) {
        if (preloading) return;

        if (preload_thread.joinable())
            preload_thread.join()

        preload_thread = std::thread([this, center_idx]() {
            preloading = true;
            for (int offset = 1; offset <= 3; ++offset) {
                // Onwards
                if (center_idx + offset )
            }
        });
    }

};

#endif