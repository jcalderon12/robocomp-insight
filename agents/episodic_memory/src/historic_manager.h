#ifndef HISTORICMANAGER_H
#define HISTORICMANAGER_H

#include "DSRDecoder.h"
#include "DSRTypeTrait.h"
#include <atomic>
#include <deque>
#include <dsr/api/dsr_api.h>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

/**
 * @brief Event meta data from a keyframe or modification
 */
struct EventMetadata {
  uint64_t timestamp;            // timestamp
  std::streampos file_position;  // position on file
  size_t line_lenght;            // line lenght
  std::string modification_type; // K - MN, MNA, ME, MEA, DN, DE
  size_t keyframe_index; // keyframe it belongs (if modification other than K)
};

/**
 * @brief Historic manager. Indexes the content of a DSR Event, stores data in a
 * LRU cache and displays the graph.
 */
class HistoricManager {
public:
  HistoricManager(std::shared_ptr<DSR::DSRGraph> graph, size_t max_cache_size = 50)
      : G(graph), max_cache_size(max_cache_size), preloading(false),
        current_keyframe_idx(0) {}

  ~HistoricManager() {
    preloading = false;
    if (preload_thread.joinable())
      preload_thread.join();
  }

  /**
   * @brief Indexes an event file, reading timestamps, event types and file
   * positions.
   */
  bool index_file(const std::string &filepath) {
    this->filepath = filepath;
    std::ifstream file(filepath);
    if (!file.is_open()) {
      std::cerr << __FUNCTION__ << " - Cannot open file " << filepath
                << std::endl;
      return false;
    }

    keyframe_metadata.clear();
    local_changes_metadata.clear();
    all_events_metadata.clear();

    std::string line;
    size_t current_keyframe = 0;
    size_t event_idx = 0;

    while (std::getline(file, line)) {
      if (line.empty())
        continue;

      std::streampos pos = file.tellg();
      pos -= static_cast<std::streamoff>(line.size() + 1);

      auto [timestamp, type] = quick_parse_header(line);
      EventMetadata meta{timestamp, pos, line.size(), type, current_keyframe};

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

    std::cout << __FUNCTION__ << " - [HistoricManager] Indexed "
              << keyframe_metadata.size() << " keyframes with "
              << all_events_metadata.size() << " total events in " << filepath
              << std::endl;

    return true;
  }

  /**
   * @brief Get number of keyframe indexed
   */
  size_t get_keyframe_count() const { return keyframe_metadata.size(); }

  /**
   * @brief Get number of local changes inside a keyframe
   */
  size_t get_local_changes_count(size_t keyframe_idx,
                                 bool apply_local_changes = false) {
    auto it = local_changes_metadata.find(keyframe_idx);
    return (it != local_changes_metadata.end()) ? it->second.size() : 0;
  }

  /**
   * @brief Change current keyframe and displays the new one
   */
  void load_keyframe(size_t keyframe_idx, bool apply_local_changes = false) {
    if (keyframe_idx >= keyframe_metadata.size()) { std::cerr << __FUNCTION__ << " - [HistoricManager] Invalid keyframe index: " << keyframe_idx << std::endl; return; }
    std::cout << __FUNCTION__ << " - [HistoricManger] Loading keyframe " << keyframe_idx << std::endl;
    // Update current keyframe for pre-loading
    current_keyframe_idx = keyframe_idx;

    // Get new event
    DSREvent *keyframe_event = get_event_by_metadata(keyframe_metadata[keyframe_idx]);
    if (!keyframe_event) { std::cout << __FUNCTION__<< " - [HistoricManager] Failed to decode keyframe " << keyframe_idx << std::endl; return; }

    // Display new graph
    reconstruct_graph_from_keyframe(*keyframe_event);

    // Apply local changes
    if (apply_local_changes)
      apply_all_local_changes(keyframe_idx);

    // Pre-load other keyframes
    start_preload(keyframe_idx);
  }

  /**
   * @brief Apply an specific local changes
   */
  void apply_local_change(size_t keyframe_idx, size_t local_change_idx) {
    auto it = local_changes_metadata.find(keyframe_idx);
    if (it == local_changes_metadata.end() ||
        local_change_idx >= it->second.size()) {
      std::cerr << __FUNCTION__
                << " - [HistoricManger] Invalid local change index."
                << std::endl;
      return;
    }

    DSREvent *change = get_event_by_metadata(it->second[local_change_idx]);
    if (!change)
      return;

    apply_modification_to_graph(*change);

    std::cout << __FUNCTION__ << " - [HistoricManager] Applied local change "
              << local_change_idx << " (" << change->modification_type << ")"
              << std::endl;
  }

  /**
   * @brief Apply all local changes up to an specific index
   */
  void apply_local_changes_up_to(size_t keyframe_idx, size_t up_to_change_idx) {
    auto it = local_changes_metadata.find(keyframe_idx);
    if (it == local_changes_metadata.end())
      return;

    size_t limit = std::min(up_to_change_idx + 1, it->second.size());

    for (size_t i = 0; i < limit; ++i) {
      DSREvent *change = get_event_by_metadata(it->second[i]);
      if (change) {
        apply_modification_to_graph(*change);
      }
    }

    std::cout << __FUNCTION__ << " - [HistoricManager] Applied " << limit
              << " local changes" << std::endl;
  }

  /**
   * @brief Get timestamp of current keyframe
   */
  uint64_t get_current_keyframe_timestamp() const {
    if (current_keyframe_idx < keyframe_metadata.size())
      return keyframe_metadata[current_keyframe_idx].timestamp;
    return 0;
  }

  /**
   * @brief Get timestamp of an arbitrary keyframe
   */
  uint64_t get_keyframe_timestamp(size_t keyframe_idx) const {
    if (keyframe_idx < keyframe_metadata.size())
      return keyframe_metadata[keyframe_idx].timestamp;
    return 0;
  }

  /**
   * @brief Returns {timestamp, modification_type} for a local change.
   *        Returns {0, ""} if the indices are out of range.
   */
  std::pair<uint64_t, std::string>
  get_local_change_info(size_t keyframe_idx, size_t local_idx) const {
    auto it = local_changes_metadata.find(keyframe_idx);
    if (it == local_changes_metadata.end() || local_idx >= it->second.size())
      return {0, ""};
    const auto &meta = it->second[local_idx];
    return {meta.timestamp, meta.modification_type};
  }

  /**
   * @brief Clean up cache
   */
  void clear_cache() {
    std::lock_guard<std::mutex> lock(cache_mutex);
    event_cache.clear();
    cache_order.clear();
    std::cout << "[HistoricManager] Cache cleared" << std::endl;
  }

private:
  std::shared_ptr<DSR::DSRGraph> G;
  std::string filepath;

  // Metadata
  std::vector<EventMetadata> keyframe_metadata;
  std::map<size_t, std::vector<EventMetadata>> local_changes_metadata;
  std::vector<EventMetadata> all_events_metadata;

  // LRU cache
  std::map<uint64_t, std::unique_ptr<DSREvent>> event_cache;
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
    std::string type =
        line.substr(first_hash + 1, second_hash - first_hash - 1);

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
  DSREvent *get_event_by_metadata(const EventMetadata &meta) {
    std::lock_guard<std::mutex> lock(cache_mutex);

    // Cache search
    auto it = event_cache.find(meta.timestamp);
    if (it != event_cache.end()) {
      update_lru(meta.timestamp);
      return it->second.get();
    }

    // Not in cache, decode from file
    std::string line = read_line_at(meta.file_position);
    auto decoded = DSRDecoder::decode(line);
    if (!decoded) {
      std::cerr << __FUNCTION__
                << " - [HistoricManager] Failed to decoded event at timestamp "
                << meta.timestamp << std::endl;
      return nullptr;
    }

    // Add to cache
    DSREvent *ptr = decoded.get();
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
  void reconstruct_graph_from_keyframe(DSREvent &keyframe) {
    // Keyframe verification
    if (keyframe.modification_type != DSRSpecialChars::K) { std::cerr << __FUNCTION__ << " - [HistoricManager] Event is not a keyframe." << std::endl; return; }
    std::cout << __FUNCTION__<< " - [HistoricManager] Reconstructing graph from keyframe with " << keyframe.nodes.size() << " nodes and " << keyframe.edges.size() << " edges." << std::endl;

    // Create sets of Node and Edge IDs of that keyframe
    std::unordered_set<uint64_t> keyframe_node_ids;
    for (const auto &node : keyframe.nodes)
      keyframe_node_ids.insert(node.id());

    std::set<std::tuple<uint64_t, uint64_t, std::string>> keyframe_edge_keys;
    for (const auto &edge : keyframe.edges)
      keyframe_edge_keys.insert({edge.from(), edge.to(), edge.type()});

    // Remove edges that are no longer in the keyframe
    auto current_nodes = G->get_nodes();
    for (const auto &node : current_nodes) {
      auto edges_opt = G->get_edges(node.id());
      if (!edges_opt.has_value())
        continue;
      for (const auto &[key, edge] : edges_opt.value()) {
        if (!keyframe_edge_keys.count({edge.from(), edge.to(), edge.type()})) {
          try {
            G->delete_edge(edge.from(), edge.to(), edge.type());
          } catch (const std::exception &e) {
            std::cerr << __FUNCTION__<< " - [HistoricManager] Error deleting edge " << edge.from() << "->" << edge.to() << ": " << e.what() << std::endl;
          }
        }
      }
    }

    // --- Upsert nodes from the keyframe ---
    for (auto &node : keyframe.nodes) {
      try {
        auto existing = G->get_node(node.id());
        if (existing.has_value()) {
          // Node already exists: update name/type/attrs in place
          auto n = existing.value();
          n.name(node.name());
          n.type(node.type());
          n.attrs() = node.attrs();
          G->update_node(n);
        } else {
          // Node does not exist (was never in this graph instance): insert with
          // ID
          G->insert_node_with_id(node);
        }
      } catch (const std::exception &e) {
        std::cerr << __FUNCTION__ << " - [HistoricManager] Error upserting node " << node.id() << ": " << e.what() << std::endl;
      }
    }

    // --- Upsert edges from the keyframe ---
    for (auto &edge : keyframe.edges) {
      try {
        G->insert_or_assign_edge(edge);
      } catch (const std::exception &e) {
        std::cerr << __FUNCTION__<< " - [HistoricManager] Error upserting edge " << edge.from() << "->" << edge.to() << ": " << e.what() << std::endl;
      }
    }

    std::cout << __FUNCTION__ << " - [HistoricManager] Graph reconstructed successfully." << std::endl; }

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
    } catch (const std::exception &e) {
      std::cerr << __FUNCTION__ << " - []" << e.what() << '\n';
    }
  }

  /**
   * @brief Apply MN - Modify Node
   */
  void apply_modify_node(const DSREvent &mod) {
    if (!mod.node_id.has_value() || !mod.type.has_value() ||
        !mod.node_name.has_value()) {
      std::cerr << __FUNCTION__
                << " - [HistoricManager] MN: Missing required fields"
                << std::endl;
      return;
    }

    // Check if node already exists
    auto node_optional = G->get_node(*mod.node_name);
    if (!node_optional.has_value()) {
      DSR::Node node;
      node.id(*mod.node_id);
      node.type(*mod.type);
      node.name(*mod.node_name);
      G->insert_node_with_id(node);
    } else {
      auto node = node_optional.value();
      if (!(node.id() == *mod.node_id && node.name() == *mod.node_name &&
            node.type() == *mod.type)) {
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
    if (!mod.node_id.has_value()) {
      std::cerr << __FUNCTION__
                << " - [HistoricManager] MNA: Missing required fields"
                << std::endl;
      return;
    }

    // Check if node exists
    auto node_optional = G->get_node(*mod.node_id);
    if (!node_optional.has_value()) {
      std::cerr << __FUNCTION__ << " - [HistoricManager] MNA: Node "
                << *mod.node_id << " not found" << std::endl;
      return;
    }

    auto node = node_optional.value();
    for (const auto &[name, attr] : mod.attributes)
      if (name != DSRAttributeNames::ID)
        node.attrs()[name] =
            attr; // G->add_or_modify_attrib_local(node, name, attr);
    G->update_node(node);
  }

  /**
   * @brief Apply ME - Modify Edge
   */
  void apply_modify_edge(const DSREvent &mod) {
    if (!mod.edge_from_id.has_value() || !mod.edge_to_id.has_value() ||
        !mod.type.has_value()) {
      std::cerr << __FUNCTION__
                << " - [HistoricManager] ME: Missing required fields"
                << std::endl;
      return;
    }

    // Check if edge already exists
    auto edge_optional =
        G->get_edge(*mod.edge_from_id, *mod.edge_to_id, *mod.type);
    if (!edge_optional.has_value()) {
      DSR::Edge edge;
      edge.from(*mod.edge_from_id);
      edge.to(*mod.edge_to_id);
      edge.type(*mod.type);
      G->insert_or_assign_edge(edge);
    } else {
      auto edge = edge_optional.value();
      if (!(edge.from() == *mod.edge_from_id && edge.to() == *mod.edge_to_id &&
            edge.type() == *mod.type)) {
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
    if (!mod.edge_from_id.has_value() || !mod.edge_to_id.has_value() ||
        !mod.type.has_value()) {
      std::cerr << __FUNCTION__
                << " - [HistoricManager] MEA: Missing required fields"
                << std::endl;
      return;
    }

    // Check if edge exists
    auto edge_optional =
        G->get_edge(*mod.edge_from_id, *mod.edge_to_id, *mod.type);
    if (!edge_optional.has_value()) {
      std::cerr << __FUNCTION__ << " - [HistoricManager] MEA: Edge not found"
                << std::endl;
      return;
    }

    auto edge = edge_optional.value();
    for (const auto &[name, attr] : mod.attributes)
      if (name != DSRAttributeNames::IDF && name != DSRAttributeNames::IDT &&
          name != DSRAttributeNames::TYPE)
        edge.attrs()[name] = attr;
    // G->add_or_modify_attrib_local<">(edge, name, attr);

    G->insert_or_assign_edge(edge);
  }

  /**
   * @brief Apply DN - Delete Node
   */
  void apply_delete_node(const DSREvent &mod) {
    // Check if node exists
    if (!mod.node_id.has_value())
      return;
    G->delete_node(*mod.node_id);
  }

  /**
   * @brief Apply DE - Delete Edge
   */
  void apply_delete_edge(const DSREvent &mod) {
    // Check if edge exists
    if (!mod.edge_from_id.has_value() || !mod.edge_to_id.has_value() ||
        !mod.type.has_value())
      return;
    G->delete_edge(*mod.edge_from_id, *mod.edge_to_id, *mod.type);
  }

  /**
   * @brief Apply every local change from a keyframe
   */
  void apply_all_local_changes(size_t keyframe_idx) {
    auto it = local_changes_metadata.find(keyframe_idx);
    if (it == local_changes_metadata.end())
      return;

    for (const auto &meta : it->second) {
      DSREvent *change = get_event_by_metadata(meta);
      if (change)
        apply_modification_to_graph(*change);
    }
  }

  /**
   * @brief Removes all nodes (and their edges) from the graph without
   * destroying the RTPS participant. Used as a fallback; prefer the
   * diff-based upsert in reconstruct_graph_from_keyframe.
   */
  void clear_graph() {
    auto nodes = G->get_nodes();
    for (const auto &node : nodes) {
      try {
        G->delete_node(node.id());
      } catch (const std::exception &e) {
        std::cerr << __FUNCTION__ << " - [HistoricManager] Error deleting node "
                  << node.id() << ": " << e.what() << std::endl;
      }
    }
    std::cout << __FUNCTION__ << " - [HistoricManager] Graph cleared"
              << std::endl;
  }

  /**
   * @brief Initiates the background pre-load of keyframes around
   */
  void start_preload(size_t center_idx) {
    if (preloading)
      return;

    if (preload_thread.joinable())
      preload_thread.join();

    preload_thread = std::thread([this, center_idx]() {
      preloading = true;
      for (int offset = 1; offset <= 3; ++offset) {
        // Onwards
        if (center_idx + offset < keyframe_metadata.size())
          get_event_by_metadata(keyframe_metadata[center_idx + offset]);
        // Backwards
        if (center_idx >= static_cast<size_t>(offset))
          get_event_by_metadata(keyframe_metadata[center_idx - offset]);
      }

      preloading = false;
      std::cout << __FUNCTION__ << " - [HistoricManger] Preload completed."
                << std::endl;
    });
  }
};

#endif