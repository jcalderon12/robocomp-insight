# DSR Episodic Memory API - Reference Documentation

## Overview

`EpisodicMemoryAPI` is a query interface for accessing historical DSR graph data from recorded files. It provides O(log n) optimized searches and automatic caching.

**Header**: `dsr_episodic_api.h`  
**Namespace**: `DSR`  
**Dependencies**: `dsr_episodic_api.cpp`, `DSRDecoder.h`, DSR types

---

## Data Structures

### `DSRData`
Public data structure returned by all query methods. Represents a single event from the history file.

```cpp
struct DSRData {
    uint64_t timestamp;                    // Nanosecond timestamp
    std::string modification_type;         // "K", "MN", "MNA", "ME", "MEA", "DN", "DE"
    
    std::optional<uint64_t> node_id;       // Node ID (MN, MNA, DN)
    std::optional<uint64_t> edge_from_id;  // Source node ID (ME, MEA, DE)
    std::optional<uint64_t> edge_to_id;    // Target node ID (ME, MEA, DE)
    std::optional<std::string> type;       // Node/edge type
    std::optional<std::string> node_name;  // Node name (MN only)
    
    std::map<std::string, DSR::Attribute> attributes;  // All attributes
    
    std::vector<DSR::Node> nodes;  // Full node list (K only)
    std::vector<DSR::Edge> edges;  // Full edge list (K only)
};
```

**Modification Types**:
- `K` - Keyframe (full graph snapshot)
- `MN` - Modify node
- `MNA` - Modify node attributes
- `ME` - Modify edge
- `MEA` - Modify edge attributes
- `DN` - Delete node
- `DE` - Delete edge

---

## Constructor & Status

### `EpisodicMemoryAPI(const std::string &filepath)`

**Purpose**: Indexes a history file immediately upon construction.

**Parameters**:
- `filepath` - Path to history file (e.g., `"keyframes_and_changes.txt"`)

**Behavior**:
- Reads entire file once
- Builds internal indices (O(n) scanning)
- Pre-computes keyframe positions
- Creates node-name lookup index
- Logs indexing statistics to stdout

**Example**:
```cpp
DSR::EpisodicMemoryAPI api("episode_001.txt");
if (api.is_ready()) {
    // Ready to query
}
```

---

### `bool is_ready() const`

**Purpose**: Check if file was indexed successfully.

**Returns**: `true` if ready, `false` if indexing failed

**Example**:
```cpp
if (!api.is_ready()) {
    std::cerr << "Failed to index: " << api.get_filepath() << std::endl;
}
```

---

### `const std::string& get_filepath() const`

**Purpose**: Get the path of currently indexed file.

**Returns**: Reference to file path string

**Example**:
```cpp
std::cout << "Using file: " << api.get_filepath() << std::endl;
```

---

## Keyframe Queries

### `size_t get_keyframe_count() const`

**Purpose**: Get total number of keyframes.

**Returns**: Number of keyframes (always ≥ 1 if ready)

**Complexity**: O(1)

**Example**:
```cpp
size_t count = api.get_keyframe_count();  // Returns 5
```

---

### `std::vector<uint64_t> get_keyframe_timestamps() const`

**Purpose**: Get all keyframe timestamps.

**Returns**: Vector of nanosecond timestamps in chronological order

**Complexity**: O(k) where k = keyframe count

**Example**:
```cpp
auto timestamps = api.get_keyframe_timestamps();
// [1000000000, 2000000000, 3000000000, ...]
```

---

### `std::optional<uint64_t> get_keyframe_timestamp(size_t keyframe_idx) const`

**Purpose**: Get timestamp of specific keyframe.

**Parameters**:
- `keyframe_idx` - Index (0-based)

**Returns**: Timestamp in nanoseconds, or `nullopt` if out of range

**Complexity**: O(1)

**Example**:
```cpp
auto ts = api.get_keyframe_timestamp(0);
if (ts) {
    std::cout << "First keyframe at: " << ts.value() << " ns" << std::endl;
}
```

---

### `std::optional<DSRData> get_keyframe(size_t keyframe_idx)`

**Purpose**: Get full keyframe (K event) with all nodes and edges.

**Parameters**:
- `keyframe_idx` - Index (0-based)

**Returns**: `DSRData` with nodes/edges populated, or `nullopt` if index invalid

**Complexity**: O(1) lookup + O(d) decode (d = decoded size)

**Cache**: Decoded event cached (LRU, 100 entries max)

**Modification Type**: Always `K`

**Example**:
```cpp
auto kf = api.get_keyframe(0);
if (kf) {
    std::cout << "Nodes: " << kf->nodes.size() << std::endl;
    std::cout << "Edges: " << kf->edges.size() << std::endl;
}
```

---

### `std::optional<DSRData> get_keyframe_at_time(uint64_t timestamp)`

**Purpose**: Get keyframe closest to and ≤ given timestamp.

**Parameters**:
- `timestamp` - Nanosecond timestamp

**Returns**: Keyframe at or before timestamp, or `nullopt` if all after timestamp

**Complexity**: O(log k + d) where k = keyframe count

**Algorithm**: Binary search on keyframe timestamps

**Example**:
```cpp
uint64_t search_time = 1500000000;  // 1.5 seconds from start
auto kf = api.get_keyframe_at_time(search_time);
// Returns keyframe at 1000000000 (last before 1.5s)
```

---

### `std::optional<size_t> get_keyframe_index_at_time(uint64_t timestamp)`

**Purpose**: Get the index of the keyframe closest to and ≤ given timestamp.

**Parameters**:
- `timestamp` - Nanosecond timestamp

**Returns**: Index (0-based) of keyframe at or before timestamp, or `nullopt` if no keyframes exist

**Complexity**: O(log k) where k = keyframe count

**Algorithm**: Binary search on keyframe timestamps

**Use Case**: When you need to work with keyframe indices after searching by time

**Example**:
```cpp
uint64_t search_time = 1500000000;
auto kf_idx = api.get_keyframe_index_at_time(search_time);
if (kf_idx) {
    // Now use index to access related data
    auto changes = api.get_local_changes(kf_idx.value());
    size_t count = api.get_local_changes_count(kf_idx.value());
}
```

---

### `std::optional<size_t> get_event_index_at_timestamp(uint64_t timestamp)`

**Purpose**: Get the index of the event at exactly the given timestamp.

**Parameters**:
- `timestamp` - Exact nanosecond timestamp

**Returns**: Index (0-based) in the event list, or `nullopt` if no exact match

**Complexity**: O(log n)

**Algorithm**: Binary search with exact match

**Use Case**: When you need the position of an event in the timeline for further processing

**Example**:
```cpp
auto event_idx = api.get_event_index_at_timestamp(1234567890000);
if (event_idx) {
    std::cout << "Event is at position: " << event_idx.value() << std::endl;
    auto event = api.get_event_at_timestamp(1234567890000);
}
```

---

## Time-Range Queries

### `std::vector<DSRData> get_events_between(uint64_t t_start, uint64_t t_end)`

**Purpose**: All events (keyframes + changes) in time range.

**Parameters**:
- `t_start` - Inclusive start timestamp
- `t_end` - Inclusive end timestamp

**Returns**: Vector of events, chronologically ordered

**Complexity**: O(log n + m) where n = total events, m = result size

**Algorithm**: Binary search to find range, linear scan within range

**Example**:
```cpp
auto events = api.get_events_between(t0, t1);
std::cout << "Found " << events.size() << " events" << std::endl;
// Includes both K and local change events
```

---

### `std::vector<DSRData> get_changes_between(uint64_t t_start, uint64_t t_end)`

**Purpose**: Local changes only (no keyframes) in time range.

**Parameters**:
- `t_start` - Inclusive start timestamp
- `t_end` - Inclusive end timestamp

**Returns**: Vector of local change events (MN, MNA, ME, MEA, DN, DE)

**Complexity**: O(log n + m)

**Algorithm**: Binary search + filter by modification type ≠ K

**Example**:
```cpp
auto changes = api.get_changes_between(t0, t1);
// Excludes keyframe (K) events, only changes
```

---

### `std::optional<DSRData> get_event_at_timestamp(uint64_t timestamp)`

**Purpose**: Find exact event at given timestamp.

**Parameters**:
- `timestamp` - Exact nanosecond timestamp

**Returns**: Event at exact timestamp, or `nullopt` if no match

**Complexity**: O(log n)

**Algorithm**: Binary search with exact match

**Example**:
```cpp
auto event = api.get_event_at_timestamp(1234567890000);
if (event) {
    std::cout << "Event type: " << event->modification_type << std::endl;
}
```

---

## Local Change Queries

### `size_t get_local_changes_count(size_t keyframe_idx) const`

**Purpose**: Count local changes after a keyframe.

**Parameters**:
- `keyframe_idx` - Keyframe index

**Returns**: Number of local changes before next keyframe

**Complexity**: O(1)

**Example**:
```cpp
size_t changes = api.get_local_changes_count(0);
std::cout << "Keyframe 0 has " << changes << " local changes" << std::endl;
```

---

### `std::vector<DSRData> get_local_changes(size_t keyframe_idx)`

**Purpose**: Get all local changes for a keyframe.

**Parameters**:
- `keyframe_idx` - Keyframe index

**Returns**: All local changes in order, from right after keyframe until next keyframe

**Complexity**: O(m) where m = number of changes

**Example**:
```cpp
auto changes = api.get_local_changes(0);
for (const auto &change : changes) {
    std::cout << "Change type: " << change.modification_type << std::endl;
}
```

---

### `std::optional<DSRData> get_local_change(size_t keyframe_idx, size_t change_idx)`

**Purpose**: Get single local change within keyframe.

**Parameters**:
- `keyframe_idx` - Which keyframe
- `change_idx` - Index within that keyframe's changes (0-based)

**Returns**: Single change event, or `nullopt` if out of range

**Complexity**: O(1) lookup + O(d) decode

**Example**:
```cpp
auto change = api.get_local_change(0, 0);  // First change of first keyframe
if (change) {
    std::cout << "Type: " << change->modification_type << std::endl;
}
```

---

## Node-Centric Queries

### `std::vector<DSRData> get_node_history(uint64_t node_id)`

**Purpose**: All events referencing a specific node.

**Parameters**:
- `node_id` - Node ID

**Returns**: Events that modify/delete this node (MN, MNA, DN types only)

**Complexity**: O(n) linear scan (cannot optimize further)

**Note**: IDs are original IDs from file, not live DSR IDs

**Example**:
```cpp
auto history = api.get_node_history(42);
for (const auto &event : history) {
    std::cout << "Node 42 changed at: " << event.timestamp << std::endl;
}
```

---

### `std::vector<DSRData> get_node_history_by_name(const std::string &node_name)`

**Purpose**: All events for node with given name.

**Parameters**:
- `node_name` - Name string (e.g., "robot")

**Returns**: All events for any node ever named this string

**Complexity**: O(1) index lookup + O(m) result collection

**Index**: Pre-built during file indexing for instant lookup

**How it works**:
1. Look up name in `node_name_index_` (O(1))
2. Get all IDs that had this name
3. Collect events for all those IDs

**Example**:
```cpp
auto events = api.get_node_history_by_name("robot");
// Instant lookup vs old method that scanned entire file twice
```

---

### `std::vector<DSRData> get_node_changes_between(uint64_t node_id, uint64_t t_start, uint64_t t_end)`

**Purpose**: Changes to specific node within time range.

**Parameters**:
- `node_id` - Node ID
- `t_start` - Start timestamp
- `t_end` - End timestamp

**Returns**: Events modifying this node in time range

**Complexity**: O(log n + m)

**Algorithm**: Binary search time range, filter by node_id

**Example**:
```cpp
auto changes = api.get_node_changes_between(42, t0, t1);
// All ways node 42 changed between t0 and t1
```

---

## Edge-Centric Queries

### `std::vector<DSRData> get_edge_history(uint64_t from_id, uint64_t to_id, const std::string &edge_type)`

**Purpose**: All events for specific edge.

**Parameters**:
- `from_id` - Source node ID
- `to_id` - Target node ID
- `edge_type` - Edge type (e.g., "parent", "connects")

**Returns**: Events modifying/deleting this exact edge

**Complexity**: O(n) linear scan

**Modification types**: ME, MEA, DE

**Example**:
```cpp
auto edge_events = api.get_edge_history(1, 2, "parent");
// All modifications to edge 1→2 of type "parent"
```

---

## Type-Based Filtering

### `std::vector<DSRData> get_events_by_type(const std::string &mod_type)`

**Purpose**: All events of specific modification type.

**Parameters**:
- `mod_type` - One of: `"K"`, `"MN"`, `"MNA"`, `"ME"`, `"MEA"`, `"DN"`, `"DE"`

**Returns**: All events matching that type

**Complexity**: O(n) linear scan

**Example**:
```cpp
auto keyframes = api.get_events_by_type("K");
auto deletes = api.get_events_by_type("DN");  // All node deletions
```

---

## Internal Details

### Caching

- **LRU Cache**: Stores decoded events (100 max)
- **Hit Ratio**: High for sequential or repeated queries
- **Automatic**: Transparent to user

### Indexing

Built once at construction:
- **Keyframe index**: Fast keyframe access
- **Event metadata**: Timestamp, type, file position
- **Name index**: Node name → ID mappings
- **All events list**: Sorted by timestamp

### File Format

Expected format (one per line):
```
<timestamp>|<type>|<encoded_data>
```

Example:
```
1000000000|K|nodes=5 edges=3 ...
1000001000|MN|node_id=1 attrs=...
1000002000|MNA|node_id=2 attrs=...
```

---

## Error Handling

All methods use `std::optional<>`:
- **Present**: Valid result available
- **Absent** (`nullopt`): Invalid parameter or no match

**Common nullopt cases**:
- Out of range index
- No events in time range
- No exact timestamp match

```cpp
auto result = api.get_keyframe(999);
if (result.has_value()) {
    // Use result
} else {
    // Index out of range
}
```

---

## Performance Characteristics

| Operation | Complexity | Notes |
|-----------|-----------|-------|
| `get_keyframe_count()` | O(1) | |
| `get_keyframe(i)` | O(log i + d) | d = decode time |
| `get_keyframe_at_time(t)` | O(log k) | k = keyframe count |
| `get_keyframe_index_at_time(t)` | O(log k) | Index-only version, no decode |
| `get_event_at_timestamp(t)` | O(log n) | Binary search |
| `get_event_index_at_timestamp(t)` | O(log n) | Index-only version |
| `get_events_between(t1,t2)` | O(log n + m) | m = result size, binary search |
| `get_changes_between(t1,t2)` | O(log n + m) | Same as above |
| `get_node_history(id)` | O(n) | Linear scan |
| `get_node_history_by_name(name)` | O(1 + m) | Index lookup |
| `get_node_changes_between()` | O(log n + m) | Binary search + filter |
| `get_edge_history()` | O(n) | Linear scan |
| `get_events_by_type()` | O(n) | Linear scan |
| `get_local_changes_count(idx)` | O(1) | |
| `get_local_changes(idx)` | O(m) | m = changes count |
| `get_local_change(idx, change_idx)` | O(1 + d) | Index + decode |

---

## Usage Example

```cpp
// Initialize
DSR::EpisodicMemoryAPI api("episode.txt");
if (!api.is_ready()) {
    std::cerr << "Failed to load history" << std::endl;
    return;
}

// Query keyframes
std::cout << "Total keyframes: " << api.get_keyframe_count() << std::endl;

// Get first keyframe
auto kf = api.get_keyframe(0);
std::cout << "Initial state: " << kf->nodes.size() << " nodes" << std::endl;

// Find state at specific time
uint64_t t_search = 5000000000;  // 5 seconds
auto state = api.get_keyframe_at_time(t_search);

// NEW: Get index to work with local changes
auto kf_idx = api.get_keyframe_index_at_time(t_search);
if (kf_idx) {
    size_t num_changes = api.get_local_changes_count(kf_idx.value());
    auto local_changes = api.get_local_changes(kf_idx.value());
    std::cout << "Keyframe " << kf_idx.value() << " has " << num_changes 
              << " local changes" << std::endl;
}

// Get all changes to specific node
auto changes = api.get_node_changes_between(42, kf->timestamp, 
                                           api.get_keyframe_timestamp(
                                               api.get_keyframe_count()-1).value());
std::cout << "Node 42 changed " << changes.size() << " times" << std::endl;

// Find by name
auto robot_events = api.get_node_history_by_name("robot");
std::cout << "Robot events: " << robot_events.size() << std::endl;

// NEW: Find exact event by timestamp and get its index
auto exact_event = api.get_event_at_timestamp(1234567890000);
auto event_idx = api.get_event_index_at_timestamp(1234567890000);
if (event_idx) {
    std::cout << "Exact event found at index: " << event_idx.value() << std::endl;
}
```

---

## Compilation

Include header and link implementation:
```cpp
#include "dsr_episodic_api.h"

// In CMakeLists.txt
target_sources(my_target PRIVATE dsr_episodic_api.cpp)
```

---

## Thread Safety

⚠️ **Not thread-safe**. Do not call methods from multiple threads simultaneously. Use external synchronization if needed.

