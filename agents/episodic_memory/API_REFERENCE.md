# Episodic Memory API - Complete Reference

## 📖 What is This?

Complete API reference documentation for the Episodic Memory API. This document contains:
- All 20+ methods with full documentation
- Working code examples (C++ and Python)
- Data structures and error handling
- Performance characteristics and caching behavior

## 🎯 Quick Navigation

- **Just give me examples** → Jump to [Usage Example](#usage-example)
- **I need quick method list** → See [Quick Reference](#quick-reference-all-20-methods) below
- **I'm using Python** → See [Python Usage Notes](#python-specific-notes)
- **I need to handle errors** → Jump to [Error Handling](#error-handling)
- **Understanding timestamps** → See [Timestamps Explained](#timestamps-explained)

---

## 📑 Table of Contents

- [Quick Reference](#quick-reference-all-20-methods) - All 20+ methods at a glance
- [Data Structures](#data-structures) - DSRData and Python bindings
- [Constructor & Status](#constructor--status) - Initialization (3 methods)
- [Keyframe Queries](#keyframe-queries) - Snapshot access (6 methods)
- [Time-Range Queries](#time-range-queries) - Range queries (4 methods)
- [Local Change Queries](#local-change-queries) - Changes between keyframes (3 methods)
- [Node-Centric Queries](#node-centric-queries) - Node history (3 methods)
- [Edge-Centric Queries](#edge-centric-queries) - Edge tracking (1 method)
- [Type-Based Filtering](#type-based-filtering) - Filter by type (1 method)
- [Internal Details](#internal-details) - Caching and indexing
- [Error Handling](#error-handling) - C++ and Python patterns
- [Timestamps Explained](#timestamps-explained) - Nanosecond timestamps
- [Usage Example](#usage-example) - Full C++ and Python programs
- [Compilation](#compilation) - Build instructions

---

## Quick Reference: All 20+ Methods

### Status
- `is_ready()` - Check if ready
- `get_filepath()` - Get file path

### Keyframes
- `get_keyframe_count()` - Total keyframes
- `get_keyframe(idx)` - Get by index
- `get_keyframe_at_time(ts)` - Get at/before time
- `get_keyframe_timestamps()` - All times
- `get_keyframe_index_at_time(ts)` - Find index by time

### Changes Between Keyframes
- `get_local_changes(kf_idx)` - Get all changes
- `get_local_changes_count(kf_idx)` - Count changes
- `get_local_change(kf_idx, idx)` - Get one change

### Time Ranges
- `get_events_between(t0, t1)` - All events in range
- `get_changes_between(t0, t1)` - Changes only in range
- `get_event_at_timestamp(ts)` - Exact time lookup
- `get_event_index_at_timestamp(ts)` - Index at time

### Node Queries
- `get_node_history(id)` - History by node ID
- `get_node_history_by_name(name)` - History by node name
- `get_node_changes_between(id, t0, t1)` - Changes in time range

### Edge Queries
- `get_edge_history(from, to, type)` - Edge history

### Filtering
- `get_events_by_type(type)` - Filter by type

---

### Building
```bash
cd agents/episodic_memory
mkdir build && cd build
cmake ..
make -j8
```

---

[Back to top ↑](#episodic-memory-api---complete-reference)

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

### Python-Specific Notes

When using the Python binding, DSRData properties are accessible as attributes:
- **C++**: `kf->nodes` → **Python**: `kf.nodes_list`
- **C++**: `kf->edges` → **Python**: `kf.edges_list`
- **C++**: `std::optional<T>` → **Python**: `None` if not present

Example Python:
```python
kf = memory.get_keyframe(0)
if kf:  # Check if result exists
    print(f"Nodes: {len(kf.nodes_list)}")
    print(f"Edges: {len(kf.edges_list)}")
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

[Back to top ↑](#episodic-memory-api---complete-reference)

---

## Constructor & Status

### `EpisodicMemoryAPI(const std::string &filepath)`

**Purpose**: Index a history file immediately upon construction.  
**Params**: `filepath` - Path to history file  
**Behavior**: Reads file, builds indices, creates noun name lookup  
**Error**: If fails, `is_ready()` returns `false`

**Examples**:
```cpp
// C++
DSR::EpisodicMemoryAPI api("episode.txt");
if (!api.is_ready()) {
    std::cerr << "Failed to load" << std::endl;}
```
```python
# Python
import episodic_memory_api as api
memory = api.EpisodicMemoryAPI("episode.txt")
if not memory.is_ready():
    print("Failed to load")
```

---

### `bool is_ready() const`

**Purpose**: Check if file was indexed successfully.  
**Returns**: `true` if ready, `false` if failed

**Examples**:
```cpp
// C++
if (!api.is_ready()) std::cerr << "Error" << std::endl;
```
```python
# Python
if not memory.is_ready():
    print("Error")
```


---

### `const std::string& get_filepath() const`

**Purpose**: Get path of currently indexed file.  
**Returns**: Reference to file path string

**Examples**:
```cpp
// C++
std::cout << "File: " << api.get_filepath() << std::endl;
```
```python
# Python
print(f"File: {memory.get_filepath()}")
```

---

[Back to top ↑](#episodic-memory-api---complete-reference)

---

## Keyframe Queries

### `size_t get_keyframe_count() const`

**Purpose**: Get total number of keyframes.  
**Returns**: `size_t` - Number of keyframes  
**Complexity**: O(1)

**Examples**:
```cpp
// C++
auto count = api.get_keyframe_count();
std::cout << "Total: " << count << std::endl;
```
```python
# Python
count = memory.get_keyframe_count()
print(f"Total: {count}")
```

---

### `std::vector<uint64_t> get_keyframe_timestamps() const`

**Purpose**: Get all keyframe timestamps.  
**Returns**: Vector of uint64_t timestamps in nanoseconds, chronologically ordered  
**Complexity**: O(k) where k = keyframe count

**Examples**:
```cpp
// C++
auto timestamps = api.get_keyframe_timestamps();
for (auto ts : timestamps) std::cout << ts << std::endl;
```
```python
# Python
timestamps = memory.get_keyframe_timestamps()
for ts in timestamps:
    print(ts)
```

---

### `std::optional<uint64_t> get_keyframe_timestamp(size_t keyframe_idx) const`

**Purpose**: Get timestamp of specific keyframe.  
**Params**: `keyframe_idx` - Index (0-based)  
**Returns**: Timestamp in nanoseconds, or `null/None` if out of range  
**Complexity**: O(1)

**Examples**:
```cpp
// C++
auto ts = api.get_keyframe_timestamp(0);
if (ts) std::cout << "At: " << ts.value() << " ns" << std::endl;
```
```python
# Python
ts = memory.get_keyframe_timestamp(0)
if ts is not None:
    print(f"At: {ts} ns")
```

---

### `std::optional<DSRData> get_keyframe(size_t keyframe_idx)`

**Purpose**: Get full keyframe with all nodes and edges.  
**Params**: `keyframe_idx` - Index (0-based)  
**Returns**: Complete graph state, or `null/None` if invalid  
**Complexity**: O(1) + decode. Cached (LRU, 100 max)

**Examples**:
```cpp
// C++
auto kf = api.get_keyframe(0);
if (kf) {
    std::cout << "Nodes: " << kf->nodes.size() << std::endl;
}
```
```python
# Python
kf = memory.get_keyframe(0)
if kf:
    print(f"Nodes: {len(kf.nodes_list)}")
```

---

### `std::optional<DSRData> get_keyframe_at_time(uint64_t timestamp)`

**Purpose**: Get keyframe at or before given timestamp.  
**Params**: `timestamp` - Nanosecond timestamp  
**Returns**: Keyframe at/before time, or `null/None` if all after  
**Complexity**: O(log k) binary search

**Examples**:
```cpp
// C++
auto kf = api.get_keyframe_at_time(1500000000);
if (kf) std::cout << "Found keyframe" << std::endl;
```
```python
# Python
kf = memory.get_keyframe_at_time(1500000000)
if kf:
    print("Found keyframe")
```

---

### `std::optional<size_t> get_keyframe_index_at_time(uint64_t timestamp)`

**Purpose**: Get keyframe index at or before timestamp.  
**Params**: `timestamp` - Nanosecond timestamp  
**Returns**: Index (0-based) of keyframe, or `null/None` if not found  
**Complexity**: O(log k) binary search

**Examples**:
```cpp
// C++
auto idx = api.get_keyframe_index_at_time(1500000000);
if (idx) {
    auto changes = api.get_local_changes(idx.value());
}
```
```python
# Python
idx = memory.get_keyframe_index_at_time(1500000000)
if idx is not None:
    changes = memory.get_local_changes(idx)
```

---

### `std::optional<size_t> get_event_index_at_timestamp(uint64_t timestamp)`

**Purpose**: Get index of exact event at timestamp.  
**Params**: `timestamp` - Exact nanosecond timestamp  
**Returns**: Event index in timline, or `null/None` if no match  
**Complexity**: O(log n) binary search

**Examples**:
```cpp
// C++
auto idx = api.get_event_index_at_timestamp(1234567890000);
if (idx) std::cout << "Position: " << idx.value() << std::endl;
```
```python
# Python
idx = memory.get_event_index_at_timestamp(1234567890000)
if idx is not None:
    print(f"Position: {idx}")
```

---

[Back to top ↑](#episodic-memory-api---complete-reference)

---

## Time-Range Queries

### `std::vector<DSRData> get_events_between(uint64_t t_start, uint64_t t_end)`

**Purpose**: All events (keyframes + changes) in time range.  
**Params**: `t_start`, `t_end` - Inclusive timestamps in nanoseconds  
**Returns**: Vector of events chronologically ordered  
**Complexity**: O(log n + m) where m = result size

**Examples**:
```cpp
// C++
auto events = api.get_events_between(t0, t1);
std::cout << "Found " << events.size() << " events" << std::endl;
```
```python
# Python
events = memory.get_events_between(t0, t1)
print(f"Found {len(events)} events")
```

---

### `std::vector<DSRData> get_changes_between(uint64_t t_start, uint64_t t_end)`

**Purpose**: Changes only (no keyframes) in time range.  
**Params**: `t_start`, `t_end` - Inclusive timestamps  
**Returns**: Vector of change events (MN, MNA, ME, MEA, DN, DE)  
**Complexity**: O(log n + m)

**Examples**:
```cpp
// C++
auto changes = api.get_changes_between(t0, t1);
for (auto &ch : changes) std::cout << ch.modification_type << std::endl;
```
```python
# Python
changes = memory.get_changes_between(t0, t1)
for ch in changes:
    print(ch.modification_type)
```

---

### `std::optional<DSRData> get_event_at_timestamp(uint64_t timestamp)`

**Purpose**: Find exact event at given timestamp.  
**Params**: `timestamp` - Exact nanosecond timestamp  
**Returns**: Event at exact time, or `null/None` if no match  
**Complexity**: O(log n) binary search

**Examples**:
```cpp
// C++
auto event = api.get_event_at_timestamp(1234567890000);
if (event) std::cout << "Type: " << event->modification_type << std::endl;
```
```python
# Python
event = memory.get_event_at_timestamp(1234567890000)
if event is not None:
    print(f"Type: {event.modification_type}")
```

---

[Back to top ↑](#episodic-memory-api---complete-reference)

---

## Local Change Queries

### `size_t get_local_changes_count(size_t keyframe_idx) const`

**Purpose**: Count local changes after a keyframe.  
**Params**: `keyframe_idx` - Keyframe index  
**Returns**: Count of local changes before next keyframe  
**Complexity**: O(1)

**Examples**:
```cpp
// C++
auto count = api.get_local_changes_count(0);
std::cout << "Changes: " << count << std::endl;
```
```python
# Python
count = memory.get_local_changes_count(0)
print(f"Changes: {count}")
```

---

### `std::vector<DSRData> get_local_changes(size_t keyframe_idx)`

**Purpose**: Get all local changes after a keyframe.  
**Params**: `keyframe_idx` - Keyframe index  
**Returns**: All changes until next keyframe, in order  
**Complexity**: O(m) where m = number of changes

**Examples**:
```cpp
// C++
auto changes = api.get_local_changes(0);
for (auto &ch : changes) std::cout << ch.modification_type << std::endl;
```
```python
# Python
changes = memory.get_local_changes(0)
for ch in changes:
    print(ch.modification_type)
```

---

### `std::optional<DSRData> get_local_change(size_t keyframe_idx, size_t change_idx)`

**Purpose**: Get single change within keyframe's changes.  
**Params**: `keyframe_idx`, `change_idx` - Both zero-based indices  
**Returns**: Single change event, or `null/None` if out of range  
**Complexity**: O(1) + decode

**Examples**:
```cpp
// C++
auto change = api.get_local_change(0, 0);
if (change) std::cout << "Type: " << change->modification_type << std::endl;
```
```python
# Python
change = memory.get_local_change(0, 0)
if change is not None:
    print(f"Type: {change.modification_type}")
```

---

[Back to top ↑](#episodic-memory-api---complete-reference)

---

## Node-Centric Queries

### `std::vector<DSRData> get_node_history(uint64_t node_id)`

**Purpose**: All events referencing a specific node.  
**Params**: `node_id` - Node ID (from file, not live DSR)  
**Returns**: Events modifying/deleting this node (MN, MNA, DN)  
**Complexity**: O(n) linear scan

**Examples**:
```cpp
// C++
auto history = api.get_node_history(42);
for (auto &event : history)
    std::cout << "Changed at: " << event.timestamp << std::endl;
```
```python
# Python
history = memory.get_node_history(42)
for event in history:
    print(f"Changed at: {event.timestamp}")
```

---

### `std::vector<DSRData> get_node_history_by_name(const std::string &node_name)`

**Purpose**: All events for nodes with given name.  
**Params**: `node_name` - Name string (e.g., "robot")  
**Returns**: All events for all nodes ever named this  
**Complexity**: O(1) index lookup + O(m) collection

**Examples**:
```cpp
// C++
auto events = api.get_node_history_by_name("robot");
std::cout << "Found " << events.size() << " events" << std::endl;
```
```python
# Python
events = memory.get_node_history_by_name("robot")
print(f"Found {len(events)} events")
```

---

### `std::vector<DSRData> get_node_changes_between(uint64_t node_id, uint64_t t_start, uint64_t t_end)`

**Purpose**: Changes to specific node in time range.  
**Params**: `node_id`, `t_start`, `t_end` - Node ID and timestamps  
**Returns**: Events modifying this node in range  
**Complexity**: O(log n + m)

**Examples**:
```cpp
// C++
auto changes = api.get_node_changes_between(42, t0, t1);
std::cout << "Found " << changes.size() << " changes" << std::endl;
```
```python
# Python
changes = memory.get_node_changes_between(42, t0, t1)
print(f"Found {len(changes)} changes")
```

---

[Back to top ↑](#episodic-memory-api---complete-reference)

---

## Edge-Centric Queries

### `std::vector<DSRData> get_edge_history(uint64_t from_id, uint64_t to_id, const std::string &edge_type)`

**Purpose**: All events for specific edge.  
**Params**: `from_id`, `to_id`, `edge_type` - Source, target, type  
**Returns**: Events modifying/deleting this exact edge  
**Complexity**: O(n) linear scan

**Examples**:
```cpp
// C++
auto events = api.get_edge_history(1, 2, "parent");
std::cout << "Found " << events.size() << " events" << std::endl;
```
```python
# Python
events = memory.get_edge_history(1, 2, "parent")
print(f"Found {len(events)} events")
```

---

[Back to top ↑](#episodic-memory-api---complete-reference)

---

## Type-Based Filtering

### `std::vector<DSRData> get_events_by_type(const std::string &mod_type)`

**Purpose**: All events of specific modification type.  
**Params**: `mod_type` - One of: `"K"`, `"MN"`, `"MNA"`, `"ME"`, `"MEA"`, `"DN"`, `"DE"`  
**Returns**: All events matching that type  
**Complexity**: O(n) linear scan

**Examples**:
```cpp
// C++
auto keyframes = api.get_events_by_type("K");
auto deletes = api.get_events_by_type("DN");
std::cout << "Deletions: " << deletes.size() << std::endl;
```
```python
# Python
keyframes = memory.get_events_by_type("K")
deletes = memory.get_events_by_type("DN")
print(f"Deletions: {len(deletes)}")
```

---

[Back to top ↑](#episodic-memory-api---complete-reference)

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

Expected format (one per line, using `#` as separator):
```
<timestamp>#<type>#<encoded_data>#@
```

Example:
```
1772107272287046051#K#name$s:robot%type$s:robot%id$ui64:200%#@
1772107277358034125#MN#id$ui64:300%type$s:object%name$s:box%#@
1772107278412056789#MNA#id$ui64:300%attr$s:color%value$s:red%#@
```

**Note**: The separator is `#` not `|`. Each line ends with `#@`.

---

[Back to top ↑](#episodic-memory-api---complete-reference)

---

## Error Handling

### C++ Style

All methods use `std::optional<>`:
- **Present**: Valid result available
- **Absent** (`nullopt`): Invalid parameter or no match

**Check result with either**:
```cpp
if (result.has_value()) {
    auto value = result.value();  // Get the value
}

// OR (shorthand)
if (result) {
    auto value = *result;  // Dereference directly
}
```

**Common nullopt cases**:
- Out of range index
- No events in time range
- No exact timestamp match

### Python Style

In Python, `std::optional<T>` is converted:
- **Present**: Actual value (int, list, dict, etc.)
- **Absent**: `None`

```python
# C++ std::optional<DSRData> becomes Python None or DSRData
result = api.get_keyframe(999)
if result is not None:        # Python way to check
    print(f"Found: {result}")
else:
    print("Index out of range")

# Works with any method returning optional
idx = api.get_keyframe_index_at_time(search_time)
if idx is not None:
    print(f"Found at index: {idx}")
```

---

## Timestamps Explained

**Format**: All timestamps are in **nanoseconds**

**Conversion examples**:
```python
# Python
ts_nanoseconds = 1772107272287046051
ts_seconds = ts_nanoseconds / 1e9           # → 1772107272.287
ts_milliseconds = ts_nanoseconds / 1e6      # → 1772107272287.046
ts_microseconds = ts_nanoseconds / 1e3      # → 1772107272287046.051

# Example
from datetime import datetime
datetime.fromtimestamp(ts_seconds)  # Convert to readable date
```

**Origin**: Timestamps typically start from the beginning of the recording session (not Unix epoch).

---

[Back to top ↑](#episodic-memory-api---complete-reference)

---

## Usage Example

### C++ Version

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
if (kf) {
    std::cout << "Initial state: " << kf->nodes.size() << " nodes" << std::endl;
}

// Find state at specific time
uint64_t t_search = 5000000000;  // 5 seconds
auto state = api.get_keyframe_at_time(t_search);

// Get index to work with local changes
auto kf_idx = api.get_keyframe_index_at_time(t_search);
if (kf_idx) {
    size_t num_changes = api.get_local_changes_count(kf_idx.value());
    auto local_changes = api.get_local_changes(kf_idx.value());
    std::cout << "Keyframe " << kf_idx.value() << " has " << num_changes 
              << " local changes" << std::endl;
}

// Find by name
auto robot_events = api.get_node_history_by_name("robot");
std::cout << "Robot events: " << robot_events.size() << std::endl;

// Find exact event by timestamp and get its index
auto exact_event = api.get_event_at_timestamp(1234567890000);
auto event_idx = api.get_event_index_at_timestamp(1234567890000);
if (event_idx) {
    std::cout << "Exact event found at index: " << event_idx.value() << std::endl;
}
```

### Python Version

```python
# Import the API
import episodic_memory_api as api

# Initialize
memory = api.EpisodicMemoryAPI("episode.txt")
if not memory.is_ready():
    print("Failed to load history")
    exit(1)

# Query keyframes
print(f"Total keyframes: {memory.get_keyframe_count()}")

# Get first keyframe
kf = memory.get_keyframe(0)
if kf:  # Note: Python uses 'is not None' or just 'if kf:'
    print(f"Initial state: {len(kf.nodes_list)} nodes")  # Note: nodes_list not nodes

# Find state at specific time
t_search = 5000000000  # 5 seconds
state = memory.get_keyframe_at_time(t_search)

# Get index to work with local changes
kf_idx = memory.get_keyframe_index_at_time(t_search)
if kf_idx is not None:  # Python way to check if not None
    num_changes = memory.get_local_changes_count(kf_idx)
    local_changes = memory.get_local_changes(kf_idx)
    print(f"Keyframe {kf_idx} has {num_changes} local changes")

# Find by name
robot_events = memory.get_node_history_by_name("robot")
print(f"Robot events: {len(robot_events)}")

# Find exact event by timestamp and get its index
exact_event = memory.get_event_at_timestamp(1234567890000)
event_idx = memory.get_event_index_at_timestamp(1234567890000)
if event_idx is not None:
    print(f"Exact event found at index: {event_idx}")
```

---

[Back to top ↑](#episodic-memory-api---complete-reference)

---

## Compilation

Include header and link implementation:
```cpp
#include "dsr_episodic_api.h"

// In CMakeLists.txt
target_sources(my_target PRIVATE dsr_episodic_api.cpp)
```

---

[Back to top ↑](#episodic-memory-api---complete-reference)