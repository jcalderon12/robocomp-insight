#!/usr/bin/env python3
"""
Example usage of the Episodic Memory API Python binding

This script demonstrates the main query patterns and best practices.
"""

import episodic_memory_api as api
import sys
import time
import math

def example_initialization(history_file):
    """Example 1: Initialize and check status"""
    print("=" * 70)
    print("Example 1: Initialization")
    print("=" * 70)
    
    memory = api.EpisodicMemoryAPI(history_file)
    
    if not memory.is_ready():
        print(f"Error: Failed to load {memory.get_filepath()}")
        return None
    
    print(f"✓ File loaded: {memory.get_filepath()}")
    print(f"✓ Keyframes: {memory.get_keyframe_count()}")
    print()
    
    return memory

def example_keyframe_queries(memory):
    """Example 2: Basic keyframe queries"""
    print("=" * 70)
    print("Example 2: Keyframe Queries")
    print("=" * 70)
    
    # Get all keyframe timestamps
    timestamps = memory.get_keyframe_timestamps()
    print(f"Keyframe timestamps ({len(timestamps)} total):")
    for i, ts in enumerate(timestamps[:3]):  # Show first 3
        print(f"  [{i}] {ts} ns")
    if len(timestamps) > 3:
        print(f"  ... and {len(timestamps) - 3} more")
    print()
    
    # Get first keyframe data
    kf = memory.get_keyframe(0)
    if kf:
        print(f"First keyframe (index=0):")
        print(f"  Timestamp: {kf.timestamp} ns")
        print(f"  Type: {kf.modification_type}")
        print(f"  Nodes: {len(kf.nodes_list)}")
        print(f"  Edges: {len(kf.edges_list)}")
        if len(kf.nodes_list) > 0:
            first_node = kf.nodes_list[0]
            print(f"  First node: name={first_node['name']}, type={first_node['type']}")
    print()

def example_time_based_search(memory):
    """Example 3: Time-based search (most common use case)"""
    print("=" * 70)
    print("Example 3: Time-Based Search")
    print("=" * 70)
    
    # Get timestamp range
    timestamps = memory.get_keyframe_timestamps()
    if len(timestamps) < 2:
        print("Skipped: Need at least 2 keyframes")
        return
    
    t_start = timestamps[0]
    t_end = timestamps[-1]
    
    print(f"Time range: {t_start} - {t_end} ns")
    print(f"  Duration: {(t_end - t_start) / 1e9:.2f} seconds")
    print()
    
    # Search at mid-point
    t_mid = (t_start + t_end) // 2
    print(f"Searching for keyframe at {t_mid} ns (mid-point)...")
    
    # Method 1: Get keyframe data directly
    kf = memory.get_keyframe_at_time(t_mid)
    if kf:
        print(f"  Found keyframe at: {kf.timestamp} ns")
    
    # Method 2: Get index first (then use for local changes)
    kf_idx = memory.get_keyframe_index_at_time(t_mid)
    if kf_idx is not None:
        print(f"  Keyframe index: {kf_idx}")
        change_count = memory.get_local_changes_count(kf_idx)
        print(f"  Local changes after this keyframe: {change_count}")
    print()

def example_local_changes(memory):
    """Example 4: Query local changes"""
    print("=" * 70)
    print("Example 4: Local Changes")
    print("=" * 70)
    
    # Get local changes for first keyframe
    change_count = memory.get_local_changes_count(0)
    print(f"Keyframe 0 has {change_count} local changes")
    
    if change_count > 0:
        changes = memory.get_local_changes(0)
        print(f"First 5 changes:")
        for i, change in enumerate(changes[:5]):
            print(f"  [{i}] Type={change.modification_type}, "
                  f"ts={change.timestamp}, "
                  f"node_id={change.node_id if change.node_id else 'N/A'}")
    print()

def example_time_range_queries(memory):
    """Example 5: Time-range queries (with binary search)"""
    print("=" * 70)
    print("Example 5: Time-Range Queries")
    print("=" * 70)
    
    timestamps = memory.get_keyframe_timestamps()
    if len(timestamps) < 2:
        print("Skipped: Need at least 2 keyframes")
        return
    
    t_start = timestamps[0]
    t_end = timestamps[-1]
    
    # Get all events in range
    events = memory.get_events_between(t_start, t_end)
    print(f"Events in range [{t_start}, {t_end}]:")
    print(f"  Total: {len(events)}")
    
    # Get only changes (no keyframes)
    changes = memory.get_changes_between(t_start, t_end)
    print(f"  Changes only (no K): {len(changes)}")
    print(f"  Keyframes only: {len(events) - len(changes)}")
    print()

def example_node_history(memory):
    """Example 6: Node-centric queries"""
    print("=" * 70)
    print("Example 6: Node History")
    print("=" * 70)
    
    # Get first node from first keyframe
    kf = memory.get_keyframe(0)
    if not kf or len(kf.nodes_list) == 0:
        print("Skipped: No nodes in first keyframe")
        return
    
    first_node = kf.nodes_list[0]
    node_id = first_node['id']
    node_name = first_node['name']
    
    print(f"First node: ID={node_id}, name={node_name}")
    
    # Get history by ID
    history_by_id = memory.get_node_history(node_id)
    print(f"  History by ID: {len(history_by_id)} events")
    
    # Get history by name (uses index, O(1))
    history_by_name = memory.get_node_history_by_name(node_name)
    print(f"  History by name: {len(history_by_name)} events")
    
    # Show event types
    if len(history_by_id) > 0:
        print(f"  Event types:")
        types_seen = set()
        for event in history_by_id:
            if event.modification_type not in types_seen:
                print(f"    - {event.modification_type}")
                types_seen.add(event.modification_type)
    print()

def example_type_filtering(memory):
    """Example 7: Filter by modification type"""
    print("=" * 70)
    print("Example 7: Type Filtering")
    print("=" * 70)
    
    types_to_check = ["K", "MN", "MNA", "ME", "MEA", "DN", "DE"]
    
    print("Event counts by modification type:")
    for mod_type in types_to_check:
        events = memory.get_events_by_type(mod_type)
        if len(events) > 0:
            print(f"  {mod_type}: {len(events)}")
    print()

def example_exact_timestamp_query(memory):
    """Example 8: Query at exact timestamp"""
    print("=" * 70)
    print("Example 8: Exact Timestamp Query")
    print("=" * 70)
    
    timestamps = memory.get_keyframe_timestamps()
    if len(timestamps) == 0:
        print("Skipped: No keyframes")
        return
    
    exact_ts = timestamps[0]
    
    # Get event at exact timestamp
    event = memory.get_event_at_timestamp(exact_ts)
    if event:
        print(f"Event at {exact_ts}:")
        print(f"  Type: {event.modification_type}")
        print(f"  Nodes: {len(event.nodes_list)}")
        print(f"  Edges: {len(event.edges_list)}")
    
    # Get index (for further queries)
    event_idx = memory.get_event_index_at_timestamp(exact_ts)
    if event_idx is not None:
        print(f"  Index: {event_idx}")
    
    # Try non-existent timestamp
    fake_ts = exact_ts + 12345
    fake_event = memory.get_event_at_timestamp(fake_ts)
    if fake_event is None:
        print(f"Timestamp {fake_ts} not found (expected)")
    print()

def example_performance_notes(memory):
    """Example 9: Performance characteristics"""
    print("=" * 70)
    print("Example 9: Performance Notes")
    print("=" * 70)
    
    kf_count = memory.get_keyframe_count()
    
    print("Complexity analysis for this file:")
    print(f"  Keyframes: {kf_count}")
    
    log_k = math.log2(kf_count) if kf_count > 0 else 0
    print(f"  get_keyframe_index_at_time(): O(log k) = O({log_k:.1f}) iterations")
    
    print()
    print("Recommended patterns:")
    print("  1. Use get_keyframe_index_at_time() + get_local_changes()")
    print("     instead of get_keyframe_at_time() if you need local changes")
    print("  2. Use get_node_history_by_name() for O(1) lookup (pre-indexed)")
    print("  3. Use get_changes_between() over get_events_between() when")
    print("     you don't need keyframes (filters automatically)")
    print()

def example_performance_benchmarks(memory):
    """Example 10: Performance Benchmarks - Measure actual query times"""
    print("=" * 70)
    print("Example 10: Performance Benchmarks")
    print("=" * 70)
    print("Measuring read and query performance...\n")
    
    kf_count = memory.get_keyframe_count()
    timestamps = memory.get_keyframe_timestamps()
    
    # ---- BENCHMARK 1: get_keyframe_count ----
    print("  Benchmark 1: get_keyframe_count() [1000 iterations]")
    start = time.time()
    for i in range(1000):
        memory.get_keyframe_count()
    elapsed = (time.time() - start) * 1000  # Convert to ms
    µs_per_call = elapsed / 1000.0 * 1000  # Convert to microseconds
    print(f"    ├─ Total: {elapsed:.3f} ms")
    print(f"    └─ Per call: {µs_per_call:.3f} µs")
    
    # ---- BENCHMARK 2: get_keyframe ----
    print("\n  Benchmark 2: get_keyframe(idx) [500 iterations, various indices]")
    start = time.time()
    for i in range(500):
        idx = i % kf_count if kf_count > 0 else 0
        memory.get_keyframe(idx)
    elapsed = (time.time() - start) * 1000
    µs_per_call = elapsed / 500.0 * 1000
    print(f"    ├─ Total: {elapsed:.3f} ms")
    print(f"    └─ Per call: {µs_per_call:.3f} µs")
    
    # ---- BENCHMARK 3: get_keyframe_index_at_time ----
    print("\n  Benchmark 3: get_keyframe_index_at_time(ts) [500 iterations]")
    start = time.time()
    for i in range(500):
        ts = timestamps[i % len(timestamps)] if timestamps else 0
        memory.get_keyframe_index_at_time(ts)
    elapsed = (time.time() - start) * 1000
    µs_per_call = elapsed / 500.0 * 1000
    print(f"    ├─ Total: {elapsed:.3f} ms")
    print(f"    └─ Per call: {µs_per_call:.3f} µs (O(log k) binary search)")
    
    # ---- BENCHMARK 4: get_local_changes ----
    print("\n  Benchmark 4: get_local_changes(idx) [500 iterations]")
    start = time.time()
    for i in range(500):
        idx = i % kf_count if kf_count > 0 else 0
        memory.get_local_changes(idx)
    elapsed = (time.time() - start) * 1000
    µs_per_call = elapsed / 500.0 * 1000
    print(f"    ├─ Total: {elapsed:.3f} ms")
    print(f"    └─ Per call: {µs_per_call:.3f} µs")
    
    # ---- BENCHMARK 5: get_events_between ----
    print("\n  Benchmark 5: get_events_between(t0, t1) [200 iterations]")
    if len(timestamps) > 1:
        t_min = timestamps[0]
        t_max = timestamps[-1]
        start = time.time()
        for i in range(200):
            memory.get_events_between(t_min, t_max)
        elapsed = (time.time() - start) * 1000
        µs_per_call = elapsed / 200.0 * 1000
        print(f"    ├─ Total: {elapsed:.3f} ms")
        print(f"    └─ Per call: {µs_per_call:.3f} µs")
    else:
        print("    └─ Skipped (not enough timestamps)")
    
    # ---- BENCHMARK 6: get_node_history ----
    print("\n  Benchmark 6: get_node_history(node_id) [300 iterations]")
    start = time.time()
    for i in range(300):
        # Use various node IDs (assuming IDs from 100-300)
        memory.get_node_history(100 + (i % 20))
    elapsed = (time.time() - start) * 1000
    µs_per_call = elapsed / 300.0 * 1000
    print(f"    ├─ Total: {elapsed:.3f} ms")
    print(f"    └─ Per call: {µs_per_call:.3f} µs")
    
    # ---- BENCHMARK 7: get_events_by_type ----
    print("\n  Benchmark 7: get_events_by_type(type) [500 iterations]")
    types = ['K', 'M', 'C']
    start = time.time()
    for i in range(500):
        type_str = types[i % len(types)]
        memory.get_events_by_type(type_str)
    elapsed = (time.time() - start) * 1000
    µs_per_call = elapsed / 500.0 * 1000
    print(f"    ├─ Total: {elapsed:.3f} ms")
    print(f"    └─ Per call: {µs_per_call:.3f} µs")
    
    # ---- SUMMARY ----
    print("\n  Performance Summary:")
    print(f"    ├─ Status: ✓ All benchmarks completed")
    print(f"    ├─ Keyframes: {kf_count}")
    print(f"    └─ Note: Times vary by hardware and dataset size")
    print()

if __name__ == "__main__":
    # Check usage
    history_file = sys.argv[1] if len(sys.argv) > 1 else None
    if history_file is not None:
        history_file = sys.argv[1]
    
        print(f"\nEpisodic Memory API - Python Binding Examples")
        print(f"Using file: {history_file}\n")
        
        # Initialize
        memory = example_initialization(history_file)
        if memory is None:
            sys.exit(1)
        
        # Run examples
        example_keyframe_queries(memory)
        example_time_based_search(memory)
        example_local_changes(memory)
        example_time_range_queries(memory)
        example_node_history(memory)
        example_type_filtering(memory)
        example_exact_timestamp_query(memory)
        example_performance_notes(memory)
        example_performance_benchmarks(memory)
        
        print("=" * 70)
        print("All examples and benchmarks completed successfully! ✓")
        print("=" * 70)
    else:
        print("Usage: python3 example_usage.py <history_file>")
