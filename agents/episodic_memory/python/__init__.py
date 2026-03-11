"""
Episodic Memory API - Python Package

Provides read-only access to historical DSR graph data with optimized
binary search and indexing capabilities.

Quick Start:
    >>> import episodic_memory_api as api
    >>> memory = api.EpisodicMemoryAPI("history.txt")
    >>> if memory.is_ready():
    ...     print(f"Keyframes: {memory.get_keyframe_count()}")
    ...     kf = memory.get_keyframe(0)
    ...     print(f"First keyframe: {kf.timestamp} ns")

Key Classes:
    - EpisodicMemoryAPI: Main query interface
    - DSRData: Event data returned by queries

Performance:
    - Keyframe lookup: O(log k) where k = keyframe count
    - Time-range queries: O(log n + m) where n = event count, m = result size
    - Node name lookup: O(1) using pre-built index
    - All operations happen in C++ (no GIL contention)

Documentation:
    See PYTHON_BINDING_GUIDE.md for detailed usage examples and API reference.
"""

from episodic_memory_api import (
    EpisodicMemoryAPI,
    DSRData,
)

__version__ = "1.0.0"
__author__ = "RoboComp Team"
__all__ = ["EpisodicMemoryAPI", "DSRData"]
