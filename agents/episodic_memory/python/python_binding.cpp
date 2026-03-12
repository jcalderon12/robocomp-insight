//
// PyBind11 wrapper for Episodic Memory API
// Following RoboComp Cortex pattern (cortex/python-wrapper)
//
// Compile with: cmake .. && make -j8 && sudo make install
//

#include "dsr_episodic_api.h"
#include <iostream>
#include <optional>

// Prevent Qt macro conflicts with pybind11
#pragma push_macro("slots")
#undef slots

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#pragma pop_macro("slots")

namespace py = pybind11;
using namespace pybind11::literals;

// ============================================================
// Helper function to convert DSR::Node to Python dict
// ============================================================
py::dict node_to_dict(const DSR::Node &node) {
    py::dict d;
    d["id"] = node.id();
    d["name"] = node.name();
    d["type"] = node.type();
    d["agent_id"] = node.agent_id();
    return d;
}

// ============================================================
// Helper function to convert DSR::Edge to Python dict
// ============================================================
py::dict edge_to_dict(const DSR::Edge &edge) {
    py::dict d;
    d["from"] = edge.from();
    d["to"] = edge.to();
    d["type"] = edge.type();
    d["agent_id"] = edge.agent_id();
    return d;
}

// ============================================================
// PYBIND11 Module Definition
// ============================================================

PYBIND11_MODULE(episodic_memory_api, m) {
    m.doc() = "Episodic Memory API for DSR graph history queries. "
              "Provides optimized binary search and indexing for historical graph data.";

    // ============================================================
    // DSRData struct binding
    // ============================================================
    py::class_<DSR::DSRData>(m, "DSRData", "Event data from episodic memory")
            .def(py::init<>())
            .def_readwrite("timestamp", &DSR::DSRData::timestamp,
                          "Nanosecond timestamp of the event")
            .def_readwrite("modification_type", &DSR::DSRData::modification_type,
                          "Event type: K (keyframe), MN, MNA, ME, MEA, DN, DE")
            .def_readwrite("node_id", &DSR::DSRData::node_id,
                          "Node ID (for MN, MNA, DN types)")
            .def_readwrite("edge_from_id", &DSR::DSRData::edge_from_id,
                          "Source node ID (for ME, MEA, DE types)")
            .def_readwrite("edge_to_id", &DSR::DSRData::edge_to_id,
                          "Target node ID (for ME, MEA, DE types)")
            .def_readwrite("type", &DSR::DSRData::type,
                          "Node or edge type string")
            .def_readwrite("node_name", &DSR::DSRData::node_name,
                          "Node name (MN events only)")
            .def_readwrite("attributes", &DSR::DSRData::attributes,
                          "Dictionary of attributes")
            .def_property_readonly("nodes_list", [](const DSR::DSRData &self) {
                py::list nodes_py;
                for (const auto &node : self.nodes) {
                    nodes_py.append(node_to_dict(node));
                }
                return nodes_py;
            }, "List of node dictionaries (K events only)")
            .def_property_readonly("edges_list", [](const DSR::DSRData &self) {
                py::list edges_py;
                for (const auto &edge : self.edges) {
                    edges_py.append(edge_to_dict(edge));
                }
                return edges_py;
            }, "List of edge dictionaries (K events only)")
            .def_readwrite("nodes", &DSR::DSRData::nodes,
                          "[DEPRECATED] Use nodes_list instead")
            .def_readwrite("edges", &DSR::DSRData::edges,
                          "[DEPRECATED] Use edges_list instead")
            .def("__repr__", [](const DSR::DSRData &self) {
                std::string repr = "DSRData(timestamp=" + std::to_string(self.timestamp) +
                                   ", type=" + self.modification_type;
                if (self.node_id.has_value())
                    repr += ", node_id=" + std::to_string(self.node_id.value());
                if (self.edge_from_id.has_value())
                    repr += ", edge=" + std::to_string(self.edge_from_id.value()) + "->" +
                           std::to_string(self.edge_to_id.value());
                repr += ")";
                return repr;
            });

    // ============================================================
    // EpisodicMemoryAPI class binding
    // ============================================================
    py::class_<DSR::EpisodicMemoryAPI>(m, "EpisodicMemoryAPI",
            "Query interface for historical DSR graph data with O(log n) binary search")
            .def(py::init<const std::string &>(),
                 py::arg("filepath"),
                 "Initialize from history file. File is indexed immediately.")

            // -------- Status methods --------
            .def("is_ready", &DSR::EpisodicMemoryAPI::is_ready,
                 "bool: Check if file was indexed successfully")
            .def("get_filepath", &DSR::EpisodicMemoryAPI::get_filepath,
                 py::return_value_policy::reference,
                 "str: Get the currently indexed file path")

            // -------- Keyframe queries --------
            .def("get_keyframe_count", &DSR::EpisodicMemoryAPI::get_keyframe_count,
                 "int: Get total number of keyframes in file")
            .def("get_keyframe_timestamps", &DSR::EpisodicMemoryAPI::get_keyframe_timestamps,
                 "list[int]: Get all keyframe timestamps in nanoseconds")
            .def("get_keyframe_timestamp", &DSR::EpisodicMemoryAPI::get_keyframe_timestamp,
                 py::arg("keyframe_idx"),
                 "int | None: Get timestamp of specific keyframe (or None if out of range)")
            .def("get_keyframe", &DSR::EpisodicMemoryAPI::get_keyframe,
                 py::arg("keyframe_idx"),
                 "DSRData | None: Get full keyframe with all nodes and edges")
            .def("get_keyframe_at_time", &DSR::EpisodicMemoryAPI::get_keyframe_at_time,
                 py::arg("timestamp"),
                 "DSRData | None: Get keyframe closest to and <= given timestamp")
            .def("get_keyframe_index_at_time", &DSR::EpisodicMemoryAPI::get_keyframe_index_at_time,
                 py::arg("timestamp"),
                 "int | None: Get index of keyframe at or before given timestamp")

            // -------- Local change queries --------
            .def("get_local_changes_count", &DSR::EpisodicMemoryAPI::get_local_changes_count,
                 py::arg("keyframe_idx"),
                 "int: Get count of local changes for a keyframe")
            .def("get_local_changes", &DSR::EpisodicMemoryAPI::get_local_changes,
                 py::arg("keyframe_idx"),
                 "list[DSRData]: Get all local changes for keyframe")
            .def("get_local_change", &DSR::EpisodicMemoryAPI::get_local_change,
                 py::arg("keyframe_idx"), py::arg("change_idx"),
                 "DSRData | None: Get single local change by keyframe and change indices")

            // -------- Time-range queries (with binary search) --------
            .def("get_events_between", &DSR::EpisodicMemoryAPI::get_events_between,
                 py::arg("t_start"), py::arg("t_end"),
                 "list[DSRData]: Get all events (keyframes + changes) in [t_start, t_end]")
            .def("get_changes_between", &DSR::EpisodicMemoryAPI::get_changes_between,
                 py::arg("t_start"), py::arg("t_end"),
                 "list[DSRData]: Get only local changes (no keyframes) in [t_start, t_end]")
            .def("get_event_at_timestamp", &DSR::EpisodicMemoryAPI::get_event_at_timestamp,
                 py::arg("timestamp"),
                 "DSRData | None: Get event at exact timestamp (requires exact match)")
            .def("get_event_index_at_timestamp", &DSR::EpisodicMemoryAPI::get_event_index_at_timestamp,
                 py::arg("timestamp"),
                 "int | None: Get index of event at exact timestamp")

            // -------- Node-centric queries --------
            .def("get_node_history", &DSR::EpisodicMemoryAPI::get_node_history,
                 py::arg("node_id"),
                 "list[DSRData]: Get all events referencing a node (MN, MNA, DN types)")
            .def("get_node_history_by_name", &DSR::EpisodicMemoryAPI::get_node_history_by_name,
                 py::arg("node_name"),
                 "list[DSRData]: Get all events for node with given name (O(1) index lookup)")
            .def("get_node_changes_between", &DSR::EpisodicMemoryAPI::get_node_changes_between,
                 py::arg("node_id"), py::arg("t_start"), py::arg("t_end"),
                 "list[DSRData]: Get all changes to specific node in time range")

            // -------- Edge-centric queries --------
            .def("get_edge_history", &DSR::EpisodicMemoryAPI::get_edge_history,
                 py::arg("from_id"), py::arg("to_id"), py::arg("edge_type"),
                 "list[DSRData]: Get all events for specific edge (ME, MEA, DE types)")

            // -------- Type-based filtering --------
            .def("get_events_by_type", &DSR::EpisodicMemoryAPI::get_events_by_type,
                 py::arg("mod_type"),
                 "list[DSRData]: Get all events of specific modification type\n"
                 "  'K': Keyframe\n"
                 "  'MN': Modify node\n"
                 "  'MNA': Modify node attributes\n"
                 "  'ME': Modify edge\n"
                 "  'MEA': Modify edge attributes\n"
                 "  'DN': Delete node\n"
                 "  'DE': Delete edge");
}
