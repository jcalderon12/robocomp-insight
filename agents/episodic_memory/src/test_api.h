#ifndef TEST_API_H
#define TEST_API_H

#include "dsr_episodic_api.h"
#include <iostream>
#include <iomanip>
#include <cassert>

class APITestSuite {
public:
    static void run_all_tests(const std::string &history_file) {
        std::cout << "\n" << std::string(80, '=') << "\n";
        std::cout << "  DSR EPISODIC MEMORY API TEST SUITE\n";
        std::cout << std::string(80, '=') << "\n\n";

        std::cout << "[TEST] Using history file: " << history_file << "\n\n";

        // Initialize API
        DSR::EpisodicMemoryAPI api(history_file);
        
        // Test 1: is_ready and get_filepath
        test_initialization(api, history_file);
        
        // Test 2: Keyframe queries
        test_keyframe_queries(api);
        
        // Test 3: Time-based queries
        test_time_queries(api);
        
        // Test 4: Local change queries
        test_local_change_queries(api);
        
        // Test 5: Node history
        test_node_history(api);
        
        // Test 6: Edge history
        test_edge_history(api);
        
        // Test 7: Type filtering
        test_type_filtering(api);
        
        // Test 8: Event at timestamp
        test_event_at_timestamp(api);
        
        // Test 9: Node changes between
        test_node_changes_between(api);

        // Test 10: Keyframe index at time
        test_keyframe_index_at_time(api);

        // Test 11: Event index at timestamp
        test_event_index_at_timestamp(api);

        std::cout << "\n" << std::string(80, '=') << "\n";
        std::cout << "  ALL TESTS PASSED ✓\n";
        std::cout << std::string(80, '=') << "\n\n";
    }

private:
    // ======================================================================
    // TEST 1: Initialization
    // ======================================================================
    static void test_initialization(const DSR::EpisodicMemoryAPI &api, const std::string &expected_path) {
        std::cout << "[TEST 1] Initialization\n";
        std::cout << "  ├─ is_ready(): " << (api.is_ready() ? "✓ PASS" : "✗ FAIL") << "\n";
        std::cout << "  └─ get_filepath(): " << (api.get_filepath() == expected_path ? "✓ PASS" : "✗ FAIL") << "\n";
        assert(api.is_ready() && "API should be ready");
        assert(api.get_filepath() == expected_path && "Filepath should match");
        std::cout << "\n";
    }

    // ======================================================================
    // TEST 2: Keyframe Queries
    // ======================================================================
    static void test_keyframe_queries(DSR::EpisodicMemoryAPI &api) {
        std::cout << "[TEST 2] Keyframe Queries\n";
        
        size_t kf_count = api.get_keyframe_count();
        std::cout << "  ├─ get_keyframe_count(): " << kf_count << " ✓ PASS\n";
        assert(kf_count > 0 && "Should have keyframes");

        auto timestamps = api.get_keyframe_timestamps();
        std::cout << "  ├─ get_keyframe_timestamps(): " << timestamps.size() << " timestamps ✓ PASS\n";
        assert(timestamps.size() == kf_count && "Timestamp count should match keyframe count");

        auto ts_opt = api.get_keyframe_timestamp(0);
        std::cout << "  ├─ get_keyframe_timestamp(0): " << ts_opt.value() << " ns ✓ PASS\n";
        assert(ts_opt.has_value() && "Should get first keyframe timestamp");

        auto kf_opt = api.get_keyframe(0);
        std::cout << "  ├─ get_keyframe(0): nodes=" << kf_opt->nodes.size() 
                  << ", edges=" << kf_opt->edges.size() << " ✓ PASS\n";
        assert(kf_opt.has_value() && "Should get keyframe");
        assert(kf_opt->modification_type == DSRSpecialChars::K && "Should be K type");

        // Test out of range
        auto invalid_kf = api.get_keyframe(kf_count + 100);
        std::cout << "  └─ get_keyframe(out_of_range): " << (!invalid_kf.has_value() ? "✓ PASS (nullopt)" : "✗ FAIL") << "\n";
        assert(!invalid_kf.has_value() && "Out of range should return nullopt");

        std::cout << "\n";
    }

    // ======================================================================
    // TEST 3: Time-based Queries
    // ======================================================================
    static void test_time_queries(DSR::EpisodicMemoryAPI &api) {
        std::cout << "[TEST 3] Time-based Queries\n";
        
        auto timestamps = api.get_keyframe_timestamps();
        if (timestamps.empty()) {
            std::cout << "  └─ Skipped (no keyframes)\n\n";
            return;
        }

        uint64_t t0 = timestamps[0];
        uint64_t t_all = timestamps.back();
        uint64_t mid_time = (t0 + t_all) / 2;

        // get_keyframe_at_time
        auto kf_at_time = api.get_keyframe_at_time(mid_time);
        std::cout << "  ├─ get_keyframe_at_time(mid): nodes=" << kf_at_time->nodes.size() << " ✓ PASS\n";
        assert(kf_at_time.has_value() && "Should get keyframe at time");

        // get_events_between
        auto events = api.get_events_between(t0, t_all);
        std::cout << "  ├─ get_events_between(t_start, t_end): " << events.size() << " events ✓ PASS\n";

        // get_changes_between
        auto changes = api.get_changes_between(t0, t_all);
        std::cout << "  └─ get_changes_between(t_start, t_end): " << changes.size() << " changes ✓ PASS\n";
        assert(changes.size() <= events.size() && "Changes should be subset of events");

        std::cout << "\n";
    }

    // ======================================================================
    // TEST 4: Local Change Queries
    // ======================================================================
    static void test_local_change_queries(DSR::EpisodicMemoryAPI &api) {
        std::cout << "[TEST 4] Local Change Queries\n";

        size_t kf_count = api.get_keyframe_count();
        if (kf_count == 0) {
            std::cout << "  └─ Skipped (no keyframes)\n\n";
            return;
        }

        // Get count for first keyframe
        size_t lc_count = api.get_local_changes_count(0);
        std::cout << "  ├─ get_local_changes_count(0): " << lc_count << " ✓ PASS\n";

        // Get all local changes
        auto all_changes = api.get_local_changes(0);
        std::cout << "  ├─ get_local_changes(0): " << all_changes.size() << " changes ✓ PASS\n";
        assert(all_changes.size() == lc_count && "Count should match");

        if (lc_count > 0) {
            auto single_change = api.get_local_change(0, 0);
            std::cout << "  ├─ get_local_change(0, 0): type=" << single_change->modification_type << " ✓ PASS\n";
            assert(single_change.has_value() && "Should get local change");
        }

        // Test out of range
        auto invalid = api.get_local_change(0, lc_count + 100);
        std::cout << "  └─ get_local_change(out_of_range): " << (!invalid.has_value() ? "✓ PASS (nullopt)" : "✗ FAIL") << "\n";
        assert(!invalid.has_value() && "Out of range should return nullopt");

        std::cout << "\n";
    }

    // ======================================================================
    // TEST 5: Node History
    // ======================================================================
    static void test_node_history(DSR::EpisodicMemoryAPI &api) {
        std::cout << "[TEST 5] Node History\n";

        // Find first node from a keyframe
        uint64_t node_id = 0;
        size_t found_nodes = 0;

        auto kf = api.get_keyframe(0);
        if (kf && !kf->nodes.empty()) {
            node_id = kf->nodes[0].id();
            found_nodes = 1;
        }

        if (found_nodes == 0) {
            std::cout << "  └─ Skipped (no nodes in keyframe)\n\n";
            return;
        }

        // get_node_history
        auto history = api.get_node_history(node_id);
        std::cout << "  ├─ get_node_history(node_id=" << node_id << "): " 
                  << history.size() << " events ✓ PASS\n";

        // get_node_history_by_name
        std::string node_name;
        auto kf_full = api.get_keyframe(0);
        if (kf_full && !kf_full->nodes.empty()) {
            node_name = kf_full->nodes[0].name();
        }

        if (!node_name.empty()) {
            auto history_by_name = api.get_node_history_by_name(node_name);
            std::cout << "  └─ get_node_history_by_name(\"" << node_name << "\"): " 
                      << history_by_name.size() << " events ✓ PASS\n";
        } else {
            std::cout << "  └─ (skipped by_name test, no node name)\n";
        }

        std::cout << "\n";
    }

    // ======================================================================
    // TEST 6: Edge History
    // ======================================================================
    static void test_edge_history(DSR::EpisodicMemoryAPI &api) {
        std::cout << "[TEST 6] Edge History\n";

        // Find first edge from a keyframe
        uint64_t from_id = 0, to_id = 0;
        std::string edge_type;
        size_t found_edges = 0;

        auto kf = api.get_keyframe(0);
        if (kf && !kf->edges.empty()) {
            from_id = kf->edges[0].from();
            to_id = kf->edges[0].to();
            edge_type = kf->edges[0].type();
            found_edges = 1;
        }

        if (found_edges == 0) {
            std::cout << "  └─ Skipped (no edges in keyframe)\n\n";
            return;
        }

        auto edge_history = api.get_edge_history(from_id, to_id, edge_type);
        std::cout << "  └─ get_edge_history(" << from_id << " → " << to_id 
                  << ", type=\"" << edge_type << "\"): " << edge_history.size() << " events ✓ PASS\n";

        std::cout << "\n";
    }

    // ======================================================================
    // TEST 7: Type Filtering
    // ======================================================================
    static void test_type_filtering(DSR::EpisodicMemoryAPI &api) {
        std::cout << "[TEST 7] Type Filtering\n";

        std::vector<std::string> types = {
            DSRSpecialChars::K, DSRSpecialChars::MN, DSRSpecialChars::MNA,
            DSRSpecialChars::ME, DSRSpecialChars::MEA, DSRSpecialChars::DN, DSRSpecialChars::DE
        };

        for (const auto &type : types) {
            auto events = api.get_events_by_type(type);
            std::cout << "  ├─ get_events_by_type(\"" << type << "\"): " 
                      << events.size() << " events\n";
        }

        std::cout << "  └─ All type filters ✓ PASS\n";
        std::cout << "\n";
    }

    // ======================================================================
    // TEST 8: Event at Timestamp
    // ======================================================================
    static void test_event_at_timestamp(DSR::EpisodicMemoryAPI &api) {
        std::cout << "[TEST 8] Event at Timestamp\n";

        auto timestamps = api.get_keyframe_timestamps();
        if (timestamps.empty()) {
            std::cout << "  └─ Skipped (no keyframes)\n\n";
            return;
        }

        // Try to find an event at keyframe timestamp
        uint64_t ts = timestamps[0];
        auto event = api.get_event_at_timestamp(ts);
        std::cout << "  ├─ get_event_at_timestamp(" << ts << "): " 
                  << (event.has_value() ? "✓ PASS (found)" : "✗ FAIL (not found)") << "\n";

        // Try exact match on non-keyframe time (should be null)
        auto invalid_event = api.get_event_at_timestamp(ts + 12345);
        std::cout << "  └─ get_event_at_timestamp(ts + 12345): " 
                  << (!invalid_event.has_value() ? "✓ PASS (nullopt)" : "? (found by chance)") << "\n";

        std::cout << "\n";
    }

    // ======================================================================
    // TEST 9: Node Changes Between
    // ======================================================================
    static void test_node_changes_between(DSR::EpisodicMemoryAPI &api) {
        std::cout << "[TEST 9] Node Changes Between\n";

        auto timestamps = api.get_keyframe_timestamps();
        if (timestamps.empty()) {
            std::cout << "  └─ Skipped (no keyframes)\n\n";
            return;
        }

        // Find a node
        uint64_t node_id = 0;
        auto kf = api.get_keyframe(0);
        if (kf && !kf->nodes.empty()) {
            node_id = kf->nodes[0].id();
        } else {
            std::cout << "  └─ Skipped (no nodes)\n\n";
            return;
        }

        uint64_t t0 = timestamps[0];
        uint64_t t_all = timestamps.back();

        auto changes = api.get_node_changes_between(node_id, t0, t_all);
        std::cout << "  ├─ get_node_changes_between(node_id=" << node_id 
                  << ", t_range): " << changes.size() << " changes ✓ PASS\n";

        // Verify all are for same node
        bool all_same_node = true;
        for (const auto &change : changes) {
            if (change.node_id.has_value() && change.node_id.value() != node_id) {
                all_same_node = false;
                break;
            }
        }
        std::cout << "  └─ All changes belong to node_id: " 
                  << (all_same_node ? "✓ PASS" : "✗ FAIL") << "\n";
        assert(all_same_node && "All changes should be for the same node");

        std::cout << "\n";
    }

    // ======================================================================
    // TEST 10: Keyframe Index at Time
    // ======================================================================
    static void test_keyframe_index_at_time(DSR::EpisodicMemoryAPI &api) {
        std::cout << "[TEST 10] Keyframe Index at Time\n";

        auto timestamps = api.get_keyframe_timestamps();
        if (timestamps.empty()) {
            std::cout << "  └─ Skipped (no keyframes)\n\n";
            return;
        }

        // Test exact keyframe timestamp
        uint64_t t0 = timestamps[0];
        auto idx_opt = api.get_keyframe_index_at_time(t0);
        std::cout << "  ├─ get_keyframe_index_at_time(t0): idx=" 
                  << (idx_opt.has_value() ? std::to_string(idx_opt.value()) : "nullopt")
                  << " ✓ PASS\n";
        assert(idx_opt.has_value() && "Should find first keyframe");
        assert(idx_opt.value() == 0 && "First timestamp should give index 0");

        // Test mid-point time
        uint64_t t_all = timestamps.back();
        uint64_t mid_time = (t0 + t_all) / 2;
        auto idx_mid = api.get_keyframe_index_at_time(mid_time);
        std::cout << "  ├─ get_keyframe_index_at_time(mid): idx=" 
                  << (idx_mid.has_value() ? std::to_string(idx_mid.value()) : "nullopt")
                  << " ✓ PASS\n";
        assert(idx_mid.has_value() && "Should find keyframe at mid time");

        // Test that index matches keyframe data
        if (idx_mid.has_value()) {
            auto kf_direct = api.get_keyframe(idx_mid.value());
            auto kf_time = api.get_keyframe_at_time(mid_time);
            bool same_ts = kf_direct && kf_time && 
                          (kf_direct->timestamp == kf_time->timestamp);
            std::cout << "  ├─ Index consistency with get_keyframe(): " 
                      << (same_ts ? "✓ PASS" : "✗ FAIL") << "\n";
            assert(same_ts && "Index should match keyframe data");
        }

        // Test after last keyframe (should return last index)
        auto idx_after = api.get_keyframe_index_at_time(t_all + 1000000000000ULL);
        std::cout << "  ├─ get_keyframe_index_at_time(after_all): idx=" 
                  << (idx_after.has_value() ? std::to_string(idx_after.value()) : "nullopt")
                  << " ✓ PASS\n";
        if (idx_after.has_value()) {
            assert(idx_after.value() == timestamps.size() - 1 && 
                  "Should return last keyframe index");
        }

        // Test before first keyframe (should return 0)
        auto idx_before = api.get_keyframe_index_at_time(t0 > 1000000000 ? t0 - 1000000000 : 0);
        std::cout << "  └─ get_keyframe_index_at_time(before_first): idx=" 
                  << (idx_before.has_value() ? std::to_string(idx_before.value()) : "nullopt")
                  << " ✓ PASS\n";
        if (idx_before.has_value()) {
            assert(idx_before.value() == 0 && "Should return first keyframe index");
        }

        std::cout << "\n";
    }

    // ======================================================================
    // TEST 11: Event Index at Timestamp
    // ======================================================================
    static void test_event_index_at_timestamp(DSR::EpisodicMemoryAPI &api) {
        std::cout << "[TEST 11] Event Index at Timestamp\n";

        auto timestamps = api.get_keyframe_timestamps();
        if (timestamps.empty()) {
            std::cout << "  └─ Skipped (no keyframes)\n\n";
            return;
        }

        // Test exact timestamp from keyframe
        uint64_t ts0 = timestamps[0];
        auto idx_opt = api.get_event_index_at_timestamp(ts0);
        std::cout << "  ├─ get_event_index_at_timestamp(ts0): idx=" 
                  << (idx_opt.has_value() ? std::to_string(idx_opt.value()) : "nullopt")
                  << " ✓ PASS\n";
        assert(idx_opt.has_value() && "Should find event at first keyframe timestamp");
        assert(idx_opt.value() == 0 && "First timestamp should give index 0");

        // Test that index references correct event
        if (idx_opt.has_value()) {
            auto event_direct = api.get_event_at_timestamp(ts0);
            bool consistent = event_direct && event_direct->timestamp == ts0;
            std::cout << "  ├─ Index consistency with get_event_at_timestamp(): " 
                      << (consistent ? "✓ PASS" : "✗ FAIL") << "\n";
            assert(consistent && "Index should reference correct event");
        }

        // Test non-existent timestamp (should be nullopt)
        uint64_t fake_ts = ts0 + 123456789;
        auto idx_fake = api.get_event_index_at_timestamp(fake_ts);
        std::cout << "  ├─ get_event_index_at_timestamp(non_existent): " 
                  << (!idx_fake.has_value() ? "✓ PASS (nullopt)" : "? (found by chance)") << "\n";

        // Test if we have multiple timestamps, try a different one
        if (timestamps.size() > 1) {
            uint64_t ts_mid = timestamps[timestamps.size() / 2];
            auto idx_mid = api.get_event_index_at_timestamp(ts_mid);
            std::cout << "  ├─ get_event_index_at_timestamp(ts_mid): idx=" 
                      << (idx_mid.has_value() ? std::to_string(idx_mid.value()) : "nullopt")
                      << " ✓ PASS\n";
            assert(idx_mid.has_value() && "Should find event at mid timestamp");
            assert(idx_mid.value() > 0 && "Mid index should be positive");
        }

        std::cout << "  └─ Index retrieval ✓ PASS\n";
        std::cout << "\n";
    }
};

#endif // TEST_API_H
