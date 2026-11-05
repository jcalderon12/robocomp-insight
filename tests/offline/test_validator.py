"""Offline test: LiveCausalValidator decision table (no robot required).

Run from the repo root:  python3 tests/offline/test_validator.py
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "agents" / "semantic"))

from rdflib import RDF

import types

from src.live_causal_validator import LiveCausalValidator
from src.ontology_mapping import AGENT_ROBOT, DUL, PHYSICAL_OBJECT_BOTTLE, PHYSICAL_PLACE_ROOM, DSRSemanticWrapper

LOST = "Bottle lost location on robot without explicit causal evidence."


class FakeDSR:
    """The nodes and RT edges the mirror reads."""

    def __init__(self, *edges):
        self.nodes = {name: types.SimpleNamespace(id=i) for i, name in enumerate(("room", "robot", "bottle"), 1)}
        self.edges = {(self.nodes[a].id, self.nodes[b].id, "RT") for a, b in edges}

    def get_node(self, name):
        return self.nodes.get(name)

    def get_edge(self, from_id, to_id, edge_type):
        return True if (from_id, to_id, edge_type) in self.edges else None


def check_mirror_never_places_a_lost_bottle(bottle, robot, room, has_location):
    """The bottle is on the robot while the robot->bottle RT edge exists; once lost it has no location,
    even if a DSR agent hangs it from the room (concept_bottle did until 2026-05-20)."""
    mirror = DSRSemanticWrapper()
    mirror.sync_bottle(FakeDSR(("robot", "bottle")))
    assert (bottle, has_location, robot) in mirror.get_state().triples
    mirror.sync_bottle(FakeDSR(("room", "bottle")))
    triples = mirror.get_state().triples
    assert (bottle, has_location, robot) not in triples and (bottle, has_location, room) not in triples, triples


def main():
    validator = LiveCausalValidator()
    bottle, robot, room = str(PHYSICAL_OBJECT_BOTTLE), str(AGENT_ROBOT), str(PHYSICAL_PLACE_ROOM)
    has_location = str(DUL.hasLocation)
    retract = (bottle, has_location, robot)
    bottle_type = (bottle, str(RDF.type), str(DUL.PhysicalObject))

    # The bottle leaves the robot with no causal evidence -> unexplained, wherever it went: nothing
    # is added (production since 2026-05-20), or an older mirror placed it in the room.
    result = validator.validate_delta(removed={retract}, added=set(), current={bottle_type})
    assert result.unexplained and result.reason == LOST, result
    result = validator.validate_delta(
        removed={retract},
        added={(bottle, has_location, room)},
        current={bottle_type, (bottle, has_location, room)},
    )
    assert result.unexplained and result.reason == LOST, result

    # Bottle entity deleted entirely -> explained by entity deletion
    result = validator.validate_delta(removed={retract, bottle_type}, added=set(), current=set())
    assert not result.unexplained and "entity deletion" in result.reason, result

    # Unrelated delta -> explained
    result = validator.validate_delta(removed=set(), added=set(), current={bottle_type})
    assert not result.unexplained, result

    check_mirror_never_places_a_lost_bottle(bottle, robot, room, has_location)
    print("test_validator OK")


if __name__ == "__main__":
    main()
