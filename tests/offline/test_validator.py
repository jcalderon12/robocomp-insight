"""Offline test: LiveCausalValidator decision table (no robot required).

Run from the repo root:  python3 tests/offline/test_validator.py
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "agents" / "semantic"))

from rdflib import RDF

from src.live_causal_validator import LiveCausalValidator
from src.ontology_mapping import AGENT_ROBOT, DUL, PHYSICAL_OBJECT_BOTTLE, PHYSICAL_PLACE_ROOM


def main():
    validator = LiveCausalValidator()
    bottle, robot, room = str(PHYSICAL_OBJECT_BOTTLE), str(AGENT_ROBOT), str(PHYSICAL_PLACE_ROOM)
    has_location = str(DUL.hasLocation)
    retract = (bottle, has_location, robot)
    bottle_type = (bottle, str(RDF.type), str(DUL.PhysicalObject))

    # Bottle moves from robot to room with no causal evidence -> unexplained
    result = validator.validate_delta(
        removed={retract},
        added={(bottle, has_location, room)},
        current={bottle_type, (bottle, has_location, room)},
    )
    assert result.unexplained, result

    # Bottle entity deleted entirely -> explained by entity deletion
    result = validator.validate_delta(removed={retract, bottle_type}, added=set(), current=set())
    assert not result.unexplained and "entity deletion" in result.reason, result

    # Unrelated delta -> explained
    result = validator.validate_delta(removed=set(), added=set(), current={bottle_type})
    assert not result.unexplained, result

    print("test_validator OK")


if __name__ == "__main__":
    main()
