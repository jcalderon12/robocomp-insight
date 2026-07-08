from dataclasses import dataclass
from typing import Optional, Set, Tuple

from rdflib import RDF

from src.ontology_mapping import (
    ACTION_FOLLOW,
    AGENT_ROBOT,
    DUL,
    PHYSICAL_OBJECT_BOTTLE,
    PHYSICAL_PLACE_ROOM,
)

# ! If we want to model causes relation when a candidate is accepted -> SOMA.causes, so for example: Event_bumpColission -> causes -> Event_observedLossofBottle
# ! With that, now we know that the bump collision caused the loss of the bottle, passing the causal_validator schema defined.

# ! We still need the implementation for explaining events (5W1H), but for now we only want to validate unexplained scenario.

Triple = Tuple[str, str, str]

@dataclass(frozen=True)
class ValidationResult:
    unexplained: bool # Flag for the compute method
    reason: str = ""
    retract: Optional[Triple] = None


class LiveCausalValidator:
    def __init__(self) -> None:
        self._bottle = str(PHYSICAL_OBJECT_BOTTLE)
        self._robot = str(AGENT_ROBOT)
        self._room = str(PHYSICAL_PLACE_ROOM)
        self._follow = str(ACTION_FOLLOW)

        self._has_location = str(DUL.hasLocation)
        self._has_participant = str(DUL.hasParticipant)
        self._bottle_type_triple = (self._bottle, str(RDF.type), str(DUL.PhysicalObject))

    def validate_delta(
        self,
        removed: Set[Triple],
        added: Set[Triple],
        current: Set[Triple],
    ) -> ValidationResult:
        """Validate only high-level structural changes from a state delta.

        Current policy: the loss of bottle location from robot requires
        explicit causal evidence. In its absence, the change is unexplained.
        """
        retract = (self._bottle, self._has_location, self._robot)
        if retract not in removed:
            return ValidationResult(unexplained=False)

        # If the bottle entity itself is gone from the ontology, the retraction is explained by entity deletion
        if self._bottle_type_triple not in current:
            return ValidationResult(
                unexplained=False,
                reason="Bottle entity removed from scene; retraction explained by entity deletion.",
                retract=retract,
            )

        moved_to_room = (self._bottle, self._has_location, self._room) in added or (
            self._bottle,
            self._has_location,
            self._room,
        ) in current

        if self._has_explicit_cause(current):
            return ValidationResult(
                unexplained=False,
                reason="Bottle location change has explicit causal evidence.",
                retract=retract,
            )

        if moved_to_room:
            return ValidationResult(
                unexplained=True,
                reason="Bottle moved from robot to room without explicit causal evidence.",
                retract=retract,
            )

        return ValidationResult(
            unexplained=True,
            reason="Bottle lost location on robot without explicit causal evidence.",
            retract=retract,
        )

    def _has_explicit_cause(self, triples: Set[Triple]) -> bool:
        """Keep causality strict and simple: follow alone does not explain a loss."""
        follow_active = (self._follow, self._has_participant, self._robot) in triples
        if not follow_active:
            return False

        # Future extension point for explicit causal assertions.
        return False
