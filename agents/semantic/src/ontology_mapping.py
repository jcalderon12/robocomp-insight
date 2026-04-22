from dataclasses import dataclass, field
from rdflib import Graph, Namespace, RDF, URIRef

DUL = Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#") # SOMA?
INSIGHT = Namespace("http://insight.local/instances#")



# Initialization logic:        
# Entities: room, robot, person, bottle
# Relations
# robot -> has location -> room
# person -> has location -> room
# bottle -> has location -> robot

# After mission start:
# Entities: follow
# Relations:
# follow -> has participant -> robot
# follow -> has participant -> person

# Bottle slips from robot tray:
# Remove relation: bottle -> has location -> robot
# Add relation: bottle -> has location -> room


AGENT_PERSON = INSIGHT.Agent_Person
AGENT_ROBOT = INSIGHT.Agent_Robot
PHYSICAL_OBJECT_BOTTLE = INSIGHT.PhysicalObject_Bottle
PHYSICAL_PLACE_ROOM = INSIGHT.PhysicalPlace_Room
ACTION_FOLLOW = INSIGHT.Action_Follow

@dataclass
class SemanticState:
    triples: tuple[tuple[str, str, str]] = field(default_factory=set)

    @property
    def signature(self) -> tuple[tuple[str, str, str], ...]:
        return tuple(sorted(self.triples))
    
    def clear(self) -> None:
        self.triples.clear()

    def add(self, subj: str, pred: str, obj: str) -> None:
        self.triples.add((subj, pred, obj))

    def remove(self, subj: str, pred: str, obj: str) -> None:
        self.triples.discard((subj, pred, obj))

    

    def to_turtle(self) -> str:
        graph = Graph()
        graph.bind("dul", DUL)
        graph.bind("insight", INSIGHT)
        for subj, pred, obj in self.triples:
            graph.add((URIRef(subj), URIRef(pred), URIRef(obj)))
        return graph.serialize(format="turtle")
    

class DSRSemanticWrapper:
    def __init__(self) -> None:
        self.state = SemanticState()

    def get_state(self) -> SemanticState:
        return self.state

    def initialize_from_dsr(self, dsr_graph) -> None:
        self.state.clear()
        self.sync_room(dsr_graph)
        self.sync_robot(dsr_graph)
        self.sync_person(dsr_graph)
        self.sync_bottle(dsr_graph)
        self.sync_follow(dsr_graph)


    def updated_node(self, dsr_graph, node_type: str) -> bool:
        return self.sync_for_node_type(dsr_graph, node_type) 
    
    def deleted_node(self, dsr_graph, node_type: str) -> bool:
        return self.sync_for_node_type(dsr_graph, node_type)

    def updated_edge(self, dsr_graph, edge_type: str) -> bool:
        return self.sync_for_edge_type(dsr_graph, edge_type)
    
    def deleted_edge(self, dsr_graph, edge_type: str) -> bool:
        return self.sync_for_edge_type(dsr_graph, edge_type)
    


    def sync_for_node_type(self, dsr_graph, node_type: str) -> bool:
        node_type = (node_type or "").lower()
        if node_type == "room":
            self.sync_room(dsr_graph)
            self.sync_robot(dsr_graph)
            self.sync_person(dsr_graph)
            self.sync_bottle(dsr_graph)
            return True
        
        if node_type == "robot":
            self.sync_robot(dsr_graph)
            self.sync_bottle(dsr_graph)
            self.sync_follow(dsr_graph)
            return True

        if node_type == "person":
            self.sync_person(dsr_graph)
            self.sync_follow(dsr_graph)
            return True

        if node_type == "bottle":
            self.sync_bottle(dsr_graph)
            return True

        return False




    def sync_for_edge_type(self, dsr_graph, edge_type: str) -> bool:
        edge_type = (edge_type or "").upper()
        if edge_type == "RT":
            self.sync_bottle(dsr_graph)
            return True
        
        if edge_type == "TARGET":
            self.sync_follow(dsr_graph)
            return True
        
        return False



    def sync_room(self, dsr_graph) -> None:
        room_existe = dsr_graph.get_node("room") is not None
        self.set_fact(room_existe, str(PHYSICAL_PLACE_ROOM), str(RDF.type), str(DUL.PhysicalPlace))


    def sync_robot(self, dsr_graph) -> None:
        room_existe = dsr_graph.get_node("room") is not None
        robot_existe = dsr_graph.get_node("robot") is not None

        self.set_fact(robot_existe, str(AGENT_ROBOT), str(RDF.type), str(DUL.PhysicalAgent))
        self.set_fact(robot_existe and room_existe, str(AGENT_ROBOT), str(DUL.hasLocation), str(PHYSICAL_PLACE_ROOM))


    def sync_person(self, dsr_graph) -> None:
        room_existe = dsr_graph.get_node("room") is not None
        person_existe = dsr_graph.get_node("person") is not None

        self.set_fact(person_existe, str(AGENT_PERSON), str(RDF.type), str(DUL.PhysicalAgent))
        self.set_fact(person_existe and room_existe, str(AGENT_PERSON), str(DUL.hasLocation), str(PHYSICAL_PLACE_ROOM))

    def sync_bottle(self, dsr_graph) -> None:
        room_existe = dsr_graph.get_node("room") is not None
        robot_existe = dsr_graph.get_node("robot") is not None
        bottle_existe = dsr_graph.get_node("bottle") is not None

        self.set_fact(bottle_existe, str(PHYSICAL_OBJECT_BOTTLE), str(RDF.type), str(DUL.PhysicalObject))

        bottle_on_robot = (bottle_existe and robot_existe and self._has_edge(dsr_graph, "robot", "bottle", "RT"))
        bottle_in_room = (bottle_existe and room_existe and self._has_edge(dsr_graph, "room", "bottle", "RT"))

        self.set_fact(bottle_on_robot and robot_existe, str(PHYSICAL_OBJECT_BOTTLE), str(DUL.hasLocation), str(AGENT_ROBOT))
        self.set_fact(bottle_in_room and room_existe, str(PHYSICAL_OBJECT_BOTTLE), str(DUL.hasLocation), str(PHYSICAL_PLACE_ROOM))

    def sync_follow(self, dsr_graph) -> None:
        robot_existe = dsr_graph.get_node("robot") is not None
        person_existe = dsr_graph.get_node("person") is not None
        follow_existe = (robot_existe and person_existe and self._has_edge(dsr_graph, "robot", "person", "TARGET"))

        self.set_fact(follow_existe, str(ACTION_FOLLOW), str(RDF.type), str(DUL.Action))
        self.set_fact(follow_existe and robot_existe, str(ACTION_FOLLOW), str(DUL.hasParticipant), str(AGENT_ROBOT))
        self.set_fact(follow_existe and person_existe, str(ACTION_FOLLOW), str(DUL.hasParticipant), str(AGENT_PERSON))



    def set_fact(self, enabled: bool, subj: str, pred: str, obj: str) -> None:
        if enabled:
            self.state.add(subj, pred, obj)
        else:
            self.state.remove(subj, pred, obj)
 


    def _has_edge(self, dsr_graph, from_node_id: str, to_node_id: str, edge_type: str) -> bool:
        from_node = dsr_graph.get_node(from_node_id)
        to_node = dsr_graph.get_node(to_node_id)
        if from_node is None or to_node is None:
            return False
        return dsr_graph.get_edge(from_node.id, to_node.id, edge_type) is not None