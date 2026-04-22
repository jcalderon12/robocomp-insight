from dataclasses import dataclass
from typing import Any, Iterable
import requests
from rdflib import Graph


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _as_float(value: Any, default: float) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except (ValueError, TypeError):
        return default




@dataclass(frozen=True)
class GraphDBConfig:
    enabled: bool
    endpoint: str
    repository: str
    named_graph: str
    timeout_seconds: float
    user: str = ""
    password: str = ""


    @classmethod
    def from_config(cls, config_data: dict[str, Any]) -> 'GraphDBConfig':
        graphdb_cfg = config_data.get("graphDB", {})
        return cls(
            enabled=_as_bool(graphdb_cfg.get("Enabled"), True),
            endpoint=str(graphdb_cfg.get("Endpoint", "http://localhost:7200")).strip(),
            repository=str(graphdb_cfg.get("Repository", "insight_prod")).strip(),
            named_graph=str(graphdb_cfg.get("NamedGraph", "urn:insight:semantic:live")).strip(),
            timeout_seconds=_as_float(graphdb_cfg.get("TimeoutSeconds"), 5.0),
            user=str(graphdb_cfg.get("User", "")).strip(),
            password=str(graphdb_cfg.get("Password", "")).strip()
        )
    

    @property
    def statements_url(self) -> str:
        endpoint = self.endpoint.rstrip('/')
        return f"{endpoint}/repositories/{self.repository}/statements"
    



class GraphDBClient:
    def __init__(self,  config: GraphDBConfig):
        self.config = config
        self.session = requests.Session()

    def get_graph_triples(self) -> set[tuple[str, str, str]]:
        response = self.session.get(
            self.config.statements_url,
            params={"context": f"<{self.config.named_graph}>"},
            headers={"Accept": "application/n-triples"},
            auth=self._auth(),
            timeout=self.config.timeout_seconds,
        )
        response.raise_for_status()

        payload = response.text.strip()
        if not payload:
            return set()

        graph = Graph()
        graph.parse(data=payload, format="nt")
        return {(str(subj), str(pred), str(obj)) for subj, pred, obj in graph}

    def apply_delta(
        self,
        *,
        added: Iterable[tuple[str, str, str]],
        removed: Iterable[tuple[str, str, str]],
    ) -> None:
        removed = tuple(sorted(removed))
        added = tuple(sorted(added))

        if removed:
            self._execute_update(self._build_data_update("DELETE DATA", removed))

        if added:
            self._execute_update(self._build_data_update("INSERT DATA", added))

    def replace_graph(self, turtle_payload: str) -> None:
        response = self.session.put(
            self.config.statements_url,
            params={"context": f"<{self.config.named_graph}>"},
            data=turtle_payload.encode('utf-8'),
            headers={"Content-Type": "text/turtle; charset=utf-8"},
            auth=self._auth(),
            timeout=self.config.timeout_seconds
        )
        response.raise_for_status()

    def _execute_update(self, update_query: str) -> None:
        response = self.session.post(
            self.config.statements_url,
            data=update_query.encode("utf-8"),
            headers={"Content-Type": "application/sparql-update; charset=utf-8"},
            auth=self._auth(),
            timeout=self.config.timeout_seconds,
        )
        response.raise_for_status()

    def _build_data_update(
        self,
        operation: str,
        triples: Iterable[tuple[str, str, str]],
    ) -> str:
        triple_block = "\n".join(self._triple_to_ntriple(triple) for triple in triples)
        return (
            f"{operation} {{\n"
            f"  GRAPH <{self.config.named_graph}> {{\n"
            f"{triple_block}\n"
            f"  }}\n"
            f"}}"
        )

    @staticmethod
    def _triple_to_ntriple(triple: tuple[str, str, str]) -> str:
        subj, pred, obj = triple
        return f"    <{subj}> <{pred}> <{obj}> ."

    def _auth(self):
        return (self.config.user, self.config.password) if self.config.user else None

    
