from dataclasses import dataclass
from typing import Any
import requests


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


    def replace_graph(self, turtle_payload: str) -> None:
        auth = (self.config.user, self.config.password) if self.config.user  else None

        response = self.session.put(
            self.config.statements_url,
            params={"context": f"<{self.config.named_graph}>"},
            data=turtle_payload.encode('utf-8'),
            headers={"Content-Type": "text/turtle; charset=utf-8"},
            auth=auth,
            timeout=self.config.timeout_seconds
        )
        response.raise_for_status()


    