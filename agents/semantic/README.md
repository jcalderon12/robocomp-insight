# semantic

Semantic memory agent (LEAKER / INSIGHT). Mirrors the high-level DSR state as RDF
triples in GraphDB, detects unexplained structural changes, generates candidate
causal hypotheses with an LLM grounded in a closed catalog of executable
simulation interventions, and consolidates the causal relation accepted by the
inner simulator back into the semantic memory (self-extension).

## Pipeline

1. **DSR → RDF mirror** (`ontology_mapping.py`, `graphdb_client.py`): managed
   triples (DUL-based) are kept in sync with GraphDB via atomic SPARQL deltas.
2. **Causal validation** (`live_causal_validator.py`): structural deltas without
   explicit causal evidence (e.g. the bottle leaves the robot) raise an
   *unexplained* flag; an `unexplained` intention node is inserted in DSR.
3. **Hypothesis generation** (`hypothesis_service.py`, `hypothesis_context.py`,
   `intervention_catalog.py`): an LLM (Ollama) produces N internal + M external
   hypotheses. Each `simulation_blueprint` must instantiate one intervention of
   `etc/intervention_catalog.json` (shared contract with `inner_simulator`) with
   parameters inside the declared bounds; hypotheses that cannot be grounded are
   kept but marked `testable: false`. The batch is written to
   `generated_hypotheses/` and its path is published as the
   `hypotheses_filepath` attribute of the intention node.
4. **Verdict ingestion** (`verdict_ingestor.py`): when the inner simulator
   publishes `verdict_filepath` on the same node, the accepted causal relation
   is written to GraphDB as `insight:Event_<Intervention> insight:causes
   insight:Event_BottleLocationChange` (plus provenance), the intention node is
   removed, and the detection cycle re-arms.

## Configuration (`etc/config`)

```
graphDB.Enabled / Endpoint / Repository / NamedGraph
hypothesisGenerator.Enabled
hypothesisGenerator.OutputDir           # hypotheses batches
hypothesisGenerator.DescriptionPath     # robot self-model capsule (description.md)
hypothesisGenerator.CatalogPath         # shared intervention catalog
hypothesisGenerator.PrimaryModel / FallbackModel / OllamaBaseUrl
hypothesisGenerator.InternalCount / ExternalCount
hypothesisGenerator.PreferredClient / RequestTimeoutSeconds / DescriptionCharLimit
```

## Starting the component

```
cd <semantic's path>
cp etc/config config
bin/semantic config
```

Offline tests for the validator, blueprint grounding and the full contrastive
loop live in `tests/offline/` at the repo root.
