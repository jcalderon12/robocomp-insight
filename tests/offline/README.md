# Offline tests (no robot / no Webots required)

These tests exercise the semantic ↔ inner_simulator integration pipeline without
launching the agent stack. Run them from the repo root.

| Test | What it checks | Needs |
|---|---|---|
| `test_validator.py` | `LiveCausalValidator` decision table (unexplained vs. explained retractions). | rdflib |
| `test_blueprint_grounding.py` | Blueprint v2 validation and grounding against `etc/intervention_catalog.json` (testable / untestable marking, bounds). | rdflib |
| `test_hypothesis_compiler.py` | Semantic batch → `hypothesis_compiler` → payloads accepted by the causes simulator's discriminated union. | pybullet |
| `test_verdict_ground_truth.py` | **Slow (~1-2 min).** Full contrastive loop with injected ground truth: a simulated `external_force` episode acts as the real one; the verdict must accept it and reject bump/wheel/friction/nominal; the semantic ingestor derives the causal triples. | pybullet |

```bash
python3 tests/offline/test_validator.py
python3 tests/offline/test_blueprint_grounding.py
PYTHONPATH=tests/offline python3 tests/offline/test_hypothesis_compiler.py
python3 tests/offline/test_verdict_ground_truth.py
```

The end-to-end run with robot (Webots + GraphDB + Ollama + Program-manager) is
described in the repo README; these tests cover every stage in between.
