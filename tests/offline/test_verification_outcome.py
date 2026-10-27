"""What the verification knows goes to the semantic memory, accepted or not, without asserting a cause
(contracts 1.13).

Three real cases of the blind evaluation of 2026-10-22 (docs_output/resultados/49), copied to
tests/offline/data/blind_20261022/ with their batch, episode and verdict:
  * A1, a 25 x 2 cm bump the verdict accepted;
  * D3, the camera covered: it abstains by the adequacy rule (a bump is the most probable, 0.97,
    but explains the record no better than the nominal);
  * D1, the camera covered: it abstains with the nominal as the most probable (0.79).
Checks the verification outcome: decision, thresholds, the probability of each mechanism and the
credible set, each run's probability and ratio; on an abstention, the statement that no simulated
cause explains the record clearly better than the nominal, the open alternatives and their
evidence (the bottle reacquired for the perception failure), with no cause and no explanation.
Also that the statement needs the numbers, that an old verdict without probabilities still gives
an outcome, that the ingestion of the verdict does not change, CQ11 on the three cases, the TBox
terms and, if owlready2 is installed, HermiT. No LLM and no simulation. Run from the repo root:

    python3 tests/offline/test_verification_outcome.py
"""
import copy
import json
import sys
import tempfile
from pathlib import Path

from rdflib import Graph, Literal, RDF, URIRef

REPO = Path(__file__).resolve().parents[2]
SEMANTIC = REPO / "agents/semantic"
sys.path.insert(0, str(SEMANTIC))

from src.case_rdf import case_iri, hypothesis_iri, run_iri, simulation_graph  # noqa: E402
from src.episode_rdf import DUL, INSIGHT, SOMA, episode_namespace  # noqa: E402
from src.memory_queries import case_dataset, load_questions, run  # noqa: E402
from src.verdict_ingestor import ingest_verdict, verification_graph  # noqa: E402

DATA = Path(__file__).resolve().parent / "data"
BLIND = DATA / "blind_20261022"
TBOX = SEMANTIC / "data/insight_tbox.ttl"
CASES = {"A1": "20261022T075935Z", "D3": "20261022T111403Z", "D1": "20261022T110816Z"}
OLD_CASE = SEMANTIC / "generated_hypotheses/semantic_unexplained_20261007T080222Z"
OLD_VERDICT = DATA / "verdict_semantic_unexplained_20261007T080222Z.json"


def load(name):
    stamp = CASES[name]
    case = BLIND / f"semantic_unexplained_{stamp}"
    verdict_path = BLIND / f"verdict_semantic_unexplained_{stamp}.json"
    return (json.loads(Path(f"{case}_episode.json").read_text()), json.loads(Path(f"{case}.json").read_text()),
            json.loads(verdict_path.read_text()), verdict_path)


def outcome_of(graph, episode):
    return episode_namespace(episode["episode_id"]).Verification


def objects(graph, subject, prop):
    return set(graph.objects(subject, prop))


def probabilities(graph, outcome):
    """mechanism name -> (probability, in credible set, TBox mechanism, runs)."""
    found = {}
    for p in graph.objects(outcome, INSIGHT.assignsProbability):
        found[str(graph.value(p, INSIGHT.mechanismName))] = (
            float(graph.value(p, INSIGHT.posteriorProbability)), graph.value(p, INSIGHT.inCredibleSet).toPython(),
            graph.value(p, INSIGHT.forMechanism), objects(graph, p, INSIGHT.aggregatesRun))
    return found


def check_always_the_probabilities_and_ratios():
    for name in CASES:
        episode, batch, verdict, verdict_path = load(name)
        before = copy.deepcopy((batch, verdict))
        graph = verification_graph(batch, verdict, episode)
        assert (batch, verdict) == before, "the outcome must not change the batch or the verdict"
        outcome = outcome_of(graph, episode)
        assert (outcome, RDF.type, INSIGHT.VerificationOutcome) in graph
        assert (case_iri(batch["case_id"]), DUL.isSettingFor, outcome) in graph
        assert str(graph.value(outcome, INSIGHT.contractsVersion)) == "1.15"
        assert str(graph.value(outcome, INSIGHT.tboxVersion)) == "0.6"
        method = verdict["method"]
        assert float(graph.value(outcome, INSIGHT.acceptanceThreshold)) == method["p_min"]
        assert float(graph.value(outcome, INSIGHT.explanationRatioMax)) == method["explanation_ratio_max"]
        assert float(graph.value(outcome, INSIGHT.credibleMass)) == verdict["credible_set"]["mass"]
        assert float(graph.value(outcome, INSIGHT.explainedRatio)) == verdict["explained_ratio"]
        assert graph.value(outcome, INSIGHT.mostProbableRun) == run_iri(batch, verdict["most_probable_hypothesis_id"])

        found = probabilities(graph, outcome)
        assert set(found) == {h["mechanism"] for h in verdict["hypotheses"]}, (name, found)
        assert abs(sum(p for p, *_ in found.values()) - 1.0) < 1e-4, (name, found)
        credible = {m["mechanism"] for m in verdict["credible_set"]["mechanisms"]}
        assert {m for m, (_, inside, *_) in found.items() if inside} == credible, (name, found)
        mechanisms = {h["mechanism"]: URIRef(h["mechanism_iri"]) for h in batch["hypotheses"] if h.get("mechanism_iri")}
        for mechanism, (_, _, tbox_mechanism, runs) in found.items():
            assert tbox_mechanism == mechanisms.get(mechanism), (name, mechanism, tbox_mechanism)
            assert runs == {run_iri(batch, h["hypothesis_id"]) for h in verdict["hypotheses"]
                            if h["mechanism"] == mechanism}, (name, mechanism)

        runs = simulation_graph(batch, verdict, str(verdict_path))
        for entry in verdict["hypotheses"]:
            r = run_iri(batch, entry["hypothesis_id"])
            assert float(runs.value(r, INSIGHT.posteriorProbability)) == entry["posterior"], (name, entry["hypothesis_id"])
            assert float(runs.value(r, INSIGHT.ratioToNominal)) == entry["ratio_to_nominal"], (name, entry["hypothesis_id"])
        ratios = [h["ratio_to_nominal"] for h in verdict["hypotheses"] if h["hypothesis_id"] != "__nominal__"]
        assert float(graph.value(outcome, INSIGHT.bestCauseRatio)) == min(ratios)


def check_accepted_links_its_explanation():
    episode, batch, verdict, verdict_path = load("A1")
    graph = verification_graph(batch, verdict, episode)
    outcome, ep = outcome_of(graph, episode), episode_namespace(episode["episode_id"])
    accepted = verdict["accepted_hypothesis_id"]
    assert str(graph.value(outcome, INSIGHT.verdictDecision)) == "accepted"
    assert graph.value(outcome, INSIGHT.selectsExplanation) == ep[f"Explanation_{accepted}"]
    assert (outcome, RDF.type, INSIGHT.NoSimulatedCauseExplainsBetter) not in graph
    for prop in (INSIGHT.openSimulatedAlternative, INSIGHT.openNonSimulableAlternative):
        assert not list(graph.triples((outcome, prop, None)))
    # The bottle is not seen again: no evidence for the unresolved perception failure.
    assert not list(graph.triples((None, INSIGHT.supportedByObservation, None)))
    # The ingestion of the verdict is as before: the legacy triples of the accepted cause.
    ingestion = ingest_verdict(verdict_path)
    cause = next(h for h in verdict["hypotheses"] if h["hypothesis_id"] == accepted)["cause"]["name"]
    assert ingestion.ok and ingestion.accepted_hypothesis_id == accepted and ingestion.accepted_intervention == cause


def check_abstention_keeps_what_is_known_and_asserts_no_cause():
    for name, reason in (("D3", "explanation_not_adequate"), ("D1", "no_hypothesis_above_threshold")):
        episode, batch, verdict, verdict_path = load(name)
        assert verdict["abstention_reason"] == reason
        graph = verification_graph(batch, verdict, episode)
        outcome, ep = outcome_of(graph, episode), episode_namespace(episode["episode_id"])
        assert str(graph.value(outcome, INSIGHT.verdictDecision)) == "abstained"
        assert (outcome, RDF.type, INSIGHT.NoSimulatedCauseExplainsBetter) in graph, name
        assert graph.value(outcome, INSIGHT.selectsExplanation) is None

        simulated = {h["hypothesis_id"] for h in verdict["hypotheses"] if h["hypothesis_id"] != "__nominal__"}
        assert objects(graph, outcome, INSIGHT.openSimulatedAlternative) == {
            hypothesis_iri(batch, h) for h in simulated}, name
        by_status = {}
        for h in batch["hypotheses"]:
            by_status.setdefault(h["status"], set()).add(hypothesis_iri(batch, h["hypothesis_id"]))
        open_non_simulable = objects(graph, outcome, INSIGHT.openNonSimulableAlternative)
        assert open_non_simulable == by_status.get("checked_not_simulable", set()) | by_status.get("not_simulable", set())
        assert not open_non_simulable & by_status.get("discarded", set())
        perception = next(h["hypothesis_id"] for h in batch["hypotheses"] if h["mechanism"] == "perception_failure")
        assert hypothesis_iri(batch, perception) in open_non_simulable
        # The episode shows the bottle again: the trace a perception failure entails.
        assert (hypothesis_iri(batch, perception), INSIGHT.supportedByObservation, ep.Ev_bottle_reacquired) in graph

        dataset = case_dataset(episode=episode, batch=batch, verdict=verdict, mirror=[], verdict_path=str(verdict_path))
        assert (case_iri(batch["case_id"]), INSIGHT.abstentionReason, Literal(reason)) in dataset
        assert not list(dataset.subjects(RDF.type, INSIGHT.SimulationSupportedExplanation))
        assert not list(dataset.subjects(RDF.type, INSIGHT.VerifiedCause))
        assert not list(dataset.triples((None, SOMA.causes, None)))
        assert not list(dataset.triples((case_iri(batch["case_id"]), URIRef("http://insight.local/instances#explainedBy"), None)))
        assert not ingest_verdict(verdict_path).triples

    # D1: the nominal is the most probable mechanism; it is in the credible set and has no TBox mechanism.
    episode, batch, verdict, _ = load("D1")
    graph = verification_graph(batch, verdict, episode)
    outcome = outcome_of(graph, episode)
    assert graph.value(outcome, INSIGHT.mostProbableRun) == run_iri(batch, "__nominal__")
    probability, inside, mechanism, _ = probabilities(graph, outcome)["nominal"]
    assert inside and mechanism is None and abs(probability - 0.793624) < 1e-6, (probability, inside, mechanism)


def check_statement_needs_the_numbers():
    episode, batch, verdict, _ = load("D3")
    outcome = outcome_of(None, episode)

    def stated(changed):
        return (outcome, RDF.type, INSIGHT.NoSimulatedCauseExplainsBetter) in verification_graph(batch, changed, episode)

    assert stated(verdict)
    # A simulated cause that explains the record clearly better (ratio below the limit): no statement.
    better = copy.deepcopy(verdict)
    next(h for h in better["hypotheses"] if h["hypothesis_id"] == "H02")["ratio_to_nominal"] = 0.5
    better["abstention_reason"] = "no_hypothesis_above_threshold"
    assert not stated(better)
    # When the simulation was not faithful, or had no record, it says nothing about the causes.
    for reason in ("simulation_not_faithful", "no_record_in_interval"):
        assert not stated({**verdict, "abstention_reason": reason})
    # Without the verdict's limit, or without any ratio of a cause, it cannot say it either.
    assert not stated({**verdict, "method": {k: v for k, v in verdict["method"].items() if k != "explanation_ratio_max"}})
    unknown = copy.deepcopy(verdict)
    for h in unknown["hypotheses"]:
        h["ratio_to_nominal"] = None if h["hypothesis_id"] != "__nominal__" else 1.0
    assert not stated(unknown)


def check_old_verdict_without_probabilities():
    """A verdict written before verdict 2.0 (07/10) has no probabilities: the outcome keeps the decision."""
    episode = json.loads(Path(f"{OLD_CASE}_episode.json").read_text())
    batch = json.loads(Path(f"{OLD_CASE}.json").read_text())
    verdict = json.loads(OLD_VERDICT.read_text())
    graph = verification_graph(batch, verdict, episode)
    outcome = outcome_of(graph, episode)
    assert str(graph.value(outcome, INSIGHT.verdictDecision)) == "accepted"
    assert not list(graph.objects(outcome, INSIGHT.assignsProbability))
    assert graph.value(outcome, INSIGHT.acceptanceThreshold) is None
    abstained = verification_graph(batch, {**verdict, "accepted_hypothesis_id": None,
                                           "abstention_reason": "nominal_reproduces_effect"}, None)
    outcome = outcome_of(abstained, episode)
    assert str(abstained.value(outcome, INSIGHT.verdictDecision)) == "abstained"
    assert (outcome, RDF.type, INSIGHT.NoSimulatedCauseExplainsBetter) not in abstained
    assert list(abstained.objects(outcome, INSIGHT.openNonSimulableAlternative))
    assert not list(abstained.triples((None, INSIGHT.supportedByObservation, None)))  # no episode, no evidence
    assert verification_graph({"schema_version": "1.0"}, verdict) is None


def three_cases():
    dataset = None
    for name in CASES:
        episode, batch, verdict, verdict_path = load(name)
        dataset = case_dataset(episode=episode, batch=batch, verdict=verdict, mirror=[],
                               verdict_path=str(verdict_path), dataset=dataset)
    return dataset


def check_cq11():
    question = next(q for q in load_questions() if q.name == "cq11_verification")
    rows = {row["case"]: row for row in run(three_cases(), question)}
    split = lambda value: set(value.split(", ")) if value else set()
    a1, d3, d1 = (rows[f"semantic_unexplained_{CASES[n]}"] for n in ("A1", "D3", "D1"))
    assert (a1["decision"], a1["reason"], a1["accepted"], a1["no_simulated_cause_better"]) == (
        "accepted", None, "obstacle_traversed", False), a1
    assert (a1["explained_ratio"], a1["ratio_limit"], a1["credible_set"]) == (0.5139, 0.615, "obstacle_traversed"), a1
    assert a1["simulated_alternatives"] is None and a1["not_simulable_alternatives"] == "perception_failure (H05)", a1
    assert a1["evidence"] is None, a1
    assert (d3["decision"], d3["reason"], d3["accepted"], d3["no_simulated_cause_better"]) == (
        "abstained", "explanation_not_adequate", None, True), d3
    assert (d3["most_probable"], d3["best_cause_ratio"]) == ("obstacle_traversed", 0.9912), d3
    assert split(d3["simulated_alternatives"]) == {"obstacle_traversed (H04)", "bottle_push (H02)"}, d3
    assert d3["not_simulable_alternatives"] == "perception_failure (H01)" and d3["evidence"] == "H01: Ev_bottle_reacquired", d3
    assert (d1["reason"], d1["most_probable"], d1["most_probable_probability"]) == (
        "no_hypothesis_above_threshold", "nominal", 0.793624), d1
    assert split(d1["credible_set"]) == {"nominal", "obstacle_traversed"}, d1
    assert split(d1["not_simulable_alternatives"]) == {"perception_failure (H01)", "suspension_failure (H03)"}, d1
    assert d1["evidence"] == "H01: Ev_bottle_reacquired", d1


def check_only_tbox_terms():
    tbox = Graph().parse(TBOX)
    declared = {s for s, _, _ in tbox}
    for name in CASES:
        episode, batch, verdict, _ = load(name)
        graph = verification_graph(batch, verdict, episode) + simulation_graph(batch, verdict)
        used = {p for _, p, _ in graph} | {o for _, p, o in graph if p == RDF.type}
        own = {t for t in used if str(t).startswith("http://insight.local/ontology#")}
        assert own <= declared, own - declared


def check_inferred_types_if_possible():
    try:
        import owlready2
    except ImportError:
        print("  (owlready2 not installed: HermiT check skipped)")
        return
    dataset = three_cases()
    graph = Graph()
    for triple in dataset.triples((None, None, None)):
        graph.add(triple)
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "verification_outcomes.owl"
        graph.serialize(path, format="xml")
        world = owlready2.World()
        ontology = world.get_ontology(path.as_uri()).load()
        with ontology:
            owlready2.sync_reasoner_hermit(world, infer_property_values=False, debug=0)
        assert not list(world.inconsistent_classes())
        for name in CASES:
            episode, batch, _, _ = load(name)
            ep = episode_namespace(episode["episode_id"])
            for iri in [ep.Verification] + [ep[f"Probability_{m}"] for m in ("nominal", "obstacle_traversed")]:
                types = world[str(iri)].INDIRECT_is_a
                assert world[str(DUL.Description)] in types, iri
                assert world[str(DUL.Event)] not in types and world[str(SOMA.Accident)] not in types, (iri, types)
            case_types = world[str(case_iri(batch["case_id"]))].INDIRECT_is_a
            assert world[str(INSIGHT.VerificationOutcome)] not in case_types, case_types
    print("  HermiT: consistent; outcomes and probabilities are descriptions, not events")


def main():
    for check in (check_always_the_probabilities_and_ratios, check_accepted_links_its_explanation,
                  check_abstention_keeps_what_is_known_and_asserts_no_cause, check_statement_needs_the_numbers,
                  check_old_verdict_without_probabilities, check_cq11, check_only_tbox_terms,
                  check_inferred_types_if_possible):
        check()
        print(f"OK {check.__name__}")
    print("Verification outcome: all checks passed")


if __name__ == "__main__":
    main()
