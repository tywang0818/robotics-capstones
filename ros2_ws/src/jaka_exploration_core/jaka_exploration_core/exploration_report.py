#!/usr/bin/env python3
"""Read an exploration_events.jsonl file and print a structured report.

Usage:
    python3 exploration_report.py /tmp/exploration_events.jsonl [--json]

Without --json: prints a human/AI-readable narrative per cycle plus
overall summary. With --json: emits a single JSON object with all
per-cycle rollups and aggregates, suitable for further ingestion by
an LLM that only sees this file.
"""
import argparse
import collections
import json
import re
import sys
from typing import Any, Dict, List, Optional


def load(path: str) -> List[Dict[str, Any]]:
    events: List[Dict[str, Any]] = []
    with open(path) as fh:
        for lineno, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError as exc:
                print(
                    f"WARN {path}:{lineno} not valid JSON: {exc}",
                    file=sys.stderr,
                )
    return events


def group_by_cycle(events):
    by_cycle: Dict[int, List[Dict[str, Any]]] = collections.defaultdict(list)
    for e in events:
        by_cycle[int(e.get("cycle", 0))].append(e)
    return by_cycle


def summarize_cycle(cycle_num: int, evts: List[Dict[str, Any]]):
    """Return a dict summarizing a single cycle."""
    start = next(
        (e for e in evts if e["type"] == "cycle_start"), None)
    env_v = start["env_version"] if start else None
    ts_start = start["ts"] if start else (evts[0]["ts"] if evts else None)

    tf = next((e for e in evts if e["type"] == "stage0_tf"), None)
    robot_xyz = tf["robot_xyz"] if tf else None

    s1 = next((e for e in evts if e["type"] == "stage1_frontier"), None)
    frontiers = s1["total_voxels"] if s1 else None

    s2 = next((e for e in evts if e["type"] == "stage2_filter"), None)
    s2_filter = None
    if s2:
        s2_filter = {"before": s2["before"], "after": s2["after"], "reasons": s2["reasons"]}

    s3done = next((e for e in evts if e["type"] == "stage3_done"), None)
    K = s3done["K"] if s3done else 0

    probs = [e for e in evts if e["type"] == "prob_candidate"]
    prob_summary = []
    for p in probs:
        prob_summary.append({
            "frontier": p["frontier_xyz"],
            "unk": p["unk"],
            "occl": p["occl"],
            "ig": p["ig"],
            "peek": p["peek"],
            "attract": p["attract"],
            "los": p["los"],
            "igate_killed": p["igate_killed"],
            "raw": p["raw"],
        })

    ranks = next((e for e in evts if e["type"] == "stage5_greedy_ranks"), None)
    pick = next((e for e in evts if e["type"] == "stage5_greedy_pick"), None)
    stage5_empty = next(
        (e for e in evts if e["type"] == "stage5_empty"), None)
    stage3c_empty = next(
        (e for e in evts if e["type"] == "stage3c_empty"), None)
    stage2_empty = next(
        (e for e in evts if e["type"] == "stage2_empty"), None)

    confirm_send = next(
        (e for e in evts if e["type"] == "confirm_probe_send"), None)
    confirm_result = next(
        (e for e in evts if e["type"] == "confirm_probe_result"), None)
    publish = next((e for e in evts if e["type"] == "publish"), None)
    bl_adds = [e for e in evts if e["type"] == "blacklist_add"]
    sticky_releases = [e for e in evts if e["type"] == "sticky_release"]
    visited_marks = [e for e in evts if e["type"] == "visited_mark"]

    outcome = "unknown"
    if publish and confirm_result and confirm_result["result"] == "OK":
        outcome = "published"
    elif confirm_result and confirm_result["result"] == "FAIL":
        outcome = "confirm_probe_rejected"
    elif stage5_empty is not None:
        outcome = "stage5_empty"
    elif stage3c_empty is not None:
        outcome = "stage3c_empty"
    elif stage2_empty is not None:
        outcome = "stage2_empty"

    return {
        "cycle": cycle_num,
        "ts_start": ts_start,
        "env_version": env_v,
        "robot_xyz": robot_xyz,
        "frontiers": frontiers,
        "stage2_filter": s2_filter,
        "K": K,
        "prob_candidates": prob_summary,
        "ranks": (ranks["ranks"] if ranks else None),
        "pick": (pick and {
            "cand": pick["cand"],
            "raw": pick["raw"],
            "cost": pick["cost"],
            "score": pick["score"],
            "goal": pick["goal"],
        }),
        "confirm_probe": (confirm_result and {
            "cost": confirm_result["cost"],
            "result": confirm_result["result"],
        }),
        "publish": (publish and {
            "task_id": publish["task_id"],
            "goal": publish["goal"],
            "surrogate": publish["surrogate"],
        }),
        "blacklist_adds": len(bl_adds),
        "sticky_releases": (sticky_releases[0]["count_cleared"]
                            if sticky_releases else 0),
        "visited_marks": len(visited_marks),
        "outcome": outcome,
    }


def format_cycle_narrative(cy: Dict[str, Any]) -> str:
    lines = []
    hdr = (
        f"Cycle {cy['cycle']}  env_v={cy['env_version']} "
        f"robot={cy['robot_xyz']}  outcome={cy['outcome']}"
    )
    lines.append(hdr)
    if cy["stage2_filter"]:
        f = cy["stage2_filter"]
        r = f["reasons"]
        lines.append(
            f"  STAGE 2: {f['before']} → {f['after']} voxels  "
            f"(too_far={r['too_far']} forbidden={r['forbidden']} "
            f"blacklist={r['blacklisted']} visited={r['visited']})")
    lines.append(f"  STAGE 3: K={cy['K']} candidates")
    if cy["prob_candidates"]:
        for i, p in enumerate(cy["prob_candidates"]):
            los = "los=1" if p["los"] else "los=0"
            ig_k = " igate=1" if p["igate_killed"] else ""
            lines.append(
                f"    i={i} frontier={p['frontier']} "
                f"unk={p['unk']:.2f} occl={p['occl']:.2f} ig={p['ig']:.2f} "
                f"peek={p['peek']:.2f} attract={p['attract']:.2f} "
                f"{los}{ig_k} raw={p['raw']:.3f}"
            )
    if cy["pick"]:
        p = cy["pick"]
        lines.append(
            f"  PICK: cand={p['cand']} raw={p['raw']:.3f} "
            f"cost={p['cost']:.2f} score={p['score']:.3f} goal={p['goal']}"
        )
    if cy["confirm_probe"]:
        c = cy["confirm_probe"]
        lines.append(f"  CONFIRM: cost={c['cost']:.3f} → {c['result']}")
    if cy["publish"]:
        p = cy["publish"]
        srg = "SURROGATE" if p["surrogate"] else "direct"
        lines.append(
            f"  PUBLISHED task={p['task_id']} goal={p['goal']} ({srg})"
        )
    if cy["blacklist_adds"]:
        lines.append(f"  blacklist_adds={cy['blacklist_adds']}")
    if cy["sticky_releases"]:
        lines.append(f"  sticky_release: {cy['sticky_releases']} entries cleared")
    if cy["visited_marks"]:
        lines.append(f"  visited_marks={cy['visited_marks']}")
    return "\n".join(lines)


def aggregate(cycles: List[Dict[str, Any]]) -> Dict[str, Any]:
    total = len(cycles)
    published = [c for c in cycles if c["publish"] is not None]
    confirm_ok = [c for c in cycles
                  if c["confirm_probe"] and c["confirm_probe"]["result"] == "OK"]
    confirm_fail = [c for c in cycles
                    if c["confirm_probe"] and c["confirm_probe"]["result"] == "FAIL"]
    empty_by_stage = {
        "stage2": sum(1 for c in cycles if c["outcome"] == "stage2_empty"),
        "stage3c": sum(1 for c in cycles if c["outcome"] == "stage3c_empty"),
        "stage5":  sum(1 for c in cycles if c["outcome"] == "stage5_empty"),
    }
    igate_kills = sum(
        1 for c in cycles for p in (c["prob_candidates"] or [])
        if p["igate_killed"])
    los_blocked = sum(
        1 for c in cycles for p in (c["prob_candidates"] or [])
        if not p["los"])
    return {
        "cycles_total": total,
        "cycles_published": len(published),
        "publish_rate": (len(published) / total if total else 0.0),
        "confirm_ok": len(confirm_ok),
        "confirm_fail": len(confirm_fail),
        "confirm_rate": (
            len(confirm_ok) / (len(confirm_ok) + len(confirm_fail))
            if (confirm_ok or confirm_fail) else 0.0
        ),
        "empty_by_stage": empty_by_stage,
        "prob_candidates_total": sum(
            len(c["prob_candidates"] or []) for c in cycles),
        "igate_kills": igate_kills,
        "los_blocked": los_blocked,
    }


def task_lifecycles(events, cycle_summaries):
    """Stitch decision_queued / decision_task_completed / publish per task_id.

    Also correlates each published task with the CYCLE that produced it
    (cause chain: frontier centroid → K candidates → GREEDY pick → publish)
    and computes effect proxies (map growth between publish and result).
    """
    # Build cycle map keyed by task_id (each cycle has at most one publish)
    cycle_by_task: Dict[str, Dict[str, Any]] = {}
    for cy in cycle_summaries:
        p = cy.get("publish")
        if p and p.get("task_id"):
            cycle_by_task[p["task_id"]] = cy

    tasks: Dict[str, Dict[str, Any]] = {}
    for e in events:
        tid = e.get("task_id")
        if not tid:
            continue
        tasks.setdefault(tid, {"task_id": tid, "events": []})
        tasks[tid]["events"].append(e)

    # Global timeseries used for effect proxies
    env_events = [e for e in events if e["type"] == "env_version"]
    frontier_events = [
        e for e in events if e["type"] == "stage1_frontier"]

    def _nearest_le(seq, ts, key="ts"):
        # last entry with ts <= given ts (or None)
        matches = [e for e in seq if e[key] <= ts]
        return matches[-1] if matches else None

    def _first_ge(seq, ts, key="ts"):
        for e in seq:
            if e[key] >= ts:
                return e
        return None

    result = []
    for tid, info in tasks.items():
        published = next(
            (e for e in info["events"] if e["type"] == "publish"), None)
        completed = next(
            (e for e in info["events"] if e["type"] == "decision_task_completed"),
            None)
        cy = cycle_by_task.get(tid)

        # --- cause chain ---
        cause = None
        if cy and cy.get("pick") and cy.get("prob_candidates"):
            picked_cand = cy["pick"]["cand"]     # 1-based
            picked_idx = picked_cand - 1
            probs = cy["prob_candidates"]
            winning_prob = (
                probs[picked_idx] if 0 <= picked_idx < len(probs) else None)
            cause = {
                "cycle": cy["cycle"],
                "env_version": cy["env_version"],
                "robot_xyz": cy["robot_xyz"],
                "K": cy["K"],
                "picked_frontier_xyz": (
                    winning_prob and winning_prob.get("frontier")),
                "picked_raw": (winning_prob and winning_prob.get("raw")),
                "picked_ig": (winning_prob and winning_prob.get("ig")),
                "picked_unk": (winning_prob and winning_prob.get("unk")),
                "picked_attract": (winning_prob and winning_prob.get("attract")),
                "picked_los": (winning_prob and winning_prob.get("los")),
                "confirm_cost": (
                    cy.get("confirm_probe") and cy["confirm_probe"].get("cost")),
                "confirm_result": (
                    cy.get("confirm_probe") and cy["confirm_probe"].get("result")),
                "all_candidates_frontier_xyz": [
                    p.get("frontier") for p in probs],
                "all_candidates_raw": [p.get("raw") for p in probs],
            }

        # --- effect proxies ---
        effect = None
        if published and completed:
            t0 = published["ts"]
            t1 = completed["ts"]
            env_before = _nearest_le(env_events, t0)
            env_after = _first_ge(env_events, t1)
            fr_before = _nearest_le(frontier_events, t0)
            fr_after = _first_ge(frontier_events, t1)
            effect = {
                "wall_duration_sec": t1 - t0,
                "env_bumps_during_task": sum(
                    1 for e in env_events if t0 <= e["ts"] <= t1),
                "payload_bytes_before": (
                    env_before and env_before.get("payload_bytes")),
                "payload_bytes_after": (
                    env_after and env_after.get("payload_bytes")),
                "payload_bytes_delta": (
                    (env_after["payload_bytes"] - env_before["payload_bytes"])
                    if env_before and env_after else None),
                "frontier_total_before": (
                    fr_before and fr_before.get("total_voxels")),
                "frontier_total_after": (
                    fr_after and fr_after.get("total_voxels")),
                "frontier_delta": (
                    (fr_after["total_voxels"] - fr_before["total_voxels"])
                    if fr_before and fr_after else None),
            }

        result.append({
            "task_id": tid,
            "published_ts": (published and published["ts"]),
            "goal": (published and published.get("goal")),
            "surrogate": (published and published.get("surrogate")),
            "completed_ts": (completed and completed["ts"]),
            "success": (completed and completed["success"]),
            "moveit_error": (completed and completed["moveit_error"]),
            "planning_time_sec": (completed and completed["planning_time_sec"]),
            "exec_time_sec": (completed and completed["exec_time_sec"]),
            "cause": cause,
            "effect_proxies": effect,
        })
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="path to exploration_events.jsonl")
    ap.add_argument("--json", action="store_true",
                    help="emit machine-readable JSON instead of narrative")
    args = ap.parse_args()

    events = load(args.path)
    by_cycle = group_by_cycle(events)
    # cycle=0 collects pre-cycle events (logger_start, initial_scan_done, env, joint_snapshots).
    cycle_summaries = [
        summarize_cycle(k, by_cycle[k])
        for k in sorted(by_cycle.keys())
        if k > 0
    ]
    agg = aggregate(cycle_summaries)
    tasks = task_lifecycles(events, cycle_summaries)

    if args.json:
        print(json.dumps({
            "file": args.path,
            "event_count": len(events),
            "aggregate": agg,
            "cycles": cycle_summaries,
            "tasks": tasks,
        }, indent=2))
        return

    print(f"=== Exploration report ({args.path}) ===")
    print(f"events: {len(events)}, cycles: {agg['cycles_total']}, "
          f"published: {agg['cycles_published']} "
          f"({100*agg['publish_rate']:.0f}%)")
    print(f"confirm probe: {agg['confirm_ok']} OK / {agg['confirm_fail']} FAIL "
          f"({100*agg['confirm_rate']:.0f}%)")
    print(f"early exits: stage2_empty={agg['empty_by_stage']['stage2']}, "
          f"stage3c_empty={agg['empty_by_stage']['stage3c']}, "
          f"stage5_empty={agg['empty_by_stage']['stage5']}")
    print(f"prob candidates: {agg['prob_candidates_total']} evaluated, "
          f"{agg['igate_kills']} igate-killed, "
          f"{agg['los_blocked']} los-blocked")
    print("")
    print("--- per cycle ---")
    for cy in cycle_summaries:
        print(format_cycle_narrative(cy))
        print()
    print("--- task lifecycles (cause + effect proxies) ---")
    for t in tasks:
        gt = t.get("goal") or "?"
        s = t.get("success")
        me = t.get("moveit_error")
        pt = t.get("planning_time_sec")
        xt = t.get("exec_time_sec")
        print(
            f"{t['task_id']}  goal={gt}  success={s}  "
            f"moveit_error={me}  plan={pt}s  exec={xt}s"
        )
        c = t.get("cause")
        if c:
            print(
                f"  cause: cycle={c['cycle']} K={c['K']} "
                f"picked_frontier={c['picked_frontier_xyz']} "
                f"raw={c['picked_raw']} ig={c['picked_ig']} "
                f"unk={c['picked_unk']} attract={c['picked_attract']} "
                f"los={c['picked_los']} confirm={c['confirm_result']}"
                f" (cost={c['confirm_cost']})"
            )
        ef = t.get("effect_proxies")
        if ef:
            print(
                f"  effect: wall={ef['wall_duration_sec']:.1f}s "
                f"env_bumps={ef['env_bumps_during_task']} "
                f"payload {ef['payload_bytes_before']} → "
                f"{ef['payload_bytes_after']} "
                f"(Δ={ef['payload_bytes_delta']}) "
                f"frontier {ef['frontier_total_before']} → "
                f"{ef['frontier_total_after']} "
                f"(Δ={ef['frontier_delta']})"
            )


if __name__ == "__main__":
    main()
