#!/usr/bin/env python3
"""Prints the ATE RMSE [m] of several benchmark runs side by side (Markdown table).

  scripts/compare_results.py baseline coin

Each argument is the tag of a run of run_benchmark.py. The numbers reported in Table I of the
COIN-BIEVR paper are added as reference columns where available.
"""
import argparse
import json
import os

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Table I of the paper: (BIEVR-LIO, COIN-BIEVR). None marks a failed run.
PAPER = {
    "IntersectionS": (0.231, 0.166),
    "IntersectionD": (0.404, 0.303),
    "RunwayS": (0.44, 0.312),
    "RunwayD": (4.35, 2.245),
    "FieldS": (0.159, 0.153),
    "FieldD": (0.174, 0.175),
    "KatzenseeS": (0.194, 0.186),
    "KatzenseeD": (0.243, 0.22),
    "Shield1": (0.256, 0.22),
    "Shield4": (0.275, 0.245),
    "Shield5": (0.146, 0.219),
}


def find_results_root():
    path = os.path.dirname(SCRIPT_DIR)
    while path != os.path.dirname(path):
        if os.path.isdir(os.path.join(path, "results")):
            return os.path.join(path, "results")
        path = os.path.dirname(path)
    return "results"


def fmt(value):
    return "x" if value is None else "%.3f" % value


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("tags", nargs="+", help="benchmark tags to compare")
    parser.add_argument("--results", default=find_results_root(), help="result root folder")
    parser.add_argument("--registry", default=os.path.join(SCRIPT_DIR, "datasets.yaml"))
    parser.add_argument("--no-paper", action="store_true", help="omit the paper columns")
    parser.add_argument("--key", default="rmse", help="value of the evaluation to show")
    args = parser.parse_args()

    runs = {}
    for tag in args.tags:
        with open(os.path.join(args.results, tag, "summary.json")) as f:
            runs[tag] = json.load(f)
    # Sequences in the order of the registry, followed by any others.
    sequences = []
    with open(args.registry) as f:
        registry = yaml.safe_load(f)
    for spec in registry["datasets"].values():
        sequences += [name for name in spec["sequences"] if any(name in runs[t] for t in args.tags)]
    for tag in args.tags:
        sequences += [name for name in runs[tag] if name not in sequences]

    paper = not args.no_paper and args.key == "rmse" and any(s in PAPER for s in sequences)
    header = ["Sequence"] + (["BIEVR-LIO (paper)", "COIN-BIEVR (paper)"] if paper else [])
    header += args.tags
    rows = []
    for name in sequences:
        row = [name]
        if paper:
            row += [fmt(v) for v in PAPER[name]] if name in PAPER else ["-", "-"]
        for tag in args.tags:
            result = runs[tag].get(name)
            row.append("-" if result is None else fmt(result.get(args.key)))
        rows.append(row)
    widths = [max(len(r[i]) for r in [header] + rows) for i in range(len(header))]
    print("| " + " | ".join(h.ljust(w) for h, w in zip(header, widths)) + " |")
    print("|" + "|".join("-" * (w + 2) for w in widths) + "|")
    for row in rows:
        print("| " + " | ".join(c.ljust(w) for c, w in zip(row, widths)) + " |")


if __name__ == "__main__":
    main()
