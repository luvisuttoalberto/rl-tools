#!/usr/bin/env python3
"""Compare episode metrics between evaluation runs (scripted baseline vs. learned policy).

Reads the TensorBoard scalars written by baseline_oil_platform / evaluate_oil_platform
(one point per evaluation episode) and prints a per-tag mean +/- std table across runs.

Usage:
  tools/compare_eval_metrics.py <run_dir> [<run_dir> ...] [--labels a b ...] [--csv out.csv]

where <run_dir> is the seed directory of an evaluation run, e.g.
  experiments/2026-07-03_*/*_zoo_environment_algorithm/oil_platform-v1_scripted/0000
  experiments/2026-07-03_*/*_zoo_environment_algorithm/oil_platform-v1_sac-eval/0000
(the directory containing logs.tfevents)
"""
import argparse
import math
import os
import sys

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def load_scalars(run_dir):
    log_dir = os.path.join(run_dir, "logs.tfevents")
    if not os.path.isdir(log_dir):
        log_dir = run_dir
    ea = EventAccumulator(log_dir, size_guidance={"scalars": 0})
    print(f"Loading: {log_dir}", file=sys.stderr)
    ea.Reload()
    scalars = {}
    for tag in ea.Tags()["scalars"]:
        values = [e.value for e in ea.Scalars(tag)]
        scalars[tag] = values
    return scalars


def stats(values):
    n = len(values)
    mean = sum(values) / n
    std = math.sqrt(max(0.0, sum(v * v for v in values) / n - mean * mean))
    return mean, std, n


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dirs", nargs="+", help="Seed directories of evaluation runs (containing logs.tfevents)")
    parser.add_argument("--labels", nargs="*", default=None, help="Column labels (default: derived from the run path)")
    parser.add_argument("--csv", default=None, help="Also write the table to this CSV file")
    args = parser.parse_args()

    if args.labels is not None and len(args.labels) != len(args.run_dirs):
        parser.error("--labels must match the number of run directories")

    labels = args.labels
    if labels is None:
        labels = []
        for run_dir in args.run_dirs:
            parts = os.path.normpath(run_dir).split(os.sep)
            # .../oil_platform-v1_<algorithm>/<seed> -> "<algorithm>/<seed>"
            labels.append("/".join(parts[-2:]) if len(parts) >= 2 else run_dir)

    runs = [load_scalars(run_dir) for run_dir in args.run_dirs]

    tags = sorted(set().union(*[set(r.keys()) for r in runs]))
    rows = []
    for tag in tags:
        row = [tag]
        for run in runs:
            if tag in run and len(run[tag]) > 0:
                mean, std, n = stats(run[tag])
                row.append(f"{mean:.4g} ± {std:.4g} (n={n})")
            else:
                row.append("—")
        rows.append(row)

    header = ["metric"] + labels
    widths = [max(len(str(r[i])) for r in [header] + rows) for i in range(len(header))]
    def fmt(row):
        return " | ".join(str(cell).ljust(width) for cell, width in zip(row, widths))
    print(fmt(header))
    print("-|-".join("-" * width for width in widths))
    for row in rows:
        print(fmt(row))

    if args.csv:
        import csv
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["metric"] + [x for label in labels for x in (f"{label} mean", f"{label} std", f"{label} n")])
            for tag in tags:
                row = [tag]
                for run in runs:
                    if tag in run and len(run[tag]) > 0:
                        mean, std, n = stats(run[tag])
                        row += [mean, std, n]
                    else:
                        row += ["", "", 0]
                writer.writerow(row)
        print(f"CSV written to: {args.csv}", file=sys.stderr)


if __name__ == "__main__":
    main()
