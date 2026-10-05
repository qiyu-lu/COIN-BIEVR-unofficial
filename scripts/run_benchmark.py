#!/usr/bin/env python3
"""Runs `process_bag` over the sequences listed in datasets.yaml and reports the ATE RMSE.

Every run gets its own result folder `<out>/<tag>/<sequence>/` holding the generated run config,
the trajectory, the log and the evaluation. Example:

  scripts/run_benchmark.py --tag coin --datasets enwide geode --jobs 3
  scripts/run_benchmark.py --tag ablation --sequences FieldS RunwayS --set intensity.enabled=false
  scripts/run_benchmark.py --tag maps --sequences FieldS --set debug.map_path={out_dir}/map.pcd

The node is started through the `devel/setup.bash` of the workspace that contains this repository.
Every benchmark brings up its own roscore on a free port, so it neither depends on nor disturbs
other ROS sessions.
"""
import argparse
import concurrent.futures
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)
from evaluate_ate import evaluate  # noqa: E402


def find_workspace(start):
    """Walks up from the repository until a catkin workspace (devel/setup.bash) is found."""
    path = start
    while path != os.path.dirname(path):
        if os.path.isfile(os.path.join(path, "devel", "setup.bash")):
            return path
        path = os.path.dirname(path)
    return None


def parse_override(text):
    """`section.key=value` -> (section, key, parsed value)."""
    name, _, value = text.partition("=")
    section, _, key = name.partition(".")
    if not section or not key or not _:
        raise argparse.ArgumentTypeError("override must look like section.key=value: %r" % text)
    if "{out_dir}" in value:  # path placeholder, not YAML
        return section, key, value
    return section, key, yaml.safe_load(value)


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("localhost", 0))
        return s.getsockname()[1]


def start_roscore(workspace):
    """Starts a private roscore and returns (process, master uri)."""
    port = free_port()
    uri = "http://localhost:%d" % port
    process = subprocess.Popen(
        ["bash", "-c", "source %s/devel/setup.bash; exec roscore -p %d" % (workspace, port)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(100):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(("localhost", port)) == 0:
                return process, uri
        time.sleep(0.1)
    process.terminate()
    sys.exit("Could not start a roscore on port %d." % port)


def parse_log(log_path):
    """Per-scan processing time of the pipeline ("step" timer) from the timing table in the log."""
    stats = {}
    pattern = re.compile(r"^step\s+(\d+)\s+\S+\s+\((\S+) \+- \S+\)\s+\[\S+,(\S+)\]")
    with open(log_path, errors="ignore") as f:
        for line in f:
            match = pattern.match(line)
            if match:
                stats["step_mean_ms"] = 1e3 * float(match.group(2))
                stats["step_max_ms"] = 1e3 * float(match.group(3))
    return stats


def collect_runs(registry, datasets, sequences, roots):
    runs = []
    for dataset, spec in registry["datasets"].items():
        for name, seq in spec["sequences"].items():
            if sequences and name not in sequences:
                continue
            if not sequences and datasets and dataset not in datasets:
                continue
            root = roots[dataset]
            runs.append({
                "name": name,
                "dataset": dataset,
                "bag": os.path.join(root, seq["bag"]),
                "gt": os.path.join(root, seq["gt"]) if seq.get("gt") else None,
                "sensor_config": seq.get("sensor_config", spec["sensor_config"]),
                "lever_arm": seq.get("lever_arm", spec.get("lever_arm")),
                "time_offset": seq.get("time_offset", spec.get("time_offset", 0.0)),
                "overrides": seq.get("overrides", spec.get("overrides", {})),
            })
    return runs


def run_sequence(run, args, workspace):
    out_dir = os.path.join(args.out, args.tag, run["name"])
    os.makedirs(out_dir, exist_ok=True)
    traj_path = os.path.join(out_dir, "traj.txt")
    log_path = os.path.join(out_dir, "log.txt")
    cfg_path = os.path.join(out_dir, "run_config.yaml")
    result = {"name": run["name"], "dataset": run["dataset"]}

    if not args.eval_only:
        sensor_cfg_path = run["sensor_config"]
        if not os.path.isabs(sensor_cfg_path):
            sensor_cfg_path = os.path.join(REPO_DIR, "config", "sensor_configs",
                                           sensor_cfg_path + ".yaml")
        with open(sensor_cfg_path) as f:
            cfg = yaml.safe_load(f) or {}
        # The run config is passed as the sensor config, which wins over params.yaml per leaf.
        for section, values in run["overrides"].items():
            cfg.setdefault(section, {}).update(values)
        for section, key, value in args.set:
            if isinstance(value, str):
                value = value.replace("{out_dir}", out_dir)
            cfg.setdefault(section, {})[key] = value
        debug = cfg.setdefault("debug", {})
        debug["trajectory_path"] = traj_path
        debug["dashboard"] = False
        if args.threads > 0:
            cfg["max_num_threads"] = args.threads
        with open(cfg_path, "w") as f:
            yaml.safe_dump(cfg, f, default_flow_style=None, sort_keys=False)

        binary = os.path.join(args.bin_dir, "process_bag") if args.bin_dir else os.path.join(
            workspace, "devel", "lib", "bievr_lio_ros", "process_bag")
        lib_dirs = ([args.bin_dir] if args.bin_dir else []) + args.ld_library_path
        lib_prefix = ("export LD_LIBRARY_PATH=%s:$LD_LIBRARY_PATH; " % ":".join(lib_dirs)
                      if lib_dirs else "")
        # GNU time reports the peak memory of the node.
        rss_path = os.path.join(out_dir, "max_rss_kb.txt")
        time_prefix = ("%s -f %%M -o '%s' " % (shutil.which("time"), rss_path)
                       if shutil.which("time") else "")
        cmd = ("source %s/devel/setup.bash; export ROS_MASTER_URI=%s; %sexec %s%s "
               "--sensor_config_file '%s' --params_file '%s' --bag '%s' __name:=bievr_bench_%s" %
               (workspace, args.master_uri, lib_prefix, time_prefix, binary, cfg_path, args.params,
                run["bag"], run["name"].lower()))
        if os.path.exists(traj_path):
            os.remove(traj_path)
        start = time.time()
        with open(log_path, "w") as log:
            proc = subprocess.run(["bash", "-c", cmd], stdout=log, stderr=subprocess.STDOUT)
        result["wall_time_s"] = time.time() - start
        result["exit_code"] = proc.returncode
        result.update(parse_log(log_path))
        if os.path.isfile(rss_path):
            with open(rss_path) as f:
                tokens = f.read().split()
            if tokens and tokens[-1].isdigit():
                result["max_rss_mb"] = int(tokens[-1]) / 1024.0

    if not os.path.isfile(traj_path) or os.path.getsize(traj_path) == 0:
        result["error"] = "no trajectory written"
        return result
    with open(traj_path) as f:
        result["n_poses"] = sum(1 for _ in f)
    if run["gt"]:
        try:
            res = evaluate(traj_path, run["gt"], run["lever_arm"], args.max_gap,
                           run["time_offset"])
            result.update({k: v for k, v in res.items() if not k.startswith("_")})
            if args.plot:
                plot(res, os.path.join(out_dir, "ate.png"), "%s (%s)" % (run["name"], args.tag))
        except Exception as e:  # keep the benchmark going if a single evaluation fails
            result["error"] = str(e)
    with open(os.path.join(out_dir, "eval.json"), "w") as f:
        json.dump(result, f, indent=1)
    return result


def plot(res, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    axes[0].plot(res["_p_gt"][:, 0], res["_p_gt"][:, 1], "k-", lw=1.5, label="ground truth")
    axes[0].plot(res["_p_aligned"][:, 0], res["_p_aligned"][:, 1], "r-", lw=1.0,
                 label="estimate (aligned)")
    axes[0].set_aspect("equal", "datalim")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].set_title(title)
    axes[0].legend()
    axes[1].plot(res["_t"] - res["_t"][0], res["_err"], "r-", lw=1.0)
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("position error [m]")
    axes[1].set_title("ATE RMSE %.3f m" % res["rmse"])
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def format_row(r):
    if "rmse" not in r:
        return "%-15s %-8s %s" % (r["name"], r["dataset"], r.get("error", "no ground truth"))
    timing = ""
    if "step_mean_ms" in r:
        timing = "%5.1f ms/scan (max %.0f)" % (r["step_mean_ms"], r["step_max_ms"])
    if "max_rss_mb" in r:
        timing += "  %.0f MB" % r["max_rss_mb"]
    return "%-15s %-9s ATE %7.3f m  max %7.3f  len %7.1f m  cov %5.1f %%  dt %+.3f s  %s" % (
        r["name"], r["dataset"], r["rmse"], r["max"], r["gt_length"], 100 * r["coverage"],
        r["time_offset"], timing)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tag", required=True, help="name of this benchmark run")
    parser.add_argument("--datasets", nargs="*", default=[], help="run all sequences of these")
    parser.add_argument("--sequences", nargs="*", default=[], help="run only these sequences")
    parser.add_argument("--registry", default=os.path.join(SCRIPT_DIR, "datasets.yaml"))
    parser.add_argument("--root", action="append", default=[], metavar="DATASET=PATH",
                        help="override a dataset root")
    parser.add_argument("--params", default=os.path.join(REPO_DIR, "config", "params.yaml"))
    parser.add_argument("--set", action="append", default=[], type=parse_override,
                        metavar="SECTION.KEY=VALUE",
                        help="override a config value ({out_dir} expands to the run folder)")
    parser.add_argument("--out", default=None, help="result root (default: <workspace>/results)")
    parser.add_argument("--bin-dir", default=None,
                        help="folder with an alternative process_bag + libbievr_lio.so")
    parser.add_argument("--ld-library-path", action="append", default=[], metavar="DIR",
                        help="extra library folder for the node (e.g. a local Ceres install)")
    parser.add_argument("--jobs", type=int, default=1, help="sequences to run in parallel")
    parser.add_argument("--threads", type=int, default=0, help="TBB threads per run (0 = auto)")
    parser.add_argument("--max-gap", type=float, default=1.0)
    parser.add_argument("--eval-only", action="store_true", help="re-evaluate existing runs")
    parser.add_argument("--plot", action="store_true", help="save an error plot per sequence")
    args = parser.parse_args()

    workspace = find_workspace(REPO_DIR)
    if workspace is None:
        sys.exit("Could not find a catkin workspace (devel/setup.bash) above %s" % REPO_DIR)
    if args.out is None:
        args.out = os.path.join(workspace, "results")
    with open(args.registry) as f:
        registry = yaml.safe_load(f)
    roots = dict(registry.get("roots", {}))
    for item in args.root:
        name, _, path = item.partition("=")
        roots[name] = path
    runs = collect_runs(registry, args.datasets, args.sequences, roots)
    if not runs:
        sys.exit("No sequences selected.")
    missing = [r["bag"] for r in runs if not os.path.isfile(r["bag"])]
    if missing and not args.eval_only:
        sys.exit("Missing bags:\n  " + "\n  ".join(missing))

    roscore = None
    args.master_uri = None
    if not args.eval_only:
        roscore, args.master_uri = start_roscore(workspace)

    results = []
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
            futures = {pool.submit(run_sequence, run, args, workspace): run for run in runs}
            for future in concurrent.futures.as_completed(futures):
                result = future.result()
                results.append(result)
                print(format_row(result), flush=True)
    finally:
        if roscore is not None:
            roscore.terminate()
            roscore.wait()

    order = {run["name"]: i for i, run in enumerate(runs)}
    results.sort(key=lambda r: order[r["name"]])
    summary_path = os.path.join(args.out, args.tag, "summary.json")
    summary = {}
    if os.path.isfile(summary_path):
        with open(summary_path) as f:
            summary = json.load(f)
    summary.update({r["name"]: r for r in results})
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=1)
    print("\n=== %s ===" % args.tag)
    for r in results:
        print(format_row(r))


if __name__ == "__main__":
    main()
