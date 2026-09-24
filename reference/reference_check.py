#!/usr/bin/env python3
"""Build the Ubuntu-24.04 Docker reference and compare the current Rust port.

Requires Python 3.10+, Docker, Cargo, and the sibling SCRIMMAGE checkout.
No host C++ toolchain is needed.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
BRANCH = "Ubuntu-24.04"
PLATFORM = "linux/amd64"  # Upstream install-jsbsim.sh downloads amd64 .deb files.
DEPENDENCY_DOCKERFILE = "ci/dockerfiles/ubuntu-24.04-slim-dependency-only"
MISSIONS = (
    "straight-no-gui.xml",
    "test_missions/straight_cpu.xml",
    "test_missions/straight_cpu_mul.xml",
    "verification/aircraft-substeps-spawning.xml",
    "verification/noisy-state-bias.xml",
    "networks-local-global.xml",
    "fixed-wing-6dof.xml",
    "multirotor.xml",
)
OUTPUT_FILES = ("frames.bin", "events.json", "summary.csv")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + "\n")


def capture(command, cwd=ROOT):
    return subprocess.check_output(command, cwd=cwd, text=True).strip()


def run_logged(command, log, *, timeout, cwd=ROOT, allow_failure=False):
    """Retain full logs and print a useful tail on failure, including timeouts."""
    command = [str(argument) for argument in command]
    print(f"  {Path(command[0]).name}: {log}", flush=True)
    with Path(log).open("w") as stream:
        stream.write(json.dumps(command) + "\n")
        stream.flush()
        try:
            result = subprocess.run(
                command, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT,
                timeout=timeout, check=False,
            )
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(f"timed out after {timeout}s; inspect {log}") from error
    if result.returncode and not allow_failure:
        tail = "\n".join(Path(log).read_text(errors="replace").splitlines()[-25:])
        raise RuntimeError(f"command failed ({result.returncode}); {log}\n{tail}")
    return result.returncode


def allocate_output(explicit, runs):
    if explicit is not None:
        explicit = explicit.resolve()
        explicit.parent.mkdir(parents=True, exist_ok=True)
        explicit.mkdir()  # Never reuse an earlier check, even an empty one.
        return explicit
    runs.mkdir(parents=True, exist_ok=True)
    numbers = [int(p.name[15:]) for p in runs.iterdir()
               if p.name.startswith("reference-check") and p.name[15:].isdigit()]
    number = max(numbers, default=-1) + 1
    while True:
        output = runs / f"reference-check{number:03}"
        try:
            output.mkdir()
            return output.resolve()
        except FileExistsError:
            number += 1


def prepare_mission(mission, cpp):
    """Keep physical inputs unchanged; force the C++ reference headless/serial."""
    cpp.mkdir()
    shutil.copyfile(mission, cpp / "input.xml")
    tree = ET.parse(mission)
    root = tree.getroot()
    if root.tag != "runscript":
        raise ValueError("expected a runscript mission")
    if any(node.tag.rsplit("}", 1)[-1] == "include" for node in root.iter()):
        raise ValueError("reference_check requires an expanded mission (XIncludes unsupported)")
    run = root.find("run")
    if run is None:
        raise ValueError("mission has no run element")
    run_overrides = {
        "enable_gui": "false", "network_gui": "false",
        "start_paused": "false", "time_warp": "0",
    }
    root_overrides = {
        "log_dir": "/run/logs", "create_latest_dir": "false",
        "multi_threaded": "false", "display_progress": "false",
    }
    run.attrib.update(run_overrides)
    for key, value in root_overrides.items():
        nodes = root.findall(key)
        if not nodes:
            nodes = [ET.SubElement(root, key)]
        for node in nodes:
            node.text = value
    effective = cpp / "effective.xml"
    tree.write(effective, encoding="utf-8", xml_declaration=True)
    return {
        "input_sha256": sha256(cpp / "input.xml"),
        "effective_sha256": sha256(effective),
        "run_overrides": run_overrides, "root_overrides": root_overrides,
    }


def same_outputs(left, right):
    return {name: sha256(left / name) == sha256(right / name) for name in OUTPUT_FILES}


def build_reference(args, output):
    commit = capture(["git", "-C", str(args.source), "rev-parse", f"refs/heads/{BRANCH}^{{commit}}"])
    context = output / "build-context"
    context.mkdir()
    dependency_file = context / "Dependencies.Dockerfile"
    dependency_file.write_bytes(subprocess.check_output([
        "git", "-C", str(args.source), "show", f"{commit}:{DEPENDENCY_DOCKERFILE}",
    ]))
    # Archive only committed branch files. Untracked docs and native patches are
    # not silently incorporated, and the sibling checkout is never modified.
    with (context / "source.tar").open("wb") as archive:
        subprocess.run([
            "git", "-C", str(args.source), "archive", "--format=tar", "--prefix=source/", commit,
        ], stdout=archive, check=True)
    shutil.copyfile(ROOT / "reference/Dockerfile", context / "Dockerfile")
    # Only the constant-command driver is ours; Multirotor remains upstream code.
    shutil.copytree(ROOT / "reference/fixtures", context / "fixtures")
    fixture_hashes = {path.name: sha256(path) for path in sorted((context / "fixtures").iterdir())}
    write_json(output / "source.json", {
        "branch": BRANCH, "commit": commit, "source_checkout": str(args.source),
        "source_archive_sha256": sha256(context / "source.tar"),
        "dependency_dockerfile_sha256": sha256(dependency_file),
        "build_dockerfile_sha256": sha256(context / "Dockerfile"),
        "fixture_sha256": fixture_hashes,
    })
    dependency_tag = f"scrimmage-rs-reference-deps:{sha256(dependency_file)[:12]}"
    image_tag = f"scrimmage-rs-reference:{commit[:12]}"
    dependency_id = output / "dependencies-image-id.txt"
    image_id = output / "reference-image-id.txt"
    common = ["docker", "build", "--platform", PLATFORM, "--progress", "plain"]
    run_logged(common + [
        "--file", dependency_file, "--tag", dependency_tag,
        "--iidfile", dependency_id, context,
    ], output / "dependencies-build.log", timeout=args.build_timeout)
    run_logged(common + [
        "--file", context / "Dockerfile", "--tag", image_tag,
        "--build-arg", f"DEPENDENCY_IMAGE={dependency_tag}",
        "--build-arg", f"SOURCE_COMMIT={commit}",
        "--build-arg", f"BUILD_JOBS={args.jobs}", "--iidfile", image_id, context,
    ], output / "reference-build.log", timeout=args.build_timeout)
    image = image_id.read_text().strip()
    inspect = json.loads(capture(["docker", "image", "inspect", image]))[0]
    run_logged([
        "docker", "run", "--rm", "--network", "none", "--platform", PLATFORM,
        "--entrypoint", "/bin/sh", image, "-c",
        "g++ --version && uname -m && cat /etc/os-release && dpkg-query -W",
    ], output / "cpp-environment.log", timeout=args.timeout)
    return {
        "branch": BRANCH, "commit": commit, "source_checkout": str(args.source),
        "source_policy": "committed local branch snapshot; no worktree edits",
        "source_archive_sha256": sha256(context / "source.tar"),
        "dependency_dockerfile_sha256": sha256(dependency_file),
        "dependency_image": dependency_id.read_text().strip(),
        "build_dockerfile_sha256": sha256(context / "Dockerfile"),
        "image": image, "platform": PLATFORM,
        "fixture_sha256": fixture_hashes,
        "image_architecture": inspect["Architecture"],
    }


def build_rust(args, output):
    run_logged([
        "cargo", "build", "--locked", "--package", "scrimmage-rs", "--bin", "scrimmage",
    ], output / "rust-build.log", timeout=args.build_timeout)
    metadata = json.loads(capture(["cargo", "metadata", "--no-deps", "--format-version", "1"]))
    executable = "scrimmage.exe" if os.name == "nt" else "scrimmage"
    binary = Path(metadata["target_directory"]) / "debug" / executable
    files = [ROOT / "Cargo.toml", ROOT / "Cargo.lock"]
    files += [path for path in (ROOT / "crates").rglob("*")
              if path.is_file() and path.suffix in {".rs", ".xml", ".toml"}]
    return binary, {
        "binary": str(binary), "binary_sha256": sha256(binary),
        "compiler": capture(["rustc", "--version", "--verbose"]),
        "host": platform.platform(),
        "source_hashes": {str(path.relative_to(ROOT)): sha256(path) for path in sorted(files)},
    }


def check_mission(args, mission, directory, image, rust):
    directory.mkdir()
    result = {"mission": str(mission), "passed": False}
    try:
        cpp = directory / "cpp"
        result["input"] = prepare_mission(mission, cpp)
        # Only this check's output directory is writable inside the container.
        # A unique name permits cleanup if the Docker client times out.
        container = f"scrimmage-check-{os.getpid()}-{time.time_ns()}"
        try:
            run_logged([
                "docker", "run", "--rm", "--network", "none", "--platform", PLATFORM,
                "--name", container,
                "--mount", f"type=bind,source={cpp},target=/run",
                image, "/run/effective.xml",
            ], cpp / "console.log", timeout=args.timeout)
        finally:
            subprocess.run(["docker", "rm", "--force", container],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        frames = list((cpp / "logs").rglob("frames.bin"))
        if len(frames) != 1 or not frames[0].stat().st_size:
            raise RuntimeError(f"expected one nonempty C++ frames.bin; inspect {cpp}")
        cpp_logs = frames[0].parent
        result["cpp_log_directory"] = str(cpp_logs.relative_to(directory))
        result["cpp_output_hashes"] = {
            name: sha256(cpp_logs / name) for name in ("frames.bin", "summary.csv")
        }
        # Use the saved input for Rust too; neither simulator may rewrite it.
        for workers in (1, 2, 8):
            run_logged([
                rust, "run", cpp / "input.xml", "--root", ROOT,
                "--output", directory / f"rust-{workers}",
                "--workers", workers, "--no-rerun",
            ], directory / f"rust-{workers}.log", timeout=args.timeout)
        serial = directory / "rust-1"
        result["worker_equality"] = {
            str(workers): same_outputs(serial, directory / f"rust-{workers}") for workers in (2, 8)
        }
        run_logged([
            rust, "run", cpp / "input.xml", "--root", ROOT,
            "--output", directory / "rust-rerun", "--workers", "8", "--headless",
        ], directory / "rust-rerun.log", timeout=args.timeout)
        result["recording_equality"] = same_outputs(serial, directory / "rust-rerun")
        recording = directory / "rust-rerun/recording.rrd"
        if not recording.is_file() or not recording.stat().st_size:
            raise RuntimeError("headless Rerun run did not produce a nonempty recording")
        result["recording_sha256"] = sha256(recording)
        result["rust_output_hashes"] = {
            variant: {name: sha256(directory / variant / name) for name in OUTPUT_FILES}
            for variant in ("rust-1", "rust-2", "rust-8", "rust-rerun")
        }
        report = directory / "comparison.json"
        code = run_logged([
            sys.executable, ROOT / "reference/compare.py", cpp_logs, serial, "--report", report,
        ], directory / "comparison.log", timeout=args.timeout, allow_failure=True)
        result["comparison"] = json.loads(report.read_text())
        result["passed"] = (
            code == 0 and result["comparison"]["passed"]
            and all(all(files.values()) for files in result["worker_equality"].values())
            and all(result["recording_equality"].values())
        )
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        result["error"] = str(error)
    write_json(directory / "result.json", result)
    return result


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT.parent / "scrimmage",
                        help="C++ checkout containing the local Ubuntu-24.04 branch")
    parser.add_argument("--output", type=Path, help="new check directory (default: runs/reference-checkNNN)")
    parser.add_argument("--mission", type=Path, action="append",
                        help="repeat to replace the default mission matrix; requires expanded XML")
    parser.add_argument("--jobs", type=positive_int, default=4, help="parallel C++ compilation jobs")
    parser.add_argument("--timeout", type=positive_int, default=120, help="per-run timeout in seconds")
    parser.add_argument("--build-timeout", type=positive_int, default=3600, help="per-build timeout in seconds")
    args = parser.parse_args(argv)
    args.source = args.source.resolve()
    missions = args.mission or [ROOT / "missions" / name for name in MISSIONS]
    missions = [mission.resolve(strict=True) for mission in missions]
    capture(["docker", "version", "--format", "{{.Server.Version}}"])
    output = allocate_output(args.output, ROOT / "runs")
    print(f"Reference check: {output}", flush=True)
    report = {
        "passed": False, "cases": [], "platform": PLATFORM,
        "direct_cpp_event_stream_compared": False,
        "native_viewer_launched": False,
        "notes": [
            "C++ frames and team summaries use reference/compare.py's compatibility tolerances.",
            "Rust worker/recording checks compare frames, events, and summaries byte-for-byte.",
            "Headless recording is tested; interactive viewer QA is a separate workflow.",
            "Default random engines/distributions can differ between libc++ and libstdc++.",
        ],
    }
    try:
        report["cpp"] = build_reference(args, output)
        rust, report["rust"] = build_rust(args, output)
        for index, mission in enumerate(missions):
            print(f"Checking {mission.name}", flush=True)
            result = check_mission(args, mission, output / f"{index:02}-{mission.stem}",
                                   report["cpp"]["image"], rust)
            report["cases"].append(result)
            write_json(output / "report.json", report)
            print(f"  {'PASS' if result['passed'] else 'FAIL'}: {mission.name}", flush=True)
        report["passed"] = bool(report["cases"]) and all(case["passed"] for case in report["cases"])
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        report["error"] = str(error)
        print(error, file=sys.stderr)
    finally:
        write_json(output / "report.json", report)
    print(f"{'PASS' if report['passed'] else 'FAIL'}: {output / 'report.json'}", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        raise SystemExit(str(error)) from error
