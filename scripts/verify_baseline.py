#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parent.parent
BASELINE = ROOT / "docs/baseline"


def manifest(directory):
    entries = {}
    for path in sorted(directory.rglob("*")):
        if path.is_symlink():
            raise RuntimeError(f"Unexpected symlink: {path}")
        if path.is_file():
            entries[path.relative_to(directory).as_posix()] = {
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "mode": f"{stat.S_IMODE(path.stat().st_mode):04o}",
            }
    return entries


def main():
    expected = json.loads((BASELINE / "planner-current.json").read_text())
    package = ROOT / "transplant_ws/src/path_searching"
    if manifest(package) != expected:
        raise RuntimeError("Current planner differs from staged-baseline snapshot")
    print(f"PASS current planner contents and modes: {len(expected)} files")
    evidence = json.loads((BASELINE / "evidence.json").read_text())
    if manifest(BASELINE / "evidence") != evidence:
        raise RuntimeError("Archived evidence changed")
    print(f"PASS historical evidence contents and modes: {len(evidence)} files")
    with tempfile.TemporaryDirectory(prefix="planner-baseline-") as temporary:
        workspace = Path(temporary)
        restored = workspace / "transplant_ws/src/path_searching"
        shutil.copytree(package, restored)
        subprocess.run([
            "git", "apply", "--reverse", "--check", "-p2",
            str(BASELINE / "import-to-t02a.patch"),
        ], cwd=workspace, check=True)
        subprocess.run([
            "git", "apply", "--reverse", "-p2",
            str(BASELINE / "import-to-t02a.patch"),
        ], cwd=workspace, check=True)
        original = json.loads((BASELINE / "planner-import.json").read_text())
        if manifest(restored) != original:
            raise RuntimeError("Reverse patch did not reconstruct the imported planner")
        print(f"PASS reverse patch reconstructs original import: {len(original)} files")
    staged = subprocess.check_output(
        ["git", "ls-files", "--stage", "-z"], cwd=ROOT,
    ).split(b"\0")
    tracked = 0
    for record in filter(None, staged):
        metadata, raw_name = record.split(b"\t", 1)
        mode, object_id, stage = metadata.split()
        if stage != b"0" or mode not in (b"100644", b"100755"):
            raise RuntimeError(f"Unexpected staged entry: {record!r}")
        path = ROOT / raw_name.decode()
        staged_bytes = subprocess.check_output(["git", "cat-file", "blob", object_id], cwd=ROOT)
        if staged_bytes != path.read_bytes():
            raise RuntimeError(f"Staged/worktree mismatch: {path}")
        executable = bool(path.stat().st_mode & stat.S_IXUSR)
        if executable != (mode == b"100755"):
            raise RuntimeError(f"Staged/worktree executable-mode mismatch: {path}")
        tracked += 1
    if tracked == 0:
        raise RuntimeError("No staged files; stage the explicit baseline paths first")
    print(f"PASS staged/worktree byte and executable-mode equivalence: {tracked} files")
    with tempfile.TemporaryDirectory(prefix="planner-index-export-") as temporary:
        exported = Path(temporary)
        subprocess.run([
            "git", "checkout-index", "--all", f"--prefix={exported}/",
        ], cwd=ROOT, check=True)
        for name, entry in manifest(exported).items():
            worktree = ROOT / name
            if hashlib.sha256(worktree.read_bytes()).hexdigest() != entry["sha256"]:
                raise RuntimeError(f"Export/worktree mismatch: {name}")
        if len(manifest(exported)) != tracked:
            raise RuntimeError("Staged export file count mismatch")
        print(f"PASS temporary export from index: {tracked} files")
    print("NOTE: verification only; no build, algorithm tests, commit or tag performed")


if __name__ == "__main__":
    main()
