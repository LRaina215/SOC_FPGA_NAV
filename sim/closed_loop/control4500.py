"""Reuse of the in-repository control fork, imported from 6443f56, on Windows.

Only asset paths in a generated MJCF copy are changed. Policy weights, source
runtime, actuator law and observation/history contract come from the selected tree.
"""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPO = ROOT / "control"
FROZEN_REPO = ROOT.parent / "control_repo_snapshot_20261021"


def reference_path(path, repo=DEFAULT_REPO):
    """Git symlinks may be checked out as small text files on Windows."""
    path = Path(path).resolve()
    for _ in range(8):
        if path.stat().st_size > 512:
            return path
        try:
            value = path.read_text(encoding="utf-8").strip()
        except UnicodeError:
            return path
        target = (path.parent / value).resolve()
        if not target.is_relative_to(Path(repo).resolve()) or not target.is_file() or target == path:
            return path
        path = target
    raise ValueError("Reference chain exceeds eight entries")


def prepare_xml(out, repo=DEFAULT_REPO):
    repo, out = Path(repo).resolve(), Path(out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    xml = ET.parse(repo / "sim2sim/model4500_mujoco/robot.xml")
    for mesh in xml.findall("./asset/mesh"):
        original = mesh.get("file")
        relative = original.split("/fpga/", 1)[-1]
        target = reference_path(repo / relative, repo)
        mesh.set("file", str(target))
    path = out / "robot.xml"
    xml.write(path, encoding="unicode")
    return path


def load_runtime(out=ROOT / "sim/build/control4500", repo=DEFAULT_REPO):
    repo, out = Path(repo).resolve(), Path(out).resolve()
    prepare_xml(out, repo)
    source = repo / "sim2sim/model4500_mujoco"
    sys.path.insert(0, str(source))
    spec = importlib.util.spec_from_file_location("fixed4500_runtime", source / "runtime.py")
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    runtime.ROOT = out
    runtime.CONFIG = reference_path(repo / "model/wheel_velocity_original/parking_config.json", repo)
    checkpoint = reference_path(repo / "model/wheel_velocity_original/model_4500.pt", repo)
    runtime.torch.set_num_threads(1)
    return runtime, runtime.Policy(checkpoint=checkpoint)
