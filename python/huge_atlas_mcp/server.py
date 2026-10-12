"""MCP server (stdio) for huge_atlas.

Config of an MCP client (Claude Desktop, Claude Code):
    {"mcpServers": {"huge-atlas": {"command": "python", "args": ["-m", "huge_atlas_mcp"]}}}

Environment: HUGE_ATLAS_OUTPUT is the only folder the server writes to (default ~/Documents/huge-atlas/output).
"""
from __future__ import annotations

import itertools
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import huge_atlas as ha

try:  # mcp 2.x
    from mcp.server.mcpserver import MCPServer as _Server
except ImportError:  # mcp 1.x
    from mcp.server.fastmcp import FastMCP as _Server

INSTRUCTIONS = """\
Makes UV atlases for triangle meshes.
Typical run: open_mesh(path) -> start_atlas(model) -> job_status(job, wait_seconds=30) until it is done -> get_metrics(model).
Then read what was measured and, if needed, tune and run again with start_atlas (different settings), or repack() for another
texture size / mip levels (fast: the segmentation is kept). save_atlas(model, "name.obj") writes the result.
What to look at (get_metrics / get_layout):
  efficiency      share of the texture that holds charts; low (< 0.35) means too many small charts or a big mip gap.
  unresolved_patches  must be 0; > 0: raise max_depth or patches, or quality.
  faces_without_uv    must be 0.
  layout.new.l2_p95 / conformal_p95   distortion, 1.0 is perfect; > 1.3 means the charts are stretched: more patches (quality 2).
  layout.new.seam_ratio   share of the edges that are seams; grows with the number of patches.
  mip_levels      how many mip levels stay clean: the gap between charts is 2 * (ceil(1.5 * 2^L) - 1) texels (L = 0..5: 2, 4, 10, 22, 46, 94).
More patches (quality 2, or patches=N): less distortion, more seams. Fewer patches: the opposite. A bigger texture_size or fewer
mip_levels: better efficiency. Models above 3M faces use the flat-array pipeline automatically (stats are comparable).
"""

mcp = _Server("huge-atlas", instructions=INSTRUCTIONS)


def _output_dir() -> Path:
    d = Path(os.environ.get("HUGE_ATLAS_OUTPUT", Path.home() / "Documents" / "huge-atlas" / "output")).expanduser()
    d.mkdir(parents=True, exist_ok=True)
    return d.resolve()


@dataclass
class Model:
    id: str
    path: str
    positions: Any
    faces: Any
    uv: Any
    texture: Any
    atlas: Optional[ha.Atlas] = None
    last_settings: dict = field(default_factory=dict)


@dataclass
class Job:
    id: str
    model: str
    started: float
    messages: list = field(default_factory=list)
    done: bool = False
    error: Optional[str] = None
    cancel: threading.Event = field(default_factory=threading.Event)
    finished: float = 0.0


_models: dict[str, Model] = {}
_jobs: dict[str, Job] = {}
_ids = itertools.count(1)
_lock = threading.Lock()


def _model(model: str) -> Model:
    if model not in _models:
        raise ValueError(f"unknown model {model!r}; open_mesh first (open: {sorted(_models)})")
    return _models[model]


@mcp.tool()
def open_mesh(path: str) -> dict:
    """Read an OBJ or PLY file. Returns the id of the model and what the file has (faces, texture)."""
    p = Path(path).expanduser()
    if not p.is_file():
        raise ValueError(f"no such file: {p}")
    m = ha.load_mesh(str(p))
    mid = f"m{next(_ids)}"
    model = Model(mid, str(p), m["positions"], m["faces"], m["uv"], m["texture"])
    _models[mid] = model
    faces = int(m["faces"].shape[0])
    return {
        "model": mid,
        "file": p.name,
        "vertices": int(m["positions"].shape[0]),
        "faces": faces,
        "has_uv": m["uv"] is not None,
        "has_texture": m["texture"] is not None,
        "texture_size": None if m["texture"] is None else list(m["texture"].shape[:2]),
        "mode": "huge (flat arrays)" if faces > 3_000_000 else "regular",
        "suggested_texture_size": 2048 if faces < 1_000_000 else 4096 if faces < 8_000_000 else 8192,
    }


@mcp.tool()
def start_atlas(
    model: str,
    quality: int = 1,
    patches: Optional[int] = None,
    texture_size: int = 2048,
    mip_levels: int = 3,
    smoothing: float = 0.1,
    normal_iterations: int = 0,
    normal_sigma: float = 25.0,
    lloyd_iterations: int = 30,
    max_depth: int = 10,
    split_at_once: int = 40000,
    allow_lscm: bool = True,
    repair_near_misses: bool = True,
    simplify_percent: float = 0.0,
) -> dict:
    """Start making the atlas in the background (a run takes seconds to minutes). Returns a job id for job_status.

    quality 0/1/2 (fast/balanced/detailed) sets how many patches to start from; patches=N overrides it.
    texture_size 256..8192 (power of two); mip_levels 0..5 how many mip levels stay clean (the gap between charts follows).
    smoothing: border smoothness of the patches (0 = off). normal_iterations/normal_sigma: smooth the normal field first.
    max_depth: how many times a bad patch may be split again. simplify_percent: huge mode only, size of the simplified copy.
    The texture of the file (if it has one) is baked into the atlas."""
    mdl = _model(model)
    if any(j.model == model and not j.done for j in _jobs.values()):
        raise ValueError("this model is still being worked on; wait for the job or cancel it")
    settings = dict(
        quality=quality, patches=patches, texture_size=texture_size, mip_levels=mip_levels, smoothing=smoothing,
        normal_iterations=normal_iterations, normal_sigma=normal_sigma, lloyd_iterations=lloyd_iterations, max_depth=max_depth,
        split_at_once=split_at_once, allow_lscm=allow_lscm, repair_near_misses=repair_near_misses, simplify_percent=simplify_percent)
    job = Job(f"j{next(_ids)}", model, time.time())
    _jobs[job.id] = job

    def work() -> None:
        try:
            kw = dict(settings, progress=job.messages.append, cancel=job.cancel)
            if mdl.atlas is not None:
                mdl.atlas.rerun(**kw)
            else:
                mdl.atlas = ha.make_atlas(mdl.positions, mdl.faces, uv=mdl.uv, texture=mdl.texture, **kw)
            mdl.last_settings = settings
        except Exception as e:  # noqa: BLE001 - reported to the client
            job.error = str(e)
        finally:
            job.done = True
            job.finished = time.time()

    threading.Thread(target=work, daemon=True).start()
    return {"job": job.id, "model": model, "settings": settings}


@mcp.tool()
def job_status(job: str, wait_seconds: float = 0.0) -> dict:
    """State of a job: running (with the last stage) / done / failed / cancelled. wait_seconds (up to 55) waits for the end."""
    j = _jobs.get(job)
    if j is None:
        raise ValueError(f"unknown job {job!r}")
    deadline = time.time() + max(0.0, min(wait_seconds, 55.0))
    while not j.done and time.time() < deadline:
        time.sleep(0.2)
    state = "running" if not j.done else "cancelled" if j.error == "cancelled" else "failed" if j.error else "done"
    out = {"job": job, "model": j.model, "state": state, "seconds": round((j.finished or time.time()) - j.started, 1),
           "stage": j.messages[-1] if j.messages else "starting"}
    if j.error and state == "failed":
        out["error"] = j.error
    if state == "done":
        out["metrics"] = _model(j.model).atlas.metrics()
    return out


@mcp.tool()
def cancel_job(job: str) -> dict:
    """Ask a running job to stop (it stops at the next stage; a stage of a huge scan can take a while)."""
    j = _jobs.get(job)
    if j is None:
        raise ValueError(f"unknown job {job!r}")
    j.cancel.set()
    return {"job": job, "cancelling": not j.done}


def _atlas(model: str) -> ha.Atlas:
    mdl = _model(model)
    if mdl.atlas is None:
        raise ValueError("there is no atlas yet: start_atlas and wait for the job")
    return mdl.atlas


@mcp.tool()
def get_metrics(model: str) -> dict:
    """Numbers of the last atlas (patches, unresolved patches, efficiency, gutter, seconds) and the settings that made it."""
    return {"metrics": _atlas(model).metrics(), "settings": _model(model).last_settings}


@mcp.tool()
def get_layout(model: str) -> dict:
    """Distortion (l2_*, conformal_p95), seams, density, flipped faces and mip safety of the new atlas, and of the old layout of
    the file for comparison (null when the file had no texture coordinates)."""
    return _atlas(model).layout()


@mcp.tool()
def get_report(model: str) -> str:
    """What the last run did as text: patches, how they were unfolded, splits, packing."""
    return _atlas(model).report()


@mcp.tool()
def repack(model: str, texture_size: int, mip_levels: Optional[int] = None) -> dict:
    """Pack and bake again with another texture size / mip levels; the segmentation and the unfolding stay (fast)."""
    a = _atlas(model)
    a.repack(texture_size, mip_levels)
    _model(model).last_settings.update(texture_size=texture_size, **({} if mip_levels is None else {"mip_levels": mip_levels}))
    return a.metrics()


@mcp.tool()
def save_atlas(model: str, file_name: str) -> dict:
    """Write the mesh with the atlas coordinates (OBJ+MTL or PLY by the extension) and <name>_atlas.png into the output folder."""
    name = Path(file_name).name   # no folders: the server writes only into its output folder
    if not name.lower().endswith((".obj", ".ply")):
        name += ".obj"
    out = _output_dir() / name
    _atlas(model).save(str(out))
    return {"saved": str(out), "texture": str(out.with_name(out.stem + "_atlas.png"))}


def main() -> None:
    mcp.run()


if __name__ == "__main__":
    main()
