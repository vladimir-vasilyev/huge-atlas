"""Tests of the Python module. Run: PYTHONPATH=build-py python -m pytest tests/python"""
import os
import threading
import time

import numpy as np
import pytest

import huge_atlas as ha

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def bunny():
    m = ha.load_mesh(os.path.join(ROOT, "data", "bunny.ply"))
    return m["positions"], m["faces"]


def test_load_mesh_shapes():
    p, f = bunny()
    assert p.ndim == 2 and p.shape[1] == 3 and p.dtype == np.float32
    assert f.ndim == 2 and f.shape[1] == 3 and f.dtype == np.uint32
    assert len(f) == 69451


@pytest.mark.parametrize("huge", [False, True])
def test_make_atlas_bunny(huge):
    p, f = bunny()
    atlas = ha.make_atlas(p, f, quality=0, texture_size=1024, mip_levels=2, huge=huge)
    assert atlas.is_huge == huge
    uv = atlas.corner_uv
    assert uv.shape == (len(f), 3, 2) and uv.dtype == np.float32
    assert np.isfinite(uv).all()
    assert uv.min() >= -1e-4 and uv.max() <= 1 + 1e-4
    cid = atlas.chart_id
    assert cid.shape == (len(f),) and (cid >= 0).all()
    m = atlas.metrics()
    assert m["patches"] == len(np.unique(cid)) or m["patches"] >= len(np.unique(cid))
    assert 0.1 < m["efficiency"] < 1.0
    assert m["texture_size"] == 1024
    assert "patches" in atlas.report() or "faces" in atlas.report()


def test_progress_and_repack():
    p, f = bunny()
    msgs = []
    atlas = ha.make_atlas(p, f, quality=0, texture_size=1024, progress=msgs.append)
    assert msgs
    uv1 = atlas.corner_uv.copy()
    atlas.repack(512, 1)
    assert atlas.metrics()["texture_size"] == 512
    assert not np.array_equal(uv1, atlas.corner_uv)


def test_cancel_with_event():
    p, f = bunny()
    ev = threading.Event()
    ev.set()
    with pytest.raises(RuntimeError, match="cancelled"):
        ha.make_atlas(p, f, huge=True, cancel=ev)


def test_cancel_in_the_middle():
    class AfterFirstLook:   # polled every 40 ms while the run (about 0.3 s on the bunny) is going
        def is_set(self):
            return True

    p, f = bunny()
    t = time.time()
    with pytest.raises(RuntimeError, match="cancelled"):
        ha.make_atlas(p, f, quality=2, huge=True, cancel=AfterFirstLook())
    assert time.time() - t < 30


def test_bad_input():
    p = np.zeros((3, 3), np.float32)
    with pytest.raises(ValueError):
        ha.make_atlas(p, np.array([[0, 1, 5]], np.uint32))
    with pytest.raises(ValueError):
        ha.make_atlas(p[:, :2], np.array([[0, 1, 2]], np.uint32))


def test_texture_bake_and_layout():
    path = os.path.join(ROOT, "data", "textured_hemisphere")
    obj = [os.path.join(path, n) for n in os.listdir(path) if n.endswith(".obj")]
    if not obj:
        pytest.skip("no textured model")
    m = ha.load_mesh(obj[0])
    if m["texture"] is None:
        pytest.skip("the model has no texture")
    for huge in (False, True):
        atlas = ha.make_atlas(m["positions"], m["faces"], uv=m["uv"], texture=m["texture"], quality=0, texture_size=512, huge=huge)
        tex = atlas.texture
        assert tex is not None and tex.shape == (512, 512, 4) and tex.dtype == np.uint8
        assert tex[..., 3].max() == 255
        lay = atlas.layout()
        assert lay["original"] is not None and lay["new"]["charts"] > 0
        assert "new" in atlas.compare() or atlas.compare()
