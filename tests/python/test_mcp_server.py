"""The MCP server driven by a real MCP client over stdio. Needs the `mcp` package; skipped without it."""
import asyncio
import json
import os
import sys
import tempfile

import pytest

pytest.importorskip("mcp")
from mcp import ClientSession, StdioServerParameters  # noqa: E402
from mcp.client.stdio import stdio_client  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def payload(result):
    if getattr(result, "structuredContent", None):
        sc = result.structuredContent
        return sc.get("result", sc) if isinstance(sc, dict) else sc
    return json.loads(result.content[0].text)


def test_full_session(tmp_path):
    env = dict(os.environ, HUGE_ATLAS_OUTPUT=str(tmp_path), PYTHONPATH=os.pathsep.join(
        [os.path.join(ROOT, "python")] + [p for p in os.environ.get("PYTHONPATH", "").split(os.pathsep) if p]))
    params = StdioServerParameters(command=sys.executable, args=["-m", "huge_atlas_mcp"], env=env)

    async def scenario():
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write) as s:
                await s.initialize()
                tools = {t.name for t in (await s.list_tools()).tools}
                assert {"open_mesh", "start_atlas", "job_status", "get_metrics", "get_layout", "repack", "save_atlas", "cancel_job"} <= tools
                m = payload(await s.call_tool("open_mesh", {"path": os.path.join(ROOT, "data", "bunny.ply")}))
                assert m["faces"] == 69451
                job = payload(await s.call_tool("start_atlas", {"model": m["model"], "quality": 0, "texture_size": 1024}))["job"]
                st = payload(await s.call_tool("job_status", {"job": job, "wait_seconds": 50}))
                assert st["state"] == "done", st
                met = payload(await s.call_tool("get_metrics", {"model": m["model"]}))["metrics"]
                assert met["texture_size"] == 1024 and met["unresolved_patches"] == 0
                # tune and run again
                job = payload(await s.call_tool("start_atlas", {"model": m["model"], "quality": 2, "texture_size": 2048, "mip_levels": 2}))["job"]
                st = payload(await s.call_tool("job_status", {"job": job, "wait_seconds": 50}))
                assert st["state"] == "done" and st["metrics"]["texture_size"] == 2048
                assert st["metrics"]["patches"] > met["patches"]
                lay = payload(await s.call_tool("get_layout", {"model": m["model"]}))
                assert lay["new"]["charts"] > 0 and lay["original"] is None
                rp = payload(await s.call_tool("repack", {"model": m["model"], "texture_size": 512}))
                assert rp["texture_size"] == 512
                saved = payload(await s.call_tool("save_atlas", {"model": m["model"], "file_name": "../escape.obj"}))
                assert os.path.dirname(saved["saved"]) == str(tmp_path) or os.path.realpath(os.path.dirname(saved["saved"])) == os.path.realpath(str(tmp_path))
                assert os.path.exists(saved["saved"]) and os.path.exists(saved["texture"])

    asyncio.run(scenario())
