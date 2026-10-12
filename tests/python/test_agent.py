"""The agent with a scripted fake model: it must call the tools in order and feed the results back."""
import json
import os

import pytest

from huge_atlas_mcp import agent, tools

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_tool_specs():
    specs = {s["function"]["name"]: s["function"] for s in agent.tool_specs()}
    assert set(specs) == set(tools.TOOLS)
    start = specs["start_atlas"]["parameters"]
    assert start["required"] == ["model"] and start["properties"]["quality"]["type"] == "integer"
    assert start["properties"]["patches"]["type"] == "integer" and "patches" not in start["required"]
    assert "efficiency" in specs["start_atlas"]["description"]


def test_scripted_dialog(tmp_path, monkeypatch):
    monkeypatch.setenv("HUGE_ATLAS_OUTPUT", str(tmp_path))
    script = iter([
        ("open_mesh", {"path": os.path.join(ROOT, "data", "bunny.ply")}),
        ("start_atlas", {"model": "{model}", "quality": 0, "texture_size": 1024}),
        ("job_status", {"job": "{job}", "wait_seconds": 50}),
        ("get_metrics", {"model": "{model}"}),
        ("save_atlas", {"model": "{model}", "file_name": "bunny.obj"}),
    ])
    seen = {}

    def fake_chat(messages, specs):
        last = messages[-1]
        if last["role"] == "tool":
            data = json.loads(last["content"])
            assert "error" not in data, data
            seen.update({k: v for k, v in data.items() if k in ("model", "job")})
        try:
            name, args = next(script)
        except StopIteration:
            return {"role": "assistant", "content": "done"}
        args = {k: (v.format(**seen) if isinstance(v, str) else v) for k, v in args.items()}
        return {"role": "assistant", "content": None,
                "tool_calls": [{"id": f"c{len(messages)}", "type": "function", "function": {"name": name, "arguments": json.dumps(args)}}]}

    out = []
    msgs = agent.run_agent("make the atlas", chat=fake_chat, say=out.append)
    assert out[-1] == "done"
    assert (tmp_path / "bunny.obj").exists()
    assert sum(m["role"] == "tool" for m in msgs) == 5


def test_tool_error_goes_back_to_the_model():
    assert "error" in json.loads(agent.call_tool("get_metrics", {"model": "nope"}))
    assert "error" in json.loads(agent.call_tool("no_such_tool", {}))
