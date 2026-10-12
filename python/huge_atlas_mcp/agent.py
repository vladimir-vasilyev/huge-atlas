"""A small agent: any model behind an OpenAI-compatible API (DeepSeek, OpenAI, a local Ollama / LM Studio, ...) drives the same tools as the
MCP server. No MCP client is needed.

    export HUGE_ATLAS_API_KEY=...                     # or DEEPSEEK_API_KEY / OPENAI_API_KEY
    export HUGE_ATLAS_BASE_URL=https://api.deepseek.com   # default; HUGE_ATLAS_MODEL=deepseek-chat (default)
    huge-atlas-agent "make an atlas for scan.obj, 4096 texture, check the distortion and tune it if it is above 1.3"

Only the text of the tool results (numbers, reports) goes to the provider, never the mesh.
"""
from __future__ import annotations

import inspect
import json
import os
import sys
import typing
import urllib.error
import urllib.request
from typing import Any, Callable, Optional

from . import tools

_JSON_TYPES = {int: "integer", float: "number", bool: "boolean", str: "string"}


def _schema_of(fn: Callable) -> dict:
    props: dict[str, Any] = {}
    required = []
    hints = typing.get_type_hints(fn)
    for name, p in inspect.signature(fn).parameters.items():
        t = hints.get(name, str)
        optional = False
        if typing.get_origin(t) is typing.Union:
            args = [a for a in typing.get_args(t) if a is not type(None)]
            optional = len(args) != len(typing.get_args(t))
            t = args[0]
        props[name] = {"type": _JSON_TYPES.get(t, "string")}
        if p.default is inspect.Parameter.empty and not optional:
            required.append(name)
    return {"type": "function", "function": {"name": fn.__name__, "description": inspect.getdoc(fn) or "",
                                             "parameters": {"type": "object", "properties": props, "required": required}}}


def tool_specs() -> list[dict]:
    return [_schema_of(getattr(tools, n)) for n in tools.TOOLS]


def call_tool(name: str, arguments: dict) -> str:
    if name not in tools.TOOLS:
        return json.dumps({"error": f"unknown tool {name}"})
    try:
        return json.dumps(getattr(tools, name)(**arguments), default=str)
    except Exception as e:  # noqa: BLE001 - the model gets the message and can correct itself
        return json.dumps({"error": str(e)})


def http_chat(messages: list[dict], specs: list[dict]) -> dict:
    key = os.environ.get("HUGE_ATLAS_API_KEY") or os.environ.get("DEEPSEEK_API_KEY") or os.environ.get("OPENAI_API_KEY")
    base = os.environ.get("HUGE_ATLAS_BASE_URL", "https://api.deepseek.com").rstrip("/")
    model = os.environ.get("HUGE_ATLAS_MODEL", "deepseek-chat")
    if not key and "localhost" not in base and "127.0.0.1" not in base:
        raise SystemExit("set HUGE_ATLAS_API_KEY (or DEEPSEEK_API_KEY / OPENAI_API_KEY)")
    body = json.dumps({"model": model, "messages": messages, "tools": specs}).encode()
    req = urllib.request.Request(base + "/chat/completions", body, {"Content-Type": "application/json", "Authorization": f"Bearer {key or 'none'}"})
    try:
        with urllib.request.urlopen(req, timeout=300) as r:
            return json.loads(r.read())["choices"][0]["message"]
    except urllib.error.HTTPError as e:
        raise SystemExit(f"the API answered {e.code}: {e.read().decode(errors='replace')[:500]}")


def run_agent(task: str, chat: Callable[[list[dict], list[dict]], dict] = http_chat, max_steps: int = 30,
              say: Callable[[str], None] = print, messages: Optional[list[dict]] = None) -> list[dict]:
    """Runs the dialog until the model answers without a tool call. `messages` carries a conversation over (interactive use)."""
    specs = tool_specs()
    messages = messages if messages is not None else [{"role": "system", "content": tools.INSTRUCTIONS}]
    messages.append({"role": "user", "content": task})
    for _ in range(max_steps):
        reply = chat(messages, specs)
        messages.append({k: v for k, v in reply.items() if v is not None})
        if reply.get("content"):
            say(reply["content"])
        calls = reply.get("tool_calls") or []
        if not calls:
            return messages
        for c in calls:
            fn = c["function"]
            try:
                args = json.loads(fn.get("arguments") or "{}")
            except json.JSONDecodeError:
                args = {}
            say(f"-> {fn['name']}({', '.join(f'{k}={v!r}' for k, v in args.items())})")
            messages.append({"role": "tool", "tool_call_id": c["id"], "content": call_tool(fn["name"], args)})
    say("(stopped: too many steps)")
    return messages


def main() -> None:
    task = " ".join(sys.argv[1:]).strip()
    if task:
        run_agent(task)
        return
    history: list[dict] = [{"role": "system", "content": tools.INSTRUCTIONS}]
    print("huge-atlas agent. Describe what to do; an empty line quits.")
    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            break
        if not line:
            break
        run_agent(line, messages=history)


if __name__ == "__main__":
    main()
