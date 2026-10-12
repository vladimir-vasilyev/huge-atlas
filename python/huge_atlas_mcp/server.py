"""MCP server (stdio) for huge_atlas.

Config of an MCP client (Claude Desktop, Claude Code, Cursor, Cline, ...):
    {"mcpServers": {"huge-atlas": {"command": "huge-atlas-mcp"}}}

Environment: HUGE_ATLAS_OUTPUT is the only folder the server writes to (default ~/Documents/huge-atlas/output).
"""
from __future__ import annotations

try:  # mcp 2.x
    from mcp.server.mcpserver import MCPServer as _Server
except ImportError:  # mcp 1.x
    from mcp.server.fastmcp import FastMCP as _Server

from . import tools

mcp = _Server("huge-atlas", instructions=tools.INSTRUCTIONS)
for _name in tools.TOOLS:
    mcp.tool()(getattr(tools, _name))


def main() -> None:
    mcp.run()


if __name__ == "__main__":
    main()
