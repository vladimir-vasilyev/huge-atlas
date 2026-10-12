# huge-atlas for Python

UV atlases for triangle meshes: VSA segmentation, LinABF unfolding, Horizon packing with room for mip levels, texture baking.
Works on numpy arrays; scans up to 100M faces go through a flat-array pipeline.

```python
import huge_atlas as ha

mesh = ha.load_mesh("scan.obj")            # positions (N,3), faces (F,3), uv (F,3,2) and texture (H,W,4) when the file has them
atlas = ha.make_atlas(mesh["positions"], mesh["faces"], uv=mesh["uv"], texture=mesh["texture"],
                      quality=1, texture_size=4096, mip_levels=3, progress=print)

atlas.corner_uv      # (F,3,2) float32, new coordinates in [0,1]
atlas.chart_id       # (F,) int32
atlas.texture        # old texture baked into the atlas, (R,R,4) uint8
atlas.metrics()      # patches, efficiency, gutter, seconds, ...
atlas.layout()       # distortion, seams, mip safety: new atlas and the old layout
atlas.repack(2048)   # another texture size / mip levels without running the segmentation again
atlas.save("out.obj")
```

Ctrl+C and `cancel=threading.Event()` stop a long run (at the next stage).

## MCP server

An LLM client (Claude Desktop, Claude Code) can open a mesh, make the atlas, read the numbers and tune the settings:

```
pip install "huge-atlas[mcp]"
```

```json
{"mcpServers": {"huge-atlas": {"command": "huge-atlas-mcp"}}}
```

Tools: `open_mesh`, `start_atlas`, `job_status`, `cancel_job`, `get_metrics`, `get_layout`, `get_report`, `repack`, `save_atlas`.
The server writes only into `HUGE_ATLAS_OUTPUT` (default `~/Documents/huge-atlas/output`).

## Without an MCP client: the built-in agent

Any model behind an OpenAI-compatible API (DeepSeek, OpenAI, a local Ollama or LM Studio) can drive the same tools:

```
export HUGE_ATLAS_API_KEY=...                          # or DEEPSEEK_API_KEY / OPENAI_API_KEY
export HUGE_ATLAS_BASE_URL=https://api.deepseek.com    # the default; HUGE_ATLAS_MODEL=deepseek-chat is the default model
huge-atlas-agent "make an atlas for scan.obj with a 4096 texture, check the distortion and tune it if it is above 1.3"
```

Without a task it starts a dialog. Only the text of the tool results (numbers, reports) goes to the provider, never the mesh.
