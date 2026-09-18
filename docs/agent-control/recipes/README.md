# Local AI Bridge recipes

Fixed scripts any agent can run. Host-agnostic. No machine-specific paths.

How to write a new `execute` snippet: see `docs/agent-control.md` (section 「execute 脚本格式」) and `AGENTS.md`.

| File | Hand | Command |
|---|---|---|
| `identify.py` | B | `python identify.py` |
| `scene_info.py` | B | `python scene_info.py` |
| `import_pmx.py` | A or B | `python import_pmx.py --filepath model.pmx` |
| `import_vmd.py` | A or B | `python import_vmd.py --filepath motion.vmd [--armature Name]` |
| `capture.py` | B | `python capture.py --output out.png` |
| `headless_job.py` | A | `SekaiBlender.exe --background --factory-startup --python headless_job.py -- --filepath model.pmx` |

`bridge_client.py` lives in the parent directory. Recipes insert it on `sys.path`; you can also `from bridge_client import execute_result`.

Optional env: `SEKAIBLENDER_BRIDGE_HOST`, `SEKAIBLENDER_BRIDGE_PORT`, `SEKAIBLENDER_IMPORT_TIMEOUT`, `SEKAIBLENDER_CAPTURE_TIMEOUT`.

Do not add a PNG output sandbox here. The caller chooses `--output`.
