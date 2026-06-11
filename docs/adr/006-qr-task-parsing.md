# 006 — QR task parsing (substring matching, not token equality)

**Decision**: The QR codes in the task2 world contain natural-language sentences (e.g.
`"Find all defects on the red belt."`), not single tokens. `_parse_qr_task()` uses
case-insensitive substring matching with keyword priorities, not equality comparison.

**Rationale**: Prior plans assumed `/qr_task` would contain token values like
`"defects green"`. Actual QR content is full sentences. Substring matching with
fallthrough priority handles all known QR texts without requiring exact string
normalisation.

**Known QR texts and their parsed tokens**:
| QR file | Content | Parsed token |
|---|---|---|
| `qr_redbelt.png` | "Find all defects on the red belt." | `defects_red` |
| `qr_greenbelt.png` | "Find all defects on the green belt." | `defects_green` |
| `qr_barrels.png` | "Inspect the barrels..." | `barrels` |
| `qr_rings.png` | "Find all the rings." | `rings` |
| `qr_nothing.png` | "I'm just a visitor :)" | `nothing` |
| `qr_cto.png` | "Hello there! Thanks for the report." | `report` |
| `qr_incinerator.png` | "Emergency Intelligence Incinerator: System testing complete. Fall down the Glados oven." | `emergency` |

**Matching priority**: `report`/`thanks` → red belt + defect → green belt + defect →
barrel → ring → visitor. The `"fault"` keyword is additionally checked for red/green
defect tasks as a fallback.

**Code**: `src/megatron/megatron/task2_controller.py:_parse_qr_task` (line 154)
