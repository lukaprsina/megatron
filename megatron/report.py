"""Inspection report builder.

Dataclasses for each task type and a ReportBuilder that renders them to a
styled PDF via markdown-pdf.  No ROS dependencies — import and call from
the controller at report time.
"""

from dataclasses import dataclass, field
from datetime import date as _date
from html import escape
from pathlib import Path

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from markdown_pdf import MarkdownPdf, Section

# ---------------------------------------------------------------------------
# Task dataclasses
# ---------------------------------------------------------------------------


@dataclass
class RingTask:
    requestor: str
    counts: dict = field(default_factory=dict)  # {"red": 2, "blue": 1, ...}

    @property
    def total(self) -> int:
        return sum(self.counts.values())


@dataclass
class BarrelTask:
    requestor: str
    # Each entry: {id, color, orientation, leaking, spill_count, pos}
    results: list = field(default_factory=list)

    @property
    def total(self) -> int:
        return len(self.results)


@dataclass
class AnomalyTask:
    requestor: str
    color: str  # "red" or "green"
    # Each entry: {tile_id, status, defect_area, defect_ratio}
    results: list = field(default_factory=list)

    @property
    def ok_count(self) -> int:
        return sum(1 for r in self.results if r.get("status") == "OK")

    @property
    def defect_count(self) -> int:
        return sum(1 for r in self.results if r.get("status") == "DEFECT")

    @property
    def total(self) -> int:
        return len(self.results)


# ---------------------------------------------------------------------------
# CSS
# ---------------------------------------------------------------------------

_CSS = """<style>
body {
    font-family: Arial, sans-serif;
    font-size: 13px;
}

h1 {
    color: #E63946;
    border-bottom: 3px solid #333;
    padding-bottom: 10px;
}

h2, h3 {
    color: #333;
}

table {
    width: 100%;
    border-collapse: collapse;
    margin: 15px 0;
}

table thead {
    font-weight: bold;
}

table th {
    background-color: #003366;
    color: white;
    padding: 12px;
    text-align: left;
    border: 1px solid #ddd;
}

table td {
    padding: 10px 12px;
    border: 1px solid #ddd;
}

table tbody tr:nth-child(odd) {
    background-color: #E8E8E8;
}

table tbody tr:nth-child(even) {
    background-color: #F5F5F5;
}

table tbody tr:hover {
    background-color: #D0D0D0;
}

.ok   { color: green; font-weight: bold; }
.nok  { color: red;   font-weight: bold; }
.leak { color: red;   font-weight: bold; }
.tile-defect {
    margin: 12px 0 18px 0;
    page-break-inside: avoid;
}
.tile-defect span {
    display: inline-block;
    min-width: 70px;
    font-weight: bold;
    vertical-align: top;
    padding-top: 36px;
}
.tile-defect img {
    width: 150px;
    margin-right: 14px;
    border: 1px solid #ccc;
    vertical-align: top;
}
</style>
"""

# Plain CSS (no <style> wrapper) for use as `user_css` on the per-tile-defect
# Sections created in save_pdf — see the comment there for why those are
# split out of the main flow.
_CSS_RAW = _CSS.removeprefix("<style>").removesuffix("</style>\n").removesuffix("</style>")

_TILE_DEFECT_MARKER = '<div class="tile-defect">'

# ---------------------------------------------------------------------------
# ReportBuilder
# ---------------------------------------------------------------------------
try:
    PACKAGE_SHARE = Path(get_package_share_directory("megatron"))
except PackageNotFoundError:
    PACKAGE_SHARE = Path(__file__).resolve().parents[1]
REPORT_PATH = PACKAGE_SHARE / "assets" / "report.pdf"
SNAPSHOT_DIR = PACKAGE_SHARE / "assets" / "snapshots"


class ReportBuilder:
    
    def __init__(self, logger):
        self.logger = logger

    def build(
        self,
        tasks: list,
        robot_name: str = "Megatron",
        report_date: str | None = None,
    ) -> str:
        """Render tasks to a markdown+HTML string ready for markdown-pdf."""
        today = report_date or _date.today().strftime("%d.%m.%Y")
        lines: list[str] = [_CSS]

        lines.append("# Inspection Report\n")
        lines.append(f"**Date:** {today}  ")
        lines.append(f"**Robot:** {robot_name}\n")
        lines.append("---\n")

        if not tasks:
            lines.append("_No tasks were assigned during this run._\n")
            return "\n".join(lines)

        for task in tasks:
            if isinstance(task, RingTask):
                lines += self._ring_section(task)
            elif isinstance(task, BarrelTask):
                lines += self._barrel_section(task)
            elif isinstance(task, AnomalyTask):
                lines += self._anomaly_section(task)

        return "\n".join(lines)

    # ── Section renderers ─────────────────────────────────────────────

    def _ring_section(self, task: RingTask) -> list[str]:
        lines = ["## Task: Ring Counting\n"]
        lines.append(f"**Requested by:** {task.requestor}  ")
        lines.append(f"**Total rings detected:** {task.total}\n")

        if task.counts:
            lines.append("**Detected colours:**\n")
            lines.append(
                "<table><thead><tr><th>Colour</th><th>Count</th></tr></thead><tbody>"
            )
            for color, count in sorted(task.counts.items()):
                lines.append(f"<tr><td>{color.capitalize()}</td><td>{count}</td></tr>")
            lines.append("</tbody></table>\n")

        lines.append("---\n")
        return lines

    def _leak_image_path(self, barrel_id) -> Path | None:
        path = SNAPSHOT_DIR / f"{barrel_id}_leak.png"
        return path if path.is_file() else None

    def _barrel_section(self, task: BarrelTask) -> list[str]:
        lines = ["## Task: Barrel Inspection\n"]
        lines.append(f"**Requested by:** {task.requestor}  ")
        lines.append(f"**Total barrels inspected:** {task.total}\n")

        if task.results:
            lines.append(
                "<table><thead><tr>"
                "<th>Barrel ID</th><th>Colour</th>"
                "<th>Orientation</th><th>Leak detected</th>"
                "</tr></thead><tbody>"
            )
            for b in task.results:
                leak_html = (
                    '<span class="leak">Yes</span>' if b.get("leaking") else "No"
                )
                lines.append(
                    f"<tr>"
                    f"<td>{b.get('id', '?')}</td>"
                    f"<td>{str(b.get('color', 'unknown')).capitalize()}</td>"
                    f"<td>{str(b.get('orientation', 'unknown')).capitalize()}</td>"
                    f"<td>{leak_html}</td>"
                    f"</tr>"
                )
            lines.append("</tbody></table>\n")

            for b in task.results:
                if not b.get("leaking"):
                    continue
                image_path = self._leak_image_path(b.get("id", "?"))
                if image_path is None:
                    self.logger.info(f"image path is None for barrel_id {b.get('id')}")
                    continue
                lines.append(f"**ID: {b.get('id', '?')}** ")
                lines.append(f'<img src="{image_path}" width="100">\n')

        lines.append("---\n")
        return lines

    def _anomaly_section(self, task: AnomalyTask) -> list[str]:
        lines = [f"## Task: Anomaly Detection ({task.color.capitalize()} belt)\n"]
        lines.append(f"**Requested by:** {task.requestor}  ")
        lines.append(f"**Tiles inspected:** {task.total}  ")
        lines.append(f"**OK:** {task.ok_count}  **DEFECT:** {task.defect_count}\n")

        if task.results:
            lines.append(
                "<table><thead><tr>"
                "<th>Tile ID</th><th>Status</th><th>Defect ratio</th>"
                "</tr></thead><tbody>"
            )
            for t in task.results:
                status = t.get("status", "UNKNOWN")
                css_class = "ok" if status == "OK" else "nok"
                ratio = t.get("defect_ratio", 0.0)
                lines.append(
                    f"<tr>"
                    f"<td>{t.get('tile_id', '?')}</td>"
                    f'<td><span class="{css_class}">{status}</span></td>'
                    f"<td>{ratio:.3f}</td>"
                    f"</tr>"
                )
            lines.append("</tbody></table>\n")

            for t in task.results:
                if t.get("status") != "DEFECT":
                    continue
                warped_image, mask_image = _tile_report_images(task, t)
                if warped_image is None or mask_image is None:
                    self.logger.info(
                        f"missing anomaly images for tile_id {t.get('tile_id', '?')}"
                    )
                    continue
                tile_id = escape(str(t.get("tile_id", "?")))
                lines.append(
                    '<div class="tile-defect">'
                    f"<span>ID: {tile_id}</span>"
                    f'<img src="{escape(str(warped_image))}">'
                    f'<img src="{escape(str(mask_image))}">'
                    "</div>\n"
                )

        lines.append("---\n")
        return lines

    # ── PDF output ────────────────────────────────────────────────────

    def save_pdf(
        self,
        tasks: list,
        robot_name: str = "Megatron",
        report_date: str | None = None,
    ) -> Path:
        """Render tasks and write a PDF. Returns the output path."""
        markdown = self.build(tasks, robot_name=robot_name, report_date=report_date)
        out = REPORT_PATH
        pdf = MarkdownPdf()
        # Each tile-defect block is added as its own Section instead of being
        # flowed into the same Section as the preceding tables. PyMuPDF's
        # Story layout (used by markdown-pdf) corrupts earlier tables —
        # redrawing them as blank header/row shapes with no cell text — when
        # a later block in the same Section forces/avoids a page break after
        # one or more tables. Splitting into separate Sections sidesteps the
        # bug entirely (confirmed via direct repro against pymupdf 1.27).
        for i, chunk in enumerate(_split_tile_defect_chunks(markdown)):
            # root="/" so absolute <img src> paths (snapshot files) resolve —
            # PyMuPDF's Archive treats Section's default root "." as the only
            # base for resolving src paths and otherwise drops the image silently.
            user_css = None if i == 0 else _CSS_RAW
            pdf.add_section(Section(chunk, root="/"), user_css=user_css)
        pdf.save(str(out))
        return out


def _split_tile_defect_chunks(markdown: str) -> list[str]:
    """Split rendered markdown into one chunk per tile-defect block.

    The first chunk holds everything before the first tile-defect div
    (the CSS, task summaries, and tables); each subsequent chunk holds
    exactly one tile-defect div.
    """
    parts = markdown.split(_TILE_DEFECT_MARKER)
    return [parts[0]] + [_TILE_DEFECT_MARKER + part for part in parts[1:]]


def _existing_image_path(value) -> Path | None:
    if not value:
        return None
    path = Path(str(value)).expanduser()
    return path if path.is_file() else None


def _tile_report_images(task: AnomalyTask, tile: dict) -> tuple[Path | None, Path | None]:
    warped_image = _existing_image_path(
        tile.get("warped_image") or tile.get("artifact_warped_image")
    )
    mask_image = _existing_image_path(tile.get("mask_image") or tile.get("artifact_mask_image"))
    if warped_image is not None and mask_image is not None:
        return warped_image, mask_image
    return _find_capture_images(task, tile)


def _find_capture_images(task: AnomalyTask, tile: dict) -> tuple[Path | None, Path | None]:
    tile_id = tile.get("tile_id")
    if tile_id is None:
        return None, None
    station = str(tile.get("station") or task.color)
    world = str(tile.get("world") or "task2")
    try:
        tile_number = int(tile_id)
    except (TypeError, ValueError):
        return None, None

    for root in _candidate_capture_roots():
        captures_dir = root / world / station / "captures"
        if not captures_dir.is_dir():
            continue
        metadata_paths = sorted(
            captures_dir.glob(f"tile-{tile_number:02d}-*.yaml"),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )
        for metadata_path in metadata_paths:
            pair = _images_from_capture_metadata(metadata_path)
            if pair != (None, None):
                return pair
    return None, None


def _images_from_capture_metadata(metadata_path: Path) -> tuple[Path | None, Path | None]:
    station_dir = metadata_path.parent.parent
    try:
        metadata = yaml.safe_load(metadata_path.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return None, None
    anomaly = metadata.get("anomaly", {}) or {}
    warped_image = _existing_image_path(anomaly.get("megatron_report_warped_image"))
    mask_image = _existing_image_path(anomaly.get("megatron_report_mask_image"))
    if warped_image is not None and mask_image is not None:
        return warped_image, mask_image

    warped_image = _existing_image_path(station_dir / str(metadata.get("canonical_image", "")))
    mask_image = _existing_image_path(
        station_dir
        / str(anomaly.get("report_mask_image") or anomaly.get("mask_image") or "")
    )
    return warped_image, mask_image


def _candidate_capture_roots() -> list[Path]:
    roots = [
        Path.cwd() / "artifacts" / "tile_captures",
        Path("/tmp/megatron_tile_captures"),
    ]
    roots.extend(parent / "artifacts" / "tile_captures" for parent in PACKAGE_SHARE.parents)
    unique: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        root = root.expanduser()
        if root in seen:
            continue
        seen.add(root)
        unique.append(root)
    return unique
