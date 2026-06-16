"""Inspection report builder.

Dataclasses for each task type and a ReportBuilder that renders them to a
styled PDF via markdown-pdf.  No ROS dependencies — import and call from
the controller at report time.
"""

from dataclasses import dataclass, field
from datetime import date as _date
from pathlib import Path

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
    def nok_count(self) -> int:
        return sum(1 for r in self.results if r.get("status") == "NOK")

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
    background-color: #003366;
    color: #fff;
    font-weight: bold;
}

table th {
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
</style>
"""

# ---------------------------------------------------------------------------
# ReportBuilder
# ---------------------------------------------------------------------------


class ReportBuilder:
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

        lines.append("---\n")
        return lines

    def _anomaly_section(self, task: AnomalyTask) -> list[str]:
        lines = [f"## Task: Anomaly Detection ({task.color.capitalize()} belt)\n"]
        lines.append(f"**Requested by:** {task.requestor}  ")
        lines.append(f"**Tiles inspected:** {task.total}  ")
        lines.append(f"**OK:** {task.ok_count}  **NOK:** {task.nok_count}\n")

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

        lines.append("---\n")
        return lines

    # ── PDF output ────────────────────────────────────────────────────

    def save_pdf(
        self,
        tasks: list,
        path: str | Path = "/tmp/megatron_report.pdf",
        robot_name: str = "Megatron",
        report_date: str | None = None,
    ) -> Path:
        """Render tasks and write a PDF. Returns the output path."""
        markdown = self.build(tasks, robot_name=robot_name, report_date=report_date)
        out = Path(path)
        pdf = MarkdownPdf()
        pdf.add_section(Section(markdown))
        pdf.save(str(out))
        return out
