"""打印综合报告中的时钟 Fmax 与最差路径（起点 -> 终点）。"""
import html
import re
import sys
from pathlib import Path

rpt = Path(__file__).resolve().parent / "build/impl/gwsynthesis/nav_subsystem_syn.rpt.html"
t = html.unescape(re.sub(r"<[^>]+>", " | ", rpt.read_text(encoding="utf-8", errors="replace")))
t = re.sub(r"(\s*\|\s*)+", " | ", t)
i = t.find("Max Frequency Summary")
print(t[i:t.find("Detail Timing", i)])
seen = set()
for p in t.split("Path Summary:")[1:]:
    sl = re.search(r"Slack \| (-?[\d.]+)", p)
    fr = re.search(r"From \| (\S+)", p)
    to = re.search(r"To \| (\S+)", p)
    key = (fr.group(1).rsplit("_", 2)[0], to.group(1).rsplit("_", 2)[0])
    if key not in seen:
        seen.add(key)
        print(sl.group(1), fr.group(1), "->", to.group(1))
