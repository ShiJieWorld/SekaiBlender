# SPDX-FileCopyrightText: 2026 SekaiBlender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Splash-screen sponsor list for SekaiBlender.

Names are compiled into the build. Replace placeholder entries with the real
roster; paging / sort live in the splash UI and do not require a rebuild of
this module unless the roster itself changes.

Sponsors are not ranked by named tiers. Sort keys are join time and amount.
"""

from __future__ import annotations

# Two explicit support channels. Do not collapse them into a single CTA.
XIANYU_URL = "https://m.tb.cn/h.8GOq76z?tk=TEo3TkIZ4Ul"
AFDIAN_URL = "https://afdian.com/a/Sekaiblender"
GITHUB_URL = "https://github.com/ShiJieWorld/SekaiBlender"
BILIBILI_URL = "https://b23.tv/59qbW2a"
# Kept for older call sites that still import the generic name.
CONTACT_URL = GITHUB_URL
# Kept for older call sites that still import the generic name.
SPONSOR_URL = AFDIAN_URL
# Human-readable roster refresh stamp shown on the splash. Update when names land.
ROSTER_UPDATED = "2026年9月"

# Placeholder roster so paging and sort can be judged before real names land.
# `joined` is ISO date; `amount` is an integer (CNY) used only for ordering.
SPONSORS = (
    {"name": "星野", "joined": "2026-09-18", "amount": 500},
    {"name": "Aster", "joined": "2026-09-17", "amount": 300},
    {"name": "月见", "joined": "2026-09-16", "amount": 200},
    {"name": "夜航", "joined": "2026-09-15", "amount": 150},
    {"name": "Mizuki", "joined": "2026-09-14", "amount": 120},
    {"name": "白羽", "joined": "2026-09-13", "amount": 88},
    {"name": "Lumi", "joined": "2026-09-12", "amount": 66},
    {"name": "镜川", "joined": "2026-09-11", "amount": 50},
    {"name": "Kuro", "joined": "2026-09-10", "amount": 50},
    {"name": "浮光", "joined": "2026-09-09", "amount": 30},
    {"name": "Nia", "joined": "2026-09-08", "amount": 30},
    {"name": "青时", "joined": "2026-09-07", "amount": 28},
    {"name": "遥", "joined": "2026-09-06", "amount": 20},
    {"name": "Ciel", "joined": "2026-09-05", "amount": 20},
    {"name": "风铃", "joined": "2026-09-04", "amount": 18},
    {"name": "Momo", "joined": "2026-09-03", "amount": 15},
    {"name": "千夏", "joined": "2026-09-02", "amount": 15},
    {"name": "Rin", "joined": "2026-09-01", "amount": 12},
    {"name": "薄暮", "joined": "2026-08-30", "amount": 12},
    {"name": "Noa", "joined": "2026-08-28", "amount": 10},
    {"name": "羽音", "joined": "2026-08-26", "amount": 10},
    {"name": "Sora", "joined": "2026-08-24", "amount": 10},
    {"name": "冬青", "joined": "2026-08-22", "amount": 8},
    {"name": "Ena", "joined": "2026-08-20", "amount": 8},
    {"name": "浅川", "joined": "2026-08-18", "amount": 6},
    {"name": "Hana", "joined": "2026-08-16", "amount": 6},
    {"name": "灯里", "joined": "2026-08-14", "amount": 5},
    {"name": "Yuki", "joined": "2026-08-12", "amount": 5},
    {"name": "铃兰", "joined": "2026-08-10", "amount": 5},
    {"name": "Aoi", "joined": "2026-08-08", "amount": 5},
    {"name": "雪绪", "joined": "2026-08-06", "amount": 5},
    {"name": "Ren", "joined": "2026-08-04", "amount": 5},
    {"name": "春野", "joined": "2026-08-02", "amount": 5},
    {"name": "Mio", "joined": "2026-07-30", "amount": 5},
    {"name": "橘", "joined": "2026-07-28", "amount": 5},
    {"name": "Kai", "joined": "2026-07-26", "amount": 5},
)

PAGE_SIZE = 12

SORT_ITEMS = (
    ("TIME", "按时间", "最近加入的在前"),
    ("AMOUNT", "按金额", "金额高的在前"),
)


def sorted_sponsors(sort_mode: str, descending: bool = True):
    sponsors = list(SPONSORS)
    if sort_mode == "AMOUNT":
        sponsors.sort(key=lambda item: (int(item.get("amount", 0)), item["name"].casefold()))
    else:
        sponsors.sort(key=lambda item: (item.get("joined", ""), item["name"].casefold()))
    if descending:
        sponsors.reverse()
    return sponsors


def filtered_sponsors(sort_mode: str, query: str = "", descending: bool = True):
    sponsors = sorted_sponsors(sort_mode, descending)
    needle = (query or "").strip().casefold()
    if not needle:
        return sponsors
    return [item for item in sponsors if needle in item["name"].casefold()]


def page_count(total: int, page_size: int = PAGE_SIZE) -> int:
    if total <= 0:
        return 1
    return max(1, (total + page_size - 1) // page_size)


def page_slice(sort_mode: str, page: int, page_size: int = PAGE_SIZE, query: str = "", descending: bool = True):
    sponsors = filtered_sponsors(sort_mode, query, descending)
    pages = page_count(len(sponsors), page_size)
    page = max(0, min(int(page), pages - 1))
    start = page * page_size
    items = list(sponsors[start:start + page_size])
    # Always return a full page so the grid and pager stay a fixed height.
    if len(items) < page_size:
        items.extend([None] * (page_size - len(items)))
    return items, page, pages, len(sponsors)
