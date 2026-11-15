# SPDX-FileCopyrightText: 2026 SekaiBlender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Splash-screen sponsor list for SekaiBlender.

Names are compiled into the build. This is the real sponsor roster; paging /
sort live in the splash UI and do not require a rebuild of this module unless
the roster itself changes.

Sponsors are not ranked by named tiers. Sort keys are join time and amount.
"""

from __future__ import annotations

# Two explicit support channels. Do not collapse them into a single CTA.
# Stable Xianyu personal-home short link decoded from the profile QR.
# Do not replace with m.tb.cn/h...?tk= share links; those expire.
XIANYU_URL = "https://p.goofish.com/p/8aOe0QKT"
AFDIAN_URL = "https://afdian.com/a/Sekaiblender"
GITHUB_URL = "https://github.com/ShiJieWorld/SekaiBlender"
BILIBILI_URL = "https://b23.tv/59qbW2a"
# Kept for older call sites that still import the generic name.
CONTACT_URL = GITHUB_URL
# Kept for older call sites that still import the generic name.
SPONSOR_URL = AFDIAN_URL
# Human-readable roster refresh stamp shown on the splash. Update when names land.
ROSTER_UPDATED = "2026年11月"

# Real sponsor roster. `joined` is ISO date; `amount` is an integer (CNY)
# used only for ordering.
SPONSORS = (
    {"name": "白厄逼水我兑奶喝三年", "joined": "2026-11-15", "amount": 40},
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
