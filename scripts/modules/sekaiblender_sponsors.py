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

# Purple interaction accent used by the SekaiBlender theme preset.
BRAND_ACCENT = (0.45, 0.39, 0.62, 1.0)
BRAND_ACCENT_LIGHT = (0.60, 0.53, 0.80, 1.0)
BRAND_WIDGETS = (
    "wcol_regular",
    "wcol_tool",
    "wcol_toolbar_item",
    "wcol_radio",
    "wcol_option",
    "wcol_toggle",
    "wcol_num",
    "wcol_numslider",
    "wcol_menu",
    "wcol_menu_back",
    "wcol_menu_item",
    "wcol_tooltip",
    "wcol_progress",
    "wcol_list_item",
)


def apply_default_theme(theme) -> None:
    """Apply the SekaiBlender accent to an untouched factory-default theme."""
    ui = theme.user_interface
    for widget_name in BRAND_WIDGETS:
        getattr(ui, widget_name).inner_sel = BRAND_ACCENT
    ui.wcol_num.item = BRAND_ACCENT_LIGHT
    ui.wcol_numslider.item = BRAND_ACCENT_LIGHT
    ui.wcol_progress.item = BRAND_ACCENT_LIGHT
    ui.panel_active = BRAND_ACCENT
    theme.common.anim.playhead = BRAND_ACCENT_LIGHT
    theme.name = "SekaiBlender"


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
