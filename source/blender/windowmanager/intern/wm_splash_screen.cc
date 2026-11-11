/* SPDX-FileCopyrightText: 2007 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup wm
 *
 * This file contains the splash screen logic (the `WM_OT_splash` operator).
 *
 * - Loads the splash image.
 * - Displaying version information.
 * - Two-column layout: splash art + recent/new files on the left,
 *   sponsor list (paging/sort) on the right.
 * - Links to the SekaiBlender sponsor page.
 */

#include <algorithm>
#include <cstring>

#include "DNA_screen_types.h"
#include "DNA_userdef_types.h"
#include "DNA_windowmanager_types.h"

#include "BLI_listbase.hh"
#include "BLI_math_base_c.hh"
#include "BLI_path_utils.hh"
#include "BLI_utildefines.hh"

#include "BKE_appdir.hh"
#include "BKE_blender_project.hh"
#include "BKE_blender_version.h"
#include "BKE_context.hh"
#include "BKE_global.hh"
#include "BKE_preferences.h"

#include "BLT_translation.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "ED_datafiles.h"
#include "ED_screen.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "UI_interface.hh"
#include "UI_interface_icons.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "wm.hh"

namespace blender {

/* -------------------------------------------------------------------- */
/** \name Splash Screen
 * \{ */

static void wm_block_splash_close(bContext *C, ui::Block *block)
{
  wmWindow *win = CTX_wm_window(C);
  popup_block_close(C, win, block);
}



#ifndef WITH_HEADLESS
static void wm_block_splash_image_roundcorners_add(ImBuf *ibuf, const bool round_right)
{
  uchar *rct = ibuf->byte_data_for_write();
  if (!rct) {
    return;
  }

  const bTheme *btheme = ui::theme::theme_get();
  const float roundness = btheme->tui.wcol_menu_back.roundness * UI_SCALE_FAC;
  const int size = roundness * 20;

  if (size < ibuf->x && size < ibuf->y) {
    /* Y-axis initial offset. */
    rct += 4 * (ibuf->y - size) * ibuf->x;

    for (int y = 0; y < size; y++) {
      for (int x = 0; x < size; x++, rct += 4) {
        const float pixel = 1.0 / size;
        const float u = pixel * x;
        const float v = pixel * y;
        const float distance = sqrt(u * u + v * v);

        /* Pointer offset to the alpha value of pixel. */
        /* NOTE: the left corner is flipped in the X-axis. */
        const int offset_l = 4 * (size - x - x - 1) + 3;
        const int offset_r = 4 * (ibuf->x - size) + 3;

        if (distance > 1.0) {
          rct[offset_l] = 0;
          if (round_right) {
            rct[offset_r] = 0;
          }
        }
        else {
          /* Create a single pixel wide transition for anti-aliasing.
           * Invert the distance and map its range [0, 1] to [0, pixel]. */
          const float fac = (1.0 - distance) * size;

          if (fac > 1.0) {
            continue;
          }

          const uchar alpha = unit_float_to_uchar_clamp(fac);
          rct[offset_l] = alpha;
          if (round_right) {
            rct[offset_r] = alpha;
          }
        }
      }

      /* X-axis offset to the next row. */
      rct += 4 * (ibuf->x - size);
    }
  }
}

/* Composite the Blender logo over a project splash image. */
static void wm_project_splash_logo_add(ImBuf *ibuf)
{
  /* Magic numbers match logo positioning and size in default splash image. */
  ImBuf *logo = ui::svg_icon_bitmap(ICON_BLENDER_LOGO_LARGE, ibuf->x * 0.127f, false);
  if (logo == nullptr) {
    return;
  }

  /* The monochrome logo has alpha in all channels, unpremultiply to get white with alpha. */
  IMB_unpremultiply_alpha(logo);

  /* Rows are stored bottom to top. */
  const int x = ibuf->x * 0.025f;
  const int y = ibuf->y - int(ibuf->x * 0.02f) - logo->y;
  IMB_rectblend(ibuf,
                ibuf->byte_data_for_write(),
                nullptr,
                ibuf,
                logo,
                nullptr,
                nullptr,
                nullptr,
                0.0f,
                x,
                y,
                x,
                y,
                0,
                0,
                logo->x,
                logo->y,
                IMB_BLEND_MIX,
                false);

  IMB_freeImBuf(logo);
}
#endif /* !WITH_HEADLESS */

std::string WM_project_splash_image_filepath(const Main *bmain)
{
  return BKE_blender_project_read_callback(
      bmain, [](const bke::BlenderProject *project) -> std::string {
        if (project == nullptr || project->get_splash_image_path().is_empty()) {
          return "";
        }
        return BKE_blender_project_path_resolve(project->get_splash_image_path(), *project);
      });
}

ImBuf *WM_project_splash_image_load(const StringRefNull filepath,
                                    const int width,
                                    const bool add_logo)
{
  if (filepath.is_empty() || width <= 0) {
    return nullptr;
  }

  ImBuf *ibuf = IMB_load_image_from_filepath(filepath.c_str(), ImBufFlags::ByteData);
  if (ibuf == nullptr) {
    return nullptr;
  }

  /* Convert float to byte image buffer. */
  if (ibuf->byte_data() == nullptr) {
    IMB_byte_from_float(ibuf);
    IMB_free_float_pixels(ibuf);
  }
  if (ibuf->byte_data() == nullptr || ibuf->x <= 0 || ibuf->y <= 0) {
    IMB_freeImBuf(ibuf);
    return nullptr;
  }

  /* Add an alpha channel if missing. */
  ibuf->color_mode = ImColorMode::RGBA;
  ibuf->channels = 4;

  /* Crop to 2:1 around the center. */
  const int2 size(ibuf->x, ibuf->y);
  const int2 crop_size(std::min(size.x, size.y * 2), std::max(std::min(size.y, size.x / 2), 1));
  if (crop_size != size) {
    IMB_crop(ibuf, (size - crop_size) / 2, crop_size);
  }

  /* Scale to fit specified width. */
  const int height = std::max(width / 2, 1);
  if (width != ibuf->x || height != ibuf->y) {
    IMB_scale(ibuf, width, height, IMBScaleFilter::Box, false);
  }

#ifndef WITH_HEADLESS
  if (add_logo) {
    wm_project_splash_logo_add(ibuf);
  }
#else
  UNUSED_VARS(add_logo);
#endif

  return ibuf;
}

static ImBuf *wm_block_splash_image(const Main *bmain,
                                    int width,
                                    int *r_height,
                                    const bool round_right)
{
  ImBuf *ibuf = nullptr;
  int height = 0;
#ifndef WITH_HEADLESS
  ibuf = WM_project_splash_image_load(WM_project_splash_image_filepath(bmain), width, true);

  if (ibuf == nullptr && U.app_template[0] != '\0') {
    char splash_filepath[FILE_MAX];
    char template_directory[FILE_MAX];
    if (BKE_appdir_app_template_id_search(
            U.app_template, template_directory, sizeof(template_directory)))
    {
      BLI_path_join(splash_filepath, sizeof(splash_filepath), template_directory, "splash.png");
      ibuf = IMB_load_image_from_filepath(splash_filepath, ImBufFlags::ByteData);
    }
  }

  if (ibuf == nullptr) {
    const char *custom_splash_path = BLI_getenv("BLENDER_CUSTOM_SPLASH");
    if (custom_splash_path) {
      ibuf = IMB_load_image_from_filepath(custom_splash_path, ImBufFlags::ByteData);
    }
  }

  if (ibuf == nullptr) {
    const uchar *splash_data = reinterpret_cast<const uchar *>(datatoc_splash_png);
    size_t splash_data_size = datatoc_splash_png_size;
    ibuf = IMB_load_image_from_memory(
        splash_data, splash_data_size, ImBufFlags::ByteData, "<splash screen>");
  }

  if (ibuf) {
    ibuf->color_mode = ImColorMode::RGBA; /* The image might not have an alpha channel. */
    height = (width * ibuf->y) / ibuf->x;
    if (width != ibuf->x || height != ibuf->y) {
      IMB_scale(ibuf, width, height, IMBScaleFilter::Box, false);
    }

    wm_block_splash_image_roundcorners_add(ibuf, round_right);
    IMB_premultiply_alpha(ibuf);
  }

#else
  UNUSED_VARS(width);
#endif
  *r_height = height;
  return ibuf;
}

static ImBuf *wm_block_splash_banner_image(int *r_width,
                                           int *r_height,
                                           int max_width,
                                           int max_height)
{
  ImBuf *ibuf = nullptr;
  int height = 0;
  int width = max_width;
#ifndef WITH_HEADLESS

  const char *custom_splash_path = BLI_getenv("BLENDER_CUSTOM_SPLASH_BANNER");
  if (custom_splash_path) {
    ibuf = IMB_load_image_from_filepath(custom_splash_path, ImBufFlags::ByteData);
  }

  if (!ibuf) {
    return nullptr;
  }

  ibuf->color_mode = ImColorMode::RGBA; /* The image might not have an alpha channel. */

  width = ibuf->x;
  height = ibuf->y;
  if (width > 0 && height > 0 && (width > max_width || height > max_height)) {
    const float splash_ratio = max_width / float(max_height);
    const float banner_ratio = ibuf->x / float(ibuf->y);

    if (banner_ratio > splash_ratio) {
      /* The banner is wider than the splash image. */
      width = max_width;
      height = max_width / banner_ratio;
    }
    else if (banner_ratio < splash_ratio) {
      /* The banner is taller than the splash image. */
      height = max_height;
      width = max_height * banner_ratio;
    }
    else {
      width = max_width;
      height = max_height;
    }
    if (width != ibuf->x || height != ibuf->y) {
      IMB_scale(ibuf, width, height, IMBScaleFilter::Box, false);
    }
  }

  IMB_premultiply_alpha(ibuf);

#else
  UNUSED_VARS(max_height);
#endif
  *r_height = height;
  *r_width = width;
  return ibuf;
}

/**
 * Close the splash when opening a file-selector.
 */
static void wm_block_splash_close_on_fileselect(bContext *C, void *arg1, void * /*arg2*/)
{
  wmWindow *win = CTX_wm_window(C);
  if (!win) {
    return;
  }

  /* Check for the event as this will run before the new window/area has been created. */
  bool has_fileselect = false;
  for (const wmEvent &event : win->runtime->event_queue) {
    if (event.type == EVT_FILESELECT) {
      has_fileselect = true;
      break;
    }
  }

  if (has_fileselect) {
    wm_block_splash_close(C, static_cast<ui::Block *>(arg1));
  }
}

#if defined(__APPLE__)
/* Check if Blender is running under Rosetta for the purpose of displaying a splash screen warning.
 * From Apple's WWDC 2020 Session - Explore the new system architecture of Apple Silicon Macs.
 * Time code: 14:31 - https://developer.apple.com/videos/play/wwdc2020/10686/ */

#  include <sys/sysctl.h>

static int is_using_macos_rosetta()
{
  int ret = 0;
  size_t size = sizeof(ret);

  if (sysctlbyname("sysctl.proc_translated", &ret, &size, nullptr, 0) != -1) {
    return ret;
  }
  /* If "sysctl.proc_translated" is not present then must be native. */
  if (errno == ENOENT) {
    return 0;
  }
  return -1;
}
#endif /* __APPLE__ */

static ui::Block *wm_block_splash_create(bContext *C, ARegion *region, void * /*arg*/)
{
  const uiStyle *style = ui::style_get_dpi();

  ui::Block *block = block_begin(C, region, "splash", ui::EmbossType::Emboss);

  /* Note on #BLOCK_NO_WIN_CLIP, the window size is not always synchronized
   * with the OS when the splash shows, window clipping in this case gives
   * ugly results and clipping the splash isn't useful anyway, just disable it #32938. */
  block_flag_enable(block, ui::BLOCK_LOOP | ui::BLOCK_KEEP_OPEN | ui::BLOCK_NO_WIN_CLIP);
  block_theme_style_set(block, ui::BLOCK_THEME_STYLE_POPUP);

  const bool is_quick_setup = !bke::preferences::exists();

  /* Wide enough for art + files on the left and a sponsor column on the right.
   * Keep a window clamp so 1366-wide displays still fit. */
  int splash_width = int(style->widget.points * 72 * UI_SCALE_FAC);
  CLAMP_MAX(splash_width, WM_window_native_pixel_x(CTX_wm_window(C)) * 0.8f);

  const float split_factor = is_quick_setup ? 1.0f : 0.62f;
  const int image_width = std::max(1, int(splash_width * split_factor));
  int splash_height = 0;

  /* Would be nice to support caching this, so it only has to be re-read (and likely resized) on
   * first draw or if the image changed. */
  ImBuf *ibuf = wm_block_splash_image(
      CTX_data_main(C), image_width, &splash_height, is_quick_setup);
  /* This should never happen, if it does - don't crash. */
  if (ibuf) [[likely]] {
    ui::Button *but = uiDefButImage(
        block, ibuf, 0, 0.5f * U.widget_unit, image_width, splash_height, nullptr);

    button_func_set(but, [block](bContext &C) { wm_block_splash_close(&C, block); });
    /* Version is already in the splash art; don't overlay a second "SekaiBlender 1.0.1". */
  }

  /* Banner image passed through the environment, to overlay on the splash and
   * indicate a custom Blender version. Transparency can be used. To replace the
   * full splash screen, see BLENDER_CUSTOM_SPLASH. */
  int banner_width = 0;
  int banner_height = 0;
  ImBuf *bannerbuf = wm_block_splash_banner_image(
      &banner_width, &banner_height, image_width, splash_height);
  if (bannerbuf) {
    ui::Button *banner_but = uiDefButImage(
        block, bannerbuf, 0, 0.5f * U.widget_unit, banner_width, banner_height, nullptr);

    button_func_set(banner_but, [block](bContext &C) { wm_block_splash_close(&C, block); });
  }

  const int layout_margin_x = UI_SCALE_FAC * 18;
  /* Dual `block_layout` roots are unsafe here: `block_layout_resolve` frees
   * each root while iterating, so the sponsor column never makes it into the
   * popup bounds. One full-width layout + a split keeps both panes. */
  const int image_top = splash_height + int(0.5f * U.widget_unit);
  const int layout_y = is_quick_setup ? 0 : image_top;

  /* Floating project menu over the bottom right of the splash image, if a project is active or
   * there are recent projects. */
  const std::optional<std::string> project_name = BKE_blender_project_read_callback(
      CTX_data_main(C), [](const bke::BlenderProject *project) -> std::optional<std::string> {
        if (project == nullptr) {
          return std::nullopt;
        }
        return project->get_name();
      });
  if (project_name || !BKE_blender_project_recent_get().is_empty()) {
    const int project_margin = UI_UNIT_X / 2;
    const int project_width = UI_UNIT_X * 8;
    ui::Layout &project_layout = ui::block_layout(block,
                                                  ui::LayoutDirection::Vertical,
                                                  ui::LayoutType::Panel,
                                                  splash_width - project_width - project_margin,
                                                  0.5f * U.widget_unit + project_margin +
                                                      UI_UNIT_Y,
                                                  project_width,
                                                  0,
                                                  0,
                                                  style);
    project_layout.menu("WM_MT_splash_project",
                        project_name ? StringRef(*project_name) : IFACE_("Project"),
                        ICON_PROJECT);
  }
  ui::Layout &layout = ui::block_layout(block,
                                        ui::LayoutDirection::Vertical,
                                        ui::LayoutType::Panel,
                                        layout_margin_x,
                                        layout_y,
                                        splash_width - (layout_margin_x * 2),
                                        UI_SCALE_FAC * 110,
                                        0,
                                        style);
  /* `Block` is incomplete in windowmanager; a 1px full-width label forces
   * popup bounds to keep the sponsor column instead of shrinking to the art. */
  uiDefBut(block,
           ui::ButtonType::Label,
           "",
           0,
           0,
           short(splash_width),
           1,
           nullptr,
           0,
           0,
           std::nullopt);

  MenuType *mt;
  ui::Layout *warning_layout = &layout;

  /* Draw setup screen if no preferences have been saved yet. */
  if (is_quick_setup) {
    mt = WM_menutype_find("WM_MT_splash_quick_setup", true);

    /* The #BLOCK_QUICK_SETUP flag prevents the button text from being left-aligned,
     * as it is for all menus due to the #BLOCK_LOOP flag, see in #ui_def_but. */
    block_flag_enable(block, ui::BLOCK_QUICK_SETUP);
    if (mt) {
      ui::menutype_draw(C, mt, &layout);
    }
  }
  else {
    /* Full-height columns: the left owns art + projects; the right is wholly
     * reserved for sponsor credits from top to bottom. */
    ui::Layout &split = layout.split(0.60f, false);
    ui::Layout &left = split.column(false);
    ui::Layout &sponsor_outer = split.column(false);
    /* Give the sponsor column a deliberate breathing gutter instead of
     * letting it touch the artwork/recent-file boundary. */
    ui::Layout &sponsor_split = sponsor_outer.split(0.08f, false);
    ui::Layout &sponsor_gap = sponsor_split.column(false);
    ui::Layout &sponsors = sponsor_split.column(false);
    (void)sponsor_gap;

    if (splash_height > 0) {
      const float spacer_factor = float(splash_height) / (6.0f * UI_SCALE_FAC);
      left.separator(spacer_factor, ui::LayoutSeparatorType::Space);
    }

    /* Left pane is the height source of truth (image + files + language +
     * contact). Stretch only the sponsor cells so the pager meets the left
     * bottom. Do not stretch recent files from the right pane: that feedback
     * loop made the popup grow wildly. */
    wmWindowManager *wm = CTX_wm_manager(C);
    if (wm != nullptr) {
      const int file_rows = 6;
      const float recent_scale = 1.25f;
      const int left_extra = 8;   /* header, more, seps, actions, language, contact */
      const int right_chrome = 7; /* title, heading, cta, sort, search, stamp, pager */
      const int grid_rows = 6;
      const float left_below = float(file_rows) * recent_scale + float(left_extra);
      const float leftover = std::max(
          1.0f, float(splash_height) + (left_below - float(right_chrome)) * UI_UNIT_Y);
      const float natural = float(grid_rows) * (UI_UNIT_Y + 2.0f * style->boxspace);
      const float cell_scale = std::clamp(leftover / std::max(1.0f, natural), 0.95f, 1.55f);

      PointerRNA wm_ptr = RNA_pointer_create_discrete(&wm->id, RNA_WindowManager, wm);
      if (PropertyRNA *prop = RNA_struct_find_property(&wm_ptr, "splash_sponsor_cell_scale")) {
        RNA_property_float_set(&wm_ptr, prop, cell_scale);
      }
      if (PropertyRNA *prop = RNA_struct_find_property(&wm_ptr, "splash_recent_rows")) {
        RNA_property_int_set(&wm_ptr, prop, file_rows);
      }
      if (PropertyRNA *prop = RNA_struct_find_property(&wm_ptr, "splash_recent_scale")) {
        RNA_property_float_set(&wm_ptr, prop, recent_scale);
      }
    }

    mt = WM_menutype_find("WM_MT_splash", true);
    if (mt) {
      ui::menutype_draw(C, mt, &left);
    }
    MenuType *mt_sponsors = WM_menutype_find("WM_MT_splash_sponsors", true);
    if (mt_sponsors) {
      ui::menutype_draw(C, mt_sponsors, &sponsors);
    }
    warning_layout = &left;
  }

  block_func_set(block, wm_block_splash_close_on_fileselect, block, nullptr);

/* Displays a warning if blender is being emulated via Rosetta (macOS) or XTA (Windows) */
#if defined(__APPLE__) || defined(_M_X64)
#  if defined(__APPLE__)
  if (warning_layout && is_using_macos_rosetta() > 0)
#  elif defined(_M_X64)
  const char *proc_id = BLI_getenv("PROCESSOR_IDENTIFIER");
  if (warning_layout && proc_id && strncmp(proc_id, "ARM", 3) == 0)
#  endif
  {
    ui::Layout &layout = *warning_layout;
    layout.separator(2.0f, ui::LayoutSeparatorType::Line);

    ui::Layout &split = layout.split(0.725, true);
    ui::Layout &row1 = split.row(true);
    ui::Layout &row2 = split.row(true);

    row1.label(RPT_("Intel binary detected. Expect reduced performance."),
               ICON_STATUS_WARNING_FILLED);

    PointerRNA op_ptr = row2.op("WM_OT_url_open",
                                CTX_IFACE_(BLT_I18NCONTEXT_OPERATOR_DEFAULT, "Learn More"),
                                ICON_URL,
                                wm::OpCallContext::InvokeDefault,
                                UI_ITEM_NONE);
#  if defined(__APPLE__)
    RNA_string_set(
        &op_ptr,
        "url",
        "https://docs.blender.org/manual/en/latest/getting_started/installing/macos.html");
#  elif defined(_M_X64)
    RNA_string_set(
        &op_ptr,
        "url",
        "https://docs.blender.org/manual/en/latest/getting_started/installing/windows.html");
#  endif

    layout.separator();
  }
#endif

  block_bounds_set_centered(block, 0);

  return block;
}

static wmOperatorStatus wm_splash_invoke(bContext *C,
                                         wmOperator * /*op*/,
                                         const wmEvent * /*event*/)
{
  ui::popup_block_invoke(C, wm_block_splash_create, nullptr, nullptr);

  return OPERATOR_FINISHED;
}

void WM_OT_splash(wmOperatorType *ot)
{
  ot->name = "Splash Screen";
  ot->idname = "WM_OT_splash";
  ot->description = "Open the splash screen with release info";

  ot->invoke = wm_splash_invoke;
  ot->poll = WM_operator_winactive;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Splash Screen: About
 * \{ */

static ui::Block *wm_block_about_create(bContext *C, ARegion *region, void * /*arg*/)
{
  const uiStyle *style = ui::style_get_dpi();
  const int dialog_width = style->widget.points * 42 * UI_SCALE_FAC;

  ui::Block *block = block_begin(C, region, "about", ui::EmbossType::Emboss);

  block_flag_enable(block, ui::BLOCK_KEEP_OPEN | ui::BLOCK_LOOP | ui::BLOCK_NO_WIN_CLIP);
  block_theme_style_set(block, ui::BLOCK_THEME_STYLE_POPUP);

  ui::Layout &layout = ui::block_layout(block,
                                        ui::LayoutDirection::Vertical,
                                        ui::LayoutType::Panel,
                                        0,
                                        0,
                                        dialog_width,
                                        0,
                                        0,
                                        style);

/* Blender logo. */
#ifndef WITH_HEADLESS
  constexpr bool show_color = false;
  const float size = 0.2f * dialog_width;

  ImBuf *ibuf = ui::svg_icon_bitmap(ICON_BLENDER_LOGO_LARGE, size, show_color);

  if (ibuf) {
    const bTheme *btheme = ui::theme::theme_get();
    const uchar *color = btheme->tui.wcol_menu_back.text_sel;

    /* The top margin. */
    layout.row(false).separator(0.2f);

    /* The logo image. */
    layout.row(false).alignment_set(ui::LayoutAlign::Left);
    uiDefButImage(block, ibuf, 0, U.widget_unit, ibuf->x, ibuf->y, show_color ? nullptr : color);

    /* Padding below the logo. */
    layout.row(false).separator(2.7f);
  }
#endif /* !WITH_HEADLESS */

  ui::Layout &col = layout.column(true);

  uiItemL_ex(&col, SEKAIBLENDER_PRODUCT_NAME, ICON_NONE, true, false);

  MenuType *mt = WM_menutype_find("WM_MT_splash_about", true);
  if (mt) {
    ui::menutype_draw(C, mt, &col);
  }

  block_bounds_set_centered(block, 22 * UI_SCALE_FAC);

  return block;
}

static wmOperatorStatus wm_splash_about_invoke(bContext *C,
                                               wmOperator * /*op*/,
                                               const wmEvent * /*event*/)
{
  ui::popup_block_invoke(C, wm_block_about_create, nullptr, nullptr);

  return OPERATOR_FINISHED;
}

void WM_OT_splash_about(wmOperatorType *ot)
{
  ot->name = "About SekaiBlender";
  ot->idname = "WM_OT_splash_about";
  ot->description = "Open a window with information about SekaiBlender";

  ot->invoke = wm_splash_about_invoke;
  ot->poll = WM_operator_winactive;
}

/** \} */

}  // namespace blender
