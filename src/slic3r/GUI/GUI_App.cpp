#include "libslic3r/Technologies.hpp"
#include "GUI_App.hpp"
#include "GUI_Init.hpp"
#include "GUI_ObjectList.hpp"
#include "GUI_ObjectManipulation.hpp"
#include "I18N.hpp"

#include <algorithm>
#include <iterator>
#include <exception>
#include <cstdlib>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/convert.hpp>

#include <wx/stdpaths.h>
#include <wx/imagpng.h>
#include <wx/display.h>
#include <wx/menu.h>
#include <wx/menuitem.h>
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/dir.h>
#include <wx/wupdlock.h>
#include <wx/filefn.h>
#include <wx/sysopt.h>
#include <wx/richmsgdlg.h>
#include <wx/log.h>
#include <wx/intl.h>

#include <wx/dialog.h>
#include <wx/textctrl.h>
#include <wx/splash.h>
#include <wx/fontutil.h>

#include "libslic3r/Utils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/PresetBundle.hpp"

#include "GUI.hpp"
#include "GUI_Utils.hpp"
#include "3DScene.hpp"
#include "MainFrame.hpp"
#include "Plater.hpp"
#include "GLCanvas3D.hpp"

#include "../Utils/PresetUpdater.hpp"
#include "../Utils/PrintHost.hpp"
#include "../Utils/Process.hpp"
#include "../Utils/MacDarkMode.hpp"
#include "slic3r/Config/Snapshot.hpp"
#include "ConfigSnapshotDialog.hpp"
#include "FirmwareDialog.hpp"
#include "Preferences.hpp"
#include "Tab.hpp"
#include "SysInfoDialog.hpp"
#include "KBShortcutsDialog.hpp"
#include "UpdateDialogs.hpp"
#include "Mouse3DController.hpp"
#include "RemovableDriveManager.hpp"
#include "InstanceCheck.hpp"
#include "NotificationManager.hpp"
#include "UnsavedChangesDialog.hpp"
#include "SavePresetDialog.hpp"

#include "BitmapCache.hpp"

#ifdef __WXMSW__
#include <dbt.h>
#include <shlobj.h>
#endif // __WXMSW__

#if ENABLE_THUMBNAIL_GENERATOR_DEBUG
#include <boost/beast/core/detail/base64.hpp>
#include <boost/nowide/fstream.hpp>
#endif // ENABLE_THUMBNAIL_GENERATOR_DEBUG

namespace Slic3r {
namespace GUI {

class MainFrame;

class SplashScreen : public wxSplashScreen
{
public:
    SplashScreen(const wxBitmap& bitmap, long splashStyle, int milliseconds, wxPoint pos = wxDefaultPosition)
        : wxSplashScreen(bitmap, splashStyle, milliseconds, static_cast<wxWindow*>(wxGetApp().mainframe), wxID_ANY, wxDefaultPosition, wxDefaultSize,
#ifdef __APPLE__
            wxSIMPLE_BORDER | wxFRAME_NO_TASKBAR | wxSTAY_ON_TOP
#else
            wxSIMPLE_BORDER | wxFRAME_NO_TASKBAR
#endif // !__APPLE__
        )
    {
        wxASSERT(bitmap.IsOk());

        int init_dpi = get_dpi_for_window(this);
        this->SetPosition(pos);
        this->CenterOnScreen();
        int new_dpi = get_dpi_for_window(this);

        m_scale         = (float)(new_dpi) / (float)(init_dpi);
        m_main_bitmap   = bitmap;

        scale_bitmap(m_main_bitmap, m_scale);

        // init constant texts and scale fonts
        init_constant_text();

        // this font will be used for the action string
        m_action_font = m_constant_text.credits_font.Bold();

        // draw logo and constant info text
        Decorate(m_main_bitmap);
    }

    void SetText(const wxString& text)
    {
        set_bitmap(m_main_bitmap);
        if (!text.empty()) {
            wxBitmap bitmap(m_main_bitmap);

            wxMemoryDC memDC;
            memDC.SelectObject(bitmap);

            memDC.SetFont(m_action_font);
            memDC.SetTextForeground(wxColour(237, 107, 33));
            memDC.DrawText(text, int(m_scale * 60), int(m_scale * 275));

            memDC.SelectObject(wxNullBitmap);
            set_bitmap(bitmap);
#ifdef __WXOSX__
            // without this code splash screen wouldn't be updated under OSX
            wxYield();
#endif
        }
    }

    static wxBitmap MakeBitmap(wxBitmap bmp)
    {
        if (!bmp.IsOk())
            return wxNullBitmap;

        // create dark grey background for the splashscreen
        // It will be 5/3 of the weight of the bitmap
        int width = lround((double)5 / 3 * bmp.GetWidth());
        int height = bmp.GetHeight();

        wxImage image(width, height);
        unsigned char* imgdata_ = image.GetData();
        for (int i = 0; i < width * height; ++i) {
            *imgdata_++ = 51;
            *imgdata_++ = 51;
            *imgdata_++ = 51;
        }

        wxBitmap new_bmp(image);

        wxMemoryDC memDC;
        memDC.SelectObject(new_bmp);
        memDC.DrawBitmap(bmp, width - bmp.GetWidth(), 0, true);

        return new_bmp;
    }

    void Decorate(wxBitmap& bmp)
    {
        if (!bmp.IsOk())
            return;

        // draw text to the box at the left of the splashscreen.
        // this box will be 2/5 of the weight of the bitmap, and be at the left.
        int width = lround(bmp.GetWidth() * 0.4);

        // load bitmap for logo
        BitmapCache bmp_cache;
        int logo_size = lround(width * 0.25);
        wxBitmap logo_bmp = *bmp_cache.load_svg(wxGetApp().is_editor() ? "prusa_slicer_logo" : "add_gcode", logo_size, logo_size);

        wxCoord margin = int(m_scale * 20);

        wxRect banner_rect(wxPoint(0, logo_size), wxPoint(width, bmp.GetHeight()));
        banner_rect.Deflate(margin, 2 * margin);

        // use a memory DC to draw directly onto the bitmap
        wxMemoryDC memDc(bmp);

        // draw logo
        memDc.DrawBitmap(logo_bmp, margin, margin, true);

        // draw the (white) labels inside of our black box (at the left of the splashscreen)
        memDc.SetTextForeground(wxColour(255, 255, 255));

        memDc.SetFont(m_constant_text.title_font);
        memDc.DrawLabel(m_constant_text.title,   banner_rect, wxALIGN_TOP | wxALIGN_LEFT);

        int title_height = memDc.GetTextExtent(m_constant_text.title).GetY();
        banner_rect.SetTop(banner_rect.GetTop() + title_height);
        banner_rect.SetHeight(banner_rect.GetHeight() - title_height);

        memDc.SetFont(m_constant_text.version_font);
        memDc.DrawLabel(m_constant_text.version, banner_rect, wxALIGN_TOP | wxALIGN_LEFT);

        memDc.SetFont(m_constant_text.credits_font);
        memDc.DrawLabel(m_constant_text.credits, banner_rect, wxALIGN_BOTTOM | wxALIGN_LEFT);
    }

private:
    wxBitmap    m_main_bitmap;
    wxFont      m_action_font;
    float       m_scale {1.0};

    struct ConstantText
    {
        wxString title;
        wxString version;
        wxString credits;

        wxFont   title_font;
        wxFont   version_font;
        wxFont   credits_font;

        void init(wxFont init_font)
        {
            // title
            title = wxGetApp().is_editor() ? SLIC3R_APP_NAME : GCODEVIEWER_APP_NAME;

            // dynamically get the version to display
            version = _L("Version") + " " + std::string(SLIC3R_VERSION);

            // credits infornation
            credits =   title + " " +
                        _L("is based on Slic3r by Alessandro Ranellucci and the RepRap community.") + "\n\n" +
                        title + " " + _L("is licensed under the") + " " + _L("GNU Affero General Public License, version 3") + "\n\n" +
                        _L("Contributions by Vojtech Bubnik, Enrico Turri, Oleksandra Iushchenko, Tamas Meszaros, Lukas Matena, Vojtech Kral, David Kocik and numerous others.") + "\n\n" +
                        _L("Artwork model by Nora Al-Badri and Jan Nikolai Nelles");

            title_font = version_font = credits_font = init_font;
        }
    } 
    m_constant_text;

    void init_constant_text()
    {
        m_constant_text.init(get_default_font(this));

        // As default we use a system font for current display.
        // Scale fonts in respect to banner width

        int text_banner_width = lround(0.4 * m_main_bitmap.GetWidth()) - roundl(m_scale * 50); // banner_width - margins

        float title_font_scale = (float)text_banner_width / GetTextExtent(m_constant_text.title).GetX();
        scale_font(m_constant_text.title_font, title_font_scale > 3.5f ? 3.5f : title_font_scale);

        scale_font(m_constant_text.version_font, 2.f);

        // The width of the credits information string doesn't respect to the banner width some times.
        // So, scale credits_font in the respect to the longest string width
        int   longest_string_width = word_wrap_string(m_constant_text.credits);
        float font_scale = (float)text_banner_width / longest_string_width;
        scale_font(m_constant_text.credits_font, font_scale);
    }

    void set_bitmap(wxBitmap& bmp)
    {
        m_window->SetBitmap(bmp);
        m_window->Refresh();
        m_window->Update();
    }

    void scale_bitmap(wxBitmap& bmp, float scale)
    {
        if (scale == 1.0)
            return;

        wxImage image = bmp.ConvertToImage();
        if (!image.IsOk() || image.GetWidth() == 0 || image.GetHeight() == 0)
            return;

        int width   = int(scale * image.GetWidth());
        int height  = int(scale * image.GetHeight());
        image.Rescale(width, height, wxIMAGE_QUALITY_BILINEAR);

        bmp = wxBitmap(std::move(image));
    }

    void scale_font(wxFont& font, float scale)
    {
#ifdef __WXMSW__
        // Workaround for the font scaling in respect to the current active display,
        // not for the primary display, as it's implemented in Font.cpp
        // See https://github.com/wxWidgets/wxWidgets/blob/master/src/msw/font.cpp
        // void wxNativeFontInfo::SetFractionalPointSize(float pointSizeNew)
        wxNativeFontInfo nfi= *font.GetNativeFontInfo();
        float pointSizeNew  = scale * font.GetPointSize();
        nfi.lf.lfHeight     = nfi.GetLogFontHeightAtPPI(pointSizeNew, get_dpi_for_window(this));
        nfi.pointSize       = pointSizeNew;
        font = wxFont(nfi);
#else
        font.Scale(scale);
#endif //__WXMSW__
    }

    // wrap a string for the strings no longer then 55 symbols
    // return extent of the longest string
    int word_wrap_string(wxString& input)
    {
        size_t line_len = 55;// count of symbols in one line
        int idx = -1;
        size_t cur_len = 0;

        wxString longest_sub_string;
        auto get_longest_sub_string = [longest_sub_string, input](wxString &longest_sub_str, int cur_len, size_t i) {
            if (cur_len > longest_sub_str.Len())
                longest_sub_str = input.SubString(i - cur_len + 1, i);
        };

        for (size_t i = 0; i < input.Len(); i++)
        {
            cur_len++;
            if (input[i] == ' ')
                idx = i;
            if (input[i] == '\n')
            {
                get_longest_sub_string(longest_sub_string, cur_len, i);
                idx = -1;
                cur_len = 0;
            }
            if (cur_len >= line_len && idx >= 0)
            {
                get_longest_sub_string(longest_sub_string, cur_len, i);
                input[idx] = '\n';
                cur_len = i - static_cast<size_t>(idx);
            }
        }

        return GetTextExtent(longest_sub_string).GetX();
    }
};


#ifdef __linux__
bool static check_old_linux_datadir(const wxString& app_name) {
    // If we are on Linux and the datadir does not exist yet, look into the old
    // location where the datadir was before version 2.3. If we find it there,
    // tell the user that he might wanna migrate to the new location.
    // (https://github.com/prusa3d/PrusaSlicer/issues/2911)
    // To be precise, the datadir should exist, it is created when single instance
    // lock happens. Instead of checking for existence, check the contents.

    namespace fs = boost::filesystem;

    std::string new_path = Slic3r::data_dir();

    wxString dir;
    if (! wxGetEnv(wxS("XDG_CONFIG_HOME"), &dir) || dir.empty() )
        dir = wxFileName::GetHomeDir() + wxS("/.config");
    std::string default_path = (dir + "/" + app_name).ToUTF8().data();

    if (new_path != default_path) {
        // This happens when the user specifies a custom --datadir.
        // Do not show anything in that case.
        return true;
    }

    fs::path data_dir = fs::path(new_path);
    if (! fs::is_directory(data_dir))
        return true; // This should not happen.

    int file_count = std::distance(fs::directory_iterator(data_dir), fs::directory_iterator());

    if (file_count <= 1) { // just cache dir with an instance lock
        std::string old_path = wxStandardPaths::Get().GetUserDataDir().ToUTF8().data();

        if (fs::is_directory(old_path)) {
            wxString msg = from_u8((boost::format(_u8L("Starting with %1% 2.3, configuration "
                "directory on Linux has changed (according to XDG Base Directory Specification) to \n%2%.\n\n"
                "This directory did not exist yet (maybe you run the new version for the first time).\nHowever, "
                "an old %1% configuration directory was detected in \n%3%.\n\n"
                "Consider moving the contents of the old directory to the new location in order to access "
                "your profiles, etc.\nNote that if you decide to downgrade %1% in future, it will use the old "
                "location again.\n\n"
                "What do you want to do now?")) % SLIC3R_APP_NAME % new_path % old_path).str());
            wxString caption = from_u8((boost::format(_u8L("%s - BREAKING CHANGE")) % SLIC3R_APP_NAME).str());
            wxRichMessageDialog dlg(nullptr, msg, caption, wxYES_NO);
            dlg.SetYesNoLabels(_L("Quit, I will move my data now"), _L("Start the application"));
            if (dlg.ShowModal() != wxID_NO)
                return false;
        }
    } else {
        // If the new directory exists, be silent. The user likely already saw the message.
    }
    return true;
}
#endif


wxString file_wildcards(FileType file_type, const std::string &custom_extension)
{
    static const std::string defaults[FT_SIZE] = {
        /* FT_STL */     "STL files (*.stl)|*.stl;*.STL",
        /* FT_OBJ */     "OBJ files (*.obj)|*.obj;*.OBJ",
        /* FT_AMF */     "AMF files (*.amf)|*.zip.amf;*.amf;*.AMF;*.xml;*.XML",
        /* FT_3MF */     "3MF files (*.3mf)|*.3mf;*.3MF;",
        /* FT_PRUSA */   "Prusa Control files (*.prusa)|*.prusa;*.PRUSA",
        /* FT_GCODE */   "G-code files (*.gcode, *.gco, *.g, *.ngc)|*.gcode;*.GCODE;*.gco;*.GCO;*.g;*.G;*.ngc;*.NGC",
        /* FT_MODEL */   "Known files (*.stl, *.obj, *.amf, *.xml, *.3mf, *.prusa)|*.stl;*.STL;*.obj;*.OBJ;*.amf;*.AMF;*.xml;*.XML;*.3mf;*.3MF;*.prusa;*.PRUSA",
        /* FT_PROJECT */ "Project files (*.3mf, *.amf)|*.3mf;*.3MF;*.amf;*.AMF",

        /* FT_INI */     "INI files (*.ini)|*.ini;*.INI",
        /* FT_SVG */     "SVG files (*.svg)|*.svg;*.SVG",

        /* FT_TEX */     "Texture (*.png, *.svg)|*.png;*.PNG;*.svg;*.SVG",

        /* FT_PNGZIP */  "Masked SLA files (*.sl1)|*.sl1;*.SL1",
    };

	std::string out = defaults[file_type];
    if (! custom_extension.empty()) {
        // Find the custom extension in the template.
        if (out.find(std::string("*") + custom_extension + ",") == std::string::npos && out.find(std::string("*") + custom_extension + ")") == std::string::npos) {
            // The custom extension was not found in the template.
            // Append the custom extension to the wildcards, so that the file dialog would not add the default extension to it.
			boost::replace_first(out, ")|", std::string(", *") + custom_extension + ")|");
			out += std::string(";*") + custom_extension;
        }
    }
    return from_u8(out);
}

static std::string libslic3r_translate_callback(const char *s) { return wxGetTranslation(wxString(s, wxConvUTF8)).utf8_str().data(); }

#ifdef WIN32
#if !wxVERSION_EQUAL_OR_GREATER_THAN(3,1,3)
static void register_win32_dpi_event()
{
    enum { WM_DPICHANGED_ = 0x02e0 };

    wxWindow::MSWRegisterMessageHandler(WM_DPICHANGED_, [](wxWindow *win, WXUINT nMsg, WXWPARAM wParam, WXLPARAM lParam) {
        const int dpi = wParam & 0xffff;
        const auto rect = reinterpret_cast<PRECT>(lParam);
        const wxRect wxrect(wxPoint(rect->top, rect->left), wxPoint(rect->bottom, rect->right));

        DpiChangedEvent evt(EVT_DPI_CHANGED_SLICER, dpi, wxrect);
        win->GetEventHandler()->AddPendingEvent(evt);

        return true;
    });
}
#endif // !wxVERSION_EQUAL_OR_GREATER_THAN

static GUID GUID_DEVINTERFACE_HID = { 0x4D1E55B2, 0xF16F, 0x11CF, 0x88, 0xCB, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30 };

static void register_win32_device_notification_event()
{
    wxWindow::MSWRegisterMessageHandler(WM_DEVICECHANGE, [](wxWindow *win, WXUINT /* nMsg */, WXWPARAM wParam, WXLPARAM lParam) {
        // Some messages are sent to top level windows by default, some messages are sent to only registered windows, and we explictely register on MainFrame only.
        auto main_frame = dynamic_cast<MainFrame*>(win);
        auto plater = (main_frame == nullptr) ? nullptr : main_frame->plater();
        if (plater == nullptr)
            // Maybe some other top level window like a dialog or maybe a pop-up menu?
            return true;
		PDEV_BROADCAST_HDR lpdb = (PDEV_BROADCAST_HDR)lParam;
        switch (wParam) {
        case DBT_DEVICEARRIVAL:
			if (lpdb->dbch_devicetype == DBT_DEVTYP_VOLUME)
		        plater->GetEventHandler()->AddPendingEvent(VolumeAttachedEvent(EVT_VOLUME_ATTACHED));
			else if (lpdb->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE) {
				PDEV_BROADCAST_DEVICEINTERFACE lpdbi = (PDEV_BROADCAST_DEVICEINTERFACE)lpdb;
//				if (lpdbi->dbcc_classguid == GUID_DEVINTERFACE_VOLUME) {
//					printf("DBT_DEVICEARRIVAL %d - Media has arrived: %ws\n", msg_count, lpdbi->dbcc_name);
				if (lpdbi->dbcc_classguid == GUID_DEVINTERFACE_HID)
			        plater->GetEventHandler()->AddPendingEvent(HIDDeviceAttachedEvent(EVT_HID_DEVICE_ATTACHED, boost::nowide::narrow(lpdbi->dbcc_name)));
			}
            break;
		case DBT_DEVICEREMOVECOMPLETE:
			if (lpdb->dbch_devicetype == DBT_DEVTYP_VOLUME)
                plater->GetEventHandler()->AddPendingEvent(VolumeDetachedEvent(EVT_VOLUME_DETACHED));
			else if (lpdb->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE) {
				PDEV_BROADCAST_DEVICEINTERFACE lpdbi = (PDEV_BROADCAST_DEVICEINTERFACE)lpdb;
//				if (lpdbi->dbcc_classguid == GUID_DEVINTERFACE_VOLUME)
//					printf("DBT_DEVICEARRIVAL %d - Media was removed: %ws\n", msg_count, lpdbi->dbcc_name);
				if (lpdbi->dbcc_classguid == GUID_DEVINTERFACE_HID)
        			plater->GetEventHandler()->AddPendingEvent(HIDDeviceDetachedEvent(EVT_HID_DEVICE_DETACHED, boost::nowide::narrow(lpdbi->dbcc_name)));
			}
			break;
        default:
            break;
        }
        return true;
    });

    wxWindow::MSWRegisterMessageHandler(MainFrame::WM_USER_MEDIACHANGED, [](wxWindow *win, WXUINT /* nMsg */, WXWPARAM wParam, WXLPARAM lParam) {
        // Some messages are sent to top level windows by default, some messages are sent to only registered windows, and we explictely register on MainFrame only.
        auto main_frame = dynamic_cast<MainFrame*>(win);
        auto plater = (main_frame == nullptr) ? nullptr : main_frame->plater();
        if (plater == nullptr)
            // Maybe some other top level window like a dialog or maybe a pop-up menu?
            return true;
        wchar_t sPath[MAX_PATH];
        if (lParam == SHCNE_MEDIAINSERTED || lParam == SHCNE_MEDIAREMOVED) {
            struct _ITEMIDLIST* pidl = *reinterpret_cast<struct _ITEMIDLIST**>(wParam);
            if (! SHGetPathFromIDList(pidl, sPath)) {
                BOOST_LOG_TRIVIAL(error) << "MediaInserted: SHGetPathFromIDList failed";
                return false;
            }
        }
        switch (lParam) {
        case SHCNE_MEDIAINSERTED:
        {
            //printf("SHCNE_MEDIAINSERTED %S\n", sPath);
            plater->GetEventHandler()->AddPendingEvent(VolumeAttachedEvent(EVT_VOLUME_ATTACHED));
            break;
        }
        case SHCNE_MEDIAREMOVED:
        {
            //printf("SHCNE_MEDIAREMOVED %S\n", sPath);
            plater->GetEventHandler()->AddPendingEvent(VolumeDetachedEvent(EVT_VOLUME_DETACHED));
            break;
        }
	    default:
//          printf("Unknown\n");
            break;
	    }
        return true;
    });

    wxWindow::MSWRegisterMessageHandler(WM_INPUT, [](wxWindow *win, WXUINT /* nMsg */, WXWPARAM wParam, WXLPARAM lParam) {
        auto main_frame = dynamic_cast<MainFrame*>(Slic3r::GUI::find_toplevel_parent(win));
        auto plater = (main_frame == nullptr) ? nullptr : main_frame->plater();
//        if (wParam == RIM_INPUTSINK && plater != nullptr && main_frame->IsActive()) {
        if (wParam == RIM_INPUT && plater != nullptr && main_frame->IsActive()) {
        RAWINPUT raw;
			UINT rawSize = sizeof(RAWINPUT);
			::GetRawInputData((HRAWINPUT)lParam, RID_INPUT, &raw, &rawSize, sizeof(RAWINPUTHEADER));
			if (raw.header.dwType == RIM_TYPEHID && plater->get_mouse3d_controller().handle_raw_input_win32(raw.data.hid.bRawData, raw.data.hid.dwSizeHid))
				return true;
		}
        return false;
    });

	wxWindow::MSWRegisterMessageHandler(WM_COPYDATA, [](wxWindow* win, WXUINT /* nMsg */, WXWPARAM wParam, WXLPARAM lParam) {
		COPYDATASTRUCT* copy_data_structure = { 0 };
		copy_data_structure = (COPYDATASTRUCT*)lParam;
		if (copy_data_structure->dwData == 1) {
			LPCWSTR arguments = (LPCWSTR)copy_data_structure->lpData;
			Slic3r::GUI::wxGetApp().other_instance_message_handler()->handle_message(boost::nowide::narrow(arguments));
		}
		return true;
		});
}
#endif // WIN32

static void generic_exception_handle()
{
    // Note: Some wxWidgets APIs use wxLogError() to report errors, eg. wxImage
    // - see https://docs.wxwidgets.org/3.1/classwx_image.html#aa249e657259fe6518d68a5208b9043d0
    //
    // wxLogError typically goes around exception handling and display an error dialog some time
    // after an error is logged even if exception handling and OnExceptionInMainLoop() take place.
    // This is why we use wxLogError() here as well instead of a custom dialog, because it accumulates
    // errors if multiple have been collected and displays just one error message for all of them.
    // Otherwise we would get multiple error messages for one missing png, for example.
    //
    // If a custom error message window (or some other solution) were to be used, it would be necessary
    // to turn off wxLogError() usage in wx APIs, most notably in wxImage
    // - see https://docs.wxwidgets.org/trunk/classwx_image.html#aa32e5d3507cc0f8c3330135bc0befc6a

    try {
        throw;
    } catch (const std::bad_alloc& ex) {
        // bad_alloc in main thread is most likely fatal. Report immediately to the user (wxLogError would be delayed)
        // and terminate the app so it is at least certain to happen now.
        wxString errmsg = wxString::Format(_L("%s has encountered an error. It was likely caused by running out of memory. "
                              "If you are sure you have enough RAM on your system, this may also be a bug and we would "
                              "be glad if you reported it.\n\nThe application will now terminate."), SLIC3R_APP_NAME);
        wxMessageBox(errmsg + "\n\n" + wxString(ex.what()), _L("Fatal error"), wxOK | wxICON_ERROR);
        BOOST_LOG_TRIVIAL(error) << boost::format("std::bad_alloc exception: %1%") % ex.what();
        std::terminate();
    } catch (const std::exception& ex) {
        wxLogError("Internal error: %s", ex.what());
        BOOST_LOG_TRIVIAL(error) << boost::format("Uncaught exception: %1%") % ex.what();
        throw;
    }
}

void GUI_App::post_init()
{
    assert(initialized());
    if (! this->initialized())
        throw Slic3r::RuntimeError("Calling post_init() while not yet initialized");

    if (this->init_params->start_as_gcodeviewer) {
        if (! this->init_params->input_files.empty())
            this->plater()->load_gcode(wxString::FromUTF8(this->init_params->input_files[0].c_str()));
    }
    else {
#if 0
        // Load the cummulative config over the currently active profiles.
        //FIXME if multiple configs are loaded, only the last one will have an effect.
        // We need to decide what to do about loading of separate presets (just print preset, just filament preset etc).
        // As of now only the full configs are supported here.
        if (!m_print_config.empty())
            this->gui->mainframe->load_config(m_print_config);
#endif
        if (! this->init_params->load_configs.empty())
            // Load the last config to give it a name at the UI. The name of the preset may be later
            // changed by loading an AMF or 3MF.
            //FIXME this is not strictly correct, as one may pass a print/filament/printer profile here instead of a full config.
            this->mainframe->load_config_file(this->init_params->load_configs.back());
        // If loading a 3MF file, the config is loaded from the last one.
        if (! this->init_params->input_files.empty())
            this->plater()->load_files(this->init_params->input_files, true, true);
        if (! this->init_params->extra_config.empty())
            this->mainframe->load_config(this->init_params->extra_config);
    }
}

IMPLEMENT_APP(GUI_App)

GUI_App::GUI_App(EAppMode mode)
    : wxApp()
    , m_app_mode(mode)
    , m_em_unit(10)
    , m_imgui(new ImGuiWrapper())
    , m_wizard(nullptr)
	, m_removable_drive_manager(std::make_unique<RemovableDriveManager>())
	, m_other_instance_message_handler(std::make_unique<OtherInstanceMessageHandler>())
{
	//app config initializes early becasuse it is used in instance checking in PrusaSlicer.cpp
	this->init_app_config();
}

GUI_App::~GUI_App()
{
    if (app_config != nullptr)
        delete app_config;

    if (preset_bundle != nullptr)
        delete preset_bundle;

    if (preset_updater != nullptr)
        delete preset_updater;
}

std::string GUI_App::get_gl_info(bool format_as_html, bool extensions)
{
    return OpenGLManager::get_gl_info().to_string(format_as_html, extensions);
}

wxGLContext* GUI_App::init_glcontext(wxGLCanvas& canvas)
{
    return m_opengl_mgr.init_glcontext(canvas);
}

bool GUI_App::init_opengl()
{
    return m_opengl_mgr.init_gl();
}

void GUI_App::init_app_config()
{
	// Profiles for the alpha are stored into the PrusaSlicer-alpha directory to not mix with the current release.
//	SetAppName(SLIC3R_APP_KEY);
	SetAppName(SLIC3R_APP_KEY "-beta");
//	SetAppDisplayName(SLIC3R_APP_NAME);

	// Set the Slic3r data directory at the Slic3r XS module.
	// Unix: ~/ .Slic3r
	// Windows : "C:\Users\username\AppData\Roaming\Slic3r" or "C:\Documents and Settings\username\Application Data\Slic3r"
	// Mac : "~/Library/Application Support/Slic3r"

    if (data_dir().empty()) {
        #ifndef __linux__
            set_data_dir(wxStandardPaths::Get().GetUserDataDir().ToUTF8().data());
        #else
            // Since version 2.3, config dir on Linux is in ${XDG_CONFIG_HOME}.
            // https://github.com/prusa3d/PrusaSlicer/issues/2911
            wxString dir;
            if (! wxGetEnv(wxS("XDG_CONFIG_HOME"), &dir) || dir.empty() )
                dir = wxFileName::GetHomeDir() + wxS("/.config");
            set_data_dir((dir + "/" + GetAppName()).ToUTF8().data());
        #endif
    }

	if (!app_config)
        app_config = new AppConfig(is_editor() ? AppConfig::EAppMode::Editor : AppConfig::EAppMode::GCodeViewer);

	// load settings
	m_app_conf_exists = app_config->exists();
	if (m_app_conf_exists) {
        std::string error = app_config->load();
        if (!error.empty()) {
            // Error while parsing config file. We'll customize the error message and rethrow to be displayed.
            if (is_editor()) {
                throw Slic3r::RuntimeError(
                    _u8L("Error parsing PrusaSlicer config file, it is probably corrupted. "
                        "Try to manually delete the file to recover from the error. Your user profiles will not be affected.") +
                    "\n\n" + app_config->config_path() + "\n\n" + error);
            }
            else {
                throw Slic3r::RuntimeError(
                    _u8L("Error parsing PrusaGCodeViewer config file, it is probably corrupted. "
                        "Try to manually delete the file to recover from the error.") +
                    "\n\n" + app_config->config_path() + "\n\n" + error);
            }
        }
    }
}

void GUI_App::init_single_instance_checker(const std::string &name, const std::string &path)
{
    BOOST_LOG_TRIVIAL(debug) << "init wx instance checker " << name << " "<< path; 
    m_single_instance_checker = std::make_unique<wxSingleInstanceChecker>(boost::nowide::widen(name), boost::nowide::widen(path));
}

bool GUI_App::OnInit()
{
    try {
        return on_init_inner();
    } catch (const std::exception&) {
        generic_exception_handle();
        return false;
    }
}

bool GUI_App::on_init_inner()
{
    // Verify resources path
    const wxString resources_dir = from_u8(Slic3r::resources_dir());
    wxCHECK_MSG(wxDirExists(resources_dir), false,
        wxString::Format("Resources path does not exist or is not a directory: %s", resources_dir));

#ifdef __linux__
    if (! check_old_linux_datadir(GetAppName())) {
        std::cerr << "Quitting, user chose to move his data to new location." << std::endl;
        return false;
    }
#endif

    // Enable this to get the default Win32 COMCTRL32 behavior of static boxes.
//    wxSystemOptions::SetOption("msw.staticbox.optimized-paint", 0);
    // Enable this to disable Windows Vista themes for all wxNotebooks. The themes seem to lead to terrible
    // performance when working on high resolution multi-display setups.
//    wxSystemOptions::SetOption("msw.notebook.themed-background", 0);

//     Slic3r::debugf "wxWidgets version %s, Wx version %s\n", wxVERSION_STRING, wxVERSION;

    if (is_editor()) {
        std::string msg = Http::tls_global_init();
        std::string ssl_cert_store = app_config->get("tls_accepted_cert_store_location");
        bool ssl_accept = app_config->get("tls_cert_store_accepted") == "yes" && ssl_cert_store == Http::tls_system_cert_store();

        if (!msg.empty() && !ssl_accept) {
            wxRichMessageDialog
                dlg(nullptr,
                    wxString::Format(_L("%s\nDo you want to continue?"), msg),
                    "PrusaSlicer", wxICON_QUESTION | wxYES_NO);
            dlg.ShowCheckBox(_L("Remember my choice"));
            if (dlg.ShowModal() != wxID_YES) return false;

            app_config->set("tls_cert_store_accepted",
                dlg.IsCheckBoxChecked() ? "yes" : "no");
            app_config->set("tls_accepted_cert_store_location",
                dlg.IsCheckBoxChecked() ? Http::tls_system_cert_store() : "");
        }
    }

    app_config->set("version", SLIC3R_VERSION);
    app_config->save();

    wxInitAllImageHandlers();

    SplashScreen* scrn = nullptr;
    if (app_config->get("show_splash_screen") == "1") {
        // make a bitmap with dark grey banner on the left side
        wxBitmap bmp = SplashScreen::MakeBitmap(wxBitmap(from_u8(var(is_editor() ? "splashscreen.jpg" : "splashscreen-gcodepreview.jpg")), wxBITMAP_TYPE_JPEG));

        // Detect position (display) to show the splash screen
        // Now this position is equal to the mainframe position
        wxPoint splashscreen_pos = wxDefaultPosition;
        if (app_config->has("window_mainframe")) {
            auto metrics = WindowMetrics::deserialize(app_config->get("window_mainframe"));
            if (metrics)
                splashscreen_pos = metrics->get_rect().GetPosition();
        }

        // create splash screen with updated bmp
        scrn = new SplashScreen(bmp.IsOk() ? bmp : create_scaled_bitmap("prusa_slicer_logo", nullptr, 400), 
                                wxSPLASH_CENTRE_ON_SCREEN | wxSPLASH_TIMEOUT, 4000, splashscreen_pos);
#ifndef __linux__
        wxYield();
#endif
        scrn->SetText(_L("Loading configuration")+ dots);
    }

    preset_bundle = new PresetBundle();

    // just checking for existence of Slic3r::data_dir is not enough : it may be an empty directory
    // supplied as argument to --datadir; in that case we should still run the wizard
    preset_bundle->setup_directories();

    if (is_editor()) {
#ifdef __WXMSW__ 
#if ENABLE_CUSTOMIZABLE_FILES_ASSOCIATION_ON_WIN
        if (app_config->get("associate_3mf") == "1")
#endif // ENABLE_CUSTOMIZABLE_FILES_ASSOCIATION_ON_WIN
            associate_3mf_files();
#if ENABLE_CUSTOMIZABLE_FILES_ASSOCIATION_ON_WIN
        if (app_config->get("associate_stl") == "1")
            associate_stl_files();
#endif // ENABLE_CUSTOMIZABLE_FILES_ASSOCIATION_ON_WIN
#endif // __WXMSW__

        preset_updater = new PresetUpdater();
        Bind(EVT_SLIC3R_VERSION_ONLINE, [this](const wxCommandEvent& evt) {
            app_config->set("version_online", into_u8(evt.GetString()));
            app_config->save();
            if (this->plater_ != nullptr) {
                if (*Semver::parse(SLIC3R_VERSION) < *Semver::parse(into_u8(evt.GetString()))) {
                    this->plater_->get_notification_manager()->push_notification(NotificationType::NewAppAvailable, *(this->plater_->get_current_canvas3D()));
                }
            }
            });
    }
    else {
#ifdef __WXMSW__ 
#if ENABLE_CUSTOMIZABLE_FILES_ASSOCIATION_ON_WIN
        if (app_config->get("associate_gcode") == "1")
#endif // ENABLE_CUSTOMIZABLE_FILES_ASSOCIATION_ON_WIN
            associate_gcode_files();
#endif // __WXMSW__
    }

    // initialize label colors and fonts
    init_label_colours();
    init_fonts();

    // If load_language() fails, the application closes.
    load_language(wxString(), true);

    // Suppress the '- default -' presets.
    preset_bundle->set_default_suppressed(app_config->get("no_defaults") == "1");
    try {
        preset_bundle->load_presets(*app_config);
    } catch (const std::exception &ex) {
        show_error(nullptr, ex.what());
    }

#ifdef WIN32
#if !wxVERSION_EQUAL_OR_GREATER_THAN(3,1,3)
    register_win32_dpi_event();
#endif // !wxVERSION_EQUAL_OR_GREATER_THAN
    register_win32_device_notification_event();
#endif // WIN32

    // Let the libslic3r know the callback, which will translate messages on demand.
    Slic3r::I18N::set_translate_callback(libslic3r_translate_callback);

    // application frame
    if (scrn && is_editor())
        scrn->SetText(_L("Preparing settings tabs") + dots);

    mainframe = new MainFrame();
    // hide settings tabs after first Layout
    if (is_editor())
        mainframe->select_tab(size_t(0));

    sidebar().obj_list()->init_objects(); // propagate model objects to object list
//     update_mode(); // !!! do that later
    SetTopWindow(mainframe);

    m_printhost_job_queue.reset(new PrintHostJobQueue(mainframe->printhost_queue_dlg()));


    Bind(wxEVT_IDLE, [this](wxIdleEvent& event)
    {
        if (! plater_)
            return;


        if (app_config->dirty() && app_config->get("autosave") == "1")
            app_config->save();

        this->obj_manipul()->update_if_dirty();

        static bool update_gui_after_init = true;
        if (update_gui_after_init) {
            update_gui_after_init = false;
#ifdef WIN32
            this->mainframe->register_win32_callbacks();
#endif
            this->post_init();
        }

		// Preset updating & Configwizard are done after the above initializations,
	    // and after MainFrame is created & shown.
	    // The extra CallAfter() is needed because of Mac, where this is the only way
	    // to popup a modal dialog on start without screwing combo boxes.
	    // This is ugly but I honestly found no better way to do it.
	    // Neither wxShowEvent nor wxWindowCreateEvent work reliably. 

        static bool once = true;
        if (once) {
            once = false;

            if (preset_updater != nullptr) {
                check_updates(false);

                CallAfter([this] {
                    config_wizard_startup();
                    preset_updater->slic3r_update_notify();
                    preset_updater->sync(preset_bundle);
                    });
            }

#ifdef _WIN32
			//sets window property to mainframe so other instances can indentify it
			OtherInstanceMessageHandler::init_windows_properties(mainframe, m_instance_hash_int);
#endif //WIN32
        }
    });

    if (is_gcode_viewer()) {
        mainframe->update_layout();
        if (plater_ != nullptr)
            // ensure the selected technology is ptFFF
            plater_->set_printer_technology(ptFFF);
    }
    else
        load_current_presets();
    mainframe->Show(true);

    /* Temporary workaround for the correct behavior of the Scrolled sidebar panel:
     * change min hight of object list to the normal min value (15 * wxGetApp().em_unit()) 
     * after first whole Mainframe updating/layouting
     */
    const int list_min_height = 15 * em_unit();
    if (obj_list()->GetMinSize().GetY() > list_min_height)
        obj_list()->SetMinSize(wxSize(-1, list_min_height));

    update_mode(); // update view mode after fix of the object_list size

#ifdef __APPLE__
    other_instance_message_handler()->bring_instance_forward();
#endif //__APPLE__

    m_initialized = true;
    return true;
}

unsigned GUI_App::get_colour_approx_luma(const wxColour &colour)
{
    double r = colour.Red();
    double g = colour.Green();
    double b = colour.Blue();

    return std::round(std::sqrt(
        r * r * .241 +
        g * g * .691 +
        b * b * .068
        ));
}

bool GUI_App::dark_mode()
{
#if __APPLE__
    // The check for dark mode returns false positive on 10.12 and 10.13,
    // which allowed setting dark menu bar and dock area, which is
    // is detected as dark mode. We must run on at least 10.14 where the
    // proper dark mode was first introduced.
    return wxPlatformInfo::Get().CheckOSVersion(10, 14) && mac_dark_mode();
#else
    const unsigned luma = get_colour_approx_luma(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
    return luma < 128;
#endif
}

void GUI_App::init_label_colours()
{
    if (dark_mode()) {
        m_color_label_modified = wxColour(253, 111, 40);
        m_color_label_sys = wxColour(115, 220, 103);
    }
    else {
        m_color_label_modified = wxColour(252, 77, 1);
        m_color_label_sys = wxColour(26, 132, 57);
    }
    m_color_label_default = wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT);
}

void GUI_App::update_label_colours_from_appconfig()
{
    if (app_config->has("label_clr_sys")) {
        auto str = app_config->get("label_clr_sys");
        if (str != "")
            m_color_label_sys = wxColour(str);
    }

    if (app_config->has("label_clr_modified")) {
        auto str = app_config->get("label_clr_modified");
        if (str != "")
            m_color_label_modified = wxColour(str);
    }
}

void GUI_App::init_fonts()
{
    m_small_font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    m_bold_font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT).Bold();
    m_normal_font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);

#ifdef __WXMAC__
    m_small_font.SetPointSize(11);
    m_bold_font.SetPointSize(13);
#endif /*__WXMAC__*/

    // wxSYS_OEM_FIXED_FONT and wxSYS_ANSI_FIXED_FONT use the same as
    // DEFAULT in wxGtk. Use the TELETYPE family as a work-around
    m_code_font = wxFont(wxFontInfo().Family(wxFONTFAMILY_TELETYPE));
    m_code_font.SetPointSize(m_normal_font.GetPointSize());
}

void GUI_App::update_fonts(const MainFrame *main_frame)
{
    /* Only normal and bold fonts are used for an application rescale,
     * because of under MSW small and normal fonts are the same.
     * To avoid same rescaling twice, just fill this values
     * from rescaled MainFrame
     */
	if (main_frame == nullptr)
		main_frame = this->mainframe;
    m_normal_font   = main_frame->normal_font();
    m_small_font    = m_normal_font;
    m_bold_font     = main_frame->normal_font().Bold();
    m_em_unit       = main_frame->em_unit();
    m_code_font.SetPointSize(m_normal_font.GetPointSize());
}

void GUI_App::set_label_clr_modified(const wxColour& clr) {
    m_color_label_modified = clr;
    auto clr_str = wxString::Format(wxT("#%02X%02X%02X"), clr.Red(), clr.Green(), clr.Blue());
    std::string str = clr_str.ToStdString();
    app_config->set("label_clr_modified", str);
    app_config->save();
}

void GUI_App::set_label_clr_sys(const wxColour& clr) {
    m_color_label_sys = clr;
    auto clr_str = wxString::Format(wxT("#%02X%02X%02X"), clr.Red(), clr.Green(), clr.Blue());
    std::string str = clr_str.ToStdString();
    app_config->set("label_clr_sys", str);
    app_config->save();
}

wxSize GUI_App::get_min_size() const
{
    return wxSize(76*m_em_unit, 49 * m_em_unit);
}

float GUI_App::toolbar_icon_scale(const bool is_limited/* = false*/) const
{
#ifdef __APPLE__
    const float icon_sc = 1.0f; // for Retina display will be used its own scale
#else
    const float icon_sc = m_em_unit*0.1f;
#endif // __APPLE__

    const std::string& use_val  = app_config->get("use_custom_toolbar_size");
    const std::string& val      = app_config->get("custom_toolbar_size");
    const std::string& auto_val = app_config->get("auto_toolbar_size");

    if (val.empty() || auto_val.empty() || use_val.empty())
        return icon_sc;

    int int_val = use_val == "0" ? 100 : atoi(val.c_str());
    // correct value in respect to auto_toolbar_size
    int_val = std::min(atoi(auto_val.c_str()), int_val);

    if (is_limited && int_val < 50)
        int_val = 50;

    return 0.01f * int_val * icon_sc;
}

void GUI_App::set_auto_toolbar_icon_scale(float scale) const
{
#ifdef __APPLE__
    const float icon_sc = 1.0f; // for Retina display will be used its own scale
#else
    const float icon_sc = m_em_unit * 0.1f;
#endif // __APPLE__

    long int_val = std::min(int(std::lround(scale / icon_sc * 100)), 100);
    std::string val = std::to_string(int_val);

    app_config->set("auto_toolbar_size", val);
}

// check user printer_presets for the containing information about "Print Host upload"
void GUI_App::check_printer_presets()
{
    std::vector<std::string> preset_names = PhysicalPrinter::presets_with_print_host_information(preset_bundle->printers);
    if (preset_names.empty())
        return;

    wxString msg_text =  _L("You have the following presets with saved options for \"Print Host upload\"") + ":";
    for (const std::string& preset_name : preset_names)
        msg_text += "\n    \"" + from_u8(preset_name) + "\",";
    msg_text.RemoveLast();
    msg_text += "\n\n" + _L("But since this version of PrusaSlicer we don't show this information in Printer Settings anymore.\n"
                            "Settings will be available in physical printers settings.") + "\n\n" +
                         _L("By default new Printer devices will be named as \"Printer N\" during its creation.\n"
                            "Note: This name can be changed later from the physical printers settings");

    wxMessageDialog(nullptr, msg_text, _L("Information"), wxOK | wxICON_INFORMATION).ShowModal();

    preset_bundle->physical_printers.load_printers_from_presets(preset_bundle->printers);
}

void GUI_App::recreate_GUI(const wxString& msg_name)
{
    m_is_recreating_gui = true;

    mainframe->shutdown();

    wxProgressDialog dlg(msg_name, msg_name, 100, nullptr, wxPD_AUTO_HIDE);
    dlg.Pulse();
    dlg.Update(10, _L("Recreating") + dots);

    MainFrame *old_main_frame = mainframe;
    mainframe = new MainFrame();
    if (is_editor())
        // hide settings tabs after first Layout
        mainframe->select_tab(size_t(0));
    // Propagate model objects to object list.
    sidebar().obj_list()->init_objects();
    SetTopWindow(mainframe);

    dlg.Update(30, _L("Recreating") + dots);
    old_main_frame->Destroy();
    // For this moment ConfigWizard is deleted, invalidate it.
    m_wizard = nullptr;

    dlg.Update(80, _L("Loading of current presets") + dots);
    m_printhost_job_queue.reset(new PrintHostJobQueue(mainframe->printhost_queue_dlg()));
    load_current_presets();
    mainframe->Show(true);

    dlg.Update(90, _L("Loading of a mode view") + dots);
    /* Temporary workaround for the correct behavior of the Scrolled sidebar panel:
    * change min hight of object list to the normal min value (15 * wxGetApp().em_unit())
    * after first whole Mainframe updating/layouting
    */
    const int list_min_height = 15 * em_unit();
    if (obj_list()->GetMinSize().GetY() > list_min_height)
        obj_list()->SetMinSize(wxSize(-1, list_min_height));
    update_mode();

    // #ys_FIXME_delete_after_testing  Do we still need this  ?
//     CallAfter([]() {
//         // Run the config wizard, don't offer the "reset user profile" checkbox.
//         config_wizard_startup(true);
//     });

    m_is_recreating_gui = false;
}

void GUI_App::system_info()
{
    SysInfoDialog dlg;
    dlg.ShowModal();
}

void GUI_App::keyboard_shortcuts()
{
    KBShortcutsDialog dlg;
    dlg.ShowModal();
}

// static method accepting a wxWindow object as first parameter
bool GUI_App::catch_error(std::function<void()> cb,
    //                       wxMessageDialog* message_dialog,
    const std::string& err /*= ""*/)
{
    if (!err.empty()) {
        if (cb)
            cb();
        //         if (message_dialog)
        //             message_dialog->(err, "Error", wxOK | wxICON_ERROR);
        show_error(/*this*/nullptr, err);
        return true;
    }
    return false;
}

// static method accepting a wxWindow object as first parameter
void fatal_error(wxWindow* parent)
{
    show_error(parent, "");
    //     exit 1; // #ys_FIXME
}

// Called after the Preferences dialog is closed and the program settings are saved.
// Update the UI based on the current preferences.
void GUI_App::update_ui_from_settings(bool apply_free_camera_correction)
{
    mainframe->update_ui_from_settings(apply_free_camera_correction);
}

void GUI_App::persist_window_geometry(wxTopLevelWindow *window, bool default_maximized)
{
    const std::string name = into_u8(window->GetName());

    window->Bind(wxEVT_CLOSE_WINDOW, [=](wxCloseEvent &event) {
        window_pos_save(window, name);
        event.Skip();
    });

    window_pos_restore(window, name, default_maximized);

    on_window_geometry(window, [=]() {
        window_pos_sanitize(window);
    });
}

void GUI_App::load_project(wxWindow *parent, wxString& input_file) const
{
    input_file.Clear();
    wxFileDialog dialog(parent ? parent : GetTopWindow(),
        _L("Choose one file (3MF/AMF):"),
        app_config->get_last_dir(), "",
        file_wildcards(FT_PROJECT), wxFD_OPEN | wxFD_FILE_MUST_EXIST);

    if (dialog.ShowModal() == wxID_OK)
        input_file = dialog.GetPath();
}

void GUI_App::import_model(wxWindow *parent, wxArrayString& input_files) const
{
    input_files.Clear();
    wxFileDialog dialog(parent ? parent : GetTopWindow(),
        _L("Choose one or more files (STL/OBJ/AMF/3MF/PRUSA):"),
        from_u8(app_config->get_last_dir()), "",
        file_wildcards(FT_MODEL), wxFD_OPEN | wxFD_MULTIPLE | wxFD_FILE_MUST_EXIST);

    if (dialog.ShowModal() == wxID_OK)
        dialog.GetPaths(input_files);
}

void GUI_App::load_gcode(wxWindow* parent, wxString& input_file) const
{
    input_file.Clear();
    wxFileDialog dialog(parent ? parent : GetTopWindow(),
        _L("Choose one file (GCODE/.GCO/.G/.ngc/NGC):"),
        app_config->get_last_dir(), "",
        file_wildcards(FT_GCODE), wxFD_OPEN | wxFD_FILE_MUST_EXIST);

    if (dialog.ShowModal() == wxID_OK)
        input_file = dialog.GetPath();
}

bool GUI_App::switch_language()
{
    if (select_language()) {
        recreate_GUI(_L("Changing of an application language") + dots);
        return true;
    } else {
        return false;
    }
}

// select language from the list of installed languages
bool GUI_App::select_language()
{
	wxArrayString translations = wxTranslations::Get()->GetAvailableTranslations(SLIC3R_APP_KEY);
    std::vector<const wxLanguageInfo*> language_infos;
    language_infos.emplace_back(wxLocale::GetLanguageInfo(wxLANGUAGE_ENGLISH));
#ifdef __linux__
    // wxWidgets consider the default English locale to be en_GB, which is often missing on Linux.
    // Thus we offer en_US on Linux as well.
    language_infos.emplace_back(wxLocale::GetLanguageInfo(wxLANGUAGE_ENGLISH_US));
#endif // __linux__
    for (size_t i = 0; i < translations.GetCount(); ++ i) {
	    const wxLanguageInfo *langinfo = wxLocale::FindLanguageInfo(translations[i]);
        if (langinfo != nullptr)
            language_infos.emplace_back(langinfo);
    }
    sort_remove_duplicates(language_infos);
	std::sort(language_infos.begin(), language_infos.end(), [](const wxLanguageInfo* l, const wxLanguageInfo* r) { return l->Description < r->Description; });

    wxArrayString names;
    names.Alloc(language_infos.size());

    // Some valid language should be selected since the application start up.
    const wxLanguage current_language = wxLanguage(m_wxLocale->GetLanguage());
    int 		     init_selection   		= -1;
    int 			 init_selection_alt     = -1;
    int 			 init_selection_default = -1;
    for (size_t i = 0; i < language_infos.size(); ++ i) {
        if (wxLanguage(language_infos[i]->Language) == current_language)
        	// The dictionary matches the active language and country.
            init_selection = i;
        else if ((language_infos[i]->CanonicalName.BeforeFirst('_') == m_wxLocale->GetCanonicalName().BeforeFirst('_')) ||
        		 // if the active language is Slovak, mark the Czech language as active.
        	     (language_infos[i]->CanonicalName.BeforeFirst('_') == "cs" && m_wxLocale->GetCanonicalName().BeforeFirst('_') == "sk"))
        	// The dictionary matches the active language, it does not necessarily match the country.
        	init_selection_alt = i;
        if (language_infos[i]->CanonicalName.BeforeFirst('_') == "en")
        	// This will be the default selection if the active language does not match any dictionary.
        	init_selection_default = i;
#ifdef __linux__
        // wxWidgets consider the default English locale to be en_GB, which is often missing on Linux.
        // Thus we make the distintion between "en_US" and "en_GB" clear.
        if (language_infos[i]->CanonicalName == "en_GB" && language_infos[i]->Description == "English")
            names.Add("English (U.K.)");
        else
#endif // __linux__
        names.Add(language_infos[i]->Description);
    }
    if (init_selection == -1)
    	// This is the dictionary matching the active language.
    	init_selection = init_selection_alt;
    if (init_selection != -1)
    	// This is the language to highlight in the choice dialog initially.
    	init_selection_default = init_selection;

    const long index = wxGetSingleChoiceIndex(_L("Select the language"), _L("Language"), names, init_selection_default);
	// Try to load a new language.
    if (index != -1 && (init_selection == -1 || init_selection != index)) {
    	const wxLanguageInfo *new_language_info = language_infos[index];
    	if (this->load_language(new_language_info->CanonicalName, false)) {
			// Save language at application config.
            // Which language to save as the selected dictionary language?
            // 1) Hopefully the language set to wxTranslations by this->load_language(), but that API is weird and we don't want to rely on its
            //    stability in the future:
            //    wxTranslations::Get()->GetBestTranslation(SLIC3R_APP_KEY, wxLANGUAGE_ENGLISH);
            // 2) Current locale language may not match the dictionary name, see GH issue #3901
            //    m_wxLocale->GetCanonicalName()
            // 3) new_language_info->CanonicalName is a safe bet. It points to a valid dictionary name.
			app_config->set("translation_language", new_language_info->CanonicalName.ToUTF8().data());            
			app_config->save();
    		return true;
    	}
    }

    return false;
}

// Load gettext translation files and activate them at the start of the application,
// based on the "translation_language" key stored in the application config.
bool GUI_App::load_language(wxString language, bool initial)
{
    if (initial) {
    	// There is a static list of lookup path prefixes in wxWidgets. Add ours.
	    wxFileTranslationsLoader::AddCatalogLookupPathPrefix(from_u8(localization_dir()));
    	// Get the active language from PrusaSlicer.ini, or empty string if the key does not exist.
        language = app_config->get("translation_language");
        if (! language.empty())
        	BOOST_LOG_TRIVIAL(trace) << boost::format("translation_language provided by PrusaSlicer.ini: %1%") % language;

        // Get the system language.
        {
	        const wxLanguage lang_system = wxLanguage(wxLocale::GetSystemLanguage());
	        if (lang_system != wxLANGUAGE_UNKNOWN) {
				m_language_info_system = wxLocale::GetLanguageInfo(lang_system);
	        	BOOST_LOG_TRIVIAL(trace) << boost::format("System language detected (user locales and such): %1%") % m_language_info_system->CanonicalName.ToUTF8().data();
	        }
		}
        {
	    	// Allocating a temporary locale will switch the default wxTranslations to its internal wxTranslations instance.
	    	wxLocale temp_locale;
	    	// Set the current translation's language to default, otherwise GetBestTranslation() may not work (see the wxWidgets source code).
	    	wxTranslations::Get()->SetLanguage(wxLANGUAGE_DEFAULT);
	    	// Let the wxFileTranslationsLoader enumerate all translation dictionaries for PrusaSlicer
	    	// and try to match them with the system specific "preferred languages". 
	    	// There seems to be a support for that on Windows and OSX, while on Linuxes the code just returns wxLocale::GetSystemLanguage().
	    	// The last parameter gets added to the list of detected dictionaries. This is a workaround 
	    	// for not having the English dictionary. Let's hope wxWidgets of various versions process this call the same way.
			wxString best_language = wxTranslations::Get()->GetBestTranslation(SLIC3R_APP_KEY, wxLANGUAGE_ENGLISH);
			if (! best_language.IsEmpty()) {
				m_language_info_best = wxLocale::FindLanguageInfo(best_language);
	        	BOOST_LOG_TRIVIAL(trace) << boost::format("Best translation language detected (may be different from user locales): %1%") % m_language_info_best->CanonicalName.ToUTF8().data();
			}
		}
    }

	const wxLanguageInfo *language_info = language.empty() ? nullptr : wxLocale::FindLanguageInfo(language);
	if (! language.empty() && (language_info == nullptr || language_info->CanonicalName.empty())) {
		// Fix for wxWidgets issue, where the FindLanguageInfo() returns locales with undefined ANSII code (wxLANGUAGE_KONKANI or wxLANGUAGE_MANIPURI).
		language_info = nullptr;
    	BOOST_LOG_TRIVIAL(error) << boost::format("Language code \"%1%\" is not supported") % language.ToUTF8().data();
	}

	if (language_info != nullptr && language_info->LayoutDirection == wxLayout_RightToLeft) {
    	BOOST_LOG_TRIVIAL(trace) << boost::format("The following language code requires right to left layout, which is not supported by PrusaSlicer: %1%") % language_info->CanonicalName.ToUTF8().data();
		language_info = nullptr;
	}

    if (language_info == nullptr) {
        // PrusaSlicer does not support the Right to Left languages yet.
        if (m_language_info_system != nullptr && m_language_info_system->LayoutDirection != wxLayout_RightToLeft)
            language_info = m_language_info_system;
        if (m_language_info_best != nullptr && m_language_info_best->LayoutDirection != wxLayout_RightToLeft)
        	language_info = m_language_info_best;
	    if (language_info == nullptr)
			language_info = wxLocale::GetLanguageInfo(wxLANGUAGE_ENGLISH_US);
    }

	BOOST_LOG_TRIVIAL(trace) << boost::format("Switching wxLocales to %1%") % language_info->CanonicalName.ToUTF8().data();

    // Alternate language code.
    wxLanguage language_dict = wxLanguage(language_info->Language);
    if (language_info->CanonicalName.BeforeFirst('_') == "sk") {
    	// Slovaks understand Czech well. Give them the Czech translation.
    	language_dict = wxLANGUAGE_CZECH;
		BOOST_LOG_TRIVIAL(trace) << "Using Czech dictionaries for Slovak language";
    }

    // Select language for locales. This language may be different from the language of the dictionary.
    if (language_info == m_language_info_best || language_info == m_language_info_system) {
        // The current language matches user's default profile exactly. That's great.
    } else if (m_language_info_best != nullptr && language_info->CanonicalName.BeforeFirst('_') == m_language_info_best->CanonicalName.BeforeFirst('_')) {
        // Use whatever the operating system recommends, if it the language code of the dictionary matches the recommended language.
        // This allows a Swiss guy to use a German dictionary without forcing him to German locales.
        language_info = m_language_info_best;
    } else if (m_language_info_system != nullptr && language_info->CanonicalName.BeforeFirst('_') == m_language_info_system->CanonicalName.BeforeFirst('_'))
        language_info = m_language_info_system;

    if (! wxLocale::IsAvailable(language_info->Language)) {
    	// Loading the language dictionary failed.
    	wxString message = "Switching PrusaSlicer to language " + language_info->CanonicalName + " failed.";
#if !defined(_WIN32) && !defined(__APPLE__)
        // likely some linux system
        message += "\nYou may need to reconfigure the missing locales, likely by running the \"locale-gen\" and \"dpkg-reconfigure locales\" commands.\n";
#endif
        if (initial)
        	message + "\n\nApplication will close.";
        wxMessageBox(message, "PrusaSlicer - Switching language failed", wxOK | wxICON_ERROR);
        if (initial)
			std::exit(EXIT_FAILURE);
		else
			return false;
    }

    // Release the old locales, create new locales.
    //FIXME wxWidgets cause havoc if the current locale is deleted. We just forget it causing memory leaks for now.
    m_wxLocale.release();
    m_wxLocale = Slic3r::make_unique<wxLocale>();
    m_wxLocale->Init(language_info->Language);
    // Override language at the active wxTranslations class (which is stored in the active m_wxLocale)
    // to load possibly different dictionary, for example, load Czech dictionary for Slovak language.
    wxTranslations::Get()->SetLanguage(language_dict);
    m_wxLocale->AddCatalog(SLIC3R_APP_KEY);
    m_imgui->set_language(into_u8(language_info->CanonicalName));
    //FIXME This is a temporary workaround, the correct solution is to switch to "C" locale during file import / export only.
    wxSetlocale(LC_NUMERIC, "C");
    Preset::update_suffix_modified((" (" + _L("modified") + ")").ToUTF8().data());
	return true;
}

Tab* GUI_App::get_tab(Preset::Type type)
{
    for (Tab* tab: tabs_list)
        if (tab->type() == type)
            return tab->completed() ? tab : nullptr; // To avoid actions with no-completed Tab
    return nullptr;
}

ConfigOptionMode GUI_App::get_mode()
{
    if (!app_config->has("view_mode"))
        return comSimple;

    const auto mode = app_config->get("view_mode");
    return mode == "expert" ? comExpert : 
           mode == "simple" ? comSimple : comAdvanced;
}

void GUI_App::save_mode(const /*ConfigOptionMode*/int mode) 
{
    const std::string mode_str = mode == comExpert ? "expert" :
                                 mode == comSimple ? "simple" : "advanced";
    app_config->set("view_mode", mode_str);
    app_config->save(); 
    update_mode();
}

// Update view mode according to selected menu
void GUI_App::update_mode()
{
    sidebar().update_mode();

    for (auto tab : tabs_list)
        tab->update_mode();

    plater()->update_object_menu();
    plater()->canvas3D()->update_gizmos_on_off_state();
}

void GUI_App::add_config_menu(wxMenuBar *menu)
{
    auto local_menu = new wxMenu();
    wxWindowID config_id_base = wxWindow::NewControlId(int(ConfigMenuCnt));

    const auto config_wizard_name = _(ConfigWizard::name(true));
    const auto config_wizard_tooltip = from_u8((boost::format(_utf8(L("Run %s"))) % config_wizard_name).str());
    // Cmd+, is standard on OS X - what about other operating systems?
    if (is_editor()) {
        local_menu->Append(config_id_base + ConfigMenuWizard, config_wizard_name + dots, config_wizard_tooltip);
        local_menu->Append(config_id_base + ConfigMenuSnapshots, _L("&Configuration Snapshots") + dots, _L("Inspect / activate configuration snapshots"));
        local_menu->Append(config_id_base + ConfigMenuTakeSnapshot, _L("Take Configuration &Snapshot"), _L("Capture a configuration snapshot"));
        local_menu->Append(config_id_base + ConfigMenuUpdate, _L("Check for updates"), _L("Check for configuration updates"));
        local_menu->AppendSeparator();
    }
    local_menu->Append(config_id_base + ConfigMenuPreferences, _L("&Preferences") + dots +
#ifdef __APPLE__
        "\tCtrl+,",
#else
        "\tCtrl+P",
#endif
        _L("Application preferences"));
    wxMenu* mode_menu = nullptr;
    if (is_editor()) {
        local_menu->AppendSeparator();
        mode_menu = new wxMenu();
        mode_menu->AppendRadioItem(config_id_base + ConfigMenuModeSimple, _L("Simple"), _L("Simple View Mode"));
//    mode_menu->AppendRadioItem(config_id_base + ConfigMenuModeAdvanced, _L("Advanced"), _L("Advanced View Mode"));
        mode_menu->AppendRadioItem(config_id_base + ConfigMenuModeAdvanced, _CTX(L_CONTEXT("Advanced", "Mode"), "Mode"), _L("Advanced View Mode"));
        mode_menu->AppendRadioItem(config_id_base + ConfigMenuModeExpert, _L("Expert"), _L("Expert View Mode"));
        Bind(wxEVT_UPDATE_UI, [this](wxUpdateUIEvent& evt) { if (get_mode() == comSimple) evt.Check(true); }, config_id_base + ConfigMenuModeSimple);
        Bind(wxEVT_UPDATE_UI, [this](wxUpdateUIEvent& evt) { if (get_mode() == comAdvanced) evt.Check(true); }, config_id_base + ConfigMenuModeAdvanced);
        Bind(wxEVT_UPDATE_UI, [this](wxUpdateUIEvent& evt) { if (get_mode() == comExpert) evt.Check(true); }, config_id_base + ConfigMenuModeExpert);

        local_menu->AppendSubMenu(mode_menu, _L("Mode"), wxString::Format(_L("%s View Mode"), SLIC3R_APP_NAME));
    }
    local_menu->AppendSeparator();
    local_menu->Append(config_id_base + ConfigMenuLanguage, _L("&Language"));
    if (is_editor()) {
        local_menu->AppendSeparator();
        local_menu->Append(config_id_base + ConfigMenuFlashFirmware, _L("Flash printer &firmware"), _L("Upload a firmware image into an Arduino based printer"));
        // TODO: for when we're able to flash dictionaries
        // local_menu->Append(config_id_base + FirmwareMenuDict,  _L("Flash language file"),    _L("Upload a language dictionary file into a Prusa printer"));
    }

    local_menu->Bind(wxEVT_MENU, [this, config_id_base](wxEvent &event) {
        switch (event.GetId() - config_id_base) {
        case ConfigMenuWizard:
            run_wizard(ConfigWizard::RR_USER);
            break;
		case ConfigMenuUpdate:
			check_updates(true);
			break;
        case ConfigMenuTakeSnapshot:
            // Take a configuration snapshot.
            if (check_unsaved_changes()) {
                wxTextEntryDialog dlg(nullptr, _L("Taking configuration snapshot"), _L("Snapshot name"));
                
                // set current normal font for dialog children, 
                // because of just dlg.SetFont(normal_font()) has no result;
                for (auto child : dlg.GetChildren())
                    child->SetFont(normal_font());

                if (dlg.ShowModal() == wxID_OK)
                    app_config->set("on_snapshot",
                    Slic3r::GUI::Config::SnapshotDB::singleton().take_snapshot(
                    *app_config, Slic3r::GUI::Config::Snapshot::SNAPSHOT_USER, dlg.GetValue().ToUTF8().data()).id);
            }
            break;
        case ConfigMenuSnapshots:
            if (check_unsaved_changes()) {
                std::string on_snapshot;
                if (Config::SnapshotDB::singleton().is_on_snapshot(*app_config))
                    on_snapshot = app_config->get("on_snapshot");
                ConfigSnapshotDialog dlg(Slic3r::GUI::Config::SnapshotDB::singleton(), on_snapshot);
                dlg.ShowModal();
                if (!dlg.snapshot_to_activate().empty()) {
                    if (!Config::SnapshotDB::singleton().is_on_snapshot(*app_config))
                        Config::SnapshotDB::singleton().take_snapshot(*app_config, Config::Snapshot::SNAPSHOT_BEFORE_ROLLBACK);
                    app_config->set("on_snapshot",
                        Config::SnapshotDB::singleton().restore_snapshot(dlg.snapshot_to_activate(), *app_config).id);
                    preset_bundle->load_presets(*app_config);
                    // Load the currently selected preset into the GUI, update the preset selection box.
                    load_current_presets();
                }
            }
            break;
        case ConfigMenuPreferences:
        {
            bool app_layout_changed = false;
            {
                // the dialog needs to be destroyed before the call to recreate_GUI()
                // or sometimes the application crashes into wxDialogBase() destructor
                // so we put it into an inner scope
                PreferencesDialog dlg(mainframe);
                dlg.ShowModal();
                app_layout_changed = dlg.settings_layout_changed();
                if (dlg.seq_top_layer_only_changed())
                    this->plater_->refresh_print();
#if ENABLE_CUSTOMIZABLE_FILES_ASSOCIATION_ON_WIN
#ifdef _WIN32
                if (is_editor()) {
                    if (app_config->get("associate_3mf") == "1")
                        associate_3mf_files();
                    if (app_config->get("associate_stl") == "1")
                        associate_stl_files();
                }
                else {
                    if (app_config->get("associate_gcode") == "1")
                        associate_gcode_files();
                }
#endif // _WIN32
#endif // ENABLE_CUSTOMIZABLE_FILES_ASSOCIATION_ON_WIN
            }
            if (app_layout_changed) {
                // hide full main_sizer for mainFrame
                mainframe->GetSizer()->Show(false);
                mainframe->update_layout();
                mainframe->select_tab(size_t(0));
            }
            break;
        }
        case ConfigMenuLanguage:
        {
            /* Before change application language, let's check unsaved changes on 3D-Scene
             * and draw user's attention to the application restarting after a language change
             */
            {
                // the dialog needs to be destroyed before the call to switch_language()
                // or sometimes the application crashes into wxDialogBase() destructor
                // so we put it into an inner scope
                wxString title = is_editor() ? wxString(SLIC3R_APP_NAME) : wxString(GCODEVIEWER_APP_NAME);
                title += " - " + _L("Language selection");
                wxMessageDialog dialog(nullptr,
                    _L("Switching the language will trigger application restart.\n"
                        "You will lose content of the plater.") + "\n\n" +
                    _L("Do you want to proceed?"),
                    title,
                    wxICON_QUESTION | wxOK | wxCANCEL);
                if (dialog.ShowModal() == wxID_CANCEL)
                    return;
            }

            switch_language();
            break;
        }
        case ConfigMenuFlashFirmware:
            FirmwareDialog::run(mainframe);
            break;
        default:
            break;
        }
    });
    
    using std::placeholders::_1;

    if (mode_menu != nullptr) {
        auto modfn = [this](int mode, wxCommandEvent&) { if (get_mode() != mode) save_mode(mode); };
        mode_menu->Bind(wxEVT_MENU, std::bind(modfn, comSimple, _1), config_id_base + ConfigMenuModeSimple);
        mode_menu->Bind(wxEVT_MENU, std::bind(modfn, comAdvanced, _1), config_id_base + ConfigMenuModeAdvanced);
        mode_menu->Bind(wxEVT_MENU, std::bind(modfn, comExpert, _1), config_id_base + ConfigMenuModeExpert);
    }

    menu->Append(local_menu, _L("&Configuration"));
}

// This is called when closing the application, when loading a config file or when starting the config wizard
// to notify the user whether he is aware that some preset changes will be lost.
bool GUI_App::check_unsaved_changes(const wxString &header)
{
    PrinterTechnology printer_technology = preset_bundle->printers.get_edited_preset().printer_technology();

    bool has_unsaved_changes = false;
    for (Tab* tab : tabs_list)
        if (tab->supports_printer_technology(printer_technology) && tab->current_preset_is_dirty()) {
            has_unsaved_changes = true;
            break;
        }

    if (has_unsaved_changes)
    {
        UnsavedChangesDialog dlg(header);
        if (wxGetApp().app_config->get("default_action_on_close_application") == "none" && dlg.ShowModal() == wxID_CANCEL)
            return false;

        if (dlg.save_preset())  // save selected changes
        {
            for (const std::pair<std::string, Preset::Type>& nt : dlg.get_names_and_types())
                preset_bundle->save_changes_for_preset(nt.first, nt.second, dlg.get_unselected_options(nt.second));

            // if we saved changes to the new presets, we should to 
            // synchronize config.ini with the current selections.
            preset_bundle->export_selections(*app_config);

            wxMessageBox(_L("The preset(s) modifications are successfully saved"));
        }
    }

    return true;
}

bool GUI_App::checked_tab(Tab* tab)
{
    bool ret = true;
    if (find(tabs_list.begin(), tabs_list.end(), tab) == tabs_list.end())
        ret = false;
    return ret;
}

// Update UI / Tabs to reflect changes in the currently loaded presets
void GUI_App::load_current_presets()
{
    // check printer_presets for the containing information about "Print Host upload"
    // and create physical printer from it, if any exists
    check_printer_presets();

    PrinterTechnology printer_technology = preset_bundle->printers.get_edited_preset().printer_technology();
	this->plater()->set_printer_technology(printer_technology);
    for (Tab *tab : tabs_list)
		if (tab->supports_printer_technology(printer_technology)) {
			if (tab->type() == Preset::TYPE_PRINTER) {
				static_cast<TabPrinter*>(tab)->update_pages();
				// Mark the plater to update print bed by tab->load_current_preset() from Plater::on_config_change().
				this->plater()->force_print_bed_update();
			}
			tab->load_current_preset();
		}
}

bool GUI_App::OnExceptionInMainLoop()
{
    generic_exception_handle();
    return false;
}

#ifdef __APPLE__
// This callback is called from wxEntry()->wxApp::CallOnInit()->NSApplication run
// that is, before GUI_App::OnInit(), so we have a chance to switch GUI_App
// to a G-code viewer.
void GUI_App::OSXStoreOpenFiles(const wxArrayString &fileNames)
{
    size_t num_gcodes = 0;
    for (const wxString &filename : fileNames) {
        wxString fn = filename.Upper();
        if (fn.EndsWith(".G") || fn.EndsWith(".GCODE"))
            ++ num_gcodes;
    }
    if (fileNames.size() == num_gcodes) {
        // Opening PrusaSlicer by drag & dropping a G-Code onto PrusaSlicer icon in Finder,
        // just G-codes were passed. Switch to G-code viewer mode.
        m_app_mode = EAppMode::GCodeViewer;
        unlock_lockfile(get_instance_hash_string() + ".lock", data_dir() + "/cache/");
        if(app_config != nullptr)
            delete app_config;
        app_config = nullptr;
        init_app_config();
    }
    wxApp::OSXStoreOpenFiles(fileNames);
}
// wxWidgets override to get an event on open files.
void GUI_App::MacOpenFiles(const wxArrayString &fileNames)
{
    std::vector<std::string> files;
    std::vector<wxString>    gcode_files;
    std::vector<wxString>    non_gcode_files;
    for (const auto& filename : fileNames) {
        wxString fn = filename.Upper();
        if (fn.EndsWith(".G") || fn.EndsWith(".GCODE"))
            gcode_files.emplace_back(filename);
        else {
            files.emplace_back(into_u8(filename));
            non_gcode_files.emplace_back(filename);
        }
    }
    if (m_app_mode == EAppMode::GCodeViewer) {
        // Running in G-code viewer.
        // Load the first G-code into the G-code viewer.
        // Or if no G-codes, send other files to slicer. 
        if (! gcode_files.empty())
            this->plater()->load_gcode(gcode_files.front());
        if (!non_gcode_files.empty()) 
            start_new_slicer(non_gcode_files, true);
    } else {
        if (! files.empty()) {
#if ENABLE_DRAG_AND_DROP_FIX
            wxArrayString input_files;
            for (size_t i = 0; i < non_gcode_files.size(); ++i) {
                input_files.push_back(non_gcode_files[i]);
            }
            this->plater()->load_files(input_files);
#else
            this->plater()->load_files(files, true, true);
#endif     
        }
        for (const wxString &filename : gcode_files)
            start_new_gcodeviewer(&filename);
    }
}
#endif /* __APPLE */

Sidebar& GUI_App::sidebar()
{
    return plater_->sidebar();
}

ObjectManipulation* GUI_App::obj_manipul()
{
    // If this method is called before plater_ has been initialized, return nullptr (to avoid a crash)
    return (plater_ != nullptr) ? sidebar().obj_manipul() : nullptr;
}

ObjectSettings* GUI_App::obj_settings()
{
    return sidebar().obj_settings();
}

ObjectList* GUI_App::obj_list()
{
    return sidebar().obj_list();
}

ObjectLayers* GUI_App::obj_layers()
{
    return sidebar().obj_layers();
}

Plater* GUI_App::plater()
{
    return plater_;
}

Model& GUI_App::model()
{
    return plater_->model();
}

wxNotebook* GUI_App::tab_panel() const
{
    return mainframe->m_tabpanel;
}

// extruders count from selected printer preset
int GUI_App::extruders_cnt() const
{
    const Preset& preset = preset_bundle->printers.get_selected_preset();
    return preset.printer_technology() == ptSLA ? 1 :
           preset.config.option<ConfigOptionFloats>("nozzle_diameter")->values.size();
}

// extruders count from edited printer preset
int GUI_App::extruders_edited_cnt() const
{
    const Preset& preset = preset_bundle->printers.get_edited_preset();
    return preset.printer_technology() == ptSLA ? 1 :
           preset.config.option<ConfigOptionFloats>("nozzle_diameter")->values.size();
}

wxString GUI_App::current_language_code_safe() const
{
	// Translate the language code to a code, for which Prusa Research maintains translations.
	const std::map<wxString, wxString> mapping {
		{ "cs", 	"cs_CZ", },
		{ "sk", 	"cs_CZ", },
		{ "de", 	"de_DE", },
		{ "es", 	"es_ES", },
		{ "fr", 	"fr_FR", },
		{ "it", 	"it_IT", },
		{ "ja", 	"ja_JP", },
		{ "ko", 	"ko_KR", },
		{ "pl", 	"pl_PL", },
		{ "uk", 	"uk_UA", },
		{ "zh", 	"zh_CN", },
	};
	wxString language_code = this->current_language_code().BeforeFirst('_');
	auto it = mapping.find(language_code);
	if (it != mapping.end())
		language_code = it->second;
	else
		language_code = "en_US";
	return language_code;
}

void GUI_App::open_web_page_localized(const std::string &http_address)
{
    wxLaunchDefaultBrowser(http_address + "&lng=" + this->current_language_code_safe());
}

bool GUI_App::run_wizard(ConfigWizard::RunReason reason, ConfigWizard::StartPage start_page)
{
    wxCHECK_MSG(mainframe != nullptr, false, "Internal error: Main frame not created / null");

    if (! m_wizard) {
        m_wizard = new ConfigWizard(mainframe);
    }

    const bool res = m_wizard->run(reason, start_page);

    if (res) {
        load_current_presets();

        if (preset_bundle->printers.get_edited_preset().printer_technology() == ptSLA
            && Slic3r::model_has_multi_part_objects(wxGetApp().model())) {
            GUI::show_info(nullptr,
                _L("It's impossible to print multi-part object(s) with SLA technology.") + "\n\n" +
                _L("Please check and fix your object list."),
                _L("Attention!"));
        }
    }

    return res;
}

#if ENABLE_THUMBNAIL_GENERATOR_DEBUG
void GUI_App::gcode_thumbnails_debug()
{
    const std::string BEGIN_MASK = "; thumbnail begin";
    const std::string END_MASK = "; thumbnail end";
    std::string gcode_line;
    bool reading_image = false;
    unsigned int width = 0;
    unsigned int height = 0;

    wxFileDialog dialog(GetTopWindow(), _L("Select a gcode file:"), "", "", "G-code files (*.gcode)|*.gcode;*.GCODE;", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dialog.ShowModal() != wxID_OK)
        return;

    std::string in_filename = into_u8(dialog.GetPath());
    std::string out_path = boost::filesystem::path(in_filename).remove_filename().append(L"thumbnail").string();

    boost::nowide::ifstream in_file(in_filename.c_str());
    std::vector<std::string> rows;
    std::string row;
    if (in_file.good())
    {
        while (std::getline(in_file, gcode_line))
        {
            if (in_file.good())
            {
                if (boost::starts_with(gcode_line, BEGIN_MASK))
                {
                    reading_image = true;
                    gcode_line = gcode_line.substr(BEGIN_MASK.length() + 1);
                    std::string::size_type x_pos = gcode_line.find('x');
                    std::string width_str = gcode_line.substr(0, x_pos);
                    width = (unsigned int)::atoi(width_str.c_str());
                    std::string height_str = gcode_line.substr(x_pos + 1);
                    height = (unsigned int)::atoi(height_str.c_str());
                    row.clear();
                }
                else if (reading_image && boost::starts_with(gcode_line, END_MASK))
                {
                    std::string out_filename = out_path + std::to_string(width) + "x" + std::to_string(height) + ".png";
                    boost::nowide::ofstream out_file(out_filename.c_str(), std::ios::binary);
                    if (out_file.good())
                    {
                        std::string decoded;
                        decoded.resize(boost::beast::detail::base64::decoded_size(row.size()));
                        decoded.resize(boost::beast::detail::base64::decode((void*)&decoded[0], row.data(), row.size()).first);

                        out_file.write(decoded.c_str(), decoded.size());
                        out_file.close();
                    }

                    reading_image = false;
                    width = 0;
                    height = 0;
                    rows.clear();
                }
                else if (reading_image)
                    row += gcode_line.substr(2);
            }
        }

        in_file.close();
    }
}
#endif // ENABLE_THUMBNAIL_GENERATOR_DEBUG

void GUI_App::window_pos_save(wxTopLevelWindow* window, const std::string &name)
{
    if (name.empty()) { return; }
    const auto config_key = (boost::format("window_%1%") % name).str();

    WindowMetrics metrics = WindowMetrics::from_window(window);
    app_config->set(config_key, metrics.serialize());
    app_config->save();
}

void GUI_App::window_pos_restore(wxTopLevelWindow* window, const std::string &name, bool default_maximized)
{
    if (name.empty()) { return; }
    const auto config_key = (boost::format("window_%1%") % name).str();

    if (! app_config->has(config_key)) {
        window->Maximize(default_maximized);
        return;
    }

    auto metrics = WindowMetrics::deserialize(app_config->get(config_key));
    if (! metrics) {
        window->Maximize(default_maximized);
        return;
    }

    const wxRect& rect = metrics->get_rect();
    window->SetPosition(rect.GetPosition());
    window->SetSize(rect.GetSize());
    window->Maximize(metrics->get_maximized());
}

void GUI_App::window_pos_sanitize(wxTopLevelWindow* window)
{
    /*unsigned*/int display_idx = wxDisplay::GetFromWindow(window);
    wxRect display;
    if (display_idx == wxNOT_FOUND) {
        display = wxDisplay(0u).GetClientArea();
        window->Move(display.GetTopLeft());
    } else {
        display = wxDisplay(display_idx).GetClientArea();
    }

    auto metrics = WindowMetrics::from_window(window);
    metrics.sanitize_for_display(display);
    if (window->GetScreenRect() != metrics.get_rect()) {
        window->SetSize(metrics.get_rect());
    }
}

bool GUI_App::config_wizard_startup()
{
    if (!m_app_conf_exists || preset_bundle->printers.size() <= 1) {
        run_wizard(ConfigWizard::RR_DATA_EMPTY);
        return true;
    } else if (get_app_config()->legacy_datadir()) {
        // Looks like user has legacy pre-vendorbundle data directory,
        // explain what this is and run the wizard

        MsgDataLegacy dlg;
        dlg.ShowModal();

        run_wizard(ConfigWizard::RR_DATA_LEGACY);
        return true;
    }
    return false;
}

void GUI_App::check_updates(const bool verbose)
{	
	PresetUpdater::UpdateResult updater_result;
	try {
		updater_result = preset_updater->config_update(app_config->orig_version(), verbose);
		if (updater_result == PresetUpdater::R_INCOMPAT_EXIT) {
			mainframe->Close();
		}
		else if (updater_result == PresetUpdater::R_INCOMPAT_CONFIGURED) {
            m_app_conf_exists = true;
		}
		else if (verbose && updater_result == PresetUpdater::R_NOOP) {
			MsgNoUpdates dlg;
			dlg.ShowModal();
		}
	}
	catch (const std::exception & ex) {
		show_error(nullptr, ex.what());
	}
}

// static method accepting a wxWindow object as first parameter
// void warning_catcher{
//     my($self, $message_dialog) = @_;
//     return sub{
//         my $message = shift;
//         return if $message = ~/ GLUquadricObjPtr | Attempt to free unreferenced scalar / ;
//         my @params = ($message, 'Warning', wxOK | wxICON_WARNING);
//         $message_dialog
//             ? $message_dialog->(@params)
//             : Wx::MessageDialog->new($self, @params)->ShowModal;
//     };
// }

// Do we need this function???
// void GUI_App::notify(message) {
//     auto frame = GetTopWindow();
//     // try harder to attract user attention on OS X
//     if (!frame->IsActive())
//         frame->RequestUserAttention(defined(__WXOSX__/*&Wx::wxMAC */)? wxUSER_ATTENTION_ERROR : wxUSER_ATTENTION_INFO);
// 
//     // There used to be notifier using a Growl application for OSX, but Growl is dead.
//     // The notifier also supported the Linux X D - bus notifications, but that support was broken.
//     //TODO use wxNotificationMessage ?
// }


#ifdef __WXMSW__
static bool set_into_win_registry(HKEY hkeyHive, const wchar_t* pszVar, const wchar_t* pszValue)
{
    // see as reference: https://stackoverflow.com/questions/20245262/c-program-needs-an-file-association
    wchar_t szValueCurrent[1000];
    DWORD dwType;
    DWORD dwSize = sizeof(szValueCurrent);

    int iRC = ::RegGetValueW(hkeyHive, pszVar, nullptr, RRF_RT_ANY, &dwType, szValueCurrent, &dwSize);

    bool bDidntExist = iRC == ERROR_FILE_NOT_FOUND;

    if ((iRC != ERROR_SUCCESS) && !bDidntExist)
        // an error occurred
        return false;

    if (!bDidntExist) {
        if (dwType != REG_SZ)
            // invalid type
            return false;

        if (::wcscmp(szValueCurrent, pszValue) == 0)
            // value already set
            return false;
    }

    DWORD dwDisposition;
    HKEY hkey;
    iRC = ::RegCreateKeyExW(hkeyHive, pszVar, 0, 0, 0, KEY_ALL_ACCESS, nullptr, &hkey, &dwDisposition);
    bool ret = false;
    if (iRC == ERROR_SUCCESS) {
        iRC = ::RegSetValueExW(hkey, L"", 0, REG_SZ, (BYTE*)pszValue, (::wcslen(pszValue) + 1) * sizeof(wchar_t));
        if (iRC == ERROR_SUCCESS)
            ret = true;
    }

    RegCloseKey(hkey);
    return ret;
}

void GUI_App::associate_3mf_files()
{
    wchar_t app_path[MAX_PATH];
    ::GetModuleFileNameW(nullptr, app_path, sizeof(app_path));

    std::wstring prog_path = L"\"" + std::wstring(app_path) + L"\"";
    std::wstring prog_id = L"Prusa.Slicer.1";
    std::wstring prog_desc = L"PrusaSlicer";
    std::wstring prog_command = prog_path + L" \"%1\"";
    std::wstring reg_base = L"Software\\Classes";
    std::wstring reg_extension = reg_base + L"\\.3mf";
    std::wstring reg_prog_id = reg_base + L"\\" + prog_id;
    std::wstring reg_prog_id_command = reg_prog_id + L"\\Shell\\Open\\Command";

    bool is_new = false;
    is_new |= set_into_win_registry(HKEY_CURRENT_USER, reg_extension.c_str(), prog_id.c_str());
    is_new |= set_into_win_registry(HKEY_CURRENT_USER, reg_prog_id.c_str(), prog_desc.c_str());
    is_new |= set_into_win_registry(HKEY_CURRENT_USER, reg_prog_id_command.c_str(), prog_command.c_str());

    if (is_new)
        // notify Windows only when any of the values gets changed
        ::SHChangeNotify(SHCNE_ASSOCCHANGED, SHCNF_IDLIST, nullptr, nullptr);
}

#if ENABLE_CUSTOMIZABLE_FILES_ASSOCIATION_ON_WIN
void GUI_App::associate_stl_files()
{
    wchar_t app_path[MAX_PATH];
    ::GetModuleFileNameW(nullptr, app_path, sizeof(app_path));

    std::wstring prog_path = L"\"" + std::wstring(app_path) + L"\"";
    std::wstring prog_id = L"Prusa.Slicer.1";
    std::wstring prog_desc = L"PrusaSlicer";
    std::wstring prog_command = prog_path + L" \"%1\"";
    std::wstring reg_base = L"Software\\Classes";
    std::wstring reg_extension = reg_base + L"\\.stl";
    std::wstring reg_prog_id = reg_base + L"\\" + prog_id;
    std::wstring reg_prog_id_command = reg_prog_id + L"\\Shell\\Open\\Command";

    bool is_new = false;
    is_new |= set_into_win_registry(HKEY_CURRENT_USER, reg_extension.c_str(), prog_id.c_str());
    is_new |= set_into_win_registry(HKEY_CURRENT_USER, reg_prog_id.c_str(), prog_desc.c_str());
    is_new |= set_into_win_registry(HKEY_CURRENT_USER, reg_prog_id_command.c_str(), prog_command.c_str());

    if (is_new)
        // notify Windows only when any of the values gets changed
        ::SHChangeNotify(SHCNE_ASSOCCHANGED, SHCNF_IDLIST, nullptr, nullptr);
}
#endif // ENABLE_CUSTOMIZABLE_FILES_ASSOCIATION_ON_WIN

void GUI_App::associate_gcode_files()
{
    wchar_t app_path[MAX_PATH];
    ::GetModuleFileNameW(nullptr, app_path, sizeof(app_path));

    std::wstring prog_path = L"\"" + std::wstring(app_path) + L"\"";
    std::wstring prog_id = L"PrusaSlicer.GCodeViewer.1";
    std::wstring prog_desc = L"PrusaSlicerGCodeViewer";
    std::wstring prog_command = prog_path + L" \"%1\"";
    std::wstring reg_base = L"Software\\Classes";
    std::wstring reg_extension = reg_base + L"\\.gcode";
    std::wstring reg_prog_id = reg_base + L"\\" + prog_id;
    std::wstring reg_prog_id_command = reg_prog_id + L"\\Shell\\Open\\Command";

    bool is_new = false;
    is_new |= set_into_win_registry(HKEY_CURRENT_USER, reg_extension.c_str(), prog_id.c_str());
    is_new |= set_into_win_registry(HKEY_CURRENT_USER, reg_prog_id.c_str(), prog_desc.c_str());
    is_new |= set_into_win_registry(HKEY_CURRENT_USER, reg_prog_id_command.c_str(), prog_command.c_str());

    if (is_new)
        // notify Windows only when any of the values gets changed
        ::SHChangeNotify(SHCNE_ASSOCCHANGED, SHCNF_IDLIST, nullptr, nullptr);
}
#endif // __WXMSW__

} // GUI
} //Slic3r
