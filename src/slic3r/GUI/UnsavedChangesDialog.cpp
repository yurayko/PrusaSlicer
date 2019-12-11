#include "UnsavedChangesDialog.hpp"
#include "BitmapCache.hpp"
#include "wxExtensions.hpp"
#include "slic3r/Utils/Diff.hpp"
#include <boost/range/algorithm.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <wx/clrpicker.h>
#include <wx/numformatter.h>
#include <wx/statline.h>
#include <wx/html/htmlwin.h>
#include <wx/generic/stattextg.h>
#include <sstream>
#include <string>

#define Dialog_max_width 1200
#define Dialog_max_height 800

#define Dialog_min_width 500
#define Dialog_min_height 350

#define Dialog_def_border 6
#define Dialog_child_indentation 20

namespace Slic3r {
	namespace GUI {
		typedef UnsavedChangesDialog UCD;

		void UCD::dirty_opt_entry::setWinEnabled(wxWindow* win, Gui_Type _gui_type, bool enabled) {
			if (win != nullptr) {
				if (_gui_type == Gui_Type::Diff) {
					const wxSizerItemList& children = win->GetSizer()->GetChildren();
					wxSizerItemList::const_iterator it_wins;

					for (it_wins = children.begin(); it_wins != children.end(); it_wins++) {
						(*it_wins)->GetWindow()->Enable(enabled);
					}
				}
				else {
					win->Enable(enabled);
				}

				win->Refresh();
			}
		}

		void UCD::dirty_opt_entry::setValWinsEnabled(bool enabled) {
			setWinEnabled(this->old_win, this->old_win_type, enabled);
			setWinEnabled(this->new_win, this->new_win_type, enabled);
		}

		void UCD::dirty_opt_entry::set_checkbox(bool checked) {
			this->checkbox->SetValue(checked);

			this->checkbox->SetForegroundColour(checked ?
				wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT) :
				wxSystemSettings::GetColour(wxSYS_COLOUR_GRAYTEXT));

			this->setValWinsEnabled(checked);
		}

		void UCD::dirty_opt_entry::on_checkbox_toggled() {
			bool checked = this->checkbox->GetValue();
			this->set_checkbox(checked);
			this->parent->on_child_checkbox_toggled(checked);
		}

		void UCD::dirty_opt_entry::on_parent_checkbox_toggled(bool checked) {
			if (checked != this->checkbox->GetValue()) {
				this->set_checkbox(checked);
			}
		}

		bool UCD::dirty_opt_entry::saveMe() {
			return this->checkbox->GetValue();
		}

		int UCD::dirty_opt_entry::get_checkbox_width_with_indent() {
			return this->checkbox->GetEffectiveMinSize().GetWidth() +
				get_wxSizerItem_border_size(this->parent_sizer->GetItem(this->checkbox)).GetWidth();
		}

		UCD::dirty_opt::Type UCD::dirty_opt_entry::type() {
			return this->val.type;
		}

		void UCD::dirty_opts_node::on_checkbox_toggled() {
			bool checked = this->checkbox->GetValue();
			this->set_checkbox(checked);

			if (this->parent != nullptr) {
				this->parent->on_child_checkbox_toggled(checked);
			}
			for (UCD::dirty_opts_node* cur_node : this->childs) {
				cur_node->on_parent_checkbox_toggled(checked);
			}
			for (UCD::dirty_opt_entry* cur_opt_entry : this->opts) {
				cur_opt_entry->on_parent_checkbox_toggled(checked);
			}
		}

		void UCD::dirty_opts_node::on_child_checkbox_toggled(bool checked) {
			if (this->checkbox != nullptr) {
				if (checked && !this->checkbox->GetValue()) {
					this->set_checkbox(true);

					if (this->parent != nullptr) {
						this->parent->on_child_checkbox_toggled(true);
					}
				}
				if (!checked && this->checkbox->GetValue()) {
					bool checked_child = false;
					for (UCD::dirty_opts_node* cur_node : this->childs) {
						if (cur_node->checkbox->GetValue()) {
							checked_child = true;
							break;
						}
					}
					for (size_t i = 0; i < this->opts.size() && !checked_child; i++) {
						if (this->opts[i]->checkbox->GetValue()) {
							checked_child = true;
						}
					}

					if (!checked_child) {
						this->set_checkbox(false);
						if (this->parent != nullptr) {
							this->parent->on_child_checkbox_toggled(false);
						}
					}
				}
			}
		}

		void UCD::dirty_opts_node::on_parent_checkbox_toggled(bool checked) {
			if (checked != this->checkbox->GetValue()) {
				this->set_checkbox(checked);

				for (UCD::dirty_opts_node* cur_node : this->childs) {
					cur_node->on_parent_checkbox_toggled(checked);
				}
				for (UCD::dirty_opt_entry* cur_opt_entry : this->opts) {
					cur_opt_entry->on_parent_checkbox_toggled(checked);
				}
			}
		}

		void UCD::dirty_opts_node::set_checkbox(bool checked) {
			if (this->checkbox != nullptr) {
				this->checkbox->SetValue(checked);

				this->checkbox->SetForegroundColour(checked ?
					wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT) :
					wxSystemSettings::GetColour(wxSYS_COLOUR_GRAYTEXT));

				if (this->icon != nullptr) {
					this->icon->Enable(checked);
				}
				if (this->labelCtrl != nullptr) {
					this->labelCtrl->Enable(checked);
				}
			}
		}

		void UCD::dirty_opts_node::selectChilds(bool selected) {
			for (UCD::dirty_opts_node* cur_node : this->childs) {
				cur_node->set_checkbox(selected);
				cur_node->selectChilds(selected);
			}

			for (UCD::dirty_opt_entry* cur_opt : this->opts) {
				cur_opt->set_checkbox(selected);
			}
		}

		void UCD::dirty_opts_node::getAllOptionEntries(std::vector<UCD::dirty_opt_entry*>& _opts, bool only_opts_to_restore, UCD::dirty_opt::Type type) {
			for (UCD::dirty_opts_node* cur_node : this->childs) {
				cur_node->getAllOptionEntries(_opts, only_opts_to_restore, type);
			}

			for (UCD::dirty_opt_entry* cur_opt : this->opts) {
				if ((!only_opts_to_restore || !cur_opt->saveMe()) && (type == UCD::dirty_opt::Type::Nil || cur_opt->val.type == type))
					_opts.push_back(cur_opt);
			}
		}

		bool UCD::dirty_opts_node::hasAnythingToSave() {
			for (UCD::dirty_opts_node* cur_node : this->childs) {
				if (cur_node->hasAnythingToSave()) {
					return true;
				}
			}

			for (UCD::dirty_opt_entry* cur_opt : this->opts) {
				if (cur_opt->saveMe()) {
					return true;
				}
			}

			return false;
		}

		UCD::dirty_opts_node* UCD::dirty_opts_node::getTabNode(Tab* tab) {
			for (UCD::dirty_opts_node* cur_node : this->childs) {
				if (cur_node->tab == tab) {
					return cur_node;
				}
			}

			return nullptr;
		}

		int UCD::dirty_opts_node::get_label_width_with_indent() {
			int w = 0;

			for (wxWindow* cur_win : std::vector<wxWindow*>{ checkbox, icon, labelCtrl }) {
				if (cur_win != nullptr) {
					w += cur_win->GetEffectiveMinSize().GetWidth();
					w += get_wxSizerItem_border_size(parent_sizer->GetItem(cur_win)).GetWidth();
				}
			}

			return w;
		}

		int UCD::dirty_opts_node::get_max_child_label_width_with_indent() {
			int w = 0;

			for (UCD::dirty_opts_node* cur_node : this->childs) {
				w = std::max(w, cur_node->get_label_width_with_indent());
				w = std::max(w, cur_node->get_max_child_label_width_with_indent());
			}

			for (UCD::dirty_opt_entry* cur_opt : this->opts) {
				w = std::max(w, cur_opt->get_checkbox_width_with_indent());
			}

			return w;
		}

/*----------------------------------------------------------------------------------*/

		UnsavedChangesDialog::UnsavedChangesDialog(wxWindow* parent, GUI_App* app, const wxString& header, const wxString& caption, long style, const wxPoint& pos)
			: wxDialog(parent, -1, caption, pos, wxSize(Dialog_min_width, Dialog_min_height), wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER)
		{
			m_print_tech = app->preset_bundle->printers.get_edited_preset().printer_technology();

			for (Tab* cur_tab : app->tabs_list) {
				if (cur_tab->supports_printer_technology(m_print_tech) && cur_tab->current_preset_is_dirty()) {
					m_tabs.emplace_back(cur_tab);
				}
			}
			m_external_header_str = header;
			build();
		}

		UnsavedChangesDialog::UnsavedChangesDialog(wxWindow* parent, Tab* tab, const wxString& header, const wxString& caption, long style, const wxPoint& pos)
			: wxDialog(parent, -1, caption, pos, wxSize(Dialog_min_width, Dialog_min_height), wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER)
		{
			m_print_tech = PrinterTechnology::ptUnknown;
			m_tabs = { tab };
			m_external_header_str = header;
			build();
		}

		UnsavedChangesDialog::~UnsavedChangesDialog() {
			delete this->m_dirty_tabs_tree;
		}

		void UnsavedChangesDialog::build() {
			SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));

			wxBoxSizer* main_sizer = new wxBoxSizer(wxVERTICAL);

			wxString dirty_tabs;
			buildScrollWindow(dirty_tabs);
			
			wxFont font = GUI::wxGetApp().normal_font();
			font.SetPointSize(10);
			m_header = new wxStaticText(this, wxID_ANY, get_header_msg(dirty_tabs), wxDefaultPosition, wxDefaultSize);
			m_header->SetFont(font);

			main_sizer->Add(m_header, 0, wxALL, Dialog_def_border);
			main_sizer->Add(-1, Dialog_def_border);
			main_sizer->Add(m_scroller_container, 1, wxEXPAND | wxALL, Dialog_def_border);
			main_sizer->Add(buildYesNoBtns(), 0, wxEXPAND | wxTOP, Dialog_def_border * 2);
			SetSizer(main_sizer);
			setCorrectSize();
			this->Center();
		}

		void UnsavedChangesDialog::setCorrectSize() {
			this->SetMinSize(wxSize(Dialog_min_width, Dialog_min_height));
			//this->SetMaxSize(wxSize(Dialog_max_width, Dialog_max_height)); 
			this->SetSize(wxSize(Dialog_min_width, Dialog_min_height));
			this->Layout();

			m_header->Wrap(this->GetSize().GetWidth() - Dialog_def_border * 2);

			this->Layout();
			
			wxSize size = this->GetSize();

			int margin = Dialog_def_border * 2;

			int scrolled_add_width = std::max(m_scroller->GetVirtualSize().x - m_scroller->GetSize().x + margin, margin);
			int req_width = size.GetWidth() + scrolled_add_width;

			int scrolled_add_height = std::max(m_scroller->GetVirtualSize().y - m_scroller->GetSize().y + margin, margin);
			int req_height = size.GetHeight() + scrolled_add_height;

			//account for scrollbars
			if (req_height > Dialog_max_height) {
				req_width += wxSystemSettings::GetMetric(wxSYS_VSCROLL_X);
			}
			else {
				if (req_width > Dialog_max_width) {
					req_height += wxSystemSettings::GetMetric(wxSYS_HSCROLL_Y);
				}
				if (req_height > Dialog_max_height) {
					req_width += wxSystemSettings::GetMetric(wxSYS_VSCROLL_X);
				}
			}

			this->SetSize(wxSize(std::min(req_width, Dialog_max_width), std::min(req_height, Dialog_max_height)));
		}

		wxString UnsavedChangesDialog::get_header_msg(const wxString& dirty_tabs) {
			if (m_external_header_str.empty()) {
				return dirty_tabs.empty() ?
					"" :
					_(L("The presets on the following tabs were modified")) + ": " + dirty_tabs;
			}

			return m_external_header_str;
		}

		void UnsavedChangesDialog::buildScrollWindow(wxString& dirty_tabs) {
			m_scroller_container = new wxWindow(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_SIMPLE);
			m_scroller_container->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
			wxBoxSizer* wrapper_sizer = new wxBoxSizer(wxVERTICAL);

			buildScroller(dirty_tabs);

			wxStaticLine* line = new wxStaticLine(m_scroller_container, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL);

			wxBoxSizer* btn_sizer = new wxBoxSizer(wxHORIZONTAL);
				m_btn_save = new ScalableButton(m_scroller_container, wxID_ANY, "save", _(L("Save selected")), wxDefaultSize, wxDefaultPosition, wxBU_LEFT | wxBU_EXACTFIT);
				m_btn_save->Bind(wxEVT_BUTTON, &UCD::OnBtnSaveSelected, this);

				m_btn_select_all = new wxButton(m_scroller_container, wxID_ANY, _(L("Select All")));
				m_btn_select_all->Bind(wxEVT_BUTTON, [this](wxCommandEvent& e) {
					m_dirty_tabs_tree->selectChilds(true);
					updateSaveBtn();
				});

				m_btn_select_none = new wxButton(m_scroller_container, wxID_ANY, _(L("Select None")));
				m_btn_select_none->Bind(wxEVT_BUTTON, [this](wxCommandEvent& e) {
					m_dirty_tabs_tree->selectChilds(false);
					updateSaveBtn();
				});

			btn_sizer->Add(m_btn_save);
			btn_sizer->Add(m_btn_select_all, 0, wxLEFT | wxRIGHT, Dialog_def_border);
			btn_sizer->Add(m_btn_select_none);

			wrapper_sizer->Add(m_scroller, 1, wxEXPAND);
			wrapper_sizer->AddSpacer(Dialog_def_border * 2);
			wrapper_sizer->Add(line, 0, wxLEFT | wxRIGHT | wxEXPAND, Dialog_def_border);
			wrapper_sizer->Add(btn_sizer, 0, wxALL, Dialog_def_border);

			m_scroller_container->SetSizer(wrapper_sizer);
		}

		void UnsavedChangesDialog::buildScroller(wxString& dirty_tabs) {
			if (m_dirty_tabs_tree != nullptr) {
				delete m_dirty_tabs_tree;
			}
			m_dirty_tabs_tree = new UCD::dirty_opts_node();

			if (m_scroller == nullptr) {
				m_scroller = new wxScrolledWindow(m_scroller_container, wxID_ANY);
			}
			else {
				m_scroller->DestroyChildren();
			}

			wxBoxSizer* scrolled_sizer = new wxBoxSizer(wxVERTICAL);

			wxColour_toggle bg_color(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW), wxSystemSettings::GetColour(wxSYS_COLOUR_FRAMEBK));

			int itemCount = 0;
			for (Tab* cur_tab : m_tabs) {
                wxString title = cur_tab->title();

                if (! dirty_tabs.empty())
                    dirty_tabs += wxString(", ");
                dirty_tabs += title;

				wxPanel* cur_tab_win = new wxPanel(m_scroller, wxID_ANY);
				wxBoxSizer* cur_tab_sizer = new wxBoxSizer(wxVERTICAL);

				wxPanel* line_panel;
				wxBoxSizer* line_sizer;
				buildLineContainer(cur_tab_win, line_panel, line_sizer, bg_color, wxPoint(Dialog_def_border, Dialog_def_border /2));

				title += " - \"" + cur_tab->m_presets->get_edited_preset().name + "\"";
				UCD::dirty_opts_node* cur_tab_node = buildNode(line_panel, title, this->m_dirty_tabs_tree, line_sizer, cur_tab);
				wxCheckBox* cur_tab_cb = cur_tab_node->checkbox;
				cur_tab_cb->SetFont(GUI::wxGetApp().bold_font());

				line_sizer->Add(cur_tab_cb, 0, wxLEFT | wxALIGN_LEFT | wxALIGN_TOP, Dialog_def_border);
				cur_tab_sizer->Add(line_panel, 0, wxEXPAND);
				add_dirty_options(cur_tab, cur_tab_win, cur_tab_sizer, cur_tab_node, bg_color);

				cur_tab_win->SetSizer(cur_tab_sizer);
				//cur_tab_win->SetBackgroundColour(background);

				scrolled_sizer->Add(cur_tab_win, 0, wxEXPAND);
				itemCount++;
			}

			//resize all entry checkboxes so the val windows are the same distance from the left side
			int max_width = this->m_dirty_tabs_tree->get_max_child_label_width_with_indent();
			std::vector<UCD::dirty_opt_entry*> opts;
			this->m_dirty_tabs_tree->getAllOptionEntries(opts);
			for (UCD::dirty_opt_entry* cur_opt : opts) {
				int indent = get_wxSizerItem_border_size(cur_opt->parent_sizer->GetItem(cur_opt->checkbox), wxLEFT).GetWidth();
				cur_opt->checkbox->SetMinSize(wxSize(max_width - indent + Dialog_child_indentation, -1));
			}

			//should only happen when the user selected everything, saved, and the dialog rebuilds
			if (!itemCount) {
				wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);

				wxStaticBitmap* icon = new wxStaticBitmap(m_scroller, wxID_ANY, create_scaled_bitmap(nullptr, "tick_mark", 32));
				wxStaticText* msg = new wxStaticText(m_scroller, wxID_ANY, _(L("Nothing to save")));
				wxFont msg_font = GUI::wxGetApp().normal_font();
				msg_font.SetPointSize(12);
				msg->SetFont(msg_font);

				sizer->Add(icon, 0, wxRIGHT, Dialog_def_border);
				sizer->Add(msg, 0, wxALIGN_CENTER_VERTICAL);

				scrolled_sizer->AddStretchSpacer();
				scrolled_sizer->Add(sizer, 0, wxALIGN_CENTER_HORIZONTAL);
				scrolled_sizer->AddStretchSpacer();

				m_btn_save->Enable(false);
				m_btn_select_all->Enable(false);
				m_btn_select_none->Enable(false);
			}

			m_scroller->SetSizer(scrolled_sizer, true);
			m_scroller->SetScrollRate(2, 2);
		}

		void UnsavedChangesDialog::add_dirty_options(Tab* tab, wxWindow* parent, wxBoxSizer* sizer, UCD::dirty_opts_node* parent_node, wxColour_toggle& bg_colour) {
			std::vector<dirty_opt> options;
			PageIconMap page_icons;
			get_dirty_options_for_tab(tab, options, page_icons);			

			UCD::dirty_opts_node* cur_parent_node;
			UCD::dirty_opts_node* cur_page_node;

			std::string last_page_name = "";
			std::string last_group_name = "";

			int cur_indent = 0;
			bool firstPage = true;
			for (dirty_opt& cur_opt : options) {
				std::string cur_page_name = cur_opt.page_name;
				std::string cur_group_name = cur_opt.optgroup_name;

				wxPanel* linePanel;
				wxBoxSizer* lineSizer;

				if (cur_page_name == "") {
					cur_page_name = "Other";
				}

				if (cur_page_name != last_page_name) {
					last_page_name = cur_page_name;
					last_group_name = "";

					buildLineContainer(parent, linePanel, lineSizer, bg_colour, Dialog_def_border * (firstPage ? 1 : 2));

					PageIconMap::iterator it = page_icons.find(cur_page_name);

					if (it == page_icons.end()) {
						cur_parent_node = buildNode(linePanel, cur_page_name, parent_node, lineSizer);
						
						wxCheckBox* cb = cur_parent_node->checkbox;
						cb->SetValue(true);
						cb->SetFont(GUI::wxGetApp().bold_font());
						lineSizer->Add(cb, 0, wxLEFT | wxALIGN_LEFT, Dialog_def_border + Dialog_child_indentation);
					}
					else {
						cur_parent_node = buildNode(linePanel, "", parent_node, lineSizer);
						wxCheckBox* cb = cur_parent_node->checkbox;
						cb->SetValue(true);

						GrayableStaticBitmap* icon = new GrayableStaticBitmap(linePanel, wxID_ANY, it->second);
						wxStaticText* label = new wxStaticText(linePanel, wxID_ANY, cur_page_name);
						label->SetFont(GUI::wxGetApp().bold_font());

						lineSizer->Add(cb, 0, wxLEFT, Dialog_def_border + Dialog_child_indentation);
						lineSizer->Add(icon, 0, wxLEFT | wxRIGHT, Dialog_def_border);
						lineSizer->Add(label);

						cur_parent_node->icon = icon;
						cur_parent_node->labelCtrl = label;

					}
					sizer->Add(linePanel, 0, wxALIGN_LEFT | wxEXPAND);

					cur_page_node = cur_parent_node;
					cur_indent = Dialog_def_border + Dialog_child_indentation * 2;
					firstPage = false;
				}

				if (cur_group_name != last_group_name) {
					last_group_name = cur_group_name;

					if (cur_group_name == "") {
						cur_parent_node = cur_page_node;
						cur_indent = Dialog_def_border + Dialog_child_indentation * 2;
					}
					else {
						buildLineContainer(parent, linePanel, lineSizer, bg_colour, Dialog_def_border);

						cur_parent_node = buildNode(linePanel, cur_group_name, cur_page_node, lineSizer);
						wxCheckBox* cat_cb = cur_parent_node->checkbox;
						cat_cb->SetValue(true);
						cat_cb->SetFont(GUI::wxGetApp().bold_font());
						lineSizer->Add(cat_cb, 0, wxLEFT | wxALIGN_LEFT, Dialog_def_border + Dialog_child_indentation * 2);
						sizer->Add(linePanel, 0, wxALIGN_LEFT | wxEXPAND);

						cur_indent = Dialog_def_border + Dialog_child_indentation * 3;
					}
				}
				
				buildLineContainer(parent, linePanel, lineSizer, bg_colour, Dialog_def_border, true);

					UCD::dirty_opt_entry& cur_opt_entry = buildOptionEntry(linePanel, cur_parent_node, cur_opt, bg_colour, lineSizer);

					wxCheckBox* opt_label = cur_opt_entry.checkbox;

					lineSizer->Add(opt_label, 0, wxLEFT, cur_indent);

					wxBoxSizer* old_sizer = new wxBoxSizer(wxHORIZONTAL);
					old_sizer->Add(cur_opt_entry.old_win);

					wxBoxSizer* new_sizer = new wxBoxSizer(wxHORIZONTAL);
					new_sizer->Add(cur_opt_entry.new_win);

					lineSizer->Add(old_sizer, 1, wxEXPAND);
					lineSizer->AddSpacer(30);
					lineSizer->Add(new_sizer, 1, wxEXPAND);

				sizer->Add(linePanel, 0, wxALIGN_LEFT | wxEXPAND);
			}
			sizer->AddSpacer(Dialog_def_border);
		}

		void UnsavedChangesDialog::get_dirty_options_for_tab(Tab* tab, std::vector<dirty_opt>& out, PageIconMap& page_icons_out) {
			for (t_config_option_key key : tab->m_presets->current_dirty_options()) {
				dirty_opt opt(tab->m_presets->get_selected_preset().config.def()->get(key),
					tab->m_presets->get_selected_preset().config.option(key),
					tab->m_presets->get_edited_preset().config.option(key),
					key);

				if (opt.def->category.find('#') != std::string::npos && opt.old_opt->type() & opt.new_opt->type() & coVectorType) {
					split_dirty_option_by_extruders(opt, out);
				}
				else {
					out.push_back(opt);
				}
			}

			if (tab->type() == Slic3r::Preset::TYPE_PRINTER) {
				size_t old_num_extruders = tab->m_presets->get_selected_preset().get_num_extruders();
				size_t new_num_extruders = tab->m_presets->get_edited_preset().get_num_extruders();

				if (old_num_extruders != new_num_extruders) {
					dirty_opt opt(dynamic_cast<TabPrinter*>(tab)->m_extruders_count_def, old_num_extruders, new_num_extruders);
					out.push_back(opt);
				}
			}

			for (dirty_opt& cur_opt : out) {
				PageOptGroupShp ptrs = cur_opt.key == "bed_custom_texture" || cur_opt.key == "bed_custom_model" ?
					tab->get_page_and_optgroup("bed_shape"):
					tab->get_page_and_optgroup(cur_opt.key, cur_opt.extruder_index);

				if (ptrs.first != nullptr) {
					cur_opt.page_name = ptrs.first->title();

					if (ptrs.second != nullptr) {
						cur_opt.optgroup_name = ptrs.second->title;
					}
				}

				size_t tag_pos = cur_opt.page_name.find("#");
				if (tag_pos != std::string::npos)
				{
					cur_opt.page_name.replace(tag_pos, 1, std::to_string(cur_opt.extruder_index + 1));
				}

				if (ptrs.first != nullptr) {
					page_icons_out.insert(std::pair<std::string, wxBitmap>(cur_opt.page_name, tab->get_page_icon(ptrs.first->iconID())));
				}
			}

			boost::sort(out);
		}

		wxBitmap UnsavedChangesDialog::getColourBitmap(const std::string& color) {
			const double em = Slic3r::GUI::wxGetApp().em_unit();
			const int icon_width = lround(6.4 * em);
			const int icon_height = lround(1.6 * em);

			unsigned char rgb[3];
			if (!Slic3r::PresetBundle::parse_color(color, rgb)) {
				wxBitmap bmp = BitmapCache::mksolid(icon_width, icon_height, 0, 0, 0, wxALPHA_TRANSPARENT);

				wxMemoryDC dc(bmp);
				if (!dc.IsOk()) 
					return bmp;

				dc.SetTextForeground(*wxWHITE);
				dc.SetFont(wxGetApp().normal_font());

				const wxRect rect = wxRect(0, 0, bmp.GetWidth(), bmp.GetHeight());
				dc.DrawLabel("undef", rect, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL);

				dc.SelectObject(wxNullBitmap);

				return bmp;
			}
			else {
				wxBitmap bmp = BitmapCache::mksolid(icon_width, icon_height, rgb);
				return bmp;
			}
		}

		void UnsavedChangesDialog::split_dirty_option_by_extruders(const dirty_opt& opt, std::vector<dirty_opt>& out) {
			auto* opt_old = dynamic_cast<ConfigOptionVectorBase*>(opt.old_opt);
			auto* opt_new = dynamic_cast<ConfigOptionVectorBase*>(opt.new_opt);

			assert(opt_old != nullptr && opt_new != nullptr);

			std::vector<std::string> old_vals = opt_old->v_to_string();
			std::vector<std::string> new_vals = opt_new->v_to_string();

			for (size_t i = 0; i < old_vals.size() && i < new_vals.size(); i++) {
				std::string cur_old_val = old_vals[i];
				std::string cur_new_val = new_vals[i];

				if (cur_old_val != cur_new_val) {
					dirty_opt new_opt(opt);
					new_opt.extruder_index = i;
					new_opt.ser_old_opt = cur_old_val;
					new_opt.ser_new_opt = cur_new_val;

					out.push_back(new_opt);
				}
			}
		}

		void UnsavedChangesDialog::updateSaveBtn() {
			if (m_dirty_tabs_tree->hasAnythingToSave()) {
				m_btn_save->Enable();
			}
			else {
				m_btn_save->Enable(false);
			}
		}

		void UnsavedChangesDialog::refresh_tab_list() {
			std::vector<Tab*> new_tabs;

			for (Tab* cur_tab : m_tabs) {
				if ((m_print_tech == PrinterTechnology::ptUnknown || cur_tab->supports_printer_technology(m_print_tech)) && cur_tab->current_preset_is_dirty()) {
					new_tabs.emplace_back(cur_tab);
				}
			}

			m_tabs = std::move(new_tabs);
		}

		wxBoxSizer* UnsavedChangesDialog::buildYesNoBtns() {
			wxBoxSizer* cont_stretch_sizer = new wxBoxSizer(wxHORIZONTAL);
			wxPanel* cont_win = new wxPanel(this, wxID_ANY);
			cont_win->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_FRAMEBK));
			wxBoxSizer* cont_sizer = new wxBoxSizer(wxHORIZONTAL);
			wxStaticText* cont_label = new wxStaticText(cont_win, wxID_ANY, _(L("Continue? All unsaved changes will be discarded.")), wxDefaultPosition, wxDefaultSize);
			wxButton* btn_yes = new wxButton(cont_win, wxID_ANY, _(L("Continue")));
			btn_yes->Bind(wxEVT_BUTTON, ([this](wxCommandEvent& e) {
				EndModal(wxID_YES);
			}));

			wxButton* btn_no = new wxButton(cont_win, wxID_ANY, _(L("Cancel")));
			btn_no->Bind(wxEVT_BUTTON, ([this](wxCommandEvent& e) {
				EndModal(wxID_CANCEL);
			}));
			btn_no->SetFocus();

			cont_sizer->AddStretchSpacer();
			cont_sizer->Add(cont_label, 0, wxALL | wxALIGN_CENTER_VERTICAL, Dialog_def_border * 3);

			cont_sizer->Add(btn_yes, 0, wxALIGN_CENTER_VERTICAL);
			cont_sizer->Add(btn_no, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, Dialog_def_border);

			cont_win->SetSizer(cont_sizer);
			cont_stretch_sizer->Add(cont_win, 1);

			return cont_stretch_sizer;
		}

		void UnsavedChangesDialog::buildLineContainer(wxWindow* parent, wxPanel*& cont_out, wxBoxSizer*& cont_sizer_out, wxColour_toggle& bg_colour, wxPoint v_padding, bool toggle_col) {
			wxPanel* linePanel = new wxPanel(parent);
			if (!toggle_col) {
				bg_colour.reset();
			}
			linePanel->SetBackgroundColour(bg_colour.get());
			if (toggle_col) {
				bg_colour.toggle();
			}

			wxBoxSizer* vSizer = new wxBoxSizer(wxVERTICAL);
			wxBoxSizer* lineSizer = new wxBoxSizer(wxHORIZONTAL);

			vSizer->AddSpacer(v_padding.x);
			vSizer->Add(lineSizer, 0, wxEXPAND);
			vSizer->AddSpacer(v_padding.y);
			linePanel->SetSizer(vSizer);

			cont_out = linePanel;
			cont_sizer_out = lineSizer;
		}

		void UnsavedChangesDialog::buildLineContainer(wxWindow* parent, wxPanel*& cont_out, wxBoxSizer*& cont_sizer_out, wxColour_toggle& bg_colour, int v_padding, bool toggle_col) {
			buildLineContainer(parent, cont_out, cont_sizer_out, bg_colour, wxPoint(v_padding / 2, v_padding / 2), toggle_col);
		}

		UCD::dirty_opts_node* UnsavedChangesDialog::buildNode(wxWindow* parent, const wxString& label, UCD::dirty_opts_node* parent_node, wxSizer* parent_sizer, Tab* tab) {
			UCD::dirty_opts_node* node = new UCD::dirty_opts_node(parent_node, parent_sizer, label, tab);

			node->checkbox = buildCheckbox(parent, label, [this, node](wxCommandEvent& e) {
				node->on_checkbox_toggled();
				updateSaveBtn();
			});

			parent_node->childs.push_back(node);
			return node;
		}

		UCD::dirty_opt_entry& UnsavedChangesDialog::buildOptionEntry(wxWindow* parent, UCD::dirty_opts_node* parent_node, dirty_opt opt, wxColour_toggle& bg_colour, wxSizer* parent_sizer) {
			parent_node->opts.push_back(new UCD::dirty_opt_entry(opt, nullptr, parent_node, parent_sizer));
			UCD::dirty_opt_entry& entry = *parent_node->opts.back();

			wxCheckBox* cb = buildCheckbox(parent, _(opt.def->label), [this, &entry](wxCommandEvent& e) {
				updateSaveBtn();
				entry.on_checkbox_toggled();
			}, wxDefaultSize, getTooltipText(*opt.def, opt.extruder_index));

			entry.checkbox = cb;

			buildWindowsForOpt(entry, parent, bg_colour);

			return entry;
		}

		template<typename Functor>
		wxCheckBox* UnsavedChangesDialog::buildCheckbox(wxWindow* parent, const wxString& label, const Functor& toggleCallback, wxSize size, std::string tooltip) {
			wxCheckBox* cb = new wxCheckBox(parent, wxID_ANY, label, wxDefaultPosition, size);
			cb->SetValue(true);
			cb->Bind(wxEVT_CHECKBOX, toggleCallback);
			cb->SetToolTip(tooltip);

			return cb;
		}

		void UnsavedChangesDialog::buildWindowsForOpt(UCD::dirty_opt_entry& opt, wxWindow* parent, wxColour_toggle& bg_colour) {
			std::string old_val;
			std::string new_val;

			dirty_opt& val = opt.val;

			if (opt.type() == dirty_opt::Type::ConfigOption) {
				const ConfigOption* old_opt = opt.val.old_opt;
				const ConfigOption* new_opt = opt.val.new_opt;

				if (val.extruder_index >= 0) {
					old_val = val.ser_old_opt;
					new_val = val.ser_new_opt;
				}
				else {
					old_val = old_opt->to_string();
					new_val = new_opt->to_string();
				}
			}
			else if (opt.type() == dirty_opt::Type::ExtruderCount) {
				old_val = std::to_string(opt.val.extruders_count_old);
				new_val = std::to_string(opt.val.extruders_count_new);
			}

			wxWindow* win_old_opt;
			wxWindow* win_new_opt;

			std::string tooltip = getTooltipText(*opt.val.def, opt.val.extruder_index);

			if (val.def->gui_type == "color") {
				win_old_opt = buildColorWindow(parent, old_val);
				win_new_opt = buildColorWindow(parent, new_val);

				opt.old_win_type = UCD::dirty_opt_entry::Gui_Type::Color;
				opt.new_win_type = UCD::dirty_opt_entry::Gui_Type::Color;
			}
			else {
				switch (val.def->type) {
				case coString:
				case coStrings: {
					win_old_opt = buildStringWindow(parent, bg_colour, false, old_val, new_val, opt, tooltip);
					win_new_opt = buildStringWindow(parent, bg_colour, true, old_val, new_val, opt, tooltip);

					opt.new_win_type = UCD::dirty_opt_entry::Gui_Type::Text;
					opt.old_win_type = UCD::dirty_opt_entry::Gui_Type::Diff;

					break;
				}
				case coEnum: {
					const ConfigOptionDef* def = opt.val.def;
					for (size_t i = 0; i < def->enum_values.size() && i < def->enum_labels.size(); i++) {
						if (old_val == def->enum_values[i]) {
							old_val = _(def->enum_labels[i]);
						}
						if (new_val == def->enum_values[i]) {
							new_val = _(def->enum_labels[i]);
						}
					}
				}
				case coFloatOrPercent:
				case coFloat:
				case coFloats:
				case coPercent:
				case coPercents:
				case coInt:
				case coInts:
				case coBool:
				case coBools:
				case coPoint:
				case coPoint3:
				case coPoints: {
					if (val.key == "bed_shape") {
						win_old_opt = buildShapeWindow(parent, dynamic_cast<ConfigOptionPoints*>(opt.val.old_opt), false);
						win_new_opt = buildShapeWindow(parent, dynamic_cast<ConfigOptionPoints*>(opt.val.new_opt), true);
						opt.old_win_type = UCD::dirty_opt_entry::Gui_Type::Text;
						opt.new_win_type = UCD::dirty_opt_entry::Gui_Type::Text;
					}
					else {
						win_old_opt = new wxStaticText(parent, wxID_ANY, old_val, wxDefaultPosition, wxDefaultSize);
						win_new_opt = new wxStaticText(parent, wxID_ANY, new_val, wxDefaultPosition, wxDefaultSize);
						opt.old_win_type = UCD::dirty_opt_entry::Gui_Type::Text;
						opt.new_win_type = UCD::dirty_opt_entry::Gui_Type::Text;
					}

					break;
				}
				default:
					win_old_opt = new wxStaticText(parent, wxID_ANY, "This control doesn't exist till now", wxDefaultPosition, wxDefaultSize);
					win_new_opt = new wxStaticText(parent, wxID_ANY, "This control doesn't exist till now", wxDefaultPosition, wxDefaultSize);
				}
			}

			win_new_opt->SetForegroundColour(wxGetApp().get_label_clr_modified());

			win_new_opt->SetToolTip(tooltip);
			win_old_opt->SetToolTip(tooltip);

			opt.old_win = win_old_opt;
			opt.new_win = win_new_opt;
		}

		wxWindow* UnsavedChangesDialog::buildColorWindow(wxWindow* parent, std::string col) {
			return col == "-" ?
				(wxWindow*)new wxStaticText(parent, wxID_ANY, "-", wxDefaultPosition, wxDefaultSize) :
				(wxWindow*)new GrayableStaticBitmap(parent, wxID_ANY, getColourBitmap(col));
		}

		wxWindow* UnsavedChangesDialog::buildStringWindow(wxWindow* parent, wxColour_toggle& bg_colour, bool isNew, const std::string& old_val, const std::string& new_val, UCD::dirty_opt_entry& opt, const std::string& tooltip) {
			if (isNew) {
				wxGenericStaticText* txt = new wxGenericStaticText(parent, wxID_ANY, new_val, wxDefaultPosition, wxDefaultSize);
				wxFont f = GUI::wxGetApp().normal_font();
				f.SetFamily(wxFONTFAMILY_TELETYPE);
				txt->SetFont(f);
				return txt;
			}

			auto fakeAlpha = [](wxColour front, wxColour back, unsigned char alpha) -> std::string {
				unsigned char r, g, b;
				r = front.Red() * alpha / 255 + back.Red() * (255 - alpha) / 255;
				g = front.Green() * alpha / 255 + back.Green() * (255 - alpha) / 255;
				b = front.Blue() * alpha / 255 + back.Blue() * (255 - alpha) / 255;

				return wxString::Format(wxT("#%02X%02X%02X"), r, g, b).ToStdString();
			};

			wxPanel* win = new wxPanel(parent);
			wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);

			auto add_line = [win, sizer, tooltip](const std::string& line) {
				GrayableMarkupText* txt = new GrayableMarkupText(win, wxID_ANY, line);
				//txt->SetLabelMarkup(line);
				wxFont f = GUI::wxGetApp().normal_font();
				f.SetFamily(wxFONTFAMILY_TELETYPE);
				txt->SetFont(f);
				txt->SetToolTip(tooltip);

				sizer->Add(txt);
			};

			using namespace slic3r;
			Diff diff(old_val, new_val, true);
			Diff::EditScript& editScript = diff.get_edit_script();
			editScript.split_by_newline(old_val, new_val);
			//diff.selfTest();

			std::string cur_line;

            wxColour col_rem(255, 75, 75);
            wxColour col_ins(75, 255, 75);

			for (const Diff::EditScript::Action& cur_action : editScript.actions) {
				std::string sub;
				wxColour* color;
				bool useColor = false;
				bool lb = false;

				switch (cur_action.type)
				{
					case Diff::EditScript::Action::Type::keep: {
						sub = old_val.substr(cur_action.offset, cur_action.count);
						break;
					}
					case Diff::EditScript::Action::Type::remove: {
						sub = old_val.substr(cur_action.offset, cur_action.count);
                        color = &col_rem;
						useColor = true;
						break;
					}
					case Diff::EditScript::Action::Type::insert: {
						sub = new_val.substr(cur_action.offset, cur_action.count);
                        color = &col_ins;
						useColor = true;
						break;
					}
					case Diff::EditScript::Action::Type::lineBreak: {
						lb = true;
						break;
					}
					case Diff::EditScript::Action::Type::remove_lineBreak: {
						sub = "\\n";
						lb = true;
                        color = &col_rem;
						useColor = true;
						break;
					}
					case Diff::EditScript::Action::Type::insert_lineBreak: {
						sub = "\\n";
						lb = true;
                        color = &col_ins;
						useColor = true;
						break;
					}
				}

				boost::replace_all(sub, "&", "&amp;");
				boost::replace_all(sub, "'", "&apos;");
				boost::replace_all(sub, "\"", "&quot;");
				boost::replace_all(sub, "<", "&lt;");
				boost::replace_all(sub, ">", "&gt;");

				if (useColor) {
					if (!lb || cur_line.empty()) {
						boost::format fmter("<span bgcolor=\"%1%\">%2%</span>");
						fmter%
							fakeAlpha(*color, bg_colour.get(), 130) %
							sub;

						cur_line += fmter.str();
					}
				}
				else {
					cur_line += sub;
				}

				if (lb) {
					cur_line = cur_line.empty() ? " " : cur_line;
					add_line(cur_line);
					cur_line = "";
				}
			}
			add_line(cur_line);

			win->SetSizer(sizer);
			
			return win;
		}

		wxWindow* UnsavedChangesDialog::buildShapeWindow(wxWindow* parent, ConfigOptionPoints* opt, bool isNew) {
			BedShape bed_shape(*opt);

			wxWindow* win = new wxPanel(parent);
			wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);

			auto shape = new wxStaticText(win, wxID_ANY, _(L("Shape")) + ": " + BedShapePanel::get_shape_name(bed_shape.type));
			if (isNew) {
				shape->SetForegroundColour(wxGetApp().get_label_clr_modified());
			}

			sizer->Add(shape);

			if (bed_shape.type == BedShape::TRectangular) {
				ConfigOptionDef size_def = BedShapePanel::get_ConfigOptionDef("rect_size");
				ConfigOptionDef orig_def = BedShapePanel::get_ConfigOptionDef("rect_origin");

				auto s = new wxStaticText(win, wxID_ANY, _(size_def.label) + ": " + ConfigOptionPoint(bed_shape.rectSize).to_string());
				auto o = new wxStaticText(win, wxID_ANY, _(orig_def.label) + ": " + ConfigOptionPoint(bed_shape.rectOrigin).to_string());

				s->SetToolTip(getTooltipText(size_def, -1));
				o->SetToolTip(getTooltipText(orig_def, -1));

				if (isNew) {
					s->SetForegroundColour(wxGetApp().get_label_clr_modified());
					o->SetForegroundColour(wxGetApp().get_label_clr_modified());
				}

				sizer->Add(s);
				sizer->Add(o);
			}
			else if (bed_shape.type == BedShape::TCircular) {
				ConfigOptionDef diam_def = BedShapePanel::get_ConfigOptionDef("diameter");

				auto d = new wxStaticText(win, wxID_ANY, _(diam_def.label) + ": " + wxNumberFormatter::ToString(bed_shape.diameter, 0, wxNumberFormatter::Style_None));

				d->SetToolTip(getTooltipText(diam_def, -1));

				if (isNew) {
					d->SetForegroundColour(wxGetApp().get_label_clr_modified());
				}

				sizer->Add(d);
			}

			win->SetSizer(sizer);
			return win;
		}

		std::string UnsavedChangesDialog::getTooltipText(const ConfigOptionDef& def, int extrIdx) {
			wxString default_val = def.default_value->to_string();

			std::string opt_id = def.opt_key;
			if (extrIdx >= 0) {
				opt_id += "[" + std::to_string(extrIdx) + "]";
			}

            std::string tooltip = _(def.tooltip).ToStdString();
			if (tooltip.length() > 0)
				tooltip = tooltip + "\n" + _(L("default value")) + "\t: " +
				(boost::iends_with(opt_id, "_gcode") ? "\n" : "") + default_val +
				(boost::iends_with(opt_id, "_gcode") ? "" : "\n") +
				_(L("parameter name")) + "\t: " + opt_id;

			return tooltip;
		}

		void UnsavedChangesDialog::OnBtnSaveSelected(wxCommandEvent& e)
		{
			SavePresetWindow dlg(this);

			for (UCD::dirty_opts_node* cur_tab : m_dirty_tabs_tree->childs) {
				if (!cur_tab->hasAnythingToSave()) {
					continue;
				}

				PresetCollection* m_presets = cur_tab->tab->m_presets;

				const Preset& preset = m_presets->get_selected_preset();
				auto default_name = preset.is_default ? "Untitled" :
					preset.is_system ? (boost::format(_utf8(L("%1% - Copy"))) % preset.name).str() :
					preset.name;

				bool have_extention = boost::iends_with(default_name, ".ini");
				if (have_extention) {
					size_t len = default_name.length() - 4;
					default_name.resize(len);
				}

				std::vector<std::string> values;
				for (size_t i = 0; i < m_presets->size(); ++i) {
					const Preset& preset = m_presets->preset(i);
					if (preset.is_default || preset.is_system || preset.is_external)
						continue;
					values.push_back(preset.name);
				}

				dlg.build_entry(cur_tab->tab->title(), default_name, values, m_presets, cur_tab->tab);
			}

			if (dlg.ShowModal() == wxID_OK){
				for (std::pair<Tab*, std::string> cur_pair : dlg.get_tab_name_pairs()) {
					Tab* cur_tab = cur_pair.first;
					std::string chosen_name = cur_pair.second;

					DynamicPrintConfig& edited_conf = cur_tab->m_presets->get_edited_preset().config;
					DynamicPrintConfig edited_backup(edited_conf);

					UCD::dirty_opts_node* cur_tab_node = m_dirty_tabs_tree->getTabNode(cur_tab);

					std::vector<UCD::dirty_opt_entry*> opts_to_save;
					cur_tab_node->getAllOptionEntries(opts_to_save, true);

					bool update_extr_count = true;

					//set all options (in the edited preset) - that the user does not want to save - to their old values (from selected preset)
					for (UCD::dirty_opt_entry* cur_opt_to_save : opts_to_save) {
						if (cur_opt_to_save->type() == dirty_opt::Type::ConfigOption) {
							int idx = cur_opt_to_save->val.extruder_index;

							if (idx >= 0) {
								auto* old_opt = dynamic_cast<ConfigOptionVectorBase*>(cur_opt_to_save->val.old_opt);
								auto* new_opt = dynamic_cast<ConfigOptionVectorBase*>(cur_opt_to_save->val.new_opt);

								new_opt->set_at(old_opt, idx, idx);
							}
							else {
								auto* new_opt = edited_conf.option(cur_opt_to_save->val.key);
								new_opt->set(cur_opt_to_save->val.old_opt);
							}
						}
						else if (cur_opt_to_save->type() == dirty_opt::Type::ExtruderCount) {
							size_t count_old = cur_opt_to_save->val.extruders_count_old;
							size_t count_new = cur_opt_to_save->val.extruders_count_new;

							update_extr_count = false;

							if (count_old > count_new) {
								edited_conf.import_extruder_config_options(cur_tab->m_presets->get_selected_preset().config);
							}
							else {
								edited_conf.set_num_extruders(count_old);
							}
						}
					}			

					//edited conf will become selected conf and be saved to disk
					cur_tab->m_presets->save_current_preset(chosen_name);
					edited_conf = DynamicPrintConfig(std::move(edited_backup));

					cur_tab->update_after_preset_save(update_extr_count);
				}

				refresh_tab_list();

				//refresh the ui
				wxString dirty_tabs;
				this->Freeze();
				buildScroller(dirty_tabs);
				m_header->SetLabel(get_header_msg(dirty_tabs));
				setCorrectSize();
				this->Thaw();
				this->Refresh();
			}
		}
	}
}
