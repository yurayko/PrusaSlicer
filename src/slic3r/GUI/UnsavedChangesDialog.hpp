#ifndef slic3r_GUI_UnsavedChangesDialog_hpp_
#define slic3r_GUI_UnsavedChangesDialog_hpp_

#include <wx/checkbox.h>
#include "GUI_App.hpp"
#include "Tab.hpp"
#include "Preset.hpp"
#include "PresetBundle.hpp"
#include "libslic3r/Config.hpp"

namespace Slic3r {
	namespace GUI {
		class UnsavedChangesDialog : public wxDialog
		{
		public:
			//Show all dirty tabs
			UnsavedChangesDialog(wxWindow* parent, GUI_App* app, const wxString& header, const wxString& caption = wxMessageBoxCaptionStr, long style = wxOK | wxCENTRE, const wxPoint& pos = wxDefaultPosition);

			//Show single tab
			UnsavedChangesDialog(wxWindow* parent, Tab* tab, const wxString& header, const wxString& caption = wxMessageBoxCaptionStr, long style = wxOK | wxCENTRE, const wxPoint& pos = wxDefaultPosition);

			~UnsavedChangesDialog();
		private:
			struct dirty_opt {
				enum class Type {
					Nil,
					ConfigOption,
					ExtruderCount
				};

				Type type = Type::Nil;

				std::string page_name = "";
				std::string optgroup_name = "";

				const ConfigOptionDef* def = nullptr;
				ConfigOption* old_opt = nullptr;
				ConfigOption* new_opt = nullptr;
				t_config_option_key key;

				int extruder_index = -1;
				std::string ser_old_opt;
				std::string ser_new_opt;

				//Type::ExtruderCount
				size_t extruders_count_old = 0;
				size_t extruders_count_new = 0;

				dirty_opt() {}

				dirty_opt(const ConfigOptionDef* _def, Type _type) :
					def(_def),
					type(_type),
					key(_def->opt_key),
					page_name(_def->category)
				{}

				dirty_opt(const ConfigOptionDef* def, ConfigOption* _old_opt, ConfigOption* _new_opt, t_config_option_key _key) :
					dirty_opt(def, Type::ConfigOption)
				{
					this->old_opt = _old_opt;
					this->new_opt = _new_opt;
					this->key = _key;
				}

				dirty_opt(const ConfigOptionDef* def, size_t _extruders_count_old, size_t _extruders_count_new) :
					dirty_opt(def, Type::ExtruderCount)
				{
					this->extruders_count_old = _extruders_count_old;
					this->extruders_count_new = _extruders_count_new;
				}

				bool operator <(const dirty_opt& b)
				{
					if (this->page_name != b.page_name) {
						return this->page_name < b.page_name;
					}
					else if (this->optgroup_name != b.optgroup_name) {
						if (this->optgroup_name.empty()) {
							return false;
						}
						if (b.optgroup_name.empty()) {
							return true;
						}

						return this->optgroup_name < b.optgroup_name;
					}
					else if (this->def != nullptr && b.def != nullptr) {
						return this->def->label < b.def->label;
					}

					return false;
				}
			};

			struct dirty_opts_node;

			//this binds the gui line of an option and its definition and internal value together. It's an endpoint of the tree and can't have children.
			struct dirty_opt_entry {
				enum class Gui_Type {
					Nil,
					Text,
					Color,
					Diff,
					Shape
				};
				typedef std::map<std::string, void*> Aux_Data;

				dirty_opts_node* parent;
				dirty_opt			val;

				wxCheckBox* checkbox = nullptr;
				wxSizer* parent_sizer;

				wxWindow* old_win = nullptr;
				wxWindow* new_win = nullptr;

				Gui_Type			old_win_type = Gui_Type::Nil;
				Gui_Type			new_win_type = Gui_Type::Nil;

				Aux_Data			aux_data;

				dirty_opt_entry(dirty_opt _val, wxCheckBox* _checkbox, dirty_opts_node* _parent, wxSizer* _parent_sizer)
					: val(_val), checkbox(_checkbox), parent(_parent), parent_sizer(_parent_sizer)
				{}

				~dirty_opt_entry() {
					Aux_Data::iterator it;
					for (it = this->aux_data.begin(); it != aux_data.end(); it++) {
						delete it->second;
					}
				}

				void setWinEnabled(wxWindow* win, Gui_Type _gui_type, bool enabled = true);
				void setValWinsEnabled(bool enabled = true);
				void set_checkbox(bool checked);
				void on_checkbox_toggled();
				void on_parent_checkbox_toggled(bool checked);
				bool saveMe();
				int get_checkbox_width_with_indent();
				dirty_opt::Type type();
			};

			//a node represents a parent (tab, category, ...) in the scroll window
			struct dirty_opts_node {
				dirty_opts_node*		parent = nullptr;

				wxString				label;
				wxCheckBox*				checkbox = nullptr;
				GrayableStaticBitmap*	icon = nullptr;
				wxStaticText*			labelCtrl = nullptr;
				wxSizer*				parent_sizer = nullptr;

				Tab* tab = nullptr;
				std::vector<dirty_opts_node*> childs;
				std::vector<dirty_opt_entry*> opts;


				dirty_opts_node() {}

				dirty_opts_node(dirty_opts_node* _parent, wxSizer* _parent_sizer, const wxString& _label, Tab* _tab)
					: parent(_parent), parent_sizer(_parent_sizer), label(_label), tab(_tab)
				{}

				~dirty_opts_node() {
					for (dirty_opts_node* cur_node : this->childs) {
						delete cur_node;
					}

					for (dirty_opt_entry* cur_opt_entry : this->opts) {
						delete cur_opt_entry;
					}
				}

				void on_checkbox_toggled();
				void on_child_checkbox_toggled(bool checked);
				void on_parent_checkbox_toggled(bool checked);
				void set_checkbox(bool checked);
				void selectChilds(bool selected = true);
				void getAllOptionEntries(std::vector<dirty_opt_entry*>& _opts, bool only_opts_to_restore = false, dirty_opt::Type type = dirty_opt::Type::Nil);
				bool hasAnythingToSave();
				dirty_opts_node* getTabNode(Tab* tab);
				int get_label_width_with_indent();
				int get_max_child_label_width_with_indent();
			};

			struct wxColour_toggle {
				wxColour colour_1, colour_2, cur_colour;

				wxColour_toggle(wxColour col_1, wxColour col_2) : colour_1(col_1), colour_2(col_2), cur_colour(col_1){}

				void toggle() {
					if (cur_colour == colour_1) {
						cur_colour = colour_2;
					}
					else {
						cur_colour = colour_1;
					}
				}

				wxColour get() {
					return cur_colour;
				}

				void reset() {
					cur_colour = colour_1;
				}
			};

			std::vector<Tab*>	m_tabs;

			wxStaticText*		m_header;
			wxString			m_external_header_str;
			wxWindow*			m_scroller_container;
			wxScrolledWindow*	m_scroller = nullptr;

			ScalableButton* m_btn_save;
			wxButton*		m_btn_select_all;
			wxButton*		m_btn_select_none;

			PrinterTechnology m_print_tech;

			dirty_opts_node* m_dirty_tabs_tree = nullptr;

			typedef std::map<std::string, wxBitmap> PageIconMap;

			void build();
			wxString get_header_msg(const wxString& dirty_tabs);
			void setCorrectSize();
			void buildScrollWindow(wxString& dirty_tabs);	//builds m_scroller_container
			void buildScroller(wxString& dirty_tabs);		//builds m_scroller
			void add_dirty_options(Tab* tab, wxWindow* parent, wxBoxSizer* sizer, dirty_opts_node* parent_node, wxColour_toggle& bg_colour);
			void get_dirty_options_for_tab(Tab* tab, std::vector<dirty_opt>& out, PageIconMap& page_icons_out);
			void split_dirty_option_by_extruders(const dirty_opt& pair, std::vector<dirty_opt>& out);
			wxBoxSizer* buildYesNoBtns();
			wxBitmap getColourBitmap(const std::string& color);
			void updateSaveBtn();
			void refresh_tab_list();

			void buildLineContainer(wxWindow* parent, wxPanel*& cont_out, wxBoxSizer*& cont_sizer_out, wxColour_toggle& bg_colour, wxPoint v_padding, bool toggle_col = false);
			void buildLineContainer(wxWindow* parent, wxPanel*& cont_out, wxBoxSizer*& cont_sizer_out, wxColour_toggle& bg_colour, int v_padding, bool toggle_col = false);
			dirty_opts_node* buildNode(wxWindow* parent, const wxString& label, dirty_opts_node* parent_node, wxSizer* parent_sizer, Tab* tab = nullptr);
			template<typename Functor>
			wxCheckBox* buildCheckbox(wxWindow* parent, const wxString& label, const Functor& toggleCallback, wxSize size = wxDefaultSize, std::string tooltip = "");
			dirty_opt_entry& buildOptionEntry(wxWindow* parent, dirty_opts_node* parent_node, dirty_opt opt, wxColour_toggle& bg_colour, wxSizer* parent_sizer);

			void buildWindowsForOpt(dirty_opt_entry& opt, wxWindow* parent, wxColour_toggle& bg_colour);
			wxWindow* buildColorWindow(wxWindow* parent, std::string col);
			wxWindow* buildStringWindow(wxWindow* parent, wxColour_toggle& bg_colour, bool isNew, const std::string& old_val, const std::string& new_val, dirty_opt_entry& opt, const std::string& tooltip);
			wxWindow* buildShapeWindow(wxWindow* parent, ConfigOptionPoints* opt, bool isNew);

			std::string getTooltipText(const ConfigOptionDef& def, int extrIdx);

			void OnBtnSaveSelected(wxCommandEvent& e);
		};
	}
}

#endif