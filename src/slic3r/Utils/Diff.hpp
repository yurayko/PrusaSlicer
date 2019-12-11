#ifndef slic3r_Utils_Diff_hpp_
#define slic3r_Utils_Diff_hpp_

#include <string>
#include <vector>

namespace slic3r {
	class Diff {
	public:
		class EditScript {
		public:
			struct Action;
			typedef std::vector<Action> Actions;

			struct Action {
				struct Type {
					static const int nil = 0;
					static const int baseTypeMask = 0x0FFF;
					static const int lineBreakType = 0x1000;

					static const int keep = 1; //from A
					static const int remove = 2; //from A
					static const int insert = 4; //from B

					//only when split_newline==true
					static const int lineBreak = keep + lineBreakType;
					static const int remove_lineBreak = remove + lineBreakType;
					static const int insert_lineBreak = insert + lineBreakType;
				};

				int type;
				size_t offset;
				size_t count;

				Action(int _type, size_t _off, size_t _count) :
					type(_type),
					offset(_off),
					count(_count)
				{};

				bool is_linebreak();

				template<typename Container>
				Container& get_target_str(Container& str1, Container& str2);

				Actions split_by_newline(const std::string& str1, const std::string& str2);
			};

			Actions actions;
			void split_by_newline(const std::string& str1, const std::string& str2);
			int get_char_count(int type_mask);
		};

		Diff(const std::string& _str1, const std::string& _str2, bool linewise = false, int keep_line_treshold_percent = 40);

		EditScript& get_edit_script();

#ifdef _DEBUG
		void selfTest(int count = 666);
#endif
	private:
		struct npArray;

		EditScript editScript;

		int compare_elements(char chr1, char chr2);
		int compare_elements(const std::string& str1, const std::string& str2);

		template<typename Container>
		void solve(const Container& str1, const Container& str2, std::vector<npArray>& v_snapshots, bool linewise, int keep_line_treshold_percent);
		void construct_edit_script(int len1, int len2, std::vector<npArray>& v_snapshots);
		void construct_edit_script(std::vector<npArray>& v_snapshots, std::vector<std::string>& lines_1, std::vector<std::string>& lines_2, std::vector<size_t>& line_offs_1, std::vector<size_t>& line_offs_2);
	};
}

#endif
