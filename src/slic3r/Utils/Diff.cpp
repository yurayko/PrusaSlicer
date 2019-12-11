#include "Diff.hpp"
#include <stdlib.h>
#include <string>
#include <iostream>
#include <cstring>
#include <sstream>

#ifdef _DEBUG
#include <algorithm>
#include <cassert>
#endif

#define MAXCOST 100

namespace slic3r {

    // Definitions of static const class members:
    const int Diff::EditScript::Action::Type::nil;
    const int Diff::EditScript::Action::Type::baseTypeMask;
    const int Diff::EditScript::Action::Type::lineBreakType;

    const int Diff::EditScript::Action::Type::keep;
    const int Diff::EditScript::Action::Type::remove;
    const int Diff::EditScript::Action::Type::insert;

    const int Diff::EditScript::Action::Type::lineBreak;
    const int Diff::EditScript::Action::Type::remove_lineBreak;
    const int Diff::EditScript::Action::Type::insert_lineBreak;

	struct Point {
		int x = 0;
		int y = 0;

		Point() {}

		Point(int _x, int _y) {
			x = _x;
			y = _y;
		}
	};

	struct progress_val {
		int x;

		//removal/insertion has MAXCOST
		//keeping a line has MAXCOST*changed/kept chars in that line
		int cost;

		progress_val() : x(0), cost(0) {}
		progress_val(int _x, size_t _cost) : x(_x), cost(_cost) {}

		//bool down = (k == -d || (k != d && V[k -1] < V[k + 1]));
		bool operator<(const progress_val& rhs) {
			if (this->cost == rhs.cost) {
				return this->x < rhs.x;
			}
			return this->cost > rhs.cost;
		}
	};

	struct Diff::npArray {
		progress_val* vals = NULL;
		int size;
		int absLower;

		npArray(int lower, int upper) {
			size = abs(lower) + upper + 1;

			vals = new progress_val[size];
			absLower = abs(lower);
		}

		npArray(const npArray& _val) {
			size = _val.size;

			vals = new progress_val[size];
			absLower = _val.absLower;
            memcpy(vals, _val.vals, size * sizeof(progress_val));
		}

		npArray(const npArray& _val, int lower, int upper) : npArray(lower, upper) {
            memcpy(vals, &(_val.vals[lower + _val.absLower]), size * sizeof(progress_val));
		}

		~npArray() {
			delete[] vals;
		}

		progress_val& operator[](int index) {
			return vals[index + this->absLower];
		}
	};

	typedef Diff::EditScript ES;
	typedef Diff::EditScript::Action Action;

	void Diff::EditScript::split_by_newline(const std::string& str1, const std::string& str2) {
		ES::Actions new_sol;

		for (Action& cur_action : this->actions) {
			ES::Actions acts = cur_action.split_by_newline(str1, str2);
			new_sol.insert(new_sol.end(), acts.begin(), acts.end());
		}

		this->actions = std::move(new_sol);
	}

	int Diff::EditScript::get_char_count(int type_mask) {
		size_t total = 0;
		for (Action& cur_act : actions) {
			if (cur_act.type & type_mask) {
				total += cur_act.count;
			}
		}
		return total;
	}

	bool Action::is_linebreak() {
		return this->type & Type::lineBreakType;
	}

	template<typename Container>
	Container& Action::get_target_str(Container& str1, Container& str2) {
		if (this->type == Action::Type::keep || this->type == Action::Type::remove) {
			return str1;
		}
		return str2;
	}

	ES::Actions Action::split_by_newline(const std::string& str1, const std::string& str2) {
		ES::Actions acts;

		const std::string& str = get_target_str(str1, str2);

		size_t cur_off = this->offset;
		size_t end_off = cur_off + this->count;

		//0123456789
		//1n2n3n4n
		//-------	keep
		//       -	add

		bool cont = true;
		int lb_type = this->type + Action::Type::lineBreakType;

		while (cont && cur_off < end_off) {
			size_t pos = str.find("\n", cur_off);

			if (pos == std::string::npos || pos >= end_off) {
				pos = end_off;
				cont = false;
			}

			acts.emplace_back(pos == cur_off ? lb_type : this->type, cur_off, std::max((size_t)1, pos - cur_off));
			if (cont) {
				if (pos != cur_off) {
					acts.emplace_back(lb_type, pos, 1);
				}
				cur_off = pos + 1;
			}
		}

		return acts;
	}

	Diff::Diff(const std::string& _str1, const std::string& _str2, bool linewise, int keep_line_treshold_percent)
	{
		std::vector<npArray> v_snapshots;

		if (linewise) {
			std::vector<std::string> lines_1, lines_2;
			
			//line no -> str offset
			std::vector<size_t> offs_1, offs_2;

			auto split_lines = [](const std::string& str, std::vector<std::string>& line_dest, std::vector<size_t>& off_dest) {
				std::stringstream stream(str);

				size_t cur_off = 0;
				std::string cur_line;

				while (std::getline(stream, cur_line)) {
					if (!line_dest.empty()) {
						line_dest.back() += "\n";
						cur_off++;
					}
					
					line_dest.push_back(cur_line);
					off_dest.push_back(cur_off);
					
					cur_off += cur_line.length();
				}

				//getline does not continue on an empty last line...
				if (!str.empty() && str.back() == '\n') {
					line_dest.back() += "\n";
				}
			};

			split_lines(_str1, lines_1, offs_1);
			split_lines(_str2, lines_2, offs_2);

			solve(lines_1, lines_2, v_snapshots, true, keep_line_treshold_percent);
			construct_edit_script(v_snapshots, lines_1, lines_2, offs_1, offs_2);
		}
		else {
			solve(_str1, _str2, v_snapshots, false, 0);
			construct_edit_script(_str1.length(), _str2.length(), v_snapshots);
		}
	}

	int Diff::compare_elements(char chr1, char chr2) {
		return chr1 == chr2 ? -MAXCOST : MAXCOST;
	}

	int Diff::compare_elements(const std::string& str1, const std::string& str2) {
		if (str1 == str2) {
			return -MAXCOST;
		}

		Diff sub_diff(str1, str2);

		int chars_keep = sub_diff.editScript.get_char_count(Action::Type::keep);
		int chars_rem = sub_diff.editScript.get_char_count(Action::Type::remove);
		int chars_ins = sub_diff.editScript.get_char_count(Action::Type::insert);

		int chars_total = chars_keep + chars_rem + chars_ins;

		if (chars_keep > 0) {
			return ((chars_rem + chars_ins) * MAXCOST) / chars_total - chars_keep * MAXCOST / chars_total;
		}
		return MAXCOST;
	}

	template<typename Container>
	void Diff::solve(const Container& str1, const Container& str2, std::vector<npArray>& v_snapshots, bool linewise, int keep_line_treshold_percent) {
		if (str1.empty() && str2.empty()) {
			return;
		}

		int len1 = (int)str1.size();
		int len2 = (int)str2.size();

		npArray V(-(len1 + len2), len1 + len2);
		V[1] = progress_val(0, 0);
		bool cont = true;

		int keep_line_treshold_cost = (MAXCOST * keep_line_treshold_percent / 100) - (MAXCOST * (100 - keep_line_treshold_percent) / 100);

		for (int d = 0; d <= len1 + len2 && cont; d++)
		{
			for (int k = -d; k <= d; k += 2)
			{
				// down or right?
				bool down = (k == -d || (k != d && V[k - 1] < V[k + 1]));

				int kPrev = down ? k + 1 : k - 1;

				int cost = V[kPrev].cost + MAXCOST;

				// start point
				int xStart = V[kPrev].x;
				int yStart = xStart - kPrev;

				// mid point
				int xMid = down ? xStart : xStart + 1;
				int yMid = xMid - k;

				// end point
				int xEnd = xMid;
				int yEnd = yMid;

				// follow diagonal
				while (xEnd < len1 && yEnd < len2) {
					int cur_cost = compare_elements(str1[xEnd], str2[yEnd]);
					if (cur_cost > keep_line_treshold_cost) {
						break;
					}
					cost += cur_cost;
					xEnd++; yEnd++;
				}

				// save end point
				V[k] = progress_val(xEnd, cost);

				// check for solution
				if (xEnd >= len1 && yEnd >= len2) {
					/* solution has been found */
					cont = false;
					break;
				}
			}

			v_snapshots.emplace_back(V, -d, d);
		}
	}

	void Diff::construct_edit_script(int len1, int len2, std::vector<npArray>& v_snapshots) {
		Point p = Point(len1, len2); // start at the end
		ES::Actions& solution = this->editScript.actions;

		for (int d = (int)(v_snapshots.size() - 1); p.x > 0 || p.y > 0; d--)
		{
			npArray& V = v_snapshots[d];

			int k = p.x - p.y;

			// end point is in V
			int xEnd = V[k].x;
			int yEnd = xEnd - k;

			// down or right?
			bool down = (k == -d || (k != d && V[k - 1] < V[k + 1]));

			int kPrev = down ? k + 1 : k - 1;

			int xStart, yStart, xMid, yMid;
			if (d == 0) {
				xStart = 0;
				yStart = -1;

				xMid = 0;
				yMid = 0;
			}
			else {
				xStart = V[kPrev].x;
				yStart = xStart - kPrev;

				// mid point
				xMid = down ? xStart : xStart + 1;
				yMid = xMid - k;
			}

			if (xEnd != xMid) { //keep from A
				Action act = Action(Action::Type::keep, xMid, (size_t)xEnd - xMid);
				solution.insert(solution.begin(), act);
			}

			if (down) { //insert from B
				if (yMid > 0) { //ignore the stub starting point (V[1]=0)
					size_t off = (size_t)yMid - 1;

					if (solution.size() > 0 && solution[0].type == Action::Type::insert) {
						solution[0].offset = off;
						solution[0].count++;
					}
					else {
						Action act = Action(Action::Type::insert, off, 1);
						solution.insert(solution.begin(), act);
					}
				}
			}
			else { //remove from A
				size_t off = (size_t)xMid - 1;

				if (solution.size() > 0 && solution[0].type == Action::Type::remove) {
					solution[0].offset = off;
					solution[0].count++;
				}
				else {
					Action act = Action(Action::Type::remove, off, 1);
					solution.insert(solution.begin(), act);
				}
			}

			p.x = xStart;
			p.y = yStart;
		}
	}

	void Diff::construct_edit_script(std::vector<npArray>& v_snapshots, std::vector<std::string>& lines_1, std::vector<std::string>& lines_2, std::vector<size_t>& line_offs_1, std::vector<size_t>& line_offs_2) {
		int len1 = lines_1.size();
		int len2 = lines_2.size();

		Point p = Point(len1, len2); // start at the end
		ES::Actions& solution = this->editScript.actions;

		bool last_line = true;
		auto ins_lb = [&last_line](ES::Actions& acts, int lb_type) {
			if (!last_line) {
				acts.emplace_back(Action::Type::lineBreakType +  lb_type, 0, 0);
			}
			last_line = false;
		};

		for (int d = (int)(v_snapshots.size() - 1); p.x > 0 || p.y > 0; d--)
		{
			npArray& V = v_snapshots[d];

			int k = p.x - p.y;

			// end point is in V
			int xEnd = V[k].x;
			int yEnd = xEnd - k;

			// down or right?
			bool down = (k == -d || (k != d && V[k - 1] < V[k + 1]));

			int kPrev = down ? k + 1 : k - 1;

			int xStart, yStart, xMid, yMid;
			if (d == 0) {
				xStart = 0;
				yStart = -1;

				xMid = 0;
				yMid = 0;
			}
			else {
				xStart = V[kPrev].x;
				yStart = xStart - kPrev;

				// mid point
				xMid = down ? xStart : xStart + 1;
				yMid = xMid - k;
			}

			ES::Actions new_acts;

			if (down) { //insert from B
				if (yMid > 0) { //ignore the stub starting point (V[1]=0)
					size_t off = (size_t)yMid - 1;

					new_acts.emplace_back(Action::Type::insert, line_offs_2[off], lines_2[off].length());
				}
			}
			else { //remove from A
				size_t off = (size_t)xMid - 1;

				new_acts.emplace_back(Action::Type::remove, line_offs_1[off], lines_1[off].length());
			}

			if (xEnd != xMid) { //keep from A
				for (int i = xMid, j = yMid; i < xEnd; i++, j++) {
					if (lines_1[i] == lines_2[j]) {
						new_acts.emplace_back(Action::Type::keep, line_offs_1[i], lines_1[i].size());
					}
					else {
						Diff sub_diff(lines_1[i], lines_2[j]);
						EditScript& sub_es = sub_diff.get_edit_script();

						for (Action& cur_act : sub_es.actions) {
							int act_line_no = cur_act.get_target_str(i, j);
							size_t act_off = cur_act.get_target_str(line_offs_1, line_offs_2)[act_line_no];

							cur_act.offset += act_off;

							new_acts.push_back(cur_act);
						}
					}
				}
			}

			solution.insert(solution.begin(), new_acts.begin(), new_acts.end());

			p.x = xStart;
			p.y = yStart;
		}
	}

	Diff::EditScript& Diff::get_edit_script() {
		return this->editScript;
	}

#ifdef _DEBUG
	void Diff::selfTest(int count) {
		auto ranStr = [](size_t length) -> std::string
		{
			auto randchar = []() -> char
			{
				const char charset[] =
					"0123456789"
					"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
					"abcdefghijklmnopqrstuvwxyz"
					"\n";
				const size_t max_index = (sizeof(charset) - 1);
				return charset[rand() % max_index];
			};
			std::string str(length, 0);
			std::generate_n(str.begin(), length, randchar);
			return str;
		};
		bool lw = true;
		int th = 40;
		for (int i = 0; i < count; i++, lw = !lw) {
			std::string s1, s2;
			switch (i) {
				case 0: {
					s1 = "";
					s2 = "";
					break;
				}
				case 1: {
					s1 = "";
					s2 = "Awmaaa";
					break;
				}
				case 2: {
					s1 = "aweowae";
					s2 = "";
					break;
				}
				case 3: {
					s1 = "A\nB\nCCCCDA\nAWawAWE";
					s2 = "DBCCAWEAAaWPAK";
					lw = true;
					th = 40;
					break;
				}
				case 4: {
					s1 = "ABCCCCDAAWawAWE";
					s2 = "D\nBCCAWEAAaWPAK\n";
					lw = true;
					th = 40;
					break;
				}
				case 5: {
					s1 = "ABCCCCD\nAAWaw\nAWE";
					s2 = "D\nBCCAWE\nAAaWPAK";
					lw = true;
					th = 40;
					break;
				}
				case 6: {
					s1 = "A\nB\nCCCCDAAWawAWE\n";
					s2 = "DBCCAWE\nAAaWPAK\n";
					lw = true;
					th = 40;
					break;
				}
				default: {
					 s1 = ranStr(rand() % 200);
					 s2 = ranStr(rand() % 200);

					 if (lw) {
						 th = rand() % 100 + 1;
					 }
				}
			}

			Diff diff(s1, s2, lw, th);

			std::string testStr = "";

			for (Diff::EditScript::Action& cur_action : diff.get_edit_script().actions) {
				switch (cur_action.type)
				{
					case Diff::EditScript::Action::Type::insert: {
						testStr += s2.substr(cur_action.offset, cur_action.count) ;
						break;
					}
					case Diff::EditScript::Action::Type::keep: {
						testStr += s1.substr(cur_action.offset, cur_action.count);
						break;
					}
					case Diff::EditScript::Action::Type::lineBreak:
					case Diff::EditScript::Action::Type::insert_lineBreak:
					{
						testStr += "\n";
					}
				}
			}

			assert(testStr == s2);
		}
	}
#endif
}
