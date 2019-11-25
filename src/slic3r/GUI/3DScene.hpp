#ifndef slic3r_3DScene_hpp_
#define slic3r_3DScene_hpp_

#include "slic3r/GUI/3DEngine.hpp"
#include "slic3r/GUI/GLCanvas3DManager.hpp"

namespace Slic3r {

class _3DScene
{
    static GUI::GLCanvas3DManager s_canvas_mgr;

public:
    static std::string get_gl_info(bool format_as_html, bool extensions);

    static bool add_canvas(wxGLCanvas* canvas, GUI::Bed3D& bed, GUI::Camera& camera, GUI::GLToolbar& view_toolbar);
    static bool remove_canvas(wxGLCanvas* canvas);
    static void remove_all_canvases();

    static bool init(wxGLCanvas* canvas);
    static void destroy();

    static GUI::GLCanvas3D* get_canvas(wxGLCanvas* canvas);

    static void thick_lines_to_verts(const Lines& lines, const std::vector<double>& widths, const std::vector<double>& heights, bool closed, double top_z, GLVolume& volume);
    static void thick_lines_to_verts(const Lines3& lines, const std::vector<double>& widths, const std::vector<double>& heights, bool closed, GLVolume& volume);
	static void extrusionentity_to_verts(const Polyline &polyline, float width, float height, float print_z, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionPath& extrusion_path, float print_z, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionPath& extrusion_path, float print_z, const Point& copy, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionLoop& extrusion_loop, float print_z, const Point& copy, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionMultiPath& extrusion_multi_path, float print_z, const Point& copy, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionEntityCollection& extrusion_entity_collection, float print_z, const Point& copy, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionEntity* extrusion_entity, float print_z, const Point& copy, GLVolume& volume);
    static void polyline3_to_verts(const Polyline3& polyline, double width, double height, GLVolume& volume);
    static void point3_to_verts(const Vec3crd& point, double width, double height, GLVolume& volume);
};

}

#endif
