#ifndef slic3r_GCodeProcessor_hpp_
#define slic3r_GCodeProcessor_hpp_

#if ENABLE_GCODE_PROCESSOR
#include "PrintConfig.hpp"
#include "ExtrusionEntity.hpp"

#include <string>

namespace Slic3r {

    class GCodeConfig;
    class DynamicPrintConfig;

    class GCodeLine
    {
        std::string m_raw;
        float       m_axis[NUM_AXES];
        uint32_t    m_mask;

    public:
        GCodeLine() { reset(); }
        explicit GCodeLine(const std::string& raw) { reset(); m_raw = raw; }

        void reset();

        bool  has(Axis axis) const { return (m_mask & (1 << int(axis))) != 0; }
        float value(Axis axis) const { return m_axis[axis]; }

        bool  has(char axis) const;
        float value(char axis) const;

        std::string cmd() const;
        std::string comment() const;

        const std::string& raw() const { return m_raw; }
        std::string to_string() const;

    private:
        void set_axis(Axis axis, float value);

        friend class GCodeParser;
    };

    class GCodeParser
    {
        char m_extrusion_axis;

    public:
        typedef std::function<void(const GCodeLine&)> Callback;

        GCodeParser();

        void set_extrusion_axis_from_config(const GCodeConfig& config);
//        void set_extrusion_axis_from_config(const DynamicPrintConfig& print_config);

        bool parse_file(const std::string& filename, Callback callback);
        bool parse_line(const char* ptr, GCodeLine& gline, Callback callback);

    private:
        // return false if no data has been parsed (empty line)
        bool parse_line_internal(const char* ptr, GCodeLine& gline);
    };

    class GCodeProcessor
    {
        enum EUnits : unsigned char
        {
            Millimeters,
            Inches
        };

        enum EPositioningType : unsigned char
        {
            Absolute,
            Relative
        };

        enum ETimeEstimateMode : unsigned char
        {
            Normal,
            Silent,
            Num_Modes
        };

    public:
        static const std::string Extrusion_Role_Tag;
        static const std::string Mm3_Per_Mm_Tag;
        static const std::string Width_Tag;
        static const std::string Height_Tag;
        static const std::string Color_Change_Tag;

        struct MachineLimits
        {
            // hard limit for the acceleration, to which the firmware will clamp.
            float max_acceleration;     // mm/s^2
            float retract_acceleration; // mm/s^2

            float minimum_feedrate;        // mm/s
            float minimum_travel_feedrate; // mm/s

            float axis_max_feedrate[NUM_AXES - 1];     // mm/s
            float axis_max_acceleration[NUM_AXES - 1]; // mm/s^2
            float axis_max_jerk[NUM_AXES - 1];         // mm/s

            MachineLimits() { reset(); }
            void reset();
        };

        struct Metadata
        {
            ExtrusionRole extrusion_role;
            float mm3_per_mm;
            float width;     // mm
            float height;    // mm
            float feedrate; // mm/s
            float fan_speed; // percentage
            unsigned int extruder_id;
            unsigned int color_id;

            Metadata() { reset(); }
            Metadata(ExtrusionRole extrusion_role, float mm3_per_mm, float width, float height, float feedrate, float fan_speed, 
                unsigned int extruder_id, unsigned int color_id)
                : extrusion_role(extrusion_role), mm3_per_mm(mm3_per_mm), width(width), height(height), feedrate(feedrate), fan_speed(fan_speed), 
                extruder_id(extruder_id), color_id(color_id) {}
            void reset();
        };

    private:
        typedef std::array<float, NUM_AXES - 1> Position;

        struct GCodeMove
        {
            enum EType : unsigned char
            {
                Noop,
                Retract,
                Unretract,
                Tool_change,
                Move,
                Extrude,
                Num_Types
            };

            EType type;
            Metadata data;

            Position start_position; // mm
            Position end_position;   // mm

            GCodeMove(EType type, const Metadata& data, const Position& start_position, const Position& end_position)
                : type(type), data(data), start_position(start_position), end_position(end_position) {}

            std::string to_string() const;
        };

        struct RepetierStore
        {
            Position position;
            float feedrate;

            RepetierStore() { reset(); }
            void reset();
        };

        // data to calculate color print times
        struct ColorTimes
        {
            bool enabled;
            std::vector<float> times;
            float cache;

            ColorTimes() { reset(); }
            void reset();
            void store_current_cache() { times.push_back(cache); }
        };

        GCodeParser m_parser;
        PrintConfig m_config;
        EUnits m_units;
        EPositioningType m_global_positioning_type;
        EPositioningType m_e_local_positioning_type;
        typedef std::map<unsigned int, Vec2d> ExtrudersOffsetsMap;
        ExtrudersOffsetsMap m_extruders_offsets;
        unsigned int m_extruders_count;
        MachineLimits m_machine_limits[Num_Modes];
        Metadata m_data;
        Position m_start_position;       // mm
        Position m_end_position;         // mm
        Position m_origin;               // mm
        float m_acceleration[Num_Modes]; // mm/s^2
        float m_extrude_factor_override_percentage;

        float m_additional_time; // s
        // Additional load / unload times for a filament exchange sequence.
        std::vector<double> m_filament_load_times;
        std::vector<double> m_filament_unload_times;

        RepetierStore m_repetier_store;
        ColorTimes m_color_times;

        std::vector<GCodeMove> m_moves;

    public:
        GCodeProcessor() { reset(); }

        void reset();

        GCodeFlavor get_gcode_flavor() const { return m_config.gcode_flavor; }
        void set_gcode_flavor(GCodeFlavor flavor) { m_config.gcode_flavor.value = flavor; }

        const std::map<unsigned int, Vec2d>& get_extruders_offsets() const { return m_extruders_offsets; }
        void set_extruders_offsets(const std::map<unsigned int, Vec2d>& offsets) { m_extruders_offsets = offsets; }

        unsigned int get_extruders_count() const { return m_extruders_count; }
        void set_extruders_count(unsigned int count) { m_extruders_count = count; }

        void apply_config(const PrintConfig& config);
//        void apply_config(const DynamicPrintConfig& config);

        const MachineLimits& get_machine_limits(ETimeEstimateMode mode) const { return m_machine_limits[mode]; }
        void set_machine_limits(const MachineLimits& limits, ETimeEstimateMode mode) { m_machine_limits[mode] = limits; }

        // Process the gcode contained in the file with the given filename
        // Return false if any error occourred
        bool process_file(const std::string& filename);

    private:
        bool process_gcode_line(const GCodeLine& line);

        // Move
        bool process_G1(const GCodeLine& line);
        // Dwell
        bool process_G4(const GCodeLine& line);
        // Retract
        bool process_G10(const GCodeLine& line);
        // Unretract
        bool process_G11(const GCodeLine& line);
        // Set Units to Inches
        bool process_G20(const GCodeLine& line);
        // Set Units to Millimeters
        bool process_G21(const GCodeLine& line);
        // Firmware controlled Retract
        bool process_G22(const GCodeLine& line);
        // Firmware controlled Unretract
        bool process_G23(const GCodeLine& line);
        // Set to Absolute Positioning
        bool process_G90(const GCodeLine& line);
        // Set to Relative Positioning
        bool process_G91(const GCodeLine& line);
        // Set Position
        bool process_G92(const GCodeLine& line);

        // Sleep or Conditional stop
        bool process_M1(const GCodeLine& line);
        // Set extruder to absolute mode
        bool process_M82(const GCodeLine& line);
        // Set extruder to relative mode
        bool process_M83(const GCodeLine& line);
        // Set fan speed
        bool process_M106(const GCodeLine& line);
        // Disable fan
        bool process_M107(const GCodeLine& line);
        // Sailfish: Set tool
        bool process_M108(const GCodeLine& line);
        // MakerWare: Set tool
        bool process_M135(const GCodeLine& line);
        // Set max printing acceleration
        bool process_M201(const GCodeLine& line);
        // Set maximum feedrate
        bool process_M203(const GCodeLine& line);
        // Set default acceleration
        bool process_M204(const GCodeLine& line);
        // Advanced settings
        bool process_M205(const GCodeLine& line);
        // Set extrude factor override percentage
        bool process_M221(const GCodeLine& line);
        // Repetier: Store x, y and z position
        bool process_M401(const GCodeLine& line);
        // Repetier: Go to stored position
        bool process_M402(const GCodeLine& line);
        // Set allowable instantaneous speed change
        bool process_M566(const GCodeLine& line);
        // Unload the current filament into the MK3 MMU2 unit at the end of print.
        bool process_M702(const GCodeLine& line);

        // Select Tool
        bool process_T(const GCodeLine& line);

        // Process tags embedded into comments
        bool process_gcode_comment(const GCodeLine& line);

        // Processes extrusion role tag
        bool process_extrusion_role_tag(const std::string& comment, size_t pos);

        // Processes mm3_per_mm tag
        bool process_mm3_per_mm_tag(const std::string& comment, size_t pos);

        // Processes width tag
        bool process_width_tag(const std::string& comment, size_t pos);

        // Processes height tag
        bool process_height_tag(const std::string& comment, size_t pos);

        // Processes color change tag
        bool process_color_change_tag();

        EUnits get_units() const { return m_units; }
        void set_units(EUnits units) { m_units = units; }

        EPositioningType get_global_positioning_type() const { return m_global_positioning_type; }
        void set_global_positioning_type(EPositioningType type) { m_global_positioning_type = type; }

        EPositioningType get_e_local_positioning_type() const { return m_e_local_positioning_type; }
        void set_e_local_positioning_type(EPositioningType type) { m_e_local_positioning_type = type; }

        float get_feedrate() const { return m_data.feedrate; }
        void set_feedrate(float feedrate) { m_data.feedrate = feedrate; }

        unsigned int get_extruder_id() const { return m_data.extruder_id; }
        void set_extruder_id(unsigned int id) { m_data.extruder_id = id; }

        ExtrusionRole get_extrusion_role() const { return m_data.extrusion_role; }
        void set_extrusion_role(ExtrusionRole role) { m_data.extrusion_role = role; }

        float get_mm3_per_mm() const { return m_data.mm3_per_mm; }
        void set_mm3_per_mm(float value) { m_data.mm3_per_mm = value; }

        float get_width() const { return m_data.width; }
        void set_width(float width) { m_data.width = width; }

        float get_height() const { return m_data.height; }
        void set_height(float height) { m_data.height = height; }

        float get_fan_speed() const { return m_data.fan_speed; }
        void set_fan_speed(float fan_speed_percentage) { m_data.fan_speed = fan_speed_percentage; }

        unsigned int get_color_id() const { return m_data.color_id; }
        void set_color_id(unsigned int id) { m_data.color_id = id; }

        float get_extrude_factor_override_percentage() const { return m_extrude_factor_override_percentage; }
        void set_extrude_factor_override_percentage(float percentage) { m_extrude_factor_override_percentage = percentage; }

        float get_additional_time() const { return m_additional_time; }
        void set_additional_time(float time) { m_additional_time = time; }

        float get_filament_load_time(unsigned int id_extruder);
        void set_filament_load_times(const std::vector<double>& filament_load_times) { m_filament_load_times = filament_load_times; }

        float get_filament_unload_time(unsigned int id_extruder);
        void set_filament_unload_times(const std::vector<double>& filament_unload_times) { m_filament_unload_times = filament_unload_times; }

        // Maximum acceleration for the machine. The firmware simulator will clamp the M204 Sxxx to this maximum.
        float get_max_acceleration(ETimeEstimateMode mode) const { return m_machine_limits[mode].max_acceleration; }
        void set_max_acceleration(float acceleration);

        float get_retract_acceleration(ETimeEstimateMode mode) const { return m_machine_limits[mode].retract_acceleration; }
        void set_retract_acceleration(float acceleration);

        float get_minimum_feedrate(ETimeEstimateMode mode) const { return m_machine_limits[mode].minimum_feedrate; }
        void set_minimum_feedrate(float feedrate);

        float get_minimum_travel_feedrate(ETimeEstimateMode mode) const { return m_machine_limits[mode].minimum_travel_feedrate; }
        void set_minimum_travel_feedrate(float feedrate);

        float get_axis_max_feedrate(Axis axis, ETimeEstimateMode mode) const { return m_machine_limits[mode].axis_max_feedrate[axis]; }
        void set_axis_max_feedrate(Axis axis, float feedrate);

        float get_axis_max_acceleration(Axis axis, ETimeEstimateMode mode) const { return m_machine_limits[mode].axis_max_acceleration[axis]; }
        void set_axis_max_acceleration(Axis axis, float acceleration);

        float get_axis_max_jerk(Axis axis, ETimeEstimateMode mode) const { return m_machine_limits[mode].axis_max_jerk[axis]; }
        void set_axis_max_jerk(Axis axis, float jerk);

        const Position& get_start_position() const { return m_start_position; }
        void set_start_position(const Position& position) { m_start_position = position; }

        const Position& get_end_position() const { return m_end_position; }
        void set_end_position(const Position& position) { m_end_position = position; }

        float get_axis_position(Axis axis) const { return m_end_position[axis]; }
        void set_axis_position(Axis axis, float position) { m_end_position[axis] = position; }

        const Position& get_origin() const { return m_origin; }
        void set_origin(const Position& position) { m_origin = position; }

        float get_axis_origin(Axis axis) const { return m_origin[axis]; }
        void set_axis_origin(Axis axis, float position) { m_origin[axis] = position; }

        float get_acceleration(ETimeEstimateMode mode) const { return m_acceleration[mode]; }
        void set_acceleration(float acceleration);

        const Position& get_repetier_store_position() const { return m_repetier_store.position; }
        void set_repetier_store_position(const Position& position) { m_repetier_store.position = position; }

        float get_repetier_store_feedrate() const { return m_repetier_store.feedrate; }
        void set_repetier_store_feedrate(float feedrate) { m_repetier_store.feedrate = feedrate; }

        void enable_color_times(bool enable) { m_color_times.enabled = enable; }
        float get_color_times_cache() const { return m_color_times.cache; }
        void set_color_times_cache(float time) { m_color_times.cache = time; }
        void store_current_color_times_cache() { m_color_times.store_current_cache(); }

        // Checks if the given int is a valid extrusion role (contained into enum ExtrusionRole)
        bool is_valid_extrusion_role(int value) const;

        void store_move(GCodeMove::EType type);
    };

} /* namespace Slic3r */

#endif // ENABLE_GCODE_PROCESSOR

#endif /* slic3r_GCodeProcessor_hpp_ */


