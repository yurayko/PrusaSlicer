#include "../libslic3r.h"
#include "GCodeProcessor.hpp"

#if ENABLE_GCODE_PROCESSOR
#include <boost/log/trivial.hpp>
#include <fstream>

static const float MMMIN_TO_MMSEC = 1.0f / 60.0f;
static const float INCHES_TO_MM = 25.4f;
static const float MILLISEC_TO_SEC = 0.001f;

static const float DEFAULT_AXIS_MAX_FEEDRATE[] = { 500.0f, 500.0f, 12.0f, 120.0f }; // Prusa Firmware 1_75mm_MK2
static const float DEFAULT_AXIS_MAX_ACCELERATION[] = { 9000.0f, 9000.0f, 500.0f, 10000.0f }; // Prusa Firmware 1_75mm_MK2
static const float DEFAULT_AXIS_MAX_JERK[] = { 10.0f, 10.0f, 0.4f, 2.5f }; // from Prusa Firmware (Configuration.h)
static const float DEFAULT_RETRACT_ACCELERATION = 1500.0f; // Prusa Firmware 1_75mm_MK2
static const float DEFAULT_MINIMUM_FEEDRATE = 0.0f; // from Prusa Firmware (Configuration_adv.h)
static const float DEFAULT_MINIMUM_TRAVEL_FEEDRATE = 0.0f; // from Prusa Firmware (Configuration_adv.h)
static const float DEFAULT_EXTRUDE_FACTOR_OVERRIDE_PERCENTAGE = 1.0f; // 100 percent

static const unsigned int UNLOADED_EXTRUDER_ID = (unsigned int)-1;

namespace Slic3r {

bool is_whitespace(char c) { return (c == ' ') || (c == '\t'); }
bool is_end_of_line(char c) { return (c == '\r') || (c == '\n') || (c == 0); }
bool is_end_of_gcode_line(char c) { return (c == ';') || is_end_of_line(c); }
bool is_end_of_word(char c) { return is_whitespace(c) || is_end_of_gcode_line(c); }
const char* skip_whitespaces(const char* c)
{
    for (; is_whitespace(*c); ++c)
        ; // silence -Wempty-body
    return c;
}
const char* skip_word(const char* c)
{
    for (; !is_end_of_word(*c); ++c)
        ; // silence -Wempty-body
    return c;
}

void GCodeLine::reset()
{
    m_raw.clear();
    ::memset(m_axis, 0, sizeof(m_axis));
    m_mask = 0;
}

bool GCodeLine::has(char axis) const
{
    const char* c = m_raw.c_str();
    // Skip the whitespaces.
    c = skip_whitespaces(c);
    // Skip the command.
    c = skip_word(c);
    // Up to the end of line or comment.
    while (!is_end_of_gcode_line(*c))
    {
        // Skip whitespaces.
        c = skip_whitespaces(c);
        if (is_end_of_gcode_line(*c))
            break;
        // Check the name of the axis.
        if (*c == axis)
            return true;
        // Skip the rest of the word.
        c = skip_word(c);
    }
    return false;
}

float GCodeLine::value(char axis) const
{
    const char* c = m_raw.c_str();
    // Skip the whitespaces.
    c = skip_whitespaces(c);
    // Skip the command.
    c = skip_word(c);
    // Up to the end of line or comment.
    while (!is_end_of_gcode_line(*c))
    {
        // Skip whitespaces.
        c = skip_whitespaces(c);
        if (is_end_of_gcode_line(*c))
            break;
        // Check the name of the axis.
        if (*c == axis)
        {
            // Try to parse the numeric value.
            char* pend = nullptr;
            double  v = strtod(++c, &pend);
            if (pend != nullptr && is_end_of_word(*pend))
                // The axis value has been parsed correctly.
                return float(v);
        }
        // Skip the rest of the word.
        c = skip_word(c);
    }
    return 0.0f;
}

void GCodeLine::set_axis(Axis axis, float value)
{
    m_axis[int(axis)] = value;
    m_mask |= 1 << int(axis);
}

std::string GCodeLine::cmd() const
{
    const char* cmd = skip_whitespaces(m_raw.c_str());
    return std::string(cmd, skip_word(cmd));
}

std::string GCodeLine::comment() const
{
    size_t pos = m_raw.find(';');
    return (pos == std::string::npos) ? "" : m_raw.substr(pos + 1);
}

GCodeParser::GCodeParser()
    : m_extrusion_axis('E')
{
}

void GCodeParser::set_extrusion_axis_from_config(const GCodeConfig& config)
{
    m_extrusion_axis = config.get_extrusion_axis()[0];
}

//void GCodeParser::set_extrusion_axis_from_config(const DynamicPrintConfig& print_config)
//{
//    GCodeConfig config;
//    config.apply(print_config, true);
//    m_extrusion_axis = config.get_extrusion_axis()[0];
//}

bool GCodeParser::parse_file(const std::string& filename, Callback callback)
{
    std::ifstream f(filename);
    if (!f.good())
    {
        BOOST_LOG_TRIVIAL(error) << "Could not open file: " << filename;
        return false;
    }

    std::string line;
    GCodeLine gline;

    while (std::getline(f, line))
    {
        if (!f.good())
        {
            BOOST_LOG_TRIVIAL(error) << "Error while reading file: " << filename;
            f.close();
            return false;
        }

        if (!parse_line(line.c_str(), gline, callback))
        {
            BOOST_LOG_TRIVIAL(error) << "Error while parsing file: " << filename;
            f.close();
            return false;
        }
    }

    f.close();
    return true;
}

bool GCodeParser::parse_line(const char* ptr, GCodeLine& gline, Callback callback)
{
    if (ptr == nullptr)
    {
        BOOST_LOG_TRIVIAL(error) << "Found null pointer while parsing gcode line";
        return false;
    }

    if (parse_line_internal(ptr, gline))
    {
        callback(gline);
        return true;
    }

    return false;
}

bool GCodeParser::parse_line_internal(const char* ptr, GCodeLine& gline)
{
    gline.reset();

    if (ptr == nullptr)
        return false;

    // command and args
    const char* c = ptr;
    {
        // Skip the whitespaces.
        c = skip_whitespaces(c);
        // Skip the command.
        c = skip_word(c);
        // Up to the end of line or comment.
        while (!is_end_of_gcode_line(*c))
        {
            // Skip whitespaces.
            c = skip_whitespaces(c);
            if (is_end_of_gcode_line(*c))
                break;

            // Check axes.
            Axis axis = NUM_AXES;
            switch (*c)
            {
            case 'X': axis = X; break;
            case 'Y': axis = Y; break;
            case 'Z': axis = Z; break;
            case 'F': axis = F; break;
            default:
                if (*c == m_extrusion_axis)
                    axis = E;
                break;
            }
            if (axis != NUM_AXES)
            {
                // Try to parse the numeric value.
                char* pend = nullptr;
                double  v = strtod(++c, &pend);
                if ((pend != nullptr) && is_end_of_word(*pend))
                {
                    // The axis value has been parsed correctly.
                    gline.set_axis(axis, float(v));
                    c = pend;
                }
                else
                    // Skip the rest of the word.
                    c = skip_word(c);
            }
            else
                // Skip the rest of the word.
                c = skip_word(c);
        }
    }

    // Skip the rest of the line.
    for (; !is_end_of_line(*c); ++c);

    // Copy the raw string including the comment, without the trailing newlines.
    if (c > ptr)
        gline.m_raw.assign(ptr, c);

    return true;
}

void GCodeProcessor::MachineLimits::reset()
{
    // Setting the maximum acceleration to zero means that the there is no limit and the G-code
    // is allowed to set excessive values.
    max_acceleration = 0.0f;
    retract_acceleration = DEFAULT_RETRACT_ACCELERATION;

    minimum_feedrate = DEFAULT_MINIMUM_FEEDRATE;
    minimum_travel_feedrate = DEFAULT_MINIMUM_TRAVEL_FEEDRATE;

    ::memcpy((void*)axis_max_feedrate, (const void*)DEFAULT_AXIS_MAX_FEEDRATE, (NUM_AXES - 1) * sizeof(float));
    ::memcpy((void*)axis_max_acceleration, (const void*)DEFAULT_AXIS_MAX_ACCELERATION, (NUM_AXES - 1) * sizeof(float));
    ::memcpy((void*)axis_max_jerk, (const void*)DEFAULT_AXIS_MAX_JERK, (NUM_AXES - 1) * sizeof(float));
}

void GCodeProcessor::Metadata::reset()
{
    extrusion_role = erNone;    
    mm3_per_mm = 0.0f;
    width = 0.0f;
    height = 0.0f;
    feedrate = 0.0f;
    fan_speed = 0.0f;
    // extruder_id is currently used to correctly calculate filament load / unload times 
    // into the total print time. This is currently only really used by the MK3 MMU2:
    // Extruder id (-1) means no filament is loaded yet, all the filaments are parked in the MK3 MMU2 unit.
    extruder_id = UNLOADED_EXTRUDER_ID;
    cp_color_id = 0;
}

void GCodeProcessor::reset()
{
    m_config = PrintConfig();
    m_config.gcode_flavor.value = gcfRepRap;

    m_units = Millimeters;
    m_global_positioning_type = Absolute;
    m_e_local_positioning_type = Absolute;

    m_extruders_offsets.clear();
    m_extruders_count = 1;

    ::memset(m_start_position.data(), 0, sizeof(Position));
    ::memset(m_end_position.data(), 0, sizeof(Position));
    ::memset(m_origin.data(), 0, sizeof(Position));

    m_acceleration[Normal] = 0.0f;
    m_acceleration[Silent] = 0.0f;

    m_extrude_factor_override_percentage = DEFAULT_EXTRUDE_FACTOR_OVERRIDE_PERCENTAGE;

    m_additional_time = 0.0f;

    m_machine_limits[Normal] = MachineLimits();  
    m_machine_limits[Silent] = MachineLimits(); 

    m_data.reset();

    m_filament_load_times.clear();
    m_filament_unload_times.clear();

    m_moves.clear();
}

void GCodeProcessor::apply_config(const PrintConfig& config)
{
    m_config = config;

    m_parser.set_extrusion_axis_from_config(m_config);
    std::map<unsigned int, Vec2d> extruders_offsets;
    for (unsigned int i = 0; i < (unsigned int)m_config.extruder_offset.values.size(); ++i)
    {
        Vec2d offset = m_config.extruder_offset.get_at(i);
        if (!offset.isApprox(Vec2d::Zero()))
            extruders_offsets[i] = offset;
    }
    set_extruders_offsets(extruders_offsets);

    const ConfigOptionStrings* extruders_opt = dynamic_cast<const ConfigOptionStrings*>(m_config.option("extruder_colour"));
    const ConfigOptionStrings* filamemts_opt = dynamic_cast<const ConfigOptionStrings*>(m_config.option("filament_colour"));
    set_extruders_count(std::max((unsigned int)extruders_opt->values.size(), (unsigned int)filamemts_opt->values.size()));

    if (m_config.gcode_flavor.value == gcfMarlin)
    {
        for (int i = Normal; i <= Silent; ++i)
        {
            if (i < (int)m_config.machine_max_acceleration_extruding.values.size())
            {
                MachineLimits& limits = m_machine_limits[i];
                limits.max_acceleration = (float)m_config.machine_max_acceleration_extruding.values[i];
                limits.retract_acceleration = (float)m_config.machine_max_acceleration_retracting.values[i];
                limits.minimum_feedrate = (float)m_config.machine_min_extruding_rate.values[i];
                limits.minimum_travel_feedrate = (float)m_config.machine_min_travel_rate.values[i];
                limits.axis_max_acceleration[X] = (float)m_config.machine_max_acceleration_x.values[i];
                limits.axis_max_acceleration[Y] = (float)m_config.machine_max_acceleration_y.values[i];
                limits.axis_max_acceleration[Z] = (float)m_config.machine_max_acceleration_z.values[i];
                limits.axis_max_acceleration[E] = (float)m_config.machine_max_acceleration_e.values[i];
                limits.axis_max_feedrate[X] = (float)m_config.machine_max_feedrate_x.values[i];
                limits.axis_max_feedrate[Y] = (float)m_config.machine_max_feedrate_y.values[i];
                limits.axis_max_feedrate[Z] = (float)m_config.machine_max_feedrate_z.values[i];
                limits.axis_max_feedrate[E] = (float)m_config.machine_max_feedrate_e.values[i];
                limits.axis_max_jerk[X] = (float)m_config.machine_max_jerk_x.values[i];
                limits.axis_max_jerk[Y] = (float)m_config.machine_max_jerk_y.values[i];
                limits.axis_max_jerk[Z] = (float)m_config.machine_max_jerk_z.values[i];
                limits.axis_max_jerk[E] = (float)m_config.machine_max_jerk_e.values[i];
            }
        }
    }

    // Filament load / unload times are not specific to a firmware flavor. Let anybody use it if they find it useful.
    if (m_config.single_extruder_multi_material)
    {
        // As of now the fields are shown at the UI dialog in the same combo box as the ramming values, so they
        // are considered to be active for the single extruder multi-material printers only.
        m_filament_load_times = m_config.filament_load_time.values;
        m_filament_unload_times = m_config.filament_unload_time.values;
    }
}

//void GCodeProcessor::apply_config(const DynamicPrintConfig& config)
//{
//    m_state.config.apply(config, true);
//    m_parser.set_extrusion_axis_from_config(m_state.config);
//}

bool GCodeProcessor::process_file(const std::string& filename)
{
    return m_parser.parse_file(filename, [this](const GCodeLine& line) { process_gcode_line(line); });
}

bool GCodeProcessor::process_gcode_line(const GCodeLine& line)
{
    // sets new start position
    set_start_position(get_end_position());

    std::string cmd = line.cmd();
    std::string comment = line.comment();

    if (cmd.length() > 1)
    {
        // processes command lines
        switch (::toupper(cmd[0]))
        {
        case 'G':
            {
                switch (::atoi(&cmd[1]))
                {
                // Move
                case 1: { return process_G1(line); }
                // Dwell
                case 4: { return process_G4(line); }
                // Retract
                case 10: { return process_G10(line); }
                // Unretract
                case 11: { return process_G11(line); }
                // Set Units to Inches
                case 20: { return process_G20(line); }
                // Set Units to Millimeters
                case 21: { return process_G21(line); }
                // Firmware controlled Retract
                case 22: { return process_G22(line); }
                // Firmware controlled Unretract
                case 23: { return process_G23(line); }
                // Move to Origin (Home)
                case 28: { return process_G28(line); }
                // Set to Absolute Positioning
                case 90: { return process_G90(line); }
                // Set to Relative Positioning
                case 91: { return process_G91(line); }
                // Set Position
                case 92: { return process_G92(line); }
                default:
                    {
                        BOOST_LOG_TRIVIAL(warning) << "Found unknown gcode command on line: " << line.raw();
                        return true;
                    }
                }
                break;
            }
        case 'M':
            {
                switch (::atoi(&cmd[1]))
                {
                // Sleep or Conditional stop
                case 1: { return process_M1(line); }
                // Set extruder to absolute mode
                case 82: { return process_M82(line); }
                // Set extruder to relative mode
                case 83: { return process_M83(line); }
                // Set fan speed
                case 106: { return process_M106(line); }
                // Disable fan
                case 107: { return process_M107(line); }
                // Sailfish: Set tool
                case 108: { return process_M108(line); }
                // MakerWare: Set tool
                case 135: { return process_M135(line); }
                // Set max printing acceleration
                case 201: { return process_M201(line); }
                // Set maximum feedrate
                case 203: { return process_M203(line); }
                // Set default acceleration
                case 204: { return process_M204(line); }
                // Advanced settings
                case 205: { return process_M205(line); }
                // Set extrude factor override percentage
                case 221: { return process_M221(line); }
                // Repetier: Store x, y and z position
                case 401: { return process_M401(line); }
                // Repetier: Go to stored position
                case 402: { return process_M402(line); }
                // Set allowable instantaneous speed change
                case 566: { return process_M566(line); }
                // Unload the current filament into the MK3 MMU2 unit at the end of print.
                case 702: { return process_M702(line); }
                default:
                    {
                        BOOST_LOG_TRIVIAL(warning) << "Found unknown gcode command on line: " << line.raw();
                        return true;
                    }
                }
                break;
            }
        // Select Tool
        case 'T': { return process_T(line); }
        default:
            {
                BOOST_LOG_TRIVIAL(error) << "Found unknown gcode command on line: " << line.raw();
                return false;
            }
        }
    }
    else if (comment.length() > 1)
    {
        // processes comment lines
        return process_gcode_comment(line);
    }

    return false;
}

bool GCodeProcessor::process_G1(const GCodeLine& line)
{
    // GCodeAnalyzer
    // GCodeTimeEstimator

    std::cout << "G1" << std::endl;
    return true;
}

bool GCodeProcessor::process_G4(const GCodeLine& line)
{
    GCodeFlavor flavor = get_gcode_flavor();

    if (line.has('P'))
        set_additional_time(get_additional_time() + line.value('P') * MILLISEC_TO_SEC);

    // see: http://reprap.org/wiki/G-code#G4:_Dwell
    if ((flavor == gcfRepetier) ||
        (flavor == gcfMarlin) ||
        (flavor == gcfSmoothie) ||
        (flavor == gcfRepRap))
    {
        if (line.has('S'))
            set_additional_time(get_additional_time() + line.value('P'));
    }

//    _simulate_st_synchronize();

    return true;
}

bool GCodeProcessor::process_G10(const GCodeLine& line)
{
    // stores retract move
    store_move(GCodeMove::Retract);

    return true;
}

bool GCodeProcessor::process_G11(const GCodeLine& line)
{
    // stores unretract move
    store_move(GCodeMove::Unretract);

    return true;
}

bool GCodeProcessor::process_G20(const GCodeLine& line)
{
    set_units(Inches);
    return true;
}

bool GCodeProcessor::process_G21(const GCodeLine& line)
{
    set_units(Millimeters);
    return true;
}

bool GCodeProcessor::process_G22(const GCodeLine& line)
{
    // stores retract move
    store_move(GCodeMove::Retract);

    return true;
}

bool GCodeProcessor::process_G23(const GCodeLine& line)
{
    // stores unretract move
    store_move(GCodeMove::Unretract);

    return true;
}

bool GCodeProcessor::process_G28(const GCodeLine& line)
{
    // GCodeTimeEstimator

    std::cout << "G28" << std::endl;
    return true;
}

bool GCodeProcessor::process_G90(const GCodeLine& line)
{
    set_global_positioning_type(Absolute);
    return true;
}

bool GCodeProcessor::process_G91(const GCodeLine& line)
{
    set_global_positioning_type(Relative);
    return true;
}

bool GCodeProcessor::process_G92(const GCodeLine& line)
{
    float lengthsScaleFactor = (get_units() == Inches) ? INCHES_TO_MM : 1.0f;
    bool anyFound = false;

    for (unsigned char i = X; i <= E; ++i)
    {
        Axis a = (Axis)i;
        if (line.has(a))
        {
            set_axis_origin(a, get_axis_position(a) - line.value(a) * lengthsScaleFactor);
            anyFound = true;
        }
        else if (a == E)
        {
//            _simulate_st_synchronize();
        }
    }

//    if (line.has(X))
//    {
//        set_axis_origin(X, get_axis_position(X) - line.value(X) * lengthsScaleFactor);
//        anyFound = true;
//    }
//
//    if (line.has(Y))
//    {
//        set_axis_origin(Y, get_axis_position(Y) - line.value(Y) * lengthsScaleFactor);
//        anyFound = true;
//    }
//
//    if (line.has(Z))
//    {
//        set_axis_origin(Z, get_axis_position(Z) - line.value(Z) * lengthsScaleFactor);
//        anyFound = true;
//    }
//
//    if (line.has(E))
//    {
//        set_axis_origin(E, get_axis_position(E) - line.value(E) * lengthsScaleFactor);
//        anyFound = true;
//    }

    if (!anyFound)
    {
        set_origin(get_end_position());
//        for (unsigned char i = X; i <= E; ++i)
//        {
//            Axis a = (Axis)i;
//            set_axis_origin(a, get_axis_position(a));
//        }
    }

    return true;
}

bool GCodeProcessor::process_M1(const GCodeLine& line)
{
    // GCodeTimeEstimator

    std::cout << "M1" << std::endl;
    return true;
}

bool GCodeProcessor::process_M82(const GCodeLine& line)
{
    set_e_local_positioning_type(Absolute);
    return true;
}

bool GCodeProcessor::process_M83(const GCodeLine& line)
{
    set_e_local_positioning_type(Relative);
    return true;
}

bool GCodeProcessor::process_M106(const GCodeLine& line)
{
    // GCodeAnalyzer

    std::cout << "M106" << std::endl;
    return true;
}

bool GCodeProcessor::process_M107(const GCodeLine& line)
{
    // GCodeAnalyzer

    std::cout << "M107" << std::endl;
    return true;
}

bool GCodeProcessor::process_M108(const GCodeLine& line)
{
    // GCodeAnalyzer

    std::cout << "M108" << std::endl;
    return true;
}

bool GCodeProcessor::process_M135(const GCodeLine& line)
{
    // GCodeAnalyzer

    std::cout << "M135" << std::endl;
    return true;
}

bool GCodeProcessor::process_M201(const GCodeLine& line)
{
    GCodeFlavor flavor = get_gcode_flavor();

    // see http://reprap.org/wiki/G-code#M201:_Set_max_printing_acceleration
    float factor = ((flavor != gcfRepRap) && (get_units() == Inches)) ? INCHES_TO_MM : 1.0f;

    for (unsigned char i = X; i <= E; ++i)
    {
        Axis a = (Axis)i;
        if (line.has(a))
            set_axis_max_acceleration(a, line.value(a) * factor);
    }

//    if (line.has(X))
//        set_axis_max_acceleration(X, line.value(X) * factor);
//
//    if (line.has(Y))
//        set_axis_max_acceleration(Y, line.value(Y) * factor);
//
//    if (line.has(Z))
//        set_axis_max_acceleration(Z, line.value(Z) * factor);
//
//    if (line.has(E))
//        set_axis_max_acceleration(E, line.value(E) * factor);

    return true;
}

bool GCodeProcessor::process_M203(const GCodeLine& line)
{
    GCodeFlavor flavor = get_gcode_flavor();

    // see http://reprap.org/wiki/G-code#M203:_Set_maximum_feedrate
    if (flavor == gcfRepetier)
        return true;

    // see http://reprap.org/wiki/G-code#M203:_Set_maximum_feedrate
    // http://smoothieware.org/supported-g-codes
    float factor = ((flavor == gcfMarlin) || (flavor == gcfSmoothie)) ? 1.0f : MMMIN_TO_MMSEC;

    for (unsigned char i = X; i <= E; ++i)
    {
        Axis a = (Axis)i;
        if (line.has(a))
            set_axis_max_feedrate(a, line.value(a) * factor);
    }

//    if (line.has(X))
//        set_axis_max_feedrate(X, line.value(X) * factor);
//
//    if (line.has(Y))
//        set_axis_max_feedrate(Y, line.value(Y) * factor);
//
//    if (line.has(Z))
//        set_axis_max_feedrate(Z, line.value(Z) * factor);
//
//    if (line.has(E))
//        set_axis_max_feedrate(E, line.value(E) * factor);

    return true;
}

bool GCodeProcessor::process_M204(const GCodeLine& line)
{
    if (line.has('S'))
    {
        float value = line.value('S');
        // Legacy acceleration format. This format is used by the legacy Marlin, MK2 or MK3 firmware,
        // and it is also generated by Slic3r to control acceleration per extrusion type
        // (there is a separate acceleration settings in Slicer for perimeter, first layer etc).
        set_acceleration(value);
        if (line.has('T'))
            set_retract_acceleration(line.value('T'));
    }
    else
    {
        // New acceleration format, compatible with the upstream Marlin.
        if (line.has('P'))
            set_acceleration(line.value('P'));
        if (line.has('R'))
            set_retract_acceleration(line.value('R'));
        if (line.has('T'))
        {
            // Interpret the T value as the travel acceleration in the new Marlin format.
            //FIXME Prusa3D firmware currently does not support travel acceleration value independent from the extruding acceleration value.
            // set_travel_acceleration(line.value('T'));
        }
    }

    return true;
}

bool GCodeProcessor::process_M205(const GCodeLine& line)
{
    if (line.has(X))
    {
        float max_jerk = line.value(X);
        set_axis_max_jerk(X, max_jerk);
        set_axis_max_jerk(Y, max_jerk);
    }

    if (line.has(Y))
        set_axis_max_jerk(Y, line.value(Y));

    if (line.has(Z))
        set_axis_max_jerk(Z, line.value(Z));

    if (line.has(E))
        set_axis_max_jerk(E, line.value(E));

    if (line.has('S'))
        set_minimum_feedrate(line.value('S'));

    if (line.has('T'))
        set_minimum_travel_feedrate(line.value('T'));

    return true;
}

bool GCodeProcessor::process_M221(const GCodeLine& line)
{
    if (line.has('S') && !line.has('T'))
        set_extrude_factor_override_percentage(line.value('S') * 0.01f);

    return true;
}

bool GCodeProcessor::process_M401(const GCodeLine& line)
{
    // GCodeAnalyzer

    std::cout << "M401" << std::endl;
    return true;
}

bool GCodeProcessor::process_M402(const GCodeLine& line)
{
    // GCodeAnalyzer

    std::cout << "M402" << std::endl;
    return true;
}

bool GCodeProcessor::process_M566(const GCodeLine& line)
{
    for (unsigned char i = X; i <= E; ++i)
    {
        Axis a = (Axis)i;
        if (line.has(a))
            set_axis_max_jerk(a, line.value(a) * MMMIN_TO_MMSEC);
    }

//    if (line.has(X))
//        set_axis_max_jerk(X, line.value(X) * MMMIN_TO_MMSEC);
//
//    if (line.has(Y))
//        set_axis_max_jerk(Y, line.value(Y) * MMMIN_TO_MMSEC);
//
//    if (line.has(Z))
//        set_axis_max_jerk(Z, line.value(Z) * MMMIN_TO_MMSEC);
//
//    if (line.has(E))
//        set_axis_max_jerk(E, line.value(E) * MMMIN_TO_MMSEC);

    return true;
}

bool GCodeProcessor::process_M702(const GCodeLine& line)
{
    if (line.has('C'))
    {
        // MK3 MMU2 specific M code:
        // M702 C is expected to be sent by the custom end G-code when finalizing a print.
        // The MK3 unit shall unload and park the active filament into the MMU2 unit.
        set_additional_time(get_additional_time() + get_filament_unload_time(get_extruder_id()));
        set_extruder_id(UNLOADED_EXTRUDER_ID);
//        _simulate_st_synchronize();
    }

    return true;
}

bool GCodeProcessor::process_T(const GCodeLine& line)
{
    std::string cmd = line.cmd();

    if (cmd.length() > 1)
    {
        unsigned int curr_id = get_extruder_id();
        unsigned int new_id = (unsigned int)::strtol(cmd.substr(1).c_str(), nullptr, 10);
        if (curr_id != new_id)
        {
            if (new_id >= m_extruders_count)
            {
//                if (m_extruders_count > 1)
                    BOOST_LOG_TRIVIAL(error) << "GCodeProcessor encountered an invalid toolchange, maybe from a custom gcode.";
            }
            else
            {
                // Specific to the MK3 MMU2: The initial extruder ID is set to -1 indicating
                // that the filament is parked in the MMU2 unit and there is nothing to be unloaded yet.
                set_additional_time(get_additional_time() + get_filament_unload_time(curr_id));
                set_extruder_id(new_id);
                set_additional_time(get_additional_time() + get_filament_load_time(new_id));
//                _simulate_st_synchronize();
            }

            // stores tool change move
            store_move(GCodeMove::Tool_change);
        }
    }

    return true;
}

bool GCodeProcessor::process_gcode_comment(const GCodeLine& line)
{
    // GCodeAnalyzer
    // GCodeTimeEstimator

    std::cout << "tags" << std::endl;
    return true;
}

float GCodeProcessor::get_filament_load_time(unsigned int id_extruder)
{
    return
        (m_filament_load_times.empty() || (id_extruder == UNLOADED_EXTRUDER_ID)) ?
        0.0f :
        (m_filament_load_times.size() <= id_extruder) ?
        (float)m_filament_load_times.front() :
        (float)m_filament_load_times[id_extruder];
}

float GCodeProcessor::get_filament_unload_time(unsigned int id_extruder)
{
    return
        (m_filament_unload_times.empty() || (id_extruder == UNLOADED_EXTRUDER_ID)) ?
        0.0f :
        (m_filament_unload_times.size() <= id_extruder) ?
        (float)m_filament_unload_times.front() :
        (float)m_filament_unload_times[id_extruder];
}

void GCodeProcessor::set_max_acceleration(float acceleration)
{
    for (int i = Normal; i <= Silent; ++i)
    {
        m_machine_limits[i].max_acceleration = acceleration;

        if (acceleration > 0.0f)
            m_acceleration[i] = acceleration;
    }
}

void GCodeProcessor::set_retract_acceleration(float acceleration)
{
    for (int i = Normal; i <= Silent; ++i)
    {
        m_machine_limits[i].retract_acceleration = acceleration;
    }
}

void GCodeProcessor::set_minimum_feedrate(float feedrate)
{
    for (int i = Normal; i <= Silent; ++i)
    {
        m_machine_limits[i].minimum_feedrate = feedrate;
    }
}

void GCodeProcessor::set_minimum_travel_feedrate(float feedrate)
{
    for (int i = Normal; i <= Silent; ++i)
    {
        m_machine_limits[i].minimum_travel_feedrate = feedrate;
    }
}

void GCodeProcessor::set_axis_max_feedrate(Axis axis, float feedrate)
{
    for (int i = Normal; i <= Silent; ++i)
    {
        m_machine_limits[i].axis_max_feedrate[axis] = feedrate;
    }
}

void GCodeProcessor::set_axis_max_acceleration(Axis axis, float acceleration)
{
    for (int i = Normal; i <= Silent; ++i)
    {
        m_machine_limits[i].axis_max_acceleration[axis] = acceleration;
    }
}

void GCodeProcessor::set_axis_max_jerk(Axis axis, float jerk)
{
    for (int i = Normal; i <= Silent; ++i)
    {
        m_machine_limits[i].axis_max_jerk[axis] = jerk;
    }
}

void GCodeProcessor::set_acceleration(float acceleration)
{
    for (int i = Normal; i <= Silent; ++i)
    {
        m_acceleration[i] = (m_machine_limits[i].max_acceleration == 0.0f) ?
            acceleration :
            // Clamp the acceleration with the maximum.
            std::min(m_machine_limits[i].max_acceleration, acceleration);
    }
}

void GCodeProcessor::store_move(GCodeMove::EType type)
{
    Metadata data(get_extrusion_role(), get_mm3_per_mm(), get_width(), get_height(), get_feedrate(), get_fan_speed(), get_extruder_id(), get_cp_color_id());

    ExtrudersOffsetsMap::iterator extr_it = m_extruders_offsets.find(get_extruder_id());
    Vec2d extruder_offset = (extr_it != m_extruders_offsets.end()) ? extr_it->second : Vec2d::Zero();
    Position start_position = get_start_position();
    start_position[0] += (float)extruder_offset(0);
    start_position[1] += (float)extruder_offset(1);
    Position end_position = get_end_position();
    end_position[0] += (float)extruder_offset(0);
    end_position[1] += (float)extruder_offset(1);

    m_moves.emplace_back(type, data, start_position, end_position);
}

} /* namespace Slic3r */

#endif // ENABLE_GCODE_PROCESSOR
