#include "../libslic3r.h"
#include "GCodeProcessor.hpp"

#if ENABLE_GCODE_PROCESSOR
#include <boost/log/trivial.hpp>
#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
#include <boost/filesystem/path.hpp>
#else
#include <boost/nowide/fstream.hpp>
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
#include <float.h>

#include "Utils.hpp"

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

static const float PREVIOUS_FEEDRATE_THRESHOLD = 0.0001f;

std::string to_string(const Slic3r::GCodeProcessor::AxesTuple& position)
{
    std::string ret = "(";
    for (int i = Slic3r::X; i <= Slic3r::E; ++i)
    {
        if (i > Slic3r::X)
            ret += ", ";

        ret += std::to_string(position[i]);
    }

    ret += ")";
    return ret;
}

namespace Slic3r {
// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
float max_allowable_speed(float acceleration, float target_velocity, float distance)
{
    // to avoid invalid negative numbers due to numerical imprecision 
    return ::sqrt(std::max(0.0f, sqr(target_velocity) - 2.0f * acceleration * distance));
}

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the given acceleration:
float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
    return (acceleration == 0.0f) ? 0.0f : (sqr(target_rate) - sqr(initial_rate)) / (2.0f * acceleration);
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)
float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance)
{
    return (acceleration == 0.0f) ? 0.0f : (2.0f * acceleration * distance - sqr(initial_rate) + sqr(final_rate)) / (4.0f * acceleration);
}

// This function gives the final speed while accelerating at the given constant acceleration from the given initial speed along the given distance.
float speed_from_distance(float initial_feedrate, float distance, float acceleration)
{
    // to avoid invalid negative numbers due to numerical imprecision 
    return ::sqrt(std::max(0.0f, sqr(initial_feedrate) + 2.0f * acceleration * distance));
}

// This function gives the time needed to accelerate from an initial speed to reach a final distance.
float acceleration_time_from_distance(float initial_feedrate, float distance, float acceleration)
{
    return (acceleration != 0.0f) ? (speed_from_distance(initial_feedrate, distance, acceleration) - initial_feedrate) / acceleration : 0.0f;
}

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
    boost::nowide::ifstream f(filename);
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

    update_from_gcode_enabled = true;
}

void GCodeProcessor::Metadata::reset()
{
    extrusion_role = erNone;    
    mm3_per_mm = 0.0f;
    width = 0.0f;
    height = 0.0f;
    feedrate = 0.0f;
    fan_speed = 0.0f;
    extruder_id = 0;
    color_id = 0;
}

#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
std::string GCodeProcessor::Metadata::to_string() const
{
    std::string ret = "Role:";

    switch (extrusion_role)
    {
    case erNone: {ret += "None"; break; }
    case erPerimeter: {ret += "Perimeter"; break; }
    case erExternalPerimeter: {ret += "ExternalPerimeter"; break; }
    case erOverhangPerimeter: {ret += "OverhangPerimeter"; break; }
    case erInternalInfill: {ret += "InternalInfill"; break; }
    case erSolidInfill: {ret += "SolidInfill"; break; }
    case erTopSolidInfill: {ret += "TopSolidInfill"; break; }
    case erBridgeInfill: {ret += "BridgeInfill"; break; }
    case erGapFill: {ret += "GapFill"; break; }
    case erSkirt: {ret += "Skirt"; break; }
    case erSupportMaterial: {ret += "SupportMaterial"; break; }
    case erSupportMaterialInterface: {ret += "SupportMaterialInterface"; break; }
    case erWipeTower: {ret += "WipeTower"; break; }
    case erCustom: {ret += "Custom"; break; }
    case erMixed: {ret += "Mixed"; break; }
    }

    ret += ", extr:" + std::to_string(extruder_id);
    ret += ", color:" + std::to_string(color_id);
    ret += ", w:" + std::to_string(width);
    ret += ", h:" + std::to_string(height);
    ret += ", f:" + std::to_string(feedrate);
    ret += ", fan speed:" + std::to_string(fan_speed);
    ret += ", mm3_per_mm:" + std::to_string(mm3_per_mm);

    return ret;
}
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT

void GCodeProcessor::TimeFeedrates::reset()
{
    feedrate = 0.0f;
    safe_feedrate = 0.0f;
    ::memset(axis_feedrate.data(), 0, sizeof(AxesTuple));
    ::memset(abs_axis_feedrate.data(), 0, sizeof(AxesTuple));
}

void GCodeProcessor::TimeBlock::FeedrateProfile::reset()
{
    entry = 0.0f;
    cruise = 0.0f;
    exit = 0.0f;
}

void GCodeProcessor::TimeBlock::Trapezoid::reset()
{
    profile.reset();
    distance = 0.0f;
    accelerate_until = 0.0f;
    decelerate_after = 0.0f;
}

void GCodeProcessor::TimeBlock::Flags::reset()
{
    recalculate = false;
    nominal_length = false;
}

#if !ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
const GCodeProcessor::TimeBlock GCodeProcessor::TimeBlock::Dummy = GCodeProcessor::TimeBlock();
#endif // !ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE

void GCodeProcessor::TimeBlock::reset()
{
    profile.reset();
    trapezoid.reset();
    flags.reset();

    distance = 0.0f;
    acceleration = 0.0f;
    max_entry_speed = 0.0f;
    safe_feedrate = 0.0f;
#if !ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
    elapsed_time = 0.0f;
#endif // !ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
}

void GCodeProcessor::TimeBlock::calculate_trapezoid()
{
    trapezoid.distance = distance;
    trapezoid.profile = profile;

    float accelerate_distance = std::max(0.0f, estimate_acceleration_distance(profile.entry, profile.cruise, acceleration));
    float decelerate_distance = std::max(0.0f, estimate_acceleration_distance(profile.cruise, profile.exit, -acceleration));
    float cruise_distance = distance - accelerate_distance - decelerate_distance;

    // Not enough space to reach the nominal feedrate.
    // This means no cruising, and we'll have to use intersection_distance() to calculate when to abort acceleration 
    // and start braking in order to reach the exit_feedrate exactly at the end of this block.
    if (cruise_distance < 0.0f)
    {
        accelerate_distance = clamp(0.0f, distance, intersection_distance(profile.entry, profile.exit, acceleration, distance));
        cruise_distance = 0.0f;
        trapezoid.profile.cruise = speed_from_distance(profile.entry, accelerate_distance, acceleration);
    }

    trapezoid.accelerate_until = accelerate_distance;
    trapezoid.decelerate_after = accelerate_distance + cruise_distance;
}

float GCodeProcessor::TimeBlock::calculate_time() const
{
    return acceleration_time() + cruise_time() + deceleration_time();
}

#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
std::string GCodeProcessor::TimeBlock::to_string() const
{
    std::string ret;

    ret += "distance:" + std::to_string(distance);
    ret += ", acceleration:" + std::to_string(acceleration);
    ret += ", max_entry_speed:" + std::to_string(max_entry_speed);
    ret += ", safe_feedrate:" + std::to_string(safe_feedrate);
    ret += ", trapezoid [";

    ret += "accelerate_until:" + std::to_string(trapezoid.accelerate_until);
    ret += ", decelerate_after:" + std::to_string(trapezoid.decelerate_after);
    ret += ", profile_entry:" + std::to_string(trapezoid.profile.entry);
    ret += ", profile_cruise:" + std::to_string(trapezoid.profile.cruise);
    ret += ", profile_exit:" + std::to_string(trapezoid.profile.exit);

    ret += "]";

    return ret;
}
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT

void GCodeProcessor::TimeBlock::forward_pass_kernel(const TimeBlock& prev, TimeBlock& curr)
{
    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
    if (!prev.flags.nominal_length)
    {
        if (prev.profile.entry < curr.profile.entry)
        {
            float entry_speed = std::min(curr.profile.entry, max_allowable_speed(-prev.acceleration, prev.profile.entry, prev.distance));

            // Check for junction speed change
            if (curr.profile.entry != entry_speed)
            {
                curr.profile.entry = entry_speed;
                curr.flags.recalculate = true;
            }
        }
    }
}

void GCodeProcessor::TimeBlock::reverse_pass_kernel(TimeBlock& curr, const TimeBlock& next)
{
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (curr.profile.entry != curr.max_entry_speed)
    {
        // If nominal length true, max junction speed is guaranteed to be reached. Only compute
        // for max allowable speed if block is decelerating and nominal length is false.
        if (!curr.flags.nominal_length && (curr.max_entry_speed > next.profile.entry))
            curr.profile.entry = std::min(curr.max_entry_speed, max_allowable_speed(-curr.acceleration, next.profile.entry, curr.distance));
        else
            curr.profile.entry = curr.max_entry_speed;

        curr.flags.recalculate = true;
    }
}

float GCodeProcessor::TimeBlock::acceleration_time() const
{
    return acceleration_time_from_distance(trapezoid.profile.entry, trapezoid.accelerate_until, acceleration);
}

float GCodeProcessor::TimeBlock::cruise_time() const
{
    float cruise_distance = trapezoid.decelerate_after - trapezoid.accelerate_until;
    assert(cruise_distance >= 0.0f);
    return (trapezoid.profile.cruise != 0.0f) ? cruise_distance / trapezoid.profile.cruise : 0.0f;
}

float GCodeProcessor::TimeBlock::deceleration_time() const
{
    return acceleration_time_from_distance(trapezoid.profile.cruise, (trapezoid.distance - trapezoid.decelerate_after), -acceleration);
}

#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
void GCodeProcessor::TimeEstimator::Statistics::reset()
{
    max_blocks_count = 0;
}
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT

void GCodeProcessor::TimeEstimator::reset()
{
    m_acceleration = 0.0f;
    m_time = 0.0f;
    m_blocks.clear();
#if ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
    m_processed_blocks_count = 0;
    m_elapsed_times.clear();
#else
    m_last_processed_block_id = -1;
#endif // ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
    m_statistics.reset();
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
    machine_limits = MachineLimits();
    curr_feedrates.reset();
    prev_feedrates.reset();
    color_times.reset();
    additional_time = 0.0f;
}

void GCodeProcessor::TimeEstimator::set_acceleration(float acceleration)
{
    m_acceleration = (machine_limits.max_acceleration == 0.0f) ?
        acceleration :
        // Clamp the acceleration with the maximum.
        std::min(machine_limits.max_acceleration, acceleration);
}

void GCodeProcessor::TimeEstimator::set_max_acceleration(float acceleration)
{
    if (machine_limits.update_from_gcode_enabled)
        machine_limits.max_acceleration = acceleration;

    if (acceleration > 0.0f)
        m_acceleration = acceleration;
}

#if ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
void GCodeProcessor::TimeEstimator::append_block(const TimeBlock& block)
{
    m_blocks.push_back(block);
#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
    m_statistics.max_blocks_count = std::max(m_statistics.max_blocks_count, (unsigned int)m_blocks.size());
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
}
#endif // ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE

void GCodeProcessor::TimeEstimator::calculate_time()
{
#if ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
    for (int i = 0; i < (int)m_blocks.size() - 1; ++i)
    {
        TimeBlock::forward_pass_kernel(m_blocks[i], m_blocks[i + 1]);
    }

    for (int i = (int)m_blocks.size() - 1; i >= 1; --i)
    {
        TimeBlock::reverse_pass_kernel(m_blocks[i - 1], m_blocks[i]);
    }
#else
    for (int i = m_last_processed_block_id + 1; i < (int)m_blocks.size() - 1; ++i)
    {
        TimeBlock::forward_pass_kernel(m_blocks[i], m_blocks[i + 1]);
    }

    for (int i = (int)m_blocks.size() - 1; i >= m_last_processed_block_id + 2; --i)
    {
        TimeBlock::reverse_pass_kernel(m_blocks[i - 1], m_blocks[i]);
    }
#endif // ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE

    recalculate_trapezoids();

    m_time += additional_time;
    color_times.cache += additional_time;

#if ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
    for (const TimeBlock& block : m_blocks)
    {
        float block_time = block.calculate_time();
        m_time += block_time;
        color_times.cache += block_time;
        m_elapsed_times.push_back(m_time);
    }

    m_processed_blocks_count += (unsigned int)m_blocks.size();

    // the current bunch of blocks have been processed, clear it
    m_blocks.clear();
    m_blocks.shrink_to_fit();
#else
    for (int i = m_last_processed_block_id + 1; i < (int)m_blocks.size(); ++i)
    {
        TimeBlock& block = m_blocks[i];
        float block_time = block.calculate_time();
        m_time += block_time;
        color_times.cache += block_time;
        block.elapsed_time = m_time;
    }

    m_last_processed_block_id = (int)m_blocks.size() - 1;
#endif // ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
    // The additional time has been consumed (added to the total time), reset it to zero.
    additional_time = 0.0f;
}

size_t GCodeProcessor::TimeEstimator::memory_used() const
{
    size_t out = sizeof(*this);
    out += SLIC3R_STDVEC_MEMSIZE(this->m_blocks, TimeBlock);
    out += SLIC3R_STDVEC_MEMSIZE(this->m_elapsed_times, float);
    return out;
}

void GCodeProcessor::TimeEstimator::recalculate_trapezoids()
{
    TimeBlock* curr = nullptr;
    TimeBlock* next = nullptr;

#if ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
    for (TimeBlock& b : m_blocks)
    {
        curr = next;
        next = &b;

        if (curr != nullptr)
        {
            // Recalculate if current block entry or exit junction speed has changed.
            if (curr->flags.recalculate || next->flags.recalculate)
            {
                // NOTE: Entry and exit factors always > 0 by all previous logic operations.
                TimeBlock block = *curr;
                block.profile.exit = next->profile.entry;
                block.calculate_trapezoid();
                curr->trapezoid = block.trapezoid;
                curr->flags.recalculate = false; // Reset current only to ensure next trapezoid is computed
            }
        }
    }
#else
    for (int i = m_last_processed_block_id + 1; i < (int)m_blocks.size(); ++i)
    {
        TimeBlock& b = m_blocks[i];

        curr = next;
        next = &b;

        if (curr != nullptr)
        {
            // Recalculate if current block entry or exit junction speed has changed.
            if (curr->flags.recalculate || next->flags.recalculate)
            {
                // NOTE: Entry and exit factors always > 0 by all previous logic operations.
                TimeBlock block = *curr;
                block.profile.exit = next->profile.entry;
                block.calculate_trapezoid();
                curr->trapezoid = block.trapezoid;
                curr->flags.recalculate = false; // Reset current only to ensure next trapezoid is computed
            }
        }
    }
#endif // ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE

    // Last/newest block in buffer. Always recalculated.
    if (next != nullptr)
    {
        TimeBlock block = *next;
        block.profile.exit = next->safe_feedrate;
        block.calculate_trapezoid();
        next->trapezoid = block.trapezoid;
        next->flags.recalculate = false;
    }
}

#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
std::string GCodeProcessor::GCodeMove::to_string() const
{
    std::string ret;

    switch (type)
    {
    case Noop: { ret = "Noop"; break; }
    case Retract: { ret = "Retract"; break; }
    case Unretract: { ret = "Unretract"; break; }
    case Tool_change: { ret = "Tool_change"; break; }
    case Travel: { ret = "Travel"; break; }
    case Extrude: { ret = "Extrude"; break; }
    };

    ret += ", " + data.to_string();

    ret += " start:" + ::to_string(start_position);
    ret += " end:" + ::to_string(end_position);

    return ret;
}
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT

void GCodeProcessor::RepetierStore::reset()
{
    position = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
    feedrate = FLT_MAX;
}

void GCodeProcessor::ColorTimes::reset()
{
    enabled = false;
    times.clear();
    cache = 0.0f;
}

const std::string GCodeProcessor::Extrusion_Role_Tag = "__EXTRUSION_ROLE__: ";
const std::string GCodeProcessor::Mm3_Per_Mm_Tag = "__MM3_PER_MM__:";
const std::string GCodeProcessor::Width_Tag = "__WIDTH__:";
const std::string GCodeProcessor::Height_Tag = "__HEIGHT__:";
const std::string GCodeProcessor::Color_Change_Tag = "__COLOR_CHANGE__";

void GCodeProcessor::reset()
{
    m_config = PrintConfig();
    m_config.gcode_flavor.value = gcfRepRap;

    m_units = Millimeters;
    m_global_positioning_type = Absolute;
    m_e_local_positioning_type = Absolute;

    m_extruders_offsets.clear();
    m_extruders_count = 1;

    ::memset(m_start_position.data(), 0, sizeof(AxesTuple));
    ::memset(m_end_position.data(), 0, sizeof(AxesTuple));
    ::memset(m_origin.data(), 0, sizeof(AxesTuple));

    m_extrude_factor_override_percentage = DEFAULT_EXTRUDE_FACTOR_OVERRIDE_PERCENTAGE;

    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        m_time_estimators[i].reset();
    }

    m_data.reset();
    // extruder_id is used to correctly calculate filament load / unload times 
    // into the total print time. This is currently only really used by the MK3 MMU2:
    // Extruder id (-1) means no filament is loaded yet, all the filaments are parked in the MK3 MMU2 unit.
    m_data.extruder_id = UNLOADED_EXTRUDER_ID;

    m_filament_load_times.clear();
    m_filament_unload_times.clear();

    m_repetier_store.reset();

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

//    set_extruder_id((get_extruders_count() > 0) ? -1 : 0);

    if (m_config.gcode_flavor.value == gcfMarlin)
    {
        for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
        {
            MachineLimits& limits = m_time_estimators[i].machine_limits;

            /* "Silent mode" values can be just a copy of "normal mode" values
            * (when they aren't input for a printer preset).
            * Thus, use back value from values, instead of second one, which could be absent
            */
            int id = (i < (int)m_config.machine_max_acceleration_extruding.values.size()) ? i : (int)m_config.machine_max_acceleration_extruding.values.size() - 1;
            limits.max_acceleration = (float)m_config.machine_max_acceleration_extruding.values[id];
            limits.retract_acceleration = (float)m_config.machine_max_acceleration_retracting.values[id];
            limits.minimum_feedrate = (float)m_config.machine_min_extruding_rate.values[id];
            limits.minimum_travel_feedrate = (float)m_config.machine_min_travel_rate.values[id];
            limits.axis_max_acceleration[X] = (float)m_config.machine_max_acceleration_x.values[id];
            limits.axis_max_acceleration[Y] = (float)m_config.machine_max_acceleration_y.values[id];
            limits.axis_max_acceleration[Z] = (float)m_config.machine_max_acceleration_z.values[id];
            limits.axis_max_acceleration[E] = (float)m_config.machine_max_acceleration_e.values[id];
            limits.axis_max_feedrate[X] = (float)m_config.machine_max_feedrate_x.values[id];
            limits.axis_max_feedrate[Y] = (float)m_config.machine_max_feedrate_y.values[id];
            limits.axis_max_feedrate[Z] = (float)m_config.machine_max_feedrate_z.values[id];
            limits.axis_max_feedrate[E] = (float)m_config.machine_max_feedrate_e.values[id];
            limits.axis_max_jerk[X] = (float)m_config.machine_max_jerk_x.values[id];
            limits.axis_max_jerk[Y] = (float)m_config.machine_max_jerk_y.values[id];
            limits.axis_max_jerk[Z] = (float)m_config.machine_max_jerk_z.values[id];
            limits.axis_max_jerk[E] = (float)m_config.machine_max_jerk_e.values[id];
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
#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
    boost::filesystem::path moves_path(filename);
    moves_path.replace_extension("processor_moves");
    boost::filesystem::path blocks_normal_path(filename);
    blocks_normal_path.replace_extension("processor_blocks_normal");
    boost::filesystem::path blocks_silent_path(filename);
    blocks_silent_path.replace_extension("processor_blocks_silent");

    m_out_moves.open(moves_path.string());
    m_out_blocks_normal.open(blocks_normal_path.string());
    m_out_blocks_silent.open(blocks_silent_path.string());
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT

    bool res = m_parser.parse_file(filename, [this](const GCodeLine& line) { process_gcode_line(line); });
    if (res)
    {
        for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
        {
            TimeEstimator& estimator = m_time_estimators[i];
            estimator.calculate_time();

#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
            std::cout << "GCodeProcessor " << i << " Estimated time: " << estimator.get_time() << " sec" << std::endl;
            std::cout << "GCodeProcessor " << i << " Max blocks count: " << estimator.get_statistics().max_blocks_count << " sec" << std::endl;
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT

            float final_time = estimator.get_time();
            float elapsed_time = 0.0f;
            float remaining_time = final_time;

            for (GCodeMove& m : m_moves)
            {
                if (m.block_id >= 0)
                {
#if ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
                    elapsed_time = estimator.get_elapsed_time_at_block(m.block_id);
#else
                    elapsed_time = estimator.get_block(m.block_id).elapsed_time;
#endif // ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
                    remaining_time = final_time - elapsed_time;
                }

                m.elapsed_time[i] = elapsed_time;
                m.remaining_time[i] = remaining_time;
            }
        }
    }

#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
    m_out_moves.close();
    m_out_blocks_normal.close();
    m_out_blocks_silent.close();
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
    return res;
}

size_t GCodeProcessor::memory_used() const
{
    size_t out = sizeof(*this);
    out += SLIC3R_STDVEC_MEMSIZE(m_moves, GCodeMove);
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        out += m_time_estimators[i].memory_used();
    }
    return out;
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
                // Set to Absolute Positioning
                case 90: { return process_G90(line); }
                // Set to Relative Positioning
                case 91: { return process_G91(line); }
                // Set Position
                case 92: { return process_G92(line); }
                default: { return true; }
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
                // Recall stored home offsets
                case 132: { return process_M132(line); }
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
                default: { return true; }
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
    auto axis_absolute_position = [this](Axis axis, const GCodeLine& lineG1) -> float
    {
        float current_absolute_position = get_axis_position(axis);
        float current_origin = get_axis_origin(axis);
        float lengthsScaleFactor = (get_units() == Inches) ? INCHES_TO_MM : 1.0f;

        bool is_relative = (get_global_positioning_type() == Relative);
        if (axis == E)
            is_relative |= (get_e_local_positioning_type() == Relative);

        if (lineG1.has(axis))
        {
            float ret = lineG1.value(axis) * lengthsScaleFactor;
            return is_relative ? current_absolute_position + ret : ret + current_origin;
        }
        else
            return current_absolute_position;
    };

    // updates axes positions from line
    AxesTuple new_pos;
    for (unsigned char a = X; a <= E; ++a)
    {
        new_pos[a] = axis_absolute_position((Axis)a, line);
    }

    // updates feedrate from line, if present
    if (line.has('F'))
        set_feedrate(line.value('F') * MMMIN_TO_MMSEC);

    // calculates movement deltas
    float max_abs_delta = 0.0f;
    AxesTuple delta_pos;
    for (unsigned char a = X; a <= E; ++a)
    {
        delta_pos[a] = new_pos[a] - get_axis_position((Axis)a);
        max_abs_delta = std::max(max_abs_delta, std::abs(delta_pos[a]));
    }

    // no displacement, return
    if (max_abs_delta == 0.0f)
        return true;

    // Detects move type
    GCodeMove::EType type = GCodeMove::Noop;

    if (delta_pos[E] < 0.0f)
    {
        if ((delta_pos[X] != 0.0f) || (delta_pos[Y] != 0.0f) || (delta_pos[Z] != 0.0f))
            type = GCodeMove::Travel;
        else
            type = GCodeMove::Retract;
    }
    else if (delta_pos[E] > 0.0f)
    {
        if ((delta_pos[X] == 0.0f) && (delta_pos[Y] == 0.0f) && (delta_pos[Z] == 0.0f))
            type = GCodeMove::Unretract;
        else if ((delta_pos[X] != 0.0f) || (delta_pos[Y] != 0.0f))
            type = GCodeMove::Extrude;
    }
    else if ((delta_pos[X] != 0.0f) || (delta_pos[Y] != 0.0f) || (delta_pos[Z] != 0.0f))
        type = GCodeMove::Travel;

    if (type == GCodeMove::Noop)
        return true;

    // calculates data for time estimation
    bool is_extruder_only_move = false;
    float sq_length_xyz = sqr(delta_pos[X]) + sqr(delta_pos[Y]) + sqr(delta_pos[Z]);
    float distance = 0.0f;
    if (sq_length_xyz > 0.0f)
        distance = ::sqrt(sq_length_xyz);
    else
    {
        distance = std::abs(delta_pos[E]);
        is_extruder_only_move = true;
    }

    float inv_distance = 1.0f / distance;

    std::vector<TimeBlock> blocks(Num_TimeEstimateModes);

    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        ETimeEstimateMode m = (ETimeEstimateMode)i;
        TimeEstimator& time_estimator = m_time_estimators[i];
        TimeFeedrates& curr = time_estimator.curr_feedrates;
        TimeFeedrates& prev = time_estimator.prev_feedrates;
        TimeBlock& block = blocks[i];

        block.distance = distance;

        // calculates time-block feedrate
        curr.feedrate = std::max(get_feedrate(), (type == GCodeMove::Travel) ? get_minimum_travel_feedrate(m) : get_minimum_feedrate(m));

        float min_feedrate_factor = 1.0f;
        for (unsigned char j = X; j <= E; ++j)
        {
            curr.axis_feedrate[j] = curr.feedrate * delta_pos[j] * inv_distance;
            if (j == E)
                curr.axis_feedrate[j] *= get_extrude_factor_override_percentage();

            curr.abs_axis_feedrate[j] = std::abs(curr.axis_feedrate[j]);
            if (curr.abs_axis_feedrate[j] > 0.0f)
                min_feedrate_factor = std::min(min_feedrate_factor, get_axis_max_feedrate(m, (Axis)j) / curr.abs_axis_feedrate[j]);
        }

        block.profile.cruise = min_feedrate_factor * curr.feedrate;

        if (min_feedrate_factor < 1.0f)
        {
            for (unsigned char j = X; j <= E; ++j)
            {
                curr.axis_feedrate[j] *= min_feedrate_factor;
                curr.abs_axis_feedrate[j] *= min_feedrate_factor;
            }
        }

        // calculates block acceleration
        float acceleration = is_extruder_only_move ? get_retract_acceleration(m) : get_acceleration(m);

        for (unsigned char j = X; j <= E; ++j)
        {
            float axis_max_acceleration = get_axis_max_acceleration(m, (Axis)j);
            if (acceleration * std::abs(delta_pos[j]) * inv_distance > axis_max_acceleration)
                acceleration = axis_max_acceleration;
        }

        block.acceleration = acceleration;

        // calculate block exit feedrate
        curr.safe_feedrate = block.profile.cruise;

        for (unsigned char j = X; j <= E; ++j)
        {
            float axis_max_jerk = get_axis_max_jerk(m, (Axis)j);
            if (curr.abs_axis_feedrate[j] > axis_max_jerk)
                curr.safe_feedrate = std::min(curr.safe_feedrate, axis_max_jerk);
        }

        block.profile.exit = curr.safe_feedrate;

        // calculate block entry feedrate
        float vmax_junction = curr.safe_feedrate;
        // ET_FIXME: we should check for unprocessed blocks instead of moves count see GCodeTimeEstimator 
        if (!m_moves.empty() && (prev.feedrate > PREVIOUS_FEEDRATE_THRESHOLD))
        {
            bool prev_speed_larger = prev.feedrate > block.profile.cruise;
            float smaller_speed_factor = prev_speed_larger ? (block.profile.cruise / prev.feedrate) : (prev.feedrate / block.profile.cruise);
            // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
            vmax_junction = prev_speed_larger ? block.profile.cruise : prev.feedrate;

            float v_factor = 1.0f;
            bool limited = false;

            for (unsigned char j = X; j <= E; ++j)
            {
                // Limit an axis. We have to differentiate coasting from the reversal of an axis movement, or a full stop.
                float v_exit = prev.axis_feedrate[j];
                float v_entry = curr.axis_feedrate[j];

                if (prev_speed_larger)
                    v_exit *= smaller_speed_factor;

                if (limited)
                {
                    v_exit *= v_factor;
                    v_entry *= v_factor;
                }

                // Calculate the jerk depending on whether the axis is coasting in the same direction or reversing a direction.
                float jerk =
                    (v_exit > v_entry) ?
                    (((v_entry > 0.0f) || (v_exit < 0.0f)) ?
                    // coasting
                    (v_exit - v_entry) :
                    // axis reversal
                    std::max(v_exit, -v_entry)) :
                    // v_exit <= v_entry
                    (((v_entry < 0.0f) || (v_exit > 0.0f)) ?
                    // coasting
                    (v_entry - v_exit) :
                    // axis reversal
                    std::max(-v_exit, v_entry));

                float axis_max_jerk = get_axis_max_jerk(m, (Axis)j);
                if (jerk > axis_max_jerk)
                {
                    v_factor *= axis_max_jerk / jerk;
                    limited = true;
                }
            }

            if (limited)
                vmax_junction *= v_factor;

            // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
            // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
            float vmax_junction_threshold = vmax_junction * 0.99f;

            // Not coasting. The machine will stop and start the movements anyway, better to start the segment from start.
            if ((prev.safe_feedrate > vmax_junction_threshold) && (curr.safe_feedrate > vmax_junction_threshold))
                vmax_junction = curr.safe_feedrate;
        }

        float v_allowable = max_allowable_speed(-acceleration, curr.safe_feedrate, distance);
        block.profile.entry = std::min(vmax_junction, v_allowable);

        block.max_entry_speed = vmax_junction;
        block.flags.nominal_length = (block.profile.cruise <= v_allowable);
        block.flags.recalculate = true;
        block.safe_feedrate = curr.safe_feedrate;

        // calculate block trapezoid
        block.calculate_trapezoid();

        // update previous
        prev = curr;
    }

    ExtrusionRole role = get_extrusion_role();
    if ((type == GCodeMove::Extrude) && ((get_width() == 0.0f) || (get_height() == 0.0f) || !is_valid_extrusion_role(role)))
        type = GCodeMove::Travel;

    // update position
    set_end_position(new_pos);

    // store the move and blocks
    store_move(type, (int)m_time_estimators[Normal].get_blocks_count() - 1);
    store_blocks(blocks);

#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
    BOOST_LOG_TRIVIAL(trace) << "Processor memory: " << format_memsize_MB(memory_used()) << log_memory_info();
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT

    return true;
}

bool GCodeProcessor::process_G4(const GCodeLine& line)
{
    GCodeFlavor flavor = get_gcode_flavor();

    if (line.has('P'))
        add_additional_time(line.value('P') * MILLISEC_TO_SEC);

    // see: http://reprap.org/wiki/G-code#G4:_Dwell
    if ((flavor == gcfRepetier) ||
        (flavor == gcfMarlin) ||
        (flavor == gcfSmoothie) ||
        (flavor == gcfRepRap))
    {
        if (line.has('S'))
            add_additional_time(line.value('S'));
    }

    simulate_st_synchronize();

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
            simulate_st_synchronize();
    }

    if (!anyFound)
        set_origin(get_end_position());

    return true;
}

bool GCodeProcessor::process_M1(const GCodeLine& line)
{
    simulate_st_synchronize();
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
    if (!line.has('P'))
    {
        // The absence of P means the print cooling fan, so ignore anything else.
        if (line.has('S'))
            set_fan_speed((100.0f / 256.0f) * line.value('S'));
        else
            set_fan_speed(100.0f);
    }
    return true;
}

bool GCodeProcessor::process_M107(const GCodeLine& line)
{
    set_fan_speed(0.0f);
    return true;
}

bool GCodeProcessor::process_M108(const GCodeLine& line)
{
    // M108 is used by Sailfish to change active tool.
    // They have to be processed otherwise toolchanges will be unrecognised
    // by the analyzer - see https://github.com/prusa3d/PrusaSlicer/issues/2566
    // see also process_M135()

    if (get_gcode_flavor() == gcfSailfish)
    {
        std::string cmd = line.raw();
        size_t T_pos = cmd.find("T");
        if (T_pos != std::string::npos)
        {
            cmd = cmd.substr(T_pos);
            return process_T(GCodeLine(cmd));
        }
    }
    return true;
}

bool GCodeProcessor::process_M132(const GCodeLine& line)
{
    // This command is used by Makerbot to load the current home position from EEPROM
    // see: https://github.com/makerbot/s3g/blob/master/doc/GCodeProtocol.md
    // Using this command to reset the axis origin to zero helps in fixing: https://github.com/prusa3d/PrusaSlicer/issues/3082

    if (line.has(X))
        set_axis_origin(X, 0.0f);

    if (line.has(Y))
        set_axis_origin(Y, 0.0f);

    if (line.has(Z))
        set_axis_origin(Z, 0.0f);

    if (line.has(E))
        set_axis_origin(E, 0.0f);

    return true;
}

bool GCodeProcessor::process_M135(const GCodeLine& line)
{
    // M135 is used by MakerWare to change active tool.
    // They have to be processed otherwise toolchanges will be unrecognised
    // by the analyzer - see https://github.com/prusa3d/PrusaSlicer/issues/2566
    // see also process_M108()

    if (get_gcode_flavor() == gcfMakerWare)
    {
        std::string cmd = line.raw();
        size_t T_pos = cmd.find("T");
        if (T_pos != std::string::npos)
        {
            cmd = cmd.substr(T_pos);
            return process_T(GCodeLine(cmd));
        }
    }
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
    if (get_gcode_flavor() == gcfRepetier)
    {
        set_repetier_store_position(get_end_position());
        set_repetier_store_feedrate(get_feedrate());
    }

    return true;
}

bool GCodeProcessor::process_M402(const GCodeLine& line)
{
    if (get_gcode_flavor() == gcfRepetier)
    {
        // see for reference:
        // https://github.com/repetier/Repetier-Firmware/blob/master/src/ArduinoAVR/Repetier/Repetier.ino
        // and
        // https://github.com/repetier/Repetier-Firmware/blob/master/src/ArduinoAVR/Repetier/Printer.cpp
        // void Printer::GoToMemoryPosition(bool x, bool y, bool z, bool e, float feed)

        bool has_xyz = !(line.has(X) || line.has(Y) || line.has(Z));

        const AxesTuple& store_position = get_repetier_store_position();
        float store_feedrate = get_repetier_store_feedrate();

        for (unsigned char i = X; i <= Z; ++i)
        {
            Axis a = (Axis)i;
            if (has_xyz || line.has(a))
            {
                if (store_position[i] != FLT_MAX)
                    set_axis_position(a, store_position[i]);
            }
        }

        if (store_position[E] != FLT_MAX)
            set_axis_position(E, store_position[E]);

        if (!line.has(F) && (store_feedrate != FLT_MAX))
            set_feedrate(store_feedrate);
    }

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

    return true;
}

bool GCodeProcessor::process_M702(const GCodeLine& line)
{
    if (line.has('C'))
    {
        // MK3 MMU2 specific M code:
        // M702 C is expected to be sent by the custom end G-code when finalizing a print.
        // The MK3 unit shall unload and park the active filament into the MMU2 unit.
        add_additional_time(get_filament_unload_time(get_extruder_id()));
        set_extruder_id(UNLOADED_EXTRUDER_ID);
        simulate_st_synchronize();
    }

    return true;
}

bool GCodeProcessor::process_T(const GCodeLine& line)
{
    std::string cmd = line.cmd();

    if (cmd.length() > 1)
    {
        unsigned int old_id = get_extruder_id();
        unsigned int new_id = (unsigned int)::strtol(cmd.substr(1).c_str(), nullptr, 10);
        if (old_id != new_id)
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
                add_additional_time(get_filament_unload_time(old_id));
                set_extruder_id(new_id);
                add_additional_time(get_filament_load_time(new_id));
                simulate_st_synchronize();
            }

            // stores tool change move
            if (old_id != UNLOADED_EXTRUDER_ID)
                store_move(GCodeMove::Tool_change);
        }
    }

    return true;
}

bool GCodeProcessor::process_gcode_comment(const GCodeLine& line)
{
    std::string comment = line.comment();

    // extrusion role tag
    size_t pos = comment.find(Extrusion_Role_Tag);
    if (pos != comment.npos)
        return process_extrusion_role_tag(comment, pos);

    // mm3 per mm tag
    pos = comment.find(Mm3_Per_Mm_Tag);
    if (pos != comment.npos)
        return process_mm3_per_mm_tag(comment, pos);

    // width tag
    pos = comment.find(Width_Tag);
    if (pos != comment.npos)
        return process_width_tag(comment, pos);

    // height tag
    pos = comment.find(Height_Tag);
    if (pos != comment.npos)
        return process_height_tag(comment, pos);

    // color change tag
    pos = comment.find(Color_Change_Tag);
    if (pos != comment.npos)
        return process_color_change_tag();

    return false;
}

bool GCodeProcessor::process_extrusion_role_tag(const std::string& comment, size_t pos)
{
    int role = (int)::strtol(comment.substr(pos + Extrusion_Role_Tag.length()).c_str(), nullptr, 10);
    if (is_valid_extrusion_role(role))
    {
        set_extrusion_role((ExtrusionRole)role);
        return true;
    }
    else
    {
        BOOST_LOG_TRIVIAL(warning) << "Found invalid extrusion role: " << comment;
        return false;
    }
}

bool GCodeProcessor::process_mm3_per_mm_tag(const std::string& comment, size_t pos)
{
    float value = (float)::strtod(comment.substr(pos + Mm3_Per_Mm_Tag.length()).c_str(), nullptr);
    if (value > 0.0f)
    {
        set_mm3_per_mm(value);
        return true;
    }
    else
    {
        BOOST_LOG_TRIVIAL(warning) << "Found invalid mm3_per_mm: " << comment;
        return false;
    }
}

bool GCodeProcessor::process_width_tag(const std::string& comment, size_t pos)
{
    float value = (float)::strtod(comment.substr(pos + Width_Tag.length()).c_str(), nullptr);
    if (value > 0.0f)
    {
        set_width(value);
        return true;
    }
    else
    {
        BOOST_LOG_TRIVIAL(warning) << "Found invalid width: " << comment;
        return false;
    }
}

bool GCodeProcessor::process_height_tag(const std::string& comment, size_t pos)
{
    float value = (float)::strtod(comment.substr(pos + Height_Tag.length()).c_str(), nullptr);
    if (value > 0.0f)
    {
        set_height(value);
        return true;
    }
    else
    {
        BOOST_LOG_TRIVIAL(warning) << "Found invalid height: " << comment;
        return false;
    }
}

bool GCodeProcessor::process_color_change_tag()
{
    set_color_id(get_color_id() + 1);

    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        ETimeEstimateMode m = (ETimeEstimateMode)i;
        enable_color_times(m, true);
    }

    calculate_time();

    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        ETimeEstimateMode m = (ETimeEstimateMode)i;
        if (get_color_times_cache(m) > 0.0f)
        {
            store_current_color_times_cache(m);
            set_color_times_cache(m, 0.0f);
        }
    }

    return true;
}

void GCodeProcessor::simulate_st_synchronize()
{
    calculate_time();
}

void GCodeProcessor::calculate_time()
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        m_time_estimators[i].calculate_time();
    }
}

float GCodeProcessor::get_filament_load_time(unsigned int extruder_id)
{
    return
        (m_filament_load_times.empty() || (extruder_id == UNLOADED_EXTRUDER_ID)) ?
        0.0f :
        (m_filament_load_times.size() <= extruder_id) ?
        (float)m_filament_load_times.front() :
        (float)m_filament_load_times[extruder_id];
}

float GCodeProcessor::get_filament_unload_time(unsigned int extruder_id)
{
    return
        (m_filament_unload_times.empty() || (extruder_id == UNLOADED_EXTRUDER_ID)) ?
        0.0f :
        (m_filament_unload_times.size() <= extruder_id) ?
        (float)m_filament_unload_times.front() :
        (float)m_filament_unload_times[extruder_id];
}

void GCodeProcessor::set_additional_time(float time)
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        m_time_estimators[i].additional_time = time;
    }
}

void GCodeProcessor::add_additional_time(float time)
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        m_time_estimators[i].additional_time += time;
    }
}

void GCodeProcessor::set_acceleration(float acceleration)
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        m_time_estimators[i].set_acceleration(acceleration);
    }
}

void GCodeProcessor::set_max_acceleration(float acceleration)
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        m_time_estimators[i].set_max_acceleration(acceleration);
    }
}

void GCodeProcessor::set_retract_acceleration(float acceleration)
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        m_time_estimators[i].machine_limits.retract_acceleration = acceleration;
    }
}

void GCodeProcessor::set_minimum_feedrate(float feedrate)
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        if (m_time_estimators[i].machine_limits.update_from_gcode_enabled)
            m_time_estimators[i].machine_limits.minimum_feedrate = feedrate;
    }
}

void GCodeProcessor::set_minimum_travel_feedrate(float feedrate)
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        if (m_time_estimators[i].machine_limits.update_from_gcode_enabled)
            m_time_estimators[i].machine_limits.minimum_travel_feedrate = feedrate;
    }
}

void GCodeProcessor::set_axis_max_feedrate(Axis axis, float feedrate)
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        if (m_time_estimators[i].machine_limits.update_from_gcode_enabled)
            m_time_estimators[i].machine_limits.axis_max_feedrate[axis] = feedrate;
    }
}

void GCodeProcessor::set_axis_max_acceleration(Axis axis, float acceleration)
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        if (m_time_estimators[i].machine_limits.update_from_gcode_enabled)
            m_time_estimators[i].machine_limits.axis_max_acceleration[axis] = acceleration;
    }
}

void GCodeProcessor::set_axis_max_jerk(Axis axis, float jerk)
{
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        if (m_time_estimators[i].machine_limits.update_from_gcode_enabled)
            m_time_estimators[i].machine_limits.axis_max_jerk[axis] = jerk;
    }
}

bool GCodeProcessor::is_valid_extrusion_role(int value) const
{
    return ((int)erNone <= value) && (value <= (int)erMixed);
}

void GCodeProcessor::store_move(GCodeMove::EType type, int block_id)
{
    unsigned int extruder_id = get_extruder_id();
    extruder_id = (extruder_id == UNLOADED_EXTRUDER_ID) ? 0 : extruder_id;
    Metadata data(get_extrusion_role(), get_mm3_per_mm(), get_width(), get_height(), get_feedrate(), get_fan_speed(), extruder_id, get_color_id());

    // ET_FIXME: Could this calculation be moved into process_G1() ?
    ExtrudersOffsetsMap::iterator extr_it = m_extruders_offsets.find(extruder_id);
    Vec2d extruder_offset = (extr_it != m_extruders_offsets.end()) ? extr_it->second : Vec2d::Zero();
    AxesTuple start_position = get_start_position();
    start_position[0] += (float)extruder_offset(0);
    start_position[1] += (float)extruder_offset(1);
    AxesTuple end_position = get_end_position();
    end_position[0] += (float)extruder_offset(0);
    end_position[1] += (float)extruder_offset(1);

    m_moves.emplace_back(type, data, start_position, end_position, block_id);

#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
    if (m_out_moves.good())
        m_out_moves << m_moves.back().to_string() << std::endl;
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
}

void GCodeProcessor::store_blocks(const std::vector<TimeBlock>& blocks)
{
    assert(blocks.size() == Num_TimeEstimateModes);
    for (int i = 0; i < (int)Num_TimeEstimateModes; ++i)
    {
        m_time_estimators[i].append_block(blocks[i]);
    }

#if ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
#if ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
    if (m_out_blocks_normal.good())
        m_out_blocks_normal << m_time_estimators[Normal].get_last_block().to_string() << std::endl;

    if (m_out_blocks_silent.good())
        m_out_blocks_silent << m_time_estimators[Silent].get_last_block().to_string() << std::endl;
#else
    if (m_out_blocks_normal.good())
        m_out_blocks_normal << m_time_estimators[Normal].get_block(m_time_estimators[Normal].get_blocks_count() - 1).to_string() << std::endl;

    if (m_out_blocks_silent.good())
        m_out_blocks_silent << m_time_estimators[Silent].get_block(m_time_estimators[Silent].get_blocks_count() - 1).to_string() << std::endl;
#endif // ENABLE_GCODE_PROCESSOR_DISCARD_BLOCKS_AFTER_USE
#endif // ENABLE_GCODE_PROCESSOR_DEBUG_OUTPUT
}

} /* namespace Slic3r */

#endif // ENABLE_GCODE_PROCESSOR
