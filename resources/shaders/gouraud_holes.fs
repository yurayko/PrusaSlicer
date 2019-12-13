#version 110

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#define INTENSITY_CORRECTION 0.6

// normalized values for (-0.6/1.31, 0.6/1.31, 1./1.31)
const vec3 LIGHT_TOP_DIR = vec3(-0.4574957, 0.4574957, 0.7624929);
#define LIGHT_TOP_DIFFUSE    (0.8 * INTENSITY_CORRECTION)
#define LIGHT_TOP_SPECULAR   (0.125 * INTENSITY_CORRECTION)
#define LIGHT_TOP_SHININESS  20.0

// normalized values for (1./1.43, 0.2/1.43, 1./1.43)
const vec3 LIGHT_FRONT_DIR = vec3(0.6985074, 0.1397015, 0.6985074);
#define LIGHT_FRONT_DIFFUSE  (0.3 * INTENSITY_CORRECTION)

#define INTENSITY_AMBIENT 0.3
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


const vec3 ZERO = vec3(0.0, 0.0, 0.0);
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
const int MAX_DRILLS = 16;
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//varying vec3 clipping_planes_dots;
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

// x = tainted, y = specular;
varying vec2 intensity;

varying vec3 delta_box_min;
varying vec3 delta_box_max;
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
varying vec3 world_position;
varying vec3 clipping_planes_dots;
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
uniform vec3 camera_world_position;
uniform mat4 view_matrix;
uniform mat4 projection_view_matrix;
uniform bool inside_drill_only;
uniform vec2 z_range;
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
uniform vec4 uniform_color;

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
struct Drill
{
    vec3 position;
    vec3 axis_z;
    float radius;
    float height;
};

uniform Drill drills[MAX_DRILLS];
uniform int drills_count;

void to_drill_local(vec3 pos, int id, out float proj, out vec3 perp)
{
    vec3 v = pos - drills[id].position;
    proj = dot(v, drills[id].axis_z);
    perp = v - proj * drills[id].axis_z;        
}

// return the id of the drill containing this fragment, or -1 if none
int inside_drill_id()
{
    for (int i = 0; i < drills_count; ++i)
    {
        float proj;
        vec3 perp;
        to_drill_local(world_position, i, proj, perp);
        if ((0.0 <= proj) && (proj <= drills[i].height))
        {
            if (dot(perp, perp) < drills[i].radius * drills[i].radius)
                return i;
        }
    }
    
    return -1;
}

// return true if the ray connecting this fragment with the eye intersects the drill with the given id
// if true is returned new_fragment_world will contain the world coordinate of the new fragment position
bool intersects_drill(int id, out vec3 new_fragment_position_world, out vec3 new_fragment_normal_world)
{
    vec3 eye_to_drill_position = drills[id].position - camera_world_position;
    vec3 unit_eye_to_drill_position = normalize(eye_to_drill_position);
    vec3 unit_eye_to_fragment = normalize(world_position - camera_world_position);

    vec3 a = cross(-eye_to_drill_position, drills[id].axis_z);
    vec3 b = cross(unit_eye_to_fragment, drills[id].axis_z);
    float ab = dot(a, b);
    float ab2 = ab * ab;
    float a2 = dot(a, a);
    float b2 = dot(b, b);
    float r2 = drills[id].radius * drills[id].radius;
    
    float t = (-ab + sqrt(ab2 - b2 * (a2 - r2))) / b2;
    
    vec3 i = camera_world_position + t * unit_eye_to_fragment;
    float i_proj;
    vec3 i_perp;
    to_drill_local(i, id, i_proj, i_perp);
    new_fragment_position_world = drills[id].position + i_proj * drills[id].axis_z + i_perp;
    new_fragment_normal_world = -normalize(i_perp);
    return (0.0 <= i_proj) && (i_proj <= drills[id].height);
}

vec2 intensity_on_drill(vec3 position, vec3 normal)
{
    vec2 ret;
    
    // First transform the normal into camera space and normalize the result.
    vec3 normal_eye = normalize(view_matrix * vec4(normal, 0.0)).xyz;
    
    // Compute the cos of the angle between the normal and lights direction. The light is directional so the direction is constant for every vertex.
    // Since these two are normalized the cosine is the dot product. We also need to clamp the result to the [0,1] range.
    float NdotL = max(dot(normal_eye, LIGHT_TOP_DIR), 0.0);

    ret.x = INTENSITY_AMBIENT + NdotL * LIGHT_TOP_DIFFUSE;
    ret.y = 0.0;

    if (NdotL > 0.0)
        ret.y += LIGHT_TOP_SPECULAR * pow(max(dot(normal_eye, reflect(-LIGHT_TOP_DIR, normal_eye)), 0.0), LIGHT_TOP_SHININESS);

    // Perform the same lighting calculation for the 2nd light source (no specular applied).
    NdotL = max(dot(normal_eye, LIGHT_FRONT_DIR), 0.0);
    ret.x += NdotL * LIGHT_FRONT_DIFFUSE;

    return ret;
}
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

void main()
{
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    bool reshade_on_drill = false;
    vec3 new_fragment_position_world;
    vec3 new_fragment_normal_world;
    
    if (drills_count > 0)
    {
        int drill_id = inside_drill_id();
        if (drill_id >= 0)
        {
            if (!gl_FrontFacing)
                discard;

            reshade_on_drill = intersects_drill(drill_id, new_fragment_position_world, new_fragment_normal_world);
            if (!reshade_on_drill)
                discard;
            else if ((new_fragment_position_world.z < z_range.x) || (z_range.y < new_fragment_position_world.z))
                discard;            
        }
    }    
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    if (any(lessThan(clipping_planes_dots, ZERO)))
        discard;

    // if the fragment is outside the print volume -> use darker color
    vec3 color = (any(lessThan(delta_box_min, ZERO)) || any(greaterThan(delta_box_max, ZERO))) ? mix(uniform_color.rgb, ZERO, 0.3333) : uniform_color.rgb;
        
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    if (reshade_on_drill)
    {
        vec4 new_fragment_clip = projection_view_matrix * vec4(new_fragment_position_world, 1.0);
        gl_FragDepth = 0.5 + 0.5 * new_fragment_clip.z / new_fragment_clip.w;
        vec2 new_fragment_intensity = intensity_on_drill(new_fragment_position_world, new_fragment_normal_world);
        gl_FragColor = vec4(vec3(new_fragment_intensity.y, new_fragment_intensity.y, new_fragment_intensity.y) + color * new_fragment_intensity.x, uniform_color.a);
    }
    else
    {
        if (inside_drill_only)
            discard;
        else
        {
            gl_FragDepth = gl_FragCoord.z;
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            gl_FragColor = vec4(vec3(intensity.y, intensity.y, intensity.y) + color * intensity.x, uniform_color.a);
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        }
    }
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
}
