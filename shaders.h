#ifndef __SHADERS__
#define __SHADERS__

#include "common.h"
#include "mat.h"
#include <map>

// Vertex layout: each vertex stores only position, as a 3-vector
struct vertex_p
{
    vec3 position;
};

// Vertex layout: each vertex stores 5 floats.  Only the middle three are used.
// Note that the position need not be at the beginning or the end.
struct vertex_r
{
    float unused0;
    vec3 position;
    float unused1;
};

// Vertex layout: each vertex stores position followed by color (as 3-vectors)
struct vertex_pc
{
    vec3 position;
    vec3 color;
};

struct vertex_c
{
    vec3 color;
};

// Vertex layout: like vertex_pc, but with color before position.
struct vertex_cp:vertex_c
{
    vec3 position;
};

// Vertex layout: like vertex_cp, but with positions in homogeneous coordinates
struct vertex_cph:vertex_c
{
    vec4 position;
};

// Uniform data layout: just store transform matrix
struct uniform_transform
{
    mat4 transform;
};

// Uniform data layout: store transform matrix, followed by a globally constant color
struct transform_color : public uniform_transform
{
    vec3 color;
};

extern std::map<std::string,shader_v> vertex_shader_map;
extern std::map<std::string,shader_f> fragment_shader_map;
void register_named_shaders();

#endif

