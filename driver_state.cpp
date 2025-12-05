#include "driver_state.h"
#include <cstring>
#include <vector>
#include <limits>

driver_state::driver_state()
{
}

driver_state::~driver_state()
{
    delete [] image_color;
    delete [] image_depth;
}

void initialize_render(driver_state& state, int width, int height)
{
    state.image_width=width;
    state.image_height=height;
    state.image_color=0;
    state.image_depth=0;
    
    int num_pixels = width * height;
    state.image_color = new pixel[num_pixels];
    for(int i = 0; i < num_pixels; i++)
        state.image_color[i] = make_pixel(0, 0, 0);
    
    state.image_depth = new float[num_pixels];
    for(int i = 0; i < num_pixels; i++)
        state.image_depth[i] = 1.0f;
}

void render(driver_state& state, render_type type)
{
    data_geometry* geometry_data = new data_geometry[state.num_vertices];
    
    for(int i = 0; i < state.num_vertices; i++)
    {
        data_vertex input_vertex;
        input_vertex.data = &state.vertex_data[i * state.floats_per_vertex];
        geometry_data[i].data = new float[MAX_FLOATS_PER_VERTEX];
        for(int j = 0; j < state.floats_per_vertex; j++)
            geometry_data[i].data[j] = input_vertex.data[j];
        state.vertex_shader(input_vertex, geometry_data[i], state.uniform_data);
    }
    
    switch(type)
    {
        case render_type::triangle:
            for(int i = 0; i < state.num_vertices; i += 3)
                clip_triangle(state, geometry_data[i], geometry_data[i+1], geometry_data[i+2]);
            break;
        case render_type::indexed:
            for(int i = 0; i < state.num_triangles; i++)
            {
                int i0 = state.index_data[3*i + 0];
                int i1 = state.index_data[3*i + 1];
                int i2 = state.index_data[3*i + 2];
                clip_triangle(state, geometry_data[i0], geometry_data[i1], geometry_data[i2]);
            }
            break;
        case render_type::fan:
            for(int i = 1; i < state.num_vertices - 1; i++)
                clip_triangle(state, geometry_data[0], geometry_data[i], geometry_data[i+1]);
            break;
        case render_type::strip:
            for(int i = 0; i < state.num_vertices - 2; i++)
            {
                if(i % 2 == 0)
                    clip_triangle(state, geometry_data[i], geometry_data[i+1], geometry_data[i+2]);
                else
                    clip_triangle(state, geometry_data[i+1], geometry_data[i], geometry_data[i+2]);
            }
            break;
        default:
            break;
    }
    
    for(int i = 0; i < state.num_vertices; i++)
        delete[] geometry_data[i].data;
    delete[] geometry_data;
}

void clip_triangle(driver_state& state, const data_geometry& v0,
    const data_geometry& v1, const data_geometry& v2)
{
    float flat_data[MAX_FLOATS_PER_VERTEX];
    for(int i = 0; i < state.floats_per_vertex; i++)
        flat_data[i] = v0.data[i];
    
    std::vector<data_geometry> polygon;
    for(int v = 0; v < 3; v++)
    {
        const data_geometry& src = (v == 0) ? v0 : (v == 1) ? v1 : v2;
        data_geometry copy;
        copy.gl_Position = src.gl_Position;
        copy.data = new float[MAX_FLOATS_PER_VERTEX];
        for(int i = 0; i < state.floats_per_vertex; i++)
            copy.data[i] = src.data[i];
        polygon.push_back(copy);
    }
    
    int coords[] = {2, 2, 0, 0, 1, 1};
    float signs[] = {1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f};
    
    for(int plane = 0; plane < 6; plane++)
    {
        if(polygon.size() < 3) break;
        
        int coord = coords[plane];
        float sign = signs[plane];
        std::vector<data_geometry> output;
        
        for(size_t i = 0; i < polygon.size(); i++)
        {
            const data_geometry& current = polygon[i];
            const data_geometry& next = polygon[(i + 1) % polygon.size()];
            
            float d_current = sign * current.gl_Position[coord] + current.gl_Position[3];
            float d_next = sign * next.gl_Position[coord] + next.gl_Position[3];
            
            bool current_inside = (d_current >= 0);
            bool next_inside = (d_next >= 0);
            
            if(current_inside)
            {
                data_geometry copy;
                copy.gl_Position = current.gl_Position;
                copy.data = new float[MAX_FLOATS_PER_VERTEX];
                for(int j = 0; j < state.floats_per_vertex; j++)
                    copy.data[j] = current.data[j];
                output.push_back(copy);
            }
            
            if(current_inside != next_inside)
            {
                float t = d_current / (d_current - d_next);
                float w1 = next.gl_Position[3];
                float w_new = current.gl_Position[3] + t * (next.gl_Position[3] - current.gl_Position[3]);
                float s = (t * w1) / w_new;
                
                data_geometry intersection;
                intersection.gl_Position = current.gl_Position + t * (next.gl_Position - current.gl_Position);
                intersection.data = new float[MAX_FLOATS_PER_VERTEX];
                
                for(int j = 0; j < state.floats_per_vertex; j++)
                {
                    if(state.interp_rules[j] == interp_type::flat)
                        intersection.data[j] = flat_data[j];
                    else if(state.interp_rules[j] == interp_type::noperspective)
                        intersection.data[j] = current.data[j] + s * (next.data[j] - current.data[j]);
                    else
                        intersection.data[j] = current.data[j] + t * (next.data[j] - current.data[j]);
                }
                output.push_back(intersection);
            }
        }
        
        for(size_t i = 0; i < polygon.size(); i++)
            delete[] polygon[i].data;
        polygon = output;
    }
    
    if(polygon.size() >= 3)
    {
        for(size_t i = 1; i + 1 < polygon.size(); i++)
            rasterize_triangle(state, polygon[0], polygon[i], polygon[i+1]);
    }
    
    for(size_t i = 0; i < polygon.size(); i++)
        delete[] polygon[i].data;
}

void rasterize_triangle(driver_state& state, const data_geometry& v0,
    const data_geometry& v1, const data_geometry& v2)
{
    float w0 = v0.gl_Position[3];
    float w1 = v1.gl_Position[3];
    float w2 = v2.gl_Position[3];
    
    vec4 p0 = v0.gl_Position / w0;
    vec4 p1 = v1.gl_Position / w1;
    vec4 p2 = v2.gl_Position / w2;
    
    float x0 = (p0[0] + 1.0f) * state.image_width * 0.5f;
    float y0 = (p0[1] + 1.0f) * state.image_height * 0.5f;
    float x1 = (p1[0] + 1.0f) * state.image_width * 0.5f;
    float y1 = (p1[1] + 1.0f) * state.image_height * 0.5f;
    float x2 = (p2[0] + 1.0f) * state.image_width * 0.5f;
    float y2 = (p2[1] + 1.0f) * state.image_height * 0.5f;
    
    int min_x = std::max(0, (int)std::min(std::min(x0, x1), x2));
    int max_x = std::min(state.image_width - 1, (int)std::max(std::max(x0, x1), x2));
    int min_y = std::max(0, (int)std::min(std::min(y0, y1), y2));
    int max_y = std::min(state.image_height - 1, (int)std::max(std::max(y0, y1), y2));
    
    float area = 0.5f * ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0));
    if(std::abs(area) < 1e-10f) return;
    
    for(int y = min_y; y <= max_y; y++)
    {
        for(int x = min_x; x <= max_x; x++)
        {
            float px = x + 0.5f;
            float py = y + 0.5f;
            
            float alpha = 0.5f * ((x1 - px) * (y2 - py) - (x2 - px) * (y1 - py)) / area;
            float beta = 0.5f * ((x2 - px) * (y0 - py) - (x0 - px) * (y2 - py)) / area;
            float gamma = 0.5f * ((x0 - px) * (y1 - py) - (x1 - px) * (y0 - py)) / area;
            
            if(alpha >= 0 && beta >= 0 && gamma >= 0)
            {
                float depth = alpha * p0[2] + beta * p1[2] + gamma * p2[2];
                int pixel_index = y * state.image_width + x;
                
                if(depth < state.image_depth[pixel_index])
                {
                    state.image_depth[pixel_index] = depth;
                    
                    data_fragment fragment;
                    fragment.data = new float[MAX_FLOATS_PER_VERTEX];
                    
                    for(int i = 0; i < state.floats_per_vertex; i++)
                    {
                        switch(state.interp_rules[i])
                        {
                            case interp_type::flat:
                                fragment.data[i] = v0.data[i];
                                break;
                            case interp_type::smooth:
                            {
                                float a = alpha / w0;
                                float b = beta / w1;
                                float g = gamma / w2;
                                float k = a + b + g;
                                fragment.data[i] = (a * v0.data[i] + b * v1.data[i] + g * v2.data[i]) / k;
                                break;
                            }
                            case interp_type::noperspective:
                                fragment.data[i] = alpha * v0.data[i] + beta * v1.data[i] + gamma * v2.data[i];
                                break;
                            default:
                                fragment.data[i] = 0;
                                break;
                        }
                    }
                    
                    data_output output;
                    state.fragment_shader(fragment, output, state.uniform_data);
                    
                    int r = std::min(255, std::max(0, (int)(output.output_color[0] * 255)));
                    int g = std::min(255, std::max(0, (int)(output.output_color[1] * 255)));
                    int b = std::min(255, std::max(0, (int)(output.output_color[2] * 255)));
                    state.image_color[pixel_index] = make_pixel(r, g, b);
                    
                    delete[] fragment.data;
                }
            }
        }
    }
}