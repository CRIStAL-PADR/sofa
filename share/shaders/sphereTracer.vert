#version 430

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 displacement_position;

uniform mat4 projection_matrix;
uniform mat4 object_matrix;


void main() 
{
  vec4 world_position = object_matrix * vec4(vertex_position, 1.0);
  gl_Position = projection_matrix *  world_position;
}
