#version 330

uniform vec4 u_color;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_color;

out vec4 out_color;

void main() {
  out_color = v_color;
}
