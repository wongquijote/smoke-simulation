#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE
  vec3 t = vec3(v_tangent.x, v_tangent.y, v_tangent.z);
  vec3 n = vec3(v_normal.x, v_normal.y, v_normal.z);
  vec3 b = cross(n, t);
  float dU = (h(v_uv + vec2((1.0)/u_texture_2_size.x, 0.0)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  float dV = (h(v_uv + vec2(0.0, (1.0)/u_texture_2_size.y)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  vec3 n_o = vec3(-dU, -dV, 1.0);
  mat3 tbn = mat3(t, b, n);
  n = tbn * n_o;

  vec3 p = vec3(v_position.x, v_position.y, v_position.z);
  vec3 l = u_light_pos - p;
  vec3 v = u_cam_pos - p;
  vec3 h = (l / length(l)) + (v / length(v));
  h = h / length(h);
  
  vec3 intensity = u_light_intensity / dot(l, l);
  vec3 diffuse = (1.0) * intensity * max(0.0, dot(n, l / length(l)));
  vec3 ambient = (0.1) * vec3(1.0, 1.0, 1.0);
  vec3 specular = (1.0) * intensity * pow(max(0.0, dot(n, h)), 30.0);
  out_color = vec4(ambient, 0.0) + vec4(diffuse, 0.0) + vec4(specular, 0.0);
  out_color.a = 1;
}

