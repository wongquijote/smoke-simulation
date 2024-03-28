#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 p = vec3(v_position.x, v_position.y, v_position.z);
  vec3 n = vec3(v_normal.x, v_normal.y, v_normal.z);
  vec3 w_o = normalize(p - u_cam_pos);
  vec3 w_i = reflect(w_o, n);
  out_color = texture(u_texture_cubemap, w_i);
  out_color.a = 1;
}
