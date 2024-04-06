#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  
  vec3 n = vec3(v_normal.x, v_normal.y, v_normal.z);
  vec3 p = vec3(v_position.x, v_position.y, v_position.z);
  vec3 l = u_light_pos - p;
  vec3 v = u_cam_pos - p;
  vec3 h = (l / length(l)) + (v / length(v));
  h = h / length(h);
  
  vec3 intensity = u_light_intensity / dot(l, l);
  vec3 diffuse = (1.0) * intensity * max(0.0, dot(n, l / length(l)));
  vec3 ambient = (0.15) * vec3(1.0, 1.0, 1.0);
  vec3 specular = (1.0) * intensity * pow(max(0.0, dot(n, h)), 30.0);
  out_color = vec4(ambient, 0.0) + vec4(diffuse, 0.0) + vec4(specular, 0.0);
  out_color.a = 1;
}

