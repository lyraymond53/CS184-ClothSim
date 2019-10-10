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
  
  // (Placeholder code. You will want to replace it.)
  float ka = .2;
  float kd = 1.0;
  float ks = 1.5;
  vec3 Ia = vec3(1.0, 1.0, 1.0);
  float p = 80;

  vec3 l = u_light_pos - v_position.xyz;
  float r = length(l);

  vec3 Ir2 = (u_light_intensity / (r * r));
  vec3 hnum = (u_light_pos - v_position.xyz) + (u_cam_pos - v_position.xyz);

  vec3 h = hnum / length((u_light_pos - v_position.xyz) + (u_cam_pos - v_position.xyz));

  vec3 ambient = (ka * Ia);
  vec3 diffuse = (kd * Ir2 * max(0, dot(v_normal.xyz, normalize(l))));
  vec3 specular = (ks * Ir2 * pow(max(0, dot(v_normal.xyz, h)), p));

  vec3 myVec = specular;

  out_color = vec4(myVec, 0);
  out_color.a = 1;
}

