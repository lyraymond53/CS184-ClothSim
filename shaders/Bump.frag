#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_3, uv).r;
}

void main() {
  // YOUR CODE HERE
  vec3 normal3d = normalize(v_normal.xyz);
  vec3 tan = normalize(v_tangent.xyz);
  vec3 bitan = normalize(cross(normal3d, tan));

  mat3 tbn = mat3(tan, bitan, normal3d);


  float hk = u_height_scaling * u_normal_scaling;
  float du = (h(vec2(v_uv.x + 1 / u_texture_3_size.x, v_uv.y)) - h(v_uv)) * hk;
  float dv = (h(vec2(v_uv.x, v_uv.y + 1 / u_texture_3_size.y)) - h(v_uv)) * hk;

  vec3 n0 = vec3(-du, -dv, 1);
  vec4 nd = vec4(tbn * n0, 0);

  // Phong
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
  vec3 diffuse = (kd * Ir2 * max(0, dot(nd.xyz, normalize(l))));
  vec3 specular = (ks * Ir2 * pow(max(0, dot(nd.xyz, h)), p));

  vec3 myVec = ambient + diffuse + specular;

  out_color = vec4(myVec, 0);
  // (Placeholder code. You will want to replace it.)
  out_color.a = 1;
}

