#version 130

uniform		sampler2D	gReflectionTex;
varying		vec2		tex_coord;

out         vec4        fragColor;

void main(void) {
	fragColor = vec4(texture2D(gReflectionTex, tex_coord.xy).rgb, 0);
}