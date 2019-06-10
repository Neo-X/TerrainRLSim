#version 130

uniform		sampler2D	gBufferTex;
in			vec2		tex_coord;

out         vec4        fragColor;

void main(void) {
	fragColor = texture2D( gBufferTex, tex_coord.xy );
}