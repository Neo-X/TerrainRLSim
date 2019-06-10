#version 130

in		vec3	ViewPos;
in		vec3	Normal;
in		vec4	VertColor;

out         vec4        fragData;

void main()
{
	fragData = vec4(normalize(Normal), 1.f);
}