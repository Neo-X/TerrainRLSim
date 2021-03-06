#version 130

// uniform mat4 gl_ModelViewMatrix;
// uniform mat4 gl_ProjectionMatrix;

in vec3 inPosition;
in vec3 inNormal;
in vec2 inTexCoord;

uniform vec4 v_color;
uniform mat4 projectionMat;
uniform mat4 modelviewMat;

out vec3 ViewPos;
out vec3 Normal;
out	vec4 VertColor;
out vec2 TexCoord;

void main()
{
	// vec4 view_pos = gl_ModelViewMatrix * vec4(inPosition.xyz, 1.f);
	vec4 view_pos = modelviewMat * vec4(inPosition.xyz, 1.f);
    // vec4 WVP_Pos = gl_ProjectionMatrix * view_pos;
    vec4 WVP_Pos = projectionMat * view_pos;

    gl_Position = WVP_Pos;
	ViewPos = view_pos.xyz;
	Normal = (gl_ModelViewMatrix * vec4(inNormal.xyz, 0.f)).xyz;
	// VertColor = gl_Color;
	VertColor = v_color;
	TexCoord = inTexCoord.xy;
}
