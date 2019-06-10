#version 130

in vec3 Position;
// uniform mat4 gl_ModelViewMatrix;
// uniform mat4 gl_ProjectionMatrix;
// uniform 	mat4 	projectionMat;
// uniform 	mat4 	modelviewMat;

// uniform vec4 v_color;
uniform mat4 projectionMat;
uniform mat4 modelviewMat;

out 		vec3 	TexCoord0;

void main()
{
	// vec4 view_pos = gl_ModelViewMatrix * vec4(inPosition.xyz, 1.f);
	vec3 view_pos = mat3(modelviewMat) * Position.xyz;
    // vec4 WVP_Pos = gl_ProjectionMatrix * view_pos;
    vec4 WVP_Pos = projectionMat * vec4(view_pos, 1.0f);
    
	// vec3 view_pos = mat3( gl_ModelViewMatrix ) * Position.xyz;
    // vec4 WVP_Pos = gl_ProjectionMatrix * vec4( view_pos, 1.f );
	WVP_Pos = vec4(WVP_Pos.xyww);
    gl_Position = WVP_Pos;
    TexCoord0 = Position.xyz;
}
