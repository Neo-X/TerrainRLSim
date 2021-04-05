/*
 * SimAdapter.h
 *
 *  Created on: 2017-01-20
 *      Author: gberseth
 */

#ifndef TINYRENDERERWRAPPER_H_
#define TINYRENDERERWRAPPER_H_
#include <vector>
#include <string>
#include <limits>
#include <iostream>
#include "tinyrenderer/model.h"
#include "tinyrenderer/our_gl.h"
#include <memory>
#include "scenarios/Scenario.h"
#include "scenarios/ScenarioSimChar.h"
//#include "tinyrenderer/geometry.h"

/*
 * Return true if the file exists
 */


class cTinyRendererWrapper {
public:
	cTinyRendererWrapper();
	virtual ~cTinyRendererWrapper();

	virtual void init();

//protected:
	int width; // output image size
	int height;

	vec3 light_dir; // light source
	vec3       eye; // camera position
	vec3    center; // camera direction
	vec3        up; // camera up vector

	mat<4,4> ModelView; // "OpenGL" state matrices
	mat<4,4> Projection;

	virtual void render();
	virtual std::vector<unsigned char> getPixels();

	virtual void setScene(std::shared_ptr<cScenarioSimChar> scene);
	virtual void addBoxToScene();

	std::shared_ptr<cScenarioSimChar> scenario;
	std::vector<double> zbuffer; // note that the z-buffer is initialized with minimal possible values
	TGAImage framebuffer; // the output image
};

struct Shader : IShader {
	    const Model &model;
	    cTinyRendererWrapper * renderer;
	    vec3 l;               // light direction in normalized device coordinates
	    mat<2,3> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
	    mat<3,3> varying_nrm; // normal per vertex to be interpolated by FS
	    mat<3,3> ndc_tri;     // triangle in normalized device coordinates

	    Shader(const Model &m, cTinyRendererWrapper * renderer) : model(m) {
	    	renderer = renderer;
	        l = proj<3>((renderer->Projection*renderer->ModelView*embed<4>(renderer->light_dir, 0.))).normalize(); // transform the light vector to the normalized device coordinates
	    }

	    virtual vec4 vertex(const int iface, const int nthvert) {
	        varying_uv.set_col(nthvert, model.uv(iface, nthvert));
	        varying_nrm.set_col(nthvert, proj<3>((renderer->Projection*renderer->ModelView).invert_transpose()*embed<4>(model.normal(iface, nthvert), 0.)));
	        vec4 gl_Vertex = renderer->Projection*renderer->ModelView*embed<4>(model.vert(iface, nthvert));
	        ndc_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
	        return gl_Vertex;
	    }

	    virtual bool fragment(const vec3 bar, TGAColor &color) {
	        vec3 bn = (varying_nrm*bar).normalize(); // per-vertex normal interpolation
	        vec2 uv = varying_uv*bar; // tex coord interpolation

	        // for the math refer to the tangent space normal mapping lecture
	        // https://github.com/ssloy/tinyrenderer/wiki/Lesson-6bis-tangent-space-normal-mapping
	        mat<3,3> AI = mat<3,3>{ {ndc_tri.col(1) - ndc_tri.col(0), ndc_tri.col(2) - ndc_tri.col(0), bn} }.invert();
	        vec3 i = AI * vec3(varying_uv[0][1] - varying_uv[0][0], varying_uv[0][2] - varying_uv[0][0], 0);
	        vec3 j = AI * vec3(varying_uv[1][1] - varying_uv[1][0], varying_uv[1][2] - varying_uv[1][0], 0);
	        mat<3,3> B = mat<3,3>{ {i.normalize(), j.normalize(), bn} }.transpose();

	        vec3 n = (B * model.normal(uv)).normalize(); // transform the normal from the texture to the tangent space

	        double diff = std::max(0., n*l); // diffuse light intensity
	        vec3 r = (n*(n*l)*2 - l).normalize(); // reflected light direction, specular mapping is described here: https://github.com/ssloy/tinyrenderer/wiki/Lesson-6-Shaders-for-the-software-renderer
	        double spec = std::pow(std::max(r.z, 0.), 5+model.specular(uv)); // specular intensity, note that the camera lies on the z-axis (in ndc), therefore simple r.z

	        TGAColor c = model.diffuse(uv);
	        for (int i=0; i<3; i++)
	            color[i] = std::min<int>(10 + c[i]*(diff + spec), 255); // (a bit of ambient light, diff + spec), clamp the result

	        return false; // the pixel is not discarded
	    }
	};

#endif /* SIMADAPTER_H_ */
