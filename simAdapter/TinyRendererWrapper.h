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
//#include "tinyrenderer/geometry.h"
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

	TinyRender::Vec3f light_dir; // light source
	TinyRender::Vec3f       eye; // camera position
	TinyRender::Vec3f    center; // camera direction
	TinyRender::Vec3f        up; // camera up vector

	TinyRender::Matrix ModelView; // "OpenGL" state matrices
	TinyRender::Matrix Projection;

	virtual void render();
	virtual std::vector<unsigned char> getPixels();

	virtual void setScene(std::shared_ptr<cScenarioSimChar> scene);
	virtual void addBoxToScene();

	std::shared_ptr<cScenarioSimChar> scenario;
	std::vector<double> zbuffer; // note that the z-buffer is initialized with minimal possible values
	TGAImage framebuffer; // the output image

	std::shared_ptr<TinyRender::Model> m_model;
};


struct Shader : TinyRender::IShader {
	    TinyRender::Model model;
//	    std::shared_ptr<cTinyRendererWrapper> renderer;
	    TinyRender::Matrix ModelView; // "OpenGL" state matrices
		TinyRender::Matrix Projection;

	    TinyRender::Vec3f l;               // light direction in normalized device coordinates
	    TinyRender::mat<2,3, float> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
	    TinyRender::mat<3,3, float> varying_nrm; // normal per vertex to be interpolated by FS
	    TinyRender::mat<3,3, float> ndc_tri;     // triangle in normalized device coordinates

	    Shader(const TinyRender::Model &m, TinyRender::Matrix ModelView,
	    		TinyRender::Matrix Projection, TinyRender::Vec3f light_dir) :
	    	model(m),
	    	ModelView(ModelView),
			Projection(Projection)
			{
//	    	renderer = renderer;
//	    	float x = renderer->light_dir[0];
	    	TinyRender::Vec4f l_dir = TinyRender::embed<4>(TinyRender::Vec3f(light_dir));
	        l = TinyRender::proj<3>((Projection*ModelView*l_dir)).normalize(); // transform the light vector to the normalized device coordinates
	    }

	    virtual TinyRender::Vec4f vertex(const int iface, const int nthvert) {
//	    	std::cout << "Shader: iface: " << iface << " nthvert: " << nthvert << std::endl;
//			std::cout << "Shader: verts: " << model.nverts() << " faces: " << model.nfaces() << " normals: " << model.nnormals() << std::endl;
//			std::cout << "First face size:" << model.face(iface).size() << std::endl;
//			std::cout << "First face verts:" << model.face(iface)[0] << ", "<< model.face(iface)[1] << ", " << model.face(iface)[2] << std::endl;
//			std::cout << "First vertex size:" << model.vert(0)[0] << " model.normal(iface, nthvert)" << model.normal(iface, nthvert)[1] <<std::endl;
//	        TinyRender::Vec4f gl_Vertex = TinyRender::embed<4>(model.vert(iface, nthvert));
			TinyRender::Vec4f vert;
			vert[0] = model.normal(iface, nthvert)[0];
			vert[1] = model.normal(iface, nthvert)[1];
			vert[2] = model.normal(iface, nthvert)[2];
			vert[3] = 0.0f;
	        varying_uv.set_col(nthvert, model.uv(iface, nthvert));
//	        std::cout << "vert: " << vert << std::endl;
//	        std::cout << "renderer->Projection: " << Projection << std::endl;
//	        std::cout << "renderer->ModelView: " << ModelView << std::endl;
	        TinyRender::Matrix tmp_mi = (Projection*ModelView).invert_transpose();
//	        std::cout << "tmp_mi: " << tmp_mi << std::endl;
	        TinyRender::Vec4f tmp = tmp_mi * vert;
	        TinyRender::Vec3f tmp2 = TinyRender::Vec3f(tmp[0], tmp[1], tmp[2]);
	        varying_nrm.set_col(nthvert,
	        		tmp2);
//	        				TinyRender::embed<4>(model.normal(iface, nthvert))));
//	        				vert));
	        vert[3] = 1.0f;
			TinyRender::Vec4f gl_Vertex = Projection*ModelView*vert;
	        ndc_tri.set_col(nthvert, TinyRender::proj<3>(gl_Vertex/gl_Vertex[3]));
	        return gl_Vertex;
	    }

	    virtual bool fragment(const TinyRender::Vec3f bar, TGAColor &color) {
	    	TinyRender::Vec3f bn = (varying_nrm*bar).normalize(); // per-vertex normal interpolation
	    	TinyRender::Vec2f uv = varying_uv * bar; // tex coord interpolation

	        // for the math refer to the tangent space normal mapping lecture
	        // https://github.com/ssloy/tinyrenderer/wiki/Lesson-6bis-tangent-space-normal-mapping
	    	TinyRender::mat<3,3, float> AI; // TinyRender::mat<3,3, float>{ {ndc_tri.col(1) - ndc_tri.col(0), ndc_tri.col(2) - ndc_tri.col(0), bn} }.invert();
	    	AI[0] = ndc_tri.col(1) - ndc_tri.col(0),
	    	AI[1] = ndc_tri.col(2) - ndc_tri.col(0),
			AI[2] = bn;
	    	AI = AI.invert();
	        TinyRender::Vec3f i = AI * TinyRender::Vec3f(varying_uv[0][1] - varying_uv[0][0], varying_uv[0][2] - varying_uv[0][0], 0);
	        TinyRender::Vec3f j = AI * TinyRender::Vec3f(varying_uv[1][1] - varying_uv[1][0], varying_uv[1][2] - varying_uv[1][0], 0);
	        TinyRender::mat<3,3, float> B;
	        B[0] = i.normalize();
	        B[1] = j.normalize();
	        B[2] = bn;
	    	B = B.transpose();

	        TinyRender::Vec3f n = (B * model.normal(uv)).normalize(); // transform the normal from the texture to the tangent space

	        double diff = std::max(0.f, n*l); // diffuse light intensity
	        TinyRender::Vec3f r = (n*(n*l)*2 - l).normalize(); // reflected light direction, specular mapping is described here: https://github.com/ssloy/tinyrenderer/wiki/Lesson-6-Shaders-for-the-software-renderer
	        double spec = std::pow(std::max(r.z, 0.f), 5+model.specular(uv)); // specular intensity, note that the camera lies on the z-axis (in ndc), therefore simple r.z

	        TGAColor c = model.diffuse(uv);
	        for (int i=0; i<3; i++)
	            color[i] = std::min<int>(10 + c[i]*(diff + spec), 255); // (a bit of ambient light, diff + spec), clamp the result

	        return false; // the pixel is not discarded
	    }
	};

#endif /* SIMADAPTER_H_ */
