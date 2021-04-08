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

//	std::shared_ptr<TinyRender::Model> m_model;
	std::vector<TinyRender::Model> m_models;
};


struct Shader : TinyRender::IShader {
    TinyRender::Model &model;
    TinyRender::Matrix &ModelView; // "OpenGL" state matrices
	TinyRender::Matrix &Projection;
    TinyRender::Vec3f l;               // light direction in normalized device coordinates
    TinyRender::mat<2,3, float> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
    TinyRender::mat<3,3, float> varying_nrm; // normal per vertex to be interpolated by FS
    TinyRender::mat<3,3, float> ndc_tri;     // triangle in normalized device coordinates
//    TinyRender::Vec3f m_colorRGBA;
	float m_ambient_coefficient;
	float m_diffuse_coefficient;
	float m_specular_coefficient;

    Shader(TinyRender::Model &m, TinyRender::Matrix &ModelView, TinyRender::Matrix &Projection,
    		TinyRender::Vec3f &light_dir) :
    	model(m),
		ModelView(ModelView),
		Projection(Projection)
		{
        l = TinyRender::proj<3>((Projection*ModelView*TinyRender::embed<4>(light_dir, 0.f))).normalize(); // transform the light vector to the normalized device coordinates
    	m_ambient_coefficient = 0.6;
    	m_diffuse_coefficient = 0.35;
    	m_specular_coefficient = 0.5;
    }

    virtual TinyRender::Vec4f vertex(const int iface, const int nthvert) {
        varying_uv.set_col(nthvert, model.uv(iface, nthvert));
        varying_nrm.set_col(nthvert, TinyRender::proj<3>((Projection*ModelView).invert_transpose()*TinyRender::embed<4>(model.normal(iface, nthvert), 0.f)));
        TinyRender::Vec4f gl_Vertex = Projection*ModelView*TinyRender::embed<4>(model.vert(iface, nthvert));
        ndc_tri.set_col(nthvert, TinyRender::proj<3>(gl_Vertex/gl_Vertex[3]));
        return gl_Vertex;
    }

//    virtual bool fragment(const TinyRender::Vec3f bar, TGAColor &color) {
//    	TinyRender::Vec3f bn = (varying_nrm*bar).normalize(); // per-vertex normal interpolation
//    	TinyRender::Vec2f uv = varying_uv*bar; // tex coord interpolation
//    	std::cout << "uv: " << uv << std::endl;
//
//        // for the math refer to the tangent space normal mapping lecture
//        // https://github.com/ssloy/tinyrenderer/wiki/Lesson-6bis-tangent-space-normal-mapping
//    	TinyRender::mat<3,3, float> AI; // TinyRender::mat<3,3, float>{ {ndc_tri.col(1) - ndc_tri.col(0), ndc_tri.col(2) - ndc_tri.col(0), bn} }.invert();
//		AI[0] = ndc_tri.col(1) - ndc_tri.col(0);
//		AI[1] = ndc_tri.col(2) - ndc_tri.col(0);
//		AI[2] = bn;
//		AI = AI.invert();
//		std::cout << " AI: " << AI << " ndc_tri: " << ndc_tri << std::endl;
//        TinyRender::Vec3f i = AI * TinyRender::Vec3f(varying_uv[0][1] - varying_uv[0][0], varying_uv[0][2] - varying_uv[0][0], 0);
//        TinyRender::Vec3f j = AI * TinyRender::Vec3f(varying_uv[1][1] - varying_uv[1][0], varying_uv[1][2] - varying_uv[1][0], 0);
//        TinyRender::mat<3,3, float> B;
//		B[0] = i.normalize();
//		B[1] = j.normalize();
//		B[2] = bn;
//		B = B.transpose();
//		TinyRender::Vec3f f_normal = model.normal(uv);
//		std::cout << " B: " << B << " model.normal(uv): " << f_normal << std::endl;
//
//        TinyRender::Vec3f n = (B * model.normal(uv)).normalize(); // transform the normal from the texture to the tangent space
//
//        double diff = std::max(0.2f, n*l); // diffuse light intensity
//        TinyRender::Vec3f r = (n*(n*l)*2 - l).normalize(); // reflected light direction, specular mapping is described here: https://github.com/ssloy/tinyrenderer/wiki/Lesson-6-Shaders-for-the-software-renderer
//        double spec = std::pow(std::max(r.z, 0.f), 5+model.specular(uv)); // specular intensity, note that the camera lies on the z-axis (in ndc), therefore simple r.z
//
//        TGAColor c = model.diffuse(uv);
////        c = TGAColor(255, 255, 255, 255);
//        std::cout << "text colour: " << int(c[0]) << ", " << int(c[1]) << ", " << int(c[2]) <<
//        		" diff: " << diff << " spec: " << spec << " n: " << n << " r: " << r <<  std::endl;
//        for (int i=0; i<3; i++)
//            color[i] = std::min<int>(10 + c[i]*(diff + spec), 255); // (a bit of ambient light, diff + spec), clamp the result
//
//        return false; // the pixel is not discarded
//    }
    virtual bool fragment(const TinyRender::Vec3f bar, TGAColor &color) {
    	TinyRender::Vec3f bn = (varying_nrm * bar).normalize();
    	TinyRender::Vec2f uv = varying_uv * bar;

    	TinyRender::Vec3f reflection_direction = (bn * (bn * l * 2.f) - l).normalize();
		float specular = std::pow(TinyRender::b3Max(reflection_direction.z, 0.f),
									model.specular(uv));
		float diffuse = TinyRender::b3Max(0.f, bn * l);

		color = model.diffuse(uv);

		color[0] *= model.getColorRGBA()[0];
		color[1] *= model.getColorRGBA()[1];
		color[2] *= model.getColorRGBA()[2];
		color[3] *= model.getColorRGBA()[3];

		for (int i = 0; i < 3; ++i)
		{
			int orgColor = 0;
//			float floatColor = (m_ambient_coefficient * color[i] + shadow * (m_diffuse_coefficient * diffuse + m_specular_coefficient * specular) * color[i] * m_light_color[i]);
			float floatColor = (m_ambient_coefficient * color[i] * (m_diffuse_coefficient * diffuse + m_specular_coefficient * specular) * color[i]);
			if (floatColor==floatColor)
			{
				orgColor=int(floatColor);
			}
			color[i] = TinyRender::b3Min(orgColor, 255);
		}

		return false;
	}
};

#endif /* SIMADAPTER_H_ */
