#include <limits>
#include <iostream>
#include "model.h"
#include "our_gl.h"

constexpr int width  = 800; // output image size
constexpr int height = 800;

//extern TinyRender::Matrix ModelView; // "OpenGL" state matrices
//extern TinyRender::Matrix Projection;

struct Shader : TinyRender::IShader {
    TinyRender::Model &model;
    TinyRender::Matrix &ModelView; // "OpenGL" state matrices
	TinyRender::Matrix &Projection;
    TinyRender::Vec3f l;               // light direction in normalized device coordinates
    TinyRender::mat<2,3, float> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
    TinyRender::mat<3,3, float> varying_nrm; // normal per vertex to be interpolated by FS
    TinyRender::mat<3,3, float> ndc_tri;     // triangle in normalized device coordinates

    Shader(TinyRender::Model &m, TinyRender::Matrix &ModelView, TinyRender::Matrix &Projection,
    		TinyRender::Vec3f &light_dir) :
    	model(m),
		ModelView(ModelView),
		Projection(Projection)
		{
        l = TinyRender::proj<3>((Projection*ModelView*TinyRender::embed<4>(light_dir, 0.f))).normalize(); // transform the light vector to the normalized device coordinates
    }

    virtual TinyRender::Vec4f vertex(const int iface, const int nthvert) {
        varying_uv.set_col(nthvert, model.uv(iface, nthvert));
        varying_nrm.set_col(nthvert, TinyRender::proj<3>((Projection*ModelView).invert_transpose()*TinyRender::embed<4>(model.normal(iface, nthvert), 0.f)));
        TinyRender::Vec4f gl_Vertex = Projection*ModelView*TinyRender::embed<4>(model.vert(iface, nthvert));
        ndc_tri.set_col(nthvert, TinyRender::proj<3>(gl_Vertex/gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(const TinyRender::Vec3f bar, TGAColor &color) {
    	TinyRender::Vec3f bn = (varying_nrm*bar).normalize(); // per-vertex normal interpolation
    	TinyRender::Vec2f uv = varying_uv*bar; // tex coord interpolation
//    	std::cout << "uv: " << uv << std::endl;

        // for the math refer to the tangent space normal mapping lecture
        // https://github.com/ssloy/tinyrenderer/wiki/Lesson-6bis-tangent-space-normal-mapping
    	TinyRender::mat<3,3, float> AI; // TinyRender::mat<3,3, float>{ {ndc_tri.col(1) - ndc_tri.col(0), ndc_tri.col(2) - ndc_tri.col(0), bn} }.invert();
		AI[0] = ndc_tri.col(1) - ndc_tri.col(0);
		AI[1] = ndc_tri.col(2) - ndc_tri.col(0);
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

        double diff = std::max(0.2f, n*l); // diffuse light intensity
        TinyRender::Vec3f r = (n*(n*l)*2 - l).normalize(); // reflected light direction, specular mapping is described here: https://github.com/ssloy/tinyrenderer/wiki/Lesson-6-Shaders-for-the-software-renderer
        double spec = std::pow(std::max(r.z, 0.f), 5+model.specular(uv)); // specular intensity, note that the camera lies on the z-axis (in ndc), therefore simple r.z

        TGAColor c = model.diffuse(uv);
//        c = TGAColor(255, 255, 255, 255);
//        std::cout << "text colour: " << int(c[0]) << ", " << c[1] << ", " << c[2] << std::endl;
        for (int i=0; i<3; i++)
            color[i] = std::min<int>(10 + c[i]*(diff + spec), 255); // (a bit of ambient light, diff + spec), clamp the result

        return false; // the pixel is not discarded
    }
};

int main(int argc, char** argv) {
    if (2>argc) {
        std::cerr << "Usage: " << argv[0] << " obj/model.obj" << std::endl;
        return 1;
    }
    TinyRender::Vec3f light_dir(1,1,1); // light source
    const TinyRender::Vec3f       eye(1,1,3); // camera position
    const TinyRender::Vec3f    center(0,0,0); // camera direction
    const TinyRender::Vec3f        up(0,1,0); // camera up vector

    std::vector<float> zbuffer(width*height, -std::numeric_limits<double>::max()); // note that the z-buffer is initialized with minimal possible values
    TGAImage framebuffer(width, height, TGAImage::RGB); // the output image
    TinyRender::Matrix ModelView = TinyRender::lookat(eye, center, up);                            // build the ModelView matrix
    const TinyRender::Matrix viewPort = TinyRender::viewport(width/8, height/8, width*3/4, height*3/4); // build the Viewport matrix
    TinyRender::Matrix Projection = TinyRender::projection(-1.f/(eye-center).norm());               // build the Projection matrix               // build the Projection matrix

    for (int m=1; m<argc; m++) { // iterate through all input objects
    	TinyRender::Model model(argv[m]);
        Shader shader(model, ModelView, Projection, light_dir);
        for (int i=0; i<model.nfaces(); i++) { // for every triangle
//        	TinyRender::Vec4f clip_vert[3]; // triangle coordinates (clip coordinates), written by VS, read by FS
        	TinyRender::mat<4, 4, float> clip_vert;
            for (int j=0; j<3; j++)
                clip_vert[j] = shader.vertex(i, j); // call the vertex shader for each triangle vertex
            triangle2(clip_vert, shader, framebuffer, zbuffer, viewPort); // actual rasterization routine call
        }
    }
    framebuffer.write_tga_file("framebuffer.tga"); // the vertical flip is moved inside the function
    return 0;
}

