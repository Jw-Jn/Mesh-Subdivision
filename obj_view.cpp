/*
	Sample code by Wallace Lira <http://www.sfu.ca/~wpintoli/> based on
	the four Nanogui examples and also on the sample code provided in
		  https://github.com/darrenmothersele/nanogui-test

	All rights reserved. Use of this source code is governed by a
	BSD-style license that can be found in the LICENSE.txt file.
*/

// add while windows
// #define _CRT_SECURE_NO_DEPRECATE
// #include <GL/glew.h>

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/button.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>
#include <nanogui/combobox.h>
#include <nanogui/progressbar.h>
#include <nanogui/entypo.h>
#include <nanogui/messagedialog.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/imagepanel.h>
#include <nanogui/imageview.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/colorwheel.h>
#include <nanogui/graph.h>
#include <nanogui/tabwidget.h>
#include <nanogui/glcanvas.h>
#include <iostream>
#include <string>

// Includes for the GLTexture class.
#include <cstdint>
#include <memory>
#include <utility>

#include "WingEdge.h"


#if defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#if defined(_WIN32)
#  pragma warning(push)
#  pragma warning(disable: 4457 4456 4005 4312)
#endif

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#if defined(_WIN32)
#  pragma warning(pop)
#endif
#if defined(_WIN32)
#  if defined(APIENTRY)
#    undef APIENTRY
#  endif
#  include <windows.h>
#endif

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::pair;
using std::to_string;

using nanogui::Screen;
using nanogui::Window;
using nanogui::GroupLayout;
using nanogui::Button;
using nanogui::CheckBox;
using nanogui::Vector2f;
using nanogui::Vector2i;
using nanogui::MatrixXu;
using nanogui::MatrixXf;
using nanogui::Label;
using nanogui::Arcball;

class MyGLCanvas : public nanogui::GLCanvas {
public:
	WingEdge mesh;

	MyGLCanvas(Widget *parent) : nanogui::GLCanvas(parent) {
		using namespace nanogui;

		mShader.initFromFiles("a_smooth_shader", "StandardShading.vertexshader", "StandardShading.fragmentshader");
		
		mesh.loadOBJfile("./cube.obj");
		updateMesh();

		// After binding the shader to the current context we can send data to opengl that will be handled
		// by the vertex shader and then by the fragment shader, in that order.
		// if you want to know more about modern opengl pipeline take a look at this link
		// https://www.khronos.org/opengl/wiki/Rendering_Pipeline_Overview
		mShader.bind();

		mShader.uploadAttrib("vertexPosition_modelspace", positions);
		mShader.uploadAttrib("color", colors);
		mShader.uploadAttrib("vertexNormal_modelspace", normals);

		// ViewMatrixID
		// change your rotation to work on the camera instead of rotating the entire world with the MVP matrix
		Matrix4f V;
		V.setIdentity();
		// V = lookAt(Vector3f(0,20,0), Vector3f(0,0,0), Vector3f(0,1,0));
		mShader.setUniform("V", V);

		// ModelMatrixID
		Matrix4f M;
		M.setIdentity();
		mShader.setUniform("M", M);

		// This the light origin position in your environment, which is totally arbitrary
		// however it is better if it is behind the observer
		mShader.setUniform("LightPosition_worldspace", Vector3f(-2, 6, -4));

		mScale.setIdentity();
		mRotation.setIdentity();
		mTranslation.setIdentity();
		renderMode = 0;
	}

	//flush data on call
	~MyGLCanvas() {
		mShader.free();
	}

	//Method to update the mesh itself, can change the size of it dynamically, as shown later
	void updateMeshPositions(MatrixXf newPositions) {
		positions = newPositions;
	}

	//Method to update the rotation on each axis
	void setRotation() {
		mRotation.setIdentity();
		mRotation.topLeftCorner<3, 3>() = Eigen::Matrix3f(Eigen::AngleAxisf(rotX, nanogui::Vector3f::UnitX()) *
			Eigen::AngleAxisf(rotY, nanogui::Vector3f::UnitY()) *
			Eigen::AngleAxisf(rotZ, nanogui::Vector3f::UnitZ()));
	}

	void setRotationX(float x) {
		rotX = x;
	}

	void setRotationY(float y) {
		rotY = y;
	}

	void setRotationZ(float z) {
		rotZ = z;
	}

	void setRenderMode(int a){
		renderMode = a;
	}

	void setScale(float vScale) {
		mScale.setIdentity();
		mScale.topLeftCorner<3, 3>() *= vScale;
	}

	void setTranslate() {
		mTranslation.setIdentity();
		mTranslation.topRightCorner<3, 1>() = nanogui::Vector3f(transX, transY, 0.0f);
	}

	void setTranslateX(float x) {
		transX = x;
	}

	void setTranslateY(float y) {
		transY = y;
	}

	void reset() {
		setTranslateX(0.0);
		setTranslateY(0.0);
		setRotationX(0.0);
		setRotationY(0.0);
		setRotationZ(0.0);
	}

	void updateMesh(){
		std::vector<std::vector<Wvertex *>> faces = mesh.extractVerticesOfFaces();
		faceNum = faces.size();

		positions = MatrixXf(3, faceNum *9);
		normals = MatrixXf(3, faceNum *9);
		colors = MatrixXf(3, faceNum *9);

		int k = 0;
		for (int i = 0; i < faceNum; ++i) {
			for (int j = 0; j < faces[i].size(); ++j) {
				vec3 v = mesh.getVertex(faces[i][j]);
				vec3 n = mesh.getNorm(faces[i][j]);
				positions.col(k) << v.x, v.y, v.z;
				normals.col(k) << n.x, n.y, n.z;
				colors.col(k) << 1, 0, 0;
				k++;
			}
		}

		for (int i = 0; i < faceNum; ++i) {
			for (int j = 0; j < faces[i].size(); ++j) {
				vec3 n = mesh.getNorm(faces[i][j]);
				vec3 v = mesh.getVertex(faces[i][j]) + n * 0.001;
				positions.col(k) << v.x, v.y, v.z;
				normals.col(k) << n.x, n.y, n.z;
				colors.col(k) << 0, 0, 0;
				k++;

				n = mesh.getNorm(faces[i][(j + 1) % faces[i].size()]);
				v = mesh.getVertex(faces[i][(j+1)% faces[i].size()]) + n * 0.001;
				positions.col(k) << v.x, v.y, v.z;
				normals.col(k) << n.x, n.y, n.z;
				colors.col(k) << 0, 0, 0;
				k++;
			}
		}
	}

	//OpenGL calls this method constantly to update the screen.
	virtual void drawGL() override {
		using namespace nanogui;

		//refer to the previous explanation of mShader.bind();
		mShader.bind();

		//this simple command updates the positions matrix. You need to do the same for color and indices matrices too
		mShader.uploadAttrib("vertexPosition_modelspace", positions);
		mShader.uploadAttrib("color", colors);
		mShader.uploadAttrib("vertexNormal_modelspace", normals);

		//This is a way to perform a simple rotation using a 4x4 rotation matrix represented by rmat
		//mvp stands for ModelViewProjection matrix
		Matrix4f mvp;
		mvp.setIdentity();
		mvp.topLeftCorner<3, 3>() = mvp.topLeftCorner<3, 3>();
		setTranslate();
		setRotation();
		mvp = mTranslation * mRotation * mScale * mvp;
		mShader.setUniform("MVP", mvp);

		Matrix4f V;
		V.setIdentity();
		// V = lookAt(Vector3f(0, 12, 0), Vector3f(0, 0, 0), Vector3f(0, 1, 0));
		// V = mTranslation * mRotation * mScale * V;
		mShader.setUniform("V", V);

		Matrix4f M;
		M.setIdentity();
		//M = mTranslation * mRotation * mScale * M;
		mShader.setUniform("M", M);

		// If enabled, does depth comparisons and update the depth buffer.
		// Avoid changing if you are unsure of what this means.
		glEnable(GL_DEPTH_TEST);

		/* Draw 12 triangles starting at index 0 of your indices matrix */
		/* Try changing the first input with GL_LINES, this will be useful in the assignment */
		/* Take a look at this link to better understand OpenGL primitives */
		/* https://www.khronos.org/opengl/wiki/Primitive */

		//12 triangles, each has three vertices
		MatrixXf flat_normals;

		switch (renderMode)
		{	
			// flat shaded
			case 0:	
				flat_normals = MatrixXf(3, faceNum * 3);
				for (int i = 0; i < faceNum * 3; i++)
				{
					if (i % 3 == 0)
						flat_normals.col(i) = (normals.col(i) + normals.col(i + 1) + normals.col(i + 2)) / 3;
					else
						flat_normals.col(i) = flat_normals.col(i - 1);
				}
				mShader.uploadAttrib("vertexNormal_modelspace", flat_normals);

				mShader.drawArray(GL_TRIANGLES, 0, faceNum * 3);
				break;

			// smooth shaded
			case 1:	
				mShader.drawArray(GL_TRIANGLES, 0, faceNum * 3);
				break;

			// wireframe
			case 2:
				mShader.drawArray(GL_LINES, faceNum * 3, faceNum * 9);
				break;

			// with edge
			case 3:
				mShader.drawArray(GL_TRIANGLES, 0, faceNum * 3);
				mShader.drawArray(GL_LINES, faceNum * 3, faceNum * 9);
				break;
		}

		glDisable(GL_DEPTH_TEST);
	}

	//Instantiation of the variables that can be acessed outside of this class to interact with the interface
	//Need to be updated if a interface element is interacting with something that is inside the scope of MyGLCanvas
private:
	MatrixXf positions;
	MatrixXf normals;
	MatrixXf colors;

	nanogui::GLShader mShader;
	Eigen::Matrix4f mRotation;
	Eigen::Matrix4f mScale;
	Eigen::Matrix4f mTranslation;
	float transX, transY, rotX, rotY, rotZ;
	int renderMode;
	int faceNum;
};


class ExampleApplication : public nanogui::Screen {
public:
	ExampleApplication() : nanogui::Screen(Eigen::Vector2i(900, 600), "NanoGUI Cube and Menus", false) {
		using namespace nanogui;
		//OpenGL canvas demonstration

		//First, we need to create a window context in which we will render both the interface and OpenGL canvas
		Window *window = new Window(this, "GLCanvas Demo");
		window->setPosition(Vector2i(15, 15));
		window->setLayout(new GroupLayout());

		// add while windows
		/*glewExperimental = GL_TRUE;
		GLenum err = glewInit();

		if (GLEW_OK != err){fprintf(stderr, "Error: %s\n", glewGetErrorString(err));}*/
		///////////////////

		//OpenGL canvas initialization, we can control the background color and also its size
		mCanvas = new MyGLCanvas(window);
		mCanvas->setBackgroundColor({ 100, 100, 100, 255 });
		mCanvas->setSize({ 400, 400 });

		//This is how we add widgets, in this case, they are connected to the same window as the OpenGL canvas
		Widget *tools = new Widget(window);
		tools->setLayout(new BoxLayout(Orientation::Horizontal,
			Alignment::Middle, 0, 5));

		//widgets demonstration
		nanogui::GLShader mShader;

		//Then, we can create another window and insert other widgets into it
		Window *anotherWindow = new Window(this, "Basic widgets");
		anotherWindow->setPosition(Vector2i(500, 15));
		anotherWindow->setLayout(new GroupLayout());

		// Demonstrates how a button called "New Mesh" can update the positions matrix.
		// todo This is just a demonstration, you actually need to bind mesh updates with the open file interface
		/*Button *button = new Button(anotherWindow, "New Mesh");
		button->setCallback([&] {
			//MatrixXf has dynamic size, so you can actually change its dimensions on the fly here
			//Make sure that the new mesh is not overblown by scaling it to a proper size and centering at origin
			//If you do not do that, the object may not appear at all, impacting the tests
			MatrixXf newPositions = MatrixXf(3, 8);
			newPositions.col(0) << -2, 1, 1;
			newPositions.col(1) << -2, 1, -1;
			newPositions.col(2) << 1, 1, -1;
			newPositions.col(3) << 1, 1, 1;
			newPositions.col(4) << -1, -2, 1;
			newPositions.col(5) << -1, -2, -1;
			newPositions.col(6) << 1, -1, -2;
			newPositions.col(7) << 1, -1, 2;
			mCanvas->updateMeshPositions(newPositions);
		});
		button->setTooltip("Demonstrates how a button can update the positions matrix.");*/

		//this is how we write captions on the window, if you do not want to write inside a button	
		new Label(anotherWindow, "Rotate", "sans-bold");

		Widget *panelRot = new Widget(anotherWindow);
		panelRot->setLayout(new GridLayout(Orientation::Horizontal, 2,
			Alignment::Middle, 0, 0));

		//Demonstration of rotation along one axis of the mesh using a simple slider, you can have three of these, one for each dimension
		{new Label(panelRot, "x :", "sans-bold");
		Slider *rotSliderX = new Slider(panelRot);
		rotSliderX->setValue(0.5f);
		rotSliderX->setFixedWidth(150);
		rotSliderX->setCallback([&](float value) {
			// the middle point should be 0 rad
			// then we need to multiply by 2 to make it go from -1. to 1.
			// then we make it go from -2*M_PI to 2*M_PI
			float radians = (value - 0.5f) * 2 * 2 * 3.1415;
			//then use this to rotate on just one axis
			mCanvas->setRotationX(radians);
			//float scale = (value - 0.5f) * 2;
			//mCanvas->setScale(nanogui::Vector3f(scale, scale, scale));
			//when you implement the other sliders and/or the Arcball, you need to keep track
			//of the other rotations used for the second and third axis... It will not stay as 0.0f
		}); }

		{new Label(panelRot, "y :", "sans-bold");
		Slider *rotSliderY = new Slider(panelRot);
		rotSliderY->setValue(0.5f);
		rotSliderY->setFixedWidth(150);
		rotSliderY->setCallback([&](float value) {
			float radians = (value - 0.5f) * 2 * 2 * 3.1415;
			mCanvas->setRotationY(radians);
		}); }

		{new Label(panelRot, "z :", "sans-bold");
		Slider *rotSliderZ = new Slider(panelRot);
		rotSliderZ->setValue(0.5f);
		rotSliderZ->setFixedWidth(150);
		rotSliderZ->setCallback([&](float value) {
			float radians = (value - 0.5f) * 2 * 2 * 3.1415;
			mCanvas->setRotationZ(radians);
		}); }

		//Here is how you can get the string that represents file paths both for opening and for saving.
		//you need to implement the rest of the parser logic.
		new Label(anotherWindow, "File dialog", "sans-bold");
		tools = new Widget(anotherWindow);
		tools->setLayout(new BoxLayout(Orientation::Horizontal,
			Alignment::Middle, 0, 6));
		Button *b = new Button(tools, "Open");
		b->setCallback([&] {
			bool result = mCanvas->mesh.loadOBJfile(file_dialog({ {"obj", "obj file"} }, false));
			if (result){
				cout << "File loaded!" << endl;
				mCanvas->updateMesh();
			}
			//cout << "File dialog result: " << file_dialog({ {"obj", "obj file"} }, false) << endl;
		});

		b = new Button(tools, "Save");
		b->setCallback([&] {
			bool result = mCanvas->mesh.saveToOBJfile(file_dialog({ {"obj", "obj file"} }, true));
			if (result){
				cout << "File saved!" << endl;
				auto dlg = new MessageDialog(this, MessageDialog::Type::Information, "Info", "File saved!");
				dlg->setCallback([](int result) { cout << "Dialog result: " << result << endl; });
			}
			// cout << "File dialog result: " << file_dialog({ {"obj", "obj file"} }, true) << endl;
		});

		//todo: This is how to implement a combo box, which is important in A1
		new Label(anotherWindow, "Combo box", "sans-bold");
		ComboBox *combo = new ComboBox(anotherWindow, { "flat shaded", "smooth shaded", "wireframe", "shaded with mesh edges" });
		combo->setCallback([&](int value) {
			mCanvas->setRenderMode(value);
		});

		// translation slider
		new Label(anotherWindow, "Translate", "sans-bold");

		Widget *panelTrans = new Widget(anotherWindow);
		panelTrans->setLayout(new GridLayout(Orientation::Horizontal, 2,
			Alignment::Middle, 0, 0));

		{new Label(panelTrans, "x :", "sans-bold");
		Slider *transSliderX = new Slider(panelTrans);
		transSliderX->setValue(0.5f);
		transSliderX->setFixedWidth(150);
		transSliderX->setCallback([&](float value) {
			mCanvas->setTranslateX((value - 0.5f) * 2);
		}); }

		{new Label(panelTrans, "y :", "sans-bold");
		Slider *transSliderY = new Slider(panelTrans);
		transSliderY->setValue(0.5f);
		transSliderY->setFixedWidth(150);
		transSliderY->setCallback([&](float value) {
			mCanvas->setTranslateY((value - 0.5f) * 2);
		}); }

		new Label(anotherWindow, "Zoom", "sans-bold");

		Widget *panel = new Widget(anotherWindow);
		panel->setLayout(new BoxLayout(Orientation::Horizontal,
			Alignment::Middle, 0, 20));

		//Fancy slider that has a callback function to update another interface element
		Slider *slider = new Slider(panel);
		slider->setValue(0.5f);
		slider->setFixedWidth(100);
		TextBox *textBox = new TextBox(panel);
		textBox->setFixedSize(Vector2i(60, 25));
		textBox->setValue("100");
		textBox->setUnits("%");
		slider->setCallback([textBox](float value) {
			textBox->setValue(std::to_string((int)((value*2) * 100)));
		});
		slider->setFinalCallback([&](float value) {
			mCanvas->setScale(value*2);
		});
		textBox->setFixedSize(Vector2i(60, 25));
		textBox->setFontSize(20);
		textBox->setAlignment(TextBox::Alignment::Right);
		
		// QUIT
		b = new Button(anotherWindow, "Quit");
		b->setCallback([&] {
			shutdown();
		});
		
		//Method to assemble the interface defined before it is called
		performLayout();
	}

	virtual void drawContents() override {
		// ... put your rotation code here if you use dragging the mouse, updating either your model points, the mvp matrix or the V matrix, depending on the approach used
	}

	virtual void draw(NVGcontext *ctx) {
		/* Animate the scrollbar */
		//mProgress->setValue(std::fmod((float)glfwGetTime() / 10, 1.0f));

		/* Draw the user interface */
		Screen::draw(ctx);
	}


private:
	nanogui::ProgressBar *mProgress;
	MyGLCanvas *mCanvas;
	nanogui::Vector3f rotate_start, rotate_end;
	nanogui::Vector3f translate_start, translate_end;
	int flag_r, flag_t;
};

int main(int /* argc */, char ** /* argv */) {
	try {
		nanogui::init();

		/* scoped variables */ {
			nanogui::ref<ExampleApplication> app = new ExampleApplication();
			app->drawAll();
			app->setVisible(true);
			nanogui::mainloop();
		}

		nanogui::shutdown();
	}
	catch (const std::runtime_error &e) {
		std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
	#if defined(_WIN32)
		MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
	#else
		std::cerr << error_msg << endl;
	#endif
		return -1;
	}

	return 0;
}
