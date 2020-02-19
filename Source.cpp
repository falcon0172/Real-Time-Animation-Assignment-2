// Source.cpp : This file contains the 'main' function. Program execution begins and ends there.
//define GLEW_STATIC
#define GLEW_STATIC
#include "pch.h"

#include <stdio.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <time.h>
#include "helpers.h"
#include "bone.h"
#include "movement.h"
#include "ccd.h"
//#define GLEW_STATIC
#include "GLFW/glfw3.h"
#include "Camera.h"

float speed_x = 0;
float speed_y = 0;
int lastTime = 0;
float angle_x = -27.8486;
float angle_y = 1.71429;

float anglex_top = -33.0;
float angley_top = 1.575;

float anglex_left = -28.14;
float angley_left = 1.55;

float anglex_RIGHT = -28.14;
float angley_RIGHT = -
1.55;

float zoom = 31.5f;
float cameraDirectionx = 0.0;
float cameraDirectiony = -2.75;
int gWindowWidth = 1024;
int gWindowHeight = 768;
GLFWwindow* gWindow = NULL;
glm::vec4 gClearColor(0.06f, 0.06f, 0.07f, 1.0f);
bool gWireframe = false;
const char* APP_TITLE = "Inverse Kinematics";

glm::vec3 target;
Movement* animation = new Movement();
bool animation_running = false;
float animation_fill = 0.0f;
bool root_set = false;
unsigned long long effector = 0;

GLfloat pod_color[] = { 0.0, 1.0, 0.0, 1.0 };
GLfloat bone_color[] = { 0.0, 0.0, 1.0, 1.0 };
GLfloat joint_color[] = { 1.0,1.0,0.0, 1.0 };
GLfloat tip_color[] = { 1.0,1.0,0.0, 1.0 };
GLfloat ball_color[] = { 0.5,0.0,0.0, 1.0 };

// Function prototypes



FPSCamera fpsCamera(glm::vec3(0.0f, 3.5f, 10.0f));
const double ZOOM_SENSITIVITY = -3.0;
const float MOVE_SPEED = 5.0; // units per second
const float MOUSE_SENSITIVITY = 0.1f;


Bone* root;
float r_up = 60.0f;

void randomizeTarget() {
	target.x = float(rand() % 10000) / 2000.0;
	target.y = float(rand() % 10000) / 2000.0;
	target.z = float(rand() % 10000) / 2000.0;

	GLfloat light_position[] = { target.x, target.y, target.z, 1.0 };


	glLightfv(GL_LIGHT1, GL_POSITION, light_position);
	ball_color[3] = 0.5;
	glLightfv(GL_LIGHT1, GL_DIFFUSE, ball_color);
	ball_color[3] = 1.0;

	delete animation;
	animation = new Movement();

}

void drawBones(Bone* b) {
	glPushMatrix();
	GLfloat s[] = { 0 };

	glm::mat4 previous = b->M;

	if (b->parent != NULL) {
		b->M = glm::translate(b->M, glm::vec3(0.0f, 0.0f, b->parent->length));

		b->M = glm::rotate(b->M, b->rotation.x, glm::vec3(b->M*glm::vec4(1.0f, 0.0f, 0.0f, 0.0f)));
		b->M = glm::rotate(b->M, b->rotation.y, glm::vec3(b->M*glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)));
		b->M = glm::rotate(b->M, b->rotation.z, glm::vec3(b->M*glm::vec4(0.0f, 0.0f, 1.0f, 0.0f)));

		b->M = b->parent->M * b->M;
	}
	else {
		b->M = glm::rotate(b->M, b->rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
		b->M = glm::rotate(b->M, b->rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
		b->M = glm::rotate(b->M, b->rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
	}

	glLoadMatrixf(glm::value_ptr(b->M));

	if (b->parent != NULL) {
		s[0] = 250;
		glMaterialfv(GL_FRONT, GL_SPECULAR, joint_color);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, joint_color);
		glMaterialfv(GL_FRONT, GL_SHININESS, s);
		glutSolidSphere(0.2f, 32, 32);
	}

	glMaterialfv(GL_FRONT, GL_SPECULAR, bone_color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, bone_color);
	s[0] = 11;
	glMaterialfv(GL_FRONT, GL_SHININESS, s);

	GLUquadricObj* q = gluNewQuadric();
	if (b->bones.empty())
		//gluCylinder(q, 0.1f, 0.1f, b->length - 0.5f, 32, 32);
		gluCylinder(q, 0.2f, 0.0f, b->length - 0.5f, 32, 32);
	else
	//	gluCylinder(q, 0.1f, 0.1f, b->length, 32, 32);
		gluCylinder(q, 0.2f, 0.0f, b->length, 32, 32);
	gluDeleteQuadric(q);

	glPopMatrix();


	if (b->bones.empty()) {
		glPushMatrix();

		glLoadMatrixf(glm::value_ptr(glm::translate(b->M, glm::vec3(0.0f, 0.0f, b->length - 0.5f))));
		glMaterialfv(GL_FRONT, GL_SPECULAR, tip_color);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, tip_color);
		s[0] = 250;
		glMaterialfv(GL_FRONT, GL_SHININESS, s);

		gluCylinder(q, 0.2f, 0.0f, 0.5f, 32, 32);

		glPopMatrix();
	}

	for (std::vector<Bone*>::iterator it = b->bones.begin(); it != b->bones.end(); it++) {
		drawBones(*it);
	}

	b->M = previous;
}

void displayKeyFrame(void) {
	glClearColor(0.6, 0.6, 0.7, 1);	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, 512, 512);

	glm::vec3 center = glm::vec3(0.0f + cameraDirectionx, 2.0f + cameraDirectiony, 0.0);
	glm::vec3 eye = glm::vec3(0.0f, 1.0f, - zoom);
	glm::vec3 viewpoint = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::mat4 V = glm::lookAt(eye, center, viewpoint);
	glm::mat4 temp = V;
	V = glm::rotate(V, angle_x, glm::vec3(1.0f, 0.0f, 0.0f));
	V = glm::rotate(V, angle_y, glm::vec3(0.0f, 1.0f, 0.0f));

	glm::mat4 P = glm::perspective(50.0f, 1.0f, 1.0f, 50.0f);

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(glm::value_ptr(P));
	glMatrixMode(GL_MODELVIEW);


	glm::mat4 M = glm::mat4(1.0f);
	glLoadMatrixf(glm::value_ptr(V*M));

	GLfloat s[] = { 128 };
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_SPECULAR, ball_color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, ball_color);
	glMaterialfv(GL_FRONT, GL_SHININESS, s);
	ball_color[3] = 0.5;
	glMaterialfv(GL_FRONT, GL_EMISSION, ball_color);
	ball_color[3] = 1.0;
	glLoadMatrixf(glm::value_ptr(glm::translate(M*V, target)));
	glutSolidSphere(0.3f, 32, 32);
	glPopMatrix();

	GLfloat t[] = { 0.0,0.0, 0.0,1.0 };
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_SPECULAR, pod_color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, pod_color);
	s[0] = 0;
	glMaterialfv(GL_FRONT, GL_SHININESS, s);
	glMaterialfv(GL_FRONT, GL_EMISSION, t);
	glScalef(5.9f, 0.1f, 5.9f);
	glutSolidCube(1.0f);
	glPopMatrix();

	root->M = glm::rotate(M*V, -90.0f, glm::vec3(1.0f, 0.0f, 0.0f));
	root->setRotate(root->rotation.x, root->rotation.y, root->rotation.z);


	drawBones(root);

//2
	
	glViewport(512, 512, 512, 512);
	glm::mat4 V2 = glm::lookAt(eye, center, viewpoint);

	V2 = glm::rotate(V2, anglex_top, glm::vec3(1.0f, 0.0f, 0.0f));
	V2 = glm::rotate(V2, angley_top, glm::vec3(0.0f, 1.0f, 0.0f));
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(glm::value_ptr(P));
	glMatrixMode(GL_MODELVIEW);


//	glm::mat4 M = glm::mat4(1.0f);
	glLoadMatrixf(glm::value_ptr(V2*M));

//	GLfloat s[] = { 128 };
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_SPECULAR, ball_color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, ball_color);
	glMaterialfv(GL_FRONT, GL_SHININESS, s);
	ball_color[3] = 0.5;
	glMaterialfv(GL_FRONT, GL_EMISSION, ball_color);
	ball_color[3] = 1.0;
	glLoadMatrixf(glm::value_ptr(glm::translate(M*V2, target)));
	glutSolidSphere(0.3f, 32, 32);
	glPopMatrix();

//	GLfloat t[] = { 0.0,0.0, 0.0,1.0 };
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_SPECULAR, pod_color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, pod_color);
	s[0] = 0;
	glMaterialfv(GL_FRONT, GL_SHININESS, s);
	glMaterialfv(GL_FRONT, GL_EMISSION, t);
	glScalef(5.9f, 0.1f, 5.9f);
	glutSolidCube(1.0f);
	glPopMatrix();

	root->M = glm::rotate(M*V2, -90.0f, glm::vec3(1.0f, 0.0f, 0.0f));
	root->setRotate(root->rotation.x, root->rotation.y, root->rotation.z);
	drawBones(root);

//3


	glViewport(0, 512, 512, 512);
	glm::mat4 V3 = temp;
	V3 = glm::rotate(V3, anglex_left, glm::vec3(1.0f, 0.0f, 0.0f));
	V3 = glm::rotate(V3, angley_left, glm::vec3(0.0f, 1.0f, 0.0f));
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(glm::value_ptr(P));
	glMatrixMode(GL_MODELVIEW);


	//	glm::mat4 M = glm::mat4(1.0f);
	glLoadMatrixf(glm::value_ptr(V3*M));

	//	GLfloat s[] = { 128 };
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_SPECULAR, ball_color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, ball_color);
	glMaterialfv(GL_FRONT, GL_SHININESS, s);
	ball_color[3] = 0.5;
	glMaterialfv(GL_FRONT, GL_EMISSION, ball_color);
	ball_color[3] = 1.0;
	glLoadMatrixf(glm::value_ptr(glm::translate(M*V3, target)));
	glutSolidSphere(0.3f, 32, 32);
	glPopMatrix();

	//	GLfloat t[] = { 0.0,0.0, 0.0,1.0 };
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_SPECULAR, pod_color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, pod_color);
	s[0] = 0;
	glMaterialfv(GL_FRONT, GL_SHININESS, s);
	glMaterialfv(GL_FRONT, GL_EMISSION, t);
	glScalef(5.9f, 0.1f, 5.9f);
	glutSolidCube(1.0f);
	glPopMatrix();

	root->M = glm::rotate(M*V3, -90.0f, glm::vec3(1.0f, 0.0f, 0.0f));
	root->setRotate(root->rotation.x, root->rotation.y, root->rotation.z);
	drawBones(root);

//4
	glViewport(512, 0, 512, 512);
	glm::mat4 V4 = temp;
	V4 = glm::rotate(V4, anglex_RIGHT, glm::vec3(1.0f, 0.0f, 0.0f));
	V4 = glm::rotate(V4, angley_RIGHT, glm::vec3(0.0f, 1.0f, 0.0f));
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(glm::value_ptr(P));
	glMatrixMode(GL_MODELVIEW);


	//	glm::mat4 M = glm::mat4(1.0f);
	glLoadMatrixf(glm::value_ptr(V4*M));

	//	GLfloat s[] = { 128 };
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_SPECULAR, ball_color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, ball_color);
	glMaterialfv(GL_FRONT, GL_SHININESS, s);
	ball_color[3] = 0.5;
	glMaterialfv(GL_FRONT, GL_EMISSION, ball_color);
	ball_color[3] = 1.0;
	glLoadMatrixf(glm::value_ptr(glm::translate(M*V4, target)));
	glutSolidSphere(0.3f, 32, 32);
	glPopMatrix();

	//	GLfloat t[] = { 0.0,0.0, 0.0,1.0 };
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_SPECULAR, pod_color);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, pod_color);
	s[0] = 0;
	glMaterialfv(GL_FRONT, GL_SHININESS, s);
	glMaterialfv(GL_FRONT, GL_EMISSION, t);
	glScalef(5.9f, 0.1f, 5.9f);
	glutSolidCube(1.0f);
	glPopMatrix();

	root->M = glm::rotate(M*V4, -90.0f, glm::vec3(1.0f, 0.0f, 0.0f));
	root->setRotate(root->rotation.x, root->rotation.y, root->rotation.z);
	drawBones(root);


	glutSwapBuffers();
}


void nextKeyFrame(void) {
	int actTime = glutGet(GLUT_ELAPSED_TIME);
	float interval = actTime - lastTime;
	lastTime = actTime;
	angle_x += speed_x * interval / 700.0;
	angle_y += speed_y * interval / 700.0;

	if (animation_running) {
		animation_fill += interval / 1000.0;

		if (!animation->frame(interval / 1000.0, root)) {
			animation_fill = 0.0;
			if (!animation->next()) {
				animation_running = false;
			}
		}
	}

	if (angle_x > 360) angle_x -= 360;
	if (angle_x > 360) angle_x += 360;
	if (angle_y > 360) angle_y -= 360;
	if (angle_y > 360) angle_y += 360;

	glutPostRedisplay();
}

void specKeyDown(int c, int x, int y) {
	if (glutGetModifiers() == GLUT_ACTIVE_SHIFT) {
		switch (c) {
		case GLUT_KEY_LEFT:
			cameraDirectionx -= 0.25;
			std::cout <<"Direction X:" <<cameraDirectionx << std::endl;
			std::cout << "Direction Y:"<< cameraDirectiony << std::endl;
			break;
		case GLUT_KEY_RIGHT:
			cameraDirectionx += 0.25;
			std::cout << "Direction X:" << cameraDirectionx << std::endl;
			std::cout << "Direction Y:" << cameraDirectiony << std::endl;
			break;
		case GLUT_KEY_UP:
			cameraDirectiony -= 0.25;
			std::cout << "Direction X:" << cameraDirectionx << std::endl;
			std::cout << "Direction y:" << cameraDirectiony << std::endl;
			break;
		case GLUT_KEY_DOWN:
			cameraDirectiony += 0.25;
			std::cout << "Direction X:" << cameraDirectionx << std::endl;
			std::cout << "Direction y:" << cameraDirectiony << std::endl;
			break;
		}
	}
	else {
		switch (c) {
		case GLUT_KEY_LEFT:
			speed_y = 6;
			std::cout << "Angle X:" << angle_x << std::endl;
			std::cout << "Angle y:" << angle_y << std::endl;
			break;
		case GLUT_KEY_RIGHT:
			speed_y = -6;
			std::cout << "Angle X:" << angle_x << std::endl;
			std::cout << "Angle y:" << angle_y << std::endl;
			break;
		case GLUT_KEY_UP:
			speed_x = 6;
			std::cout << "Angle X:" << angle_x << std::endl;
			std::cout << "Angle y:" << angle_y << std::endl;
			break;
		case GLUT_KEY_DOWN:
			speed_x = -6;
			std::cout << "Angle X:" << angle_x << std::endl;
			std::cout << "Angle y:" << angle_y << std::endl;
			break;
		}
	}
}

void specKeyUp(int c, int x, int y) {
	switch (c) {
	case GLUT_KEY_LEFT:
		speed_y = 0;
		break;
	case GLUT_KEY_RIGHT:
		speed_y = -0;
		break;
	case GLUT_KEY_UP:
		speed_x = 0;
		break;
	case GLUT_KEY_DOWN:
		speed_x = -0;

		break;
	}
}

void keyDown(unsigned char c, int x, int y) {
	try {
		switch (c) {
		case '-':
			zoom += 0.5f;
			std::cout <<"Zoom-"<<zoom << std::endl;
			break;
		case '=':
		case '+':
			zoom -= 0.5f;
			std::cout << "Zoom+" << zoom << std::endl;
			break;

		case 'z':
			root->bone(1111)->rotate(-0.5f, 0.0f, 0.0f);
			break;

		case 'u':
			root->bone(111)->rotate(-0.5f, 0.0f, 0.0f);
			break;
		case 'j':
			root->bone(111)->rotate(0.5f, 0.0f, 0.0f);
			break;
		case 'i':
			root->bone(111)->rotate(0.0f, -0.5f, 0.0f);
			break;
		case 'k':
			root->bone(111)->rotate(0.0f, +0.5f, 0.0f);
			break;
		case 'o':
			root->bone(111)->rotate(0.0f, 0.0f, -0.5f);
			break;
		case 'l':
			root->bone(111)->rotate(0.0f, 0.0f, +0.5f);
			break;


		case 'r':
			root->bone(11)->rotate(-0.5f, 0.0f, 0.0f);
			break;
		case 'f':
			root->bone(11)->rotate(+0.5f, 0.0f, 0.0f);
			break;
		case 't':
			root->bone(11)->rotate(0.0f, -0.5f, 0.0f);
			break;
		case 'g':
			root->bone(11)->rotate(0.0f, +0.5f, 0.0f);
			break;
		case 'y':
			root->bone(11)->rotate(0.0f, 0.0f, -0.5f);
			break;
		case 'h':
			root->bone(11)->rotate(0.0f, 0.0f, +0.5f);
			break;

		case 'q':
			root->bone(1)->rotate(-0.5f, 0.0f, 0.0f);
			break;
		case 'a':
			root->bone(1)->rotate(+0.5f, 0.0f, 0.0f);
			break;
		case 'w':
			root->bone(1)->rotate(0.0f, -0.5f, 0.0f);
			break;
		case 's':
			root->bone(1)->rotate(0.0f, +0.5f, 0.0f);
			break;
		case 'e':
			root->bone(1)->rotate(0.0f, 0.0f, -0.5f);
			break;
		case 'd':
			root->bone(1)->rotate(0.0f, 0.0f, +0.5f);
			break;
		case 'Q':
			target.z -= 2.0f;
			break;
		case 'E':
			target.z += 2.0f;
			break;
		case 'W':
			target.y += 2.0;
			break;
		case 'S':
			target.y -= 2;
			break;
		case 'A':
			target.x -= 2;
			break;
		case 'D':
			target.x += 2;
			break;
		case ' ':
			randomizeTarget();
			break;

		case 'm':
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			break;

		case 'n':
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			break;

		}
	}
	catch (ConstraintException* e) {
		printf("Cannot move further!\n");
	}
}

void displayVec3(glm::vec3 vec) {
	printf("x: %f, y: %f, z: %f\n", vec.x, vec.y, vec.z);
}

void displayVec3(glm::vec4 vec) {
	printf("x: %f, y: %f, z: %f\n", vec.x, vec.y, vec.z);
}

void keyUp(unsigned char c, int x, int y) {
	switch (c) {
	case '.':
		ccd::findNewAngles(root->bone(effector), target);
		break;
	case ',':
		Bone *b = new Bone(*root);

		ccd::findNewAngles(b->bone(effector), target);

		delete animation;
		animation = new Movement();

		animation->set(root)->keyframe();

		for (int z = effector; z != 0; z /= 10) {
			animation->set(b->bone(z), b->bone(z)->rotation)->keyframe();
		}

		delete b;

		animation->start()->next();
		animation_running = true;

		break;
	}
}


int main(int argc, char* argv[]) {

/*	if (!initOpenGL())
	{
		// An error occured
		std::cerr << "GLFW initialization failed" << std::endl;
		return -1;
	} */

	srand(time(0));
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1024, 1024);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Inverse Kinematics");

	glutDisplayFunc(displayKeyFrame);
	glutIdleFunc(nextKeyFrame);

	glutSpecialFunc(specKeyDown);
	glutSpecialUpFunc(specKeyUp);
	glutKeyboardFunc(keyDown);
	glutKeyboardUpFunc(keyUp);

	glShadeModel(GL_SMOOTH);

	GLfloat light_ambient[] = { 0.1, 0.1, 0.1, 1.0 };
	GLfloat light_position[] = { 1.0, 0.0, 0.0, 1.0 };
	GLfloat light_diffuse[] = { 0.5, 0.5, 0.5, 1.0 };
	GLfloat light_specular[] = { 0.5, 0.5, 0.5, 1.0 };



	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

	randomizeTarget();

	std::cout << cameraDirectionx << std::endl;
	std::cout << cameraDirectiony << std::endl;

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

	glEnable(GL_DEPTH_TEST);

	root = new Bone(0.0f);

	root->add(new Bone(1))
		->add(new Bone(1))->rotate(0, 20, 0)
		->add(new Bone(1))->rotate(0, 30, 0)
		->add(new Bone(1))->rotate(0, 40, 0)
		->add(new Bone(1))->rotate(0, 50, 0)
		->add(new Bone(1))->rotate(0, 60, 0)
		->add(new Bone(1))->rotate(0, 70, 0)
		->add(new Bone(1))->rotate(0, 80, 0);
	//	->add(new Bone(1))->rotate(0, 80, 0);

//	glfwSetInputMode(gWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
//	glfwSetCursorPos(gWindow, gWindowWidth / 2.0, gWindowHeight / 2.0);


	effector = 11111111;

	glutMainLoop();

	delete root;
	delete animation;

	return 0;
}
/*
bool initOpenGL()
{

	glewExperimental = TRUE;
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		// Problem: glewInit failed, something is seriously wrong.
		std::cout << "glewInit failed: " << glewGetErrorString(err) << std::endl;
		exit(1);
	}
	// Intialize GLFW 
	// GLFW is configured.  Must be called before calling any GLFW functions
	if (!glfwInit())
	{
		// An error occured
		std::cerr << "GLFW initialization failed" << std::endl;
		return false;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 1);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);	// forward compatible with newer versions of OpenGL as they become available but not backward compatible (it will not run on devices that do not support OpenGL 3.3


	// Create an OpenGL 3.3 core, forward compatible context window
	gWindow = glfwCreateWindow(gWindowWidth, gWindowHeight, APP_TITLE, NULL, NULL);
	if (gWindow == NULL)
	{
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return false;
	}

	// Make the window's context the current one
	glfwMakeContextCurrent(gWindow);

	// Initialize GLEW
	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK)
	{
		std::cerr << "Failed to initialize GLEW" << std::endl;
		return false;
	}

	// Set the required callback functions
	glfwSetKeyCallback(gWindow, glfw_onKey);
	glfwSetFramebufferSizeCallback(gWindow, glfw_onFramebufferSize);
	glfwSetScrollCallback(gWindow, glfw_onMouseScroll);

	// Hides and grabs cursor, unlimited movement
	glfwSetInputMode(gWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetCursorPos(gWindow, gWindowWidth / 2.0, gWindowHeight / 2.0);

	glClearColor(gClearColor.r, gClearColor.g, gClearColor.b, gClearColor.a);

	// Define the viewport dimensions
	glViewport(0, 0, gWindowWidth, gWindowHeight);
	glEnable(GL_DEPTH_TEST);

	return true;
}

void glfw_onKey(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	if (key == GLFW_KEY_F1 && action == GLFW_PRESS)
	{
		gWireframe = !gWireframe;
		if (gWireframe)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

}

void update(double elapsedTime)
{
	// Camera orientation
	double mouseX, mouseY;

	// Get the current mouse cursor position delta
	glfwGetCursorPos(gWindow, &mouseX, &mouseY);

	// Rotate the camera the difference in mouse distance from the center screen.  Multiply this delta by a speed scaler
	fpsCamera.rotate((float)(gWindowWidth / 2.0 - mouseX) * MOUSE_SENSITIVITY, (float)(gWindowHeight / 2.0 - mouseY) * MOUSE_SENSITIVITY);

	// Clamp mouse cursor to center of screen
	glfwSetCursorPos(gWindow, gWindowWidth / 2.0, gWindowHeight / 2.0);

	// Camera FPS movement

	// Forward/backward
	if (glfwGetKey(gWindow, GLFW_KEY_W) == GLFW_PRESS)
		fpsCamera.move(MOVE_SPEED * (float)elapsedTime * fpsCamera.getLook());
	else if (glfwGetKey(gWindow, GLFW_KEY_S) == GLFW_PRESS)
		fpsCamera.move(MOVE_SPEED * (float)elapsedTime * -fpsCamera.getLook());

	// Strafe left/right
	if (glfwGetKey(gWindow, GLFW_KEY_A) == GLFW_PRESS)
		fpsCamera.move(MOVE_SPEED * (float)elapsedTime * -fpsCamera.getRight());
	else if (glfwGetKey(gWindow, GLFW_KEY_D) == GLFW_PRESS)
		fpsCamera.move(MOVE_SPEED * (float)elapsedTime * fpsCamera.getRight());

	// Up/down
	if (glfwGetKey(gWindow, GLFW_KEY_Z) == GLFW_PRESS)
		fpsCamera.move(MOVE_SPEED * (float)elapsedTime * glm::vec3(0.0f, 1.0f, 0.0f));
	else if (glfwGetKey(gWindow, GLFW_KEY_X) == GLFW_PRESS)
		fpsCamera.move(MOVE_SPEED * (float)elapsedTime * -glm::vec3(0.0f, 1.0f, 0.0f));
}

//-----------------------------------------------------------------------------
// Code computes the average frames per second, and also the average time it takes
// to render one frame.  These stats are appended to the window caption bar.
//-----------------------------------------------------------------------------
void showFPS(GLFWwindow* window)
{
	static double previousSeconds = 0.0;
	static int frameCount = 0;
	double elapsedSeconds;
	double currentSeconds = glfwGetTime(); // returns number of seconds since GLFW started, as double float

	elapsedSeconds = currentSeconds - previousSeconds;

	// Limit text updates to 4 times per second
	if (elapsedSeconds > 0.25)
	{
		previousSeconds = currentSeconds;
		double fps = (double)frameCount / elapsedSeconds;
		double msPerFrame = 1000.0 / fps;

		// The C++ way of setting the window title
		std::ostringstream outs;
		outs.precision(3);	// decimal places
		outs << std::fixed
			<< APP_TITLE << "    "
			<< "FPS: " << fps << "    "
			<< "Frame Time: " << msPerFrame << " (ms)";
		glfwSetWindowTitle(window, outs.str().c_str());

		// Reset for next average.
		frameCount = 0;
	}

	frameCount++;
}
//-----------------------------------------------------------------------------
// Is called when the window is resized
//-----------------------------------------------------------------------------
void glfw_onFramebufferSize(GLFWwindow* window, int width, int height)
{
	gWindowWidth = width;
	gWindowHeight = height;
	glViewport(0, 0, gWindowWidth, gWindowHeight);
}

//-----------------------------------------------------------------------------
// Called by GLFW when the mouse wheel is rotated
//-----------------------------------------------------------------------------
void glfw_onMouseScroll(GLFWwindow* window, double deltaX, double deltaY)
{
	double fov = fpsCamera.getFOV() + deltaY * ZOOM_SENSITIVITY;

	fov = glm::clamp(fov, 1.0, 120.0);

	fpsCamera.setFOV((float)fov);
}*/