#include "stdafx.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <iostream>
#include <fstream>

#include <stdio.h>
#include "RigidBody.h"
#include "Game.h"

GT1::Game game;

void resize(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
	game.RecalculateProjection(width, height);
}

// Called for keyboard events
void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods) {
	game.Keyboard(key, action);
}

void mouse(GLFWwindow* window, int button, int action, int mods) {
	game.MouseButton(button, action);
}

// Screen coordinates
void mouseMoved(GLFWwindow* window, double xpos, double ypos) {
	game.MouseMoved(xpos, ypos);
}

void InitGL() {
	glEnable(GL_BLEND);
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0, 0, 0.2f, 0);                   // background color
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDepthFunc(GL_LEQUAL);

	glViewport(0, 0, SCRWIDTH, SCRHEIGHT); // breaks on window resize
}

GLFWwindow* InitGLFW() {
	if (!glfwInit()) exit(EXIT_FAILURE);
	GLFWwindow* window = glfwCreateWindow(SCRWIDTH, SCRHEIGHT, "Physics Test", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, keyboard);
	glfwSetMouseButtonCallback(window, mouse);
	glfwSetWindowSizeCallback(window, resize);
	glfwSetCursorPosCallback(window, mouseMoved);

	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

	return window;
}

int _tmain(int argc, _TCHAR* argv[])
{
	GLFWwindow* window = InitGLFW();
	if(glewInit() != GLEW_OK) {
		glfwTerminate();
		exit(-1);
	}

	InitGL();
	game.Init();
	double lastTime = glfwGetTime();
	float accumulator = 0.0f;

	while (!glfwWindowShouldClose(window))
	{	
		double now = glfwGetTime();
		accumulator += ((float) now - (float) lastTime);
		// 100 Hertz
		while(accumulator > 0.01f) {
			accumulator -= 0.01f;
			game.Tick(0.01f);
			glfwSwapBuffers(window);
			glfwPollEvents();
			if(glfwWindowShouldClose(window)) break;
		}
		lastTime = now;
	}
	game.Cleanup();

	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}
