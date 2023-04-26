#ifndef SK_DEPTH_VIEWER_H
#define SK_DEPTH_VIEWER_H

#include <GLFW/glfw3.h>

#include <GL/gl.h>
#include <iostream>
#include <assert.h>
#include <Eigen/Dense>
#include <map>
#include <mutex>

#include <k4a/k4a.hpp>

class SKDepthViewer{
private:
public: 
    SKDepthViewer(GLFWwindow *window);
};

#endif