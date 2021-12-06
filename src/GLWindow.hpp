#pragma once
#define GLFW_INCLUDE_GLCOREARB
#include <GLFW/glfw3.h>
#include <OpenGL/gl.h>
#include <iostream>
#include <Eigen/Dense>

void  printErrorCallback(const int err, const char * desc){
    std::cerr << "Error: " << err << '\t' << desc << std::endl;
    std::exit(-1);
}

// void GLAPIENTRY
// MessageCallback( GLenum source,
//                  GLenum type,
//                  GLuint id,
//                  GLenum severity,
//                  GLsizei length,
//                  const GLchar* message,
//                  const void* userParam )
// {
//   fprintf( stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
//            ( type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : "" ),
//             type, severity, message );
// }


using Color_t = Eigen::Vector3f;

Color_t hsv2rgb(const Color_t & hsv ){

    const float_t hh = hsv.x();
    
    const long i = hh * 6;
    const float_t ff = hh - i;
    const float_t p = hsv.z() * (1.0 - hsv.y());
    const float_t q = hsv.z() * (1.0 - (hsv.y() * ff));
    const float_t t = hsv.z() * (1.0 - (hsv.y() * (1.0 - ff)));

    switch(i) {
        case 0:
            return {hsv.z(), t, p};


        case 1:
            return {q, hsv.z(), p};


        case 2:
            return {p, hsv.z(), t};


        case 3:
            return {p, q, hsv.z()};


        case 4:
            return {t, p, hsv.z()};


        case 5:
        
            return {hsv.z(), p, q};

        default:
            assert(false);

    }
         
}




GLFWwindow * CreateOpenWindow(const unsigned int w, const unsigned int h){

    glfwSetErrorCallback(printErrorCallback);

    // glEnable              ( GL_DEBUG_OUTPUT );
    // glDebugMessageCallback( MessageCallback, 0 );

    GLFWwindow * window = nullptr;

    /* Initialize the library */
    if ( glfwInit() ){


        // #ifdef __APPLE__
        //     /* We need to explicitly ask for a 3.2 context on OS X */
        //     glfwWindowHint (GLFW_CONTEXT_VERSION_MAJOR, 3);
        //     glfwWindowHint (GLFW_CONTEXT_VERSION_MINOR, 2);
        //     glfwWindowHint (GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        //     glfwWindowHint (GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        // #endif

        /* Create a windowed mode window and its OpenGL context */
        window = glfwCreateWindow( w, h, "Test", NULL, NULL );

        glfwMakeContextCurrent(window);

        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_DEPTH_TEST);

        // UpdateProjectionMatrix();
    }

    return window;
}

void DrawFilledCircle(GLfloat x, GLfloat y, GLfloat radius){

	constexpr size_t triangleAmount = 20; //# of triangles used to draw circle
	
	// GLfloat twicePi = 2.0f * M_PI;
	
	glBegin(GL_TRIANGLE_FAN);

		glVertex2f(x, y); // center of circle

		for(size_t i = 0; i <= triangleAmount; i++) { 

      const GLfloat alpha = i * M_PI * 2 / triangleAmount;

			glVertex2f(
		              x + (radius * std::cos(alpha)), 
			            y + (radius * std::sin(alpha))
			); 
		}

	glEnd();
}


char const* glErrorToString(GLenum const err) noexcept
{
  switch (err)
  {
    // opengl 2 errors (8)
    case GL_NO_ERROR:
      return "GL_NO_ERROR";

    case GL_INVALID_ENUM:
      return "GL_INVALID_ENUM";

    case GL_INVALID_VALUE:
      return "GL_INVALID_VALUE";

    case GL_INVALID_OPERATION:
      return "GL_INVALID_OPERATION";

    case GL_STACK_OVERFLOW:
      return "GL_STACK_OVERFLOW";

    case GL_STACK_UNDERFLOW:
      return "GL_STACK_UNDERFLOW";

    case GL_OUT_OF_MEMORY:
      return "GL_OUT_OF_MEMORY";

    case GL_TABLE_TOO_LARGE:
      return "GL_TABLE_TOO_LARGE";

    // opengl 3 errors (1)
    case GL_INVALID_FRAMEBUFFER_OPERATION:
      return "GL_INVALID_FRAMEBUFFER_OPERATION";

    // gles 2, 3 and gl 4 error are handled by the switch above
    default:
      return "unknown error";
  }
}