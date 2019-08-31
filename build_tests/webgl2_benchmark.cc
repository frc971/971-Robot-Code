#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <cmath>
#include <emscripten/emscripten.h>
#include <emscripten/html5.h>
#include <iostream>
#include <unistd.h>
#include <GLES3/gl3.h>

namespace {
constexpr int kNPoints = 10 * 1000 * 1000;
}  // namespace

// Shader and program construction taken from examples at
// https://github.com/emscripten-core/emscripten/blob/incoming/tests/webgl2_draw_packed_triangle.c
GLuint compile_shader(GLenum shaderType, const char *src) {
  GLuint shader = glCreateShader(shaderType);
  glShaderSource(shader, 1, &src, NULL);
  glCompileShader(shader);

  GLint isCompiled = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &isCompiled);
  if (!isCompiled) {
    GLint maxLength = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &maxLength);
    char *buf = (char *)malloc(maxLength + 1);
    glGetShaderInfoLog(shader, maxLength, &maxLength, buf);
    printf("%s\n", buf);
    free(buf);
    return 0;
  }

  return shader;
}

GLuint create_program(GLuint vertexShader, GLuint fragmentShader) {
   GLuint program = glCreateProgram();
   glAttachShader(program, vertexShader);
   glAttachShader(program, fragmentShader);
   glBindAttribLocation(program, 0, "apos");
   glBindAttribLocation(program, 1, "acolor");
   glLinkProgram(program);
   return program;
}

struct Vector {
  int x;
  int y;
  int z;
  int w;
};

// Packs a vector for use with GL_INT_2_10_10_10_REV.
uint32_t PackVector(const Vector &vec) {
  uint32_t retval = 0;
  retval = vec.w;
  retval <<= 10;
  retval |= vec.z & 0x3FF;
  retval <<= 10;
  retval |= vec.y & 0x3FF;
  retval <<= 10;
  retval |= vec.x & 0x3FF;
  return retval;
}

struct AnimationState {
  // The time, in seconds, at which the last animation frame occurred.
  double last_animation_time = 0.0;
  // The location for the "scale" uniform to modify on each animation call.
  GLint scale_uniform_location;
};

// This function modifies the "scale" uniform to vary from 0.5->1.0->0.5
// in a cycle, and redraws the points on each iteration.
int Redraw(double time, void *data) {
  AnimationState *state = reinterpret_cast<AnimationState*>(data);
  time /= 1000.0;
  const double difftime = time - state->last_animation_time;
  const double wrap_time = std::fmod(time, 1.0);
  const double offset = wrap_time > 0.5 ? 1.0 - wrap_time : wrap_time;
  glUniform1f(state->scale_uniform_location, std::min(0.5 + offset, 100.0));
  std::cout << 1.0 / difftime  << "fps\n";
  glDrawArrays(GL_POINTS, 0, kNPoints);
  state->last_animation_time = time;
  assert(glGetError() == GL_NO_ERROR && "glDrawArray failed");
  return 1;
}

int main() {
  EmscriptenWebGLContextAttributes attr;
  emscripten_webgl_init_context_attributes(&attr);
  attr.majorVersion = 2;
  EMSCRIPTEN_WEBGL_CONTEXT_HANDLE ctx = emscripten_webgl_create_context("#canvas", &attr);
  assert(ctx && "Failed to create WebGL2 context");
  emscripten_webgl_make_context_current(ctx);

  static const char vertex_shader[] =
    "#version 100\n"
    "attribute vec4 apos;"
    "attribute vec4 acolor;"
    "varying vec4 color;"
    "uniform float scale;"
    "void main() {"
      "color = acolor;"
      "gl_Position = apos;"
      "gl_Position.x = apos.x * scale;"
      "gl_Position.y = apos.y * 0.5 / scale;"
      "gl_PointSize = 1.0;"
    "}";
  GLuint vs = compile_shader(GL_VERTEX_SHADER, vertex_shader);

  static const char fragment_shader[] =
    "#version 100\n"
    "precision lowp float;"
    "varying vec4 color;"
    "void main() {"
      "gl_FragColor = color;"
    "}";
  GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fragment_shader);

  GLuint program = create_program(vs, fs);
  glUseProgram(program);

  // Go through and generate randomly located and colored points.
  static uint32_t pos_and_color[kNPoints * 2];
  for (int ii = 0; ii < kNPoints; ++ii) {
    uint16_t w = 1;
    uint16_t z = 0;
    uint16_t y = rand() % 1024;
    uint16_t x = rand() % 1024;
    uint32_t wzyx = PackVector({x, y, z, w});
    uint16_t a = 3;
    uint16_t b = rand() % 1024;
    uint16_t g = rand() % 1024;
    uint16_t r = rand() % 1024;
    uint32_t argb = PackVector({r, g, b, a});
    pos_and_color[2 * ii] = wzyx;
    pos_and_color[2 * ii + 1] = argb;
  }

  GLuint vbo;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(pos_and_color), pos_and_color, GL_STATIC_DRAW);
  glVertexAttribPointer(0, 4, GL_INT_2_10_10_10_REV, GL_TRUE, 8, 0);
  assert(glGetError() == GL_NO_ERROR && "glVertexAttribPointer with GL_INT_2_10_10_10_REV failed");
  glVertexAttribPointer(1, 4, GL_UNSIGNED_INT_2_10_10_10_REV, GL_TRUE, 8, (void*)4);
  assert(glGetError() == GL_NO_ERROR && "glVertexAttribPointer with GL_UNSIGNED_INT_2_10_10_10_REV failed");

  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  // Note that the animation_state must last until Redraw stops being called,
  // which we cannot provide any bound on. As such, we don't currently destroy
  // the memory until the webpage is closed.
  AnimationState *animation_state = new AnimationState();
  animation_state->scale_uniform_location =
      glGetUniformLocation(program, "scale");
  emscripten_request_animation_frame_loop(&Redraw, animation_state);
}
