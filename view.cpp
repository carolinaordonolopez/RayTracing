/* view.cpp
 *
 * Realidad Virtual y Aumentada.
 *
 * Practice 1.
 * Ray casting.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */

#include "glm/glm.hpp" // glm::vec3, glm::normalize, glm::dot, glm::cross

#include "view.h"


/* Constructor */
View::View() {
  angle = 40.0;
  d = -1;
  u = glm::vec3(1,0,0);
  v = glm::vec3(0,1,0);
  n = glm::vec3(0,0,1);
  umin = -0.5;
  vmin = -0.5;
  umax = 0.5;
  vmax = 0.5;
}

/* Define the viewpoint copying the style of function gluLookAt of OpenGL */
void View::SetLookAt(const glm::vec3& eye, const glm::vec3& center, const glm::vec3& up) {
  glm::vec3 vup;

  vup = glm::normalize(up);

  n = glm::normalize(eye - center);
  u = glm::cross(vup, n);
  v = glm::cross(n, u);
}

/* Define the perspective projection copying the style of function gluPerspective of OpenGL */
void View::SetPerspective(float fovy, float aspect) {
  
  angle = fovy;

  d = -0.5f / (float)tan(glm::radians(fovy)/2.0f);
  
  umax = aspect/2.0f;
  vmax = 0.5;

  umin = -umax;
  vmin = -0.5;
}
