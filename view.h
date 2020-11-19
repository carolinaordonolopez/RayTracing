/* view.h
 *
 * Realidad Virtual y Aumentada.
 *
 * Practice 2.
 * Ray tracing.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */

#ifndef _VIEW_H_
#define _VIEW_H_

#include "glm/vec3.hpp" // glm::vec3


 /**************/
 /* View class */
 /**************/

class View {

public:

  /* Constructor */
  View();

  /* Functions */

  /* Define the viewpoint copying the style of function gluLookAt of OpenGL */
  void SetLookAt(const glm::vec3& eye, const glm::vec3& center, const glm::vec3& up);
  
  /* Define the perspective projection copying the style of function gluPerspective of OpenGL */
  void SetPerspective(float fovy, float aspect);


  float angle;        // Vertical field of view
  float d;            // Distance of the view plane to the center of projection, vp - cop
  glm::vec3 u, v, n;  // Vectors that define the viewing reference system
  float umin,vmin;    // Minimum coordinates of view window
  float umax, vmax;   // Maximum coordinates of view window
};


#endif  // !defined _VIEW_H_
