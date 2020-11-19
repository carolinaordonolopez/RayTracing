/* sphere.cpp
 *
 * Realidad Virtual y Aumentada.
 *
 * Practice 2.
 * Ray tracing.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */


#include <windows.h>

#include "GL/glew.h"
#include "GL/glut.h"

#include "glm/glm.hpp" // glm::vec3, glm::dot
#include "glm/gtc/matrix_transform.hpp" // glm::translate, glm::rotate, glm::scale

#include "object.h"
#include "limits.h"


/* Constructor */
Sphere::Sphere(const glm::vec3& Center, float Radius, Material *pMat) {
  center = Center;
  radius = Radius;
  pMaterial = pMat;
  pMaterial->IncRefs();
}

/* Computes the nearest intersection point between a ray and the sphere, 
   in the direction of the ray. Returns the t value of that point if it
   is positive, otherwise returns 0. */
float Sphere::NearestInt(const glm::vec3& pos, const glm::vec3& dir)
{
  glm::vec3 dif;
  float A, B, C, aux;
  float t1, t2;

  A = glm::dot(dir, dir);
  if (A == 0.0)
	  return 0;

  dif = pos - center;
  B = 2.0f * glm::dot(dir, dif);
  C = glm::dot(dif, dif) - radius*radius;

  aux = (B * B) - (4 * A * C);

  if (aux < 0.0) return 0;

  if (aux == 0.0) {
    return -B / (2.0f * A);
  }

  aux = (float)glm::sqrt(aux);
  t1 = (-B + aux) / (2.0f * A);
  t2 = (-B - aux) / (2.0f * A);

  if (TMIN < t1)
	  if (TMIN < t2)
		  return (t1 < t2) ? t1 : t2;
	  else
		  return t1;

  if (TMIN < t2)
	  return t2;

  return 0;
}

/* Returns the color of the sphere at the intersection point with the ray */
glm::vec3 Sphere::Shade(ShadingInfo &shadInfo) {

	glm::vec3 color;

	shadInfo.point = shadInfo.rayPos + (shadInfo.t * shadInfo.rayDir);
	shadInfo.normal = glm::normalize(shadInfo.point - center);  // 1-unit-long normal

	/* If the viewer is staring at an interior or back face, we will only illuminate it if
	   the object is transparent, in that case we will have to reverse the normal. That is
	   done in the Shade function of the Material class */
	color = pMaterial->Shade(shadInfo);
	return color;
}

/* Draw the object with OpenGL and GLUT */
void Sphere::Draw() {
	glm::mat4 viewMat, sphereMat, modelviewMat;
	glGetFloatv(GL_MODELVIEW_MATRIX, &viewMat[0][0]);

	sphereMat = glm::translate(sphereMat, glm::vec3(center.x, center.y, center.z));
	modelviewMat = viewMat * sphereMat;
	glLoadMatrixf(&modelviewMat[0][0]);
	glColor3f(pMaterial->Kd.x, pMaterial->Kd.y, pMaterial->Kd.z);
	glutSolidSphere(radius, 20, 20);

	glLoadMatrixf(&viewMat[0][0]);
}
