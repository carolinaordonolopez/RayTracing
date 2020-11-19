/* material.h
*
* Realidad Virtual y Aumentada.
*
* Practice 2.
* Ray tracing.
*
* Jose Pascual Molina Masso.
* Escuela Superior de Ingenieria Informatica de Albacete.
*/

#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include "glm/vec3.hpp" // glm::vec3

#include "shadinfo.h"


/******************/
/* Material class */
/******************/

class Material {

public:

  /* Constructors */
  Material(const glm::vec3 &diff);
  Material(const glm::vec3 &amb, const glm::vec3 &diff, const glm::vec3 &diffTrans, 
	  const glm::vec3 &spec, const glm::vec3 &specTrans, int shine, const glm::vec3 &emis, 
	  const glm::vec3 &refl, const glm::vec3 &trans, float index);

  /* Implements the global illumination model */
  glm::vec3 Shade(ShadingInfo &shadInfo);

  /* As different objects can point and share the same material, they all must
     call IncRefs when start using it, and DecRefs when no longer need it */
  void IncRefs() { refs++; }
  void DecRefs() {
    refs--;
    if (!refs) delete this;
  }

//private:

	glm::vec3 Ka, Kd, Kdt, Ks, Kst, Ie, Kr, Kt;  // Local and global coefficients
	int n;  // Specular-reflection exponent
	float ior;  // Index of refraction
	
	int refs;  // Number of objects using this material
};

#endif  // !defined _MATERIAL_H_
