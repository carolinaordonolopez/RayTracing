/* shadinfo.h
*
* Realidad Virtual y Aumentada.
*
* Practice 2.
* Ray tracing.
*
* Jose Pascual Molina Masso.
* Escuela Superior de Ingenieria Informatica de Albacete.
*/

#ifndef _SHADINFO_H_
#define _SHADINFO_H_

#include "glm/vec3.hpp" // glm::vec3

/*********************/
/* ShadingInfo class */
/*********************/

class World;  // Forward declaration

class ShadingInfo {

public:

	glm::vec3 rayPos;  // Ray origin and ...
	glm::vec3 rayDir;  // ... direction
	float t;           // Value of t at the intersection 
	glm::vec3 point;   // Intersection point 
	glm::vec3 normal;  // Normal to the surface at that point
	int depth;         // Current depth
	World *pWorld;     // Pointer to World object
};

#endif  // !defined _SHADINFO_H_