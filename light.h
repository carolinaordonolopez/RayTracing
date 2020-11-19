/* light.h
 *
 * Realidad Virtual y Aumentada.
 *
 * Practice 2.
 * Ray tracing.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */

#ifndef _LIGHT_H_
#define _LIGHT_H_

#include "glm/vec3.hpp" // glm::vec3


/****************/
/* Light class */
/****************/

class Light {

public:

	/* Constructor */
	Light(const glm::vec3& Position, const glm::vec3& diff);
	Light(const glm::vec3& Position, const glm::vec3& amb, const glm::vec3& diff, const glm::vec3& spec);

	glm::vec3 position;
	glm::vec3 Ia, Id, Is; 
};

/* Constructor */
inline Light::Light(const glm::vec3& Position, const glm::vec3& diff) {
	position = Position;
	Ia = glm::vec3(0.0, 0.0, 0.0);
	Id = diff;
	Is = glm::vec3(0.0, 0.0, 0.0);
}

inline Light::Light(const glm::vec3& Position, const glm::vec3& amb, const glm::vec3& diff, const glm::vec3& spec) {
	position = Position;
	Ia = amb;
	Id = diff;
	Is = spec;
}


#endif  // !defined _LIGHT_H_
