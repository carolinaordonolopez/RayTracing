/* matrices.h
*
* Realidad Virtual y Aumentada.
*
* Practice 1.
* Ray casting.
*
* Jose Pascual Molina Masso.
* Escuela Superior de Ingenieria Informatica de Albacete.
*/

#ifndef _MATRICES_H_
#define _MATRICES_H_

#include "glm/glm.hpp"
#include "glm/vec3.hpp"  // glm::vec3
#include "glm/mat4x4.hpp"  // glm::mat4


class Matrices  
{
public:
	Matrices();
	virtual ~Matrices();
	static glm::mat4 translate(const float &tx, const float &ty, const float &tz);
	static glm::mat4 scale(const float &sx, const float &sy, const float &sz);
	static glm::mat4 rotateX(const float degrees);
	static glm::mat4 rotateY(const float degrees);
	static glm::mat4 rotateZ(const float degrees);
	static glm::mat4 lookAt(const glm::vec3 &eye, const glm::vec3 &center, const glm::vec3 &up);
};

#endif  // !defined(_MATRICES_H_) 

