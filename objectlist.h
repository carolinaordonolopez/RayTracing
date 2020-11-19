/* objectlist.h
 *
 * Realidad Virtual y Aumentada.
 *
 * Practice 2.
 * Ray tracing.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */

#ifndef _OBJECTLIST_H_
#define _OBJECTLIST_H_

#include "glm/vec3.hpp" // glm::vec3

#include "object.h"
#include "list.h"


/********************/
/* ObjectList class */
/********************/

class ObjectList : public List<Object*> {

public:

	/* Returns the nearest object in the direction of the ray. Returns a pointer to that object and 
	   the t value of the intersection point, otherwise returns NULL */
	Object* NearestInt(const glm::vec3& pos, const glm::vec3& dir, float& tnear, float tmax);
};


#endif  // !defined _OBJECTLIST_H_
