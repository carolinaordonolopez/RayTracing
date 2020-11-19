/* objectlist.cpp
 *
 * Realidad Virtual y Aumentada.
 *
 * Practice 2.
 * Ray tracing.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */


#include "objectlist.h"
#include "limits.h"


/* Returns the nearest object in the direction of the ray. Returns a pointer to that object and
   the t value of the intersection point, otherwise returns NULL */
Object* ObjectList::NearestInt(const glm::vec3& pos, const glm::vec3& dir, float& tnear, float tmax) {

	Object *optr, *nearObj = NULL;
	float t;

	if (Length() > 0) {

		tnear = tmax;

		/* Access the object list in sequence */
		optr = First();
		while (optr != NULL) {

			/* Compute ray-object intersection */
			t = optr->NearestInt(pos, dir);
			if ((TMIN < t) && (t < tnear)) {  // Beware of self-occlusion
				nearObj = optr;
				tnear = t;
			}
			optr = Next();
		}
	}

	return nearObj;
}
