/* raytracer.h
 *
 * Realidad Virtual y Aumentada.
 *
 * Practice 2.
 * Ray tracing.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */

#ifndef _RAYTRACER_H_
#define _RAYTRACER_H_

#include "world.h"
#include "view.h"
#include "frame.h"


/*******************/
/* RayTracer class */
/*******************/

class RayTracer {

public:

	/* Constructor */
	RayTracer::RayTracer() : world(), view(), frame() { };

	/* Loads a NFF (Neutral File Format) file */
	int LoadFileNFF(const char *name);

	World world;  // Objects, lights, camera, bgcolor, statistics
	View view;    // View parameters
	Frame frame;  // Frame buffer
};


#endif  // !defined _RAYTRACER_H_
