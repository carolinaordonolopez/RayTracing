/* NFF.cpp
*
* Realidad Virtual y Aumentada.
*
* Practice 2.
* Ray tracing.
*
* Jose Pascual Molina Masso.
* Escuela Superior de Ingenieria Informatica de Albacete.
*/

#include <fstream>
using namespace std;

#include "glm/glm.hpp" // glm::vec3, glm::dot

#include "raytracer.h"


/* Loads a NFF (Neutral File Format) file */
int RayTracer::LoadFileNFF(const char *name)
{
  char sigToken[256];
  char token;
  Material *pMaterial = 0;

  ifstream filestr(name);
  if (! filestr)
    return -1;  // No se puede abrir el archivo

  sigToken[0] = '\0';
  while (! filestr.eof())
  {
	if (sigToken[0] != 0) {
	  token = sigToken[0];
	  sigToken[0] = '\0';
	}
	else {
	  filestr >> token;
	  if (! filestr)
	    break;
	}

	switch (token)
    {
	  case 'v':  // Viewpoint location
	  {
	    float Fx, Fy, Fz;
	    float Ax, Ay, Az;
	    float Ux, Uy, Uz;
	    float angle, hither;
	    int xres, yres;

		filestr >> sigToken;
		if (strcmp(sigToken, "from"))
		  return -1;
		filestr >> Fx >> Fy >> Fz >> sigToken;
		if (strcmp(sigToken, "at"))
          return -1;
		filestr >> Ax >> Ay >> Az >> sigToken;
		if (strcmp(sigToken, "up"))
          return -1;
		filestr >> Ux >> Uy >> Uz >> sigToken;
		if (strcmp(sigToken, "angle"))
          return -1;
		filestr >> angle >> sigToken;
		if (strcmp(sigToken, "hither"))
		  return -1;
		filestr >> hither >> sigToken;
        if (strcmp(sigToken, "resolution"))
          return -1;
        filestr >> xres >> yres;

		world.eye = glm::vec3(Fx, Fy, Fz);
		world.center = glm::vec3(Ax, Ay, Az);
		world.up = glm::vec3(Ux, Uy, Uz);
		view.SetLookAt(world.eye, world.center, world.up);
		view.SetPerspective(angle, (float)xres / yres);
		frame.SetSize(xres, yres);

		sigToken[0] = '\0';
	  	break;
      }
	  case 'b':  // Background color
	  {
		 float R, G, B;

		 filestr >> R >> G >> B;
		 world.bgcolor = glm::vec3(R, G, B);
         break;
	  }
	  case 'l':  // Point light
	  {
		float X, Y, Z;
		float R, G, B;
		char *endptr;
		Light *pLight;

		filestr >> X >> Y >> Z;
		filestr >> sigToken;
		R = strtod(sigToken, &endptr);
		if (endptr == sigToken)
        {
			pLight = new Light(glm::vec3(X, Y, Z), 
				glm::vec3(1.0, 1.0, 1.0), glm::vec3(1.0, 1.0, 1.0), glm::vec3(1.0, 1.0, 1.0));
			world.lights.Add(pLight);
        }
		else
        {
		  filestr >> G >> B;
		  pLight = new Light(glm::vec3(X, Y, Z), 
			  glm::vec3(R, G, B), glm::vec3(R, G, B), glm::vec3(R, G, B));
		  world.lights.Add(pLight);
		  sigToken[0] = '\0';
		}
        break;
	  }
      case 'f':  // Fill color
      {
		float r, g, b, kd, ks, shine, t, ior;

		filestr >> r >> g >> b >> kd >> ks >> shine >> t >> ior;
		if (t == 0.0)
			pMaterial = new Material(glm::vec3(0.0, 0.0, 0.0), glm::vec3(kd*r, kd*g, kd*b), glm::vec3(0.0, 0.0, 0.0),
				glm::vec3(ks*r, ks*g, ks*b), glm::vec3(0.0, 0.0, 0.0), shine, glm::vec3(0.0, 0.0, 0.0), 
				glm::vec3(ks*r, ks*g, ks*b), glm::vec3(0.0, 0.0, 0.0), ior);
		else
			pMaterial = new Material(glm::vec3(0.0, 0.0, 0.0), glm::vec3(kd*r, kd*g, kd*b), glm::vec3(kd*r, kd*g, kd*b),
				glm::vec3(ks*r, ks*g, ks*b), glm::vec3(ks*r, ks*g, ks*b), shine, glm::vec3(0.0, 0.0, 0.0),
				glm::vec3(ks*r, ks*g, ks*b), glm::vec3(t*r, t*g, t*b), ior);
	    break;
      }
      case 's':  // Sphere
      {
		float centerx, centery, centerz, radius;
		Sphere *pSphere;

		filestr >> centerx >> centery >> centerz >> radius;
		pSphere = new Sphere(glm::vec3(centerx, centery, centerz), radius, pMaterial);
		world.objects.Add(pSphere);
        break;
      }
      case 'p':
      {
        int n;

		if (filestr.peek() == 'p')  // Polygonal patch
        {
		  filestr >> token;
		  filestr >> n;
		  for (int i = 0; i < n; i++)
          {
		    float vertx, verty, vertz, normx, normy, normz;
		    filestr >> vertx >> verty >> vertz >> normx >> normy >> normz;
		  }
		}
		else  // Polygon
        {
          filestr >> n;
		  for (int i = 0; i < n; i++)
          {
		    float vertx, verty, vertz;
		    filestr >> vertx >> verty >> vertz;
          }
		}

        break;
      }
	  case '#':  // Comment
      {
		filestr.getline(sigToken, 256);
		sigToken[0] = '\0';
		break;
      }
    }
  }
  return 0;
}
