/* frame.h
 *
 * Realidad Virtual y Aumentada.
 *
 * Practice 2.
 * Ray tracing.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */

#ifndef _FRAME_H_
#define _FRAME_H_

#include "glm/vec3.hpp" // glm::vec3


class Frame {

public:

  /* Constructors and destructor */
  Frame();
  Frame(int Width, int Height);
  ~Frame();

  /* Operators and functions */

  /* Save the new width and height of the image, and if necessary
     it allocates more memory for the buffer where it is stored */
  int SetSize(int Width, int Height);

  /* Retrieves the width and height of the image */
  int GetWidth();
  int GetHeight();

  /* Save the color of a pixel at the given position */
  int SetPixel(int x, int y, const glm::vec3& color);

  /* Draw the image with OpenGL */
  void Draw() const;

private:
  
  long allocated;
  long width, height;
  float *buffer;
};


#endif // _FRAME_H_
