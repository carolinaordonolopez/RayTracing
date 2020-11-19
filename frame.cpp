/* frame.cpp
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
#include <GL/gl.h>

#include "frame.h"


/* Constructors */
Frame::Frame() 
{ 
  allocated = 0;
  buffer = NULL;
}

Frame::Frame(int Width, int Height)
{
  allocated = 0;
  buffer = NULL;
  SetSize(Width,Height); 
}

/* Destructor */
Frame::~Frame() 
{
  delete[] buffer;
}

/* Save the new width and height of the image, and if necessary
   it allocates more memory for the buffer where it is stored */
int Frame::SetSize(int Width, int Height) {
  long newAlloc = ((long)Width)*((long)Height)*3;
  if (newAlloc > allocated) {
    delete[] buffer;
	buffer = new float[newAlloc];
	if (buffer == NULL)
      return 0;
	allocated = newAlloc;
  }
  width = Width;
  height = Height;

  return 1;
}

/* Retrieves the width of the image */
int Frame::GetWidth() { 
	return width;
}

/* Retrieves the height of the image */
int Frame::GetHeight() { 
	return height;
}

/* Save the color of a pixel at the given position */
int Frame::SetPixel(int x, int y, const glm::vec3& color)
{
  float* cptr;
  
  if (buffer == NULL)
    return 0;

  cptr = const_cast<float*>(buffer + 3*( ((long)y)*width + ((long)x) ));
  *(cptr++) = color.x;
  *(cptr++) = color.y;
  *(cptr) = color.z;

  return 1;
}

/* Draw the image with OpenGL */
void Frame::Draw() const
{
  glPixelStorei(GL_UNPACK_ROW_LENGTH, width);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
  glRasterPos2i(0,0);		// Position at base of window
  glDrawPixels(width, height, GL_RGB, GL_FLOAT, buffer);
}
