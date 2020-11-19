/* matrices.cpp
*
* Realidad Virtual y Aumentada.
*
* Practice 2.
* Ray tracing.
*
* Jose Pascual Molina Masso.
* Escuela Superior de Ingenieria Informatica de Albacete.
*/

#include "matrices.h"


// Note: when you construct a matrix using mat4(), it will be COLUMN-MAJOR

//*******************************************************************

glm::mat4 Matrices::translate(const float &tx, const float &ty, const float &tz)
{
	glm::mat4 ret;

	// Implement translation 

	// first column
	ret[0][0] = 1.0;
	ret[0][1] = 0.0;
	ret[0][2] = 0.0;
	ret[0][3] = 0.0;
	// second column
	ret[1][0] = 0.0;
	ret[1][1] = 1.0;
	ret[1][2] = 0.0;
	ret[1][3] = 0.0;
	// third column
	ret[2][0] = 0.0;
	ret[2][1] = 0.0;
	ret[2][2] = 1.0;
	ret[2][3] = 0.0;
	// fourth column
	ret[3][0] = tx;
	ret[3][1] = ty;
	ret[3][2] = tz;
	ret[3][3] = 1.0;

	return ret;
}

//*******************************************************************

glm::mat4 Matrices::scale(const float &sx, const float &sy, const float &sz) 
{
    glm::mat4 ret;

    // Implement scaling 

	// first column
	ret[0][0] = sx;
	ret[0][1] = 0.0;
	ret[0][2] = 0.0;
	ret[0][3] = 0.0;
	// second column
	ret[1][0] = 0.0;
	ret[1][1] = sy;
	ret[1][2] = 0.0;
	ret[1][3] = 0.0;
	// third column
	ret[2][0] = 0.0;
	ret[2][1] = 0.0;
	ret[2][2] = sz;
	ret[2][3] = 0.0;
	// fourth column
	ret[3][0] = 0.0;
	ret[3][1] = 0.0;
	ret[3][2] = 0.0;
	ret[3][3] = 1.0;

    return ret;
}

//*******************************************************************

glm::mat4 Matrices::rotateX(const float degrees)
{
	glm::mat4 ret;

	// Implement rotation about X-axis

	float radians;
	radians = glm::radians(degrees);

	// first column
	ret[0][0] = 1.0;
	ret[0][1] = 0.0;
	ret[0][2] = 0.0;
	ret[0][3] = 0.0;
	// second column
	ret[1][0] = 0.0;
	ret[1][1] = glm::cos(radians);
	ret[1][2] = glm::sin(radians);
	ret[1][3] = 0.0;
	// third column
	ret[2][0] = 0.0;
	ret[2][1] = -glm::sin(radians);
	ret[2][2] = glm::cos(radians);
	ret[2][3] = 0.0;
	// fourth column
	ret[3][0] = 0.0;
	ret[3][1] = 0.0;
	ret[3][2] = 0.0;
	ret[3][3] = 1.0;

	return ret;
}

//*******************************************************************

glm::mat4 Matrices::rotateY(const float degrees)
{
	glm::mat4 ret;

	// Implement rotation about Y-axis 

	float radians;
	radians = glm::radians(degrees);

	// first column
	ret[0][0] = glm::cos(radians);
	ret[0][1] = 0.0;
	ret[0][2] = -glm::sin(radians);
	ret[0][3] = 0.0;
	// second column
	ret[1][0] = 0.0;
	ret[1][1] = 1.0;
	ret[1][2] = 0.0;
	ret[1][3] = 0.0;
	// third column
	ret[2][0] = glm::sin(radians);
	ret[2][1] = 0.0;
	ret[2][2] = glm::cos(radians);
	ret[2][3] = 0.0;
	// fourth column
	ret[3][0] = 0.0;
	ret[3][1] = 0.0;
	ret[3][2] = 0.0;
	ret[3][3] = 1.0;

	return ret;
}

//*******************************************************************

glm::mat4 Matrices::rotateZ(const float degrees)
{
	glm::mat4 ret;

	// Implement rotation about Z-axis

	float radians;
	radians = glm::radians(degrees);

	// first column
	ret[0][0] = glm::cos(radians);
	ret[0][1] = glm::sin(radians);
	ret[0][2] = 0.0;
	ret[0][3] = 0.0;
	// second column
	ret[1][0] = -glm::sin(radians);
	ret[1][1] = glm::cos(radians);
	ret[1][2] = 0.0;
	ret[1][3] = 0.0;
	// third column
	ret[2][0] = 0.0;
	ret[2][1] = 0.0;
	ret[2][2] = 1.0;
	ret[2][3] = 0.0;
	// fourth column
	ret[3][0] = 0.0;
	ret[3][1] = 0.0;
	ret[3][2] = 0.0;
	ret[3][3] = 1.0;

	return ret;
}

//*******************************************************************

glm::mat4 Matrices::lookAt(const glm::vec3 &eye, const glm::vec3 &center, const glm::vec3 &up)
{
	glm::mat4 ret;

	glm::vec3 u, v, n;
	glm::mat4 T, R;

	n = glm::normalize(eye - center);
	u = glm::normalize(glm::cross(up, n));
	v = glm::cross(n, u);

	T = glm::mat4(1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		-eye.x, -eye.y, -eye.z, 1.0);

	R = glm::mat4(u.x, v.x, n.x, 0.0,
		u.y, v.y, n.y, 0.0,
		u.z, v.z, n.z, 0.0,
		0.0, 0.0, 0.0, 1.0);

	ret = R*T;

	return ret;
}

//*******************************************************************

Matrices::Matrices()
{

}

//*******************************************************************

Matrices::~Matrices()
{

}
