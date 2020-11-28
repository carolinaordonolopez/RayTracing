/* Material.cpp
*
* Realidad Virtual y Aumentada.
*
* Practice 2.
* Ray tracing.
*
* Carolina Ordoño López.
* Escuela Superior de Ingenieria Informatica de Albacete.
*/


#include "glm/glm.hpp" // glm::vec3, glm::dot

#include "Material.h"
#include "light.h"
#include "lightlist.h"
#include "world.h"
#include "limits.h"


/* Constructors */
Material::Material(const glm::vec3& diff) {

	Ka = glm::vec3(0.0, 0.0, 0.0);
	Kd = diff;
	Kdt = glm::vec3(0.0, 0.0, 0.0);
	Ks = glm::vec3(0.0, 0.0, 0.0);
	Kst = glm::vec3(0.0, 0.0, 0.0);
	n = 0;
	Ie = glm::vec3(0.0, 0.0, 0.0);
	Kr = glm::vec3(0.0, 0.0, 0.0);
	Kt = glm::vec3(0.0, 0.0, 0.0);
	ior = 0.0;
}

Material::Material(const glm::vec3& amb, const glm::vec3& diff, const glm::vec3& diffTrans,
	const glm::vec3& spec, const glm::vec3& specTrans, int shine, const glm::vec3& emis,
	const glm::vec3& refl, const glm::vec3& trans, float index) {

	Ka = amb;
	Kd = diff;
	Kdt = diffTrans;
	Ks = spec;
	Kst = specTrans;
	n = shine;
	Ie = emis;
	Kr = refl;
	Kt = trans;
	ior = index;
}

/* Implements the global illumination model */
glm::vec3 Material::Shade(ShadingInfo& shadInfo)
{
	glm::vec3 color(0.0, 0.0, 0.0), V;
	float VdotN, ratio = 0.0;
	bool isTrans;

	V = -shadInfo.rayDir;
	VdotN = glm::dot(V, shadInfo.normal);
	isTrans = (Kt != glm::vec3(0.0, 0.0, 0.0));
	if (VdotN < 0) {

		// The viewer stares at an interior or back face of the object,
		// we will only illuminate it if material is transparent
		if (isTrans) {
			shadInfo.normal = -shadInfo.normal;  // Reverse normal
			VdotN = -VdotN;
			ratio = 1.0 / ior; // Ray always comes from vacuum (hollow objects)
			//ratio = ior;  // Use this instead for solid objects
		}
		else
			return color;
	}
	else {

		// The viewer stares at a front face of the object
		if (isTrans)
			ratio = 1.0 / ior; // Ray comes from vacuum
	}

	// To do ...



	///////////////////////
	// Iluminación LOCAL //
	///////////////////////

	glm::vec3 Ilocal, Id, Idt, Is, Ist, Ia;
	glm::vec3 L, R, T;
	float LdotN, Ldot_N, RdotV, TdotV;

	Light* currentLight = shadInfo.pWorld->lights.First();

	while (currentLight != NULL) {
		Id = Idt = Is = Ist = Ia = glm::vec3(0.0, 0.0, 0.0);
		// L = vector desde el punto de interés hacia la fuente de luz
		L = glm::normalize(currentLight->position - shadInfo.point);
		LdotN = glm::dot(L, shadInfo.normal);
		shadInfo.pWorld->numShadRays++;


		if (LdotN > 0) { //Si=1 por lo que habría reflexión

			// Reflexión difusa => Id = Kd * sum(Si*Ilid*(Li*N))
			Id = Kd * currentLight->Id * LdotN;

			//Reflexión especular => Is = ks* sum(Si*Ilis*(Ri*V))^n
			R = glm::normalize(2 * LdotN * shadInfo.normal - L);
			V = glm::normalize(-shadInfo.rayDir);
			RdotV = glm::dot(R, V);
			if (RdotV > 0) {
				Is = Ks * currentLight->Is * pow(RdotV, n);
			}
		}
		else if (LdotN < 0) { //Si'=1 por lo que habría transmisión

			//Transmisión difusa => Idt = Kdt * sum(S'i*Ilid*(Li*(-N)))
			Ldot_N = glm::dot(L, -shadInfo.normal);
			if (Ldot_N > 0) {	
				Idt = Kdt * currentLight->Id * Ldot_N;
			}

			//Transmisión especular => Ist = Kst* sum(Si'*Ilis*(Ti*V)^n

			// Calculo del RAYO REFRACTADO
			float b = 0.0;
			float cos = glm::dot(L, shadInfo.normal);
			float aux = 1 + ratio * ratio * (cos * cos - 1);
			if (aux >= 0) {
				b = ratio * cos - sqrt(aux);
			}
			T = glm::normalize(ratio * L + b * shadInfo.normal);
			TdotV = glm::dot(T, V);

			if (TdotV > 0) {
				Ist = Kst * currentLight->Is * pow(TdotV, n);
			}
		}

		// Luz ambiental de otros cuerpos => Ia = Ka*sum(Ilia))
		Ia = Ka * currentLight->Ia;


		/////////////////////////////////////////////////////////////////////////////////////
		////// Calcular cuanta luz es bloqueada por superficies opacas y transparentes //////

		// Atenuación en un punto para sombras
		glm::vec3 atenuation = glm::vec3(1.0, 1.0, 1.0);

		Object* currentObject = shadInfo.pWorld->objects.First();
		float t_aux = 0.0f;

		while (currentObject != NULL) { //Recorremos los objetos
			t_aux = currentObject->NearestInt(shadInfo.point, L);
			if (t_aux > TMIN) {
				atenuation *= currentObject->pMaterial->Kt; //Si es opaco se multiplicará por 0
			}
			currentObject = shadInfo.pWorld->objects.Next();
		}

		//Sumamos las componentes de la luz teniendo en cuenta la atenuación
		Ilocal += Ia + (Id + Is + Idt + Ist) * atenuation; 

		currentLight = shadInfo.pWorld->lights.Next();
	}

	color += Ie;
	color += Ilocal;


	////////////////////////
	// Iluminación GLOBAL //
	////////////////////////

	if (shadInfo.depth < shadInfo.pWorld->maxDepth) {

		bool isSpecular = (Ks != glm::vec3(0.0, 0.0, 0.0) || Kst != glm::vec3(0.0, 0.0, 0.0));
		if (isSpecular) {
			//rayo reflexion = en la direccion de perfecta reflexion desde el punto de interseccion = Cambiar L por V en el R
			if (VdotN > 0) {
				glm::vec3 R = glm::normalize(2 * VdotN * shadInfo.normal - V);
				color += Kr * shadInfo.pWorld->Trace(shadInfo.point, R, shadInfo.depth + 1);
				shadInfo.pWorld->numReflRays++;
			}
		}

		if (isTrans) {
			if (VdotN > 0) {
				//RAYO REFRACCIÓN = en la dirección de refracción desde el punto de intersección
				float b = 0.0;
				float aux = 1 + ratio * ratio * (VdotN * VdotN - 1);
				if (aux >= 0) {
					b = ratio * VdotN - sqrt(aux);
				}
				glm::vec3 T = glm::normalize(ratio * (-V) + b * shadInfo.normal);

				color += Kt * shadInfo.pWorld->Trace(shadInfo.point, T, shadInfo.depth + 1);
				shadInfo.pWorld->numRefrRays++;
			}
		}
	}

	return color;
}




