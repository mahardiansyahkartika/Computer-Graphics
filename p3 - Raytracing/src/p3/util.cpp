#include "p3/util.hpp"
namespace _462{


//ensures that the light has a maximum intensity of 1, and in order
//to cancel this scaling factor, returns the probability of continuing.
real_t montecarlo(Color3& light){
    real_t factor=std::max(light.r,std::max(light.g,light.b));
    light*=1/factor;
    return factor;
}
   
real_t computeFresnelCoefficient(Intersection* next, Ray &ray, real_t index, real_t newIndex) {
	real_t cos_theta = dot(-ray.d, next->int_point.normal);

	real_t R_o = pow(((newIndex - 1) / (newIndex + 1)), 2);
	real_t R = R_o + ((1.0 - R_o)*pow(1.0 - cos_theta, 5));

	return R;
}

Vector3 reflect(Vector3 norm, Vector3 inc) {
	return normalize(inc - (2 * dot(inc, norm)*norm));
}

Vector3 refract(Vector3 norm, Vector3 inc, real_t ratio) {
	real_t squareRootValue = real_t(1.0) - ((pow(ratio, 2))*(real_t(1.0) - pow(dot(inc, norm), 2)));

	if (squareRootValue < 0){
		// Total Internal Reflection
		return Vector3(0, 0, 0);
	}
	// Refraction
	return normalize((ratio*(inc - norm*dot(inc, norm))) - (norm*sqrt(squareRootValue)));
}

}
