#include "scene/bound.hpp"
namespace _462{

bool Bound::collides(Vector3 position, real_t radius) {
	real_t dmin = 0;

	Vector3 center = position;
	Vector3 bmin = lower;
	Vector3 bmax = upper;

	if (center.x < bmin.x) {
		dmin += pow(center.x - bmin.x, 2);
	}
	else if (center.x > bmax.x) {
		dmin += pow(center.x - bmax.x, 2);
	}

	if (center.y < bmin.y) {
		dmin += pow(center.y - bmin.y, 2);
	}
	else if (center.y > bmax.y) {
		dmin += pow(center.y - bmax.y, 2);
	}

	if (center.z < bmin.z) {
		dmin += pow(center.z - bmin.z, 2);
	}
	else if (center.z > bmax.z) {
		dmin += pow(center.z - bmax.z, 2);
	}

	return dmin <= pow(radius, 2);
}

int Bound::longestAxis() {
	// axis-x = 0, axis-y = 1, axis-z = 2
	int result = 0;
	real_t longestAxis = upper.x - lower.x;
	
	real_t yAxis = upper.y - lower.y;
	if (yAxis > longestAxis) {
		longestAxis = yAxis;
		result = 1;
	}

	real_t zAxis = upper.z - lower.z;
	if (zAxis > longestAxis) {
		longestAxis = zAxis;
		result = 2;
	}

	return result;
}

void Bound::expand(Bound a) {
	lower.x = std::min(lower.x, a.lower.x);
	lower.y = std::min(lower.y, a.lower.y);
	lower.z = std::min(lower.z, a.lower.z);

	upper.x = std::max(upper.x, a.upper.x);
	upper.y = std::max(upper.y, a.upper.y);
	upper.z = std::max(upper.z, a.upper.z);
}
}