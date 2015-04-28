#include "scene/bound.hpp"
namespace _462{

real_t Bound::squared_dist(Vector3 point) {
	auto check = [&](const real_t pn, const real_t bmin, const real_t bmax) -> real_t {
		real_t out = 0;
		real_t v = pn;

		if (v < bmin) {
			real_t val = (bmin - v);
			out += val * val;
		}

		if (v > bmax) {
			real_t val = (v - bmax);
			out += val * val;
		}

		return out;
	};

	// Squared distance
	real_t sq = 0.0;

	sq += check(point.x, lower.x, upper.x);
	sq += check(point.y, lower.y, upper.y);
	sq += check(point.z, lower.z, upper.z);

	return sq;
}

bool Bound::collides(SphereBody* body) {
	return	squared_dist(body->position) <= body->radius * body->radius;
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