#include "scene/bound.hpp"
namespace _462{

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