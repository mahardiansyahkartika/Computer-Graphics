varying vec3 norm;
varying vec3 cam_dir;
varying vec3 color;

// Declare any additional variables here. You may need some uniform variables.
uniform samplerCube cubeMap;

void main(void)
{
	// Your shader code here.
	// Note that this shader won't compile since gl_FragColor is never set.
	
	vec3 reflectedDirection = reflect(cam_dir, normalize(norm));
	gl_FragColor = textureCube(cubeMap, reflectedDirection);
}
