varying vec3 normalVec;
varying vec3 viewVec;

uniform sampler2D planeTexture;
uniform float altitude;
uniform int axis; //0 = +X, 1 = +Y, 2 = +Z, 3 = -X, 4 = -Y, 5 = -Z
uniform float border_gamma, border_alpha;

varying vec4 diffuse,ambientGlobal, ambient, specular;
varying vec3 lightDir,halfVector,normalView;
varying float dist;


vec3 reflect(vec3 I, vec3 N)
{
  return I - 2.0 * N * dot(N, I);
}

void main()
{
	float reflect_factor = 1.0;
	
	//Phong
	vec3 n,halfV;
	float NdotL,NdotHV;
	vec4 color = ambientGlobal + ambient;
	float att,spotEffect;
	
	/* a fragment shader can't write a verying variable, hence we need
	a new variable to store the normalized interpolated normal */
	n = normalize(normalView);
	
	/* compute the dot product between normal and ldir */
	NdotL = max(dot(n,normalize(lightDir)),0.0);

	if (NdotL > 0.0) {
	
		/*spotEffect = dot(normalize(gl_LightSource[0].spotDirection), normalize(-lightDir));
		if (spotEffect > gl_LightSource[0].spotCosCutoff) {
			spotEffect = pow(spotEffect, gl_LightSource[0].spotExponent);
			att = spotEffect / (gl_LightSource[0].constantAttenuation +
					gl_LightSource[0].linearAttenuation * dist +
					gl_LightSource[0].quadraticAttenuation * dist * dist);*/
				
			color += /*att * */ (diffuse * NdotL) ;
			
		//}
	}
	
	//end phong
	color = gl_Color;
	
	// Perform a simple 2D texture look up.
	vec3 base_color = gl_Color.xyz;//texture2D(planeTexture, reflectVec.xz).rgb;
	
	vec3 cube_color = vec3(0.0,0.0,0.0);
	
	vec3 reflectVec = reflect(viewVec, normalVec);
	
	bool testAxis = false;
	float t = 0.0;
	
	vec2 subReflectVec = vec2(0.0,0.0);
	
	if (axis == 0){
		testAxis = (reflectVec.x>0.0);
		t = altitude/reflectVec.x;
		subReflectVec = reflectVec.yz;
	}
	
	if (axis == 1){
		testAxis = (reflectVec.y>0.0);
		t = altitude/reflectVec.y;
		subReflectVec = reflectVec.xz;
	}
	if (axis == 2){
		testAxis = (reflectVec.z>0.0);
		t = altitude/reflectVec.z;
		subReflectVec = reflectVec.xy;
	}
	
	if (testAxis)
	{
		// Perform a cube map look up.
        cube_color = texture2D(planeTexture, subReflectVec*t+vec2(0.5,0.5)).rgb;
        cube_color *= specular.xyz;

	}
	vec3 unitNormalVec = normalize(normalVec);
	vec3 unitViewVec = normalize(viewVec);
	float alpha_color = color.w;
	float alpha_color2 = alpha_color + (border_alpha - alpha_color)* (pow( 1.0 - abs(dot(unitNormalVec,unitViewVec)), border_gamma));
	
	// Write the final pixel.
	gl_FragColor = vec4((color.xyz*alpha_color2)+cube_color,alpha_color2); //vec4( mix(base_color, cube_color, reflect_factor), 1.0);

}
