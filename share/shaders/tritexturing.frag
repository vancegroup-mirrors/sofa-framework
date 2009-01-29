uniform sampler2D planarTextureX, planarTextureY, planarTextureZ;

varying vec3 normalVec;
varying vec3 viewVec;
varying vec4 vPosition, vPositionW;

uniform vec2 scaleTexture;

varying vec4 diffuse, ambient, specular;
varying vec3 lightDir, normalView, halfVector;
varying float dist;

void main()
{
	//XY = 1 0 0
	//XZ = 0 1 0
	//YZ = 0 0 1
	vec4 color = vec4(0.0,0.0,0.0,0.0);//gl_Color;
	vec4 pos = (vPositionW);
	
	vec3 unitNormalVec = normalize(normalVec);
	vec3 unitViewVec = normalize(viewVec);

	// Write the final pixel.
	//gl_FragColor = texture2D(planarTexture,gl_TexCoord[0].st);
	//XY -> Z
	//color.g = pos.y;
	color += abs(texture2D(planarTextureZ,vec2(pos.x/scaleTexture.x,pos.y/scaleTexture.y) ) * unitNormalVec.z);
	//XZ -> Y
	//color.r = pos.x;
	color += abs(texture2D(planarTextureY,vec2(pos.x/scaleTexture.x,pos.z/scaleTexture.y) ) * unitNormalVec.y);
	//YZ -> X
	//color.b = pos.z;
	color += abs(texture2D(planarTextureX,vec2(pos.y/scaleTexture.x,pos.z/scaleTexture.y) ) * unitNormalVec.x);

	color.a = 1.0;
	
	//Phong
	vec3 n,halfV;
	float NdotL,NdotHV;
	float att,spotEffect;
	vec4 phong_color = color * ambient;
	
	/* a fragment shader can't write a verying variable, hence we need
	a new variable to store the normalized interpolated normal */
	n = normalize(normalView);
	
	/* compute the dot product between normal and ldir */
	NdotL = max(dot(n,normalize(lightDir)),0.0);

	if (NdotL > 0.0) {
	
		spotEffect = dot(normalize(gl_LightSource[0].spotDirection), normalize(-lightDir));
		if (spotEffect > gl_LightSource[0].spotCosCutoff) {
			spotEffect = pow(spotEffect, gl_LightSource[0].spotExponent);
			att = spotEffect / (gl_LightSource[0].constantAttenuation +
					gl_LightSource[0].linearAttenuation * dist +
					gl_LightSource[0].quadraticAttenuation * dist * dist);
				
			phong_color += att *  (diffuse*color * NdotL) ;
			
			halfV = normalize(halfVector);
			NdotHV = max(dot(n,halfV),0.0);
			phong_color += att * gl_FrontMaterial.specular * gl_LightSource[0].specular * pow(NdotHV,gl_FrontMaterial.shininess);
			
		}
	}
	
	gl_FragColor = phong_color;
}
