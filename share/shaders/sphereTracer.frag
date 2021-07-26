#version 460
#extension GL_ARB_gpu_shader5 : enable

layout( location = 0 ) out vec4 fragColor;

layout (std140, location = 1) uniform TetrahedronDetails
{
    mat4 T;
    mat4 Tinv;
    mat4 U;
} tetras[];

uniform float zNear ; 
uniform float zFar ; 

uniform mat4 projection_matrix;
uniform mat4 object_matrix;

const int MAX_MARCHING_STEPS = 512;
const float MIN_DIST = 0.00001;
const float MAX_DIST = 100.0;
const float EPSILON = 0.00001;

const vec3 lightPos=vec3(0.0,1.0,1.0);

float depthSample(float linearDepth)
{
    float nonLinearDepth = (zFar + zNear - 2.0 * zNear * zFar / linearDepth) / (zFar - zNear);
    nonLinearDepth = (nonLinearDepth + 1.0) / 2.0;
    return nonLinearDepth;
}

float sdSphere( vec3 p, vec3 center, float radius )
{

  return length(p-center)-radius;
}

float displacement(in vec3 p)
{
    return sin(1*p.x)*sin(4*p.y)*sin(4*p.z);
}

vec4 map(in vec3 pos)
{   
    float d1 = sdSphere(pos, vec3(0.0,0.0,0.0), 1.0); 
    float d2 = displacement(pos);
	return vec4(d1+d2, vec3(1.0,1.0,1.0)) ;
}

vec4 castRay( in vec3 ro, in vec3 rd )
{
   float depth = depthSample(gl_FragCoord.z);
   vec4 c;    
   vec3 m = vec3(-1.0, -1.0, -1.0);
   for( int i = 0; i < MAX_MARCHING_STEPS; i++ )
   {
       vec4 p_i = vec4(ro+rd*depth, 1);
       
       /// Compute the first three values
       //c = tetras[0].Tinv * (p_i); 
       
       /// Compute the first three values
       //c.w = 1.0 - c.x - c.y - c.z;
       
       //vec4 u_i = tetras[0].U * c;
       vec4 res = map( p_i.xyz );
       if(res.x < EPSILON && res.x > -EPSILON) 
       {
           return vec4( depth, m );
       }
       //if( depth > MAX_DIST )
       // break;     
       depth += res.x*0.1;
       m = res.yzw;
   }
   
   return vec4( MAX_DIST+1.0, m );    
}

vec4 castRay2( in vec3 ro, in vec3 rd )
{
   float depth = depthSample(gl_FragCoord.z);
   
   vec3 m = vec3(-1.0, -1.0, -1.0);
   
   ro = ro + rd * depth; 
   for( int i = 0; i < MAX_MARCHING_STEPS; i++ )
   {
       vec4 res = map( ro+rd*depth );
       if( res.x < EPSILON || depth > MAX_DIST ) 
       	break;
       depth += res.x;
       m = res.yzw;
   }
   return vec4( depth, m );
}


float softshadow( in vec3 ro, in vec3 rd, in float mint, in float tmax )
{
   float res = 1.0;
   float t = mint;
   for( int i=0; i<16; i++ )
   {
      float h = map( ro + rd*t ).x;
      res = min( res, 8.0*h/t );
      t += clamp( h, 0.02, 0.10 );
      if( h<0.001 || t>tmax ) break;
   }
   return clamp( res, 0.0, 1.0 );
}

vec3 estimateNormal(vec3 p) {
   return normalize(vec3(
       map(vec3(p.x + EPSILON, p.y, p.z)).x - map(vec3(p.x - EPSILON, p.y, p.z)).x,
       map(vec3(p.x, p.y + EPSILON, p.z)).x - map(vec3(p.x, p.y - EPSILON, p.z)).x,
       map(vec3(p.x, p.y, p.z  + EPSILON)).x - map(vec3(p.x, p.y, p.z - EPSILON)).x
   ));
}

vec4 render( in vec3 ro, in vec3 rd )
{
    
   vec4 res = castRay(ro,rd);
   
   if (res.x > MAX_DIST) 
   	return vec4(1.0, 0.0, 1.0, res.x);
    	
   float dist = res.x;
   vec3 col = res.yzw;
   vec3 pos = ro + dist * rd;
   vec3 nor = estimateNormal( pos );
   vec3 ref = reflect( rd, nor );

   vec3  lig = normalize(lightPos);
   float amb = clamp( 0.5+0.5*nor.y, 0.0, 1.0 );
   float dif = clamp( dot( nor, lig ), 0.0, 1.0 );
   float dom = smoothstep( -0.1, 0.1, ref.y );
   float spe = pow(clamp( dot( ref, lig ), 0.0, 1.0 ),16.0);
   dif *= softshadow( pos, lightPos, 0.02, 2.5 );
 
   vec3 lin = vec3(0.0);
   lin += 1.30*dif*vec3(1.00,0.80,0.55);
   lin += 2.00*spe*vec3(1.00,0.90,0.70)*dif;
   lin += 0.40*amb*vec3(0.40,0.60,1.00);
   lin += 0.50*dom*vec3(0.40,0.60,1.00);
   col = col*lin;

   return vec4( clamp(col,0.0,1.0), res.x );
}



vec3 cameraPos(mat4 a_modelView)
{
 // Get the 3 basis vector planes at the camera origin and transform them into model space.
  //  
  // NOTE: Planes have to be transformed by the inverse transpose of a matrix
  //       Nice reference here: http://www.opengl.org/discussion_boards/showthread.php/159564-Clever-way-to-transform-plane-by-matrix
  //
  //       So for a transform to model space we need to do:
  //            inverse(transpose(inverse(MV)))
  //       This equals : transpose(MV) - see Lemma 5 in http://mathrefresher.blogspot.com.au/2007/06/transpose-of-matrix.html
  //
  // As each plane is simply (1,0,0,0), (0,1,0,0), (0,0,1,0) we can pull the data directly from the transpose matrix.
  //  
  mat4 modelViewT = transpose(a_modelView);
 
  // Get plane normals 
  vec3 n1=modelViewT[0].xyz;
  vec3 n2=modelViewT[1].xyz;
  vec3 n3=modelViewT[2].xyz;
 
  // Get plane distances
  float d1=modelViewT[0].w;
  float d2=modelViewT[1].w;
  float d3=modelViewT[2].w;
 
  // Get the intersection of these 3 planes 
  // (uisng math from RealTime Collision Detection by Christer Ericson)
  vec3 n2n3 = cross(n2, n3);
  float denom = dot(n1, n2n3);
 
  vec3 top = (n2n3 * d1) + cross(n1, (d3*n2) - (d2*n3));
  return top / -denom;
}

void main()
{		
   /// Generate the ray 
   vec2 rayPos = gl_FragCoord.st / vec2(779,600) - vec2(0.5,0.5);
   vec4 rayNormalized = vec4(rayPos, -1.0, 1.0) ;
   vec4 rayEye = inverse(projection_matrix) * rayNormalized ;   
   rayEye = vec4(rayEye.xy, -1.0, 0.0) ; 
   vec3 rayWorld = normalize((inverse(object_matrix) * rayEye).xyz) ;

   vec3 rayOrigin2=cameraPos(object_matrix);      //set ray origin
   vec4 res = render( rayOrigin2, rayWorld );
   //gl_FragDepth = depthSample(res.w);
   //gl_FragDepth = 1.0-res.w;
   if (res.w > MAX_DIST) 
    discard;
        
   fragColor = vec4(1.0, rayPos.x, rayPos.y, 1.0);
   fragColor = vec4(res.xyz , 1.0 );   
   
   //fragColor = vec4(1.0,1.0,0.0,0.0);
   //fragColor = vec4(1.0,0.0,1.0, 1.0 );   
   //fragColor = vec4(rayOrigin,1);
}
