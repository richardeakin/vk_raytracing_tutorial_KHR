#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#extension GL_ARB_shader_clock : enable
#include "raycommon.glsl"
#include "wavefront.glsl"

hitAttributeEXT vec3 attribs;

layout(location = 0) rayPayloadInEXT hitPayload prd;
layout(location = 1) rayPayloadEXT bool isShadowed;

layout(binding = 0, set = 0) uniform accelerationStructureEXT topLevelAS;

layout(binding = 1, set = 1, scalar) buffer MatColorBufferObject {
    WaveFrontMaterial m[];
} materials[];

layout(binding = 2, set = 1, scalar) buffer ScnDesc {
    sceneDesc i[];
} scnDesc;

layout(binding = 3, set = 1) uniform sampler2D textureSamplers[];

layout(binding = 4, set = 1)  buffer MatIndexColorBuffer {
    int i[];
} matIndex[];

layout(binding = 5, set = 1, scalar) buffer Vertices {
    Vertex v[];
} vertices[];

layout(binding = 6, set = 1) buffer Indices {
    uint i[];
} indices[];

layout(push_constant) uniform Constants
{
  vec4  clearColor;
  vec3  lightPosition;
  float lightIntensity;
  int   lightType;
}
pushC;

// http://www.neilmendoza.com/glsl-rotation-about-an-arbitrary-axis/
mat4 angleAxis( float angle, vec3 axis )
{
    axis = normalize(axis);
    float s = sin(angle);
    float c = cos(angle);
    float oc = 1.0 - c;
    
    return mat4( oc * axis.x * axis.x + c,           oc * axis.x * axis.y - axis.z * s,  oc * axis.z * axis.x + axis.y * s,  0.0,
                 oc * axis.x * axis.y + axis.z * s,  oc * axis.y * axis.y + c,           oc * axis.y * axis.z - axis.x * s,  0.0,
                 oc * axis.z * axis.x - axis.y * s,  oc * axis.y * axis.z + axis.x * s,  oc * axis.z * axis.z + c,           0.0,
                 0.0,                                0.0,                                0.0,                                1.0 );
}

#ifndef PI
#define PI 3.14159265359
#endif

float nextRand( float seed )
{
    return fract( sin( seed * 30103.03 ) * 121.1 ); // TODO: improve this
}

// Returns a random direction vector inside a cone
// Angle defined in radians
// Example: direction=(0,1,0) and angle=pi returns ([-1,1],[0,1],[-1,1])
vec3 getConeSample( float randSeed, vec3 direction, float coneAngle )
{
    float cosAngle = cos(coneAngle);

    // Generate points on the spherical cap around the north pole
    // https://math.stackexchange.com/a/205589/81266
    float z = nextRand(randSeed) * (1.0f - cosAngle) + cosAngle;
    float phi = nextRand(randSeed) * 2.0f * PI;

    float x = sqrt( 1.0 - z * z ) * cos(phi);
    float y = sqrt( 1.0 - z * z ) * sin(phi);
    vec3 north = vec3( 0, 1, 0 );

    // Find the rotation axis `u` and rotation angle `rot` [1]
    vec3 axis = normalize(cross(north, normalize(direction)));
    float angle = acos(dot(normalize(direction), north));

    // Convert rotation axis and angle to 3x3 rotation matrix [2]
    mat3 R = mat3( angleAxis( angle, axis ) );

    return R * vec3(x, y, z);
}

void main()
{
//  hitValue = vec3(0.05, 0.7, 0.5);

    // Object of this instance
    uint objId = scnDesc.i[gl_InstanceCustomIndexEXT].objId;

    // Indices of the triangle
    ivec3 ind = ivec3(indices[nonuniformEXT(objId)].i[3 * gl_PrimitiveID + 0],   //
                    indices[nonuniformEXT(objId)].i[3 * gl_PrimitiveID + 1],   //
                    indices[nonuniformEXT(objId)].i[3 * gl_PrimitiveID + 2]);  //
    // Vertex of the triangle
    Vertex v0 = vertices[nonuniformEXT(objId)].v[ind.x];
    Vertex v1 = vertices[nonuniformEXT(objId)].v[ind.y];
    Vertex v2 = vertices[nonuniformEXT(objId)].v[ind.z];

    const vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    // Computing the normal at hit position
    vec3 normal = v0.nrm * barycentrics.x + v1.nrm * barycentrics.y + v2.nrm * barycentrics.z;
    // Transforming the normal to world space
    normal = normalize(vec3(scnDesc.i[gl_InstanceCustomIndexEXT].transfoIT * vec4(normal, 0.0)));

    // Computing the coordinates of the hit position using vertex world positions
    vec3 worldPos = v0.pos * barycentrics.x + v1.pos * barycentrics.y + v2.pos * barycentrics.z;
    // Transforming the position to world space
    worldPos = vec3(scnDesc.i[gl_InstanceCustomIndexEXT].transfo * vec4(worldPos, 1.0));

    // Vector toward the light
    vec3  L;
    float lightIntensity = pushC.lightIntensity;
    float lightDistance  = 100000.0;
    // Point light
    if(pushC.lightType == 0)
    {
        vec3 lDir      = pushC.lightPosition - worldPos;
        lightDistance  = length(lDir);
        lightIntensity = pushC.lightIntensity / (lightDistance * lightDistance);
        L              = normalize(lDir);
    }
    else // Directional light
    {
        L = normalize(pushC.lightPosition - vec3(0));
    }

    // Material of the object
    int               matIdx = matIndex[nonuniformEXT(objId)].i[gl_PrimitiveID]; // ??? (rte): why nonuniformEXT() ?
    WaveFrontMaterial mat    = materials[nonuniformEXT(objId)].m[matIdx];

    // Diffuse
    vec3 diffuse = computeDiffuse(mat, L, normal);
    if(mat.textureId >= 0)
    {
        uint txtId = mat.textureId + scnDesc.i[gl_InstanceCustomIndexEXT].txtOffset;
        vec2 texCoord =
            v0.texCoord * barycentrics.x + v1.texCoord * barycentrics.y + v2.texCoord * barycentrics.z;
        diffuse *= texture(textureSamplers[nonuniformEXT(txtId)], texCoord).xyz;
    }
  
    // Specular
    vec3  specular    = vec3(0);
    float attenuation = 1;

    // Tracing shadow ray only if the light is visible from the surface
    if(dot(normal, L) > 0)
    {
        // calculate cone angle for soft shadow
        // https://medium.com/@alexander.wester/ray-tracing-soft-shadows-in-real-time-a53b836d123b

        // Calculate a vector perpendicular to L
        vec3 perpL = cross( L, vec3( 0, 1, 0 ) );
        // Handle case where L = up -> perpL should then be (1,0,0)
        if( perpL == vec3( 0 ) ) {
            perpL.x = 1.0;
        }
        // Use perpL to get a vector from worldPosition to the edge of the light sphere
        const float lightRadius = 20.0; // TODO: add to pushC
        vec3 toLightEdge = normalize( ( pushC.lightPosition + perpL * lightRadius ) - worldPos );
        // Angle between L and toLightEdge. Used as the cone angle when sampling shadow rays
        float coneAngle = acos( dot( L, toLightEdge ) ) * 2.0;

        float tMin   = 0.001;
        float tMax   = lightDistance;
        vec3  origin = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;
//        vec3  rayDir = L;

        float randSeed = float( clockARB() ) + worldPos.x + worldPos.y + worldPos.z;
        vec3 rayDir = getConeSample( randSeed, L, coneAngle );

        uint  flags = gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT | gl_RayFlagsSkipClosestHitShaderEXT;
        isShadowed = true;
        traceRayEXT(topLevelAS,  // acceleration structure
                flags,       // rayFlags
                0xFF,        // cullMask
                0,           // sbtRecordOffset
                0,           // sbtRecordStride
                1,           // missIndex
                origin,      // ray origin
                tMin,        // ray min range
                rayDir,      // ray direction
                tMax,        // ray max range
                1            // payload (location = 1)
        );
    }

    if(isShadowed)
    {
        attenuation = 0.3;
    }
    else
    {
        // Specular
        specular = computeSpecular(mat, gl_WorldRayDirectionEXT, L, normal);
    }

    prd.hitValue = vec3(lightIntensity * attenuation * (diffuse + specular));
}
