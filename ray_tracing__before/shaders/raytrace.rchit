#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"
#include "wavefront.glsl"

hitAttributeEXT vec3 attribs;

layout(location = 0) rayPayloadInEXT hitPayload prd;

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
    vec3 specular = computeSpecular(mat, gl_WorldRayDirectionEXT, L, normal);

    prd.hitValue = vec3(lightIntensity * (diffuse + specular));
}
