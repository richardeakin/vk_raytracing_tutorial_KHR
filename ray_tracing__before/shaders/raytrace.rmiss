#version 460
#extension GL_EXT_ray_tracing : require

//layout(location = 0) rayPayloadInEXT vec3 hitValue;
//
//void main()
//{
//    hitValue = vec3(0.7, 0.2, 0.1);
//}
//
#extension GL_GOOGLE_include_directive : enable
#include "raycommon.glsl"

layout(location = 0) rayPayloadInEXT hitPayload prd;

layout(push_constant) uniform Constants
{
  vec4 clearColor;
};

void main()
{
  prd.hitValue = clearColor.xyz;
}