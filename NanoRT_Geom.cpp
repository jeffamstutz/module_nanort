// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

// ospray
#include "NanoRT_Geom.h"
#include "ospray/common/Model.h"
#include "../include/ospray/ospray.h"
// embree
#include "embree2/rtcore.h"
#include "embree2/rtcore_scene.h"
#include "embree2/rtcore_geometry.h"

//NOTE(jda) - Use a copy of rtcore_ray.h which works with earlier version of
//            Embree (e.g. 2.7.1 packaged with OSPRay source)
#include "rtcore_ray.h"

#include "NanoRT_Geom_ispc.h"

namespace ospray {

template<typename T>
void getRay(const T& rays, RTCRay &ray, int i)
{
  ray.org[0] = rays.orgx[i];
  ray.org[1] = rays.orgy[i];
  ray.org[2] = rays.orgz[i];

  ray.dir[0] = rays.dirx[i];
  ray.dir[1] = rays.diry[i];
  ray.dir[2] = rays.dirz[i];

  ray.tnear  = rays.tnear[i];
  ray.tfar   = rays.tfar[i];

  ray.time   = rays.time[i];
  ray.mask   = rays.mask[i];

  ray.primID = rays.primID[i];
  ray.geomID = rays.geomID[i];
  ray.instID = rays.instID[i];
}

template<typename T>
void setRay(const RTCRay& ray, T &rays, int i)
{
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) {
    rays.Ngx[i] = ray.Ng[0];
    rays.Ngy[i] = ray.Ng[1];
    rays.Ngz[i] = ray.Ng[2];

    rays.primID[i] = ray.primID;
    rays.geomID[i] = ray.geomID;
    rays.instID[i] = ray.instID;
    rays.tfar[i]   = ray.tfar;
    rays.u[i]      = ray.u;
    rays.v[i]      = ray.v;
  }
}

static void nanortBoundsFunc(const NanoRT* nrts,
                             size_t        item,
                             RTCBounds*    bounds_o)
{
  const NanoRT& nrt = nrts[item];
  bounds_o->lower_x = nrt.bounds.lower.x;
  bounds_o->lower_y = nrt.bounds.lower.y;
  bounds_o->lower_z = nrt.bounds.lower.z;
  bounds_o->upper_x = nrt.bounds.upper.x;
  bounds_o->upper_y = nrt.bounds.upper.y;
  bounds_o->upper_z = nrt.bounds.upper.z;
}

static void traceRay(const NanoRT &nrt, RTCRay &_ray)
{
  // Setup ray
  nanort::Ray<float> ray;
  // fill ray org and ray dir.
  ray.org[0] = _ray.org[0];
  ray.org[1] = _ray.org[1];
  ray.org[2] = _ray.org[2];

  ray.dir[0] = _ray.dir[0];
  ray.dir[1] = _ray.dir[1];
  ray.dir[2] = _ray.dir[2];

  // fill minimum and maximum hit distance.
  ray.min_t = _ray.tnear;
  ray.max_t = _ray.tfar;

  nanort::TriangleIntersector<> triangle_intersecter(nrt.vertex,
                                                     nrt.uindex.get(),
                                                     sizeof(float)*nrt.vtxSize);

  // Trace ray
  nanort::BVHTraceOptions trace_options;
  bool hit = nrt.accel.Traverse(ray, trace_options, triangle_intersecter);

  // Save hit data
  if (hit) {
    _ray.tfar   = triangle_intersecter.intersection.t;
    _ray.primID = triangle_intersecter.intersection.prim_id;
    _ray.geomID = nrt.eMesh;

    _ray.u = triangle_intersecter.intersection.u;
    _ray.v = triangle_intersecter.intersection.v;

    // TODO: set normal...
    _ray.Ng[0]  = -ray.dir[0];
    _ray.Ng[1]  = -ray.dir[1];
    _ray.Ng[2]  = -ray.dir[2];
  }
}

static void nanortIntersectFunc(const NanoRT* nrts,
                                RTCRay&       ray,
                                size_t        item)
{
  const NanoRT& nrt = nrts[item];
  traceRay(nrt, ray);
}

static void nanortIntersectFuncN(const int*                 valid,
                                 const NanoRT*              nrts,
                                 const RTCIntersectContext* context,
                                 RTCRayNp*                  rays,
                                 size_t                     N,
                                 size_t                     item)
{
  UNUSED(context);
  const NanoRT& nrt = nrts[item];

  for (int i = 0; i < N; ++i) {
    if (valid[i]) {
      RTCRay ray;
      getRay(*rays, ray, i);
      traceRay(nrt, ray);
      setRay(ray, *rays, i);
    }
  }
}

static void nanortIntersectFunc1Mp(const NanoRT*              nrts,
                                   const RTCIntersectContext* context,
                                   RTCRay**                   rays,
                                   size_t                     M,
                                   size_t                     item)
{
  UNUSED(context);
  const NanoRT& nrt = nrts[item];

  for (size_t i = 0; i < M; ++i){
    traceRay(nrt, *rays[i]);
  }
}

template<int SIZE>
static void nanortIntersectFuncNt(const int*      mask,
                                  const NanoRT*   nrts,
                                  RTCRayNt<SIZE>& _rays,
                                  size_t          item)
{
  RTCIntersectContext ctx;
  RTCRayNp rays {_rays.orgx, _rays.orgy, _rays.orgz, _rays.dirx, _rays.diry,
                 _rays.dirz, _rays.tnear, _rays.tfar, _rays.time, _rays.mask,
                 _rays.Ngx, _rays.Ngy, _rays.Ngz, _rays.u, _rays.v,
                 _rays.geomID, _rays.primID, _rays.instID};
  nanortIntersectFuncN(mask, nrts, &ctx, &rays, SIZE, item);
}

// NanoRT definitions //////////////////////////////////////////////////

NanoRT::NanoRT()
  : eMesh(RTC_INVALID_GEOMETRY_ID)
{
  this->ispcMaterialPtrs = nullptr;
  this->ispcEquivalent = ispc::NanoRT_create(this);
}

std::string NanoRT::toString() const
{
  return "ospray::NanoRT";
}

void NanoRT::finalize(Model *model)
{
  Assert(model && "invalid model pointer");

  RTCScene embreeSceneHandle = model->embreeSceneHandle;

  vertexData = getParamData("vertex",getParamData("position"));
  normalData = getParamData("vertex.normal",getParamData("normal"));
  colorData  = getParamData("vertex.color",getParamData("color"));
  texcoordData = getParamData("vertex.texcoord",getParamData("texcoord"));
  indexData  = getParamData("index",getParamData("triangle"));
  prim_materialIDData = getParamData("prim.materialID");
  materialListData = getParamData("materialList");
  geom_materialID = getParam1i("geom.materialID",-1);

  this->index = (int*)indexData->data;
  this->vertex = (float*)vertexData->data;
  this->normal = normalData ? (float*)normalData->data : nullptr;
  this->color  = colorData ? (vec4f*)colorData->data : nullptr;
  this->texcoord = texcoordData ? (vec2f*)texcoordData->data : nullptr;
  this->prim_materialID =
      prim_materialIDData ? (uint32*)prim_materialIDData->data : nullptr;
  this->materialList =
      materialListData ? (ospray::Material**)materialListData->data : nullptr;

#if 0
  if (materialList && !ispcMaterialPtrs) {
    const int num_materials = materialListData->numItems;
    ispcMaterialPtrs = new void*[num_materials];
    for (int i = 0; i < num_materials; i++) {
      assert(this->materialList[i] != nullptr &&
             "Materials in list should never be nullptr");
      this->ispcMaterialPtrs[i] = this->materialList[i]->getIE();
    }
  }
#endif

  size_t numVerts = -1;
  switch (indexData->type) {
  case OSP_INT:
  case OSP_UINT:  numTris = indexData->size() / 3; idxSize = 3; break;
  case OSP_INT3:
  case OSP_UINT3: numTris = indexData->size(); idxSize = 3; break;
  case OSP_UINT4:
  case OSP_INT4:  numTris = indexData->size(); idxSize = 4; break;
  default:
    throw std::runtime_error("unsupported trianglemesh.index data type");
  }

  switch (vertexData->type) {
  case OSP_FLOAT:   numVerts = vertexData->size() / 4; vtxSize = 4; break;
  case OSP_FLOAT3:  numVerts = vertexData->size(); vtxSize = 3; break;
  case OSP_FLOAT3A: numVerts = vertexData->size(); vtxSize = 4; break;
  case OSP_FLOAT4 : numVerts = vertexData->size(); vtxSize = 4; break;
  default:
    throw std::runtime_error("unsupported trianglemesh.vertex data type");
  }
  if (normalData) switch (normalData->type) {
  case OSP_FLOAT3:  norSize = 3; break;
  case OSP_FLOAT:
  case OSP_FLOAT3A: norSize = 4; break;
  default:
    throw std::runtime_error("unsupported vertex.normal data type");
  }

  eMesh = rtcNewUserGeometry(embreeSceneHandle, 1);

  /////////////////////////////////////////////////////////////////////////////
  // Setup triangle data for NanoRT ///////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  //NOTE(jda) - Use an array of unsigned int for triangle indices (req'd by nrt)
  uindex = std::unique_ptr<unsigned int>(new unsigned int[numTris*3]);
  for (int i = 0; i < numTris; ++i) {
    auto *t = &index[idxSize*i];
    auto *uind = uindex.get();
    uind[3*i+0] = static_cast<unsigned int>(t[0]);
    uind[3*i+1] = static_cast<unsigned int>(t[1]);
    uind[3*i+2] = static_cast<unsigned int>(t[2]);
  }

  nanort::BVHBuildOptions<float> options; // Use default option

  triangle_mesh =
      make_unique<nanort::TriangleMesh<float>>(vertex, uindex.get(),
                                               sizeof(float) * vtxSize);

  nanort::TriangleSAHPred<float> triangle_pred(vertex, uindex.get(),
                                               sizeof(float) * vtxSize);
  auto ret = accel.Build(numTris, options, *triangle_mesh, triangle_pred);

  if (ret != true)
    throw std::runtime_error("NanoRT: Failed to build BVH!");

  /////////////////////////////////////////////////////////////////////////////
  // Finish Embree setup //////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  rtcSetUserData(embreeSceneHandle, eMesh, this);

  rtcSetBoundsFunction(embreeSceneHandle,
                       eMesh,
                       (RTCBoundsFunc)&nanortBoundsFunc);

#ifdef OSPRAY_USE_EMBREE_STREAMS
  rtcSetIntersectFunction1Mp(embreeSceneHandle,
                             eMesh,
                             (RTCIntersectFunc1Mp)&nanortIntersectFunc1Mp);

  rtcSetIntersectFunctionN(embreeSceneHandle,
                           eMesh,
                           (RTCIntersectFuncN)&nanortIntersectFuncN);

  rtcSetOccludedFunction1Mp(embreeSceneHandle,
                            eMesh,
                            (RTCOccludedFunc1Mp)&nanortIntersectFunc1Mp);

  rtcSetOccludedFunctionN(embreeSceneHandle,
                          eMesh,
                          (RTCOccludedFuncN)&nanortIntersectFuncN);
#else
  rtcSetIntersectFunction(embreeSceneHandle,
                          eMesh,
                          (RTCIntersectFunc)&nanortIntersectFunc);

  rtcSetIntersectFunction4(embreeSceneHandle,
                           eMesh,
                           (RTCIntersectFunc4)&nanortIntersectFuncNt<4>);

  rtcSetIntersectFunction8(embreeSceneHandle,
                           eMesh,
                           (RTCIntersectFunc8)&nanortIntersectFuncNt<8>);

  rtcSetIntersectFunction16(embreeSceneHandle,
                            eMesh,
                            (RTCIntersectFunc16)&nanortIntersectFuncNt<16>);

  rtcSetOccludedFunction(embreeSceneHandle,
                         eMesh,
                         (RTCOccludedFunc)&nanortIntersectFunc);

  rtcSetOccludedFunction4(embreeSceneHandle,
                          eMesh,
                          (RTCOccludedFunc4)&nanortIntersectFuncNt<4>);

  rtcSetOccludedFunction8(embreeSceneHandle,
                          eMesh,
                          (RTCOccludedFunc8)&nanortIntersectFuncNt<8>);

  rtcSetOccludedFunction16(embreeSceneHandle,
                           eMesh,
                           (RTCOccludedFunc16)&nanortIntersectFuncNt<16>);
#endif

  bounds = empty;

  for (size_t i = 0; i < numVerts*vtxSize; i += vtxSize)
    bounds.extend(*(vec3f*)(vertex + i));

  ispc::NanoRT_set(getIE(),
                   model->getIE(),
                   eMesh,
                   numTris,
                   idxSize,
                   vtxSize,
                   norSize,
                   (int*)index,
                   (float*)vertex,
                   (float*)normal,
                   (ispc::vec4f*)color,
                   (ispc::vec2f*)texcoord,
                   geom_materialID,
                   getMaterial()?getMaterial()->getIE() : nullptr,
                   ispcMaterialPtrs,
                   (uint32*)prim_materialID);
}

OSP_REGISTER_GEOMETRY(NanoRT, nrt);
OSP_REGISTER_GEOMETRY(NanoRT, nanort);

extern "C" void ospray_init_module_nanort()
{
  printf("Loaded plugin 'nanort' ...\n");
}

} // ::ospray
