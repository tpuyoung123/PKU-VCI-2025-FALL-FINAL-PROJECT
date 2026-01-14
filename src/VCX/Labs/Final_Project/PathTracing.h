#include "Engine/Scene.h"
#include "Labs/Final_Project/Ray.h"
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/random.hpp>
#include <numeric>
#include <queue>
#include <random>
#include <spdlog/spdlog.h>
#include <vector>

namespace VCX::Labs::Rendering {

    constexpr float pt_EPS1 = 1e-2f; // distance to prevent self-intersection
    constexpr float pt_EPS2 = 1e-8f; // angle for parallel judgement
    constexpr float pt_EPS3 = 1e-4f; // relative distance to enlarge kdtree

    glm::vec4 pt_GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord);

    glm::vec4 pt_GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord);

    struct pt_Intersection {
        float t, u, v; // ray parameter t, barycentric coordinates (u, v)
    };

    bool pt_IntersectTriangle(pt_Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3);

    struct pt_RayHit {
        bool              IntersectState;
        Engine::BlendMode IntersectMode;
        glm::vec3         IntersectPosition;
        glm::vec3         IntersectNormal;
        glm::vec4         IntersectAlbedo;   // [Albedo   (vec3), Alpha     (float)]
        glm::vec4         IntersectMetaSpec; // [Specular (vec3), Shininess (float)]
    };

    struct pt_TrivialRayIntersector {
        Engine::Scene const * InternalScene = nullptr;

        pt_TrivialRayIntersector() = default;

        void InitScene(Engine::Scene const * scene) {
            InternalScene = scene;
        }

        pt_RayHit IntersectRay(Ray const & ray) const {
            pt_RayHit result;
            if (! InternalScene) {
                spdlog::warn("VCX::Labs::Rendering::RayIntersector::IntersectRay(..): uninitialized intersector.");
                result.IntersectState = false;
                return result;
            }
            int             modelIdx, meshIdx;
            pt_Intersection its;
            float           tmin     = 1e7, umin, vmin;
            int             maxmodel = InternalScene->Models.size();
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                for (int j = 0; j < maxidx; j += 3) {
                    std::uint32_t const * face = model.Mesh.Indices.data() + j;
                    glm::vec3 const &     p1   = model.Mesh.Positions[face[0]];
                    glm::vec3 const &     p2   = model.Mesh.Positions[face[1]];
                    glm::vec3 const &     p3   = model.Mesh.Positions[face[2]];
                    if (! pt_IntersectTriangle(its, ray, p1, p2, p3)) continue;
                    if (its.t < pt_EPS1 || its.t > tmin) continue;
                    tmin = its.t, umin = its.u, vmin = its.v, modelIdx = i, meshIdx = j;
                }
            }
            if (tmin == 1e7) {
                result.IntersectState = false;
                return result;
            }
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            result.IntersectAlbedo          = pt_GetAlbedo(material, uvCoord);
            result.IntersectMetaSpec        = pt_GetTexture(material.MetaSpec, uvCoord);

            return result;
        }
    };

    // using PathIntersector = pt_TrivialRayIntersector;

    class AABB {
    public:
        glm::vec3 Min = glm::vec3(FLT_MAX);
        glm::vec3 Max = glm::vec3(-FLT_MAX);

        AABB() = default;
        AABB(glm::vec3 const & p): Min(p), Max(p) {}

        void Extend(glm::vec3 const & p) {
            Min = glm::min(Min, p);
            Max = glm::max(Max, p);
        }

        bool Intersect(Ray const & ray, float & tmin_out, float & tmax_out) {
            glm::vec3 invD = 1.0f / (ray.Direction + glm::vec3(1e-9f));
            glm::vec3 t0   = (Min - ray.Origin) * invD;
            glm::vec3 t1   = (Max - ray.Origin) * invD;
            glm::vec3 Near = glm::min(t0, t1);
            glm::vec3 Far  = glm::max(t0, t1);

            float tmin = glm::max(glm::max(Near.x, Near.y), Near.z);
            float tmax = glm::min(glm::min(Far.x, Far.y), Far.z);

            tmin_out = tmin;
            tmax_out = tmax;

            return tmax >= glm::max(0.0f, tmin);
        }
    };

    class KDTree {
    public:
        struct KDTreeNode {
            AABB             Bounds;
            KDTreeNode *     Left  = nullptr;
            KDTreeNode *     Right = nullptr;
            std::vector<int> TraingleIndices;
            bool             IsLeaf = false;
            ~KDTreeNode() {
                delete Left;
                delete Right;
            }
        };

        KDTreeNode * Root = nullptr;

        ~KDTree() { delete Root; }

        KDTreeNode * BuildKDTree(const std::vector<int> & indices, const std::vector<AABB> & itemBounds, int depth) {
            KDTreeNode * node = new KDTreeNode();

            for (int idx : indices) {
                node->Bounds.Extend(itemBounds[idx].Min);
                node->Bounds.Extend(itemBounds[idx].Max);
            }

            if (indices.size() <= 4 || depth >= 15) {
                node->IsLeaf          = true;
                node->TraingleIndices = indices;
                return node;
            }

            int axis = depth % 3;

            std::vector<int> sortedIndices = indices;
            std::sort(sortedIndices.begin(), sortedIndices.end(), [&](int a, int b) {
                return (itemBounds[a].Min[axis] + itemBounds[a].Max[axis]) < (itemBounds[b].Min[axis] + itemBounds[b].Max[axis]);
            });

            int mid     = sortedIndices.size() / 2;
            node->Left  = BuildKDTree({ sortedIndices.begin(), sortedIndices.begin() + mid }, itemBounds, depth + 1);
            node->Right = BuildKDTree({ sortedIndices.begin() + mid, sortedIndices.end() }, itemBounds, depth + 1);

            return node;
        }
    };

    struct PQNode {
        KDTree::KDTreeNode * node;
        float                t_near;
        bool                 operator>(const PQNode & other) const {
            return t_near > other.t_near;
        }
    };

    struct PathKDTreeRayIntersector {
        Engine::Scene const * InternalScene = nullptr;
        KDTree                Tree;

        struct TriangleRef {
            int modelIdx;
            int meshIdx;
        };
        std::vector<TriangleRef> TriangleRefs;

        PathKDTreeRayIntersector() = default;

        void InitScene(Engine::Scene const * scene) {
            InternalScene = scene;

            std::vector<AABB> itemBounds;
            TriangleRefs.clear();

            int maxmodel = InternalScene->Models.size();
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                for (int j = 0; j < maxidx; j += 3) {
                    std::uint32_t const * face = model.Mesh.Indices.data() + j;
                    glm::vec3 const &     p1   = model.Mesh.Positions[face[0]];
                    glm::vec3 const &     p2   = model.Mesh.Positions[face[1]];
                    glm::vec3 const &     p3   = model.Mesh.Positions[face[2]];

                    AABB triAABB(p1);
                    triAABB.Extend(p2);
                    triAABB.Extend(p3);

                    itemBounds.push_back(triAABB);
                    TriangleRefs.push_back({ i, j });
                }
            }

            std::vector<int> allIndices(TriangleRefs.size());
            std::iota(allIndices.begin(), allIndices.end(), 0);

            if (Tree.Root) {
                delete Tree.Root;
                Tree.Root = nullptr;
            }
            Tree.Root = Tree.BuildKDTree(allIndices, itemBounds, 0);
        }

        bool Traverse(KDTree::KDTreeNode * node, Ray const & ray, float & t_min, pt_Intersection & best_its, int & best_refIdx) const {
            if (! node) return false;

            float t_node_near, t_node_far;

            if (! node->Bounds.Intersect(ray, t_node_near, t_node_far)) return false;
            if (t_node_near > t_min) return false;

            bool hit = false;

            if (node->IsLeaf) {
                for (int refIdx : node->TraingleIndices) {
                    auto const &          ref   = TriangleRefs[refIdx];
                    auto const &          model = InternalScene->Models[ref.modelIdx];
                    std::uint32_t const * face  = model.Mesh.Indices.data() + ref.meshIdx;

                    pt_Intersection   its;
                    glm::vec3 const & p1 = model.Mesh.Positions[face[0]];
                    glm::vec3 const & p2 = model.Mesh.Positions[face[1]];
                    glm::vec3 const & p3 = model.Mesh.Positions[face[2]];

                    if (pt_IntersectTriangle(its, ray, p1, p2, p3)) {
                        if (its.t > 1e-4f && its.t < t_min) {
                            t_min       = its.t;
                            best_its    = its;
                            best_refIdx = refIdx;
                            hit         = true;
                        }
                    }
                }
                return hit;
            }

            KDTree::KDTreeNode * Near = node->Left;
            KDTree::KDTreeNode * Far  = node->Right;

            float t_near_min, t_near_max, t_far_min, t_far_max;
            bool  hitNearBox = Near ? Near->Bounds.Intersect(ray, t_near_min, t_near_max) : false;
            bool  hitFarBox  = Far ? Far->Bounds.Intersect(ray, t_far_min, t_far_max) : false;

            if (hitNearBox && hitFarBox) {
                if (t_near_min > t_far_min) {
                    std::swap(Near, Far);
                    std::swap(t_near_min, t_far_min);
                }

                hit |= Traverse(Near, ray, t_min, best_its, best_refIdx);

                if (t_far_min < t_min) {
                    hit |= Traverse(Far, ray, t_min, best_its, best_refIdx);
                }
            } else if (hitNearBox) {
                hit |= Traverse(Near, ray, t_min, best_its, best_refIdx);
            } else if (hitFarBox) {
                hit |= Traverse(Far, ray, t_min, best_its, best_refIdx);
            }

            return hit;
        }

        pt_RayHit IntersectRay(Ray const & ray) const {
            pt_RayHit result;
            result.IntersectState = false;

            if (! InternalScene || ! Tree.Root) {
                return result;
            }

            float           tmin = 1e30f;
            pt_Intersection bestIts;
            int             bestRefIdx = -1;

            if (Traverse(Tree.Root, ray, tmin, bestIts, bestRefIdx)) {
                auto const & ref       = TriangleRefs[bestRefIdx];
                auto const & model     = InternalScene->Models[ref.modelIdx];
                auto const & normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
                auto const & texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();

                std::uint32_t const * face = model.Mesh.Indices.data() + ref.meshIdx;
                float                 umin = bestIts.u;
                float                 vmin = bestIts.v;
                float                 w    = 1.0f - umin - vmin;

                result.IntersectState    = true;
                auto const & material    = InternalScene->Materials[model.MaterialIndex];
                result.IntersectMode     = material.Blend;
                result.IntersectPosition = w * model.Mesh.Positions[face[0]] + umin * model.Mesh.Positions[face[1]] + vmin * model.Mesh.Positions[face[2]];
                result.IntersectNormal   = glm::normalize(w * normals[face[0]] + umin * normals[face[1]] + vmin * normals[face[2]]);

                glm::vec2 uvCoord        = w * texcoords[face[0]] + umin * texcoords[face[1]] + vmin * texcoords[face[2]];
                result.IntersectAlbedo   = pt_GetAlbedo(material, uvCoord);
                result.IntersectMetaSpec = pt_GetTexture(material.MetaSpec, uvCoord);
            }

            return result;
        }
    };

    using PathIntersector = PathKDTreeRayIntersector;
    // using PathIntersector = pt_TrivialRayIntersector;

    glm::vec3 PathRayTrace(const PathIntersector & intersector, Ray ray, int maxDepth, bool enableShadow);
} // namespace VCX::Labs::Rendering
