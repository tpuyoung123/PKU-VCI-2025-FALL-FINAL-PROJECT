#include "Labs/Final_Project/PathTracing.h"
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
    glm::vec4 pt_GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv      = glm::fract(uvCoord);
        uv.x              = uv.x * texture.GetSizeX() - .5f;
        uv.y              = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin  = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin  = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax  = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax  = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 pt_GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = pt_GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }

    bool pt_IntersectTriangle(pt_Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        // your code here
        auto  E1  = p2 - p1;
        auto  E2  = p3 - p1;
        auto  O   = ray.Origin;
        auto  D   = ray.Direction;
        auto  T   = O - p1;
        auto  P   = glm::cross(D, E2);
        auto  Q   = glm::cross(T, E1);
        float eps = -1e-6;
        float t   = glm::dot(Q, E2) / glm::dot(P, E1);
        float u   = glm::dot(P, T) / glm::dot(P, E1);
        float v   = glm::dot(Q, D) / glm::dot(P, E1);
        if (t > eps && u > eps && v > eps && 1 - u - v > eps) {
            output.t = t;
            output.u = u;
            output.v = v;
            return true;
        }
        return false;
    }

    glm::vec3 PathRayTrace(const PathIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 throughput(1.0f);

        for (int depth = 0; depth < maxDepth; ++depth) {
            auto rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;

            const glm::vec3 pos       = rayHit.IntersectPosition;
            const glm::vec3 n         = rayHit.IntersectNormal;
            const glm::vec3 kd        = rayHit.IntersectAlbedo;
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;
            const float     alpha     = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256.0f;

            glm::vec3 directLight(0.0f);

            for (const auto & light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation = 1.0f;
                float     distSq      = 0.0f;

                if (light.Type == Engine::LightType::Point) {
                    l           = light.Position - pos;
                    distSq      = glm::dot(l, l);
                    attenuation = 1.0f / distSq;
                    l           = glm::normalize(l);
                } else if (light.Type == Engine::LightType::Directional) {
                    l           = glm::normalize(light.Direction);
                    attenuation = 1.0f;
                    distSq      = FLT_MAX;
                }

                if (enableShadow && attenuation > 0.0f) {
                    Ray  shadowRay(pos + n * 1e-4f, l);
                    auto shadowHit = intersector.IntersectRay(shadowRay);

                    while (shadowHit.IntersectState && shadowHit.IntersectAlbedo.w < 0.2f) {
                        shadowRay = Ray(shadowHit.IntersectPosition + l * 1e-4f, l);
                        shadowHit = intersector.IntersectRay(shadowRay);
                    }

                    if (shadowHit.IntersectState) {
                        float hitDistSq = glm::dot(shadowHit.IntersectPosition - pos, shadowHit.IntersectPosition - pos);
                        if (light.Type == Engine::LightType::Point) {
                            if (hitDistSq < distSq) attenuation = 0.0f;
                        } else {
                            attenuation = 0.0f;
                        }
                    }
                }

                if (attenuation > 0.0f) {
                    glm::vec3 h    = glm::normalize(-ray.Direction + l);
                    float     spec = glm::pow(glm::max(glm::dot(h, n), 0.0f), shininess);
                    float     diff = glm::max(glm::dot(l, n), 0.0f);
                    directLight += light.Intensity * attenuation * (diff * kd + spec * ks);
                }
            }
            float maxIntensity = 10.0f;
            directLight        = glm::min(directLight, glm::vec3(maxIntensity));
            color += throughput * directLight;

            float specProbability = glm::max(ks.x, glm::max(ks.y, ks.z));
            float roll            = glm::linearRand(0.0f, 1.0f);

            if (alpha < 0.9f) {
                throughput *= kd;
                ray = Ray(pos - n * 1e-4f, ray.Direction);
            } else if (roll < specProbability) {
                glm::vec3 reflectDir = ray.Direction - 2.0f * glm::dot(n, ray.Direction) * n;
                throughput *= ks / specProbability;
                ray = Ray(pos + n * 1e-4f, glm::normalize(reflectDir));
            } else {
                float r1 = glm::linearRand(0.0f, 1.0f);
                float r2 = glm::linearRand(0.0f, 1.0f);

                glm::vec3 w = n;
                glm::vec3 a = (glm::abs(w.x) > 0.9f) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
                glm::vec3 v = glm::normalize(glm::cross(w, a));
                glm::vec3 u = glm::cross(w, v);

                float phi      = 2.0f * glm::pi<float>() * r1;
                float cosTheta = glm::sqrt(r2);
                float sinTheta = glm::sqrt(1.0f - r2);

                glm::vec3 localDir(sinTheta * glm::cos(phi), sinTheta * glm::sin(phi), cosTheta);

                glm::vec3 nextDir = glm::normalize(localDir.x * u + localDir.y * v + localDir.z * w);

                throughput *= kd / (1.0f - specProbability);
                ray = Ray(pos + n * 1e-4f, nextDir);
            }

            if (depth > 4) {
                float p = glm::max(throughput.x, glm::max(throughput.y, throughput.z));
                if (glm::linearRand(0.0f, 1.0f) > p) break;
                throughput /= p;
            }
        }

        return color;
    }
} // namespace VCX::Labs::Rendering
