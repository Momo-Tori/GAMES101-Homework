//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
const float small=1e-5;

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    if (auto shot = intersect(ray); shot.happened)
        return shade(shot, ray.direction);
    else
        return Vector3f();

    // TODO: Implement Path Tracing Algorithm here
}

Vector3f Scene::shade(Intersection const &pInter, Vector3f wo) const
{
    auto p = pInter.coords;
    Intersection inter;
    float pdf_light;
    sampleLight(inter, pdf_light);
    auto ws = (inter.coords - p).normalized();
    Vector3f L_dir;
    if (float cos1 = dotProduct(-ws, inter.normal), cos2 = dotProduct(ws, pInter.normal); cos1 >= 0 && cos2 >= 0)
    {
        auto shot = intersect(Ray(p + ws * small, ws));
        if ((shot.coords - inter.coords).norm() < EPSILON)
        {
            L_dir = inter.emit * pInter.m->eval(wo, ws, pInter.normal) * cos1 * cos2 / pow((inter.coords - p).norm(), 2) / pdf_light;
        }
    }
    Vector3f L_indir;
    if (get_random_float() < RussianRoulette)
    {
        auto wi = pInter.m->sample(wo, pInter.normal);
        auto shot = intersect(Ray(p+ wi * small, wi));
        if (auto pdf=pInter.m->pdf(wo, wi, pInter.normal);pdf>=EPSILON&&shot.happened && shot.emit.norm() < EPSILON)
        {
            L_indir = shade(shot, wi) * pInter.m->eval(wo, wi, pInter.normal) * dotProduct(wi, pInter.normal) / pdf / RussianRoulette;
        }
    }
    Vector3f L_self = pInter.emit * dotProduct(-wo, pInter.normal);
    return L_dir + L_indir + L_self;
}

/*
shade(p, wo)
sampleLight(inter , pdf_light)
Get x, ws, NN, emit from inter
Shoot a ray from p to x
If the ray is not blocked in the middle
L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws,
NN) / |x-p|^2 / pdf_light
L_indir = 0.0
Test Russian Roulette with probability RussianRoulette
wi = sample(wo, N)
Trace a ray r(p, wi)
If ray r hit a non-emitting object at q
L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)
/ pdf(wo, wi, N) / RussianRoulette
Return L_dir + L_indir
*/