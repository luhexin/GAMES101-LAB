//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
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
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
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
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = Scene::intersect(ray);
    if(intersection.happened) {
        Material *m = intersection.m;//Matiral
        Vector3f L_dir(0.0), L_indir(0.0);

        // sampleLight(inter, pdf_light)
        Intersection inter;
        float pdf_light;
        sampleLight(inter, pdf_light);
        
        // Get x, ws, NN, emit from inter
        Vector3f X = inter.coords;
        Vector3f P = intersection.coords;
        Vector3f ws = (X - P).normalized();
        Vector3f N = intersection.normal; 
        Vector3f NN = inter.normal;
        Vector3f emit = inter.emit;

        // Shoot a ray from p to x(inter)
        Ray ray_p_x(P + EPSILON * N, ws);
        
        Intersection intersection_p_x = Scene::intersect(ray_p_x);
     
        // If the ray is not blocked in the middle
        if((intersection_p_x.coords - inter.coords).norm() < EPSILON) {
            L_dir = emit * m->eval(ray.direction, ws, N) 
                    * dotProduct(ws, N) 
                    * dotProduct(-ws, NN) 
                    / std::pow(intersection_p_x.distance,2)
                    / pdf_light;
        }

        // 使用俄罗斯轮盘赌判断反射光线是否应停止继续反射
        if(get_random_float() <= RussianRoulette) {
            // Trace a ray r(p, wi)
            Vector3f wi = m->sample(ray.direction, N).normalized();
            Ray r(P, wi);
            Intersection intersection_r = Scene::intersect(r);
            
            // If ray r hit a non-emitting object at q
            if(intersection_r.happened && !intersection_r.m->hasEmission()) {
                L_indir = castRay(r, depth+1) 
                          * m->eval(ray.direction, wi, N) 
                          * dotProduct(wi, N) 
                          / m->pdf(ray.direction, wi, N) 
                          / RussianRoulette;
            }
        }
        return m->getEmission() + L_dir + L_indir;
    } 
    return this->backgroundColor;
}