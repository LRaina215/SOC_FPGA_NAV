/* Batch the public mj_ray function without changing its intersection rules.
 * No mjModel layout assumptions; function pointer is from installed MuJoCo.
 * mj_multiRay3.3.7 angular culling missed a raised-box hit in this study.
 */
#include <stdint.h>
typedef double (*ray_fn)(const void*,const void*,const double*,const double*,
                        const unsigned char*,unsigned char,int,int*);
__declspec(dllexport) void terrain_rays(ray_fn ray,const void* model,const void* data,
    const double* origin,const double* vectors,int count,const unsigned char* groups,
    double* distances,int* geoms) {
    for(int i=0;i<count;++i)
        distances[i]=ray(model,data,origin,vectors+3*i,groups,1,-1,geoms+i);
}
