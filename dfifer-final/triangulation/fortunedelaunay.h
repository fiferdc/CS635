#ifndef FORTUNEDELAUNAY_ALREADY_INCLUDED
#define FORTUNEDELAUNAY_ALREADY_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

void delaunay_triangulation(float *points, int n, int **tris, int *ntris);

#ifdef __cplusplus
};
#endif

#endif
