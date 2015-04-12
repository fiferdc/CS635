#include <stdio.h>
#include "defs.h"
struct Site *readone();
struct Site *nextone();

int triangulate, sorted, plot, debug;
float xmin, xmax, ymin, ymax, deltax, deltay;



#ifdef LIBRARY

static float *points = NULL;
static int npoints = 0;
int *outtris = NULL;
int ntris = 0;
int maxtris = 0;

void 
delaunay_triangulation(float *_points, int _npoints, 
                       int **_outtris, int *_ntris)
{
int c;
struct Site *(*next)();

points = _points;
npoints = _npoints;
ntris = 0;
maxtris = 512;
outtris = (int *)calloc(maxtris, 3*sizeof(int));
sorted = 0; triangulate = 1; plot = 0; debug = 0;

#else

main(argc,argv) 
char **argv; 
int argc;
{	

int c, narg;
struct Site *(*next)();

sorted = 0; triangulate = 0; plot = 0; debug = 0;
narg = 1;
while(narg < argc) {
   c = argv[narg][1];
	switch(c) {
	case 'd': debug = 1;
		  break;
	case 's': sorted = 1;
		  break;
	case 't': triangulate = 1;
		  break;
	case 'p': plot = 1;
		  break;
		  };
   narg++;
}
#endif

freeinit(&sfl, sizeof *sites);
if(sorted)
{	scanf("%d %f %f %f %f", &nsites, &xmin, &xmax, &ymin, &ymax);
	next = readone;
}
else 
{	readsites();
	next = nextone;
};

siteidx = 0;
geominit();
if(plot) plotinit();

voronoi(triangulate, next); 

#ifdef LIBRARY
*_outtris = outtris;
*_ntris = ntris;
outtris = NULL;
free(sites);
freemyalloc(); /* free free lists */
#else
exit(0);
#endif
}

/* sort sites on y, then x, coord */
int scomp(s1,s2)
struct Point *s1,*s2;
{
	if(s1 -> y < s2 -> y) return(-1);
	if(s1 -> y > s2 -> y) return(1);
	if(s1 -> x < s2 -> x) return(-1);
	if(s1 -> x > s2 -> x) return(1);
	return(0);
}

/* return a single in-storage site */
struct Site *nextone()
{
struct Site *s;
for (;siteidx<nsites; siteidx+= 1)
{	if (siteidx==0 || sites[siteidx].coord.x!=sites[siteidx-1].coord.x 
		       || sites[siteidx].coord.y!=sites[siteidx-1].coord.y)
	{	siteidx += 1;
		return (&sites[siteidx-1]);
	};
};
return( (struct Site *)NULL);
}


/* read all sites, sort, and compute xmin, xmax, ymin, ymax */
readsites()
{
int i;

nsites=0;
sites = (struct Site *) calloc(4000,sizeof *sites);
#ifdef LIBRARY
while (nsites < npoints) 
{
   sites[nsites].coord.x = points[nsites*2+0];
   sites[nsites].coord.y = points[nsites*2+1];
#else
while(scanf("%f %f", &sites[nsites].coord.x, &sites[nsites].coord.y)!=EOF)
{	
#endif
   sites[nsites].sitenbr = nsites;
	sites[nsites].refcnt = 0;
	nsites += 1;
	if (nsites % 4000 == 0)
		sites = (struct Site *) realloc(sites,(nsites+4000)*sizeof*sites);
};
qsort(sites, nsites, sizeof *sites, scomp);
xmin=sites[0].coord.x; 
xmax=sites[0].coord.x;
for(i=1; i<nsites; i+=1)
{	if(sites[i].coord.x < xmin) xmin = sites[i].coord.x;
	if(sites[i].coord.x > xmax) xmax = sites[i].coord.x;
};
ymin = sites[0].coord.y;
ymax = sites[nsites-1].coord.y;

return (1);
}

/* read one site */
struct Site *readone()
{
struct Site *s;

s = (struct Site *) getfree(&sfl);
s -> refcnt = 0;
s -> sitenbr = siteidx;
siteidx += 1;
if(scanf("%f %f", &(s->coord.x), &(s->coord.y)) == EOF)
	return ((struct Site *) NULL );
return(s);
}
