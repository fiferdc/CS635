#
#include "defs.h"
#include <stdio.h>

freeinit(fl, size)
struct	Freelist *fl;
int	size;
{
fl -> head = (struct Freenode *) NULL;
fl -> nodesize = size;
}


freefree(fl)
struct	Freelist *fl;
{
}


char *getfree(fl)
struct	Freelist *fl;
{
int i; struct Freenode *t;
if(fl->head == (struct Freenode *) NULL)
{
 	t =  (struct Freenode *) myalloc(sqrt_nsites * fl->nodesize);
	for(i=0; i<sqrt_nsites; i+=1) 	
		makefree((struct Freenode *)((char *)t+i*fl->nodesize), fl);
};
t = fl -> head;
fl -> head = (fl -> head) -> nextfree;
return((char *)t);
}



makefree(curr,fl)
struct Freenode *curr;
struct Freelist *fl;
{
curr -> nextfree = fl -> head;
fl -> head = curr;

return (1);
}

static int total_alloc=0;
static char **ptr=NULL;
static int maxalloc=0;
static int numalloc=0;

char *myalloc(n)
unsigned n;
{
char *t;
if ((t=malloc(n)) == (char *) 0)
{  fprintf(stderr,"Insufficient memory processing site %d (%d bytes in use)\n",
		siteidx, total_alloc);
     exit(1);
};

if (numalloc == maxalloc) {
   if (!ptr)
      maxalloc = 256;
   else
      maxalloc = numalloc * 2;
   if (ptr)
      ptr = (char **)realloc(ptr, sizeof(char *)*maxalloc);
   else
      ptr = (char **)calloc(sizeof(char *), maxalloc);
}

ptr[numalloc] = t;
numalloc++;

total_alloc += n;
return(t);
}


void freemyalloc()
{
   int i;

   for (i=0; i<numalloc; i++) {
      free(ptr[i]);
      ptr[i] = NULL;
   }
   numalloc = 0;
}
