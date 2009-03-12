#include <stdio.h>
#include <string.h>

#define MAX 64
#define START 2

//#define CG_ITER "WARNING[linear solver(CGLinearSolver)]: CGLinearSolver::solve, nbiter = %d"
#define CG_ITER "WARNING[default6(CGLinearSolver)]: CGLinearSolver::solve, nbiter = %d"

//#define CG_TIME "CGLinearSolver::solve, CG = %f"
#define CG_TIME "CGLinearSolver::solve, CG = %f bluid = %f"
#define PCG_ITER "PCGLinearSolver::solve, nbiter = %d"
#define PCG_TIME "PCGLinearSolver::solve, CG = %f preconditioner = %f build = %f invert = %f"

#define MAX_IT 500

int main(int argc, char ** argv) {
	FILE * files[MAX];
	int val[MAX];
	float tim[MAX][4];
	float tmp[MAX][4];
	int i,j,k,l;
	char * ret ;
	char filename[256];
	
	for (i=START;i<argc;i++) {
		sprintf(filename,"%s/%s",argv[1],argv[i]);
		files[i-START] = fopen(filename,"r");
		if (files[i-START] == NULL) {
			printf("Cant find %s\n",filename);
			return 1;
		}
	}

	sprintf(filename,"%s/%s",argv[1],"out-iter");
	FILE * out1 = fopen(filename,"w");
	//sprintf(filename,"%s/%s",argv[1],"out-time");
	//FILE * out2 = fopen(filename,"w");
	char buf[MAX][1024];
	tim[0][1] = tim[0][3] = 0.0;
	do {
		ret=fgets(buf[0],1024,files[0]);
		l=sscanf(buf[0],CG_TIME,&tim[0][0],&tim[0][2]);
	} while (l!=2 && ret!=NULL);	
	fgets(buf[0],1024,files[0]);
	sscanf(buf[0],CG_ITER,&val[0]);

	for (i=1;i<argc-START;i++) {
		do {
			ret=fgets(buf[i],1024,files[i]);
			l=sscanf(buf[i],PCG_TIME,&tim[i][0],&tim[i][1],&tim[i][2],&tim[i][3]);
		} while (l!=4 && ret!=NULL);
		fgets(buf[i],1024,files[i]);
		sscanf(buf[i],PCG_ITER,&val[i]);
	}

	k=1;
	fprintf(out1,"%d ",k);
	for (i=0;i<argc-START;i++) fprintf(out1,"%d ",val[i]);
	fprintf(out1,"\n");

        //fprintf(out2,"%d ",k);
	//for (i=0;i<argc-START;i++) fprintf(out2,"%f %f %f %f | ",tim1[i],tim2[i],tim3[i],tim4[i]);
	//fprintf(out2,"\n");
	
#ifdef MAX_IT
	while ((fgets(buf[0],1024,files[0])!=NULL) && (k<MAX_IT)) {
#else
	while (fgets(buf[0],1024,files[0])!=NULL) {
#endif
	  k++;
	  for (i=0;i<argc-START;i++) {
	  	tmp[i][0] = tmp[i][1] = tmp[i][2] = tmp[i][3] = 0.0;
	  	val[i] = 0;
	  }

	  sscanf(buf[0],CG_TIME,&tmp[0][0],&tmp[0][2]);
	  tim[0][0] += tmp[0][0];
	  tim[0][2] += tmp[0][2];
	  fgets(buf[0],1024,files[0]);
	  sscanf(buf[0],CG_ITER,&val[0]);

	  for (i=1;i<argc-START;i++) {
		fgets(buf[i],1024,files[i]);
		sscanf(buf[i],PCG_TIME,&tmp[i][0],&tmp[i][1],&tmp[i][2],&tmp[i][3]);
		tim[i][0] += tmp[i][0];
		tim[i][1] += tmp[i][1];
		tim[i][2] += tmp[i][2];
		tim[i][3] += tmp[i][3];
		fgets(buf[i],1024,files[i]);
		sscanf(buf[i],PCG_ITER,&val[i]);
	  }

	  fprintf(out1,"%d ",k);
	  for (i=0;i<argc-START;i++) fprintf(out1,"%d ",val[i]);
	  fprintf(out1,"\n");

	  //fprintf(out2,"%d ",k);
	  //for (i=0;i<argc-START;i++) fprintf(out2,"%f %f %f %f |",tim1[i],tim2[i],tim3[i],tim4[i]);
	  //fprintf(out2,"\n");
	}
	for (i=0;i<argc-START;i++) {
		tim[i][0] /= 1.0*k;
		tim[i][1] /= 1.0*k;
		tim[i][2] /= 1.0*k;
		tim[i][3] /= 1.0*k;
		if (strlen(argv[i+START])<6) 
			printf("%s :\t\tCG = %f\tpreconditioner = %f\tbuild = %f\tinvert = %f\n",argv[i+START],tim[i][0],tim[i][1],tim[i][2],tim[i][3]);
		else 
			printf("%s :\tCG = %f\tpreconditioner = %f\tbuild = %f\tinvert = %f\n",argv[i+START],tim[i][0],tim[i][1],tim[i][2],tim[i][3]);
	}
	//fprintf(out2,"\n");

	fclose(out1);
	//fclose(out2);
	
	for (i=0;i<argc-START;i++) fclose(files[i]);

	return 0;
}

 
