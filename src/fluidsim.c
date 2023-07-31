#include <MiniFB.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define NUM_X (150+2)
#define NUM_Y (75+2)

#define NUM_CELLS NUM_X*NUM_Y
static uint32_t  g_width  = NUM_Y;
static uint32_t  g_height = NUM_X;
//static uint32_t *g_buffer = 0x0;

uint32_t g_buffer [NUM_CELLS];

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void 
resize(struct mfb_window *window, int width, int height) {
    (void) window;
    g_width  = width;
    g_height = height;
    g_buffer = realloc(g_buffer, g_width * g_height * 4);
}
*/



#define OVER_RELAXATION 1.9
#define DENSITY 1000.0
#define H 0.01
//enum {
//    NUM_X=150+2,
//    NUM_Y=75+2,
//    float OVER_RELAXATION=1.9,
//    float DENSITY=1000.0,
//    NUM_CELLS=NUM_X*NUM_Y,
//    float H=0.01,
//};
float gU[NUM_CELLS];
float gV[NUM_CELLS];
float gNEW_U[NUM_CELLS];
float gNEW_V[NUM_CELLS];
float gP[NUM_CELLS];
float gS[NUM_CELLS];
float gM[NUM_CELLS];
float gNEW_M[NUM_CELLS];

const int WIDTH =NUM_Y;
const int HEIGHT=NUM_X;

float floor32(float a);

void setWallsAndIncoming(float u[], float s[], float m[]);

void set_obstacle(float x, float y, float s[],float u[],float v[],float m[]);

void integrate(float dt, float gravity, float s[],float v[]);

void solveIncompressibility(size_t numIters,
	float dt,float s[], float u[], float v[], float P[]);

void extrapolate(float u[], float v[]);

float max32(float x, float y);

float min32(float x, float y);

float sampleField(float xi, float yi, char field, float f[]);

float avgU(int i, int j, float u[]);

float avgV(int i, int j, float v[]);

void advectVel(float dt, float u[], float newU[], float v[], float newV[], float s[]);


void advectSmoke(float dt, float m[], float newM[], float s[], float u[], float v[]);


void simulate(float dt, float gravity, size_t numIters, float u[], float newU[], float v[], float newV[], float s[], float P[], float m[], float newM[]);



//float floor(int a){
//    return (float) ((int) a);
//}

//for (int i=0; i<NUM_CELLS, ++i){
//
    
//}
//
float floor32(float a){
    return (float) ((int)a);
}

int main(void) {

    for (int i=0; i< NUM_CELLS; ++i){
	gM[i]=1.0;
    }

    //printf(" %i\n", NUM_X);
    setWallsAndIncoming(gU, gS, gM);
    set_obstacle(0.4,0.35, gS,gU,gV, gM);
    //int i=0;


    //uint32_t  noise, carry, seed = 0xbeef;

    uint32_t  noise=0;
    struct mfb_window *window = mfb_open_ex("Noise Test", g_width, g_height, WF_RESIZABLE);
    if (!window)
        return 0;

    //g_buffer = (uint32_t *) malloc(g_width * g_height * 4);
    
    //mfb_set_resize_callback(window, resize);

    //mfb_set_viewport(window, 50, 50, g_width - 50 - 50, g_height - 50 - 50);
    //resize(window, g_width - 100, g_height - 100);  // to resize buffer

    mfb_update_state state;
    do {

        simulate(0.01, 0.0, 100, gU, gNEW_U, gV, gNEW_V, gS, gP, gM, gNEW_M);

        for (int index = 0; index <  NUM_CELLS; ++index) {
            /*
            noise = seed;
            noise >>= 3;
            noise ^= seed;
            carry = noise & 1;
            noise >>= 1;
            seed >>= 1;
            seed |= (carry << 30);
            noise &= 0xFF;
            */

            noise = (uint32_t) (255*gM[index]);
            g_buffer[index] = MFB_ARGB(0xff, noise, noise, noise);
            //g_buffer[index]=noise;
            //printf("%i \n", g_buffer[i]);
        }

        state = mfb_update_ex(window, &g_buffer, g_width, g_height);
        if (state != STATE_OK) {
            window = 0x0;
            break;
        }
        //while(true);
    } while(mfb_wait_sync(window));

    return 0;




}




void setWallsAndIncoming(float u[], float s[], float m[]){
    int n=NUM_Y;
    float inVel=2.0;
    for (int i=0; i<NUM_X;++i){
	for (int j=0; j<NUM_Y;++j){
	    float s_loc=1.0;
	    if (i==0 || j==0 || j==NUM_Y-1){
		s_loc=0.0;

	    }
	    s[i*n+j]=s_loc;
	    if (i<=1){
		u[i*n+j]=inVel;
	    }
	}
    }
    float pipeH=0.1* ((float) NUM_Y);
    int minJ=(int) floor32(0.5*((float)NUM_Y)-0.5*pipeH);
    int maxJ=(int) floor32(0.5*((float)NUM_Y)+0.5*pipeH);
    for (int i=0; i<2; ++i){
	for (int j=minJ; j<maxJ-1; ++j){
	    m[i*n+j]=0.0;
	}

    }
}


void set_obstacle(float x, float y, float s[],float u[],float v[],float m[]){
    float vx=0.0;
    float vy=0.0;
    float r=0.15;
    for (int i=1;i<NUM_X-1;++i){
	for (int j=1; j<NUM_Y-1;++j){
	    s[i*NUM_Y+j]=1.0;
	    float dx=(((float)i)+0.5)*H-x;
	    float dy=(((float)j)+0.5)*H-y;
	    if (dx*dx+dy*dy<r*r){
		s[i*NUM_Y+j]=0.0;
		m[i*NUM_Y+j]=1.0;
		u[i*NUM_Y+j]=vx;
		u[(i+1)*NUM_Y+j]=vx;
		v[i*NUM_Y+j]=vy;
		v[i*NUM_Y+j+1]=vy;

	    }

	}

    }


}


void integrate(float dt, float gravity, float s[],float v[]){
    int n=NUM_Y;
    for (int i=1; i<NUM_X; ++i){
	for (int j=1;j<NUM_Y;++j){
	    if(s[i*n+j]!=0.0 && s[i*n+j-1]!=0.0){
		v[i*n+j]+=gravity*dt;

	    }

	}

    }

}


void solveIncompressibility(size_t numIters,
	float dt,float s[], float u[], float v[], float P[]){
    int n=NUM_Y;
    float cp=DENSITY*H/dt;
    for (size_t iter=0;iter<numIters; ++iter){
	for (int i=1; i<NUM_X-1;++i){
	    for (int j=1; j<NUM_Y-1; ++j){
		if (s[i*n+j]==0.0){
		    continue;
		}
		float sx0=s[(i-1)*n+j];
		float sx1=s[(i+1)*n+j];
		float sy0=s[i*n+j-1];
		float sy1=s[i*n+j+1];
		float s_local=sx0+sx1+sy0+sy1;
		if (s_local==0.0) {
		    continue;
		}
		float div=u[(i+1)*n+j]-u[i*n+j]+v[i*n+j+1]-v[i*n+j];
		float p=-div/s_local;
		p *=OVER_RELAXATION;
		P[i*n+j]+=cp*p;
		u[i*n+j]-=sx0*p;
		u[(i+1)*n+j]+=sx1*p;
		v[i*n+j]-=sy0*p;
		v[i*n+j+1]+=sy1*p;


	    }
	}

    }
}

void extrapolate(float u[], float v[]){
    int n=NUM_Y;
    for (int i=0; i<NUM_X;++i){
	u[i*n]=u[i*n+1];
	u[i*n+NUM_Y-1]=u[i*n+NUM_Y-2];
    }
    for (int j=0; j<NUM_Y;++j){
	v[0*n+j]=v[1*n+j];
	v[(NUM_X-1)*n+j]=v[(NUM_X-2)*n+j];

    }
}


float max32(float x, float y){
    if (x>y){
	return x;
    }else{
	return y;
    }
}
float min32(float x, float y){
    if (y>x){
	return x;
    }else{
	return y;
    }
}

float sampleField(float xi, float yi, char field, float f[]){
    int n=NUM_Y;
    float h=H;
    float h1=1.0/h;
    float h2=0.5*h;
    float x=max32(min32(xi, ((float)NUM_X)),h);
    float y=max32(min32(yi,(float)NUM_Y),h);
    float dx=0.0;
    float dy=0.0;
    switch (field) {
	case 'u' :
	    dy=h2;
	    break;
	
	case 'v':
	    dx=h2;
	    break;

	
	case 'm':
	    dx=h2;
	    dy=h2;
	    break;
	
	default:
	    printf("invalid field");
	
    }
    float x0=min32(floor32((x-dx)*h1),(float) (NUM_X-1));
    float tx=((x-dx)-x0*h)*h1;
    float x1=min32(x0+1.0,(float)(NUM_X-1));
    float y0=min32(floor32((y-dy)*h1),(float) (NUM_Y-1));
    float ty=((y-dy)-y0*h)*h1;
    float y1=min32(y0+1.0,(float)(NUM_Y-1))+0.5;
    float sx=1.0-tx;
    float sy=1.0-ty;
    float val=sx*sy*f[(size_t) (x0*((float)n)+y0)]
	+tx*sy*f[(size_t) (x1*((float)n) +y0 )]
	+tx*ty*f[(size_t)( x1*((float)n )+y1  )]
	+sx*ty*f[(size_t) (x0*((float)n)+y1)];
    return val;


}


float avgU(int i, int j, float u[]){
    int n=NUM_Y;
    return (u[i*n+j-1]+u[i*n+j]+u[(i+1)*n+j-1])*0.25;

}

float avgV(int i, int j, float v[]){
    int n=NUM_Y;
    return (v[(i-1)*n+j]+v[i*n+j]+v[(i-1)*n+j+1]+v[i*n+j+1])*0.25;
}

void advectVel(float dt, float u[], float newU[], float v[], float newV[], float s[]){
    for (int i=0; i<NUM_CELLS; ++i){
	newU[i]=u[i];
    }

    for (int i=0; i<NUM_CELLS; ++i){
	newV[i]=v[i];
    }
    int n=NUM_Y;
    float h=H;
    float h2=0.5*h;
    for (int i=1; i<NUM_X-1; ++i){
	for (int j=1; j<NUM_Y-1;++j){
	    if(s[i*n+j]!=0.0 && s[(i-1)*n+j]!=0.0 &&j<(NUM_Y-1)){
		float x=((float)i)*h;
		float y=((float)j)*h+h2;
		float u_local=u[i*n+j];
		float v_local=avgV(i,j,v);
		x=x-dt*u_local;
		y=y-dt*v_local;
		u_local=sampleField(x,y,'u', u);
		newU[i*n+j]=u_local;

	    }
	    if (s[i*n+j]!=0.0 && s[i*n+j-1]!=0.0 && j <NUM_X-1){
		float x=((float)i)*h+h2;
		float y=((float)j)*h;
		float u_local2=avgU(i,j,u);
		float v_local2=v[i*n+j];
		x=x-dt*u_local2;
		y=y-dt*v_local2;
		v_local2=sampleField(x,y,'v',v);
		newV[i*n+j]=v_local2;

	    }
	}
    }
    for (int i=0; i<NUM_CELLS; ++i){
	u[i]=newU[i];
    }
    for (int i=0; i<NUM_CELLS; ++i){
	v[i]=newV[i];
    }

}

void advectSmoke(float dt, float m[], float newM[], float s[], float u[], float v[]){
    for (int i=0; i<NUM_CELLS; ++i){
	newM[i]=m[i];
    }
    int n=NUM_Y;
    float h=H;
    float h2=0.5*h;
    for (int i=1; i<(NUM_X-1);++i){
	for (int j=1; j<(NUM_Y-1);++j){
	    if (s[i*n+j]!=0.0) {
		float u_local=(u[i*n+j]+u[(i+1)*n+j])*0.5;
		float v_local=(v[i*n+j]+v[i*n+j+1])*0.5;
		float x=((float)i)*h+h2-dt*u_local;
		float y=((float)j)*h+h2-dt*v_local;
		newM[i*n+j]=sampleField(x,y,'m', m);
	    }

	}
    }

    for (int i=0; i<NUM_CELLS;++i){
	m[i]=newM[i];
    }
}

void simulate(float dt, float gravity, size_t numIters, float u[], float newU[], float v[], float newV[], float s[], float P[], float m[], float newM[]){
    for (int i=0; i<NUM_CELLS; ++i){
	P[i]=0.0;
    }
    solveIncompressibility(numIters, dt,s,u,v,P);
    extrapolate(u,v);
    advectVel(dt,u,newU,v,newV,s);
    advectSmoke(dt,m,newM,s,u,v);
    //return u[0];

}

