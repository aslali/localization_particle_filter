
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/led.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>     
#include <time.h>      
#include <math.h>
#include <float.h>
#include <webots/display.h>



#define TIME_STEP 64
#define max(a,b) (((a)>(b))?(a):(b))
#define min(a,b) (((a)<(b))?(a):(b))
#define pi 3.1415926535897932384626433832795
#define ee     2.7183
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23

float line(float z,float x1,float x2,float y1,float y2){
  return (z-x2)*(y2-y1)/(x2-x1)+y2;
}
double generateGaussianNoise1(double mu, double sigma);
double generateGaussianNoise(double mu, double sigma);
void rayCasting(int map[176][226],int map1[176][226],double p[3] ,double z[8],const double theta[8],const double rE,double maxRange);
int checkObstacle(int map1[176][226],double pTemp[2]);
double arrayMax(double *arr,int len);
void aaarrayMax(double *arr,int len,double maxArr[2]);

double dlr;
double da;
//void wb differential wheels set encoders (double left, double right);
static void compute_odometry() {
  double l = wb_differential_wheels_get_left_encoder();
  double r = wb_differential_wheels_get_right_encoder();
  double dl = l / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by left wheel in meter
  double dr = r / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by right wheel in meter
  da = (dr - dl) / AXLE_LENGTH; // delta orientation
  dlr=(dl+dr)/2;
}

 
 
int main(int argc, char **argv)
{


  /* necessary to initialize webots stuff */
  wb_robot_init();
  WbDeviceTag distance_sensor[8],led[10];
  int i;int j;
  double roomSize[2]={1.13,0.88};
  int nP=10000;
  double particle[nP][3];
  double tempParticle[nP][3];
  //double pos[400];
  double zs[8];
  double ds[8];
  double z[8]={1, 2 ,3 ,4, 5, 6, 7, 8};
  double Wl;
  double Wr;
  int map[176][226];
  int map1[176][226];
  double diff;
  double wp[nP];
  double sumW;
  int k;
  double weight;
  double maxInfo[2];
  double maxWp;

  
  const double thetaS[8]= {5.9614,5.4340,4.7124,3.6052,2.6779,1.5708,0.8491,0.3217};

  const double rE= .035;
  const double maxRange=0.05;
  const double a1=0.0014*30;
  const double a2=0.0873*30;
  const double a3=0.0047*30;
  const double a4=0;

  
  FILE* mapFile= fopen("map.txt","r");
  for (i=175;i>=0;i--){
    for (j=0;j<226;j++){
      fscanf(mapFile,"%d",&map1[i][j]);
    }
  }
  fclose(mapFile);
  
  
  FILE* mapFile1= fopen("map.txt","r");
  for (i=0;i<176;i++){
    for (j=0;j<226;j++){
      fscanf(mapFile1,"%d",&map[i][j]);
    }
  }
  fclose(mapFile1);
  
  
  
  int World0[176][226];
	for(i = 0; i < 176 ; i++){
		for(j = 0 ; j < 226 ; j++){
			World0[i][j] = map1[i][j];
      }
    }

	int World1[176][226];

	//int k;
	for (k = 0 ; k < 7 ; k++)
	{
		for(i = 1 ; i < 175 ; i++)
		{
			for(j = 1 ; j < 225 ; j++)
				World1[i][j] = World0[i][j] + World0[i+1][j] + World0[i][j+1] + World0[i-1][j] + World0[i][j-1] + World0[i+1][j+1] + World0[i-1][j+1] + World0[i-1][j-1]+World0[i+1][j-1];
			World1[i][0] = 1;
			World1[i][225] = 1;
		}
		for(j = 0 ; j < 226 ; j++)
		{
			World1[0][j] = 1;
			World1[175][j]=1;
		}
      for(j = 0 ; j < 176 ; j++)
		{
			World1[j][0] = 1;
			World1[j][225]=1;
		}
		for(i=0;i < 176;i++){
			for(j=0;j < 226;j++){
				World0[i][j]=min(World1[i][j],1);
        }}
	}
  
  
  srand(time(NULL));
  
  for (i = 0; i < 8; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i],TIME_STEP*4);
  }
  
  
 char text[5] = "led0";
  for(i=0; i<10; i++) {
    led[i] = wb_robot_get_device(text);
    text[3]++;
    wb_led_set(led[i],0);
  }
  

  
  //communication = wb_robot_get_device("emitter");
  wb_differential_wheels_enable_encoders(TIME_STEP);

 

  
  i=0;
  int deg;
  	while(i < nP)
	{
		particle[i][0] = ((double) rand() /(double) RAND_MAX) *roomSize[0];
    //particle[i][0] = .7+((double) rand() /(double) RAND_MAX) *0.2;
    //particle[i][0] = 0.809499;
		particle[i][1] = ((double) rand() /(double) RAND_MAX) *roomSize[1];
    //particle[i][1] = .4+((double) rand() /(double) RAND_MAX) *0.2;
    //particle[i][1] = 0.530892;

    double ppp[2]={particle[i][0] , particle[i][1]};

		if( checkObstacle( World0,ppp) == 0)
		{
			//particle[i][2]= ((float) rand() /(float) RAND_MAX) *2*pi;
      //particle[i][2]= pi;
      deg=rand()%4;
      if (deg==0) particle[i][2]= -0.2+2*((float) rand() /(float) RAND_MAX) *0.2;
      if (deg==1) particle[i][2]=pi/2 -0.2+2*((float) rand() /(float) RAND_MAX) *0.2;
      if (deg==2) particle[i][2]=pi -0.2+2*((float) rand() /(float) RAND_MAX) *0.2;
      if (deg==3) particle[i][2]=3*pi/2 -0.2+2*((float) rand() /(float) RAND_MAX) *0.2;
			i++;
		}
	}
  
  
    FILE* parts= fopen("particles.txt","w");
  for (i=0;i<nP;i++){
    
    for (j=0;j<2;j++){
      fprintf(parts,"%f ",particle[i][j]);
    }
    fprintf(parts,"\n  ");
  }
  fclose(parts);
  
  
  FILE *checkMap;
  checkMap = fopen( "map2.txt" , "w" );  
  for (i=0;i<176;i++){
    fprintf(checkMap,"\n"  );
    for (j=0;j<226;j++){
      fprintf(checkMap,"%d ",World0[i][j]  );
    }
  }

  fclose(checkMap);
  

  int counter=0;
  
  float wFast=0;
  float wSlow=0;
  float aSlow=.03;
  float aFast=.3;
  bool turn=false;
  int leftSpeed,rightSpeed;
  int right;
  int go=0;
  double deltaL;
  double deltaA;
  int ledValue;
  int ledCheck=0;
  /* main loop */
  do {
    FILE* leds= fopen("led.txt","r");
    fscanf(leds,"%d",&ledValue);
    fclose(leds);
    if (ledValue==1 && ledCheck==0){
      ledCheck=1;
      for(i=0; i<10; i++) {
        wb_led_set(led[i],1);
      }
      wb_robot_step(TIME_STEP);

      printf("check=%d",ledCheck);
    }
    else if (ledValue==2 && ledCheck==0){
      ledCheck=1;
      for(i=0; i<10; i+=2) {
        wb_led_set(led[i],1);
      }
      printf("check=%d",ledCheck);
      wb_robot_step(TIME_STEP);

    }
    else if (ledValue==0 && ledCheck==1){
      ledCheck=0;
      for(i=0; i<10; i++) {
        wb_led_set(led[i],0);
      }
      printf("check=%d",ledCheck);
      wb_robot_step(TIME_STEP);

    }
  
    Wl=100;
    Wr=100;
    compute_odometry();
    if (dlr<0.008 && fabs(da)<0.175 && go==0){
      leftSpeed  = Wl;
      rightSpeed = Wr;
      for (i=0;i<8;i++){
        zs[i]=wb_distance_sensor_get_value(distance_sensor[i]);
      }

      // turn to left or to right when there is an obstacle
      if (zs[6]+zs[7] > 1600 || zs[0]+zs[1] > 1600) {
        if (!turn) {
          turn = true;
          if (zs[5]>=zs[2])
            right=1;
          else
            right=0;
        }
        if (right) {
          leftSpeed  =  Wl;
          rightSpeed = -Wr;
        } else {
          leftSpeed  = -Wl;
          rightSpeed =  Wr;
        }
      } else {
        turn=false;
      }
      
      

      wb_differential_wheels_set_speed(leftSpeed, rightSpeed);
      wb_robot_step(TIME_STEP);
      compute_odometry();
    }
    else if ((dlr>=0.008 || fabs(da)>=0.175) ){
      wb_differential_wheels_set_speed(0, 0);
      wb_robot_step(TIME_STEP);
      compute_odometry();
      deltaL=dlr;
      deltaA=da;
      wb_differential_wheels_set_encoders(0.0,0.0);
      wb_robot_step(TIME_STEP);
      
      go=1;
      
    }
    else{
      go=0;  
      for (i=0;i<8;i++){
        zs[i]=wb_distance_sensor_get_value(distance_sensor[i]);
        if (i==0){
          if (zs[i]<=437.3){
            ds[i]=0.05;
          }
          else if (437<zs[i] && zs[i]<=473.5){
            ds[i]=line(zs[i],437,473.5,0.05,0.04);
          }
          else if (473.5<zs[i] && zs[i]<=552.3){
            ds[i]=line(zs[i],473.5,552.3,0.04,0.03);
          }
          else if (552.3<zs[i] && zs[i]<=756.4){
            ds[i]=line(zs[i],552.3,756.4,0.03,0.02);
          }
          else if (756.4<zs[i]&&zs[i]<=1695){
            ds[i]=line(zs[i],756.4,1695,0.02,0.01);
          }
          else if (1695<zs[i]&&zs[i]<=3760){
            ds[i]=line(zs[i],1695,3760,0.01,0);
          }
          else{
            ds[i]=0.001;
          }
        }
        
        
        
        if (i==1){
          if (zs[i]<=206.9){
            ds[i]=0.05;
          }
          else if (206.9<zs[i]&&zs[i]<=246){
            ds[i]=line(zs[i],206.9,246,0.05,0.04);
          }
          else if (246<zs[i]&&zs[i]<=352.4){
            ds[i]=line(zs[i],246,352.4,0.04,0.03);
          }
          else if (352.4<zs[i]&&zs[i]<=604.9){
            ds[i]=line(zs[i],352.4,604.9,0.03,0.02);
          }
          else if (604.9<zs[i]&&zs[i]<=1986.6){
            ds[i]=line(zs[i],604.9,1986.6,0.02,0.01);
          }
          else if (1986.6<zs[i]&&zs[i]<=3771){
            ds[i]=line(zs[i],1986.6,3771,0.01,0);
          }
          else{
            ds[i]=0.001;
          }
        }
        
        if (i==2){
          if (zs[i]<=426.3){
            ds[i]=0.05;
          }
          else if (426.3<zs[i]&&zs[i]<=448.9){
            ds[i]=line(zs[i],385.2,422.6,0.05,0.04);
          }
          else if (448.9<zs[i]&&zs[i]<=512.6){
            ds[i]=line(zs[i],422.6,512.6,0.04,0.03);
          }
          else if (512.6<zs[i]&&zs[i]<=738.15){
            ds[i]=line(zs[i],512.6,738.15,0.03,0.02);
          }
          else if (738.15<zs[i]&&zs[i]<=2002){
            ds[i]=line(zs[i],738.15,2002,0.02,0.01);
          }
          else if (2002<zs[i]&&zs[i]<=3746){
            ds[i]=line(zs[i],2002,3746,0.01,0);
          }
          else{
            ds[i]=0.001;
          }
        }
        
        
        if (i==3){
          if (zs[i]<=441.7){
            ds[i]=0.05;
          }
          else if (441.7<zs[i]&&zs[i]<=484.9){
            ds[i]=line(zs[i],372.76,425.5,0.05,0.04);
          }
          else if (484.9<zs[i]&&zs[i]<=544){
            ds[i]=line(zs[i],425.5,544,0.04,0.03);
          }
          else if (544<zs[i]&&zs[i]<=905){
            ds[i]=line(zs[i],544,905,0.03,0.02);
          }
          else if (905<zs[i]&&zs[i]<=2213){
            ds[i]=line(zs[i],905,2213,0.02,0.01);
          }
          else if (2213<zs[i]&&zs[i]<=3770){
            ds[i]=line(zs[i],2213,3770,0.01,0);
          }
          else{
            ds[i]=0.001;
          }
        }
        
        
        if (i==4){
          if (zs[i]<=512){
            ds[i]=0.05;
          }
          else if (512<zs[i]&&zs[i]<=565){
            ds[i]=line(zs[i],430.9,483.9,0.05,0.04);
          }
          else if (565<zs[i]&&zs[i]<=681.5){
            ds[i]=line(zs[i],483.9,591,0.04,0.03);
          }
          else if (681.5<zs[i]&&zs[i]<=888.9){
            ds[i]=line(zs[i],591,888.9,0.03,0.02);
          }
          else if (888.9<zs[i]&&zs[i]<=2147.7){
            ds[i]=line(zs[i],888.9,2147.7,0.02,0.01);
          }
          else if (2147.7<zs[i]&&zs[i]<=3793){
            ds[i]=line(zs[i],2147.7,3793,0.01,0);
          }
          else{
            ds[i]=0.001;
          }
        }
        
        
        if (i==5){
          if (zs[i]<=441.4){
            ds[i]=0.05;
          }
          else if (441.4<zs[i]&&zs[i]<=484){
            ds[i]=line(zs[i],387.5,431.5,0.05,0.04);
          }
          else if (484<zs[i]&&zs[i]<=515.9){
            ds[i]=line(zs[i],431.5,515.9,0.04,0.03);
          }
          else if (515.9<zs[i]&&zs[i]<=783.9){
            ds[i]=line(zs[i],515.9,783.9,0.03,0.02);
          }
          else if (783.9<zs[i]&&zs[i]<=2105.4){
            ds[i]=line(zs[i],783.9,2105.4,0.02,0.01);
          }
          else if (2105.4<zs[i]&&zs[i]<=3787){
            ds[i]=line(zs[i],2105.4,3787,0.01,0);
          }
          else{
            ds[i]=0.001;
          }
        }
        
        
        if (i==6){
          if (zs[i]<=192.9){
            ds[i]=0.05;
          }
          else if (192.9<zs[i]&&zs[i]<=242){
            ds[i]=line(zs[i],192.9,242,0.05,0.04);
          }
          else if (242<zs[i]&&zs[i]<=338){
            ds[i]=line(zs[i],242,338,0.04,0.03);
          }
          else if (338<zs[i]&&zs[i]<=622){
            ds[i]=line(zs[i],338,622,0.03,0.02);
          }
          else if (622<zs[i]&&zs[i]<=1943){
            ds[i]=line(zs[i],622,1943,0.02,0.01);
          }
          else if (1943<zs[i]&&zs[i]<=3865){
            ds[i]=line(zs[i],1943,3865,0.01,0);
          }
          else{
            ds[i]=0.001;
          }
        }
        
        
        if (i==7){
          if (zs[i]<=1000){
            ds[i]=0.05;
          }
          else if (1000<zs[i]&&zs[i]<=1049){
            ds[i]=line(zs[i],1000,1049,0.05,0.04);
          }
          else if (1049<zs[i]&&zs[i]<=1150){
            ds[i]=line(zs[i],1049,1150.3,0.04,0.03);
          }
          else if (1150<zs[i]&&zs[i]<=1401){
            ds[i]=line(zs[i],1150,1401,0.03,0.02);
          }
          else if (1401<zs[i]&&zs[i]<=2251){
            ds[i]=line(zs[i],1401,2251,0.02,0.01);
          }
          else if (2251<zs[i]&&zs[i]<=3830){
            ds[i]=line(zs[i],2251,3830,0.01,0);
          }
          else{
            ds[i]=0.001;
          }
        }
      }
      

      sumW=0;
      
      for (i=0;i<nP;i++){

        particle[i][2]+=(deltaA)+generateGaussianNoise(deltaA*0.026,a1*deltaA+a2*deltaL);
        //particle[i][2]+=(deltaA)+generateGaussianNoise(0,a1*deltaA+a2*deltaL);

        particle[i][0]=min(1.13,particle[i][0]+cos(particle[i][2])*(deltaL+generateGaussianNoise(deltaL*0.008,a3*deltaL+a4*deltaA)));//deltaL*0.0143
        //particle[i][0]=min(0.9,particle[i][0]+cos(particle[i][2])*(deltaL+generateGaussianNoise(0,a3*deltaL+a4*deltaA)));
        particle[i][0]=max(0,particle[i][0]);
        particle[i][1]=min(0.88,particle[i][1]+sin(particle[i][2])*(deltaL+generateGaussianNoise(deltaL*0.008,a3*deltaL+a4*deltaA)));
        //particle[i][1]=min(0.6,particle[i][1]+sin(particle[i][2])*(deltaL+generateGaussianNoise(0,a3*deltaL+a4*deltaA)));
        particle[i][1]=max(0,particle[i][1]);
        rayCasting(map,map1,particle[i],z,thetaS, rE,maxRange);
           
        weight=1;
        for (j=0;j<8;j++){
          if((ds[j]==maxRange)) weight*=(0.9+.2*.2);

          else{
            diff=pow(100*(ds[j]-z[j]),2);
            weight =weight*(7.5*( 1/sqrt(2*pi*1.5) )* exp((-diff/(2 * 1.5)))+.2*.2);
          }
        }
            
        wp[i]=weight;
        sumW+=wp[i];
      }
      
      aaarrayMax(wp,sizeof(wp)/sizeof(double),maxInfo);
      maxWp=maxInfo[0];
      for (i=0;i<nP;i++){
        wp[i]+=maxWp;
      }
      sumW+=(float)nP*maxWp;
      
  

      double avg=sumW/(float)nP;
      double avg1=(sumW-(float)nP*maxWp)/(float)nP;
      double sum1=0;
      for (i=0;i<nP;i++){
        sum1 = sum1 + pow((wp[i] - avg), 2);
      } 
      float var=sum1/(float)nP;
      for (i=0;i<nP;i++){
        wp[i]/=sumW;
      }

      





printf("\n");
      wSlow=wSlow+aSlow*(avg1-wSlow);
      wFast=wFast+aFast*(avg1-wFast);
      float rProb=1-wFast/wSlow;
      printf("avg=%f\n",avg1);
      printf("wSlow=%f\n",wSlow);
      printf("wFast=%f\n",wFast);
      printf("Prand=%f\n",rProb);
      var=1;
      //rProb=1000;
      if (var>0.008 && rProb<.80){

        float CDF[nP];
        CDF[0] = wp[0];
        for(i = 1 ;i < nP ; i++)
        {
          CDF[i] = CDF[i-1] + wp[i];
        }
        double r;
        srand(time(NULL));
        for(i = 0;i < nP ; i++)
        {
          r = ((float)rand() / (float)(RAND_MAX))/(float)nP;
          r+=(float)i/(float)nP;
          j = 0;
          while (CDF[j] < r)
            j++;
          tempParticle[i][0] = particle[j][0];
          tempParticle[i][1] = particle[j][1];
          tempParticle[i][2] = particle[j][2];
        }  	

        for (i=0;i<nP;i++){
          particle[i][0]=tempParticle[i][0];
          particle[i][1]=tempParticle[i][1];
          particle[i][2]=tempParticle[i][2];
        }
      }
      else if(rProb>=.80){
        i=0;
        while(i < nP)
        {
          //particle[i][0] = ((double) rand() /(double) RAND_MAX) *roomSize[0];
          particle[i][0] = ((double) rand() /(double) RAND_MAX) *1.13;
          //particle[i][1] = ((double) rand() /(double) RAND_MAX) *roomSize[1];
          particle[i][1] = ((double) rand() /(double) RAND_MAX) *0.88;

          double ppp[2]={particle[i][0] , particle[i][1]};

          if( checkObstacle( World0,ppp) == 0)
          {
            particle[i][2]= ((float) rand() /(float) RAND_MAX) *2*pi;
            i++;
          }
        }
        wSlow=0;
        wFast=0;
        
      }








      
      FILE* data1= fopen("Xparticle.txt","w");
      for (i=0;i<nP;i++){
        fprintf(data1,"%f ",particle[i][0]);
      }
      //fprintf(data1,"\n  ");
      fclose(data1);
      
      
      FILE* data2= fopen("Yparticle.txt","w");
      for (i=0;i<nP;i++){
        fprintf(data2,"%f ",particle[i][1]);
      }
      //fprintf(data2,"\n  ");
      fclose(data2);

    }
    counter++;
     
  } while (wb_robot_step(TIME_STEP) != -1);
  

  wb_robot_cleanup();
  
  return 0;
}





//************************Functions***********************//

double generateGaussianNoise(double mu, double sigma)
{
	const double epsilon = 2.22507*pow(10,-308);
	const double two_pi = 2.0*3.14159265358979323846;
 
	static double z0, z1;
	static bool generate;
	generate = !generate;
 
	if (!generate)
	   return z1 * sigma + mu;
 
	double u1, u2;
	do
	 {
	   u1 = rand() * (1.0 / RAND_MAX);
	   u2 = rand() * (1.0 / RAND_MAX);
	 }
	while ( u1 <= epsilon );
 
	z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
	return z0 * sigma + mu;
}


void rayCasting(int map[176][226],int map1[176][226],double p[3] ,double zd[8],const double theta[8],const double rE,double maxRange){
  int i;
  double ps[2];
  double pTemp[2];

  double rs;
  bool flag=false;

 
  for(i=0;i<8;i++){
    flag=false;
    ps[0]=p[0]+rE*cos(theta[i]+p[2]);
    ps[1]=p[1]+rE*sin(theta[i]+p[2]);
    rs=0;
    while (!flag){
      rs+=0.003;
      pTemp[0]=ps[0]+rs*cos(theta[i]+p[2]);
      pTemp[1]=ps[1]+rs*sin(theta[i]+p[2]);
      pTemp[1]=0.88-pTemp[1];
      if (rs>maxRange){
        zd[i]=maxRange;
        flag=true;
      }
      if(pTemp[0]<=0.003||pTemp[1]<=0.003||pTemp[0]>=1.125||pTemp[1]>=0.875||checkObstacle(map,pTemp)==1){
        zd[i]=rs;
        flag=true;
      }
    }
  }
  
}


int checkObstacle(int map1[176][226],double pTemp[2]){
  int pCheck[2];
  pCheck[0]=round((100*pTemp[0])/0.5);
  pCheck[1]=round((100*pTemp[1])/0.5);  
  if (map1[pCheck[1]][pCheck[0]]==1)
    return 1;
  else
    return 0;
}

double arrayMax(double *arr,int len){
  int i;
  double maxArr;
  for(i=0;i<len;i++){   
    maxArr=(maxArr>arr[i]) ? maxArr:arr[i];
  }
  return maxArr;
}

void aaarrayMax(double *arr,int len,double maxArr[2]){
  int i;
   maxArr[0]=-1000000;
   maxArr[1]=0;
  for(i=0;i<len;i++){
    if (arr[i]>maxArr[0]){
      maxArr[0]=arr[i];
      maxArr[1]=i;
    }
  }
}


double generateGaussianNoise1(double mu, double sigma){
  float s=0;
  float r;
  int i;
  for(i=0;i<12;i++){
    r=(float)rand()/((float)RAND_MAX);
    s+=-sigma+2*sigma*r;
  }
  return mu+0.5*s;
}





  
