/*************************************************************************************************
FOND 4 MOTEURS : A server that receives an (X,Y,Zmomentum) and sets a corresponding thrust output
*************************************************************************************************/

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "rosfond/i2c8bit.h"
#include "rosfond/controlleurPWMPCA9685.h"
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include "rosfond/ADCmcp3008Spi.h"


/// The falling edge of the pwm is set on a register up to 4096, the rising edge being at 0 and the frequency 50Hz, the period is 20ms
#define MOTEURSTART 302 // 1500 μs
#define MOTEURMAX   362
#define MOTEURMIN   242
#define MOTEURSTOP  302 // 1500 μs
#define MOTEURDEADBANDMIN  293
#define MOTEURDEADBANDMAX  307

#define IMAX 3500

#define IDMOTEUR1    0
#define IDMOTEUR2    1
#define IDMOTEUR3    2
#define IDMOTEUR4    3

#define IDMOTEUR5    4
#define IDMOTEUR6    5
#define IDMOTEUR7    6




#define X    0
#define Y    1
#define OMEGA    2
#define RAYON 1

#define XRES 800
#define YRES 600
#define MINX -5.0
#define MINY -6.0
#define MAXX 10.0
#define MAXY 10.0


float userDirectionInput[4] ;  //destine a contenir les valeurs des axes X Y OMEGA

float intensite[4] ;  //destine a contenir les valeurs des axes X Y OMEGA



using namespace std;


void sleepMs(float ms)
{
    usleep((int)(ms*1000)); //convert to microseconds
    return;
}



float rescaleWithDeadband(float valeurAConvertir, float inMinBandeUn, float inMaxBandeUn, float outMinBandeUn, float outMaxBandeUn,float inMinBandeDeux, float inMaxBandeDeux, float outMinBandeDeux, float outMaxBandeDeux)
{
    if( valeurAConvertir < inMinBandeUn) return outMinBandeUn;

    if( valeurAConvertir >= inMinBandeUn && valeurAConvertir < inMaxBandeUn )
        return (valeurAConvertir - inMinBandeUn) * (outMaxBandeUn - outMinBandeUn) / (inMaxBandeUn - inMinBandeUn) + outMinBandeUn;

    if( valeurAConvertir >= inMaxBandeUn && valeurAConvertir < inMinBandeDeux )
        return ( outMaxBandeUn + outMinBandeDeux)/2 ;

    if( valeurAConvertir >= inMinBandeDeux && valeurAConvertir < inMaxBandeDeux )
        return (valeurAConvertir - inMinBandeDeux) * (outMaxBandeDeux - outMinBandeDeux) / (inMaxBandeDeux - inMinBandeDeux) + outMinBandeDeux;

    if( valeurAConvertir >= inMaxBandeDeux) return outMaxBandeDeux;

    return MOTEURSTOP ; // Just in case
}


void thrustCalculation (float* userDirectionInput, float* moteurs)
{
    moteurs[IDMOTEUR1]= userDirectionInput[X] - userDirectionInput[Y] + userDirectionInput[2] ;
    moteurs[IDMOTEUR2]= userDirectionInput[X] + userDirectionInput[Y] - userDirectionInput[2] ;
    moteurs[IDMOTEUR3]=-userDirectionInput[X] - userDirectionInput[Y] - userDirectionInput[2] ;
    moteurs[IDMOTEUR4]=-userDirectionInput[X] + userDirectionInput[Y] + userDirectionInput[2] ;

    moteurs[IDMOTEUR5]=-userDirectionInput[3];
    moteurs[IDMOTEUR6]=-userDirectionInput[3] ;
    moteurs[IDMOTEUR7]=-userDirectionInput[3] ;
}


void callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
printf("hello");
userDirectionInput[0]=msg->data[0];
userDirectionInput[1]=msg->data[1];
userDirectionInput[2]=msg->data[2];
userDirectionInput[3]=msg->data[3];
cout << "-----------------------------------------------------\n" << userDirectionInput[3] << "-------------------------------------------------" << endl;
}

void callbackIntensite(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
intensite[0]=msg->data[0];
intensite[1]=msg->data[1];
intensite[2]=msg->data[2];
intensite[3]=msg->data[3];
}


int main(int argc, char *argv[])
{
    int mode,i;
    unsigned char i2cBuffer;
    float temp;
	int zoneCritique ;
    float thrustOutput[16]= {MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP,MOTEURSTOP};
    int channelActif=0;
    string coutRes[8]= {"Intensity 1 "," Intensity 2 "," Intensity 3 ","Intensity 4","","","",""};
	string coutCommande[4]= {" x = "," y =  "," Omega =  "," h = "};

    i2c8Bit thermometre(0x18,string("/dev/i2c-1"));	// 0x18 est l'adresse du thermometre mcp9808 et le bus i2c utilisé est le bus 1 de la pi
    controlleurPWMPCA9685 adafruit(0x40,string("/dev/i2c-1"));      // 40 en hexa corresponds à l'adresse i2c standard de notre carte adafruit
    unsigned char data[3];




    ros::init(argc,argv,"motor_node");

    ros::NodeHandle noeud;

    ros::Subscriber subscribeCommandes = noeud.subscribe<std_msgs::Int32MultiArray>("commandeUtilisateur",1000,callback);

    ros::Subscriber subscribeIntensite = noeud.subscribe<std_msgs::Float32MultiArray>("intensite",1000,callbackIntensite);

    ros::Rate loop_rate(100);






    adafruit.setFreq(50); //ON SET LA F A 50 Hz --> periode de 20ms
    adafruit.reset();     //  on remets le mode de configuration par defaut
  for (i=0;i<7;i++)  adafruit.setPWM(i,0,MOTEURSTART);  //  Initialisation de tous les moteurss



        while ( ros::ok() ) {
        

                ///TEMPERATURE
                /*
        thermometre.readReg(0x05,i2cBuffer);
                temp = i2cBuffer & 0x0FFF;
                temp /=  16.0;
                if (i2cBuffer & 0x1000) temp -= 256;
          printf("\nTemperature: %f",temp);
*/


          /*************              RECEPTION DES COMMANDES             **************/
  
  		

		

                
                //    if there is just a really smal variation, we consider the joystick as still
              

if (intensite[0] >IMAX || intensite[1] > IMAX || intensite[2] >IMAX || intensite[3] >IMAX )
{
printf("ERROR INTENSITY SURCHARGE OVER %d",IMAX);
zoneCritique++ ;
cout << endl << "Zone Critiaue Depuis : " << zoneCritique << endl ;
for ( i = 0 ; i <4; i ++ )
{
cout << "intensite numero   " << i <<  " egale "  << intensite [i] ;
} 
if(zoneCritique>20)break ; 
}
else zoneCritique=0;

 		if(userDirectionInput[X]<7 && userDirectionInput[X] > -7) userDirectionInput[X]=0;
                if(userDirectionInput[Y]<7 && userDirectionInput[Y] > -7) userDirectionInput[Y]=0;
                if(userDirectionInput[OMEGA]<7 && userDirectionInput[OMEGA] > -7) userDirectionInput[OMEGA]=0;
                if(userDirectionInput[7]<3 && userDirectionInput[3] > -7) userDirectionInput[3]=0;
                

   for (i=0;i<4;i++)   printf( " intensite %d egql %f ",i,intensite[i]) ;
      printf ("\n\n");


	  for (i=0;i<4;i++)    cout <<coutCommande[i]<< userDirectionInput[i]<<endl ;
                        printf("\n\n");


                thrustCalculation(userDirectionInput,thrustOutput);

                for (i=0;i<7;i++)  printf("Brut %d :\t%f",i,thrustOutput[i]);
                        printf("\n");

                // we rescale the thrust on the corrsponding values from the specific esc
                for (i=0;i<7;i++)       thrustOutput[i]=rescaleWithDeadband(thrustOutput[i],-300,-3,MOTEURMIN,MOTEURDEADBANDMIN,3,300,MOTEURDEADBANDMAX,MOTEURMAX);


                for (i=0;i<7;i++)  printf("Envoye %d :\t%f",i,thrustOutput[i]);
                        printf("\n");


               // and then set those values for the motors to thrust what we asked for
               for (i=0;i<7;i++)  adafruit.setPWM(i,0,thrustOutput[i]);

ros::spinOnce();          
}



        //we set the motors still when the client disconnects
               for (i=0;i<7;i++)  adafruit.setPWM(i,0,MOTEURSTOP);

 
return 0;
}
