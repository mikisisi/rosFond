
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "rosfond/i2c8bit.h"
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>



///PID
#define KP -100
#define KD -500
#define KI -0
#define DIVISEUR 400


#define XRES 800
#define YRES 600
#define MINX -5.0
#define MINY -6.0
#define MAXX 10.0
#define MAXY 10.0

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600


#ifdef WIN32
#pragma comment(lib,"sdl.lib")
#pragma comment(lib,"sdlmain.lib")
#endif
using namespace std;


float remiseALEchelle(float valeurAConvertir, float in_min, float in_max, float out_min, float out_max)
{
    if( valeurAConvertir < in_min) return out_min;
    if( valeurAConvertir > in_max) return out_max;
    return (valeurAConvertir - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}







int main(int argc, char *argv[])
{

    unsigned char data1,data2  ;
    int valeurs[800];
    int i  ;
    int x=0;
    int stableDepuis =  0 ;
    int cap=0;
    int angleActuel;
    int erreurActuel=0;
    int erreurPrecedent=0;
    int sommeErreurs=0;
    float commandeMoteur[4]= {510,510,510};
    std_msgs::Int32MultiArray tableauAEnvoyer ;




    i2c8Bit compas(0x60,string("/dev/i2c-1"));      /// 40 en hexa corresponds à l'adresse i2c standard de notre carte adafruit




    ros::init(argc,argv,"compas_node");

    ros::NodeHandle noeud;

    ros::Publisher publisherCommande = noeud.advertise<std_msgs::Int32MultiArray>("compas",1000);

    ros::Rate loop_rate(100);

    while ( ros::ok() )
    {
        compas.readReg(2,data1);
        compas.readReg(3,data2);
        angleActuel= (int)(256*(int)data1+(int)data2);

        tableauAEnvoyer.data.clear();

        tableauAEnvoyer.data.push_back(angleActuel);


        publisherCommande.publish(tableauAEnvoyer);
        ros::spinOnce();
        


    }











    return 0;
}
