/***************************************************************************************************
A client that connects on the server and sends the XYZ command in order to control the motors thrust
***************************************************************************************************/
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include "rosfond/ADCmcp3008Spi.h"



using namespace std;



void sleepMs(int ms)
{
    usleep(ms*1000); //convert to microseconds
    return;
}

int absoluteValue(int a )
{
    if(a<0 ) return -a ;
    return a ;
}

int isThereABigDifference(int a ,int b ,int thatIsABigDifference)
{
    if (absoluteValue(a-b)>thatIsABigDifference) return 1 ;
    return 0;
}

int main(int argc, char *argv[])
{

    char buffer[256];
    int valeurFin=205;
    int a2dVal[4] = {510,5110,510,0}; // destine a contenir les valeurs des intensites
    int a2dBuff[4] = {510,5110,510,0}; // destine a contenir les valeurs des intensites
    int a2dChannel = 0;
    unsigned char data[3];
    int i,channelActif=0;
    string coutRes[4]= {"intensite 1  ="," intensite 2 ="," intensite 3 ="," intensite 4 = "};
    float intensity = 0 ;
    float bufCourant[65];
    ADCmcp3008Spi convertisseurIntensites("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
    std_msgs::Int32MultiArray tableauAEnvoyer ;




    ros::init(argc,argv,"intensite_node");

    ros::NodeHandle noeud;

    ros::Publisher publisherCommande = noeud.advertise<std_msgs::Int32MultiArray>("intensite",1000);

    ros::Rate loop_rate(100);



    /// Command loop

    while(ros::ok() )        ///Lorsque l'on tourne le 3eme axe , on "eteint le moteur"
    {

        for(a2dChannel=0; a2dChannel<4; a2dChannel++)
        {
            intensity=0;

            for (i=0;i<65;i++)
            {

                data[0] = 1;  //  first byte transmitted -> start bit
                data[1] = 0b10000000 |( ((a2dChannel & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1, D2=D1=D0=0)
                data[2] = 0; // third byte transmitted....don't care
                convertisseurIntensites.spiWriteRead(data, sizeof(data) );
                a2dBuff[a2dChannel] = 0;
                a2dBuff[a2dChannel] = (data[1]<< 8) & 0b1100000000; //merge data[1] & data[2] to get result
                a2dBuff[a2dChannel] |=  (data[2] & 0xff);


                bufCourant[i]=a2dBuff[4] ;

                intensity += bufCourant[i];
            }

            intensity/=65;
            intensity-=510;


            a2dVal[a2dChannel]=intensity *6*66*1.33/10000;

        }


        tableauAEnvoyer.data.clear();

        for(a2dChannel=0; a2dChannel<3; a2dChannel++)
        {
            std::cout << a2dBuff[a2dChannel] << std::endl ;
            tableauAEnvoyer.data.push_back(a2dVal[a2dChannel]);
        }

        publisherCommande.publish(tableauAEnvoyer);

        ros::spinOnce();


    }


    return 0;
}
