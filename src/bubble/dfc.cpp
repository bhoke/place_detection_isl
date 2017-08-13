// dfc.cpp : Defines the entry point for the console application.
//

#include "bubbleprocess.h"
#include <stdlib.h>    // exit(..), NULL
#include <stdio.h>     // FILE structure
#include <iostream>  // cin, cout objects & methods
#include <fstream>  // cin, cout objects & methods
#include <cmath>
#include <ctime>
#include <QDebug>
using namespace std;

#define PI 3.14

DFCoefficients bubbleProcess::calculateDFCoefficients(const std::vector<bubblePoint>& bubble, int harmonic1, int harmonic2)
{
    DFCoefficients dfcoeff;

    dfcoeff.a.resize(harmonic1, std::vector<float>(harmonic2, 0));
    dfcoeff.b.resize(harmonic1, std::vector<float>(harmonic2, 0));
    dfcoeff.c.resize(harmonic1, std::vector<float>(harmonic2, 0));
    dfcoeff.d.resize(harmonic1, std::vector<float>(harmonic2, 0));

    float f1,f2,rho;
    float angles[2];

    for(uint i = 0; i < bubble.size(); i++)
    {
        angles[0] = bubble.at(i).panAng;
        angles[1] = bubble.at(i).tiltAng;
        //Reconstruct: rho value is changed, now it is in interval [0.5,2]
        rho = bubble[i].val + 1;

        while (angles[0]<0 || angles[0]>360) {
            if (angles[0]<0)
                angles[0]=360+angles[0];
            if (angles[0]>360)
                angles[0]=angles[0]-360;
        }
        while (angles[1]<0 || angles[1]>360) {
            if (angles[1]<0)
                angles[1]=360+angles[1];
            if (angles[1]>360)
                angles[1]=angles[1]-360;
        }

        for (int m=0;m<harmonic1;m++)
            for (int n=0;n<harmonic2;n++) {
                f1= angles[0];
                f2= angles[1];
                dfcoeff.a[m][n] += rho*cos(m*f1*PI/180)*cos(n*f2*PI/180)*4;
                dfcoeff.b[m][n] += rho*sin(m*f1*PI/180)*cos(n*f2*PI/180)*4;
                dfcoeff.c[m][n] += rho*cos(m*f1*PI/180)*sin(n*f2*PI/180)*4;
                dfcoeff.d[m][n] += rho*sin(m*f1*PI/180)*sin(n*f2*PI/180)*4;
            }
    }
    return dfcoeff;
}
