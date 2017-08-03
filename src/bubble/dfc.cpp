// dfc.cpp : Defines the entry point for the console application.
//

#include "bubbleprocess.h"
#include <stdlib.h>    // exit(..), NULL
#include <stdio.h>     // FILE structure
//#include <conio.h>     // clrscr(), getch()
#include <iostream>  // cin, cout objects & methods
#include <fstream>  // cin, cout objects & methods
#include <cmath>
#include <ctime>
#include <QDebug>
using namespace std;

#define PI 3.14
//#define M 30
//#define N 30

#define PLACES 77
#define STARTNO 1


DFCoefficients bubbleProcess::calculateDFCoefficients(const std::vector<bubblePoint>& bubble, int harmonic1, int harmonic2)
{

    int M = harmonic1;

    int N = harmonic2;

    DFCoefficients dfcoeff;

    dfcoeff.a.resize(M, std::vector<float>(N, 0));
    dfcoeff.b.resize(M, std::vector<float>(N, 0));
    dfcoeff.c.resize(M, std::vector<float>(N, 0));
    dfcoeff.d.resize(M, std::vector<float>(N, 0));

    float f1,f2;
    float angle=5.5, rho;
    f1=angle*M/360;
    float angles[2];
    int angleIndex[2],newFixation=1,fixations=0;

    for(uint i = 0; i < bubble.size(); i++)
    {
        angles[0] = bubble.at(i).panAng;
        angles[1] = bubble.at(i).tiltAng;
        rho = bubble.at(i).val;

        rho=(rho*10 + 5);

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
        angleIndex[0]=int(angles[0]);
        angleIndex[1]=int(angles[1]);


        if (newFixation==1)
        {

            fixations++;

            for (int m=0;m<M;m++)
                for (int n=0;n<N;n++) {
                    f1=float(angleIndex[0]);
                    f2=float(angleIndex[1]);
                    dfcoeff.a[m][n]=dfcoeff.a[m][n]+rho*cos(m*f1*PI/180)*cos(n*f2*PI/180)*4;
                    dfcoeff.b[m][n]=dfcoeff.b[m][n]+rho*sin(m*f1*PI/180)*cos(n*f2*PI/180)*4;
                    dfcoeff.c[m][n]=dfcoeff.c[m][n]+rho*cos(m*f1*PI/180)*sin(n*f2*PI/180)*4;
                    dfcoeff.d[m][n]=dfcoeff.d[m][n]+rho*sin(m*f1*PI/180)*sin(n*f2*PI/180)*4;
                }
        }
        newFixation=1;
    }
    return dfcoeff;
}









