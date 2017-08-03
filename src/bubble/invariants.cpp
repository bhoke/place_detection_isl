// invariants.cpp : Defines the entry point for the console application.
//
#include "bubbleprocess.h"
//#include "stdafx.h"
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

#define PLACES 63
#define STARTNO 0



using namespace std;


cv::Mat bubbleProcess::calculateInvariantsMat(DFCoefficients coeff, int harmonic1, int harmonic2)
{

    cv::Mat result(1,harmonic2*harmonic1,CV_32FC1);

    for (int m=0;m<harmonic1;m++)

        for (int n=0;n<harmonic2;n++)
        {

            float val = 0;
            if (m==0)
                val =coeff.a[m][n]*coeff.a[m][n]+coeff.c[m][n]*coeff.c[m][n];
            else
                val =coeff.a[m][n]*coeff.a[m][n]+coeff.b[m][n]*coeff.b[m][n]+coeff.c[m][n]*coeff.c[m][n]+coeff.d[m][n]*coeff.d[m][n];

            result.at<float>(0,10*m+n) = val;
        }

    return result;
}


