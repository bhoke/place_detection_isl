// invariants.cpp : Defines the entry point for the console application.
//
#include "bubbleprocess.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <QDebug>
using namespace std;

#define PI 3.14

cv::Mat bubbleProcess::calculateInvariantsMat(DFCoefficients coeff, int harmonic1, int harmonic2)
{

    cv::Mat result(harmonic1*harmonic2,1,CV_32FC1);

    for (int m=0;m<harmonic1;m++)

        for (int n=0;n<harmonic2;n++)
        {
            float val = 0;
            if (m==0)
                val =coeff.a[m][n]*coeff.a[m][n]+coeff.c[m][n]*coeff.c[m][n];
            else
                val =coeff.a[m][n]*coeff.a[m][n]+coeff.b[m][n]*coeff.b[m][n]+coeff.c[m][n]*coeff.c[m][n]+coeff.d[m][n]*coeff.d[m][n];

            result.at<float>(10*m+n,0) = val;
        }

    return result;
}


