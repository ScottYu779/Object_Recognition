/*
 * The MIT License (MIT)

Copyright (c) 2015 Jetsonhacks

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
// servoExample.cpp
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <termios.h>
#include <time.h>
#include <JHPWMPCA9685.h>
#include <iostream>
#include <math.h>
#include <fstream>

#define PI 3.14159265358979323846

using namespace std;

// Calibrated for a Robot Geek RGS-13 Servo
// Make sure these are appropriate for the servo being used!


int servoMin[14]={120, 135, 172, 155, 160, 150, 120, 135, 172, 155, 160, 150, 150, 150};
int servoMax[14]={612, 390, 659, 410, 675, 370, 612, 390, 659, 410, 675, 370, 650, 650};
int max_angle[14]={180, 90, 180, 90, 180, 90, 180, 90, 180, 90, 180, 90, 180, 180};

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

// Map an integer from one coordinate system to another
// This is used to map the servo values to degrees
// e.g. map(90,0,180,servoMin, servoMax)
// Maps 90 degrees to the servo value

double motor_map ( int x, int in_min, int in_max, int out_min, int out_max) {
    double toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn ;
}

double invmap(double y,double in_min,double in_max,double out_min,double out_max)
{
double toReturn = ((y-out_min)*(in_max-in_min)/(out_max-out_min)+in_min);
return toReturn;
}

double regread(int chan ,PCA9685 *a)
{
	double  read,read1,read2,dread;
	read1=a->getPWMofl(chan);
	usleep(100);
	read2=a->getPWMofh(chan);
	usleep(100);
	read=(read2*256)+read1;
	dread=invmap(read,0,max_angle[chan],servoMin[chan],servoMax[chan]);
	return dread;
}

void rotate(int d[14],PCA9685 *a)
{
    double j[14],temp[14],count[14],flag[14];   
	for(int i=0;i<14;i++)
	{
		temp[i]=round(regread(i,a));
		j[i]=temp[i];
		flag[i]=0;
		if(temp[i]>d[i])
		{
		count[i]=-1;
		}
		else if(temp[i]<d[i])
		{
		count[i]=1;
		}
		else if(temp[i]==d[i])
		count[i]=0;
	}
	
	while(true)
	{
		for(int i=0;i<14;i++)
		{
		temp[i]=round(regread(i,a));
		if(d[i]!=temp[i])
		{	
			j[i]+=count[i];
			a->setPWM(i,0,motor_map(j[i],0,max_angle[i],servoMin[i],servoMax[i]));
		}
		else
		flag[i]=1;
		}
		for(int i=0;i<14;i++)
		{
		cout<<temp[i]<<"->"<<d[i]<<"..";
		}
		cout<<endl;
		usleep(5000);
		if(flag[0]==1 &&flag[1]==1 && flag[2]==1 && flag[3]==1 && flag[4]==1 && flag[5]==1 && flag[6]==1 && flag[7]==1 && flag[8]==1 && flag[9]==1 && flag[10]==1 && flag[11]==1 && flag[12]==1 && flag[13]==1)
			break;
	}
}


