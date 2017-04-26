#include <iostream>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <string>
#include <fstream>


using namespace std;

struct box_params
{
	int n;
	Rect r;
	Vec3f v;
};

int main()
{
  box_params box_obj;
  ifstream file;
  file.open("coordinates.dat", ios::binary | ios::in);
  file.read(&box_obj,sizeof(box_obj));
  cout<<box_obj.n;
  file.close();
}
