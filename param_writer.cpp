#include <fstream>
#include <iostream>
#include <string>

using namespace std;

ofstream mpcfile_w ("mpc_params.txt");

static void write_params(double c, double e, double v, double d, double a, double dd, double da)
{
  if (mpcfile_w.is_open())
  {
    mpcfile_w << c << " " << e << " " << v << " " << d << " " << a << " " << dd << " " << da << endl;
    mpcfile_w.close();
  }
  else cout << "Unable to open file";
}

int main(int argc, char *argv[])
{
  
  double c, e, v, d, a, dd, da;
  c = stof(argv[1]);
  e = stof(argv[2]);
  v = stof(argv[3]);
  d = stof(argv[4]);
  a = stof(argv[5]);
  dd = stof(argv[6]);
  da = stof(argv[7]);
  cout << "Params written to file: " << c << " " << e << " " << v << " " << d << " " << a << " " << dd << " " << da << endl;

  write_params(c, e, v, d, a, dd, da);
}
