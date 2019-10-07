# Bezier
Library for least-square fitting [composite Bezier curves](https://en.wikipedia.org/wiki/Composite_B%C3%A9zier_curve).

## Features

* Method for least square fitting _C<sup>1</sup>_ continuous composite Bezier curves

* Postscript interface for plotting data points and composite Bezier curves.

## Installation

### Dependencies

The only dependencies are [Eigen](http://eigen.tuxfamily.org/) (tested with 3.3.4) and `cmake`.

### Install

To install the library into the directory `installation` run

```
git clone https://github.com/barkm/bezier.git
cd bezier
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=installation/ ..
make -j
make install
```

To uninstall 

```
cd build
make uninstall
```

## Example
The following example generates a postscript file showing data points sampled from a sine wave with a composite bezier curve fitted to the data. 

```cpp
#include <bezier/bezier.h>
using namespace Eigen;
int main(){
  // Generate sin wave data
  ArrayXd theta = ArrayXd::LinSpaced(30, 0, 2 * M_PI);
  ArrayXd sin = Eigen::sin(theta);
  std::vector<VectorXd> data;
  for(int i = 0; i < theta.rows(); ++i){
    data.push_back(Vector2d(theta(i), sin(i)));
  }

  // Fit composite Bezier curve
  bezier::CompositeBezierCurve bezier =
                bezier::fit_composite_bezier_curve(
                                                  data,       // the data
                                                  {9, 19},    // joints
                                                  {3, 4, 5},  // curve degrees
                                                  false       // closed curve
                                                  );

  // Write output to postscript file
  bezier::PostScriptWriter ps_writer(
                                    "example.eps",                                    // file name
                                    {Vector2d(-0.5, -1.5), Vector2d(2*M_PI+0.5, 1.5)} // limits
                                    );
  ps_writer.color(1, 0, 0); ps_writer.filled(true);
  for(const auto & p : data){
    ps_writer.circle(p, 3);
  }
  ps_writer.color(0, 0, 0); ps_writer.filled(false);
  bezier::write_curve(ps_writer, &bezier);
}
```

If the library is installed in the directory `installation`, then you should be able to compile the example with 

```bash
g++ example.cpp -o example.out -std=c++11 -I /usr/local/include/eigen3/ -I installation/include -L installation/lib -l bezier
```


More examples can be found in the [examples](examples/) directory. Samples can be found in the [samples](samples/) directory.
