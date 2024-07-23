#pragma once

#include <BasicLinearAlgebra.h>

using namespace BLA;

static const int systemOrder = 2;

struct ss{
  Matrix<systemOrder,systemOrder> A;
  Matrix<systemOrder,1> B;
  Matrix<1,systemOrder> C;
  Matrix<1,1> D;
};

// ** Modelo 1 - H INFINITO**
// ss sys = {
//   {
//     1.621, 1,
//     -0.6211, 0
//   },
//   {
//     0.001411, 0.0001098
//   },
//   {1, 0},
//   {0}
// };

// ** Modelo 2 - MOLA FORTE - H2**
ss sys = {
  {
    1.97531085404608, 1,
    -0.975319493951527, 0
  },
  {
      2.59935147074799e-05,
      1.72494796427973e-06
  },
  {1, 0},
  {0}
};

// ** Modelo 3 - MOLA MEDIA - H2**
// ss sys = {
//   {
//     -1.98459118905302, 1,
//     0.984594439497667, 0
//   },
//   {
//     6.04195037933545e-05,
//     -3.10626011333367e-05
//   },
//   {1, 0},
//   {0}
// };