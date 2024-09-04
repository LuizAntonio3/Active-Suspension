#pragma once
#include <BasicLinearAlgebra.h>

using namespace BLA;

static const int systemOrderGeral = 5;
// static const int saidas = 1;

struct ssGeral{
  Matrix<systemOrderGeral,systemOrderGeral> A;
  Matrix<systemOrderGeral,1> B;
  Matrix<1,systemOrderGeral> C;
  Matrix<1,1> D;
};

// Modelo final utilizado - dados: "(30-150)-2ident-crtl-novo-40ms-2-susp-05-02-2024 17-45.csv" - 2 entradas
ssGeral sysGeral = {
  {
    0.565226745901493   , 1, 0, 0, 0,
    0.147586643392935   , 0, 1, 0, 0,
    -0.304242172000966  , 0, 0, 1, 0,
    -0.00538050249672198, 0, 0, 0, 1,
    0.067355248576711   , 0, 0, 0, 0
  },
  { 
    0.299578174721714,
    -0.383620653895442,
    -0.0402082391731603,
    0.034053455293313,
    0.090257543531445
  },
  {
    1, 0, 0, 0, 0
  },
  {0}
};

// Modelos não utilizados
/*

// modelo mola media
ssGeral sysGeral = {
  {
    0.529595214929287   , 1, 0, 0, 0,
    0.485010316061038   , 0, 1, 0, 0,
    -0.225636271019077  , 0, 0, 1, 0,
    0.0485043700385894, 0, 0, 0, 1,
    0.0613224476230946   , 0, 0, 0, 0
  },
  { 
        0.0371790900110465,
        -0.073400009236704,
        0.0377270917609542,
     -2.76847604474093e-05,
       -0.0015003834159776
  },
  {
    1, 0, 0, 0, 0
  },
  {0}
};

*/