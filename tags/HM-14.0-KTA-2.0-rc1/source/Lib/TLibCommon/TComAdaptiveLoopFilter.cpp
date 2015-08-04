/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComAdaptiveLoopFilter.cpp
    \brief    adaptive loop filter class
*/

#include "TComAdaptiveLoopFilter.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// ====================================================================================================================
// Tables
// ====================================================================================================================

#if ALF_HM3_QC_REFACTOR
Int TComAdaptiveLoopFilter::m_pattern9x9Sym[39] = 
{
                   0,
               1,  2,  3,
           4,  5,  6,  7,  8,
       9, 10, 11, 12, 13, 14, 15,
      16, 17, 18, 19, 18, 17, 16,
      15, 14, 13, 12, 11, 10,  9, 
           8,  7,  6,  5,  4,
               3,  2,  1,
                   0
};
 
Int TComAdaptiveLoopFilter::m_weights9x9Sym[21] = 
{
               2,  2,  2,   
           2,  2,  2,  2,  2, 
       2,  2,  2,  2,  2,  2,  2,  
   2,  2,  2,  2,  1,  1
};

Int TComAdaptiveLoopFilter::m_pattern9x9Sym_Quart[42] = 
{
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  1,  2,  3,  0,  0,  0,
   0,  0,  4,  5,  6,  7,  8,  0,  0,  
   0,  9, 10, 11, 12, 13, 14, 15,  0,
  16, 17, 18, 19, 20, 21
};

Int TComAdaptiveLoopFilter::m_pattern7x7Sym[25] = 
{
                   0,
             1,  2,  3,
       4,  5,  6,  7,  8,
       9, 10, 11, 12, 11, 10, 9,
           8,  7,  6,  5,  4,
           3,  2,  1,
             0
};

Int TComAdaptiveLoopFilter::m_weights7x7Sym[14] = 
{
                  2,  
              2,  2,  2,   
        2,  2,  2,  2,  2,    
      2,  2,  2,  1,  1
};

Int TComAdaptiveLoopFilter::m_pattern7x7Sym_Quart[42] = 
{
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  1,  0,  0,  0,  0,
   0,  0,  0,  2,  3,  4,  0,  0,  0,
   0,  0,  5,  6,  7,  8,  9,  0,  0,  
   0, 10, 11, 12, 13, 14,  
};

Int TComAdaptiveLoopFilter::m_pattern5x5Sym[13] = 
{
                   0,
             1,  2,  3,
       4,  5,  6,  5,  4,
               3,  2,  1,
             0
};

Int TComAdaptiveLoopFilter::m_weights5x5Sym[8] = 
{
           2, 
        2, 2, 2,
     2, 2, 1, 1
};

Int TComAdaptiveLoopFilter::m_pattern5x5Sym_Quart[45] = 
{
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  1,  0,  0,  0,  0,
   0,  0,  0,  2,  3,  4,  0,  0,  0,
   0,  0,  5,  6,  7,  8,  0,  0,  0,  
};

Int TComAdaptiveLoopFilter::m_pattern9x9Sym_9[39] = 
{
              12, 13, 14,  
          20, 21, 22, 23, 24, 
      28, 29, 30, 31, 32, 33, 34,      
  36, 37, 38, 39, 40, 39, 38, 37, 36, 
      34, 33, 32, 31, 30, 29, 28,  
          24, 23, 22, 21, 20, 
              14, 13, 12,
};

Int TComAdaptiveLoopFilter::m_pattern9x9Sym_7[25] = 
{    
               13,   
           21, 22, 23,  
       29, 30, 31, 32, 33,       
   37, 38, 39, 40, 39, 38, 37,  
       33, 32, 31, 30, 29,   
           23, 22, 21,  
               13  
                     
};

Int TComAdaptiveLoopFilter::m_pattern9x9Sym_5[13] = 
{
          22, 
      30, 31, 32,    
  38, 39, 40, 39, 38, 
      32, 31, 30, 
          22,  
 };

Int* TComAdaptiveLoopFilter::m_patternTab_filt[NO_TEST_FILT] =
{
  m_pattern9x9Sym_9, m_pattern9x9Sym_7, m_pattern9x9Sym_5
}; 

Int* TComAdaptiveLoopFilter::m_patternTab[NO_TEST_FILT] =
{
  m_pattern9x9Sym, m_pattern7x7Sym, m_pattern5x5Sym
}; 

Int* TComAdaptiveLoopFilter::m_patternMapTab[NO_TEST_FILT] =
{
  m_pattern9x9Sym_Quart, m_pattern7x7Sym_Quart, m_pattern5x5Sym_Quart
};

Int* TComAdaptiveLoopFilter::m_weightsTab[NO_TEST_FILT] =
{
  m_weights9x9Sym, m_weights7x7Sym, m_weights5x5Sym
};

Int TComAdaptiveLoopFilter::m_flTab[NO_TEST_FILT] =
{
  9/2, 7/2, 5/2
};

Int TComAdaptiveLoopFilter::m_sqrFiltLengthTab[NO_TEST_FILT] =
{
  SQR_FILT_LENGTH_9SYM, SQR_FILT_LENGTH_7SYM, SQR_FILT_LENGTH_5SYM
};

Int depthInt9x9Sym[21] = 
{
           5, 6, 5, 
        5, 6, 7, 6, 5,
     5, 6, 7, 8, 7, 6, 5,
  5, 6, 7, 8, 9, 9 
};

Int depthInt7x7Sym[14] = 
{
           4, 
        4, 5, 4, 
     4, 5, 6, 5, 4, 
  4, 5, 6, 7, 7 
};

Int depthInt5x5Sym[8] = 
{
        3,   
     3, 4, 3,
  3, 4, 5, 5  
};

Int* pDepthIntTab[NO_TEST_FILT] =
{
  depthInt5x5Sym, depthInt7x7Sym, depthInt9x9Sym
};

// scaling factor for quantization of filter coefficients (9x9)
const Int TComAdaptiveLoopFilter::m_aiSymmetricMag9x9[41] =
{
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 1
};

// scaling factor for quantization of filter coefficients (7x7)
const Int TComAdaptiveLoopFilter::m_aiSymmetricMag7x7[25] =
{
  2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 1
};

// scaling factor for quantization of filter coefficients (5x5)
const Int TComAdaptiveLoopFilter::m_aiSymmetricMag5x5[13] =
{
  2, 2, 2, 2, 2,
  2, 2, 2, 2, 2,
  2, 2, 1
};

const Int TComAdaptiveLoopFilter::m_aiSymmetricMag9x7[32] =
{
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 1
};

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComAdaptiveLoopFilter::TComAdaptiveLoopFilter()
{
  m_img_height = 0;
  m_img_width = 0;
  m_nInputBitDepth = 0;
  m_pcTempPicYuv = NULL;
  m_imgY_var = NULL;
  m_imgY_temp = NULL;
  m_imgY_ver = NULL;
  m_imgY_hor = NULL;
  m_filterCoeffSym = NULL;
  m_filterCoeffPrevSelected = NULL;
  m_filterCoeffTmp = NULL;
  m_filterCoeffSymTmp = NULL;
  m_varImgMethods = NULL;
  m_filterCoeffShort = NULL;
  m_alfClipTable = NULL;
  m_alfClipOffset = 0;
}

Void TComAdaptiveLoopFilter:: xError(const char *text, int code)
{
  fprintf(stderr, "%s\n", text);
  exit(code);
}

Void TComAdaptiveLoopFilter:: no_mem_exit(const char *where)
{
  char errortext[200];
  sprintf(errortext, "Could not allocate memory: %s",where);
  xError (errortext, 100);
}

Void TComAdaptiveLoopFilter::initMatrix_imgpel(imgpel ***m2D, int d1, int d2)
{
  int i;
  
  if(!(*m2D = (imgpel **) calloc(d1, sizeof(imgpel *))))
    FATAL_ERROR_0("initMatrix_imgpel: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (imgpel *) calloc(d1 * d2, sizeof(imgpel))))
    FATAL_ERROR_0("initMatrix_imgpel: memory allocation problem\n", -1);
  
  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

Void TComAdaptiveLoopFilter::initMatrix_int(int ***m2D, int d1, int d2)
{
  int i;
  
  if(!(*m2D = (int **) calloc(d1, sizeof(int *))))
    FATAL_ERROR_0("initMatrix_int: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (int *) calloc(d1 * d2, sizeof(int))))
    FATAL_ERROR_0("initMatrix_int: memory allocation problem\n", -1);
  
  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

Void TComAdaptiveLoopFilter::initMatrix_short(short ***m2D, int d1, int d2)
{
  int i;
  
  if(!(*m2D = (short **) calloc(d1, sizeof(short *))))
    FATAL_ERROR_0("initMatrix_int: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (short *) calloc(d1 * d2, sizeof(short))))
    FATAL_ERROR_0("initMatrix_int: memory allocation problem\n", -1);
  
  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

Void TComAdaptiveLoopFilter::destroyMatrix_short(short **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_int: memory free problem\n", -1);
    free(m2D);
  } 
}

Void TComAdaptiveLoopFilter::destroyMatrix_int(int **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_int: memory free problem\n", -1);
    free(m2D);
  } 
}

Void TComAdaptiveLoopFilter::destroyMatrix_imgpel(imgpel **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_imgpel: memory free problem\n", -1);
    free(m2D);
  } 
}

Void TComAdaptiveLoopFilter::get_mem2Dpel(imgpel ***array2D, int rows, int columns)
{
  int i;
  
  if((*array2D      = (imgpel**)calloc(rows,        sizeof(imgpel*))) == NULL)
    no_mem_exit("get_mem2Dpel: array2D");
  if(((*array2D)[0] = (imgpel* )calloc(rows*columns,sizeof(imgpel ))) == NULL)
    no_mem_exit("get_mem2Dpel: array2D");
  
  for(i=1 ; i<rows ; i++)
    (*array2D)[i] =  (*array2D)[i-1] + columns  ;
}

Void TComAdaptiveLoopFilter::free_mem2Dpel(imgpel **array2D)
{
  if (array2D)
  {
    if (array2D[0])
      free (array2D[0]);
    else xError ("free_mem2Dpel: trying to free unused memory",100);
    
    free (array2D);
  }
}

Void TComAdaptiveLoopFilter::initMatrix_double(double ***m2D, int d1, int d2)
{
  int i;
  
  if(!(*m2D = (double **) calloc(d1, sizeof(double *))))
    FATAL_ERROR_0("initMatrix_double: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (double *) calloc(d1 * d2, sizeof(double))))
    FATAL_ERROR_0("initMatrix_double: memory allocation problem\n", -1);
  
  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

Void TComAdaptiveLoopFilter::initMatrix3D_double(double ****m3D, int d1, int d2, int d3)
{
  int  j;
  
  if(!((*m3D) = (double ***) calloc(d1, sizeof(double **))))
    FATAL_ERROR_0("initMatrix3D_double: memory allocation problem\n", -1);
  
  for(j = 0; j < d1; j++)
    initMatrix_double((*m3D) + j, d2, d3);
}


Void TComAdaptiveLoopFilter::initMatrix4D_double(double *****m4D, int d1, int d2, int d3, int d4)
{
  int  j;
  
  if(!((*m4D) = (double ****) calloc(d1, sizeof(double ***))))
    FATAL_ERROR_0("initMatrix4D_double: memory allocation problem\n", -1);
  
  for(j = 0; j < d1; j++)
    initMatrix3D_double((*m4D) + j, d2, d3, d4);
}


Void TComAdaptiveLoopFilter::destroyMatrix_double(double **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_double: memory free problem\n", -1);
    free(m2D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix_double: memory free problem\n", -1);
  }
}

Void TComAdaptiveLoopFilter::destroyMatrix3D_double(double ***m3D, int d1)
{
  int i;
  
  if(m3D)
  {
    for(i = 0; i < d1; i++)
      destroyMatrix_double(m3D[i]);
    free(m3D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix3D_double: memory free problem\n", -1);
  }
}


Void TComAdaptiveLoopFilter::destroyMatrix4D_double(double ****m4D, int d1, int d2)
{
  int  j;
  
  if(m4D)
  {
    for(j = 0; j < d1; j++)
      destroyMatrix3D_double(m4D[j], d2);
    free(m4D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix4D_double: memory free problem\n", -1);
  }
}

Void TComAdaptiveLoopFilter::create( Int iPicWidth, Int iPicHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth , Int nInputBitDepth , Int nInternalBitDepth )
{
  assert( uiMaxCUWidth == g_uiMaxCUWidth && uiMaxCUHeight == g_uiMaxCUHeight && uiMaxCUDepth == g_uiMaxCUDepth );
  if( iPicWidth == m_img_width && iPicHeight == m_img_height && nInputBitDepth == m_nInputBitDepth && nInternalBitDepth == m_nInternalBitDepth )
  {
    m_pcTempPicYuv->setBorderExtension( false );
    return;
  }
  destroy();
  m_nInputBitDepth = nInputBitDepth;
  m_nInternalBitDepth = nInternalBitDepth;
  m_nBitIncrement = nInternalBitDepth - 8; // according to ALF on HM-3
  m_nIBDIMax = ( 1 << m_nInternalBitDepth ) - 1;

  assert( m_nBitIncrement >= 0 );
  if ( !m_pcTempPicYuv )
  {
    m_pcTempPicYuv = new TComPicYuv;
    m_pcTempPicYuv->create( iPicWidth, iPicHeight, uiMaxCUWidth, uiMaxCUHeight, uiMaxCUDepth );
  }
  m_img_height = iPicHeight;
  m_img_width = iPicWidth;
  initMatrix_int(&m_imgY_temp, ALF_WIN_VERSIZE+2*VAR_SIZE+3, ALF_WIN_HORSIZE+2*VAR_SIZE+3);
  initMatrix_int(&m_imgY_ver, ALF_WIN_VERSIZE+2*VAR_SIZE+3, ALF_WIN_HORSIZE+2*VAR_SIZE+3);
  initMatrix_int(&m_imgY_hor, ALF_WIN_VERSIZE+2*VAR_SIZE+3, ALF_WIN_HORSIZE+2*VAR_SIZE+3);
  get_mem2Dpel(&m_varImgMethods, m_img_width, m_img_width);
  
  initMatrix_int(&m_filterCoeffSym, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);
  initMatrix_int(&m_filterCoeffPrevSelected, NO_VAR_BINS, MAX_SQR_FILT_LENGTH); 
  initMatrix_int(&m_filterCoeffTmp, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);      
  initMatrix_int(&m_filterCoeffSymTmp, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);   

  initMatrix_short(&m_filterCoeffShort, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);
}

Void TComAdaptiveLoopFilter::destroy()
{
  if ( m_pcTempPicYuv )
  {
    m_pcTempPicYuv->destroy();
    delete m_pcTempPicYuv;
    m_pcTempPicYuv = NULL;
  }
  destroyMatrix_int(m_imgY_temp);

  destroyMatrix_int(m_imgY_ver);
  destroyMatrix_int(m_imgY_hor);
  free_mem2Dpel(m_varImgMethods);

  destroyMatrix_short(m_filterCoeffShort);

  if( m_alfClipTable )
  {
    free( m_alfClipTable );
    m_alfClipTable = NULL;
  }

  destroyMatrix_int(m_filterCoeffSym);
  destroyMatrix_int(m_filterCoeffPrevSelected);
  destroyMatrix_int(m_filterCoeffTmp);
  destroyMatrix_int(m_filterCoeffSymTmp);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Int TComAdaptiveLoopFilter::ALFTapHToTapV(Int tapH)
{
  return min<UInt>(tapH, 7);
}

Int TComAdaptiveLoopFilter::ALFFlHToFlV(Int flH)
{
  return min<UInt>(flH, 7/2);
}

Int TComAdaptiveLoopFilter::ALFTapHToNumCoeff(Int tapH)
{
  Int num_coeff;
  
  num_coeff = (Int)(tapH*tapH)/4 + 2;
  if (tapH == 9)
    num_coeff -= 1;
  else
    assert(tapH < 9);
  
  return num_coeff;
}

// --------------------------------------------------------------------------------------------------------------------
// allocate / free / copy functions
// --------------------------------------------------------------------------------------------------------------------

Void TComAdaptiveLoopFilter::allocALFParam(ALFParam* pAlfParam)
{
  pAlfParam->alf_flag = 0;
  pAlfParam->cu_control_flag = 0;
  pAlfParam->alf_max_depth = 0;
  
  pAlfParam->coeff        = new Int[ALF_MAX_NUM_COEF];
  pAlfParam->coeff_chroma = new Int[ALF_MAX_NUM_COEF_C];
  
  ::memset(pAlfParam->coeff,        0, sizeof(Int)*ALF_MAX_NUM_COEF   );
  ::memset(pAlfParam->coeff_chroma, 0, sizeof(Int)*ALF_MAX_NUM_COEF_C );
  pAlfParam->coeffmulti = new Int*[NO_VAR_BINS];
  for (int i=0; i<NO_VAR_BINS; i++)
  {
    pAlfParam->coeffmulti[i] = new Int[ALF_MAX_NUM_COEF];
    ::memset(pAlfParam->coeffmulti[i],        0, sizeof(Int)*ALF_MAX_NUM_COEF );
  }
  pAlfParam->num_cus_in_frame = m_uiNumCUsInFrame;
  pAlfParam->num_alf_cu_flag  = 0;
  pAlfParam->alf_cu_flag      = new UInt[(m_uiNumCUsInFrame << ((g_uiMaxCUDepth-1)*2))];
  ::memset(pAlfParam->kMinTab , 0 , sizeof( pAlfParam->kMinTab ) );

#if QC_ALF_TMEPORAL_NUM
  pAlfParam->temproalPredFlag = false;
  pAlfParam->prevIdx = -1;
  pAlfParam->alfCoeffLuma   = new Int*[NO_VAR_BINS];
  for (int i=0; i<NO_VAR_BINS; i++)
  {
    pAlfParam->alfCoeffLuma[i] = new Int[ALF_MAX_NUM_COEF];
    ::memset(pAlfParam->alfCoeffLuma[i],        0, sizeof(Int)*ALF_MAX_NUM_COEF );
  }
  pAlfParam->alfCoeffChroma = new Int[ALF_MAX_NUM_COEF_C];
  ::memset(pAlfParam->alfCoeffChroma,        0, sizeof(Int)*ALF_MAX_NUM_COEF_C );
#endif
}

Void TComAdaptiveLoopFilter::printALFParam(ALFParam* pAlfParam , Bool bDecoder)
{
  Int FiltLengthTab[] = {22, 14, 8}; //0:9tap
  Int FiltLength = FiltLengthTab[pAlfParam->realfiltNo];

  printf( "\nALF param" );

  printf( "\nalf_flag %d" , pAlfParam->alf_flag );
  if( pAlfParam->alf_flag == 0 )
  {
    printf( "\n" );
    return;
  }
  printf( "\nrealfiltNo %d" , pAlfParam->realfiltNo );
  printf( "\nnoFilters %d" , pAlfParam->noFilters - bDecoder );
  printf( "\nfilterPattern" );
  for( Int n = 0 ; n < 16 ; n++ )
    printf( "(%d)%d, " , n , pAlfParam->filterPattern[n] );
  printf( "\npredMethod %d" , pAlfParam->predMethod );
  printf( "\nminKStart %d" , pAlfParam->minKStart );
  printf( "\nkMinTab " );
  for( Int n = 0 ; n < 42 ; n++ )
    printf( "(%d)%d, " , n , pAlfParam->kMinTab[n] );
  for(Int ind=0; ind<pAlfParam->filters_per_group_diff; ind++)
  {
    printf("\ncoeffmulti(%d):", ind);
    for (Int i=0; i<FiltLength; i++) printf("%d ", pAlfParam->coeffmulti[ind][i]);
  }
  printf( "\nchroma_idc %d" , pAlfParam->chroma_idc );
  printf( "\ntap_chroma %d" , pAlfParam->tap_chroma );
  printf( "\ncu_control_flag %d" , pAlfParam->cu_control_flag );
  printf( "\nalf_max_depth %d" , pAlfParam->alf_max_depth );
  printf( "\nnum_alf_cu_flag %d" , pAlfParam->num_alf_cu_flag );

  printf( "\nvarIndTab: " );
  for( Int i = 0 ; i < ALF_HM3_NO_VAR_BIN ; i++ )
    printf( "(%d)%d " , i , pAlfParam->varIndTab[i]  );

  printf("\n");
}

Void TComAdaptiveLoopFilter::freeALFParam(ALFParam* pAlfParam)
{
  assert(pAlfParam != NULL);
  
  if (pAlfParam->coeff != NULL)
  {
    delete[] pAlfParam->coeff;
    pAlfParam->coeff = NULL;
  }
  
  if (pAlfParam->coeff_chroma != NULL)
  {
    delete[] pAlfParam->coeff_chroma;
    pAlfParam->coeff_chroma = NULL;
  }

  if( pAlfParam->coeffmulti != NULL )
  {
    for (int i=0; i<NO_VAR_BINS; i++)
    {
      delete[] pAlfParam->coeffmulti[i];
      pAlfParam->coeffmulti[i] = NULL;
    }
    delete[] pAlfParam->coeffmulti;
    pAlfParam->coeffmulti = NULL;
  }

  if(pAlfParam->alf_cu_flag != NULL)
  {
    delete[] pAlfParam->alf_cu_flag;
    pAlfParam->alf_cu_flag = NULL;
  }
#if QC_ALF_TMEPORAL_NUM
  if( pAlfParam->alfCoeffLuma != NULL )
  {
    for (int i=0; i<NO_VAR_BINS; i++)
    {
      delete[] pAlfParam->alfCoeffLuma[i];
      pAlfParam->alfCoeffLuma[i] = NULL;
    }
    delete[] pAlfParam->alfCoeffLuma;
    pAlfParam->alfCoeffLuma = NULL;
  }

  if( pAlfParam->alfCoeffChroma != NULL )
  {
    delete[] pAlfParam->alfCoeffChroma;
    pAlfParam->alfCoeffChroma = NULL;
  }
#endif
}

Void TComAdaptiveLoopFilter::copyALFParam(ALFParam* pDesAlfParam, ALFParam* pSrcAlfParam)
{
#if QC_ALF_TMEPORAL_NUM
  if( !pDesAlfParam->temproalPredFlag )
  {
#endif
  pDesAlfParam->alf_flag = pSrcAlfParam->alf_flag;
  pDesAlfParam->cu_control_flag = pSrcAlfParam->cu_control_flag;
  pDesAlfParam->chroma_idc = pSrcAlfParam->chroma_idc;
#if QC_ALF_TMEPORAL_NUM
  }
#endif
  pDesAlfParam->tap = pSrcAlfParam->tap;
  pDesAlfParam->tapV = pSrcAlfParam->tapV;
  pDesAlfParam->num_coeff = pSrcAlfParam->num_coeff;
  pDesAlfParam->tap_chroma = pSrcAlfParam->tap_chroma;
  pDesAlfParam->num_coeff_chroma = pSrcAlfParam->num_coeff_chroma;

  ::memcpy(pDesAlfParam->coeff, pSrcAlfParam->coeff, sizeof(Int)*ALF_MAX_NUM_COEF);
  ::memcpy(pDesAlfParam->coeff_chroma, pSrcAlfParam->coeff_chroma, sizeof(Int)*ALF_MAX_NUM_COEF_C);
  pDesAlfParam->realfiltNo = pSrcAlfParam->realfiltNo;
  pDesAlfParam->filtNo = pSrcAlfParam->filtNo;
  ::memcpy(pDesAlfParam->filterPattern, pSrcAlfParam->filterPattern, sizeof(Int)*NO_VAR_BINS);
  pDesAlfParam->startSecondFilter = pSrcAlfParam->startSecondFilter;
  pDesAlfParam->noFilters = pSrcAlfParam->noFilters;
  
  //Coeff send related
  pDesAlfParam->filters_per_group_diff = pSrcAlfParam->filters_per_group_diff; //this can be updated using codedVarBins
  pDesAlfParam->filters_per_group = pSrcAlfParam->filters_per_group; //this can be updated using codedVarBins
  ::memcpy(pDesAlfParam->codedVarBins, pSrcAlfParam->codedVarBins, sizeof(Int)*NO_VAR_BINS);
  pDesAlfParam->forceCoeff0 = pSrcAlfParam->forceCoeff0;
  pDesAlfParam->predMethod = pSrcAlfParam->predMethod;
  for (int i=0; i<NO_VAR_BINS; i++)
  {
    ::memcpy(pDesAlfParam->coeffmulti[i], pSrcAlfParam->coeffmulti[i], sizeof(Int)*ALF_MAX_NUM_COEF);
#if QC_ALF_TMEPORAL_NUM
    ::memcpy(pDesAlfParam->alfCoeffLuma[i], pSrcAlfParam->alfCoeffLuma[i], sizeof(Int)*ALF_MAX_NUM_COEF);
#endif
  }
#if QC_ALF_TMEPORAL_NUM
  ::memcpy(pDesAlfParam->alfCoeffChroma, pSrcAlfParam->alfCoeffChroma, sizeof(Int)*ALF_MAX_NUM_COEF_C);
#endif
  pDesAlfParam->minKStart = pSrcAlfParam->minKStart;
  ::memcpy( pDesAlfParam->kMinTab , pSrcAlfParam->kMinTab , sizeof( pSrcAlfParam->kMinTab ) );

  ::memcpy( pDesAlfParam->varIndTab , pSrcAlfParam->varIndTab , sizeof( pSrcAlfParam->varIndTab ) );

#if QC_ALF_TMEPORAL_NUM
  if( !pDesAlfParam->temproalPredFlag )
  {
#endif
  pDesAlfParam->num_alf_cu_flag = pSrcAlfParam->num_alf_cu_flag;
  ::memcpy(pDesAlfParam->alf_cu_flag, pSrcAlfParam->alf_cu_flag, sizeof(UInt)*pSrcAlfParam->num_alf_cu_flag);
#if QC_ALF_TMEPORAL_NUM
  }
#endif
}

Void TComAdaptiveLoopFilter::resetALFParam(ALFParam* pDesAlfParam)
{
  pDesAlfParam->alf_flag = 0;
  pDesAlfParam->cu_control_flag = 0;
  pDesAlfParam->chroma_idc = 0;
  pDesAlfParam->tap = 0;
  pDesAlfParam->tapV = 0;
  pDesAlfParam->num_coeff = 0;
  pDesAlfParam->tap_chroma = 0;
  pDesAlfParam->num_coeff_chroma = 0;

  ::memset(pDesAlfParam->coeff, 0, sizeof(Int)*ALF_MAX_NUM_COEF);
  ::memset(pDesAlfParam->coeff_chroma, 0, sizeof(Int)*ALF_MAX_NUM_COEF_C);
  pDesAlfParam->realfiltNo = 0;
  pDesAlfParam->filtNo = 0;
  ::memset(pDesAlfParam->filterPattern, 0, sizeof(Int)*NO_VAR_BINS);
  pDesAlfParam->startSecondFilter = 0;
  pDesAlfParam->noFilters = 0;
  ::memset(pDesAlfParam->kMinTab , 0 , sizeof( pDesAlfParam->kMinTab ) );

  //Coeff send related
  pDesAlfParam->filters_per_group_diff = 0; //this can be updated using codedVarBins
  pDesAlfParam->filters_per_group = 0; //this can be updated using codedVarBins
  ::memset(pDesAlfParam->codedVarBins, 0, sizeof(Int)*NO_VAR_BINS);
  pDesAlfParam->forceCoeff0 = 0;
  pDesAlfParam->predMethod = 0;
  for (int i=0; i<NO_VAR_BINS; i++)
  {
    ::memset(pDesAlfParam->coeffmulti[i], 0, sizeof(Int)*ALF_MAX_NUM_COEF);
#if QC_ALF_TMEPORAL_NUM
    ::memset(pDesAlfParam->alfCoeffLuma[i], 0, sizeof(Int)*ALF_MAX_NUM_COEF);
#endif
  }
#if QC_ALF_TMEPORAL_NUM
  ::memset(pDesAlfParam->alfCoeffChroma, 0, sizeof(Int)*ALF_MAX_NUM_COEF_C);
  pDesAlfParam->temproalPredFlag = false;
  pDesAlfParam->prevIdx = -1;
#endif
  ::memset( pDesAlfParam->varIndTab , 0 , sizeof( pDesAlfParam->varIndTab ) );

  pDesAlfParam->num_alf_cu_flag = 0;
}

// --------------------------------------------------------------------------------------------------------------------
// prediction of filter coefficients
// --------------------------------------------------------------------------------------------------------------------

Void TComAdaptiveLoopFilter::predictALFCoeff( ALFParam* pAlfParam)
{
  Int i, sum, pred, tap, N;
  const Int* pFiltMag = NULL;
  
  tap = pAlfParam->tap;
  Int tapV = pAlfParam->tapV;
  
  switch(tap)
  {
    case 5:
      pFiltMag = m_aiSymmetricMag5x5;
      break;
    case 7:
      pFiltMag = m_aiSymmetricMag7x7;
      break;
    case 9:
      pFiltMag = m_aiSymmetricMag9x7;
      break;
    default:
      assert(0);
      break;
  }
  N = (tap * tapV + 1) >> 1;
  sum=0;
  for(i=0; i<N-1;i++)
  {
    sum+=pFiltMag[i]*pAlfParam->coeff[i];
  }
  pred=(1<<ALF_NUM_BIT_SHIFT)-sum;
  pAlfParam->coeff[N-1]=pred-pAlfParam->coeff[N-1];
}

Void TComAdaptiveLoopFilter::predictALFCoeffChroma( ALFParam* pAlfParam )
{
  Int i, sum, pred, tap, N;
  const Int* pFiltMag = NULL;
  
  tap = pAlfParam->tap_chroma;
  switch(tap)
  {
    case 5:
      pFiltMag = m_aiSymmetricMag5x5;
      break;
    case 7:
      pFiltMag = m_aiSymmetricMag7x7;
      break;
    case 9:
      pFiltMag = m_aiSymmetricMag9x9;
      break;
    default:
      assert(0);
      break;
  }
  N = (tap*tap+1)>>1;
  sum=0;
  for(i=0; i<N;i++)
  {
    sum+=pFiltMag[i]*pAlfParam->coeff_chroma[i];
  }
  pred=(1<<ALF_NUM_BIT_SHIFT)-(sum-pAlfParam->coeff_chroma[N-1]);
  pAlfParam->coeff_chroma[N-1]=pred-pAlfParam->coeff_chroma[N-1];
}

// --------------------------------------------------------------------------------------------------------------------
// interface function for actual ALF process
// --------------------------------------------------------------------------------------------------------------------

/**
 \param pcPic         picture (TComPic) class (input/output)
 \param pcAlfParam    ALF parameter
 \todo   for temporal buffer, it uses residual picture buffer, which may not be safe. Make it be safer.
 */
Void TComAdaptiveLoopFilter::ALFProcess(TComPic* pcPic, ALFParam* pcAlfParam)
{
  if(!pcAlfParam->alf_flag)
  {
    return;
  }
  
  m_pcTempPicYuv = pcPic->replacePicYuvRecPointer( m_pcTempPicYuv );
  TComPicYuv* pcPicYuvRec    = pcPic->getPicYuvRec();
  TComPicYuv* pcPicYuvExtRec = m_pcTempPicYuv;
  pcPicYuvExtRec->setBorderExtension ( false );
  pcPicYuvExtRec->extendPicBorder    (FILTER_LENGTH >> 1);

  if(pcAlfParam->cu_control_flag)
  {
    UInt idx = 0;
    for(UInt uiCUAddr = 0; uiCUAddr < pcPic->getNumCUsInFrame(); uiCUAddr++)
    {
      TComDataCU *pcCU = pcPic->getCU(uiCUAddr);
      setAlfCtrlFlags(pcAlfParam, pcCU, 0, 0, idx);
    }
  }
  xALFLuma_qc(pcPic, pcAlfParam, pcPicYuvExtRec, pcPicYuvRec);
  
  if(pcAlfParam->chroma_idc)
  {
#if QC_ALF_TMEPORAL_NUM
    if( !pcAlfParam->temproalPredFlag )
#endif
    predictALFCoeffChroma(pcAlfParam);
#if QC_ALF_TMEPORAL_NUM
    memcpy( pcAlfParam->alfCoeffChroma, pcAlfParam->coeff_chroma, sizeof(Int)*ALF_MAX_NUM_COEF_C );
#endif
    xALFChroma( pcAlfParam, pcPicYuvExtRec, pcPicYuvRec);
  }
  else
  {
    pcPicYuvExtRec->copyToPicCb( pcPicYuvRec , false );
    pcPicYuvExtRec->copyToPicCr( pcPicYuvRec , false );
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// ALF for luma
// --------------------------------------------------------------------------------------------------------------------
Void TComAdaptiveLoopFilter::xALFLuma_qc(TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
  Int    LumaStride = pcPicDec->getStride();
  imgpel* pDec = (imgpel*)pcPicDec->getLumaAddr();
  imgpel* pRest = (imgpel*)pcPicRest->getLumaAddr();
  
  //Decode and reconst filter coefficients
  DecFilter_qc(pDec,pcAlfParam,LumaStride);
  //set maskImg using cu adaptive one.

  m_imgY_var       = m_varImgMethods;

  if(pcAlfParam->cu_control_flag)
  {
    xCUAdaptive_qc(pcPic, pcAlfParam, pRest, pDec, LumaStride);
  }  
  else
  {
    //then do whole frame filtering
    filterFrame(pRest, pDec, pcAlfParam->realfiltNo, LumaStride);
  }
}


Void TComAdaptiveLoopFilter::DecFilter_qc(imgpel* imgY_rec,ALFParam* pcAlfParam, int Stride)
{
  int i;
  int numBits = NUM_BITS; 
  int **pfilterCoeffSym;
  pfilterCoeffSym= m_filterCoeffSym;
#if QC_ALF_TMEPORAL_NUM
  if( pcAlfParam->temproalPredFlag )
  {
    for(i = 0; i < NO_VAR_BINS; i++)
    {
      memcpy(pfilterCoeffSym[i], &pcAlfParam->alfCoeffLuma[i][0] ,sizeof(int)*MAX_SQR_FILT_LENGTH);
    }

    int k, varInd;
    int *patternMap;
    int *patternMapTab[3]={m_pattern9x9Sym_Quart, m_pattern7x7Sym_Quart, m_pattern5x5Sym_Quart};
    {
      for(varInd=0; varInd<NO_VAR_BINS; ++varInd)
      {
        memset(m_filterCoeffPrevSelected[varInd],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
      }
      patternMap=patternMapTab[pcAlfParam->realfiltNo];
      for(varInd=0; varInd<NO_VAR_BINS; ++varInd)
      {
        k=0;
        for(i = 0; i < MAX_SQR_FILT_LENGTH; i++)
        {
          if (patternMap[i]>0)
          {
            m_filterCoeffPrevSelected[varInd][i]=pcAlfParam->alfCoeffLuma[varInd][k];
            k++;
          }
          else
          {
            m_filterCoeffPrevSelected[varInd][i]=0;
          }
        }
      }
    }
  }
  else
  {
#endif
  if(pcAlfParam->filtNo>=0)
  {
    //// Reconstruct filter coefficients
    reconstructFilterCoeffs( pcAlfParam, pfilterCoeffSym, numBits);
  }
  else
  {
    for(i = 0; i < NO_VAR_BINS; i++)
    {
      pcAlfParam->varIndTab[i]=0;
      memset(pfilterCoeffSym[i],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
    }
  }
  getCurrentFilter(pfilterCoeffSym,pcAlfParam);
#if QC_ALF_TMEPORAL_NUM
  }
#endif

  Int *coef;
  Int maxPxlVal = m_nIBDIMax;
  Int maxSampleValue, minSampleValue = 0;
  Int clipRange[2] = { 0, 0 }, flipTableSize;
  Int sumCoef[2];
  Int numBitsMinus1= NUM_BITS-1;
  Int offset = (1<<(NUM_BITS-2));
  Int lastCoef = MAX_SQR_FILT_LENGTH-1;
  Int centerCoef = MAX_SQR_FILT_LENGTH-2;

  for(Int varInd=0; varInd<NO_VAR_BINS; ++varInd)
  {
    coef = m_filterCoeffPrevSelected[varInd];
    sumCoef[0] = 0;
    sumCoef[1] = 0;
    for(i = 0; i < centerCoef; i++)
    {
      assert( coef[i]<=32767 && coef[i]>=-32768 );
      m_filterCoeffShort[varInd][i] = (Short)coef[i];
      sumCoef[ coef[i] > 0 ? 0 : 1 ] += (coef[i] << 1);
    }
    assert( coef[centerCoef]<=32767 && coef[centerCoef]>=-32768 );
    assert( coef[lastCoef]  <=32767 && coef[lastCoef]  >=-32768 );

    m_filterCoeffShort[varInd][centerCoef] = (Short)coef[centerCoef];
    m_filterCoeffShort[varInd][lastCoef]   = (Short)coef[lastCoef];

    sumCoef[ coef[centerCoef] > 0 ? 0 : 1 ] += coef[centerCoef];

    maxSampleValue = ( maxPxlVal * sumCoef[0] + coef[lastCoef] + offset ) >> numBitsMinus1;
    minSampleValue = ( maxPxlVal * sumCoef[1] + coef[lastCoef] + offset ) >> numBitsMinus1;

    if( clipRange[0] < maxSampleValue )
    {
      clipRange[0] = maxSampleValue;
    }
    if( clipRange[1] > minSampleValue )
    {
      clipRange[1] = minSampleValue;
    }
  }

  Int clipTableMax = ( ( ALF_HM3_QC_CLIP_RANGE - ALF_HM3_QC_CLIP_OFFSET - 1 ) << (g_bitDepthY-8) );
  Int clipTableMin = ( ( - ALF_HM3_QC_CLIP_OFFSET ) << (g_bitDepthY-8) );

  assert( clipRange[0]<=clipTableMax && clipRange[1]>=clipTableMin );

  flipTableSize   = (ALF_HM3_QC_CLIP_RANGE <<(g_bitDepthY-8));
  m_alfClipOffset = (ALF_HM3_QC_CLIP_OFFSET<<(g_bitDepthY-8));

  if( m_alfClipTable )
  {
    free(m_alfClipTable);
  }

  m_alfClipTable = (imgpel *)calloc(flipTableSize, sizeof(imgpel));

  for(Int k=0; k< flipTableSize; k++)
  {
    m_alfClipTable[k] = max( 0, min( k-m_alfClipOffset, maxPxlVal ) );
  }

  memset(m_imgY_temp[0],0,sizeof(int)*(ALF_WIN_VERSIZE+2*VAR_SIZE)*(ALF_WIN_HORSIZE+2*VAR_SIZE));
  m_imgY_var       = m_varImgMethods;
}

Void TComAdaptiveLoopFilter::getCurrentFilter(int **filterCoeffSym,ALFParam* pcAlfParam)
{ 
  int i,  k, varInd;
  int *patternMap;
  int *patternMapTab[3]={m_pattern9x9Sym_Quart, m_pattern7x7Sym_Quart, m_pattern5x5Sym_Quart};
  {
    for(varInd=0; varInd<NO_VAR_BINS; ++varInd)
    {
      memset(m_filterCoeffPrevSelected[varInd],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
    }
    patternMap=patternMapTab[pcAlfParam->realfiltNo];
    for(varInd=0; varInd<NO_VAR_BINS; ++varInd)
    {
      k=0;
      for(i = 0; i < MAX_SQR_FILT_LENGTH; i++)
      {
        if (patternMap[i]>0)
        {
          m_filterCoeffPrevSelected[varInd][i]=filterCoeffSym[pcAlfParam->varIndTab[varInd]][k];
#if QC_ALF_TMEPORAL_NUM
          pcAlfParam->alfCoeffLuma[varInd][k] = m_filterCoeffPrevSelected[varInd][i];
#endif
          k++;
        }
        else
        {
          m_filterCoeffPrevSelected[varInd][i]=0;
        }
      }
    }
  }
}

Void TComAdaptiveLoopFilter::reconstructFilterCoeffs(ALFParam* pcAlfParam,int **pfilterCoeffSym, int bit_depth)
{
  int i, src, ind;
  
  // Copy non zero filters in filterCoeffTmp
  for(ind = 0; ind < pcAlfParam->filters_per_group_diff; ++ind)
  {
    for(i = 0; i < pcAlfParam->num_coeff; i++)
      m_filterCoeffTmp[ind][i] = pcAlfParam->coeffmulti[ind][i];
  }
  // Undo prediction
  for(ind = 0; ind < pcAlfParam->filters_per_group_diff; ++ind)
  {
    if((!pcAlfParam->predMethod) || (ind == 0)) 
    {
      memcpy(m_filterCoeffSymTmp[ind],m_filterCoeffTmp[ind],sizeof(int)*pcAlfParam->num_coeff);
    }
    else
    {
      // Prediction
      for(i = 0; i < pcAlfParam->num_coeff; ++i)
        m_filterCoeffSymTmp[ind][i] = (int)(m_filterCoeffTmp[ind][i] + m_filterCoeffSymTmp[ind - 1][i]);
    }
  }
  
  // Inverse quantization
  // Add filters forced to zero
  if(pcAlfParam->forceCoeff0)
  {
    assert(pcAlfParam->filters_per_group_diff < pcAlfParam->filters_per_group);
    src = 0;
    for(ind = 0; ind < pcAlfParam->filters_per_group; ++ind)
    {
      if(pcAlfParam->codedVarBins[ind])
      {
        memcpy(pfilterCoeffSym[ind],m_filterCoeffSymTmp[src],sizeof(int)*pcAlfParam->num_coeff);
        ++src;
      }
      else
      {
        memset(pfilterCoeffSym[ind],0,sizeof(int)*pcAlfParam->num_coeff);
      }
    }
    assert(src == pcAlfParam->filters_per_group_diff);
  }
  else
  {
    assert(pcAlfParam->filters_per_group_diff == pcAlfParam->filters_per_group);
    for(ind = 0; ind < pcAlfParam->filters_per_group; ++ind)
      memcpy(pfilterCoeffSym[ind],m_filterCoeffSymTmp[ind],sizeof(int)*pcAlfParam->num_coeff);
  }
}


static imgpel Clip_post(int high, int val)
{
  return (imgpel)(((val > high)? high: val));
}

Void TComAdaptiveLoopFilter::calcVar(imgpel **imgY_var, imgpel *imgY_pad, int pad_size, int fl, int img_height, int img_width, int img_stride, int start_width , int start_height )
{
  Int i, j;

  Int end_height = start_height + img_height;
  Int end_width  = start_width + img_width;

  for(i = start_height; i < end_height; i+=ALF_WIN_VERSIZE)
  {
    for(j = start_width; j < end_width; j+=ALF_WIN_HORSIZE) 
    {
      Int nHeight = min( i + ALF_WIN_VERSIZE, end_height ) - i;
      Int nWidth  = min( j + ALF_WIN_HORSIZE, end_width  ) - j;
      xCalcVar( imgY_var, imgY_pad, pad_size, fl, nHeight, nWidth, img_stride, j, i );
    }
  }
}

Void TComAdaptiveLoopFilter::xCalcVar(imgpel **imgY_var, imgpel *imgY_pad, int pad_size, int fl, int img_height, int img_width, int img_stride, int start_width , int start_height )
{
  static Int shift_h     = (Int)(log((double)ALF_HM3_VAR_SIZE_H)/log(2.0));
  static Int shift_w     = (Int)(log((double)ALF_HM3_VAR_SIZE_W)/log(2.0));

  Int i, j;
  Int *p_imgY_temp;
#if FULL_NBIT
  Int shift= (11+ m_nBitIncrement + m_nInputBitDepth - 8);
#else
  Int shift= (11+ m_nBitIncrement);
#endif
  Int fl2plusOne= (VAR_SIZE<<1)+1; //3
  Int pad_offset = pad_size-fl-1;
  Int var_max= NO_VAR_BINS-1;
  Int mult_fact_int_tab[4]= {1,114,41,21};
  Int mult_fact_int = mult_fact_int_tab[VAR_SIZE];
  Int avg_var;
  Int vertical, horizontal;
  Int direction;
  Int step1 = NO_VAR_BINS/3 - 1;
  Int th[NO_VAR_BINS] = {0, 1, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4}; 
  Int pixY;

  for(i = 1; i < img_height + fl2plusOne; i++)
  {
    Int yoffset = (pad_offset+i+start_height-pad_size) * img_stride + pad_offset-pad_size;
    imgpel *p_imgY_pad = &imgY_pad[yoffset];
    imgpel *p_imgY_pad_up   = &imgY_pad[yoffset + img_stride];
    imgpel *p_imgY_pad_down = &imgY_pad[yoffset - img_stride];
    p_imgY_temp = (Int*)&m_imgY_temp[i-1][0];
    for(j = 1; j < img_width +fl2plusOne; j++)  
    {
      pixY = j + start_width;
      vertical = abs((p_imgY_pad[pixY]<<1) - p_imgY_pad_down[pixY] - p_imgY_pad_up[pixY]);
      horizontal = abs((p_imgY_pad[pixY]<<1) - p_imgY_pad[pixY+1] - p_imgY_pad[pixY-1]);
      m_imgY_ver[i-1][j-1] = vertical;
      m_imgY_hor[i-1][j-1] = horizontal;
      *(p_imgY_temp++) = vertical + horizontal;
    }

    for(j = 1; j < img_width + fl2plusOne; j=j+4)  
    {
      m_imgY_temp [i-1][j] =  (m_imgY_temp [i-1][j-1] + m_imgY_temp [i-1][j+4])
        + ((m_imgY_temp [i-1][j] + m_imgY_temp [i-1][j+3]) << 1)
        + ((m_imgY_temp [i-1][j+1] + m_imgY_temp [i-1][j+2]) * 3);
      m_imgY_ver[i-1][j] = m_imgY_ver[i-1][j] + m_imgY_ver[i-1][j+1] + m_imgY_ver[i-1][j+2] + m_imgY_ver[i-1][j+3];      
      m_imgY_hor[i-1][j] = m_imgY_hor[i-1][j] + m_imgY_hor[i-1][j+1] + m_imgY_hor[i-1][j+2] + m_imgY_hor[i-1][j+3];    
    }
  }

  for(i = 1; i < img_height + 1; i=i+4)
  {
    for(j = 1; j < img_width + 1; j=j+4)  
    {
      m_imgY_temp [i-1][j-1] =  (m_imgY_temp [i-1][j]+m_imgY_temp [i+4][j])
        + ((m_imgY_temp [i][j]+m_imgY_temp [i+3][j]) << 1)
        + ((m_imgY_temp [i+1][j]+m_imgY_temp [i+2][j]) * 3);

      m_imgY_ver[i-1][j-1] = m_imgY_ver[i][j] + m_imgY_ver[i+1][j] + m_imgY_ver[i+2][j] + m_imgY_ver[i+3][j];  
      m_imgY_hor[i-1][j-1] = m_imgY_hor[i][j] + m_imgY_hor[i+1][j] + m_imgY_hor[i+2][j] + m_imgY_hor[i+3][j];    
      avg_var = m_imgY_temp [i-1][j-1]>>(shift_h + shift_w);
      avg_var = (imgpel) Clip_post(var_max, (avg_var * mult_fact_int)>>shift);
      avg_var = th[avg_var];

      direction = 0;
      if (m_imgY_ver[i-1][j-1] > 2*m_imgY_hor[i-1][j-1]) direction = 1; //vertical
      if (m_imgY_hor[i-1][j-1] > 2*m_imgY_ver[i-1][j-1]) direction = 2; //horizontal

      avg_var = Clip_post(step1, (Int) avg_var ) + (step1+1)*direction; 
      imgY_var[(i + start_height - 1)>>shift_h][(j + start_width - 1)>>shift_w] = avg_var;
    }
  }
}

Void TComAdaptiveLoopFilter::filterFrame(imgpel *imgYRecPost, imgpel *imgYRec, int filtNo, int stride)
{
  Int i, j;
  for (i = 0; i < m_img_height; i+=ALF_WIN_VERSIZE)
  {
    for (j = 0; j < m_img_width; j+=ALF_WIN_HORSIZE)
    {
      Int nHeight = min( i + ALF_WIN_VERSIZE, m_img_height ) - i;
      Int nWidth  = min( j + ALF_WIN_HORSIZE, m_img_width  ) - j;

      calcVar( m_imgY_var, imgYRec, FILTER_LENGTH/2, VAR_SIZE, nHeight, nWidth, stride , j , i );
      subfilterFrame(imgYRecPost, imgYRec, filtNo, i, i + nHeight, j, j + nWidth, stride );
    }
  }
}

Void TComAdaptiveLoopFilter::subfilterFrame(imgpel *imgYRecPost, imgpel *imgYRec, int filtNo, int startHeight, int endHeight, int startWidth, int endWidth, int stride)
{
  Int varStepSizeWidth = ALF_HM3_VAR_SIZE_W;
  Int varStepSizeHeight = ALF_HM3_VAR_SIZE_H;
  Int shiftHeight = (Int)(log((double)varStepSizeHeight)/log(2.0));
  Int shiftWidth = (Int)(log((double)varStepSizeWidth)/log(2.0));
  Int i, j, pixelInt;
  imgpel *pImgYVar,*pImgYPad;
  imgpel *pImgYPad1,*pImgYPad2,*pImgYPad3,*pImgYPad4,*pImgYPad5,*pImgYPad6;

  Short *coef = m_filterCoeffShort[0];
  imgpel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;
  imgpel *pImgYRec;
  imgpel *pClipTable = m_alfClipTable + m_alfClipOffset;

  Int numBitsMinus1= NUM_BITS-1;
  Int offset = (1<<(NUM_BITS-2));
  
  imgYRecPost += startHeight*stride;

  switch(filtNo)
  {
  case 2:
    for (i =  startHeight; i < endHeight; i++)
    {
      pImgYVar = m_imgY_var[i>>shiftHeight] + (startWidth>>shiftWidth);
      pImgYPad = imgYRec + i*stride;
      pImgYPad1 = imgYRec + (i+1)*stride;
      pImgYPad2 = imgYRec + (i-1)*stride;
      pImgYPad3 = imgYRec + (i+2)*stride;
      pImgYPad4 = imgYRec + (i-2)*stride;

      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
        if ((j&(varStepSizeWidth-1))==0) coef = m_filterCoeffShort[*(pImgYVar++)];
        pixelInt = coef[MAX_SQR_FILT_LENGTH-1];

        pImg0 = pImgYPad  + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;

        pixelInt += coef[22]* (pImg3[+0]+pImg4[+0]);

        pixelInt += coef[30]* (pImg1[+1]+pImg2[-1]);
        pixelInt += coef[31]* (pImg1[+0]+pImg2[+0]);
        pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);

        pixelInt += coef[38]* (pImg0[-2]+pImg0[+2]);
        pixelInt += coef[39]* (pImg0[-1]+pImg0[+1]);
        pixelInt += coef[40]* (pImg0[+0]);

        pixelInt=(Int)((pixelInt+offset) >> (numBitsMinus1));
        *(pImgYRec++) = pClipTable[pixelInt];
      }
      imgYRecPost += stride;
    }
    break;

  case 1:
    for (i =  startHeight; i < endHeight; i++)
    {
      pImgYVar = m_imgY_var[i>>shiftHeight] + (startWidth>>shiftWidth);
      pImgYPad = imgYRec + i*stride;
      pImgYPad1 = imgYRec + (i+1)*stride;
      pImgYPad2 = imgYRec + (i-1)*stride;
      pImgYPad3 = imgYRec + (i+2)*stride;
      pImgYPad4 = imgYRec + (i-2)*stride;
      pImgYPad5 = imgYRec + (i+3)*stride;
      pImgYPad6 = imgYRec + (i-3)*stride;

      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
        if ((j&(varStepSizeWidth-1))==0) coef = m_filterCoeffShort[*(pImgYVar++)];
        pixelInt = coef[MAX_SQR_FILT_LENGTH-1];

        pImg0 = pImgYPad  + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;
        pImg5 = pImgYPad5 + j;
        pImg6 = pImgYPad6 + j;

        pixelInt += coef[13]* (pImg5[0]+pImg6[0]);

        pixelInt += coef[21]* (pImg3[+1]+pImg4[-1]);
        pixelInt += coef[22]* (pImg3[+0]+pImg4[+0]);
        pixelInt += coef[23]* (pImg3[-1]+pImg4[+1]);

        pixelInt += coef[29]* (pImg1[+2]+pImg2[-2]);
        pixelInt += coef[30]* (pImg1[+1]+pImg2[-1]);
        pixelInt += coef[31]* (pImg1[+0]+pImg2[+0]);
        pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);
        pixelInt += coef[33]* (pImg1[-2]+pImg2[+2]);

        pixelInt += coef[37]* (pImg0[+3]+pImg0[-3]);
        pixelInt += coef[38]* (pImg0[+2]+pImg0[-2]);
        pixelInt += coef[39]* (pImg0[+1]+pImg0[-1]);
        pixelInt += coef[40]* (pImg0[+0]);

        pixelInt=(Int)((pixelInt+offset) >> (numBitsMinus1));
        *(pImgYRec++) = pClipTable[pixelInt];
      }
      imgYRecPost += stride;
    }
    break;

  case 0:
    for (i =  startHeight; i < endHeight; i++)
    {
      pImgYVar = m_imgY_var[i>>shiftHeight] + (startWidth>>shiftWidth);
      pImgYPad = imgYRec + i*stride;
      pImgYPad1 = imgYRec + (i+1)*stride;
      pImgYPad2 = imgYRec + (i-1)*stride;
      pImgYPad3 = imgYRec + (i+2)*stride;
      pImgYPad4 = imgYRec + (i-2)*stride;
      pImgYPad5 = imgYRec + (i+3)*stride;
      pImgYPad6 = imgYRec + (i-3)*stride;

      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
        if ((j&(varStepSizeWidth-1))==0) coef = m_filterCoeffShort[*(pImgYVar++)];
        pixelInt = coef[MAX_SQR_FILT_LENGTH-1];

        pImg0 = pImgYPad  + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;
        pImg5 = pImgYPad5 + j;
        pImg6 = pImgYPad6 + j;

        pixelInt += coef[12]* (pImg5[1]+pImg6[-1]);
        pixelInt += coef[13]* (pImg5[0]+pImg6[0]);
        pixelInt += coef[14]* (pImg5[-1]+pImg6[1]);

        pixelInt += coef[20]* (pImg3[2]+pImg4[-2]);
        pixelInt += coef[21]* (pImg3[1]+pImg4[-1]);
        pixelInt += coef[22]* (pImg3[0]+pImg4[0]);
        pixelInt += coef[23]* (pImg3[-1]+pImg4[+1]);
        pixelInt += coef[24]* (pImg3[-2]+pImg4[+2]);

        pixelInt += coef[28]* (pImg1[3]+pImg2[-3]);
        pixelInt += coef[29]* (pImg1[2]+pImg2[-2]);
        pixelInt += coef[30]* (pImg1[1]+pImg2[-1]);
        pixelInt += coef[31]* (pImg1[0]+pImg2[0]);
        pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);
        pixelInt += coef[33]* (pImg1[-2]+pImg2[+2]);
        pixelInt += coef[34]* (pImg1[-3]+pImg2[+3]);

        pixelInt += coef[36]* (pImg0[+4]+pImg0[-4]);
        pixelInt += coef[37]* (pImg0[+3]+pImg0[-3]);
        pixelInt += coef[38]* (pImg0[+2]+pImg0[-2]);
        pixelInt += coef[39]* (pImg0[+1]+pImg0[-1]);
        pixelInt += coef[40]* (pImg0[0]);

        pixelInt=(Int)((pixelInt+offset) >> (numBitsMinus1));

        *(pImgYRec++) = pClipTable[pixelInt];
      }
      imgYRecPost += stride;
    }
    break;
  }
}


Void TComAdaptiveLoopFilter::xCUAdaptive_qc(TComPic* pcPic, ALFParam* pcAlfParam, imgpel *imgY_rec_post, imgpel *imgY_rec, Int Stride)
{
  // for every CU, call CU-adaptive ALF process
  for( UInt uiCUAddr = 0; uiCUAddr < pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = pcPic->getCU( uiCUAddr );
    xSubCUAdaptive_qc(pcCU, pcAlfParam, imgY_rec_post, imgY_rec, 0, 0, Stride);
  }
}

Void TComAdaptiveLoopFilter::xSubCUAdaptive_qc(TComDataCU* pcCU, ALFParam* pcAlfParam, imgpel *imgY_rec_post, imgpel *imgY_rec, UInt uiAbsPartIdx, UInt uiDepth, Int Stride)
{
  TComPic* pcPic = pcCU->getPic();
  
  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
  
  // check picture boundary
  if ( ( uiRPelX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) || ( uiBPelY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
  {
    bBoundary = true;
  }
  
  // go to sub-CU?
  if ( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) && uiDepth < pcAlfParam->alf_max_depth ) || bBoundary )
  {
    UInt uiQNumParts = ( pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
      
      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
        xSubCUAdaptive_qc(pcCU, pcAlfParam, imgY_rec_post, imgY_rec, uiAbsPartIdx, uiDepth+1, Stride);
    }
    return;
  }
  
  // check maskImagedec
  if ( pcCU->getAlfCtrlFlag(uiAbsPartIdx) )
  {
    Int nHeight = min(uiBPelY+1,(unsigned int)(m_img_height)) - uiTPelY;
    Int nWidth  = min(uiRPelX+1,(unsigned int)(m_img_width)) - uiLPelX;
    calcVar( m_imgY_var, imgY_rec, FILTER_LENGTH/2, VAR_SIZE, nHeight, nWidth, Stride , uiLPelX , uiTPelY );
    subfilterFrame(imgY_rec_post, imgY_rec, pcAlfParam->realfiltNo, uiTPelY, min(uiBPelY+1,(unsigned int)(m_img_height)), uiLPelX, min(uiRPelX+1,(unsigned int)(m_img_width)), Stride);
  }
  else
  {
    // copy to rec
    Int nOffset = uiTPelY * Stride + uiLPelX;
    imgpel * pSrc = imgY_rec + nOffset;
    imgpel * pDst = imgY_rec_post + nOffset;
    Int nHeight = min(uiBPelY+1,(unsigned int)(m_img_height)) - uiTPelY;
    Int nWidth  = min(uiRPelX+1,(unsigned int)(m_img_width)) - uiLPelX;
    Int nSize = nWidth * sizeof( imgpel );
    for( Int n = 0 ; n < nHeight ; n++ )
    {
      memcpy( pDst , pSrc , nSize );
      pSrc += Stride;
      pDst += Stride;
    }
  }
}

// --------------------------------------------------------------------------------------------------------------------
// ALF for chroma
// --------------------------------------------------------------------------------------------------------------------

Void TComAdaptiveLoopFilter::xALFChroma(ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
  if((pcAlfParam->chroma_idc>>1)&0x01)
  {
    xFrameChroma(pcPicDec, pcPicRest, pcAlfParam->coeff_chroma, pcAlfParam->tap_chroma, 0);
  }
  else
  {
    pcPicDec->copyToPicCb( pcPicRest , false );
  }
  
  if(pcAlfParam->chroma_idc&0x01)
  {
    xFrameChroma(pcPicDec, pcPicRest, pcAlfParam->coeff_chroma, pcAlfParam->tap_chroma, 1);
  }
  else
  {
    pcPicDec->copyToPicCr( pcPicRest , false );
  }
}

/** 
 \param pcPicDec    picture before ALF
 \param pcPicRest   picture after  ALF
 \param qh          filter coefficient
 \param iTap        filter tap
 \param iColor      0 for Cb and 1 for Cr
 */
Void TComAdaptiveLoopFilter::xFrameChroma( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Int *qh, Int iTap, Int iColor )
{
  Int i, x, y, value, N;//, offset;
  Pel PixSum[ALF_MAX_NUM_COEF]; 
  
  N      = (iTap*iTap+1)>>1;
  //offset = iTap>>1;
  Int iHeight = pcPicRest->getHeight() >> 1;
  Int iWidth = pcPicRest->getWidth() >> 1;
  Pel* pDec;
  Int iDecStride = pcPicDec->getCStride();
  
  Pel* pRest;
  Int iRestStride = pcPicRest->getCStride();
  
  Int iShift = m_nInputBitDepth + m_nBitIncrement - 8;
  
  if (iColor)
  {
    pDec = pcPicDec->getCrAddr();
    pRest = pcPicRest->getCrAddr();
  }
  else
  {
    pDec = pcPicDec->getCbAddr();
    pRest = pcPicRest->getCbAddr();
  }

  Pel* pTmpDec1, *pTmpDec2;
  Pel* pTmpPixSum;
  
  switch(iTap)
  {
    case 5:
    {
      Int iJump = iDecStride - 4;
      pDec -= iDecStride*2;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-2;
          pTmpDec2 = pTmpDec1+4+(4*iDecStride);
          pTmpPixSum = PixSum;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum = (*pTmpDec1);
          
          value = 0;
          for(i=0; i<N; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[N] << iShift;
          value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;
          
          pRest[x] = (Pel) ClipC(value);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
      break;
    case 7:
    {
      Int iJump = iDecStride - 6;
      pDec -= iDecStride*3;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-3;
          pTmpDec2 = pTmpDec1+6+(6*iDecStride);
          pTmpPixSum = PixSum;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum = (*pTmpDec1);
          
          value = 0;
          for(i=0; i<N; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[N] << iShift;
          value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;
          
          pRest[x] = (Pel) ClipC(value);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
      break;
    case 9:
    {
      Int iJump = iDecStride - 8;
      pDec -= iDecStride*4;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-4;
          pTmpDec2 = pTmpDec1+8+(8*iDecStride);
          pTmpPixSum = PixSum;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;
          
          
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum =(*pTmpDec1);
          
          value = 0;
          for(i=0; i<N; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[N] << iShift;
          value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;
          
          pRest[x] = (Pel) ClipC(value);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
      break;
    default:
      assert(0);
      break;
  }
}

Void TComAdaptiveLoopFilter::setNumCUsInFrame(TComPic *pcPic)
{
  m_uiNumCUsInFrame = pcPic->getNumCUsInFrame();
}

#if QC_ALF_TMEPORAL_NUM
Void TComAdaptiveLoopFilter::setNumCUsInFrame(UInt uiNumCUsInFrame)
{
  m_uiNumCUsInFrame = uiNumCUsInFrame;
}
#endif

Void TComAdaptiveLoopFilter::setAlfCtrlFlags(ALFParam *pAlfParam, TComDataCU *pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt &idx)
{
  TComPic* pcPic = pcCU->getPic();
  UInt uiCurNumParts    = pcPic->getNumPartInCU() >> (uiDepth<<1);
  UInt uiQNumParts      = uiCurNumParts>>2;
  
  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
  
  if( ( uiRPelX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) || ( uiBPelY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
  {
    bBoundary = true;
  }
  
  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth ) ) || bBoundary )
  {
    UInt uiIdx = uiAbsPartIdx;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];
      
      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
      {
        setAlfCtrlFlags(pAlfParam, pcCU, uiIdx, uiDepth+1, idx);
      }
      uiIdx += uiQNumParts;
    }
    
    return;
  }
  
  if( uiDepth <= pAlfParam->alf_max_depth || pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, pAlfParam->alf_max_depth))
  {
    if (uiDepth > pAlfParam->alf_max_depth)
    {
      pcCU->setAlfCtrlFlagSubParts(pAlfParam->alf_cu_flag[idx], uiAbsPartIdx, pAlfParam->alf_max_depth);
    }
    else
    {
      pcCU->setAlfCtrlFlagSubParts(pAlfParam->alf_cu_flag[idx], uiAbsPartIdx, uiDepth );
    }
    idx++;
  }
}

#endif

