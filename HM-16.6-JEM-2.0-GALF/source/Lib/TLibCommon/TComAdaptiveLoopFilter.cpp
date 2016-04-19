/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
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
#if ALF_HM3_REFACTOR
#if QC_ALF_IMPROVEMENT 
Int TComAdaptiveLoopFilter::m_pattern9x9Sym[41] = 
{
                   0,
               1,  2,  3,
           4,  5,  6,  7,  8,
       9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 19, 18, 17, 16,
      15, 14, 13, 12, 11, 10,  9, 
           8,  7,  6,  5,  4,
               3,  2,  1,
                   0
};
Int TComAdaptiveLoopFilter::m_weights9x9Sym[22] = 
{
                   2,
               2,  2,  2,   
           2,  2,  2,  2,  2, 
       2,  2,  2,  2,  2,  2,  2,  
   2,  2,  2,  2,  1,  1
};

Int TComAdaptiveLoopFilter::m_pattern9x9Sym_Quart[42] = 
{

   0,  0,  0,  0,  1,  0,  0,  0,  0,
   0,  0,  0,  2,  3,  4,  0,  0,  0,
   0,  0,  5,  6,  7,  8,  9,  0,  0,  
   0, 10, 11, 12, 13, 14, 15, 16,  0,
  17, 18, 19, 20, 21, 22
};
#else
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
#endif

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

Int* TComAdaptiveLoopFilter::m_patternTab_filt[m_NO_TEST_FILT] =
{
  m_pattern9x9Sym_9, m_pattern9x9Sym_7, m_pattern9x9Sym_5
}; 

Int* TComAdaptiveLoopFilter::m_patternTab[m_NO_TEST_FILT] =
{
  m_pattern9x9Sym, m_pattern7x7Sym, m_pattern5x5Sym
}; 

Int* TComAdaptiveLoopFilter::m_patternMapTab[m_NO_TEST_FILT] =
{
  m_pattern9x9Sym_Quart, m_pattern7x7Sym_Quart, m_pattern5x5Sym_Quart
};

Int* TComAdaptiveLoopFilter::m_weightsTab[m_NO_TEST_FILT] =
{
  m_weights9x9Sym, m_weights7x7Sym, m_weights5x5Sym
};

Int TComAdaptiveLoopFilter::m_flTab[m_NO_TEST_FILT] =
{
  9/2, 7/2, 5/2
};

Int TComAdaptiveLoopFilter::m_sqrFiltLengthTab[m_NO_TEST_FILT] =
{
  m_SQR_FILT_LENGTH_9SYM, m_SQR_FILT_LENGTH_7SYM, m_SQR_FILT_LENGTH_5SYM
};


#if QC_ALF_IMPROVEMENT
const Int TComAdaptiveLoopFilter:: depthInt9x9Cut[21] = 
{
              1, 
           1, 2, 1,
        1, 2, 3, 2, 1,
     1, 2, 3, 4, 3, 2, 1,
  1, 2, 3, 4, 5,
};

const Int TComAdaptiveLoopFilter:: depthInt7x7Cut[14] = 
{
           1, 
        1, 2, 1,
     1, 2, 3, 2, 1, 
  1, 2, 3, 4, 4 
};


const Int TComAdaptiveLoopFilter:: depthInt5x5Cut[8] = 
{
        1,  
     1, 2, 1,
  1, 2, 3, 3  
};
#else
const Int TComAdaptiveLoopFilter::m_depthInt9x9Sym[21] = 
{
           5, 6, 5, 
        5, 6, 7, 6, 5,
     5, 6, 7, 8, 7, 6, 5,
  5, 6, 7, 8, 9, 9 
};

const Int TComAdaptiveLoopFilter::m_depthInt7x7Sym[14] = 
{
           4, 
        4, 5, 4, 
     4, 5, 6, 5, 4, 
  4, 5, 6, 7, 7 
};

const Int TComAdaptiveLoopFilter::m_depthInt5x5Sym[8] = 
{
        3,   
     3, 4, 3,
  3, 4, 5, 5  
};
#endif

const Int* TComAdaptiveLoopFilter::m_pDepthIntTab[m_NO_TEST_FILT] =
{
#if QC_ALF_IMPROVEMENT
  depthInt5x5Cut,   depthInt7x7Cut,   depthInt9x9Cut, 
#else
  m_depthInt5x5Sym, m_depthInt7x7Sym, m_depthInt9x9Sym
#endif
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

#if !QC_ALF_IMPROVEMENT || EE_USE_HM3_CHROMA
// scaling factor for quantization of filter coefficients (5x5)
const Int TComAdaptiveLoopFilter::m_aiSymmetricMag5x5[13] =
{
  2, 2, 2, 2, 2,
  2, 2, 2, 2, 2,
  2, 2, 1
};
#endif


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
#if QC_ALF_IMPROVEMENT
  m_imgY_dig0 = NULL;
  m_imgY_dig1 = NULL;
#endif
  m_filterCoeffSym = NULL;
#if QC_ALF_IMPROVEMENT
  m_filterCoeffFinal = NULL;
#endif
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

Void TComAdaptiveLoopFilter::create( Int iPicWidth, Int iPicHeight, ChromaFormat chromaFormatIDC, Int uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth , Int nInputBitDepth , Int nInternalBitDepth )
{
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
  m_uiMaxTotalCUDepth = uiMaxCUDepth;

  assert( m_nBitIncrement >= 0 );
  if ( !m_pcTempPicYuv )
  {
    m_pcTempPicYuv = new TComPicYuv;
    m_pcTempPicYuv->create( iPicWidth, iPicHeight, chromaFormatIDC, uiMaxCUWidth, uiMaxCUHeight, uiMaxCUDepth , true);
  }
  m_img_height = iPicHeight;
  m_img_width = iPicWidth;
#if QC_ALF_IMPROVEMENT
  Int iPadOffset = max(2, SHIFT_VAL_HALFW);
  initMatrix_int(&m_imgY_temp, m_ALF_WIN_VERSIZE+2*iPadOffset+3, m_ALF_WIN_HORSIZE+2*iPadOffset+3);
  initMatrix_int(&m_imgY_ver,  m_ALF_WIN_VERSIZE+2*iPadOffset+3, m_ALF_WIN_HORSIZE+2*iPadOffset+3);
  initMatrix_int(&m_imgY_hor,  m_ALF_WIN_VERSIZE+2*iPadOffset+3, m_ALF_WIN_HORSIZE+2*iPadOffset+3);
  initMatrix_int(&m_imgY_dig0,  m_ALF_WIN_VERSIZE+2*iPadOffset+3, m_ALF_WIN_HORSIZE+2*iPadOffset+3);
  initMatrix_int(&m_imgY_dig1,  m_ALF_WIN_VERSIZE+2*iPadOffset+3, m_ALF_WIN_HORSIZE+2*iPadOffset+3);
#else
  initMatrix_int(&m_imgY_temp, m_ALF_WIN_VERSIZE+2*m_VAR_SIZE+3, m_ALF_WIN_HORSIZE+2*m_VAR_SIZE+3);
  initMatrix_int(&m_imgY_ver, m_ALF_WIN_VERSIZE+2*m_VAR_SIZE+3, m_ALF_WIN_HORSIZE+2*m_VAR_SIZE+3);
  initMatrix_int(&m_imgY_hor, m_ALF_WIN_VERSIZE+2*m_VAR_SIZE+3, m_ALF_WIN_HORSIZE+2*m_VAR_SIZE+3);
#endif


  get_mem2Dpel(&m_varImgMethods, m_img_width, m_img_width);
  initMatrix_int(&m_filterCoeffSym, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);
  initMatrix_int(&m_filterCoeffPrevSelected, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH); 
  initMatrix_int(&m_filterCoeffTmp, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);      
  initMatrix_int(&m_filterCoeffSymTmp, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);   

  initMatrix_short(&m_filterCoeffShort, m_NO_VAR_BINS, m_MAX_SQR_FILT_LENGTH);
#if QC_ALF_IMPROVEMENT
  initMatrix_int(&m_filterCoeffFinal, m_NO_VAR_BINS, (m_MAX_SQR_FILT_LENGTH/2+1));
#endif
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
#if QC_ALF_IMPROVEMENT
  destroyMatrix_int(m_imgY_dig0);
  destroyMatrix_int(m_imgY_dig1);
#endif
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
#if QC_ALF_IMPROVEMENT
  destroyMatrix_int(m_filterCoeffFinal);
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Int TComAdaptiveLoopFilter::ALFTapHToTapV(Int tapH)
{
#if QC_ALF_IMPROVEMENT
  return tapH;
#else
  return min<UInt>(tapH, 7);
#endif
}

Int TComAdaptiveLoopFilter::ALFFlHToFlV(Int flH)
{
#if QC_ALF_IMPROVEMENT
  return flH;
#else
  return min<UInt>(flH, 7/2);
#endif
}

Int TComAdaptiveLoopFilter::ALFTapHToNumCoeff(Int tapH)
{
  Int num_coeff;
  
  num_coeff = (Int)(tapH*tapH)/4 + 2;
#if QC_ALF_IMPROVEMENT
  num_coeff -= 1;
#else
  if (tapH == 9)
    num_coeff -= 1;
  else
    assert(tapH < 9);
#endif
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
#if QC_ALF_IMPROVEMENT 
  pAlfParam->log2UnitSize  = 0;
#if EE_DISABLE_PRED_FIXEDFILTER
  pAlfParam->iAvailableFilters = 0;
#else
  pAlfParam->iAvailableFilters = NO_PREV_FILTERS;
#endif
  pAlfParam->iPredPattern = 0;
#else
  pAlfParam->coeff        = new Int[m_ALF_MAX_NUM_COEF];
  ::memset(pAlfParam->coeff,        0, sizeof(Int)*m_ALF_MAX_NUM_COEF   );
#endif
  pAlfParam->coeff_chroma = new Int[m_ALF_MAX_NUM_COEF_C];
  ::memset(pAlfParam->coeff_chroma, 0, sizeof(Int)*m_ALF_MAX_NUM_COEF_C );

  pAlfParam->coeffmulti = new Int*[m_NO_VAR_BINS];

  for (int i=0; i<m_NO_VAR_BINS; i++)
  {
    pAlfParam->coeffmulti[i] = new Int[m_ALF_MAX_NUM_COEF];
    ::memset(pAlfParam->coeffmulti[i],        0, sizeof(Int)*m_ALF_MAX_NUM_COEF );
  }
  pAlfParam->num_cus_in_frame = m_uiNumCUsInFrame;
  pAlfParam->num_alf_cu_flag  = 0;
  pAlfParam->alf_cu_flag      = new UInt[(m_uiNumCUsInFrame << ((m_uiMaxTotalCUDepth-1)*2))];
  ::memset(pAlfParam->kMinTab , 0 , sizeof( pAlfParam->kMinTab ) );

#if COM16_C806_ALF_TEMPPRED_NUM
  pAlfParam->temproalPredFlag = false;
  pAlfParam->prevIdx = -1;
  pAlfParam->alfCoeffLuma   = new Int*[m_NO_VAR_BINS];
  for (int i=0; i<m_NO_VAR_BINS; i++)
  {
    pAlfParam->alfCoeffLuma[i] = new Int[m_ALF_MAX_NUM_COEF];
    ::memset(pAlfParam->alfCoeffLuma[i],        0, sizeof(Int)*m_ALF_MAX_NUM_COEF );
  }

  pAlfParam->alfCoeffChroma = new Int[m_ALF_MAX_NUM_COEF_C];
  ::memset(pAlfParam->alfCoeffChroma,        0, sizeof(Int)*m_ALF_MAX_NUM_COEF_C );
#endif
}
#if !QC_ALF_IMPROVEMENT
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
  printf( "\ncu_control_flag %d" , pAlfParam->cu_control_flag );
  printf( "\nalf_max_depth %d" , pAlfParam->alf_max_depth );
  printf( "\nnum_alf_cu_flag %d" , pAlfParam->num_alf_cu_flag );

  printf( "\nvarIndTab: " );
  for( Int i = 0 ; i < ALF_NO_VAR_BIN ; i++ )
    printf( "(%d)%d " , i , pAlfParam->varIndTab[i]  );

  printf("\n");
}
#endif

Void TComAdaptiveLoopFilter::freeALFParam(ALFParam* pAlfParam)
{
  assert(pAlfParam != NULL);
#if !QC_ALF_IMPROVEMENT  
  if (pAlfParam->coeff != NULL)
  {
    delete[] pAlfParam->coeff;
    pAlfParam->coeff = NULL;
  }
#endif
  if (pAlfParam->coeff_chroma != NULL)
  {
    delete[] pAlfParam->coeff_chroma;
    pAlfParam->coeff_chroma = NULL;
  }


  if( pAlfParam->coeffmulti != NULL )
  {
    for (int i=0; i<m_NO_VAR_BINS; i++)
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
#if COM16_C806_ALF_TEMPPRED_NUM
  if( pAlfParam->alfCoeffLuma != NULL )
  {
    for (int i=0; i<m_NO_VAR_BINS; i++)
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
#if QC_ALF_IMPROVEMENT
  pDesAlfParam->log2UnitSize = pSrcAlfParam->log2UnitSize; //copy best unit size
#endif
#if COM16_C806_ALF_TEMPPRED_NUM
  if( !pDesAlfParam->temproalPredFlag )
  {
#endif
  pDesAlfParam->alf_flag = pSrcAlfParam->alf_flag;
  pDesAlfParam->cu_control_flag = pSrcAlfParam->cu_control_flag;
  pDesAlfParam->chroma_idc = pSrcAlfParam->chroma_idc;
#if COM16_C806_ALF_TEMPPRED_NUM
  }
#endif
  pDesAlfParam->tap = pSrcAlfParam->tap;
  pDesAlfParam->tapV = pSrcAlfParam->tapV;
  pDesAlfParam->num_coeff = pSrcAlfParam->num_coeff;

  pDesAlfParam->tap_chroma = pSrcAlfParam->tap_chroma;
  pDesAlfParam->num_coeff_chroma = pSrcAlfParam->num_coeff_chroma;
#if !QC_ALF_IMPROVEMENT
  ::memcpy(pDesAlfParam->coeff, pSrcAlfParam->coeff, sizeof(Int)*m_ALF_MAX_NUM_COEF);
#endif
  ::memcpy(pDesAlfParam->coeff_chroma, pSrcAlfParam->coeff_chroma, sizeof(Int)*m_ALF_MAX_NUM_COEF_C);

  pDesAlfParam->realfiltNo = pSrcAlfParam->realfiltNo;
  pDesAlfParam->filtNo = pSrcAlfParam->filtNo;
  ::memcpy(pDesAlfParam->filterPattern, pSrcAlfParam->filterPattern, sizeof(Int)*m_NO_VAR_BINS);
#if QC_ALF_IMPROVEMENT
  ::memcpy(pDesAlfParam->PrevFiltIdx,   pSrcAlfParam->PrevFiltIdx,   sizeof(pSrcAlfParam->PrevFiltIdx ));
  ::memcpy(pDesAlfParam->codedVarBins,  pSrcAlfParam->codedVarBins,  sizeof(pSrcAlfParam->codedVarBins));
  pDesAlfParam->iAvailableFilters = pSrcAlfParam->iAvailableFilters;
  pDesAlfParam->forceCoeff0       = pSrcAlfParam->forceCoeff0;
  pDesAlfParam->iPredPattern      = pSrcAlfParam->iPredPattern;
#endif
  pDesAlfParam->startSecondFilter = pSrcAlfParam->startSecondFilter;
  pDesAlfParam->noFilters = pSrcAlfParam->noFilters;
  
  //Coeff send related
  pDesAlfParam->filters_per_group_diff = pSrcAlfParam->filters_per_group_diff; //this can be updated using codedVarBins
  pDesAlfParam->filters_per_group = pSrcAlfParam->filters_per_group; //this can be updated using codedVarBins
#if !QC_ALF_IMPROVEMENT
  ::memcpy(pDesAlfParam->codedVarBins, pSrcAlfParam->codedVarBins, sizeof(Int)*m_NO_VAR_BINS);
  pDesAlfParam->forceCoeff0 = pSrcAlfParam->forceCoeff0;
#endif
  pDesAlfParam->predMethod = pSrcAlfParam->predMethod;
  for (int i=0; i<m_NO_VAR_BINS; i++)
  {
    ::memcpy(pDesAlfParam->coeffmulti[i], pSrcAlfParam->coeffmulti[i], sizeof(Int)*m_ALF_MAX_NUM_COEF);
#if COM16_C806_ALF_TEMPPRED_NUM
    ::memcpy(pDesAlfParam->alfCoeffLuma[i], pSrcAlfParam->alfCoeffLuma[i], sizeof(Int)*m_ALF_MAX_NUM_COEF);
#endif
  }
#if COM16_C806_ALF_TEMPPRED_NUM
  ::memcpy(pDesAlfParam->alfCoeffChroma, pSrcAlfParam->alfCoeffChroma, sizeof(Int)*m_ALF_MAX_NUM_COEF_C);
#endif


  pDesAlfParam->minKStart = pSrcAlfParam->minKStart;
  ::memcpy( pDesAlfParam->kMinTab , pSrcAlfParam->kMinTab , sizeof( pSrcAlfParam->kMinTab ) );

  ::memcpy( pDesAlfParam->varIndTab , pSrcAlfParam->varIndTab , sizeof( pSrcAlfParam->varIndTab ) );

#if COM16_C806_ALF_TEMPPRED_NUM
  if( !pDesAlfParam->temproalPredFlag )
  {
#endif
  pDesAlfParam->num_alf_cu_flag = pSrcAlfParam->num_alf_cu_flag;
  ::memcpy(pDesAlfParam->alf_cu_flag, pSrcAlfParam->alf_cu_flag, sizeof(UInt)*pSrcAlfParam->num_alf_cu_flag);
#if COM16_C806_ALF_TEMPPRED_NUM
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
#if QC_ALF_IMPROVEMENT && !EE_USE_HM3_CHROMA
  pDesAlfParam->tap_chroma = m_ALF_MAX_NUM_TAP_C;
#else
  pDesAlfParam->tap_chroma = 0;
  pDesAlfParam->num_coeff_chroma = 0;
#endif  
#if QC_ALF_IMPROVEMENT 
  pDesAlfParam->log2UnitSize     = 0;
  pDesAlfParam->iAvailableFilters = NO_PREV_FILTERS;
  pDesAlfParam->forceCoeff0       = 0;
  pDesAlfParam->iPredPattern      = 0;
#endif

#if !QC_ALF_IMPROVEMENT
  ::memset(pDesAlfParam->coeff, 0, sizeof(Int)*m_ALF_MAX_NUM_COEF);
#endif
  ::memset(pDesAlfParam->coeff_chroma, 0, sizeof(Int)*m_ALF_MAX_NUM_COEF_C);

  pDesAlfParam->realfiltNo = 0;
  pDesAlfParam->filtNo = 0;
  ::memset(pDesAlfParam->filterPattern, 0, sizeof(Int)*m_NO_VAR_BINS);
#if QC_ALF_IMPROVEMENT
  ::memset(pDesAlfParam->PrevFiltIdx,   0, sizeof(Int)*m_NO_VAR_BINS);
  for(int ind = 0; ind < ALF_NO_VAR_BIN; ind++)
  {
    pDesAlfParam->codedVarBins[ind] = 1;
  }
#endif
  pDesAlfParam->startSecondFilter = 0;
  pDesAlfParam->noFilters = 0;
  ::memset(pDesAlfParam->kMinTab , 0 , sizeof( pDesAlfParam->kMinTab ) );

  //Coeff send related
  pDesAlfParam->filters_per_group_diff = 0; //this can be updated using codedVarBins
  pDesAlfParam->filters_per_group = 0; //this can be updated using codedVarBins
#if !QC_ALF_IMPROVEMENT
  ::memset(pDesAlfParam->codedVarBins, 0, sizeof(Int)*m_NO_VAR_BINS);
  pDesAlfParam->forceCoeff0 = 0;
#endif
  pDesAlfParam->predMethod = 0;
  for (int i=0; i<m_NO_VAR_BINS; i++)
  {
    ::memset(pDesAlfParam->coeffmulti[i], 0, sizeof(Int)*m_ALF_MAX_NUM_COEF);
#if COM16_C806_ALF_TEMPPRED_NUM
    ::memset(pDesAlfParam->alfCoeffLuma[i], 0, sizeof(Int)*m_ALF_MAX_NUM_COEF);
#endif
  }
#if COM16_C806_ALF_TEMPPRED_NUM
  ::memset(pDesAlfParam->alfCoeffChroma, 0, sizeof(Int)*m_ALF_MAX_NUM_COEF_C);
  pDesAlfParam->temproalPredFlag = false;
  pDesAlfParam->prevIdx = -1;
#endif

  ::memset( pDesAlfParam->varIndTab , 0 , sizeof( pDesAlfParam->varIndTab ) );

  pDesAlfParam->num_alf_cu_flag = 0;
}
#if QC_ALF_IMPROVEMENT && !EE_USE_HM3_CHROMA
Void TComAdaptiveLoopFilter::initVarForChroma  (ALFParam* pcAlfParam, Bool bUpdatedDCCoef)
{
  Int k, i;
  //initilization for clip operation in subfilterFrame()
  Int flipTableSize   = (m_ALF_HM3_QC_CLIP_RANGE <<m_nBitIncrement);
  m_alfClipOffset = (m_ALF_HM3_QC_CLIP_OFFSET<<m_nBitIncrement);

  if( m_alfClipTable )
  {
    free(m_alfClipTable);
  }

  m_alfClipTable = (imgpel *)calloc(flipTableSize, sizeof(imgpel));

  for( k=0; k< flipTableSize; k++)
  {
    m_alfClipTable[k] = max( 0, min( k-m_alfClipOffset, m_nIBDIMax ) );
  }
  Int filtNo = pcAlfParam->tap_chroma==9? 0: (pcAlfParam->tap_chroma==7?1: 2);
  if (!bUpdatedDCCoef)
  {
    Int * weights = TComAdaptiveLoopFilter::m_weightsTab[filtNo]; 
    Int quantCoeffSum=0;
    Int factor = (1 << (m_NUM_BITS-1)); 
    for (i=0; i< pcAlfParam->num_coeff_chroma - 1; i++)
    {
      quantCoeffSum += weights[i] * pcAlfParam->coeff_chroma [i];
    }
    pcAlfParam->coeff_chroma[pcAlfParam->num_coeff_chroma - 1] = factor - quantCoeffSum;
  }
  //fill in the ALF coefficients
  Int* patternMap = m_patternMapTab[filtNo];
  k=0;
  for( i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
  {
    if (patternMap[i]>0)
    {
      m_filterCoeffShort[0][i]=pcAlfParam->coeff_chroma[k];
      k++;
    }
    else
    {
      m_filterCoeffShort[0][i]=0;
    }
  }  
}
#endif
// --------------------------------------------------------------------------------------------------------------------
// prediction of filter coefficients
// --------------------------------------------------------------------------------------------------------------------
#if !QC_ALF_IMPROVEMENT
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
  pred=(1<<m_ALF_NUM_BIT_SHIFT)-sum;
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
  pred=(1<<m_ALF_NUM_BIT_SHIFT)-(sum-pAlfParam->coeff_chroma[N-1]);
  pAlfParam->coeff_chroma[N-1]=pred-pAlfParam->coeff_chroma[N-1];
}
#endif

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
#if QC_ALF_IMPROVEMENT
  pcPicYuvExtRec->extendPicBorder    (m_PADDING_W_ALF);
  m_max_NO_VAR_BINS = TComAdaptiveLoopFilter::m_NO_VAR_BINS ;
  m_max_NO_FILTERS  = TComAdaptiveLoopFilter::m_NO_FILTERS  ;
#else
  pcPicYuvExtRec->extendPicBorder    (m_FILTER_LENGTH >> 1);
#endif

  if(pcAlfParam->cu_control_flag)
  {
    UInt idx = 0;
    for(UInt uiCUAddr = 0; uiCUAddr < pcPic->getNumberOfCtusInFrame(); uiCUAddr++)
    {
      TComDataCU *pcCU = pcPic->getCtu(uiCUAddr);
      setAlfCtrlFlags(pcAlfParam, pcCU, 0, 0, idx);
    }
  }

  xALFLuma_qc(pcPic, pcAlfParam, pcPicYuvExtRec, pcPicYuvRec);
 
  if(pcAlfParam->chroma_idc)
  {
#if QC_ALF_IMPROVEMENT
#if !EE_USE_HM3_CHROMA
#if COM16_C806_ALF_TEMPPRED_NUM
    initVarForChroma(pcAlfParam, (pcAlfParam->temproalPredFlag ? true : false));
#else
    initVarForChroma(pcAlfParam, false);
#endif
#endif
#else
#if COM16_C806_ALF_TEMPPRED_NUM
    if( !pcAlfParam->temproalPredFlag )
#endif
    predictALFCoeffChroma(pcAlfParam);
#endif

#if COM16_C806_ALF_TEMPPRED_NUM
    memcpy( pcAlfParam->alfCoeffChroma, pcAlfParam->coeff_chroma, sizeof(Int)*m_ALF_MAX_NUM_COEF_C );
#endif
    xALFChroma( pcAlfParam, pcPicYuvExtRec, pcPicYuvRec);
  }
  else
  {
    pcPicYuvExtRec->copyToPic( pcPicYuvRec , COMPONENT_Cb , false );
    pcPicYuvExtRec->copyToPic( pcPicYuvRec , COMPONENT_Cr , false );
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
  Int    LumaStride = pcPicDec->getStride(COMPONENT_Y);
  imgpel* pDec = (imgpel*)pcPicDec->getAddr(COMPONENT_Y);
  imgpel* pRest = (imgpel*)pcPicRest->getAddr(COMPONENT_Y);
  
  //Decode and reconst filter coefficients
#if ENABLE_FIXEDFILTER_INTERSLICE
  DecFilter_qc(pDec,pcAlfParam,LumaStride);
#else
  DecFilter_qc(pDec,pcAlfParam,LumaStride, pcPic->getSlice(0));
#endif
  //set maskImg using cu adaptive one.

  m_imgY_var       = m_varImgMethods;

  if(pcAlfParam->cu_control_flag)
  {
    xCUAdaptive_qc(pcPic, pcAlfParam, pRest, pDec, LumaStride);
  }  
  else
  {
    //then do whole frame filtering
#if QC_ALF_IMPROVEMENT
#if ENABLE_FIXEDFILTER_INTERSLICE
    filterFrame(pRest, pDec, pcAlfParam, LumaStride);
#else
    filterFrame(pRest, pDec, pcAlfParam, LumaStride, (pcPic->getSlice(0)->isIntra()? true: false));
#endif
#else
    filterFrame(pRest, pDec, pcAlfParam->realfiltNo, LumaStride);
#endif
  }
}

#if ENABLE_FIXEDFILTER_INTERSLICE
Void TComAdaptiveLoopFilter::DecFilter_qc(imgpel* imgY_rec,ALFParam* pcAlfParam, int Stride)
#else
Void TComAdaptiveLoopFilter::DecFilter_qc(imgpel* imgY_rec,ALFParam* pcAlfParam, int Stride, TComSlice *pSlice)
#endif
{
  int i;
  int numBits = m_NUM_BITS; 
  int **pfilterCoeffSym;
  pfilterCoeffSym= m_filterCoeffSym;
#if COM16_C806_ALF_TEMPPRED_NUM
  if( pcAlfParam->temproalPredFlag )
  {
#if QC_ALF_IMPROVEMENT
    for(i = 0; i < m_max_NO_VAR_BINS; i++)
#else
    for(i = 0; i < m_NO_VAR_BINS; i++)
#endif
    {
      memcpy(pfilterCoeffSym[i], &pcAlfParam->alfCoeffLuma[i][0] ,sizeof(int)*m_MAX_SQR_FILT_LENGTH);
    }

    int k, varInd;
    int *patternMap;
    int *patternMapTab[3]={m_pattern9x9Sym_Quart, m_pattern7x7Sym_Quart, m_pattern5x5Sym_Quart};
    {
#if QC_ALF_IMPROVEMENT
      for(varInd=0; varInd<m_max_NO_VAR_BINS; ++varInd)
#else
      for(varInd=0; varInd<m_NO_VAR_BINS; ++varInd)
#endif
      {
        memset(m_filterCoeffPrevSelected[varInd],0,sizeof(int)*m_MAX_SQR_FILT_LENGTH);
      }
      patternMap=patternMapTab[pcAlfParam->realfiltNo];
#if QC_ALF_IMPROVEMENT
      for(varInd=0; varInd<m_max_NO_VAR_BINS; ++varInd)
#else
      for(varInd=0; varInd<m_NO_VAR_BINS; ++varInd)
#endif
      {
        k=0;
        for(i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
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
#if QC_ALF_IMPROVEMENT
    for(i = 0; i < m_max_NO_VAR_BINS; i++)
#else
    for(i = 0; i < m_NO_VAR_BINS; i++)
#endif
    {
      pcAlfParam->varIndTab[i]=0;
      memset(pfilterCoeffSym[i],0,sizeof(int)*m_MAX_SQR_FILT_LENGTH);
    }
  }
#if QC_ALF_IMPROVEMENT && !ENABLE_FIXEDFILTER_INTERSLICE  
  getCurrentFilter(pfilterCoeffSym,pcAlfParam, pSlice);
#else  
  getCurrentFilter(pfilterCoeffSym,pcAlfParam);
#endif
#if COM16_C806_ALF_TEMPPRED_NUM
  }
#endif

  Int *coef;
  Int maxPxlVal = m_nIBDIMax;
#if QC_ALF_IMPROVEMENT 
  Int flipTableSize;
  Int centerCoef = m_MAX_SQR_FILT_LENGTH - 1;
#else
  Int maxSampleValue, minSampleValue = 0;
  Int clipRange[2] = { 0, 0 }, flipTableSize;
  Int sumCoef[2];

  Int numBitsMinus1= m_NUM_BITS-1;
  Int offset = (1<<(m_NUM_BITS-2));
  Int lastCoef = m_MAX_SQR_FILT_LENGTH-1;
  Int centerCoef = m_MAX_SQR_FILT_LENGTH-2;
#endif

#if QC_ALF_IMPROVEMENT
  for(Int varInd=0; varInd<m_max_NO_VAR_BINS; ++varInd)
  {
    coef = m_filterCoeffPrevSelected[varInd];
    for(i = 0; i < centerCoef; i++)
    {
      m_filterCoeffShort[varInd][i] = (Short)coef[i];
    }
    m_filterCoeffShort[varInd][centerCoef] = (Short)coef[centerCoef];
  }
#else
  for(Int varInd=0; varInd<m_NO_VAR_BINS; ++varInd)
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
  Int clipTableMax = ( ( m_ALF_HM3_QC_CLIP_RANGE - m_ALF_HM3_QC_CLIP_OFFSET - 1 ) << m_nBitIncrement );
  Int clipTableMin = ( ( - m_ALF_HM3_QC_CLIP_OFFSET ) << m_nBitIncrement );

  assert( clipRange[0]<=clipTableMax && clipRange[1]>=clipTableMin );
#endif


  flipTableSize   = (m_ALF_HM3_QC_CLIP_RANGE <<m_nBitIncrement);
  m_alfClipOffset = (m_ALF_HM3_QC_CLIP_OFFSET<<m_nBitIncrement);

  if( m_alfClipTable )
  {
    free(m_alfClipTable);
  }

  m_alfClipTable = (imgpel *)calloc(flipTableSize, sizeof(imgpel));

  for(Int k=0; k< flipTableSize; k++)
  {
    m_alfClipTable[k] = max( 0, min( k-m_alfClipOffset, maxPxlVal ) );
  }

  memset(m_imgY_temp[0],0,sizeof(int)*(m_ALF_WIN_VERSIZE+2*m_VAR_SIZE)*(m_ALF_WIN_HORSIZE+2*m_VAR_SIZE));
  m_imgY_var       = m_varImgMethods;
}


#if QC_ALF_IMPROVEMENT
#if ENABLE_FIXEDFILTER_INTERSLICE
Void TComAdaptiveLoopFilter::getCurrentFilter(int **filterCoeffSym,ALFParam* pcAlfParam)
#else
Void TComAdaptiveLoopFilter::getCurrentFilter(int **filterCoeffSym,ALFParam* pcAlfParam, TComSlice *pSlice)
#endif
{
  int i,  k, varInd, filterNo;
  int *patternMap;
  int *patternMapTab[3]={m_pattern9x9Sym_Quart, m_pattern7x7Sym_Quart, m_pattern5x5Sym_Quart};
#if !ENABLE_FIXEDFILTER_INTERSLICE  
  Bool bIntraSlice = pSlice->isIntra();
#endif
  
  Int ** filterCoeffFinal;
  Int factor = (1<<(TComAdaptiveLoopFilter::m_NUM_BITS-1)); 
  Int iMaxNumCoeff = (m_MAX_SQR_FILT_LENGTH/2+1);

  initMatrix_int(&filterCoeffFinal, m_max_NO_VAR_BINS, iMaxNumCoeff);
  
  for(varInd=0; varInd<m_max_NO_VAR_BINS; ++varInd)
  {
    memset(m_filterCoeffPrevSelected[varInd], 0, sizeof(int)*m_MAX_SQR_FILT_LENGTH);
    memset(filterCoeffFinal[varInd],          0, sizeof(int)*iMaxNumCoeff         );
  }

  if(pcAlfParam->iPredPattern)
  {
    for(varInd=0; varInd< m_max_NO_VAR_BINS; ++varInd)
    {
      if(pcAlfParam->PrevFiltIdx[varInd])
      {
        Int iPrevFiltIdx = pcAlfParam->PrevFiltIdx[varInd] - 1;
        for(i = 0; i < iMaxNumCoeff; i++)
        {
          filterNo = varInd*NO_PREV_FILTERS + iPrevFiltIdx;
          filterCoeffFinal[varInd][i] = m_ALFfilterCoeffFixed[filterNo][i]; 
        }
      }
      else
      {
        memset(filterCoeffFinal[varInd], 0, sizeof(Int)*(iMaxNumCoeff-1));
        filterCoeffFinal[varInd][iMaxNumCoeff-1] = factor;
      }
    }
  }
  else
  {
    for(varInd=0; varInd< iMaxNumCoeff; ++varInd)
    {
      memset(filterCoeffFinal[varInd], 0, (iMaxNumCoeff - 1)*sizeof(Int) );
      filterCoeffFinal[varInd][(iMaxNumCoeff - 1)] = factor;
    }
  }
  patternMap=patternMapTab[pcAlfParam->realfiltNo];

  for(varInd=0; varInd<m_max_NO_VAR_BINS; ++varInd)
  {
    k=0;
    for(i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
    {
      if (m_pattern9x9Sym_Quart[i] > 0 && patternMap[i]>0)
      {
        m_filterCoeffPrevSelected[varInd][i]= (filterCoeffFinal[varInd][m_pattern9x9Sym_Quart[i]-1]+ filterCoeffSym[pcAlfParam->varIndTab[varInd]][patternMap[i]-1]);
#if ENABLE_FIXEDFILTER_INTERSLICE
        pcAlfParam->alfCoeffLuma[varInd][m_pattern9x9Sym_Quart[i]-1] = m_filterCoeffPrevSelected[varInd][i];
#else
        if(bIntraSlice)
        {
          pcAlfParam->alfCoeffLuma[varInd][m_pattern9x9Sym_Quart[i]-1] = m_filterCoeffPrevSelected[varInd][i];
        }
        else
        {
          pcAlfParam->alfCoeffLuma[varInd][k] = m_filterCoeffPrevSelected[varInd][i];
        }
#endif
        k++;
      }
#if ENABLE_FIXEDFILTER_INTERSLICE
      else if(m_pattern9x9Sym_Quart[i] > 0)
#else
      else if(bIntraSlice && m_pattern9x9Sym_Quart[i] > 0)
#endif
      {
        m_filterCoeffPrevSelected[varInd][i] = filterCoeffFinal[varInd][m_pattern9x9Sym_Quart[i]-1];
        pcAlfParam->alfCoeffLuma[varInd][m_pattern9x9Sym_Quart[i]-1]  = filterCoeffFinal[varInd][m_pattern9x9Sym_Quart[i]-1];
      }
      else
      {
        m_filterCoeffPrevSelected[varInd][i]=0;
      }
    }
  }
#if ENABLE_FIXEDFILTER_INTERSLICE
  Int iNumCoeffMinus1 = m_MAX_SQT_FILT_SYM_LENGTH - 1, quantCoeffSum = 0;
  Int * weights=TComAdaptiveLoopFilter::m_weightsTab[0];   
#else
  Int iNumCoeffMinus1 = bIntraSlice ? m_MAX_SQT_FILT_SYM_LENGTH - 1 : pcAlfParam->num_coeff - 1, quantCoeffSum = 0;
  int * weights=TComAdaptiveLoopFilter::m_weightsTab[bIntraSlice ? 0: pcAlfParam->realfiltNo];   
#endif
  for(varInd = 0; varInd < m_max_NO_VAR_BINS; ++ varInd)
  {
    quantCoeffSum = 0 ;
    for(i = 0; i < iNumCoeffMinus1; i++)
    {
      quantCoeffSum += weights[i]* pcAlfParam->alfCoeffLuma[varInd][i];
    }
    pcAlfParam->alfCoeffLuma [varInd][iNumCoeffMinus1] = factor-quantCoeffSum;
    m_filterCoeffPrevSelected[varInd][m_MAX_SQR_FILT_LENGTH - 1] = factor-quantCoeffSum;
  }
  destroyMatrix_int(filterCoeffFinal);
}
#else
Void TComAdaptiveLoopFilter::getCurrentFilter(int **filterCoeffSym,ALFParam* pcAlfParam)
{ 
  int i,  k, varInd;
  int *patternMap;
  int *patternMapTab[3]={m_pattern9x9Sym_Quart, m_pattern7x7Sym_Quart, m_pattern5x5Sym_Quart};
  {
    for(varInd=0; varInd<m_NO_VAR_BINS; ++varInd)
    {
      memset(m_filterCoeffPrevSelected[varInd],0,sizeof(int)*m_MAX_SQR_FILT_LENGTH);
    }
    patternMap=patternMapTab[pcAlfParam->realfiltNo];
    for(varInd=0; varInd<m_NO_VAR_BINS; ++varInd)
    {
      k=0;
      for(i = 0; i < m_MAX_SQR_FILT_LENGTH; i++)
      {
        if (patternMap[i]>0)
        {
          m_filterCoeffPrevSelected[varInd][i]=filterCoeffSym[pcAlfParam->varIndTab[varInd]][k];
#if COM16_C806_ALF_TEMPPRED_NUM
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
#endif

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
#if !QC_ALF_IMPROVEMENT
    assert(pcAlfParam->filters_per_group_diff < pcAlfParam->filters_per_group);
#endif
    src = 0;
    for(ind = 0; ind < pcAlfParam->filters_per_group; ++ind)
    {
      if(pcAlfParam->codedVarBins[ind])
      {
#if QC_ALF_IMPROVEMENT
        memcpy(pfilterCoeffSym[ind],m_filterCoeffSymTmp[ind],sizeof(int)*pcAlfParam->num_coeff);
#else
        memcpy(pfilterCoeffSym[ind],m_filterCoeffSymTmp[src],sizeof(int)*pcAlfParam->num_coeff);
#endif
        ++src;
      }
      else
      {
        memset(pfilterCoeffSym[ind],0,sizeof(int)*pcAlfParam->num_coeff);
      }
    }
#if QC_ALF_IMPROVEMENT
    pcAlfParam->filters_per_group_diff = src;
    assert(pcAlfParam->filters_per_group_diff < pcAlfParam->filters_per_group);
#else
    assert(src == pcAlfParam->filters_per_group_diff);
#endif
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
#if QC_ALF_IMPROVEMENT
Void TComAdaptiveLoopFilter::calcVar(imgpel **imgY_var, imgpel *imgY_pad, int pad_size, int fl, int img_height, int img_width, int img_stride, int start_width , int start_height, int log2UnitSize )
#else
Void TComAdaptiveLoopFilter::calcVar(imgpel **imgY_var, imgpel *imgY_pad, int pad_size, int fl, int img_height, int img_width, int img_stride, int start_width , int start_height )
#endif
{
  Int i, j;

  Int end_height = start_height + img_height;
  Int end_width  = start_width + img_width;
  for(i = start_height; i < end_height; i+=m_ALF_WIN_VERSIZE)
  {
    for(j = start_width; j < end_width; j+=m_ALF_WIN_HORSIZE) 
    {
      Int nHeight = min( i + m_ALF_WIN_VERSIZE, end_height ) - i;
      Int nWidth  = min( j + m_ALF_WIN_HORSIZE, end_width  ) - j;
#if QC_ALF_IMPROVEMENT
      xCalcVar( imgY_var, imgY_pad, pad_size, fl, nHeight, nWidth, img_stride, j, i, log2UnitSize );
#else
      xCalcVar( imgY_var, imgY_pad, pad_size, fl, nHeight, nWidth, img_stride, j, i );
#endif
    }
  }
}
#if QC_ALF_IMPROVEMENT
Void TComAdaptiveLoopFilter::xCalcVarPerPixel(imgpel **imgY_var, imgpel *imgY_pad, int pad_size, int fl, int img_height, int img_width, int img_stride, int start_width , int start_height, int log2UnitSize )
{
  Int th[16] = {0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4};  
  fl              = 2;
  Int i, j;
#if FULL_NBIT
  Int shift= (11+ m_nBitIncrement + m_nInputBitDepth - 8);
#else
  Int shift= (11+ m_nBitIncrement);
#endif
  Int flplusOne = fl + 1;
  Int fl2plusTwo = 2*fl + 2; 
  Int var_max= 15;

  Int avg_var;
  Int mainDirection, secondaryDirection, dirTempHV, dirTempD;

  Int pixY; 
  Int iTempAct = 0;

  Int imgHExtended = img_height + fl2plusTwo;
  Int imgWExtended = img_width  + fl2plusTwo;
  Int start_height1 = start_height - flplusOne;

  for(i = 2; i < imgHExtended; i+=2)
  {
    Int yoffset = (i - 1 + start_height1) * img_stride - flplusOne;
    imgpel *p_imgY_pad_down = &imgY_pad[yoffset - img_stride  ];
    imgpel *p_imgY_pad      = &imgY_pad[yoffset               ];
    imgpel *p_imgY_pad_up   = &imgY_pad[yoffset + img_stride  ];
    imgpel *p_imgY_pad_up2  = &imgY_pad[yoffset + img_stride*2];
    for(j = 2; j < imgWExtended; j+=2)  
    {
      pixY = j - 1 + start_width;
      m_imgY_ver [i-2][j-2] = abs((p_imgY_pad   [pixY  ]<<1) - p_imgY_pad_down[pixY  ] - p_imgY_pad_up  [pixY  ])+   
                               abs((p_imgY_pad   [pixY+1]<<1) - p_imgY_pad_down[pixY+1] - p_imgY_pad_up  [pixY+1])+   
                               abs((p_imgY_pad_up[pixY  ]<<1) - p_imgY_pad     [pixY  ] - p_imgY_pad_up2 [pixY  ])+   
                               abs((p_imgY_pad_up[pixY+1]<<1) - p_imgY_pad     [pixY+1] - p_imgY_pad_up2 [pixY+1]);   

      m_imgY_hor [i-2][j-2] = abs((p_imgY_pad   [pixY  ]<<1) - p_imgY_pad     [pixY+1] - p_imgY_pad     [pixY-1])+        
                               abs((p_imgY_pad   [pixY+1]<<1) - p_imgY_pad     [pixY+2] - p_imgY_pad     [pixY  ])+        
                               abs((p_imgY_pad_up[pixY  ]<<1) - p_imgY_pad_up  [pixY+1] - p_imgY_pad_up  [pixY-1])+        
                               abs((p_imgY_pad_up[pixY+1]<<1) - p_imgY_pad_up  [pixY+2] - p_imgY_pad_up  [pixY  ]);        
      m_imgY_dig0[i-2][j-2] = abs((p_imgY_pad   [pixY  ]<<1) - p_imgY_pad_down[pixY-1] - p_imgY_pad_up  [pixY+1])+
                              abs((p_imgY_pad   [pixY+1]<<1) - p_imgY_pad_down[pixY  ] - p_imgY_pad_up  [pixY+2])+
                              abs((p_imgY_pad_up[pixY  ]<<1) - p_imgY_pad     [pixY-1] - p_imgY_pad_up2 [pixY+1])+
                              abs((p_imgY_pad_up[pixY+1]<<1) - p_imgY_pad     [pixY  ] - p_imgY_pad_up2 [pixY+2]);

      m_imgY_dig1[i-2][j-2] = abs((p_imgY_pad   [pixY  ]<<1) - p_imgY_pad_up  [pixY-1] - p_imgY_pad_down[pixY+1])+
                              abs((p_imgY_pad   [pixY+1]<<1) - p_imgY_pad_up  [pixY  ] - p_imgY_pad_down[pixY+2])+
                              abs((p_imgY_pad_up[pixY  ]<<1) - p_imgY_pad_up2 [pixY-1] - p_imgY_pad     [pixY+1])+
                              abs((p_imgY_pad_up[pixY+1]<<1) - p_imgY_pad_up2 [pixY  ] - p_imgY_pad     [pixY+2]);
      if (j > 4 )
      {
        m_imgY_ver [i-2][j-6] = m_imgY_ver [i-2][j-6]+m_imgY_ver [i-2][j-4]+m_imgY_ver [i-2][j-2];
        m_imgY_hor [i-2][j-6] = m_imgY_hor [i-2][j-6]+m_imgY_hor [i-2][j-4]+m_imgY_hor [i-2][j-2];
        m_imgY_dig0[i-2][j-6] = m_imgY_dig0[i-2][j-6]+m_imgY_dig0[i-2][j-4]+m_imgY_dig0[i-2][j-2];
        m_imgY_dig1[i-2][j-6] = m_imgY_dig1[i-2][j-6]+m_imgY_dig1[i-2][j-4]+m_imgY_dig1[i-2][j-2];
      }
    }
  }

  for(i = 0; i < img_height; i+=2)
  {
    for(j = 0; j < img_width; j+=2)  
    {
      Int sum_V   = m_imgY_ver [i][j] +m_imgY_ver [i+2][j]+m_imgY_ver [i+4][j];
      Int sum_H   = m_imgY_hor [i][j] +m_imgY_hor [i+2][j]+m_imgY_hor [i+4][j]; 
      Int sum_D0  = m_imgY_dig0[i][j] +m_imgY_dig0[i+2][j]+m_imgY_dig0[i+4][j];
      Int sum_D1  = m_imgY_dig1[i][j] +m_imgY_dig1[i+2][j]+m_imgY_dig1[i+4][j];
      iTempAct    = sum_V+sum_H;
      avg_var     = (imgpel) Clip_post(var_max, (iTempAct*24)>>(shift));     
      avg_var     = th[avg_var];
      Int HV_high, HV_low;
      Int D_high, D_low;
      Int HV_D_high, HV_D_low;
      if (sum_V>sum_H)
      {
        HV_high = sum_V;
        HV_low  = sum_H;
        dirTempHV=1;
      }
      else
      {
        HV_high = sum_H;
        HV_low  = sum_V;
        dirTempHV=3;
      }
      if (sum_D0 > sum_D1)
      {
        D_high = sum_D0;
        D_low  = sum_D1;
        dirTempD=0;
      }
      else
      {
        D_high = sum_D1;
        D_low  = sum_D0;
        dirTempD=2;
      }
      if ( D_high*HV_low > HV_high*D_low)
      {
        HV_D_high =  D_high;
        HV_D_low  =  D_low;
        mainDirection      = dirTempD;
        secondaryDirection = dirTempHV;
      }
      else
      {
        HV_D_high =  HV_high;
        HV_D_low  =  HV_low;
        mainDirection      = dirTempHV;
        secondaryDirection = dirTempD;
      }
      avg_var += ((2* (mainDirection) + (secondaryDirection)/2)<<NO_VALS_LAGR_SHIFT);
      if (HV_D_high > 2*HV_D_low)
      {
        avg_var += (8<<NO_VALS_LAGR_SHIFT);
      }
      if (HV_D_high*2 > 9*HV_D_low)
      {
        avg_var += (8<<NO_VALS_LAGR_SHIFT);
      }
      Int yOffset = (i + start_height);
      Int xOffset = (j + start_width );
      imgY_var[yOffset  ][xOffset  ] = imgY_var[yOffset  ][xOffset+1] = 
      imgY_var[yOffset+1][xOffset  ] = imgY_var[yOffset+1][xOffset+1] = avg_var;
    }
  }
}

Int TComAdaptiveLoopFilter::selectTransposeVarInd(Int varInd, Int *transpose)
{
    int aTransTable[8] ={0,1,0,2,2,3,1,3};
  int direction  = varInd>>NO_VALS_LAGR_SHIFT;
  int varIndMod  = varInd&((1<<NO_VALS_LAGR_SHIFT)-1);
  int dirRatio   = direction>>3;

  direction = direction&0x07;
  *transpose= aTransTable[direction];

  if (dirRatio)
  {
    varIndMod +=((direction&0x02)+dirRatio)*NO_VALS_LAGR;
  }
  return(varIndMod);
}

Void TComAdaptiveLoopFilter::xCalcVar(imgpel **imgY_var, imgpel *imgY_pad, int pad_size, int fl, int img_height, int img_width, int img_stride, int start_width , int start_height, int log2UnitSize  )
{
  xCalcVarPerPixel(imgY_var, imgY_pad, pad_size, fl, img_height, img_width, img_stride, start_width , start_height, log2UnitSize );
  return;
}
#else
Void TComAdaptiveLoopFilter::xCalcVar(imgpel **imgY_var, imgpel *imgY_pad, int pad_size, int fl, int img_height, int img_width, int img_stride, int start_width , int start_height )
{
  static Int shift_h     = (Int)(log((double)m_ALF_VAR_SIZE_H)/log(2.0));
  static Int shift_w     = (Int)(log((double)m_ALF_VAR_SIZE_W)/log(2.0));

  Int i, j;
  Int *p_imgY_temp;
#if FULL_NBIT
  Int shift= (11+ m_nBitIncrement + m_nInputBitDepth - 8);
#else
  Int shift= (11+ m_nBitIncrement);
#endif
  Int fl2plusOne= (m_VAR_SIZE<<1)+1; //3
  Int pad_offset = pad_size-fl-1;
  Int var_max= m_NO_VAR_BINS-1;
  Int mult_fact_int_tab[4]= {1,114,41,21};
  Int mult_fact_int = mult_fact_int_tab[m_VAR_SIZE];
  Int avg_var;
  Int vertical, horizontal;
  Int direction;
  Int step1 = m_NO_VAR_BINS/3 - 1;
  Int th[m_NO_VAR_BINS] = {0, 1, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4}; 
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
#endif

#if QC_ALF_IMPROVEMENT
#if ENABLE_FIXEDFILTER_INTERSLICE
Void TComAdaptiveLoopFilter::filterFrame(imgpel *imgYRecPost, imgpel *imgYRec, ALFParam* pcAlfPara, int stride)
#else
Void TComAdaptiveLoopFilter::filterFrame(imgpel *imgYRecPost, imgpel *imgYRec, ALFParam* pcAlfPara, int stride, Bool bIntraSlice)
#endif
#else
Void TComAdaptiveLoopFilter::filterFrame(imgpel *imgYRecPost, imgpel *imgYRec, int filtNo, int stride)
#endif
{
  Int i, j;
  for (i = 0; i < m_img_height; i+=m_ALF_WIN_VERSIZE)
  {
    for (j = 0; j < m_img_width; j+=m_ALF_WIN_HORSIZE)
    {
      Int nHeight = min( i + m_ALF_WIN_VERSIZE, m_img_height ) - i;
      Int nWidth  = min( j + m_ALF_WIN_HORSIZE, m_img_width  ) - j;
#if QC_ALF_IMPROVEMENT
      calcVar( m_imgY_var, imgYRec, m_FILTER_LENGTH/2, SHIFT_VAL_HALFW, nHeight, nWidth, stride , j , i, pcAlfPara->log2UnitSize );
#if ENABLE_FIXEDFILTER_INTERSLICE
      subfilterFrame(imgYRecPost, imgYRec, pcAlfPara, i, i + nHeight, j, j + nWidth, stride);
#else
      subfilterFrame(imgYRecPost, imgYRec, pcAlfPara, i, i + nHeight, j, j + nWidth, stride, bIntraSlice);
#endif
#else
      calcVar( m_imgY_var, imgYRec, m_FILTER_LENGTH/2, m_VAR_SIZE, nHeight, nWidth, stride , j , i );
      subfilterFrame(imgYRecPost, imgYRec, filtNo, i, i + nHeight, j, j + nWidth, stride );
#endif
    }
  }
}
#if QC_ALF_IMPROVEMENT
#if ENABLE_FIXEDFILTER_INTERSLICE
Void TComAdaptiveLoopFilter::subfilterFrame(imgpel *imgYRecPost, imgpel *imgYRec, ALFParam* pcAlfPara, int startHeight, int endHeight, int startWidth, int endWidth, int stride, Bool bChroma)
#else
Void TComAdaptiveLoopFilter::subfilterFrame(imgpel *imgYRecPost, imgpel *imgYRec, ALFParam* pcAlfPara, int startHeight, int endHeight, int startWidth, int endWidth, int stride, Bool bIntraSlice, Bool bChroma)
#endif
#else
Void TComAdaptiveLoopFilter::subfilterFrame(imgpel *imgYRecPost, imgpel *imgYRec, int filtNo, int startHeight, int endHeight, int startWidth, int endWidth, int stride)
#endif
{
#if QC_ALF_IMPROVEMENT
  Int filtNo            = pcAlfPara->realfiltNo;

  if(bChroma)
  {
    filtNo = (pcAlfPara->tap_chroma==9? 0: (pcAlfPara->tap_chroma==7 ? 1: 2));
  }
  else
  {
#if ENABLE_FIXEDFILTER_INTERSLICE
    filtNo = 0;
#else
    if(bIntraSlice)
    {
      filtNo = 0;
    }
#endif
  }
  Int varStepSizeWidth  = 1 << pcAlfPara->log2UnitSize;
  Int varStepSizeHeight = 1 << pcAlfPara->log2UnitSize;
#else
  Int varStepSizeWidth = m_ALF_VAR_SIZE_W;
  Int varStepSizeHeight = m_ALF_VAR_SIZE_H;
#endif
  Int shiftHeight = (Int)(log((double)varStepSizeHeight)/log(2.0));
  Int shiftWidth = (Int)(log((double)varStepSizeWidth)/log(2.0));
  Int i, j, pixelInt;
  imgpel *pImgYVar,*pImgYPad;
  imgpel *pImgYPad1,*pImgYPad2,*pImgYPad3,*pImgYPad4,*pImgYPad5,*pImgYPad6;

  Short *coef = m_filterCoeffShort[0];
  imgpel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;
  imgpel *pImgYRec;
#if QC_ALF_IMPROVEMENT
  imgpel *pImgYPad7, *pImgYPad8;
#endif
  imgpel *pClipTable = m_alfClipTable + m_alfClipOffset;

  Int numBitsMinus1= m_NUM_BITS-1;
  Int offset = (1<<(m_NUM_BITS-2));
#if QC_ALF_IMPROVEMENT
  if(bChroma)
  {
    pImgYVar = NULL;
  }
  Int transpose = 0;
#endif

  imgYRecPost += startHeight*stride;

  switch(filtNo)
  {
  case 2:
    for (i =  startHeight; i < endHeight; i++)
    {
#if QC_ALF_IMPROVEMENT
      if(!bChroma)
      {
        pImgYVar = m_imgY_var[i>>shiftHeight] + (startWidth>>shiftWidth);
      }
#else
      pImgYVar = m_imgY_var[i>>shiftHeight] + (startWidth>>shiftWidth);
#endif
      pImgYPad = imgYRec + i*stride;
      pImgYPad1 = imgYRec + (i+1)*stride;
      pImgYPad2 = imgYRec + (i-1)*stride;
      pImgYPad3 = imgYRec + (i+2)*stride;
      pImgYPad4 = imgYRec + (i-2)*stride;

      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
#if QC_ALF_IMPROVEMENT
        if(!bChroma)
        {
          Int varIndMod = selectTransposeVarInd(*(pImgYVar++), &transpose);
          coef = m_filterCoeffShort[varIndMod];
        }
        pixelInt= 0;
#else
        if ((j&(varStepSizeWidth-1))==0) coef = m_filterCoeffShort[*(pImgYVar++)];
        pixelInt = coef[m_MAX_SQR_FILT_LENGTH-1];
#endif
        pImg0 = pImgYPad  + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;
#if QC_ALF_IMPROVEMENT
        if(transpose==1)
        {
          pixelInt += coef[38]* (pImg3[+0]+pImg4[+0]);

          pixelInt += coef[30]* (pImg1[+1]+pImg2[-1]);
          pixelInt += coef[39]* (pImg1[+0]+pImg2[+0]);
          pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);

          pixelInt += coef[22]* (pImg0[-2]+pImg0[+2]);
          pixelInt += coef[31]* (pImg0[-1]+pImg0[+1]);
          pixelInt += coef[40]* (pImg0[+0]);          
        }
        else if(transpose == 3)
        {
          pixelInt += coef[38]* (pImg3[+0]+pImg4[+0]);

          pixelInt += coef[32]* (pImg1[+1]+pImg2[-1]);
          pixelInt += coef[39]* (pImg1[+0]+pImg2[+0]);
          pixelInt += coef[30]* (pImg1[-1]+pImg2[+1]);

          pixelInt += coef[22]* (pImg0[-2]+pImg0[+2]);
          pixelInt += coef[31]* (pImg0[-1]+pImg0[+1]);
          pixelInt += coef[40]* (pImg0[+0]);
        }
        else if(transpose == 2)
        {
          pixelInt += coef[22]* (pImg3[+0]+pImg4[+0]);

          pixelInt += coef[32]* (pImg1[+1]+pImg2[-1]);
          pixelInt += coef[31]* (pImg1[+0]+pImg2[+0]);
          pixelInt += coef[30]* (pImg1[-1]+pImg2[+1]);

          pixelInt += coef[38]* (pImg0[-2]+pImg0[+2]);
          pixelInt += coef[39]* (pImg0[-1]+pImg0[+1]);
          pixelInt += coef[40]* (pImg0[+0]);
        }
        else
        {
          pixelInt += coef[22]* (pImg3[+0]+pImg4[+0]);

          pixelInt += coef[30]* (pImg1[+1]+pImg2[-1]);
          pixelInt += coef[31]* (pImg1[+0]+pImg2[+0]);
          pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);

          pixelInt += coef[38]* (pImg0[-2]+pImg0[+2]);
          pixelInt += coef[39]* (pImg0[-1]+pImg0[+1]);
          pixelInt += coef[40]* (pImg0[+0]);
        }
#else

        pixelInt += coef[22]* (pImg3[+0]+pImg4[+0]);

        pixelInt += coef[30]* (pImg1[+1]+pImg2[-1]);
        pixelInt += coef[31]* (pImg1[+0]+pImg2[+0]);
        pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);

        pixelInt += coef[38]* (pImg0[-2]+pImg0[+2]);
        pixelInt += coef[39]* (pImg0[-1]+pImg0[+1]);
        pixelInt += coef[40]* (pImg0[+0]);
#endif
        pixelInt=(Int)((pixelInt+offset) >> (numBitsMinus1));
        *(pImgYRec++) = pClipTable[pixelInt];
      }
      imgYRecPost += stride;
    }
    break;

  case 1:
    for (i =  startHeight; i < endHeight; i++)
    {
#if QC_ALF_IMPROVEMENT
      if(!bChroma)
      {
        pImgYVar = m_imgY_var[i>>shiftHeight] + (startWidth>>shiftWidth);
      }
#else
      pImgYVar = m_imgY_var[i>>shiftHeight] + (startWidth>>shiftWidth);
#endif
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
#if QC_ALF_IMPROVEMENT
        if(!bChroma)
        {
          Int varIndMod = selectTransposeVarInd(*(pImgYVar++), &transpose);
          coef = m_filterCoeffShort[varIndMod];
        }
        pixelInt= 0;
#else
        if ((j&(varStepSizeWidth-1))==0) coef = m_filterCoeffShort[*(pImgYVar++)];
        pixelInt = coef[m_MAX_SQR_FILT_LENGTH-1];
#endif
        pImg0 = pImgYPad  + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;
        pImg5 = pImgYPad5 + j;
        pImg6 = pImgYPad6 + j;
#if QC_ALF_IMPROVEMENT
        if(transpose == 1)
        {
          pixelInt += coef[37]* (pImg5[0]+pImg6[0]);

          pixelInt += coef[29]* (pImg3[+1]+pImg4[-1]);
          pixelInt += coef[38]* (pImg3[+0]+pImg4[+0]);
          pixelInt += coef[33]* (pImg3[-1]+pImg4[+1]);

          pixelInt += coef[21]* (pImg1[+2]+pImg2[-2]);
          pixelInt += coef[30]* (pImg1[+1]+pImg2[-1]);
          pixelInt += coef[39]* (pImg1[+0]+pImg2[+0]);
          pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);
          pixelInt += coef[23]* (pImg1[-2]+pImg2[+2]);

          pixelInt += coef[13]* (pImg0[+3]+pImg0[-3]);
          pixelInt += coef[22]* (pImg0[+2]+pImg0[-2]);
          pixelInt += coef[31]* (pImg0[+1]+pImg0[-1]);
          pixelInt += coef[40]* (pImg0[+0]);
        }
        else if(transpose == 3)
        {
          pixelInt += coef[37]* (pImg5[0]+pImg6[0]);

          pixelInt += coef[33]* (pImg3[+1]+pImg4[-1]);
          pixelInt += coef[38]* (pImg3[+0]+pImg4[+0]);
          pixelInt += coef[29]* (pImg3[-1]+pImg4[+1]);

          pixelInt += coef[23]* (pImg1[+2]+pImg2[-2]);
          pixelInt += coef[32]* (pImg1[+1]+pImg2[-1]);
          pixelInt += coef[39]* (pImg1[+0]+pImg2[+0]);
          pixelInt += coef[30]* (pImg1[-1]+pImg2[+1]);
          pixelInt += coef[21]* (pImg1[-2]+pImg2[+2]);

          pixelInt += coef[13]* (pImg0[+3]+pImg0[-3]);
          pixelInt += coef[22]* (pImg0[+2]+pImg0[-2]);
          pixelInt += coef[31]* (pImg0[+1]+pImg0[-1]);
          pixelInt += coef[40]* (pImg0[+0]);
        }
        else if(transpose == 2)
        {
          pixelInt += coef[13]* (pImg5[0]+pImg6[0]);

          pixelInt += coef[23]* (pImg3[+1]+pImg4[-1]);
          pixelInt += coef[22]* (pImg3[+0]+pImg4[+0]);
          pixelInt += coef[21]* (pImg3[-1]+pImg4[+1]);

          pixelInt += coef[33]* (pImg1[+2]+pImg2[-2]);
          pixelInt += coef[32]* (pImg1[+1]+pImg2[-1]);
          pixelInt += coef[31]* (pImg1[+0]+pImg2[+0]);
          pixelInt += coef[30]* (pImg1[-1]+pImg2[+1]);
          pixelInt += coef[29]* (pImg1[-2]+pImg2[+2]);

          pixelInt += coef[37]* (pImg0[+3]+pImg0[-3]);
          pixelInt += coef[38]* (pImg0[+2]+pImg0[-2]);
          pixelInt += coef[39]* (pImg0[+1]+pImg0[-1]);
          pixelInt += coef[40]* (pImg0[+0]);
        }
        else
        {
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
        }
#else
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
#endif

        pixelInt=(Int)((pixelInt+offset) >> (numBitsMinus1));
        *(pImgYRec++) = pClipTable[pixelInt];
      }
      imgYRecPost += stride;
    }
    break;

  case 0:
    for (i =  startHeight; i < endHeight; i++)
    {
#if QC_ALF_IMPROVEMENT
      if(!bChroma)
      {
        pImgYVar = m_imgY_var[i>>shiftHeight] + (startWidth>>shiftWidth);
      }
#else
      pImgYVar = m_imgY_var[i>>shiftHeight] + (startWidth>>shiftWidth);
#endif
      pImgYPad = imgYRec + i*stride;
      pImgYPad1 = imgYRec + (i+1)*stride;
      pImgYPad2 = imgYRec + (i-1)*stride;
      pImgYPad3 = imgYRec + (i+2)*stride;
      pImgYPad4 = imgYRec + (i-2)*stride;
      pImgYPad5 = imgYRec + (i+3)*stride;
      pImgYPad6 = imgYRec + (i-3)*stride;
#if QC_ALF_IMPROVEMENT
      pImgYPad7 = imgYRec + (i+4)*stride;
      pImgYPad8 = imgYRec + (i-4)*stride;
#endif
      pImgYRec = imgYRecPost + startWidth;

      for (j = startWidth; j < endWidth; j++)
      {
#if QC_ALF_IMPROVEMENT
        if(!bChroma)
        {
          Int varIndMod = selectTransposeVarInd(*(pImgYVar++), &transpose);
          coef = m_filterCoeffShort[varIndMod];
        }
        pixelInt= 0;
#else
        if ((j&(varStepSizeWidth-1))==0) coef = m_filterCoeffShort[*(pImgYVar++)];
        pixelInt = coef[m_MAX_SQR_FILT_LENGTH-1];
#endif
        pImg0 = pImgYPad  + j;
        pImg1 = pImgYPad1 + j;
        pImg2 = pImgYPad2 + j;
        pImg3 = pImgYPad3 + j;
        pImg4 = pImgYPad4 + j;
        pImg5 = pImgYPad5 + j;
        pImg6 = pImgYPad6 + j;
#if QC_ALF_IMPROVEMENT
        if(transpose == 1)
        {
          pixelInt += coef[36]* (pImgYPad7[j] + pImgYPad8[j]);
          pixelInt += coef[28]* (pImg5[1]+pImg6[-1]);
          pixelInt += coef[37]* (pImg5[0]+pImg6[0]);
          pixelInt += coef[34]* (pImg5[-1]+pImg6[1]);

          pixelInt += coef[20]* (pImg3[2]+pImg4[-2]);
          pixelInt += coef[29]* (pImg3[1]+pImg4[-1]);
          pixelInt += coef[38]* (pImg3[0]+pImg4[0]);
          pixelInt += coef[33]* (pImg3[-1]+pImg4[+1]);
          pixelInt += coef[24]* (pImg3[-2]+pImg4[+2]);

          pixelInt += coef[12]* (pImg1[3]+pImg2[-3]);
          pixelInt += coef[21]* (pImg1[2]+pImg2[-2]);
          pixelInt += coef[30]* (pImg1[1]+pImg2[-1]);
          pixelInt += coef[39]* (pImg1[0]+pImg2[0]);
          pixelInt += coef[32]* (pImg1[-1]+pImg2[+1]);
          pixelInt += coef[23]* (pImg1[-2]+pImg2[+2]);
          pixelInt += coef[14]* (pImg1[-3]+pImg2[+3]);

          pixelInt += coef[ 4]* (pImg0[+4]+pImg0[-4]);
          pixelInt += coef[13]* (pImg0[+3]+pImg0[-3]);
          pixelInt += coef[22]* (pImg0[+2]+pImg0[-2]);
          pixelInt += coef[31]* (pImg0[+1]+pImg0[-1]);
          pixelInt += coef[40]* (pImg0[0]);
        }
        else if(transpose == 3)
        {
          pixelInt += coef[36]* (pImgYPad7[j] + pImgYPad8[j]);
          pixelInt += coef[34]* (pImg5[1]+pImg6[-1]);
          pixelInt += coef[37]* (pImg5[0]+pImg6[0]);
          pixelInt += coef[28]* (pImg5[-1]+pImg6[1]);

          pixelInt += coef[24]* (pImg3[2]+pImg4[-2]);
          pixelInt += coef[33]* (pImg3[1]+pImg4[-1]);
          pixelInt += coef[38]* (pImg3[0]+pImg4[0]);
          pixelInt += coef[29]* (pImg3[-1]+pImg4[+1]);
          pixelInt += coef[20]* (pImg3[-2]+pImg4[+2]);

          pixelInt += coef[14]* (pImg1[3]+pImg2[-3]);
          pixelInt += coef[23]* (pImg1[2]+pImg2[-2]);
          pixelInt += coef[32]* (pImg1[1]+pImg2[-1]);
          pixelInt += coef[39]* (pImg1[0]+pImg2[0]);
          pixelInt += coef[30]* (pImg1[-1]+pImg2[+1]);
          pixelInt += coef[21]* (pImg1[-2]+pImg2[+2]);
          pixelInt += coef[12]* (pImg1[-3]+pImg2[+3]);

          pixelInt += coef[ 4]* (pImg0[+4]+pImg0[-4]);
          pixelInt += coef[13]* (pImg0[+3]+pImg0[-3]);
          pixelInt += coef[22]* (pImg0[+2]+pImg0[-2]);
          pixelInt += coef[31]* (pImg0[+1]+pImg0[-1]);
          pixelInt += coef[40]* (pImg0[0]);
        }
        else if(transpose == 2)
        {
          pixelInt += coef[4]* (pImgYPad7[j] + pImgYPad8[j]);
          pixelInt += coef[14]* (pImg5[1]+pImg6[-1]);
          pixelInt += coef[13]* (pImg5[0]+pImg6[0]);
          pixelInt += coef[12]* (pImg5[-1]+pImg6[1]);

          pixelInt += coef[24]* (pImg3[2]+pImg4[-2]);
          pixelInt += coef[23]* (pImg3[1]+pImg4[-1]);
          pixelInt += coef[22]* (pImg3[0]+pImg4[0]);
          pixelInt += coef[21]* (pImg3[-1]+pImg4[+1]);
          pixelInt += coef[20]* (pImg3[-2]+pImg4[+2]);

          pixelInt += coef[34]* (pImg1[3]+pImg2[-3]);
          pixelInt += coef[33]* (pImg1[2]+pImg2[-2]);
          pixelInt += coef[32]* (pImg1[1]+pImg2[-1]);
          pixelInt += coef[31]* (pImg1[0]+pImg2[0]);
          pixelInt += coef[30]* (pImg1[-1]+pImg2[+1]);
          pixelInt += coef[29]* (pImg1[-2]+pImg2[+2]);
          pixelInt += coef[28]* (pImg1[-3]+pImg2[+3]);

          pixelInt += coef[36]* (pImg0[+4]+pImg0[-4]);
          pixelInt += coef[37]* (pImg0[+3]+pImg0[-3]);
          pixelInt += coef[38]* (pImg0[+2]+pImg0[-2]);
          pixelInt += coef[39]* (pImg0[+1]+pImg0[-1]);
          pixelInt += coef[40]* (pImg0[0]);
        }
        else
        {
          pixelInt += coef[4]* (pImgYPad7[j] + pImgYPad8[j]);
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
        }
#else
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
#endif
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
  for( UInt uiCUAddr = 0; uiCUAddr < pcPic->getNumberOfCtusInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = pcPic->getCtu( uiCUAddr );
    xSubCUAdaptive_qc(pcCU, pcAlfParam, imgY_rec_post, imgY_rec, 0, 0, Stride);
  }
}

Void TComAdaptiveLoopFilter::xSubCUAdaptive_qc(TComDataCU* pcCU, ALFParam* pcAlfParam, imgpel *imgY_rec_post, imgpel *imgY_rec, UInt uiAbsPartIdx, UInt uiDepth, Int Stride)
{
  TComPic* pcPic = pcCU->getPic();
  
  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (pcCU->getSlice()->getSPS()->getMaxCUWidth()>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (pcCU->getSlice()->getSPS()->getMaxCUHeight() >>uiDepth) - 1;
  
  // check picture boundary
  if ( ( uiRPelX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) || ( uiBPelY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
  {
    bBoundary = true;
  }
  
  // go to sub-CU?
  if ( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < pcCU->getSlice()->getSPS()->getLog2DiffMaxMinCodingBlockSize() ) && uiDepth < pcAlfParam->alf_max_depth ) || bBoundary )
  {
    UInt uiQNumParts = ( pcPic->getNumPartitionsInCtu() >> (uiDepth<<1) )>>2;
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
#if QC_ALF_IMPROVEMENT
    calcVar( m_imgY_var, imgY_rec, m_FILTER_LENGTH/2, SHIFT_VAL_HALFW, nHeight, nWidth, Stride , uiLPelX , uiTPelY, pcAlfParam->log2UnitSize);
#if ENABLE_FIXEDFILTER_INTERSLICE
    subfilterFrame(imgY_rec_post, imgY_rec, pcAlfParam, uiTPelY, min(uiBPelY+1,(unsigned int)(m_img_height)), uiLPelX, min(uiRPelX+1,(unsigned int)(m_img_width)), Stride );
#else
    subfilterFrame(imgY_rec_post, imgY_rec, pcAlfParam, uiTPelY, min(uiBPelY+1,(unsigned int)(m_img_height)), uiLPelX, min(uiRPelX+1,(unsigned int)(m_img_width)), Stride, (pcCU->getSlice()->isIntra()? true: false) );
#endif
#else
    calcVar( m_imgY_var, imgY_rec, m_FILTER_LENGTH/2, m_VAR_SIZE, nHeight, nWidth, Stride , uiLPelX , uiTPelY );
    subfilterFrame(imgY_rec_post, imgY_rec, pcAlfParam->realfiltNo, uiTPelY, min(uiBPelY+1,(unsigned int)(m_img_height)), uiLPelX, min(uiRPelX+1,(unsigned int)(m_img_width)), Stride);
#endif
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
#if QC_ALF_IMPROVEMENT && !EE_USE_HM3_CHROMA
    xFrameChroma( pcAlfParam, pcPicDec, pcPicRest, 0 );
#else
    xFrameChroma(pcPicDec, pcPicRest, pcAlfParam->coeff_chroma, pcAlfParam->tap_chroma, 0);
#endif
  }
  else
  {
    pcPicDec->copyToPic( pcPicRest , COMPONENT_Cb , false );
  }
  
  if(pcAlfParam->chroma_idc&0x01)
  {
#if QC_ALF_IMPROVEMENT && !EE_USE_HM3_CHROMA
    xFrameChroma( pcAlfParam, pcPicDec, pcPicRest, 1 );
#else
    xFrameChroma(pcPicDec, pcPicRest, pcAlfParam->coeff_chroma, pcAlfParam->tap_chroma, 1);
#endif
  }
  else
  {
    pcPicDec->copyToPic( pcPicRest , COMPONENT_Cr , false );
  }
}

/** 
 \param pcPicDec    picture before ALF
 \param pcPicRest   picture after  ALF
 \param qh          filter coefficient
 \param iTap        filter tap
 \param iColor      0 for Cb and 1 for Cr
 */
#if QC_ALF_IMPROVEMENT && !EE_USE_HM3_CHROMA
Void TComAdaptiveLoopFilter::xFrameChroma(ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Int iColor )
{
  imgpel* pDec;
  imgpel* pRest;

 
  Int iHeight = pcPicRest->getHeight(COMPONENT_Cb);
  Int iWidth = pcPicRest->getWidth(COMPONENT_Cb);
  Int iDecStride = pcPicDec->getStride(COMPONENT_Cb);
  if (iColor)
  {
    pDec  =  (imgpel*)pcPicDec->getAddr(COMPONENT_Cr);
    pRest =  (imgpel*)pcPicRest->getAddr(COMPONENT_Cr);
  }
  else
  {
    pDec  =  (imgpel*)pcPicDec->getAddr(COMPONENT_Cb);
    pRest =  (imgpel*)pcPicRest->getAddr(COMPONENT_Cb);
  }
#if ENABLE_FIXEDFILTER_INTERSLICE
  subfilterFrame(pRest, pDec, pcAlfParam, 0, iHeight, 0, iWidth, iDecStride, true);
#else
  subfilterFrame(pRest, pDec, pcAlfParam, 0, iHeight, 0, iWidth, iDecStride, false, true);
#endif
}
#else
Void TComAdaptiveLoopFilter::xFrameChroma( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Int *qh, Int iTap, Int iColor )
{
  Int i, x, y, value, N;//, offset;
  Pel PixSum[m_ALF_MAX_NUM_COEF]; 
  
  N      = (iTap*iTap+1)>>1;
  //offset = iTap>>1;
  Int iHeight = pcPicRest->getHeight(COMPONENT_Cb);
  Int iWidth = pcPicRest->getWidth(COMPONENT_Cb);
  Pel* pDec;
  Int iDecStride = pcPicDec->getStride(COMPONENT_Cb);
  
  Pel* pRest;
  Int iRestStride = pcPicRest->getStride(COMPONENT_Cb);
  
  Int iShift = m_nInputBitDepth + m_nBitIncrement - 8;
  
  if (iColor)
  {
    pDec = pcPicDec->getAddr(COMPONENT_Cr);
    pRest = pcPicRest->getAddr(COMPONENT_Cr);
  }
  else
  {
    pDec = pcPicDec->getAddr(COMPONENT_Cb);
    pRest = pcPicRest->getAddr(COMPONENT_Cb);
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
          value = (value + m_ALF_ROUND_OFFSET)>>m_ALF_NUM_BIT_SHIFT;
          
          pRest[x] = (Pel) Clip3(0, m_nIBDIMax, value);
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
          value = (value + m_ALF_ROUND_OFFSET)>>m_ALF_NUM_BIT_SHIFT;
          
          pRest[x] = (Pel) Clip3(0, m_nIBDIMax, value);
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
          value = (value + m_ALF_ROUND_OFFSET)>>m_ALF_NUM_BIT_SHIFT;
          
          pRest[x] = (Pel) Clip3(0, m_nIBDIMax, value);
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
#endif

Void TComAdaptiveLoopFilter::setNumCUsInFrame(TComPic *pcPic)
{
  m_uiNumCUsInFrame = pcPic->getNumberOfCtusInFrame();
}

#if COM16_C806_ALF_TEMPPRED_NUM
Void TComAdaptiveLoopFilter::setNumCUsInFrame(UInt uiNumCUsInFrame)
{
  m_uiNumCUsInFrame = uiNumCUsInFrame;
}
#endif

Void TComAdaptiveLoopFilter::setAlfCtrlFlags(ALFParam *pAlfParam, TComDataCU *pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt &idx)
{
  TComPic* pcPic = pcCU->getPic();
  UInt uiCurNumParts    = pcPic->getNumPartitionsInCtu() >> (uiDepth<<1);
  UInt uiQNumParts      = uiCurNumParts>>2;
  
  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (pcCU->getSlice()->getSPS()->getMaxCUWidth() >>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (pcCU->getSlice()->getSPS()->getMaxCUHeight() >>uiDepth) - 1;
  
  if( ( uiRPelX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) || ( uiBPelY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
  {
    bBoundary = true;
  }
  
  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < pcCU->getSlice()->getSPS()->getLog2DiffMaxMinCodingBlockSize() ) ) || bBoundary )
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
#if QC_ALF_IMPROVEMENT
Void TComAdaptiveLoopFilter::initFixedFilters()
{
  int factor = (1<<(TComAdaptiveLoopFilter::m_NUM_BITS-1));
  int maxFilterLength = TComAdaptiveLoopFilter::m_MAX_SQR_FILT_LENGTH/2 + 1;
  for(int i = 0; i < maxFilterLength; i++)
  {
    for (int j=0; j<25*NO_PREV_FILTERS; j++)
    {
      m_filterCoeffPrev[j][i]=(Double)m_ALFfilterCoeffFixed[j][i]/(Double)factor;
    }
  }
  memset( m_filterCoeffDefault, 0, (maxFilterLength-1)*sizeof(Double) );
  m_filterCoeffDefault[maxFilterLength - 1] = 1.0;
}
Void TComAdaptiveLoopFilter::resetALFPredParam(ALFParam *pAlfParam, Bool bIntra)
{
  //reset to 9x9 filter shape
#if !ENABLE_FIXEDFILTER_INTERSLICE
  if(bIntra)
#endif
  {
    pAlfParam->filtNo     = 0;
    pAlfParam->realfiltNo = 0;
    pAlfParam->tap        = 9;
    pAlfParam->tapV       = 9;
    pAlfParam->num_coeff  = TComAdaptiveLoopFilter::m_SQR_FILT_LENGTH_9SYM;
  }
}
#endif

#if FIX_TICKET12
Bool TComAdaptiveLoopFilter::refreshAlfTempPred( NalUnitType naluType , Int poc )
{
  static bool pendingRefresh = false;
  static Int pocLastCRA = 0;
  Bool refresh = false;

  if( pendingRefresh == true && pocLastCRA < poc )
  {
    refresh = true;
    pendingRefresh = false;   
  }

  if( NAL_UNIT_CODED_SLICE_BLA_W_LP <= naluType && naluType <= NAL_UNIT_CODED_SLICE_IDR_N_LP )
  {
    refresh = true;
    pendingRefresh = true;
    pocLastCRA = poc;
  }
  else if( naluType == NAL_UNIT_CODED_SLICE_CRA )
  {
    pendingRefresh = true;
    pocLastCRA = poc;
  }

  return( refresh );
}
#endif

#endif

