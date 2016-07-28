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

/** \file     TComRom.cpp
    \brief    global variables & functions
*/

#include "TComRom.h"
#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#if COM16_C806_EMT || COM16_C806_T64
#include <math.h>
#endif
#include <iomanip>
#include <assert.h>
#include "TComDataCU.h"
#include "Debug.h"
// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

//! \ingroup TLibCommon
//! \{
#if JVET_C0024_AMAX_BT
UInt g_uiBlkSize[ 10 ];
UInt g_uiNumBlk[ 10 ];
#if JVET_C0024_AMAX_BT_FIX
UInt g_uiPrevISlicePOC = 0;
Bool g_bInitAMaxBT = false;
#endif
#endif

#if VCEG_AZ08_KLT_COMMON
short **g_ppsEigenVector[USE_MORE_BLOCKSIZE_DEPTH_MAX];
#define MAX_KLTAREA (1<<(((USE_MORE_BLOCKSIZE_DEPTH_MAX)<<1) + 2))
#if VCEG_AZ08_INTER_KLT
Bool g_bEnableCheck = true;
#endif
Void reOrderCoeff(TCoeff *pcCoef, const UInt *scan, UInt uiWidth, UInt uiHeight)
{
    TCoeff coeff[MAX_KLTAREA];
    UInt uiMaxNumCoeff = uiWidth * uiHeight;
    memcpy(coeff, pcCoef, uiMaxNumCoeff*sizeof(TCoeff));

    for (UInt i = 0; i < uiMaxNumCoeff; i++)
    {
        pcCoef[scan[i]] = coeff[i];
    }
}
Void recoverOrderCoeff(TCoeff *pcCoef, const UInt *scan, UInt uiWidth, UInt uiHeight)
{
    TCoeff coeff[MAX_KLTAREA];
    UInt uiMaxNumCoeff = uiWidth * uiHeight;
    memcpy(coeff, pcCoef, uiMaxNumCoeff*sizeof(TCoeff));
    for (UInt i = 0; i < uiMaxNumCoeff; i++)
    {
        pcCoef[i] = coeff[scan[i]];
    }
}
#endif
#if VCEG_AZ08_INTRA_KLT
Int getZorder(Int iLCUX, Int iLCUY, Int NumInRow)
{
    //get raster id
    Int rasterId = (iLCUY >> 2)*NumInRow + (iLCUX >> 2);
    Int zOrder = g_auiRasterToZscan[rasterId];
    return zOrder;
}
#endif

const Char* nalUnitTypeToString(NalUnitType type)
{
  switch (type)
  {
  case NAL_UNIT_CODED_SLICE_TRAIL_R:    return "TRAIL_R";
  case NAL_UNIT_CODED_SLICE_TRAIL_N:    return "TRAIL_N";
  case NAL_UNIT_CODED_SLICE_TSA_R:      return "TSA_R";
  case NAL_UNIT_CODED_SLICE_TSA_N:      return "TSA_N";
  case NAL_UNIT_CODED_SLICE_STSA_R:     return "STSA_R";
  case NAL_UNIT_CODED_SLICE_STSA_N:     return "STSA_N";
  case NAL_UNIT_CODED_SLICE_BLA_W_LP:   return "BLA_W_LP";
  case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "BLA_W_RADL";
  case NAL_UNIT_CODED_SLICE_BLA_N_LP:   return "BLA_N_LP";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_RADL_R:     return "RADL_R";
  case NAL_UNIT_CODED_SLICE_RADL_N:     return "RADL_N";
  case NAL_UNIT_CODED_SLICE_RASL_R:     return "RASL_R";
  case NAL_UNIT_CODED_SLICE_RASL_N:     return "RASL_N";
  case NAL_UNIT_VPS:                    return "VPS";
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_FILLER_DATA:            return "FILLER";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  default:                              return "UNK";
  }
}

class ScanGenerator
{
private:
  UInt m_line, m_column;
  const UInt m_blockWidth, m_blockHeight;
  const UInt m_stride;
  const COEFF_SCAN_TYPE m_scanType;

public:
  ScanGenerator(UInt blockWidth, UInt blockHeight, UInt stride, COEFF_SCAN_TYPE scanType)
    : m_line(0), m_column(0), m_blockWidth(blockWidth), m_blockHeight(blockHeight), m_stride(stride), m_scanType(scanType)
  { }

  UInt GetCurrentX() const { return m_column; }
  UInt GetCurrentY() const { return m_line; }

  UInt GetNextIndex(UInt blockOffsetX, UInt blockOffsetY)
  {
    Int rtn=((m_line + blockOffsetY) * m_stride) + m_column + blockOffsetX;

    //advance line and column to the next position
    switch (m_scanType)
    {
      //------------------------------------------------

      case SCAN_DIAG:
        {
          if ((m_column == (m_blockWidth - 1)) || (m_line == 0)) //if we reach the end of a rank, go diagonally down to the next one
          {
            m_line   += m_column + 1;
            m_column  = 0;

            if (m_line >= m_blockHeight) //if that takes us outside the block, adjust so that we are back on the bottom row
            {
              m_column += m_line - (m_blockHeight - 1);
              m_line    = m_blockHeight - 1;
            }
          }
          else
          {
            m_column++;
            m_line--;
          }
        }
        break;

      //------------------------------------------------

      case SCAN_HOR:
        {
          if (m_column == (m_blockWidth - 1))
          {
            m_line++;
            m_column = 0;
          }
          else
          {
            m_column++;
          }
        }
        break;

      //------------------------------------------------

      case SCAN_VER:
        {
          if (m_line == (m_blockHeight - 1))
          {
            m_column++;
            m_line = 0;
          }
          else
          {
            m_line++;
          }
        }
        break;

      //------------------------------------------------

      default:
        {
          std::cerr << "ERROR: Unknown scan type \"" << m_scanType << "\"in ScanGenerator::GetNextIndex" << std::endl;
          exit(1);
        }
        break;
    }

    return rtn;
  }
};

// initialize ROM variables
Void initROM()
{
  Int i, c;

  // g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...
  ::memset( g_aucConvertToBit,   -1, sizeof( g_aucConvertToBit ) );
  c=0;
#if JVET_C0024_QTBT
  for ( i=1<<MIN_CU_LOG2; i<=MAX_CU_SIZE; i*=2 )
#else
  for ( i=4; i<=MAX_CU_SIZE; i*=2 )
#endif
  {
    g_aucConvertToBit[ i ] = c;
    c++;
  }
#if JVET_C0024_QTBT
  if (MIN_CU_LOG2>=1)
  {
    c = -1;
    for ( i=(1<<(MIN_CU_LOG2-1)); i>=1; i>>=1 )
    {
      g_aucConvertToBit[ i ] = c;
      c--;
    }
  }
#endif

#if COM16_C806_EMT || COM16_C806_T64
#if JVET_C0024_QTBT
  c = 2;
#else
  c = 4;
#endif

#if JVET_C0024_QTBT
  for ( i=0; i<7; i++ ) 
#else
  for ( i=0; i<5; i++ )
#endif
  {
    TMatrixCoeff *iT = NULL;
    const Double s = sqrt((Double)c) * ( 64 << COM16_C806_TRANS_PREC );
    const Double PI = 3.14159265358979323846;

    switch(i)
    {
#if JVET_C0024_QTBT
      case 0: iT = g_aiTr2 [0][0]; break; 
      case 1: iT = g_aiTr4 [0][0]; break;
      case 2: iT = g_aiTr8[0][0]; break;
      case 3: iT = g_aiTr16[0][0]; break;
      case 4: iT = g_aiTr32[0][0]; break;
      case 5: iT = g_aiTr64[0][0]; break;
      case 6: iT = g_aiTr128[0][0]; break;
      case 7: exit(0); break;
#else
    case 0: iT = g_aiTr4 [0][0]; break;
    case 1: iT = g_aiTr8 [0][0]; break;
    case 2: iT = g_aiTr16[0][0]; break;
    case 3: iT = g_aiTr32[0][0]; break;
    case 4: iT = g_aiTr64[0][0]; break;
    case 5: exit(0); break;
#endif
    }

    for( Int k=0; k<c; k++ )
    {
      for( Int n=0; n<c; n++ )
      {
        Double w0, w1, v;

        // DCT-II
        w0 = k==0 ? sqrt(0.5) : 1;
        v = cos(PI*(n+0.5)*k/c ) * w0 * sqrt(2.0/c);
        iT[DCT2*c*c + k*c + n] = (Short) ( s * v + ( v > 0 ? 0.5 : -0.5) );

        // DCT-V
        w0 = ( k==0 ) ? sqrt(0.5) : 1.0;
        w1 = ( n==0 ) ? sqrt(0.5) : 1.0;
        v = cos(PI*n*k/(c-0.5)) * w0 * w1 * sqrt(2.0/(c-0.5));
        iT[DCT5*c*c + k*c + n] = (Short) ( s * v + ( v > 0 ? 0.5 : -0.5) );

        // DCT-VIII
        v = cos(PI*(k+0.5)*(n+0.5)/(c+0.5) ) * sqrt(2.0/(c+0.5));
        iT[DCT8*c*c + k*c + n] = (Short) ( s * v + ( v > 0 ? 0.5 : -0.5) );

        // DST-I
        v = sin(PI*(n+1)*(k+1)/(c+1)) * sqrt(2.0/(c+1));
        iT[DST1*c*c + k*c + n] = (Short) ( s * v + ( v > 0 ? 0.5 : -0.5) );

        // DST-VII
        v = sin(PI*(k+0.5)*(n+1)/(c+0.5)) * sqrt(2.0/(c+0.5));
        iT[DST7*c*c + k*c + n] = (Short) ( s * v + ( v > 0 ? 0.5 : -0.5) );
      }
    }
    c <<= 1;
  }
#endif

#if VCEG_AZ07_INTRA_4TAP_FILTER
  for( i=17; i<32; i++ )
  {
    for( c=0; c<4; c++ )
    {
      g_aiIntraCubicFilter[i][c] = g_aiIntraCubicFilter[32-i][3-c];
      g_aiIntraGaussFilter[i][c] = g_aiIntraGaussFilter[32-i][3-c];
    }
  }
#endif

  // initialise scan orders
#if COM16_C806_T64
  for(UInt log2BlockHeight = 0; log2BlockHeight < MAX_LOG2_TU_SIZE_PLUS_ONE; log2BlockHeight++)
  {
    for(UInt log2BlockWidth = 0; log2BlockWidth < MAX_LOG2_TU_SIZE_PLUS_ONE; log2BlockWidth++)
#else
  for(UInt log2BlockHeight = 0; log2BlockHeight < MAX_CU_DEPTH; log2BlockHeight++)
  {
    for(UInt log2BlockWidth = 0; log2BlockWidth < MAX_CU_DEPTH; log2BlockWidth++)
#endif
    {
      const UInt blockWidth  = 1 << log2BlockWidth;
      const UInt blockHeight = 1 << log2BlockHeight;
      const UInt totalValues = blockWidth * blockHeight;

      //--------------------------------------------------------------------------------------------------

      //non-grouped scan orders

      for (UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const COEFF_SCAN_TYPE scanType = COEFF_SCAN_TYPE(scanTypeIndex);

        g_scanOrder[SCAN_UNGROUPED][scanType][log2BlockWidth][log2BlockHeight] = new UInt[totalValues];

#if VCEG_AZ07_CTX_RESIDUALCODING
        if (scanType == SCAN_VER && log2BlockWidth == 1 && log2BlockHeight == 1) 
        {
          for (UInt scanPosition = 0; scanPosition < totalValues; scanPosition++)
          {
            g_scanOrder[SCAN_UNGROUPED][scanTypeIndex][log2BlockWidth][log2BlockHeight][scanPosition] = g_scanOrder[SCAN_UNGROUPED][scanTypeIndex - 1][log2BlockWidth][log2BlockHeight][scanPosition];
          }
          continue;
        }
#endif
        ScanGenerator fullBlockScan(blockWidth, blockHeight, blockWidth, scanType);

        for (UInt scanPosition = 0; scanPosition < totalValues; scanPosition++)
        {
          g_scanOrder[SCAN_UNGROUPED][scanType][log2BlockWidth][log2BlockHeight][scanPosition] = fullBlockScan.GetNextIndex(0, 0);
        }
      }

      //--------------------------------------------------------------------------------------------------

      //grouped scan orders
#if VCEG_AZ07_CTX_RESIDUALCODING || JVET_C0024_QTBT
      UInt  groupWidth                 = 1           << MLS_CG_LOG2_WIDTH;
      UInt  groupHeight                = 1           << MLS_CG_LOG2_HEIGHT;
      UInt  widthInGroups              = blockWidth  >> MLS_CG_LOG2_WIDTH;
      UInt  heightInGroups             = blockHeight >> MLS_CG_LOG2_HEIGHT;
#else
      const UInt  groupWidth           = 1           << MLS_CG_LOG2_WIDTH;
      const UInt  groupHeight          = 1           << MLS_CG_LOG2_HEIGHT;
      const UInt  widthInGroups        = blockWidth  >> MLS_CG_LOG2_WIDTH;
      const UInt  heightInGroups       = blockHeight >> MLS_CG_LOG2_HEIGHT;
#endif

#if JVET_C0024_QTBT
      if (log2BlockWidth==1 || log2BlockHeight==1)
      {
        groupWidth                 = 1           << (MLS_CG_LOG2_WIDTH-1);
        groupHeight                = 1           << (MLS_CG_LOG2_HEIGHT-1);
        widthInGroups              = blockWidth  >> (MLS_CG_LOG2_WIDTH-1);
        heightInGroups             = blockHeight >> (MLS_CG_LOG2_HEIGHT-1);
      }
#endif

      const UInt  groupSize            = groupWidth    * groupHeight;
      const UInt  totalGroups          = widthInGroups * heightInGroups;
      for (UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const COEFF_SCAN_TYPE scanType = COEFF_SCAN_TYPE(scanTypeIndex);

        g_scanOrder[SCAN_GROUPED_4x4][scanType][log2BlockWidth][log2BlockHeight] = new UInt[totalValues];
#if VCEG_AZ07_CTX_RESIDUALCODING
        Bool bHorVerCGScan = (scanType && log2BlockWidth == 3 && log2BlockHeight == 3) ;
        if ( bHorVerCGScan ) 
        {
          for (UInt scanPosition = 0; scanPosition < totalValues; scanPosition++)
          {
            g_scanOrder[SCAN_GROUPED_4x4][scanType][log2BlockWidth][log2BlockHeight][scanPosition] = g_scanOrder[SCAN_UNGROUPED][scanType][log2BlockWidth][log2BlockHeight][scanPosition];
          }
        }
        else
        {
#endif
        ScanGenerator fullBlockScan(widthInGroups, heightInGroups, groupWidth, scanType);

        for (UInt groupIndex = 0; groupIndex < totalGroups; groupIndex++)
        {
          const UInt groupPositionY  = fullBlockScan.GetCurrentY();
          const UInt groupPositionX  = fullBlockScan.GetCurrentX();
          const UInt groupOffsetX    = groupPositionX * groupWidth;
          const UInt groupOffsetY    = groupPositionY * groupHeight;
          const UInt groupOffsetScan = groupIndex     * groupSize;

          ScanGenerator groupScan(groupWidth, groupHeight, blockWidth, scanType);

          for (UInt scanPosition = 0; scanPosition < groupSize; scanPosition++)
          {
            g_scanOrder[SCAN_GROUPED_4x4][scanType][log2BlockWidth][log2BlockHeight][groupOffsetScan + scanPosition] = groupScan.GetNextIndex(groupOffsetX, groupOffsetY);
          }

          fullBlockScan.GetNextIndex(0,0);
        }
#if VCEG_AZ07_CTX_RESIDUALCODING
        }
#endif
      }

      //--------------------------------------------------------------------------------------------------
    }
  }
}

Void destroyROM()
{
  for(UInt groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++)
  {
    for (UInt scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++)
    {
#if COM16_C806_T64
      for (UInt log2BlockWidth = 0; log2BlockWidth < MAX_LOG2_TU_SIZE_PLUS_ONE; log2BlockWidth++)
      {
        for (UInt log2BlockHeight = 0; log2BlockHeight < MAX_LOG2_TU_SIZE_PLUS_ONE; log2BlockHeight++)
#else
      for (UInt log2BlockWidth = 0; log2BlockWidth < MAX_CU_DEPTH; log2BlockWidth++)
      {
        for (UInt log2BlockHeight = 0; log2BlockHeight < MAX_CU_DEPTH; log2BlockHeight++)
#endif
        {
          delete [] g_scanOrder[groupTypeIndex][scanOrderIndex][log2BlockWidth][log2BlockHeight];
        }
      }
    }
  }
}

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================
UInt g_auiZscanToRaster [ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ] = { 0, };
UInt g_auiRasterToZscan [ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ] = { 0, };
UInt g_auiRasterToPelX  [ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ] = { 0, };
UInt g_auiRasterToPelY  [ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ] = { 0, };

#if !JVET_C0024_QTBT
const UInt g_auiPUOffset[NUMBER_OF_PART_SIZES] = { 0, 8, 4, 4, 2, 10, 1, 5};
#endif
#if VCEG_AZ07_CTX_RESIDUALCODING
const UInt g_auiGoRiceRange[MAX_GR_ORDER_RESIDUAL] =
{
  6, 5, 6, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION
};
#if !COM16_C806_T64
const UInt g_uiLastCtx[ 28 ]    =         //!!!!to be modified for when COM16_C806_T64 = 1  
{
  0,   1,  2,  2,                         // 4x4    4
  3,   4,  5,  5, 2,  2,                  // 8x8    6  
  6,   7,  8,  8, 9,  9, 2, 2,            // 16x16  8
  10, 11, 12, 12, 13, 13, 14, 14, 2, 2    // 32x32  10
                                          // 64x64  12    
};
#endif
#endif
Void initZscanToRaster ( Int iMaxDepth, Int iDepth, UInt uiStartVal, UInt*& rpuiCurrIdx )
{
  Int iStride = 1 << ( iMaxDepth - 1 );

  if ( iDepth == iMaxDepth )
  {
    rpuiCurrIdx[0] = uiStartVal;
    rpuiCurrIdx++;
  }
  else
  {
    Int iStep = iStride >> iDepth;
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal,                     rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep,               rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep*iStride,       rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep*iStride+iStep, rpuiCurrIdx );
  }
}

Void initRasterToZscan ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth )
{
  UInt  uiMinCUWidth  = uiMaxCUWidth  >> ( uiMaxDepth - 1 );
  UInt  uiMinCUHeight = uiMaxCUHeight >> ( uiMaxDepth - 1 );

  UInt  uiNumPartInWidth  = (UInt)uiMaxCUWidth  / uiMinCUWidth;
  UInt  uiNumPartInHeight = (UInt)uiMaxCUHeight / uiMinCUHeight;

  for ( UInt i = 0; i < uiNumPartInWidth*uiNumPartInHeight; i++ )
  {
    g_auiRasterToZscan[ g_auiZscanToRaster[i] ] = i;
  }
}

Void initRasterToPelXY ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth )
{
  UInt    i;

  UInt* uiTempX = &g_auiRasterToPelX[0];
  UInt* uiTempY = &g_auiRasterToPelY[0];

  UInt  uiMinCUWidth  = uiMaxCUWidth  >> ( uiMaxDepth - 1 );
  UInt  uiMinCUHeight = uiMaxCUHeight >> ( uiMaxDepth - 1 );

  UInt  uiNumPartInWidth  = uiMaxCUWidth  / uiMinCUWidth;
  UInt  uiNumPartInHeight = uiMaxCUHeight / uiMinCUHeight;

  uiTempX[0] = 0; uiTempX++;
  for ( i = 1; i < uiNumPartInWidth; i++ )
  {
    uiTempX[0] = uiTempX[-1] + uiMinCUWidth; uiTempX++;
  }
  for ( i = 1; i < uiNumPartInHeight; i++ )
  {
    memcpy(uiTempX, uiTempX-uiNumPartInWidth, sizeof(UInt)*uiNumPartInWidth);
    uiTempX += uiNumPartInWidth;
  }

  for ( i = 1; i < uiNumPartInWidth*uiNumPartInHeight; i++ )
  {
    uiTempY[i] = ( i / uiNumPartInWidth ) * uiMinCUWidth;
  }
}

const Int g_quantScales[SCALING_LIST_REM_NUM] =
{
  26214,23302,20560,18396,16384,14564
};

const Int g_invQuantScales[SCALING_LIST_REM_NUM] =
{
  40,45,51,57,64,72
};

//--------------------------------------------------------------------------------------------------

//structures

#define DEFINE_DST4x4_MATRIX(a,b,c,d) \
{ \
  {  a,  b,  c,  d }, \
  {  c,  c,  0, -c }, \
  {  d, -a, -c,  b }, \
  {  b, -d,  c, -a }, \
}

#define DEFINE_DCT4x4_MATRIX(a,b,c) \
{ \
  { a,  a,  a,  a}, \
  { b,  c, -c, -b}, \
  { a, -a, -a,  a}, \
  { c, -b,  b, -c}  \
}

#define DEFINE_DCT8x8_MATRIX(a,b,c,d,e,f,g) \
{ \
  { a,  a,  a,  a,  a,  a,  a,  a}, \
  { d,  e,  f,  g, -g, -f, -e, -d}, \
  { b,  c, -c, -b, -b, -c,  c,  b}, \
  { e, -g, -d, -f,  f,  d,  g, -e}, \
  { a, -a, -a,  a,  a, -a, -a,  a}, \
  { f, -d,  g,  e, -e, -g,  d, -f}, \
  { c, -b,  b, -c, -c,  b, -b,  c}, \
  { g, -f,  e, -d,  d, -e,  f, -g}  \
}

#define DEFINE_DCT16x16_MATRIX(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o) \
{ \
  { a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a}, \
  { h,  i,  j,  k,  l,  m,  n,  o, -o, -n, -m, -l, -k, -j, -i, -h}, \
  { d,  e,  f,  g, -g, -f, -e, -d, -d, -e, -f, -g,  g,  f,  e,  d}, \
  { i,  l,  o, -m, -j, -h, -k, -n,  n,  k,  h,  j,  m, -o, -l, -i}, \
  { b,  c, -c, -b, -b, -c,  c,  b,  b,  c, -c, -b, -b, -c,  c,  b}, \
  { j,  o, -k, -i, -n,  l,  h,  m, -m, -h, -l,  n,  i,  k, -o, -j}, \
  { e, -g, -d, -f,  f,  d,  g, -e, -e,  g,  d,  f, -f, -d, -g,  e}, \
  { k, -m, -i,  o,  h,  n, -j, -l,  l,  j, -n, -h, -o,  i,  m, -k}, \
  { a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a}, \
  { l, -j, -n,  h, -o, -i,  m,  k, -k, -m,  i,  o, -h,  n,  j, -l}, \
  { f, -d,  g,  e, -e, -g,  d, -f, -f,  d, -g, -e,  e,  g, -d,  f}, \
  { m, -h,  l,  n, -i,  k,  o, -j,  j, -o, -k,  i, -n, -l,  h, -m}, \
  { c, -b,  b, -c, -c,  b, -b,  c,  c, -b,  b, -c, -c,  b, -b,  c}, \
  { n, -k,  h, -j,  m,  o, -l,  i, -i,  l, -o, -m,  j, -h,  k, -n}, \
  { g, -f,  e, -d,  d, -e,  f, -g, -g,  f, -e,  d, -d,  e, -f,  g}, \
  { o, -n,  m, -l,  k, -j,  i, -h,  h, -i,  j, -k,  l, -m,  n, -o}  \
}

#define DEFINE_DCT32x32_MATRIX(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,A,B,C,D,E) \
{ \
  { a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a,  a}, \
  { p,  q,  r,  s,  t,  u,  v,  w,  x,  y,  z,  A,  B,  C,  D,  E, -E, -D, -C, -B, -A, -z, -y, -x, -w, -v, -u, -t, -s, -r, -q, -p}, \
  { h,  i,  j,  k,  l,  m,  n,  o, -o, -n, -m, -l, -k, -j, -i, -h, -h, -i, -j, -k, -l, -m, -n, -o,  o,  n,  m,  l,  k,  j,  i,  h}, \
  { q,  t,  w,  z,  C, -E, -B, -y, -v, -s, -p, -r, -u, -x, -A, -D,  D,  A,  x,  u,  r,  p,  s,  v,  y,  B,  E, -C, -z, -w, -t, -q}, \
  { d,  e,  f,  g, -g, -f, -e, -d, -d, -e, -f, -g,  g,  f,  e,  d,  d,  e,  f,  g, -g, -f, -e, -d, -d, -e, -f, -g,  g,  f,  e,  d}, \
  { r,  w,  B, -D, -y, -t, -p, -u, -z, -E,  A,  v,  q,  s,  x,  C, -C, -x, -s, -q, -v, -A,  E,  z,  u,  p,  t,  y,  D, -B, -w, -r}, \
  { i,  l,  o, -m, -j, -h, -k, -n,  n,  k,  h,  j,  m, -o, -l, -i, -i, -l, -o,  m,  j,  h,  k,  n, -n, -k, -h, -j, -m,  o,  l,  i}, \
  { s,  z, -D, -w, -p, -v, -C,  A,  t,  r,  y, -E, -x, -q, -u, -B,  B,  u,  q,  x,  E, -y, -r, -t, -A,  C,  v,  p,  w,  D, -z, -s}, \
  { b,  c, -c, -b, -b, -c,  c,  b,  b,  c, -c, -b, -b, -c,  c,  b,  b,  c, -c, -b, -b, -c,  c,  b,  b,  c, -c, -b, -b, -c,  c,  b}, \
  { t,  C, -y, -p, -x,  D,  u,  s,  B, -z, -q, -w,  E,  v,  r,  A, -A, -r, -v, -E,  w,  q,  z, -B, -s, -u, -D,  x,  p,  y, -C, -t}, \
  { j,  o, -k, -i, -n,  l,  h,  m, -m, -h, -l,  n,  i,  k, -o, -j, -j, -o,  k,  i,  n, -l, -h, -m,  m,  h,  l, -n, -i, -k,  o,  j}, \
  { u, -E, -t, -v,  D,  s,  w, -C, -r, -x,  B,  q,  y, -A, -p, -z,  z,  p,  A, -y, -q, -B,  x,  r,  C, -w, -s, -D,  v,  t,  E, -u}, \
  { e, -g, -d, -f,  f,  d,  g, -e, -e,  g,  d,  f, -f, -d, -g,  e,  e, -g, -d, -f,  f,  d,  g, -e, -e,  g,  d,  f, -f, -d, -g,  e}, \
  { v, -B, -p, -C,  u,  w, -A, -q, -D,  t,  x, -z, -r, -E,  s,  y, -y, -s,  E,  r,  z, -x, -t,  D,  q,  A, -w, -u,  C,  p,  B, -v}, \
  { k, -m, -i,  o,  h,  n, -j, -l,  l,  j, -n, -h, -o,  i,  m, -k, -k,  m,  i, -o, -h, -n,  j,  l, -l, -j,  n,  h,  o, -i, -m,  k}, \
  { w, -y, -u,  A,  s, -C, -q,  E,  p,  D, -r, -B,  t,  z, -v, -x,  x,  v, -z, -t,  B,  r, -D, -p, -E,  q,  C, -s, -A,  u,  y, -w}, \
  { a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a,  a, -a, -a,  a}, \
  { x, -v, -z,  t,  B, -r, -D,  p, -E, -q,  C,  s, -A, -u,  y,  w, -w, -y,  u,  A, -s, -C,  q,  E, -p,  D,  r, -B, -t,  z,  v, -x}, \
  { l, -j, -n,  h, -o, -i,  m,  k, -k, -m,  i,  o, -h,  n,  j, -l, -l,  j,  n, -h,  o,  i, -m, -k,  k,  m, -i, -o,  h, -n, -j,  l}, \
  { y, -s, -E,  r, -z, -x,  t,  D, -q,  A,  w, -u, -C,  p, -B, -v,  v,  B, -p,  C,  u, -w, -A,  q, -D, -t,  x,  z, -r,  E,  s, -y}, \
  { f, -d,  g,  e, -e, -g,  d, -f, -f,  d, -g, -e,  e,  g, -d,  f,  f, -d,  g,  e, -e, -g,  d, -f, -f,  d, -g, -e,  e,  g, -d,  f}, \
  { z, -p,  A,  y, -q,  B,  x, -r,  C,  w, -s,  D,  v, -t,  E,  u, -u, -E,  t, -v, -D,  s, -w, -C,  r, -x, -B,  q, -y, -A,  p, -z}, \
  { m, -h,  l,  n, -i,  k,  o, -j,  j, -o, -k,  i, -n, -l,  h, -m, -m,  h, -l, -n,  i, -k, -o,  j, -j,  o,  k, -i,  n,  l, -h,  m}, \
  { A, -r,  v, -E, -w,  q, -z, -B,  s, -u,  D,  x, -p,  y,  C, -t,  t, -C, -y,  p, -x, -D,  u, -s,  B,  z, -q,  w,  E, -v,  r, -A}, \
  { c, -b,  b, -c, -c,  b, -b,  c,  c, -b,  b, -c, -c,  b, -b,  c,  c, -b,  b, -c, -c,  b, -b,  c,  c, -b,  b, -c, -c,  b, -b,  c}, \
  { B, -u,  q, -x,  E,  y, -r,  t, -A, -C,  v, -p,  w, -D, -z,  s, -s,  z,  D, -w,  p, -v,  C,  A, -t,  r, -y, -E,  x, -q,  u, -B}, \
  { n, -k,  h, -j,  m,  o, -l,  i, -i,  l, -o, -m,  j, -h,  k, -n, -n,  k, -h,  j, -m, -o,  l, -i,  i, -l,  o,  m, -j,  h, -k,  n}, \
  { C, -x,  s, -q,  v, -A, -E,  z, -u,  p, -t,  y, -D, -B,  w, -r,  r, -w,  B,  D, -y,  t, -p,  u, -z,  E,  A, -v,  q, -s,  x, -C}, \
  { g, -f,  e, -d,  d, -e,  f, -g, -g,  f, -e,  d, -d,  e, -f,  g,  g, -f,  e, -d,  d, -e,  f, -g, -g,  f, -e,  d, -d,  e, -f,  g}, \
  { D, -A,  x, -u,  r, -p,  s, -v,  y, -B,  E,  C, -z,  w, -t,  q, -q,  t, -w,  z, -C, -E,  B, -y,  v, -s,  p, -r,  u, -x,  A, -D}, \
  { o, -n,  m, -l,  k, -j,  i, -h,  h, -i,  j, -k,  l, -m,  n, -o, -o,  n, -m,  l, -k,  j, -i,  h, -h,  i, -j,  k, -l,  m, -n,  o}, \
  { E, -D,  C, -B,  A, -z,  y, -x,  w, -v,  u, -t,  s, -r,  q, -p,  p, -q,  r, -s,  t, -u,  v, -w,  x, -y,  z, -A,  B, -C,  D, -E}  \
}

#if COM16_C806_EMT
Int g_aiTrSubsetIntra[3][2] = { {DST7, DCT8}, {DST7, DST1}, {DST7, DCT5} };
Int g_aiTrSubsetInter   [4] =   {DCT8, DST7};

#if VCEG_AZ07_INTRA_65ANG_MODES
const UChar g_aucTrSetVert[NUM_INTRA_MODE-1] =
{//0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66
   2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0
};
const UChar g_aucTrSetHorz[NUM_INTRA_MODE-1] =
{//0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66
   2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0
};
#else
const UChar g_aucTrSetVert[35] =
{//0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34
   2, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 2, 2, 2, 2, 2, 1, 0, 1, 0, 1, 0
};
const UChar g_aucTrSetHorz[35] =
{//0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34
   2, 1, 0, 1, 0, 1, 0, 1, 2, 2, 2, 2, 2, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0
};
#endif

const UInt g_iEmtSigNumThr = 2;
#endif

#if COM16_C806_EMT || COM16_C806_T64
#if JVET_C0024_QTBT
TMatrixCoeff g_aiTr2 [NUM_TRANS_TYPE][ 2][ 2];
TMatrixCoeff g_aiTr128 [NUM_TRANS_TYPE][ 128][ 128];
#endif
TMatrixCoeff g_aiTr4 [NUM_TRANS_TYPE][ 4][ 4];
TMatrixCoeff g_aiTr8 [NUM_TRANS_TYPE][ 8][ 8];
TMatrixCoeff g_aiTr16[NUM_TRANS_TYPE][16][16];
TMatrixCoeff g_aiTr32[NUM_TRANS_TYPE][32][32];
TMatrixCoeff g_aiTr64[NUM_TRANS_TYPE][64][64];
#endif

#if COM16_C1044_NSST
#if VCEG_AZ07_INTRA_65ANG_MODES
const UChar g_NsstLut[NUM_INTRA_MODE-1] = 
{//0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66
   0, 0, 1, 2, 1, 2, 1, 2, 3, 4, 3, 4, 3, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 9, 8, 9, 8, 9,10,11,10,11,10,11,10,11,10,11,10, 9, 8, 9, 8, 9, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 3, 4, 3, 4, 3, 2, 1, 2, 1, 2, 1
};
#else
const UChar g_NsstLut[NUM_INTRA_MODE-1] = 
{//0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34
   0, 0, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9,10,10,11,11,11,10,10, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1
};
#endif

#if VCEG_AZ07_CTX_RESIDUALCODING
const UInt g_auiCoefScanFirstCG8x8[ 3 ][ 16 ] = {
  { 0, 8,  1, 16, 9, 2, 24, 17, 10,  3, 25, 18, 11, 26, 19, 27 },
  { 0, 1,  2,  3, 8, 9, 10, 11, 16, 17, 18, 19, 24, 25, 26, 27 },
  { 0, 8, 16, 24, 1, 9, 17, 25,  2, 10, 18, 26,  3, 11, 19, 27 },
};
#endif

const Int g_aiNsst4x4[12][3][16][16] = 
{
  { // 0
    {
      {    246,    -36,    -35,     -2,    -41,      9,      6,      0,    -27,      6,      6,     -1,     -2,      0,     -1,      0 },
      {    -42,   -220,     39,     26,   -100,     20,     18,     -3,     28,     38,     -9,     -6,      5,     -2,     -2,      1 },
      {    -18,    106,      3,    -17,   -222,    -16,     51,      4,     24,    -19,     -6,      3,     23,      1,     -7,      0 },
      {    -30,    -36,   -150,     17,    -11,   -181,      6,     30,    -59,     17,     45,     -4,      9,     25,     -2,     -5 },
      {    -30,     -3,   -192,     13,     -8,    150,      2,    -30,     50,      4,     29,      0,      5,    -24,     -2,      6 },
      {    -31,     -7,     18,      7,    -24,     76,     13,    -15,   -230,    -33,     49,      4,     26,     -7,     -6,      3 },
      {      3,     22,     10,     16,     47,      8,    208,      1,     -6,    127,      2,    -28,     13,     -6,    -43,      1 },
      {     -2,     38,     11,    -11,    -28,     11,   -129,     -6,    -23,    204,     10,    -42,     46,     -3,     10,      3 },
      {     11,     21,      8,    215,      6,     -2,    -18,      5,     11,    -31,    -41,    -37,    116,      9,    -23,      3 },
      {     -4,     25,      4,    117,    -27,      1,     -5,      3,    -21,     28,      4,    -34,   -209,    -38,     54,      7 },
      {     -9,     -7,    -48,    -33,     -2,    -13,      0,    -29,    -55,      6,   -233,     -8,     -7,    -55,     -5,     17 },
      {      4,     -4,     14,     -3,      3,    -36,     -7,      5,     13,    -11,     54,     11,     32,   -238,    -27,     49 },
      {      3,      1,     10,      7,      0,    -44,      5,   -246,      8,     -5,     27,     -7,      1,     16,      7,     42 },
      {     -2,      0,      0,     -2,     -8,      0,    -42,     -9,      0,     -8,      4,    -30,    -61,     16,   -241,    -22 },
      {      1,      8,      1,     48,     -3,     -1,     -7,     -8,     -5,     51,    -13,    243,    -12,      7,    -28,      2 },
      {      0,      0,      0,     -5,      0,      7,     -2,     44,      1,     -1,      2,     -3,     -8,     51,    -17,    246 },
    },
    {
      {   -242,     48,     31,      1,     51,     -7,     -8,     -1,     26,    -10,     -3,      1,      1,      1,      1,      0 },
      {    -60,   -155,     43,     12,   -173,     30,     37,     -2,     45,     34,    -14,     -4,     15,     -7,     -3,      1 },
      {     -4,   -187,     13,     28,    161,    -21,    -30,      2,    -25,     32,      0,     -6,    -23,      6,      6,     -1 },
      {    -31,    -10,    -63,     12,    -49,   -189,     37,     26,   -120,     30,     51,     -7,     25,     34,    -12,     -5 },
      {     22,     25,    225,    -16,     -1,    -10,     20,     -3,   -111,     -1,    -20,      2,      1,     11,     -4,     -1 },
      {     38,     12,     68,    -12,     14,   -154,      4,     27,    175,      6,    -52,      3,    -24,     18,      1,     -6 },
      {      6,     41,     -6,     10,     44,     31,    118,    -12,     18,    198,    -23,    -42,     50,    -21,    -46,      8 },
      {      2,    -19,     -9,     26,     36,      9,    212,     -4,     10,   -125,     10,     11,    -42,      1,    -22,      3 },
      {     10,    -15,      7,     41,     24,     -6,    -10,      3,     16,    -71,    -34,      9,    232,      0,    -50,      5 },
      {     -7,    -33,    -16,   -241,     17,     -2,     24,    -27,      1,     -1,     12,     53,     37,      6,    -17,      1 },
      {    -10,     -4,    -44,    -10,     -9,    -32,      1,    -33,    -59,    -16,   -212,      5,    -26,   -100,     14,     38 },
      {      5,     -2,     27,     -3,      5,    -30,     -2,     27,     15,     -5,    104,      5,     13,   -224,     16,     40 },
      {      4,      1,      9,     24,     -4,    -37,    -11,   -243,     17,     -6,     41,    -18,      0,      4,     -3,     45 },
      {      3,      4,      0,      2,      9,      1,     44,     -6,      3,     18,     -1,     19,     58,     20,    243,      6 },
      {      1,      7,      0,     54,      0,      0,      2,    -13,      0,     43,      2,    245,    -10,      1,    -22,      5 },
      {      0,      0,      2,     -4,      0,      8,     -4,     47,      3,      0,      9,     -3,     -5,     53,     -7,    245 },
    },
    {
      {   -251,     21,     33,      5,     18,     -7,      1,     -1,     27,     -1,     -6,      0,      6,     -1,      0,      0 },
      {     23,    194,    -14,    -21,    152,     40,    -28,     -9,     -4,    -32,     -5,      4,    -14,     -8,      4,      2 },
      {      0,   -150,    -31,     19,    192,    -10,    -49,      0,     26,     33,     -2,     -4,    -20,      2,      7,      0 },
      {    -19,     20,   -120,     -2,     -7,   -187,    -56,     27,    -76,    -62,     29,     16,     -3,     22,     14,     -3 },
      {     26,     -5,    203,     19,     47,   -102,     41,     26,    -73,    -44,    -26,      4,    -19,      8,     -2,     -4 },
      {     33,     34,     24,     -5,    -10,   -105,    -23,     17,    212,     19,    -60,     -3,     34,     23,    -10,     -4 },
      {      1,     45,     -6,      1,     22,    -53,     79,     15,    -32,    205,     74,    -34,     23,     58,     -4,    -16 },
      {      0,     15,     59,     -8,    -41,     10,   -210,    -34,    -49,     89,    -46,    -29,      4,     18,     37,      2 },
      {      3,     24,    -23,    230,     -3,     23,    -10,     71,    -12,     -5,    -41,    -41,     45,      6,    -17,    -13 },
      {      1,    -18,      1,    -50,     28,     15,      2,    -21,    -41,    -30,    -34,      9,    226,     58,    -57,     -8 },
      {      8,     -2,     53,     33,     -3,     25,    -55,     25,     51,    -48,    203,     45,     45,     49,     69,     -1 },
      {     -4,     -6,    -14,     -4,     -3,     34,     26,    -16,      0,    -40,    -55,    -18,    -62,    223,     55,    -41 },
      {      5,      4,      2,     65,      3,    -41,     24,   -221,     12,    -26,     34,    -84,      8,     -7,      8,     34 },
      {     -1,      0,     13,     -1,     -7,      6,    -50,     -8,      8,    -16,     60,    -18,    -60,     44,   -223,    -57 },
      {      1,      8,     -2,     58,     -2,     -6,      2,    -67,     -7,     43,    -28,    219,    -15,     32,    -34,     65 },
      {     -1,     -1,      1,    -14,      0,      7,     -6,     52,      0,     -8,     10,    -60,    -10,     49,    -40,    234 },
    },
  },
  { // 1
    {
      {   -238,     31,     30,      7,    -73,     27,     19,      1,      9,     10,      0,     -2,     17,      1,     -4,     -1 },
      {     59,    -67,     19,      2,   -212,    -15,     48,      9,    -86,     33,     28,      0,     -1,     21,      2,     -4 },
      {     50,    186,    -25,    -24,    -54,    142,    -36,    -21,    -30,     24,    -24,     -4,     -2,    -10,     -8,      3 },
      {    -34,     35,   -135,      9,    -28,    -97,   -118,     27,    -93,    -89,     -2,     36,    -30,     -9,     30,     14 },
      {      9,    -77,   -102,     -3,    -68,     40,    -95,      3,    142,     45,    -64,      2,     81,     -7,    -26,      3 },
      {    -35,    -81,    -63,     17,     66,     81,    -40,     -2,   -106,    145,     -4,    -20,    -73,     54,      5,    -11 },
      {     -8,    -65,     85,    -74,    -10,     26,    -63,    -87,    -49,    -67,   -143,    -20,    -55,    -57,    -42,     25 },
      {      7,    -47,     20,    132,     -8,    109,      3,    112,     14,    -73,    -10,     56,    -68,    -79,     29,     21 },
      {    -13,    -52,    -87,   -103,     -6,     76,     52,    -79,     31,    -65,    115,    -21,    -53,    -63,     64,      2 },
      {      4,    -17,    105,    -57,     -5,     31,   -118,      9,     22,     -6,     62,     92,     11,     97,    114,     19 },
      {     -4,    -41,     -5,      9,     45,     62,     10,    -10,   -112,    -44,     14,     13,    203,    -34,      7,     17 },
      {     -6,      6,    -26,   -105,     14,    -18,     66,     61,     -8,     65,    -31,    160,     -7,    -52,    -38,    101 },
      {     -3,    -19,    -48,    -32,      6,     58,     64,     33,      3,   -112,    -39,     28,    -13,    177,    -81,    -20 },
      {     -4,    -11,     37,    -87,      5,     11,    -71,    112,    -14,     -3,     83,    -22,     -2,    -56,   -123,   -114 },
      {     -1,      4,    -15,    -78,      4,     -5,     52,     99,     -8,      6,   -112,    -41,     16,    -12,    145,   -105 },
      {      0,      0,      9,    -45,      0,      4,    -15,    103,      1,    -10,     16,   -151,      4,     24,      7,    170 },
    },
    {
      {    191,   -130,     23,     -7,    -96,     26,     28,     -8,      4,     25,    -13,     -3,     -5,     -2,     -3,      1 },
      {   -105,    -68,     86,    -18,    -72,    162,    -62,     -4,     55,    -43,    -37,     24,     -3,     -4,     16,      3 },
      {   -104,    -90,      9,     31,     -6,    -18,    140,    -51,    -52,    116,    -67,    -26,     18,    -10,    -23,     24 },
      {      4,   -114,    129,    -39,    160,    -46,    -31,     -3,    -39,    -35,     38,      2,    -16,     32,      2,    -14 },
      {     69,     52,     -3,     17,     98,     67,    -33,    -73,    -44,     16,   -156,     67,     24,    -51,     38,     41 },
      {     -2,    -72,    -95,     97,     68,     13,     76,    -37,    117,   -116,     -4,     -8,    -33,     -6,     44,     -4 },
      {     33,     87,     99,    -78,     25,      2,     83,    -33,    140,      9,    -34,    -72,    -32,     -5,    -69,     26 },
      {     20,     -6,    -70,     18,     65,    129,    -19,     42,    -58,     21,     22,   -116,    -46,     63,   -101,     53 },
      {    -18,    -72,    -71,    -79,     39,    -41,    -57,    136,     84,     56,    -64,    -11,     28,    -83,     -3,     43 },
      {     24,     19,     30,     65,     57,     80,     20,     26,     41,     84,     86,    -22,    124,    -73,     22,   -105 },
      {      0,     -4,      2,     82,     17,    -18,    -67,     -5,     83,    126,    -18,     82,   -107,     84,    -39,    -60 },
      {      4,     24,    -21,    -93,     23,     64,     99,     80,    -37,     20,     -8,     29,   -106,     20,    115,    -85 },
      {    -13,    -35,   -103,   -140,     14,     17,    -32,   -145,     18,     19,     32,     13,     29,     10,    -42,    -83 },
      {     -9,     -5,     15,     34,    -21,    -33,    -68,    -43,    -32,     -1,    -35,   -134,   -108,   -127,     23,    -86 },
      {      1,     -1,      3,     13,     -8,    -16,     25,     68,    -27,    -83,   -115,     10,     52,     34,   -118,   -145 },
      {      3,      3,     -1,     -1,     -9,    -26,    -44,    -12,     28,     23,    -69,   -121,     77,    144,    122,    -20 },
    },
    {
      {    234,    -76,    -10,     -6,    -52,    -32,     21,      1,    -12,     20,      9,     -5,     -6,      1,     -4,     -3 },
      {     75,    123,    -49,     -2,    172,    -71,    -57,      6,    -18,    -59,     22,      9,     -8,      1,     12,     -2 },
      {     63,    118,     -1,    -20,    -44,    174,    -67,    -24,     41,      5,    -83,     13,      4,    -31,    -10,     17 },
      {     12,   -135,     78,     -5,    147,     69,      1,    -25,     95,    -39,    -54,     -1,     -6,    -26,      8,      9 },
      {    -16,    -70,    -49,     -3,    -70,    -21,   -143,     29,      5,   -138,     -3,     78,    -15,    -14,     72,     15 },
      {     11,     19,    164,    -37,     -3,     27,      6,    -47,   -156,    -67,     10,     11,    -49,     36,     35,      2 },
      {    -18,    -73,   -119,     13,     52,     41,    -30,    -24,   -151,     62,    -94,     -9,    -37,      5,    -24,     44 },
      {     -9,     24,     31,    -48,    -37,   -130,     12,    -83,     17,    -36,   -148,    -21,     28,    -79,    -50,     54 },
      {      6,     24,    -48,    109,    -12,     37,    146,     26,    -15,   -112,    -18,    -24,    -56,    -85,     50,     26 },
      {     13,     14,     77,    139,     12,    -28,    -15,     88,    -24,     60,    -66,    104,     91,      7,     26,     51 },
      {      2,     -6,    -54,    -66,     14,     30,     73,    -90,    -19,    -14,     28,     55,    163,     18,    104,     38 },
      {     -6,     17,      1,    -99,     11,    -20,     46,     36,     29,     86,      5,    122,   -124,    -54,     72,     68 },
      {      2,     18,    -10,     85,    -16,    -33,     -8,   -118,     64,     30,    -46,    -16,    -79,    136,    101,      5 },
      {     -3,     -7,     20,     67,      3,    -10,    -62,   -101,    -23,     74,     57,     -6,      0,   -163,     63,    -81 },
      {     -1,      5,     -1,    -62,      4,    -14,     13,     96,    -14,     18,   -109,    -62,     19,      4,    122,   -143 },
      {      1,      1,     26,    -12,     -2,     -2,    -53,     51,      2,     30,     38,   -158,     18,    -24,     86,    153 },
    },
  },
  { // 2
    {
      {    221,    -86,      9,     -7,    -88,     -6,     22,     -2,      3,     27,     -4,     -2,     -3,     -5,     -5,      0 },
      {     88,    103,    -56,      7,    123,   -148,     11,      8,    -61,      5,     50,     -9,      2,     16,    -10,     -7 },
      {    -20,    136,   -105,     18,   -166,    -10,     22,     14,      0,     72,     -3,    -10,     25,    -18,    -23,      6 },
      {    -75,   -110,     15,      7,    -11,    -76,    111,    -18,   -103,    120,      0,    -34,     20,     18,    -42,     12 },
      {     10,    -20,    -89,     49,     80,      6,    111,    -25,    141,     20,    -88,    -26,     13,    -75,      0,     32 },
      {    -44,    -71,    -51,     19,    -70,   -106,     44,     31,     81,   -111,    111,     -9,    -44,     33,     37,    -31 },
      {     -6,     56,    154,    -81,    -13,    -74,     13,      2,    130,     59,     30,    -20,     31,    -22,    -55,    -11 },
      {     16,     -1,    -42,     15,     50,    113,     11,     45,     43,     62,    108,    -79,    -35,    100,    -97,    -40 },
      {     11,     73,     55,     -5,    -20,     18,    110,    -94,    -17,    -42,    -51,     -8,   -154,     81,      6,      6 },
      {     18,     40,     51,     68,     -1,     61,     96,    -76,    -20,    -42,     83,      1,    151,     11,     64,    -40 },
      {     -4,    -10,     21,    152,      2,    -39,    -71,    -52,     58,     98,     -5,    108,    -28,     69,     28,    -42 },
      {      2,     18,     15,    -37,     22,     38,     52,     91,     -8,     93,     54,     28,    -76,    -66,    162,    -42 },
      {     -3,    -15,    -73,   -147,     13,      0,     21,    -21,     35,     24,    -42,     90,     68,    134,     44,    -30 },
      {     -9,    -21,    -56,    -66,      6,     12,    -34,   -164,     -7,     22,     74,     26,    -45,   -108,    -38,    -86 },
      {      2,      8,     13,     14,      6,     22,     82,     85,    -22,    -43,     -1,    168,     -9,    -45,   -121,    -58 },
      {     -1,      1,      7,     16,     -7,    -23,     -9,     33,    -12,    -18,   -104,    -86,     14,      4,      5,   -211 },
    },
    {
      {    237,    -45,     -3,     -5,    -73,    -30,     14,     -1,     -5,     26,      8,     -1,     -3,     -3,     -8,     -2 },
      {    -74,    -86,     18,     -1,   -199,     76,     31,      1,     44,     56,    -24,     -4,      7,    -12,    -12,      2 },
      {    -49,     12,    -18,      4,    -59,   -138,     40,      7,   -158,     74,     66,     -8,     18,     54,    -22,    -10 },
      {    -23,   -216,     22,     11,     87,    -57,     67,      5,     24,    -22,     28,    -13,    -16,      6,     13,     -8 },
      {      2,     39,      5,      3,     66,    -24,     74,     -8,    100,    164,    -13,    -39,     74,    -35,    -91,     -1 },
      {    -22,     41,    -26,      1,    -68,   -157,    -14,     22,    149,    -76,     56,     15,     12,    -27,     28,     -6 },
      {    -12,    -56,   -192,      5,      6,    -29,    -79,     24,    -25,     24,    -88,     17,     17,    -79,    -24,     39 },
      {     -7,     19,     99,     -6,    -17,    -76,     54,    -60,    -48,    -45,   -134,    -17,    -29,   -124,    -12,     63 },
      {     -4,     37,    -70,      4,     -1,     15,     87,     23,     17,     20,     27,    -43,   -216,    -21,    -21,     -5 },
      {     11,     33,    -75,     51,     -4,     52,    164,     29,    -28,    -57,     26,    -10,     97,    -50,     92,     18 },
      {      3,     10,      0,     67,      8,    -37,     10,     -7,     37,     83,   -102,     46,    -34,    111,    146,     61 },
      {      1,      5,    -70,    -84,    -13,     -9,     52,    -99,     22,    -74,    -75,    -90,     27,    133,    -50,     -8 },
      {     -6,     -4,    -19,   -184,     13,      6,      6,    -90,      1,     59,     58,     49,     -7,    -54,    103,     22 },
      {      2,      6,     16,   -107,      6,     -2,     63,    150,      1,    -26,    -48,    124,      3,     48,    -64,     50 },
      {      1,     -2,     30,    -65,      0,     -4,    -46,    125,     -4,     16,      0,   -182,     13,     -3,     62,     73 },
      {     -1,      6,      5,    -40,      1,    -20,     10,     57,    -11,     16,    -89,    -13,      1,    -25,     60,   -218 },
    },
    {
      {    209,    -76,      6,     -5,   -117,      9,     19,     -2,     19,     31,     -9,     -1,     -4,    -12,     -6,      1 },
      {    -97,    -77,     35,     -2,   -107,    154,    -25,     -1,     91,    -47,    -38,     11,    -16,    -12,     24,      1 },
      {     82,    -48,     47,     -7,    132,     30,    -66,      2,     38,   -149,     39,     13,    -53,     39,     40,    -18 },
      {     35,    186,    -55,     -7,    -65,    -14,    -52,     27,    121,    -55,     24,      5,    -15,    -21,      7,     -9 },
      {    -38,    -13,     50,    -25,   -105,    -44,    -58,     33,    -62,    -19,    146,    -12,    -32,    116,    -44,    -37 },
      {    -43,    -83,    -30,     10,      1,   -125,    117,      3,    143,    -19,     37,    -33,    -58,      6,     -6,     -5 },
      {     -7,    -21,   -165,     58,    -52,      3,     43,     -5,    -95,   -103,     -4,     20,    -11,     26,     95,     -8 },
      {    -23,    -25,     87,    -18,    -49,   -121,    -50,    -16,     -8,    -43,    -11,     74,     87,    -90,    116,    -15 },
      {      3,    -55,    -74,     29,     32,     36,    -33,     51,     38,     -3,    116,    -40,    169,    -59,    -43,    -11 },
      {     17,     63,     74,     65,      1,     50,    123,   -108,     24,     -9,     50,     32,     89,     81,     35,    -50 },
      {      4,     -4,    -27,    -47,     37,     38,      3,     89,     31,    129,     44,     38,    -28,     34,    148,    -84 },
      {     -7,    -35,    -47,    137,     10,    -30,   -110,    -77,     43,     67,    -10,    108,    -31,     26,    -43,    -48 },
      {      0,    -22,    -52,   -113,      5,    -31,    -16,     12,     41,    -20,   -102,     60,    102,    142,    -39,     -6 },
      {     -5,     -4,    -40,   -120,      6,     27,     43,    -70,    -27,    -23,     44,     91,    -38,    -96,    -76,   -119 },
      {     -5,    -13,    -50,    -83,      1,     -2,    -68,   -174,     20,     41,     41,    -98,     -2,     14,     69,     52 },
      {      1,      0,    -11,    -28,      9,     22,     30,      5,      8,     17,     85,    144,    -15,      0,      2,    186 },
    },
  },
  { // 3
    {
      {    168,    -71,      5,     -6,   -155,     50,      6,      2,     68,      7,    -13,      0,    -20,    -19,      7,      1 },
      {    138,     26,    -28,      1,     40,   -123,     36,     -5,   -127,     88,      6,     -3,     51,     -7,    -20,      3 },
      {     90,   -100,     40,    -10,    125,     36,    -42,      3,    -44,   -116,     38,      2,    -54,     84,      2,     -8 },
      {    -72,   -138,     63,     -4,    -50,     79,     -2,     -8,   -128,     75,    -14,      3,     76,     15,    -21,      7 },
      {     18,    -89,     17,      3,    128,     12,     31,    -17,     79,     56,    -86,      6,     14,   -145,     33,     14 },
      {    -57,   -107,    -33,     17,    -26,   -111,    121,    -15,     19,     23,    -15,    -15,   -125,     67,      4,      2 },
      {    -12,    -16,    155,    -58,     -7,    -98,    -95,     35,     59,     68,     79,    -21,    -33,    -16,    -44,      0 },
      {     21,     79,     39,    -18,     23,     86,    -27,     -6,    -45,    116,    -84,      3,   -146,     47,     47,      3 },
      {    -21,     -7,     31,    -12,    -60,    -65,    -42,      9,   -101,    -90,    -24,     31,    -55,   -112,    137,     -7 },
      {     11,     31,    119,    -17,     -7,    -50,     46,    -33,     42,    -39,   -124,     50,     93,    102,     55,    -28 },
      {     -9,    -49,   -100,    -57,     13,    -22,    -80,    113,     37,     58,     -9,    -27,     52,     70,    123,    -44 },
      {      3,    -16,      4,    182,      2,    -16,    -74,   -103,     28,     51,     59,     37,     10,     28,     74,    -33 },
      {      9,     30,     56,    -42,     18,     57,    134,    -10,      7,     24,    142,    -43,     20,     -7,    120,    -18 },
      {     -8,    -14,    -51,   -118,      3,      3,     -9,    -88,      9,     25,     45,    179,    -14,     -5,    -16,    -75 },
      {     -4,     -9,    -37,    -90,     -7,    -18,    -54,   -160,      9,     -5,     -7,    -90,     18,     24,     38,    128 },
      {     -2,     -1,     -5,    -24,     -5,      0,    -14,    -75,    -13,    -13,    -32,   -131,     -6,    -24,    -31,   -197 },
    },
    {
      {    237,    -31,    -22,     -6,     78,    -29,    -13,     -1,      5,    -17,      0,      1,    -16,     -8,      2,      0 },
      {    -71,     62,    -12,      3,    215,      9,    -35,     -4,     78,    -45,    -19,      1,      5,    -20,      1,      2 },
      {    -55,   -128,     20,     14,     50,   -157,     32,     11,    -80,    -78,     44,      4,    -47,      8,     25,     -2 },
      {     -6,   -134,     25,      8,    -27,    -30,     23,      6,    187,     18,    -18,     -5,     89,    -28,    -19,     -2 },
      {      2,    -32,     83,     -9,     86,     31,     94,    -25,    -38,    145,     41,    -31,     31,    114,    -18,    -17 },
      {    -19,    -32,   -146,     21,     12,    -56,   -117,     24,    -40,     77,    -41,     12,     84,     87,    -30,     -2 },
      {    -23,   -127,    -37,      9,     21,    113,    -49,      3,      7,     53,    -42,     -2,   -164,    -16,      9,      6 },
      {      1,    -23,     95,    -16,     14,    -12,    -23,    -36,    -76,     -7,   -167,     -6,     32,    -56,   -124,     22 },
      {     -8,     19,    -79,     55,     24,    -35,     89,     38,    -31,    109,     29,      6,      2,   -171,    -39,     15 },
      {     -7,     61,    -14,   -110,    -25,   -128,    -10,    -86,     69,     77,    -24,    -44,   -107,      9,    -18,    -10 },
      {    -14,    -60,    -68,   -142,     16,     62,      6,   -118,    -52,    -32,     51,    -44,     79,    -62,     23,      1 },
      {     -1,      2,   -102,     31,     -9,     27,    113,    -10,     20,    -89,    -17,    -74,    -32,     68,   -134,    -50 },
      {      5,      6,     16,    130,     -7,    -11,    -40,    -83,    -11,      9,    -26,   -164,     16,    -29,     81,    -65 },
      {      0,     -5,     58,     -9,     -3,      6,   -119,     40,      1,      5,    143,    -50,    -12,    -35,   -132,    -65 },
      {      2,      1,     -1,    101,     -8,     -1,    -21,   -153,     16,     -6,     67,     64,    -12,     19,    -58,    136 },
      {     -1,     -3,     -1,     32,      0,      1,      5,    -89,     -2,      6,     -8,    146,     -2,    -14,     -2,   -187 },
    },
    {
      {    196,    -70,     -2,     -6,   -136,     30,     11,      1,     41,     19,    -12,      0,    -12,    -18,      2,      1 },
      {   -148,     -5,     19,      2,   -154,     90,      3,      2,    100,     18,    -23,      0,      6,    -25,      3,      2 },
      {     35,      1,     -1,     -1,    109,     63,    -43,      1,    170,   -108,    -20,      5,    -57,    -42,     27,     -1 },
      {     45,    223,    -65,     -7,    -56,     42,    -52,      6,    -25,    -29,      2,     11,     -5,      2,     12,     -2 },
      {     25,     50,    -22,      3,     57,    -27,     27,    -11,     86,    143,    -53,    -11,    125,    -93,    -45,     11 },
      {     20,    -24,     42,    -10,     63,    205,    -46,    -17,    -77,     61,    -45,      1,     24,     40,    -22,      4 },
      {    -26,    -69,   -172,     38,      0,     24,    -40,     24,    -71,    -13,    -38,     14,    -19,   -131,     37,     19 },
      {     -5,    -23,   -146,     37,     16,      9,    -10,     26,     73,     81,     44,    -24,    -32,    157,    -23,    -26 },
      {      4,    -39,    -30,      8,    -19,      4,    -57,     20,      7,   -116,     27,     19,    204,     48,    -19,    -14 },
      {      7,     19,    -44,     21,     18,     81,    209,    -30,    -10,    -52,     79,    -36,     27,    -21,      6,     -1 },
      {      1,     -8,     29,    -25,      9,     25,    -73,     30,      8,     62,    212,    -14,      9,    -71,     50,    -21 },
      {     -6,     -7,    -19,    -51,     -8,      6,    -19,      3,     -6,    -36,     45,    -12,    -43,    -41,   -234,     21 },
      {     11,     17,     57,    237,     -5,      3,    -21,     34,     -7,     -6,     13,    -36,    -10,    -18,    -45,     -4 },
      {      4,      9,     21,    -22,     13,     15,     56,    228,     -3,      8,    -18,     89,     -4,     -2,     -9,    -28 },
      {     -1,      1,      5,    -45,     -4,     -3,    -13,     79,    -14,    -19,    -42,   -228,      8,     -9,     15,    -48 },
      {      2,      2,      4,     -4,      2,      0,     -3,     47,      9,      0,     17,    -37,      9,     28,     21,    245 },
    },
  },
  { // 4
    {
      {    246,    -37,     -5,     -7,    -56,    -12,      4,      0,     13,     12,      0,      0,     -6,    -11,      0,      0 },
      {     61,     42,    -13,      1,    232,    -40,    -19,     -2,    -19,    -58,     10,      0,    -14,      7,      8,     -1 },
      {      2,     18,      2,     -2,     35,     94,    -18,      0,    218,      1,    -40,      0,     27,    -69,     -2,      2 },
      {    -31,   -153,     13,      7,    -11,    -82,     12,      9,     53,   -129,     22,      3,    -98,    -50,     43,     -1 },
      {    -14,   -133,     22,     10,     57,    -83,     55,     -2,     43,    108,     13,    -14,    127,     47,    -43,     -7 },
      {     -3,    113,    -54,      3,    -58,   -162,    -12,     14,     55,    -65,     34,     10,    101,    -30,      6,     -5 },
      {     12,     52,    203,    -22,     -4,      7,    117,    -38,     -5,    -61,     16,    -16,     34,    -17,     -4,     -2 },
      {     10,    -36,     18,     -4,    -20,     62,    -79,      9,     16,   -121,     -5,     18,     89,    171,     27,    -14 },
      {      0,    -56,     -9,      0,      5,     35,    -40,     -6,    -92,    -66,   -103,     18,    119,   -136,    -42,     32 },
      {      3,    -18,   -106,      9,      7,     97,    122,      9,    -22,    -45,    141,    -20,     60,    -34,     39,    -20 },
      {     -3,    -18,     77,    -46,      7,      6,   -131,      5,    -17,     61,    125,     37,     38,    -68,    113,     -5 },
      {     -9,    -11,    -44,   -216,     -2,     -8,     -4,   -117,      8,    -12,      7,    -28,     -7,      9,    -41,     -1 },
      {     -1,      4,    -29,      5,      4,    -17,     55,    -76,     -4,     26,   -108,    -19,     23,     16,    204,     24 },
      {     -1,      3,     -4,    -99,     10,      0,     69,    131,      2,     10,    -34,    173,     -3,     14,     24,     41 },
      {      1,     -2,    -10,     73,     -2,      2,      1,   -149,     13,     -2,     48,    137,     -9,     14,    -34,    123 },
      {      0,     -2,     -5,     25,      1,     -1,      8,    -70,     -6,      5,    -23,    112,     -3,    -15,     -6,   -215 },
    },
    {
      {   -187,     58,     -4,      5,    147,    -26,     -6,     -2,    -60,    -13,      9,     -1,     19,     17,     -4,      1 },
      {    116,     51,    -30,      6,     49,   -129,     31,     -5,   -132,     87,      8,     -4,     57,     -2,    -19,      2 },
      {     67,   -149,     64,     -8,    105,     68,    -35,     -3,    -95,    -66,     21,      2,     -7,     63,     -4,     -2 },
      {     76,    128,    -12,    -12,     39,    -39,    -48,     16,     49,   -131,     70,     -3,    -66,     91,    -11,    -11 },
      {     51,    -39,    -33,     15,    147,    -39,     63,    -14,    110,     16,    -59,     -4,    -92,    -84,     37,      6 },
      {    -23,    -39,   -152,     58,    -46,      6,    122,    -23,    -59,    -57,    -35,     -9,    -54,     91,     15,      4 },
      {     53,     63,    -71,     32,     48,    120,     13,    -28,     11,    -70,    -34,     23,    156,    -62,     13,      2 },
      {    -12,    -72,    -54,     17,     26,    -23,     32,      7,     96,     28,    124,    -36,     76,     30,   -146,     11 },
      {      9,     25,     85,    -42,     21,      1,     77,    -22,     69,     42,   -115,     35,     63,    154,    -19,     -6 },
      {    -24,    -70,     37,    -27,    -36,   -149,     33,      1,     19,   -141,    -14,     23,     90,    -42,     50,    -10 },
      {     -3,      1,     58,    178,    -10,    -27,    -30,   -151,     15,      7,     27,     69,     -9,     11,      0,    -17 },
      {     -1,    -44,    -77,     49,     17,    -29,   -110,     62,     52,     62,      2,    -17,     58,     82,    137,    -48 },
      {      1,     18,     39,    -48,      7,     42,    124,    -27,      4,     34,    152,     23,      7,      3,    123,    -60 },
      {     -7,    -25,    -83,    -98,      3,     -8,    -49,    -42,     -2,     15,     -3,    181,    -21,     -7,    -44,    -93 },
      {     -3,    -12,    -45,   -106,     -2,    -12,    -53,   -163,     12,     12,     14,    -55,     10,     22,     47,    127 },
      {     -1,      3,     -2,    -30,     -4,     -1,    -15,    -86,     -6,    -12,    -37,   -143,      1,    -13,    -29,   -185 },
    },
    {
      {    202,    -65,      4,     -5,   -135,     22,      9,      0,     37,     17,     -8,      1,     -7,    -13,      1,     -1 },
      {    112,     92,    -49,      4,     72,   -139,     27,      0,   -108,     49,     27,     -8,     33,     19,    -24,      1 },
      {    -41,    165,    -78,      5,   -115,    -49,     20,     13,    117,      4,      1,     -4,    -13,    -24,      1,     -1 },
      {     63,    -20,    -23,      8,    135,    -40,     33,     -7,    134,    -66,     -6,     -6,   -129,      2,     22,     -3 },
      {     52,     96,     55,    -36,      0,     47,   -127,     28,    -22,   -138,     72,     15,     -3,     66,     13,    -21 },
      {      8,    -15,   -173,     68,     -3,     90,     73,    -21,    -53,   -106,      3,     -4,     31,     41,      8,      0 },
      {    -55,    -64,      1,    -11,    -90,    -90,     33,     18,    -47,    -17,     63,    -19,   -122,    134,    -31,    -10 },
      {     -8,    -49,     51,      2,      2,    -84,     57,    -24,     92,    -80,     31,      3,    169,     59,    -43,     -7 },
      {     -5,    -53,    -78,     11,     25,     17,    -75,     66,     45,     53,    162,    -65,     19,    -54,    -84,     -2 },
      {    -24,    -59,    -21,      1,    -40,   -110,    -32,     21,    -50,   -104,     19,     12,     -1,   -149,    100,    -11 },
      {     -1,    -11,    -44,   -195,     16,     17,     55,    133,     -4,    -14,    -47,    -29,     24,      8,      7,     -6 },
      {     -3,    -41,    -71,     30,     12,    -31,    -98,     45,     42,     75,    -32,     53,     45,    100,    130,    -72 },
      {     -7,    -33,    -73,    -50,     -4,    -39,    -95,    -49,      2,    -26,    -81,    124,    -14,    -13,   -144,      3 },
      {     -3,      8,     -5,    -87,      6,     34,     72,    -89,      1,     37,    124,    130,     -9,    -24,     41,    -88 },
      {      4,     13,     47,     99,      4,     12,     58,    152,    -12,    -13,    -14,     91,    -13,    -29,    -64,   -113 },
      {     -2,      3,     -4,    -19,     -5,     -3,    -20,    -73,     -9,    -11,    -49,   -127,     -3,    -17,    -35,   -198 },
    },
  },
  { // 5
    {
      {   -220,     47,     10,      5,    118,    -15,     -9,     -2,    -21,     -7,      3,      0,      4,      6,      0,      0 },
      {    114,      8,    -10,      0,    176,    -55,     -6,     -3,   -133,      1,     14,      0,     10,     16,     -3,      0 },
      {     45,    -10,      3,     -1,    121,     15,    -17,      0,    181,    -52,    -12,     -1,   -111,    -12,     15,     -1 },
      {     23,      9,     -7,      1,     55,      5,      0,     -1,    102,     33,    -14,      1,    221,    -32,    -21,      0 },
      {     28,    223,    -45,     -6,    -40,    -97,      4,      8,     31,    -24,     13,      0,     -9,     13,      0,     -1 },
      {     18,     74,     -4,     -3,      6,    187,    -57,     -1,    -46,   -134,      8,     10,     31,    -10,     18,     -1 },
      {    -12,    -73,     11,     -2,    -24,    -99,      7,      6,     14,   -182,     51,      6,     51,    105,     -5,     -8 },
      {     17,     34,    225,    -35,     -9,    -24,    -98,      0,      3,     22,    -20,     12,      6,     27,     12,     -2 },
      {      7,     23,    -17,      5,     15,     76,     20,      1,     26,     80,     13,    -11,     -2,    221,    -41,    -11 },
      {     -7,    -27,    -79,      9,    -11,    -21,   -199,     39,     12,     47,    116,     -1,      0,      0,     20,    -17 },
      {      6,     16,     68,     15,      8,     30,     82,     -3,     12,     11,    205,    -37,    -11,    -50,    -79,     -4 },
      {      7,      8,     23,    236,     -7,     -9,    -22,    -89,      1,     -3,    -10,    -17,      2,      6,     18,      8 },
      {     -2,     -5,    -17,     -6,     -7,    -10,    -64,    -46,     -5,    -24,    -55,     21,    -21,    -19,   -231,     29 },
      {     -2,     -2,    -20,    -78,     -3,     -2,     -6,   -219,      7,     14,     45,     77,      2,      8,     46,     25 },
      {     -1,     -2,     -7,    -44,     -4,     -5,    -25,    -55,     -2,    -10,    -16,   -229,      5,      6,     13,     82 },
      {      1,      0,      3,     17,      2,      2,      6,     53,      4,      6,     22,     68,      2,     13,     17,    238 },
    },
    {
      {    191,    -63,     -4,     -4,   -144,     38,      9,      1,     50,     -2,     -8,      1,    -14,     -4,      3,      0 },
      {   -125,    -16,     21,     -1,    -74,     66,    -14,      2,    171,    -50,    -11,      2,    -83,      5,     15,     -3 },
      {     12,    174,    -84,      3,    -64,   -126,     57,      5,     61,     28,     -7,     -8,    -16,     -2,     -4,      2 },
      {   -100,    -12,      1,      4,   -152,     27,     19,     -3,    -56,     60,    -11,      1,    151,    -38,    -12,      3 },
      {     -6,    -94,     28,     -3,     47,    -75,     40,     -5,     70,    168,    -72,      0,    -23,    -86,     18,      7 },
      {     47,     30,     -2,      4,     88,     30,    -22,     -3,    140,    -36,      2,      2,    176,    -34,    -13,      0 },
      {    -15,    -60,   -170,     71,     34,     61,    132,    -48,     -7,    -34,    -43,     11,     -1,      5,      6,      1 },
      {     20,    103,    -26,      3,     14,    135,    -47,      7,    -35,     39,    -30,     -1,    -49,   -157,     48,      3 },
      {    -12,    -47,   -102,     -9,      4,      4,    -48,     68,     22,     54,    168,    -69,    -18,    -38,    -86,     14 },
      {     -6,    -11,    -32,   -192,     13,     21,     65,    129,     -8,    -28,    -58,    -33,     13,      8,     20,      3 },
      {     12,     56,     -5,      1,     20,    108,    -13,      4,     18,    140,    -39,     -9,     -2,    159,    -56,      1 },
      {      8,     34,    103,    -28,     10,     39,    140,    -57,     -2,    -17,     39,    -23,    -24,    -51,   -143,     52 },
      {      3,      3,     35,    102,     -1,     -4,      9,     71,     -7,    -25,    -54,   -189,      7,     11,     32,     91 },
      {      6,     19,     56,     13,     11,     35,    109,     12,     11,     42,    133,    -20,     13,     27,    153,    -65 },
      {      3,      6,     36,    105,      3,      2,     37,    164,     -5,    -14,    -27,     66,     -7,    -21,    -73,   -118 },
      {      1,      2,      6,     31,      3,      6,     18,     81,      4,     10,     36,    137,      2,     13,     36,    190 },
    },
    {
      {   -230,     84,     -9,      6,    -64,     28,     -1,      0,     15,      0,     -1,      0,     15,     -2,     -1,      0 },
      {     54,    -32,      9,     -3,   -227,     51,     10,      1,    -72,     36,      1,      1,     27,      1,     -3,      0 },
      {     78,    187,   -119,     24,      5,     74,    -46,      6,    -28,    -24,     12,     -3,    -12,    -16,      9,     -1 },
      {    -39,     -1,     -2,      5,     43,    -49,     13,     -2,   -221,     22,     23,     -3,    -92,     32,      9,     -1 },
      {    -13,    -44,     44,    -16,     39,    176,    -70,      7,     13,     95,    -54,      4,   -107,    -17,     10,     -2 },
      {     -4,      7,     10,     -2,     70,     38,    -15,     -4,    -71,    107,    -34,      1,    200,      2,    -30,      3 },
      {     24,     89,    170,    -98,     13,     47,     60,    -43,    -33,    -84,      4,      6,     12,    -48,      0,      9 },
      {     28,     92,     27,    -24,    -15,    -58,     89,    -33,     55,    141,    -47,     -3,    -54,    109,    -48,      1 },
      {     -7,    -34,    -51,     43,     27,     80,    160,    -69,     11,     34,    120,    -60,     -3,    -54,      1,      1 },
      {      3,     21,    -12,     17,    -12,    -78,     29,     -2,     -4,     69,    -89,     24,    -26,   -206,     26,     14 },
      {     -4,    -25,    -26,     88,      9,     50,     79,    -15,    -26,    -92,   -144,     33,     -7,     21,   -123,     34 },
      {     16,     42,    116,    183,      1,      0,     11,    108,      5,     13,     26,    -33,      2,      3,     31,    -42 },
      {     -2,     -3,    -27,    -70,     11,     22,     70,    179,      2,     15,     55,    124,     -6,    -16,    -59,     -7 },
      {      2,      9,     31,      6,     -8,    -29,    -86,     -6,      4,     20,     83,    -66,    -24,    -57,   -197,     44 },
      {      4,      7,     30,     75,     -1,     -2,    -32,    -87,      9,     21,     68,    177,     -1,     11,     19,    115 },
      {      0,      1,     -3,    -19,      1,      4,     20,     79,      1,      0,    -19,    -90,      4,     17,     52,    217 },
    },
  },
  { // 6
    {
      {   -249,     36,     22,      6,    -17,      6,      5,      0,     34,     -5,     -3,     -1,     16,     -3,     -1,      0 },
      {     19,     -7,      2,     -1,   -248,     35,     21,      5,     -7,      4,      3,      0,     42,     -6,     -3,     -1 },
      {     35,     -3,     -2,     -2,     -2,      7,     -5,      0,    250,    -30,    -21,     -3,     12,     -2,     -3,      0 },
      {     22,    214,    -56,    -10,    -14,     66,    -18,     -6,     -2,    -30,      6,      2,   -100,    -14,     14,      2 },
      {     21,     85,    -28,     -6,     43,     39,    -13,     -3,    -15,      0,     -2,     -1,    228,    -28,    -16,      0 },
      {     -6,    -66,     36,     -1,     27,    225,    -49,    -11,     -1,     59,    -16,     -5,    -22,    -38,      7,      2 },
      {     29,     60,    230,    -42,      8,      2,     70,    -15,     -6,    -28,    -18,      3,      2,     -1,    -18,      3 },
      {     -7,    -50,     -1,     -2,      2,     33,    -38,      7,    -27,   -235,     39,     14,      5,    -49,     16,      5 },
      {     -3,    -16,    -57,     35,     26,     53,    222,    -31,      8,    -13,     75,    -15,     -5,    -21,    -24,      3 },
      {    -13,    -21,    -40,   -240,      0,     -1,     10,    -63,      4,      8,     29,     17,     -1,      8,      5,     16 },
      {      0,     -1,      3,     -6,     -4,    -55,      6,      0,      4,     40,    -21,      8,    -16,   -242,     34,     14 },
      {     -4,    -12,    -45,     -9,      2,     11,     52,    -29,    -24,    -44,   -228,     27,     -8,      4,    -64,     11 },
      {     -2,      0,     -9,    -57,     11,     19,     37,    236,      1,      6,     -5,     62,     -4,     -3,    -22,    -21 },
      {     -1,      1,      0,     -2,     -4,     -9,    -47,     -3,      2,      4,     52,    -38,    -21,    -35,   -238,     22 },
      {     -1,     -1,     -5,    -37,      1,     -1,      9,     50,     -8,    -17,    -36,   -239,     -1,     -6,     25,    -46 },
      {      0,      1,      1,      4,      0,      1,      3,     35,      0,      1,     -3,    -39,      3,     17,     25,    249 },
    },
    {
      {   -241,     53,     23,      4,    -53,     15,      8,     -1,     28,     -5,     -3,     -1,     19,     -4,     -2,      0 },
      {     50,    -12,      2,     -2,   -237,     47,     21,      3,    -49,     12,      6,      0,     35,     -8,     -3,      0 },
      {     36,    -24,      6,     -1,    -35,      6,     -1,      1,    241,    -35,    -24,     -1,     47,     -8,     -8,      0 },
      {     44,    213,    -79,    -14,     12,     96,    -34,     -9,     12,    -16,      5,     -1,      4,    -23,      6,      1 },
      {    -10,    -71,     60,     -8,     26,    199,    -49,    -14,     15,     80,    -31,     -4,    -78,    -17,      9,      0 },
      {      0,     23,    -17,      4,    -55,    -50,     16,      6,     39,    -40,     15,      3,   -233,     28,     23,     -1 },
      {    -37,    -82,   -206,     61,    -12,      7,    -97,     30,      4,     19,      6,      0,      3,     -1,     17,     -4 },
      {      5,     43,    -33,      4,     -5,    -47,     66,    -11,     36,    216,    -30,    -22,     -4,     78,    -24,     -7 },
      {      8,     34,     66,    -47,    -30,    -72,   -191,     42,     -8,     26,   -103,     27,      1,     35,     10,     -4 },
      {     17,     36,     62,    226,      3,      6,      2,     90,     -3,      0,    -25,    -10,      0,     -7,     -5,    -17 },
      {      1,     -4,     11,      4,      5,     53,    -22,     15,      0,    -54,     73,    -11,     25,    228,    -22,    -21 },
      {      4,     12,     49,     -9,     -6,    -28,    -68,     38,     27,     68,    199,    -34,      9,    -48,     86,    -14 },
      {      4,      4,     17,     75,    -13,    -25,    -57,   -209,     -3,    -11,     -6,   -105,      2,     10,     23,     13 },
      {     -1,      0,      4,    -14,     -4,     -9,    -45,     24,      1,      2,     52,    -84,    -26,    -41,   -224,     13 },
      {      2,      5,     11,     49,     -4,     -7,    -28,    -82,      9,     26,     62,    205,     -4,      0,    -63,     71 },
      {      0,      1,      1,     -2,      1,      4,      6,     44,      0,     -1,     -7,    -53,      4,     21,     32,    243 },
    },
    {
      {   -186,    143,    -43,      6,     79,    -47,     -3,      8,      1,     -9,     10,     -3,      5,     -1,      0,      0 },
      {   -124,    -37,     88,    -30,   -104,    130,    -59,      4,     80,    -47,     -6,     13,     -2,     -4,      7,     -3 },
      {    -54,   -114,    136,    -40,    148,    -24,    -26,     -3,    -71,     33,     -9,      7,     -3,     10,     -4,      0 },
      {    -94,    -96,    -32,     69,    -64,    -35,    143,    -72,     -6,     78,    -62,      4,     22,    -16,     -3,     10 },
      {    -11,     51,      9,    -23,    -81,     49,    -48,     19,   -185,     80,     -3,      1,     91,    -40,     -1,      3 },
      {     19,     73,    117,   -118,    -80,   -129,     41,     16,     51,     52,    -41,     12,     -6,      1,     -8,      3 },
      {     45,     45,     14,      2,     83,     62,      0,    -46,     89,     39,    -96,     43,    134,    -94,     33,      2 },
      {     28,     82,     55,     -4,     12,     73,     76,   -128,    -77,    -71,    -56,     63,    -86,     45,      2,      1 },
      {      1,    -12,     42,      0,    -15,    -47,     57,    -43,    -17,   -145,     93,    -13,    157,     11,    -36,      7 },
      {    -16,    -42,   -102,   -185,     30,     60,     91,     61,    -16,    -31,    -24,     14,     11,      5,     -8,      1 },
      {     14,     40,     23,     -6,     29,     84,     40,    -24,     44,    105,     59,    -87,     24,     98,   -130,     47 },
      {      6,     19,     42,    -13,     13,     36,     75,    -10,     -4,      8,    102,   -100,    -67,   -169,     49,     15 },
      {    -10,    -28,    -62,    -82,    -10,    -31,    -80,   -161,     22,     54,    102,     77,    -13,    -27,    -11,     -1 },
      {     -4,    -16,    -24,    -12,     -8,    -34,    -62,    -29,    -16,    -63,    -95,    -54,    -30,    -84,   -133,    129 },
      {     -2,     -6,    -23,    -47,     -5,    -14,    -45,    -87,     -4,    -13,    -60,   -172,     24,     69,    119,     -5 },
      {      1,      2,      5,      7,      1,      4,     12,     25,      3,     14,     38,     54,      9,     44,    108,    215 },
    },
  },
  { // 7
    {
      {    179,   -138,     43,     -8,    -94,     58,     -3,     -5,     13,      3,     -9,      3,     -5,     -2,      2,      0 },
      {    131,     30,    -63,     18,     78,   -134,     67,    -10,   -101,     64,     -5,     -8,     19,      0,    -10,      4 },
      {    -46,   -123,    127,    -36,    144,     -1,    -46,      7,    -83,     30,      3,      3,      2,      4,     -2,     -1 },
      {    -94,    -39,     14,     10,   -124,     -8,     64,    -27,    -83,    124,    -69,     10,     77,    -39,     -2,      7 },
      {    -28,    -66,     27,      2,     27,   -101,     92,    -30,    161,     42,    -70,     12,    -79,      7,     14,      0 },
      {    -35,    -98,   -126,    122,     65,     92,     66,    -70,    -24,    -39,      5,      1,    -12,     -4,      1,      5 },
      {     38,     20,    -21,     19,     80,     46,    -59,     11,    101,     70,    -48,      7,    140,   -112,     32,      2 },
      {    -16,    -46,     30,     -2,    -12,    -74,     56,    -18,     37,   -114,     60,    -15,    172,     45,    -57,     10 },
      {     33,     76,     70,    -24,     30,     53,     61,    -92,    -34,    -82,   -130,    101,     20,     22,     33,    -16 },
      {    -10,    -18,      9,    -50,    -15,    -35,     59,      6,    -33,    -90,     46,    -16,    -32,   -193,     90,     -7 },
      {     10,     34,     96,    164,    -31,    -59,    -76,   -112,      1,      7,     57,     13,    -19,    -34,     17,     -2 },
      {    -19,    -60,    -64,     36,    -28,    -85,    -94,     68,    -21,    -53,    -68,     75,     24,     43,    114,    -76 },
      {     -9,    -24,    -64,   -100,     -5,    -15,    -40,   -103,     16,     43,     95,    155,     -4,    -17,    -41,    -35 },
      {      5,     22,     26,    -17,     11,     45,     69,    -24,     17,     63,     99,    -38,     33,     80,    158,    -91 },
      {      6,     18,     49,     80,      9,     27,     78,    131,      5,     13,     42,    107,     -8,    -34,    -75,   -110 },
      {     -1,     -3,    -14,    -27,     -2,     -8,    -31,    -66,     -3,    -15,    -52,   -108,     -6,    -28,    -78,   -194 },
    },
    {
      {    211,    -19,    -20,     -3,   -137,     14,     14,      2,     33,     -6,     -3,     -1,      1,      1,     -1,      0 },
      {   -115,     17,     -1,      2,   -135,     -1,     14,      2,    176,    -12,    -15,     -2,    -48,      7,      5,      1 },
      {    -74,     38,     -9,      3,   -145,      0,     14,      0,   -124,     -9,     14,      1,    146,     -8,    -16,     -2 },
      {    -36,   -184,     36,      7,    -28,    157,    -23,     -8,    -33,    -36,     -1,      5,    -24,     -9,      8,      0 },
      {    -21,     47,    -10,     -5,    -71,     -9,      7,      4,   -124,     40,      8,     -1,   -198,    -28,     18,     -1 },
      {      9,    110,    -34,      0,     21,     98,      9,     -7,     -9,   -194,     23,      8,    -27,     53,    -12,     -4 },
      {     18,     47,    191,    -28,    -22,    -11,   -150,     17,      5,    -17,     45,     -6,      0,     -4,      3,     -1 },
      {      6,     74,    -64,     13,     18,    119,    -50,     -2,     39,     76,     81,    -13,     21,   -155,    -13,      8 },
      {     12,     71,     86,     -5,     12,     81,     69,    -10,     -3,     31,   -182,      8,     19,    -44,     68,      2 },
      {     -2,    -41,     31,   -116,      6,    -65,     79,     64,      2,    -99,     15,    -15,     -3,   -156,    -13,      3 },
      {      6,    -15,     48,    167,     -2,    -58,     25,   -132,      0,    -61,     10,     30,     -9,    -91,    -13,     14 },
      {     -5,    -22,    -97,     22,     -9,    -50,   -141,     28,     -9,    -62,    -82,    -22,      9,    -47,    135,      3 },
      {     -2,     -1,    -25,   -110,     -4,     -7,    -32,   -114,      4,      7,     -1,    188,      3,     -9,     19,    -52 },
      {      1,      2,     26,    -37,      3,      6,     69,    -72,      6,     10,    111,    -56,     16,     29,    187,     43 },
      {     -3,     -2,    -24,    -90,     -4,     -4,    -44,   -134,     -6,     -4,    -55,   -100,     -6,      3,    -76,    134 },
      {      0,     -2,     -4,    -28,      0,     -2,     -6,    -80,      0,     -1,    -12,   -123,     -1,     -6,    -14,   -207 },
    },
    {
      {    250,    -27,    -19,     -4,    -32,      8,      2,      1,    -28,      2,      3,      0,     -4,      0,      0,      0 },
      {    -28,     14,     -3,      0,   -246,     20,     19,      3,     20,    -18,      1,     -1,     50,     -4,     -5,      0 },
      {     31,     -2,      3,     -2,     14,    -26,      3,      0,    250,    -14,    -24,     -1,     -5,     23,     -2,      2 },
      {     -8,     25,     -3,      0,    -49,      4,      2,      1,      0,     21,     -3,      0,   -248,      1,     25,      1 },
      {    -26,   -243,     25,     17,    -17,     14,     -2,     -1,      6,     63,    -11,     -4,    -14,      2,      3,      1 },
      {     -2,      1,    -20,      7,     23,    239,    -22,    -16,     30,    -38,     20,     -2,     -3,    -62,      7,      4 },
      {     -7,    -65,    -13,      0,      7,    -32,     32,      1,    -18,   -239,     12,     18,    -30,     13,    -15,      0 },
      {     19,     18,    243,    -10,     -5,     12,    -32,     -4,     -8,    -27,    -52,      5,     -3,    -26,      0,      2 },
      {      0,    -11,    -19,      8,    -16,    -61,   -194,      7,     15,    -21,     80,     -9,      2,   -117,     49,      5 },
      {      3,     -3,     -1,     -1,      9,    -43,    132,     -7,     13,     15,     -2,      3,     -4,   -211,    -29,     14 },
      {      7,     10,     56,    148,      2,     -4,     48,    -45,     12,     10,    181,    -33,     -3,     36,    -23,      8 },
      {     -3,    -13,     32,   -196,      2,      6,     30,      9,     12,     17,    147,     48,     -2,     21,    -10,      4 },
      {     -1,     -1,    -15,    -25,     -7,    -14,    -32,   -235,     -4,      4,    -28,     39,     -4,      3,    -57,     53 },
      {      1,     -2,      7,    -17,      3,     -6,     57,    -68,     -1,    -12,      0,    -47,     22,      6,    233,     12 },
      {      1,      3,      0,     61,     -1,      2,      6,     16,      3,     14,      4,    241,      7,     -1,     57,     -9 },
      {      0,      1,      0,      7,      1,      3,      0,     56,     -3,      0,     -3,      3,      2,     13,      4,    249 },
    },
  },
  { // 8
    {
      {    206,    -65,      4,     -6,   -125,     40,      2,      2,     39,    -10,     -4,      1,     -9,      1,      2,      0 },
      {    115,    -41,     12,     -5,    153,    -13,    -12,      0,   -153,     46,      9,      0,     28,    -15,     -1,      1 },
      {    -72,   -184,     86,     -2,     24,    118,    -61,      0,     18,    -14,     12,      2,     -7,    -14,      3,      0 },
      {    -56,    -18,     30,    -10,   -144,    -21,      5,      7,   -138,     41,     -8,     -2,    133,    -41,     -6,      0 },
      {     -5,    -56,     72,    -26,     -7,   -134,     -1,     16,     60,    160,    -59,     -5,    -56,    -35,     35,     -1 },
      {     13,    -62,     53,    -16,     37,   -117,     32,     -3,     60,    -96,    -19,      9,    126,    103,    -48,     -4 },
      {     28,    114,    150,    -72,     -1,      7,   -134,     59,      8,    -29,     37,    -14,      9,      1,     14,     -4 },
      {    -20,    -21,     55,    -29,    -48,    -43,     43,      2,   -117,    -81,    -37,     25,   -157,     88,      4,    -17 },
      {     -3,     32,    -15,     -6,      2,     92,    -24,      6,     -2,     98,   -110,     15,     39,    173,     28,    -29 },
      {     10,     41,     73,    -61,     35,     65,    120,    -19,     18,    -43,   -131,     55,     30,    -98,     40,    -10 },
      {      3,    -20,    -59,     83,      0,    -52,   -124,     29,    -13,    -83,   -119,     14,     10,    -43,    110,    -22 },
      {    -14,    -43,   -109,   -176,      7,     -1,      1,    140,      2,    -15,     -3,    -17,      3,     -9,     11,    -19 },
      {     -2,     -9,     14,    -18,      2,     -4,     61,    -36,      3,     -7,     99,    -45,     27,     40,    213,    -17 },
      {     -3,     -6,    -40,    -86,     -8,    -23,    -67,   -124,      5,     12,     41,    178,      4,     -4,      8,    -49 },
      {      2,      3,     27,     95,      4,      6,     50,    141,      8,     18,     60,    120,      6,     -1,      4,   -122 },
      {      2,      1,     -2,    -18,      1,     -2,     -9,    -66,      1,     -6,    -15,   -112,     -5,    -24,    -39,   -214 },
    },
    {
      {    251,      2,    -11,     -5,    -27,     33,      1,      2,    -26,     -3,      2,      0,     -4,      3,      0,      0 },
      {     23,    -44,      5,     -2,    233,     10,    -26,     -2,    -17,     77,     -3,      1,    -42,      3,     11,     -1 },
      {    -33,    -26,      0,      2,     -9,     85,     -6,      0,   -215,    -18,     46,      1,     27,    -84,      0,      1 },
      {      0,   -155,     -4,     12,     28,     85,    -18,     -8,     65,   -102,     15,      3,    121,     36,    -49,      1 },
      {      3,    179,    -17,    -15,     91,     27,     51,     -6,     -4,   -112,      5,     15,     81,     20,    -39,      2 },
      {    -22,     50,    -35,     13,    -18,    198,    -11,    -18,     86,     37,     65,    -10,    -82,    -23,     25,      8 },
      {     12,     49,     70,      6,     -7,     -4,    -74,      9,     45,    121,     37,    -34,    160,    -86,      3,     17 },
      {     17,    -10,    147,     10,     21,    -26,     -1,      8,     48,   -105,     32,      9,    -81,   -141,    -24,     21 },
      {      9,    -23,   -183,      4,     15,    -71,     25,    -30,     39,    -11,     59,      1,      7,   -136,    -20,     23 },
      {      1,     35,    -41,     35,      2,    -32,   -221,     -9,    -16,    -78,     26,    -41,    -30,     35,     28,      2 },
      {      4,     -9,     36,    -21,      0,    -54,     38,    -34,    -11,      8,    221,      5,     -3,     90,     15,     32 },
      {      2,     16,      5,    179,     -8,     -7,      4,    -30,    -18,     43,      7,      3,    -25,     25,   -170,     10 },
      {     -6,      4,     -7,   -166,    -16,      9,    -66,     15,     -1,     33,      7,     67,    -30,      4,   -163,      5 },
      {      2,      0,     27,    -26,     -6,      0,    -10,   -245,     -6,      4,    -47,     24,      6,     -6,     16,     26 },
      {     -2,     -6,      3,    -55,      4,      1,     34,    -19,     -8,     -7,    -11,   -239,    -19,      6,    -55,     17 },
      {     -3,     -2,     -6,      3,     -2,      9,      0,     34,    -10,     -1,    -37,     14,     -1,     18,     13,    249 },
    },
    {
      {    215,    -53,     -2,     -5,    123,     -1,    -15,      0,      8,     26,     -9,      1,    -13,     13,      0,      0 },
      {   -111,    -13,      9,      0,    171,    -86,     -1,     -3,    118,     14,    -27,      3,     -5,     37,     -9,      0 },
      {     58,    -48,     17,     -1,    -96,    -78,     31,      2,    141,   -121,     -9,      5,     85,     12,    -35,      5 },
      {     25,    119,    -62,      6,    -28,    121,    -46,     -5,    134,     66,    -23,    -12,     20,     77,      0,     -6 },
      {    -14,   -138,     37,      1,    -89,    -33,    -45,     20,     13,    131,    -72,     -1,    -40,     94,     13,    -17 },
      {    -41,   -108,      9,      4,     41,    111,    -80,      5,     -2,     10,     10,    -11,    165,    -65,    -11,     -1 },
      {      0,     32,   -128,     40,      7,    -71,   -100,     29,    -85,    -49,   -110,     12,     48,     48,    -63,    -14 },
      {     -7,     16,     76,    -27,     31,     48,    108,    -16,    -73,    -19,    -15,     21,     79,    161,    -70,     -4 },
      {     29,    101,     72,    -30,    -14,    -94,     25,     12,    -12,    121,    -63,      2,    115,    -79,     16,     -3 },
      {      1,    -26,   -129,     19,     -7,    -69,     55,    -50,    -17,     64,    130,    -53,     77,     55,     66,     -4 },
      {      7,     49,    103,     -1,      7,    -33,   -124,     50,    -22,    -62,     63,    -59,     22,     80,    116,    -35 },
      {     -4,     -4,    -10,   -173,     -6,     -7,    -48,   -162,     -8,    -25,    -42,    -64,    -11,     -1,      2,    -11 },
      {     -6,    -24,    -46,    -15,     14,     42,     87,     26,     -1,    -48,   -120,     23,     18,     -4,    176,    -61 },
      {      4,      2,     39,    141,      4,     12,     33,    -75,     -2,     -8,    -58,   -172,    -12,    -20,    -25,    -57 },
      {     -4,     -8,    -36,   -106,      4,      6,     38,    150,      7,      7,     20,   -114,    -14,    -17,    -67,   -103 },
      {     -4,     -6,    -11,    -24,      3,      9,     20,     61,     -8,     -9,    -43,   -106,      2,     13,     29,    215 },
    },
  },
  { // 9
    {
      {    222,    -53,    -11,     -6,    103,      9,    -20,      0,    -11,     36,     -9,      0,    -21,     15,      2,     -1 },
      {    -89,    -31,     13,      2,    177,   -100,     -6,     -1,    106,     20,    -37,      2,    -12,     40,     -9,     -2 },
      {     63,    -85,     27,      2,    -91,   -101,     40,      8,    104,   -127,    -13,     11,     66,      5,    -38,      6 },
      {    -43,   -171,     65,      5,    -10,    -73,     -4,     17,   -141,     36,     -3,      7,    -30,    -43,     18,      0 },
      {      6,     67,     11,     -8,    103,    -32,     73,    -16,    -76,   -129,     86,     15,     14,   -112,     -7,     24 },
      {     38,     95,     29,    -12,    -53,   -127,     89,      4,    -16,     33,    -19,     12,   -149,     47,     30,      0 },
      {     -1,     32,   -154,     39,     -4,    -84,   -111,     28,    -75,    -64,    -87,      7,     -7,      1,    -37,    -14 },
      {     15,     75,     84,    -26,     15,    -32,     41,     21,    -79,     51,    -98,     20,    153,     45,    -64,    -16 },
      {    -25,    -45,     -8,      1,     29,     81,     61,    -13,    -60,    -97,      2,     35,    -54,    170,    -74,     -6 },
      {     -3,    -26,   -130,     -3,     -6,    -64,     79,    -50,    -27,     64,    107,    -37,     95,     73,     55,     -6 },
      {     10,     40,     90,     36,      8,    -30,   -111,     65,    -23,    -62,     78,    -38,     30,     96,    116,    -31 },
      {     -4,     -1,     16,   -183,     -6,    -14,    -60,   -139,    -17,    -41,    -32,    -66,     -8,     12,     25,    -16 },
      {     -7,    -23,    -36,     -9,     20,     50,     88,     37,      0,    -52,   -126,     13,     16,    -17,    172,    -54 },
      {      6,      4,     33,    141,      5,      6,     30,    -97,    -11,    -15,    -46,   -169,    -10,    -11,    -22,    -49 },
      {     -5,     -9,    -25,    -88,      6,      6,     35,    151,      6,      2,     29,   -125,    -21,    -19,    -65,   -107 },
      {      4,      3,      9,     21,     -4,    -10,    -16,    -63,     10,     10,     38,    108,     -4,    -22,    -18,   -216 },
    },
    {
      {   -209,    125,    -34,     16,     55,    -38,     10,     -3,      9,     -8,      2,     -2,      2,      1,     -1,      0 },
      {    -66,      4,     12,      0,   -205,     95,     -9,      6,     77,    -58,      7,      1,     25,     -7,     -3,     -1 },
      {    108,    150,   -152,     33,    -54,    -19,     51,    -14,    -21,    -16,     15,     -2,      6,      0,     -6,      2 },
      {    -16,     22,    -32,     16,     25,    164,   -102,     11,   -137,     47,     39,    -11,     21,    -43,     21,     -1 },
      {    -34,    -22,     44,    -18,    -97,    -93,     76,     -6,   -166,     40,    -18,     13,     78,    -43,    -12,      3 },
      {      4,    -29,      9,      4,     53,      6,     29,    -23,    -40,   -200,    113,      7,     53,    -30,    -44,     14 },
      {      6,     60,     77,    -68,    -33,      6,     26,      3,    -65,    -36,     42,     10,   -203,      6,     32,      1 },
      {    -55,   -125,   -105,    110,    -11,     26,     80,    -76,    -27,     15,      7,      3,    -96,     19,     -3,      9 },
      {      6,     -2,      4,     12,      9,    -18,     29,    -19,     63,     34,     48,    -28,    -10,   -216,     91,     24 },
      {     12,     45,     59,    -52,     50,    122,    148,   -104,     22,     35,    -38,     39,     45,     11,    -14,      8 },
      {     -2,      4,     27,     -9,    -17,    -15,     21,    -18,     28,    101,    188,   -101,     23,     73,    -28,     22 },
      {     10,     26,     36,     52,     -3,      5,    -23,      9,     12,     31,    -21,      7,    -46,    -75,   -209,     83 },
      {     30,     59,    124,    194,     -5,     -7,    -12,    -39,    -13,    -21,    -19,    -30,     11,     30,     57,    -14 },
      {     -2,     -8,      6,     42,     26,     53,    121,    207,     -1,     -2,     -7,    -46,     -2,     -2,     -2,    -22 },
      {     -3,     -4,    -11,    -44,      3,      9,      0,    -44,    -22,    -48,    -94,   -222,     -8,     -7,    -10,     20 },
      {     -2,     -3,     -5,     -2,      0,      2,      2,     30,     -7,    -11,    -14,     20,     16,     45,     74,    237 },
    },
    {
      {    168,    -33,    -18,     -4,   -166,     34,     20,      4,     75,    -18,    -10,      0,    -18,      5,      2,      0 },
      {     65,    157,    -43,    -21,    -17,   -154,     45,     20,    -48,     64,    -19,     -6,     32,     -8,      1,     -1 },
      {   -117,     54,      3,     -1,    -45,    -54,     15,      6,    179,      8,    -34,     -8,   -100,     12,     21,      2 },
      {     79,    -81,    -31,      9,    106,    -13,     19,     -7,     24,    139,    -32,    -10,   -110,    -72,     38,     11 },
      {     24,      8,    159,    -26,    -27,    -45,   -157,     35,     13,     59,     66,    -29,     -1,    -32,     -3,     12 },
      {    -98,    -73,    -14,     21,   -116,     -8,     39,     -7,    -24,    112,    -24,    -13,    100,   -105,      4,     12 },
      {    -18,    100,    -56,     31,    -36,     82,    -14,    -32,    -41,    -12,    115,      2,    -95,   -127,    -32,     24 },
      {     17,    -43,    -75,     57,     40,    -60,    -20,    -46,     94,     29,    125,      4,     70,     36,   -123,     11 },
      {    -19,    -32,    -60,   -160,      7,     18,     26,    156,      6,      0,     50,    -69,     -6,     -5,    -48,     10 },
      {     40,     79,     12,    -15,     80,     83,     -7,     11,    114,    -18,    -54,    -14,    127,   -103,     18,     17 },
      {     14,    -62,     72,     -7,     17,   -110,     71,     -2,      0,   -129,    -12,     26,    -24,   -126,    -67,     18 },
      {      3,    -20,    -49,     97,      0,    -40,    -38,     30,    -13,    -65,     -9,   -172,      4,     -1,     68,    102 },
      {      6,     41,    102,     53,      3,     64,     95,     10,    -14,     56,    -44,    -85,    -25,     49,   -141,     47 },
      {      7,      0,     78,    -23,     14,      5,    137,    -35,     21,     11,    142,    -24,     27,     26,    124,     22 },
      {      6,      3,     14,    134,      9,      3,     27,    145,      8,     -3,     34,      5,      2,    -19,     18,   -153 },
      {      1,     -3,     -2,    -56,     -1,     -8,     -6,   -114,     -1,    -14,     -8,   -146,     -4,    -22,    -16,   -165 },
    },
  },
  { // 10
    {
      {    239,     24,    -17,     -2,    -65,     51,     13,      0,    -13,    -21,     13,      2,      1,      2,     -3,      2 },
      {     39,   -106,     -3,      2,    168,     93,    -51,     -7,    -71,     69,     47,     -7,     -7,    -32,     15,     10 },
      {    -66,    -65,    -42,     12,    -73,    136,     40,    -24,    -49,   -129,     64,     26,     38,    -21,    -60,      7 },
      {     15,   -198,      0,     17,    -90,    -34,    -69,     -1,     89,     25,    -27,    -22,      0,     41,      8,     -7 },
      {    -29,     52,     54,     13,   -103,     33,    -88,    -11,      0,     91,    128,    -44,     -4,    -85,     44,     59 },
      {     16,    -29,    135,      0,    -17,    -80,    -64,     47,   -146,    -64,    -33,    -20,     64,    -24,    -38,     -7 },
      {     29,     -7,    112,     20,     70,    -35,     11,    -19,    124,    -63,     87,     52,    -32,    -27,   -104,     52 },
      {    -12,     53,     98,     23,     26,    126,    -63,     14,     66,    -45,    -47,    -61,     49,    117,     56,     -7 },
      {     -7,    -53,    113,    -45,    -28,     50,    166,     79,      6,     49,    -32,     10,    -22,    -45,     67,     19 },
      {    -14,     20,     44,     63,    -39,     69,    -44,      0,    -25,     46,    -75,     53,   -152,    -19,    -88,    -92 },
      {     -3,      8,    -40,   -159,     12,     25,    -65,    102,     47,    -60,    -26,    -80,    -58,    -75,    -36,     -4 },
      {      1,    -18,      5,     35,      7,    -33,      3,    -51,    -46,   -106,      5,    -52,   -161,     16,    111,     74 },
      {     11,      2,    -21,    131,     27,      3,     10,     -5,     57,    -41,    -74,    -67,     52,   -165,     33,    -32 },
      {      5,     -8,     54,   -100,      1,     -8,      1,   -148,     15,    -26,     26,      8,      9,    -41,     63,   -152 },
      {     -1,      7,      1,    -49,     -7,     27,    -72,    -44,     12,    -15,   -103,    162,     27,    -56,     54,    107 },
      {     -1,     -1,     19,    -43,     -9,     19,     41,   -137,    -16,     51,    -84,   -113,      9,     -1,   -103,     95 },
    },
    {
      {    243,     -9,    -15,     -2,    -16,     69,     -1,      0,    -28,      1,     22,     -1,     -1,     -8,      4,      6 },
      {     -4,    -91,     16,     -2,    195,     22,    -62,      2,    -40,    106,     12,    -14,    -15,    -13,     33,      4 },
      {    -73,    -15,    -14,     11,    -31,    158,     -7,    -21,   -108,    -45,    113,      3,     29,    -70,    -20,     27 },
      {      2,   -215,     12,     15,    -48,    -18,    -46,      2,     22,    -80,    -27,      7,     30,     29,    -67,    -15 },
      {    -10,    -28,     24,      2,   -144,    -11,   -111,     13,      2,    130,     17,    -66,    -10,    -29,     91,      8 },
      {      5,     10,     16,    -11,    -18,    -67,      4,     43,   -207,     -7,   -105,     -2,     32,    -22,     -3,    -56 },
      {     29,     13,    147,     -2,     15,   -115,    -10,     19,     -4,    -31,    100,      5,     13,   -101,    -45,     75 },
      {     -2,     57,    130,     -7,     30,     86,    -76,     56,     29,    -68,    -49,    -75,     88,     78,     26,    -13 },
      {      1,    -55,    -35,    -56,     10,    -34,    114,      2,      5,    -25,     22,    -55,    129,     -9,    143,     52 },
      {    -13,    -55,    130,    -19,    -33,     72,    136,     58,      2,     45,    -29,     56,    -94,     11,     38,    -13 },
      {      1,      2,    -12,    -46,      6,     40,     11,     45,     83,     37,    -55,      6,     79,   -169,    -48,   -105 },
      {      1,     10,     29,     58,    -22,      3,     27,    -27,     -7,    121,     18,    107,    151,     76,    -64,     18 },
      {     -9,      2,    -64,    -97,     -7,     17,    -17,    153,     -2,     30,    -27,      5,     -4,     20,    -73,    143 },
      {     -1,      0,     25,   -204,    -11,    -13,     -6,    -72,    -17,     24,     66,    -15,     -2,     65,    -48,    -76 },
      {      0,      8,      1,    -62,      1,      2,    -92,     -7,      8,    -56,    -21,    192,      5,    -22,    109,     15 },
      {      1,      0,    -38,     36,      7,    -37,     -4,    157,      0,    -14,    133,     25,      3,     50,     29,   -122 },
    },
    {
      {    220,   -113,     23,     -7,     41,     18,    -19,      4,    -22,     26,     -1,     -3,     -8,      3,      5,      0 },
      {    -60,    -55,     41,     -6,    181,   -135,      3,     13,     15,     48,    -48,      7,    -23,     25,      3,     -8 },
      {     49,    147,   -120,     22,    129,     59,    -24,    -16,    -22,     50,     32,    -16,    -20,     -1,     27,      1 },
      {     78,     63,    -23,     -2,     11,    -85,    109,    -25,    122,   -128,     17,     29,     -6,     28,    -42,     17 },
      {     17,     48,     54,    -37,     -4,    -50,    139,    -30,   -173,     20,     44,     23,     -2,    -49,     -2,     24 },
      {    -44,    -50,     72,    -38,     93,     97,     10,    -41,     47,    -59,    146,    -22,     31,    -62,     15,     43 },
      {    -23,    -90,   -133,    111,     13,    -57,    -24,     12,    -79,    -84,     52,     -4,     41,    -48,    -49,     18 },
      {    -22,    -73,    -70,     34,    -48,    -18,     85,   -105,     46,     96,     65,    -59,    -76,     55,     72,      0 },
      {    -33,    -36,     -3,      9,     33,    103,     29,    -11,    -47,    -42,    -22,     75,   -133,    103,   -109,      6 },
      {     15,     42,     50,      6,    -30,    -75,    -95,     68,    -25,    -49,     99,    -53,   -152,     13,     40,     17 },
      {      5,     10,     35,    124,    -13,     13,     37,     96,     59,     92,     31,    108,    -14,    -47,     -2,    104 },
      {     11,     33,     34,      1,    -17,    -40,    -58,    -51,     -2,     72,     61,    -61,     73,     91,   -149,     83 },
      {     -2,    -17,    -71,   -125,    -24,    -39,    -66,    -71,     26,     31,    -13,     87,    -64,   -110,    -39,     78 },
      {    -13,    -32,    -76,   -117,     -3,     -1,     23,    118,    -19,     -5,     42,     33,     56,    114,     67,     80 },
      {      4,     15,     42,     61,      4,     -7,    -62,   -119,    -39,    -62,    -49,     66,     28,     66,    115,    105 },
      {      4,      7,      8,      0,    -12,    -34,    -48,    -32,      0,     28,    111,    152,     41,     45,      7,   -145 },
    },
  },
  { // 11
    {
      {   -167,     71,      1,      6,    152,    -56,     -6,     -3,    -74,     19,     10,      1,     24,     -3,     -7,      2 },
      {    149,     83,    -69,     -2,     28,    -98,     58,     -1,    -97,     77,     -7,     -3,     40,    -27,     -5,      7 },
      {    -68,    134,    -61,      3,    -93,    -83,     68,     -1,    125,     -8,    -34,      1,    -47,     15,      9,     -1 },
      {     17,    -67,     78,    -27,     53,   -104,    -52,     30,     85,    136,    -49,    -23,    -77,    -14,     48,     -4 },
      {     78,     26,    -68,     10,    150,     44,      6,    -20,     67,    -70,     26,      2,   -119,     71,     10,     -2 },
      {    -42,   -132,    -93,     62,     23,      9,    132,    -59,      1,     34,    -97,     36,      6,     -9,     17,     -6 },
      {     22,    -47,     83,    -30,      7,   -125,     44,     20,    -20,   -116,    -55,     34,     26,    116,    -68,    -24 },
      {      7,     62,     46,   -105,     18,     89,     47,     63,    -37,    -18,   -138,     20,     -5,    -10,    106,    -40 },
      {    -17,    -35,   -136,    -12,    -14,    -13,    -95,     85,     -7,      4,    -43,   -102,     61,    102,     33,    -36 },
      {     21,     55,     40,    135,    -18,     -1,    -94,   -107,    -19,      9,    -66,     58,     19,     63,     76,    -59 },
      {    -35,    -10,     -8,      4,    -77,      4,     10,     17,   -138,     42,     18,     -2,   -178,     67,    -15,    -17 },
      {      6,      6,    -51,     26,     10,     -2,    -81,     66,      4,    -24,    -81,     80,    -41,   -109,   -128,    -89 },
      {      5,    -24,      4,     80,     -5,    -70,     -9,     32,    -41,   -129,    -17,    -84,    -49,   -113,     97,     47 },
      {     13,     45,     63,    108,     16,     66,     33,     54,     10,     37,    -88,   -101,      2,     34,   -100,     92 },
      {      5,      4,     47,     52,     12,     12,     80,     22,     16,     11,     62,    -97,     11,    -15,     -1,   -198 },
      {      2,     -6,      5,     95,      4,      3,     35,    169,     14,     21,     69,    123,     23,     34,     63,     21 },
    },
    {
      {   -217,    111,    -11,      7,    -56,    -15,     25,     -4,     25,    -33,      1,      4,     10,     -4,     -7,      1 },
      {     70,    118,    -92,      7,    -82,    152,    -29,    -18,    -33,      0,     61,    -26,     14,    -33,     24,      7 },
      {     -1,   -107,     91,    -12,   -197,     36,     22,     11,    -20,    -57,      4,     14,     17,     -6,    -27,      1 },
      {    -89,    -69,     14,     13,      2,     68,   -123,     27,    -82,    134,    -37,    -35,    -13,      1,     54,    -40 },
      {     -5,     41,     94,    -56,     28,     -1,    127,    -36,   -142,     69,     64,     21,    -25,    -41,     28,     34 },
      {     51,     72,    -56,     20,    -86,    -85,     30,     37,    -80,     51,   -138,     38,    -45,     57,    -18,    -48 },
      {    -25,    -60,   -100,    104,     17,    -56,    -14,     39,   -127,    -62,     55,     22,     28,    -83,    -54,     34 },
      {    -33,    -55,    -51,     41,     19,     73,     46,    -61,    -16,    -18,     46,     29,   -143,    137,    -51,    -10 },
      {     -4,     38,     64,     -9,     66,    111,    -28,     78,    -19,    -34,    -76,    132,     16,    -12,   -102,     29 },
      {     23,     52,     71,     80,    -36,    -49,    -62,    105,     43,     34,     81,     17,   -114,      5,     27,     93 },
      {     -9,    -46,    -53,     81,    -26,     30,     93,    -15,     84,    138,    -17,     72,     71,    -15,      2,     83 },
      {      2,     -4,     28,    105,     31,     48,     88,     88,      6,    -55,     -3,     16,     14,      4,    132,   -119 },
      {     17,     46,     83,    110,     14,      3,      8,     -8,    -24,     31,     21,   -116,     94,     84,   -110,    -20 },
      {    -14,    -19,    -46,    -74,     10,     12,     27,    110,    -43,    -32,     -3,    -45,     69,    131,     60,    118 },
      {     11,     25,     42,     57,     -4,    -31,    -83,   -126,    -42,    -43,      0,    107,     68,     76,     95,     39 },
      {      2,      4,    -26,    -67,    -18,    -42,    -26,     57,     12,     58,    138,    100,     66,     58,    -39,   -113 },
    },
    {
      {    236,    -11,    -22,     -5,    -69,     62,      8,     -1,    -16,    -15,     15,      0,     -2,     -2,      0,      3 },
      {     28,    -84,     10,      3,    175,     70,    -66,     -7,    -83,     80,     49,    -13,     -3,    -34,     22,     11 },
      {     39,    205,    -28,    -22,     89,    -50,     57,      6,    -81,     11,      0,     11,      6,    -19,      9,      0 },
      {     66,    -16,     28,    -15,     27,   -136,    -38,     31,     82,    121,    -90,    -32,    -38,     39,     68,    -17 },
      {    -30,     86,    -37,      7,    -47,     89,    -33,    -31,    106,     99,     99,    -37,    -49,    -40,     76,     41 },
      {     12,    -28,   -114,     10,    106,     45,    101,    -44,    111,    -32,    -37,     58,    -59,     61,    -15,    -19 },
      {     41,     27,    146,      8,     66,    -38,    -23,     -1,    103,    -68,     83,     34,    -38,    -14,    -81,     64 },
      {     -3,     64,     69,     -8,     38,    113,    -70,     28,     29,    -73,    -73,    -61,     47,    117,     65,    -40 },
      {     16,     10,   -121,     42,     16,    -86,   -126,    -66,     15,    -46,     65,    -48,     75,     68,    -36,     21 },
      {     -4,     45,     -2,     77,    -24,     25,   -109,    -37,    -37,     14,    -57,     49,   -142,    -11,    -84,    -93 },
      {     -8,     -9,    -72,   -153,     14,     -4,    -52,    107,     11,    -71,      2,    -74,    -91,    -52,    -29,      0 },
      {     -6,    -25,     15,     43,    -10,    -43,     24,    -46,    -77,    -82,     30,    -35,   -137,     56,    126,     75 },
      {    -16,      2,      9,    -91,    -33,      9,      2,     35,    -46,     79,     79,     75,    -29,    175,    -51,     14 },
      {      5,    -12,     35,   -114,     -1,    -32,    -12,   -122,     10,    -35,     59,     47,     11,    -28,     74,   -147 },
      {     -3,     10,    -18,    -72,     -9,     17,    -83,    -51,     -2,    -14,    -97,    139,     22,    -34,     42,    134 },
      {     -2,      3,     36,    -73,     -6,     21,     38,   -151,    -16,     40,    -64,   -127,    -16,     14,    -94,     54 },
    },
  },
};
#endif

//--------------------------------------------------------------------------------------------------

#if VCEG_AZ07_INTRA_4TAP_FILTER
Int g_aiIntraCubicFilter[32][4] = {
  {   0, 256,   0,   0 }, //  0 Integer-Pel
  {  -3, 252,   8,  -1 }, //  1
  {  -5, 247,  17,  -3 }, //  2
  {  -7, 242,  25,  -4 }, //  3
  {  -9, 236,  34,  -5 }, //  4
  { -10, 230,  43,  -7 }, //  5
  { -12, 224,  52,  -8 }, //  6
  { -13, 217,  61,  -9 }, //  7
  { -14, 210,  70, -10 }, //  8
  { -15, 203,  79, -11 }, //  9
  { -16, 195,  89, -12 }, // 10
  { -16, 187,  98, -13 }, // 11
  { -16, 179, 107, -14 }, // 12
  { -16, 170, 116, -14 }, // 13
  { -17, 162, 126, -15 }, // 14
  { -16, 153, 135, -16 }, // 15
  { -16, 144, 144, -16 }, // 16 Half-Pel
};
Int g_aiIntraGaussFilter[32][4] = {
  {  47, 161,  47,   1 }, //  0 Integer-Pel
  {  43, 161,  51,   1 }, //  1
  {  40, 160,  54,   2 }, //  2
  {  37, 159,  58,   2 }, //  3
  {  34, 158,  62,   2 }, //  4
  {  31, 156,  67,   2 }, //  5
  {  28, 154,  71,   3 }, //  6
  {  26, 151,  76,   3 }, //  7
  {  23, 149,  80,   4 }, //  8
  {  21, 146,  85,   4 }, //  9
  {  19, 142,  90,   5 }, // 10
  {  17, 139,  94,   6 }, // 11
  {  16, 135,  99,   6 }, // 12
  {  14, 131, 104,   7 }, // 13
  {  13, 127, 108,   8 }, // 14
  {  11, 123, 113,   9 }, // 15
  {  10, 118, 118,  10 }, // 16 Half-Pel
};
#endif

//coefficients

#if RExt__HIGH_PRECISION_FORWARD_TRANSFORM
const TMatrixCoeff g_aiT4 [TRANSFORM_NUMBER_OF_DIRECTIONS][4][4]   =
{
  DEFINE_DCT4x4_MATRIX  (16384, 21266,  9224),
  DEFINE_DCT4x4_MATRIX  (   64,    83,    36)
};

const TMatrixCoeff g_aiT8 [TRANSFORM_NUMBER_OF_DIRECTIONS][8][8]   =
{
  DEFINE_DCT8x8_MATRIX  (16384, 21266,  9224, 22813, 19244, 12769,  4563),
  DEFINE_DCT8x8_MATRIX  (   64,    83,    36,    89,    75,    50,    18)
};

const TMatrixCoeff g_aiT16[TRANSFORM_NUMBER_OF_DIRECTIONS][16][16] =
{
  DEFINE_DCT16x16_MATRIX(16384, 21266,  9224, 22813, 19244, 12769,  4563, 23120, 22063, 20450, 17972, 14642, 11109,  6446,  2316),
  DEFINE_DCT16x16_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9)
};

const TMatrixCoeff g_aiT32[TRANSFORM_NUMBER_OF_DIRECTIONS][32][32] =
{
  DEFINE_DCT32x32_MATRIX(16384, 21266,  9224, 22813, 19244, 12769,  4563, 23120, 22063, 20450, 17972, 14642, 11109,  6446,  2316, 23106, 22852, 22445, 21848, 20995, 19810, 18601, 17143, 15718, 13853, 11749,  9846,  7908,  5573,  3281,   946),
  DEFINE_DCT32x32_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9,    90,    90,    88,    85,    82,    78,    73,    67,    61,    54,    46,    38,    31,    22,    13,     4)
};

const TMatrixCoeff g_as_DST_MAT_4[TRANSFORM_NUMBER_OF_DIRECTIONS][4][4] =
{
  DEFINE_DST4x4_MATRIX( 7424, 14081, 18893, 21505),
  DEFINE_DST4x4_MATRIX(   29,    55,    74,    84)
};

#else

const TMatrixCoeff g_aiT4 [TRANSFORM_NUMBER_OF_DIRECTIONS][4][4]   =
{
  DEFINE_DCT4x4_MATRIX  (   64,    83,    36),
  DEFINE_DCT4x4_MATRIX  (   64,    83,    36)
};

const TMatrixCoeff g_aiT8 [TRANSFORM_NUMBER_OF_DIRECTIONS][8][8]   =
{
  DEFINE_DCT8x8_MATRIX  (   64,    83,    36,    89,    75,    50,    18),
  DEFINE_DCT8x8_MATRIX  (   64,    83,    36,    89,    75,    50,    18)
};

const TMatrixCoeff g_aiT16[TRANSFORM_NUMBER_OF_DIRECTIONS][16][16] =
{
  DEFINE_DCT16x16_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9),
  DEFINE_DCT16x16_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9)
};

const TMatrixCoeff g_aiT32[TRANSFORM_NUMBER_OF_DIRECTIONS][32][32] =
{
  DEFINE_DCT32x32_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9,    90,    90,    88,    85,    82,    78,    73,    67,    61,    54,    46,    38,    31,    22,    13,     4),
  DEFINE_DCT32x32_MATRIX(   64,    83,    36,    89,    75,    50,    18,    90,    87,    80,    70,    57,    43,    25,     9,    90,    90,    88,    85,    82,    78,    73,    67,    61,    54,    46,    38,    31,    22,    13,     4)
};

const TMatrixCoeff g_as_DST_MAT_4[TRANSFORM_NUMBER_OF_DIRECTIONS][4][4] =
{
  DEFINE_DST4x4_MATRIX(   29,    55,    74,    84),
  DEFINE_DST4x4_MATRIX(   29,    55,    74,    84)
};
#endif


//--------------------------------------------------------------------------------------------------

#undef DEFINE_DST4x4_MATRIX
#undef DEFINE_DCT4x4_MATRIX
#undef DEFINE_DCT8x8_MATRIX
#undef DEFINE_DCT16x16_MATRIX
#undef DEFINE_DCT32x32_MATRIX

//--------------------------------------------------------------------------------------------------


const UChar g_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize]=
{
  //0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,29,30,31,32,33,33,34,34,35,35,36,36,37,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51,51,51 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51,51,51 }
};

// ====================================================================================================================
// Intra prediction
// ====================================================================================================================

#if JVET_C0024_QTBT
const UChar g_aucIntraModeNumFast_UseMPM[7-MIN_CU_LOG2+1][7-MIN_CU_LOG2+1] =
{
#if MIN_CU_LOG2==1
    {2, 2, 2, 2, 2, 2, 2},
    {3, 3, 3, 3, 2, 2, 2},  //4x2, 4x4, 4x8, 4x16, 4x32, 4x64, 4x128
    {3, 3, 3, 3, 3, 2, 2},  //8x2, 8x4, 8x8, 8x16, 8x32, 8x64, 8x128, 
    {3, 3, 3, 3, 3, 2, 2},  //16x2, 16x4, 16x8, 16x16, 16x32, 16x64, 16x128, 
    {3, 3, 3, 3, 3, 2, 2},  //32x2, 32x4, 32x8, 32x16, 32x32, 32x64, 32x128, 
    {2, 3, 3, 3, 3, 2, 2},  //64x2, 64x4, 64x8, 64x16, 64x32, 64x64, 64x128, 
    {2, 2, 2, 2, 2, 3, 2},  //128x2, 128x4, 128x8, 128x16, 128x32, 128x64, 128x128,  
#elif MIN_CU_LOG2==2
    {3, 3, 3, 3, 2, 2},  //4x4, 4x8, 4x16, 4x32, 4x64, 4x128
    {3, 3, 3, 3, 3, 2},  //8x4, 8x8, 8x16, 8x32, 8x64, 8x128, 
    {3, 3, 3, 3, 3, 2},  //16x4, 16x8, 16x16, 16x32, 16x64, 16x128, 
    {3, 3, 3, 3, 3, 2},  //32x4, 32x8, 32x16, 32x32, 32x64, 32x128, 
    {2, 3, 3, 3, 3, 2},  //64x4, 64x8, 64x16, 64x32, 64x64, 64x128, 
    {2, 2, 2, 2, 2, 3},  //128x4, 128x8, 128x16, 128x32, 128x64, 128x128, 
#endif
};
#else
const UChar g_aucIntraModeNumFast_UseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  8,  //   4x4
  8,  //   8x8
  3,  //  16x16
  3,  //  32x32
  3   //  64x64
#if COM16_C806_LARGE_CTU
  ,3  //  128x128   
  ,3  //  256x256
#endif
};
const UChar g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  9,  //   4x4
  9,  //   8x8
  4,  //  16x16   33
  4,  //  32x32   33
  5   //  64x64   33
#if COM16_C806_LARGE_CTU
  ,9 //  128x128
  ,9 //  256x2565
#endif
};
#endif

const UChar g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE] =
#if VCEG_AZ07_INTRA_65ANG_MODES
  //                                                               H                                                               D                                                               V
  //0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
  { 0, 1, 2, 2, 2, 2, 2, 2, 2, 3,  4,  5,  7,  9, 11, 13, 15, 16, 18, 20, 21, 23, 25, 27, 29, 31, 32, 33, 34, 35, 37, 37, 49, 40, 40, 41, 42, 43, 43, 44, 44, 45, 45, 46, 46, 47, 47, 48, 49, 49, 50, 51, 51, 52, 53, 53, 54, 54, 55, 55, 56, 56, 57, 57, 58, 59, 60, DM_CHROMA_IDX};
#else
  //0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, DM
  { 0, 1, 2, 2, 2, 2, 3, 5, 7, 8, 10, 12, 13, 15, 17, 18, 19, 20, 21, 22, 23, 23, 24, 24, 25, 25, 26, 27, 27, 28, 28, 29, 29, 30, 31, DM_CHROMA_IDX};
#endif

// ====================================================================================================================
// Misc.
// ====================================================================================================================

Char  g_aucConvertToBit  [ MAX_CU_SIZE+1 ];

#if ENC_DEC_TRACE
FILE*  g_hTrace = NULL; // Set to NULL to open up a file. Set to stdout to use the current output
const Bool g_bEncDecTraceEnable  = true;
const Bool g_bEncDecTraceDisable = false;
Bool   g_HLSTraceEnable = true;
Bool   g_bJustDoIt = false;
UInt64 g_nSymbolCounter = 0;
#endif

#if JVET_B0051_NON_MPM_MODE || JVET_C0038_GALF
UChar g_NonMPM[257] = { 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                          4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
                          5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                          6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                          6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7,
                          7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
                          7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
                          7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
                          7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
                          7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8};
#endif
// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table
#if COM16_C806_T64
UInt* g_scanOrder[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][ MAX_LOG2_TU_SIZE_PLUS_ONE ][ MAX_LOG2_TU_SIZE_PLUS_ONE ];
#else
UInt* g_scanOrder[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][ MAX_CU_DEPTH ][ MAX_CU_DEPTH ];
#endif

const UInt ctxIndMap4x4[4*4] =
{
  0, 1, 4, 5,
  2, 3, 4, 5,
  6, 6, 8, 8,
  7, 7, 8, 8
};

#if COM16_C806_T64
#if JVET_C0024_QTBT
const UInt g_uiMinInGroup[  LAST_SIGNIFICANT_GROUPS ] = {0,1,2,3,4,6,8,12,16,24,32,48,64,96};
const UInt g_uiGroupIdx  [  MAX_TU_SIZE ] = {0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11
,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12
,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13};
#else
const UInt g_uiMinInGroup[  LAST_SIGNIFICANT_GROUPS ] = {0,1,2,3,4,6,8,12,16,24,32,48};
const UInt g_uiGroupIdx  [  MAX_TU_SIZE ] = {0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11};
#endif
#else
const UInt g_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ] = {0,1,2,3,4,6,8,12,16,24};
const UInt g_uiGroupIdx[ MAX_TU_SIZE ]   = {0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9};
#endif

const Char *MatrixType[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
    "INTRA4X4_LUMA",
    "INTRA4X4_CHROMAU",
    "INTRA4X4_CHROMAV",
    "INTER4X4_LUMA",
    "INTER4X4_CHROMAU",
    "INTER4X4_CHROMAV"
  },
  {
    "INTRA8X8_LUMA",
    "INTRA8X8_CHROMAU",
    "INTRA8X8_CHROMAV",
    "INTER8X8_LUMA",
    "INTER8X8_CHROMAU",
    "INTER8X8_CHROMAV"
  },
  {
    "INTRA16X16_LUMA",
    "INTRA16X16_CHROMAU",
    "INTRA16X16_CHROMAV",
    "INTER16X16_LUMA",
    "INTER16X16_CHROMAU",
    "INTER16X16_CHROMAV"
  },
  {
   "INTRA32X32_LUMA",
   "INTRA32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTRA32X32_CHROMAV_FROM16x16_CHROMAV",
   "INTER32X32_LUMA",
   "INTER32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTER32X32_CHROMAV_FROM16x16_CHROMAV"
  },
};

const Char *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
  },
  {
  },
  {
    "INTRA16X16_LUMA_DC",
    "INTRA16X16_CHROMAU_DC",
    "INTRA16X16_CHROMAV_DC",
    "INTER16X16_LUMA_DC",
    "INTER16X16_CHROMAU_DC",
    "INTER16X16_CHROMAV_DC"
  },
  {
    "INTRA32X32_LUMA_DC",
    "INTRA32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTRA32X32_CHROMAV_DC_FROM16x16_CHROMAV",
    "INTER32X32_LUMA_DC",
    "INTER32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTER32X32_CHROMAV_DC_FROM16x16_CHROMAV"
  },
};

const Int g_quantTSDefault4x4[4*4] =
{
  16,16,16,16,
  16,16,16,16,
  16,16,16,16,
  16,16,16,16
};

const Int g_quantIntraDefault8x8[8*8] =
{
  16,16,16,16,17,18,21,24,
  16,16,16,16,17,19,22,25,
  16,16,17,18,20,22,25,29,
  16,16,18,21,24,27,31,36,
  17,17,20,24,30,35,41,47,
  18,19,22,27,35,44,54,65,
  21,22,25,31,41,54,70,88,
  24,25,29,36,47,65,88,115
};

const Int g_quantInterDefault8x8[8*8] =
{
  16,16,16,16,17,18,20,24,
  16,16,16,17,18,20,24,25,
  16,16,17,18,20,24,25,28,
  16,17,18,20,24,25,28,33,
  17,18,20,24,25,28,33,41,
  18,20,24,25,28,33,41,54,
  20,24,25,28,33,41,54,71,
  24,25,28,33,41,54,71,91
};

#if JVET_C0024_QTBT
const UInt g_scalingListSize   [SCALING_LIST_SIZE_NUM] = {4,16,64,256,1024,4096};
const UInt g_scalingListSizeX  [SCALING_LIST_SIZE_NUM] = {2, 4, 8, 16,  32, 64};
#else
const UInt g_scalingListSize   [SCALING_LIST_SIZE_NUM] = {16,64,256,1024
#if COM16_C806_T64
  , 4096
#endif
};
const UInt g_scalingListSizeX  [SCALING_LIST_SIZE_NUM] = { 4, 8, 16,  32
#if COM16_C806_T64
  , 64
#endif
};
#endif

#if COM16_C1046_PDPC_INTRA
#if JVET_C0024_QTBT // lossless change, just remove unused entries from the table
const Int g_pdpc_pred_param[5][35][6] = {
{ {  33,   7,  33,   7,  30,    3 },
  {  25,   5,  25,   5,   0,    0 },
  {  10,   8,  29,   4,  11,    1 },
  {  10,   8,  29,   4,  11,    1 },
  {  17,   5,  20,   5,  52,    1 },
  {  17,   5,  20,   5,  52,    1 },
  {  21,   3,  18,   7,  70,    2 },
  {  21,   3,  18,   7,  70,    2 },
  {  20,   1,  18,  11,  63,    2 },
  {  20,   1,  18,  11,  63,    2 },
  {  16,   1,  30,  24,  56,    1 },
  {  16,   1,  30,  24,  56,    1 },
  {  15,   0,  15,  14,  67,    3 },
  {  15,   0,  15,  14,  67,    3 },
  {  15,   2,   9,   2,  62,    1 },
  {  15,   2,   9,   2,  62,    1 },
  {  11,   4,  10,   2,  40,    1 },
  {  11,   4,  10,   2,  40,    1 },
  {   4,   3,   4,   3,  22,    1 },
  {  10,   2,  11,   4,  40,    1 },
  {  10,   2,  11,   4,  40,    1 },
  {   9,   2,  15,   2,  62,    1 },
  {   9,   2,  15,   2,  62,    1 },
  {  15,  14,  15,   0,  67,    3 },
  {  15,  14,  15,   0,  67,    3 },
  {  30,  24,  16,   1,  56,    1 },
  {  30,  24,  16,   1,  56,    1 },
  {  18,  11,  20,   1,  63,    2 },
  {  18,  11,  20,   1,  63,    2 },
  {  18,   7,  21,   3,  70,    2 },
  {  18,   7,  21,   3,  70,    2 },
  {  20,   5,  17,   5,  52,    1 },
  {  20,   5,  17,   5,  52,    1 },
  {  29,   4,  10,   8,  11,    1 },
  {  29,   4,  10,   8,  11,    1 } },
{ {  36,   7,  36,   7,  26,    3 },
  {  33,   8,  33,   8,   0,    0 },
  {  22,   7,  32,   6,  24,    3 },
  {  22,   7,  32,   6,  24,    3 },
  {  35,   4,  29,   8,  45,    2 },
  {  35,   4,  29,   8,  45,    2 },
  {  41,   3,  27,  12,  65,    3 },
  {  41,   3,  27,  12,  65,    3 },
  {  54,   1,  26,  16,  63,    2 },
  {  54,   1,  26,  16,  63,    2 },
  {  54,  -1,  34,  25,  52,    1 },
  {  54,  -1,  34,  25,  52,    1 },
  {  24,  -1,  21,  20,  62,    1 },
  {  24,  -1,  21,  20,  62,    1 },
  {  21,   3,  19,   3,  35,    1 },
  {  21,   3,  19,   3,  35,    1 },
  {  19,   4,  21,   3,  36,    2 },
  {  19,   4,  21,   3,  36,    2 },
  {  15,   6,  15,   6,  23,    2 },
  {  21,   3,  19,   4,  36,    2 },
  {  21,   3,  19,   4,  36,    2 },
  {  19,   3,  21,   3,  35,    1 },
  {  19,   3,  21,   3,  35,    1 },
  {  21,  20,  24,  -1,  62,    1 },
  {  21,  20,  24,  -1,  62,    1 },
  {  34,  25,  54,  -1,  52,    1 },
  {  34,  25,  54,  -1,  52,    1 },
  {  26,  16,  54,   1,  63,    2 },
  {  26,  16,  54,   1,  63,    2 },
  {  27,  12,  41,   3,  65,    3 },
  {  27,  12,  41,   3,  65,    3 },
  {  29,   8,  35,   4,  45,    2 },
  {  29,   8,  35,   4,  45,    2 },
  {  32,   6,  22,   7,  24,    3 },
  {  32,   6,  22,   7,  24,    3 } },
{ {  45,   5,  45,   5,  -5,    3 },
  {  36,   8,  36,   8,   0,    0 },
  {  30,   6,  46,   6, -15,    3 },
  {  30,   6,  46,   6, -15,    3 },
  {  31,   5,  39,   8,  15,    3 },
  {  31,   5,  39,   8,  15,    3 },
  {  35,   3,  35,  11,  42,    3 },
  {  35,   3,  35,  11,  42,    3 },
  {  45,   1,  35,  19,  46,    3 },
  {  45,   1,  35,  19,  46,    3 },
  {  32,   0,  40,  32,  47,    3 },
  {  32,   0,  40,  32,  47,    3 },
  {  38,   0,  23,  13,  38,    2 },
  {  38,   0,  23,  13,  38,    2 },
  {  26,   2,  24,   0,  28,    3 },
  {  26,   2,  24,   0,  28,    3 },
  {  25,   2,  23,   0,  19,    3 },
  {  25,   2,  23,   0,  19,    3 },
  {  29,   1,  29,   1,  -7,    3 },
  {  24,   0,  25,   2,  19,    3 },
  {  24,   0,  25,   2,  19,    3 },
  {  24,   0,  26,   2,  28,    3 },
  {  24,   0,  26,   2,  28,    3 },
  {  23,  13,  38,   0,  38,    2 },
  {  23,  13,  38,   0,  38,    2 },
  {  40,  32,  32,   0,  47,    3 },
  {  40,  32,  32,   0,  47,    3 },
  {  35,  19,  45,   1,  46,    3 },
  {  35,  19,  45,   1,  46,    3 },
  {  35,  11,  35,   3,  42,    3 },
  {  35,  11,  35,   3,  42,    3 },
  {  39,   8,  31,   5,  15,    3 },
  {  39,   8,  31,   5,  15,    3 },
  {  46,   6,  30,   6, -15,    3 },
  {  46,   6,  30,   6, -15,    3 } },
{ {  46,   6,  46,   6,  -3,    5 },
  {  44,   4,  44,   4,   0,    0 },
  {  33,   3,  52,   4, -18,    5 },
  {  33,   3,  52,   4, -18,    5 },
  {  38,   3,  50,   5,  -5,    5 },
  {  38,   3,  50,   5,  -5,    5 },
  {  40,   2,  47,   9,  16,    5 },
  {  40,   2,  47,   9,  16,    5 },
  {  48,   1,  45,  17,  22,    5 },
  {  48,   1,  45,  17,  22,    5 },
  {  45,  -1,  46,  30,  36,    5 },
  {  45,  -1,  46,  30,  36,    5 },
  {  41,   1,  37,  -1,  14,    5 },
  {  41,   1,  37,  -1,  14,    5 },
  {  35,   1,  39,  -2,   3,    5 },
  {  35,   1,  39,  -2,   3,    5 },
  {  41,  -1,  43,  -1,  -7,    5 },
  {  41,  -1,  43,  -1,  -7,    5 },
  {  32,   0,  32,   0,  -6,    5 },
  {  43,  -1,  41,  -1,  -7,    5 },
  {  43,  -1,  41,  -1,  -7,    5 },
  {  39,  -2,  35,   1,   3,    5 },
  {  39,  -2,  35,   1,   3,    5 },
  {  37,  -1,  41,   1,  14,    5 },
  {  37,  -1,  41,   1,  14,    5 },
  {  46,  30,  45,  -1,  36,    5 },
  {  46,  30,  45,  -1,  36,    5 },
  {  45,  17,  48,   1,  22,    5 },
  {  45,  17,  48,   1,  22,    5 },
  {  47,   9,  40,   2,  16,    5 },
  {  47,   9,  40,   2,  16,    5 },
  {  50,   5,  38,   3,  -5,    5 },
  {  50,   5,  38,   3,  -5,    5 },
  {  52,   4,  33,   3, -18,    5 },
  {  52,   4,  33,   3, -18,    5 } },
{ {  42,   5,  42,   5,  -5,    7 },
  {  40,   3,  40,   3,   0,    0 },
  {  28,   2,  49,   3, -22,    7 },
  {  28,   2,  49,   3, -22,    7 },
  {  27,   2,  48,   3, -16,    7 },
  {  27,   2,  48,   3, -16,    7 },
  {  27,   1,  44,   5,   8,    7 },
  {  27,   1,  44,   5,   8,    7 },
  {  41,   2,  39,  12,  16,    7 },
  {  41,   2,  39,  12,  16,    7 },
  {  42,   0,  38,  21,  24,    7 },
  {  42,   0,  38,  21,  24,    7 },
  {  38,   0,  34,  -4,   5,    7 },
  {  38,   0,  34,  -4,   5,    7 },
  {  37,   1,  43,  -1,  -5,    7 },
  {  37,   1,  43,  -1,  -5,    7 },
  {  25,   0,  42,  -1, -13,    7 },
  {  25,   0,  42,  -1, -13,    7 },
  {  27,  -1,  27,  -1, -11,    7 },
  {  42,  -1,  25,   0, -13,    7 },
  {  42,  -1,  25,   0, -13,    7 },
  {  43,  -1,  37,   1,  -5,    7 },
  {  43,  -1,  37,   1,  -5,    7 },
  {  34,  -4,  38,   0,   5,    7 },
  {  34,  -4,  38,   0,   5,    7 },
  {  38,  21,  42,   0,  24,    7 },
  {  38,  21,  42,   0,  24,    7 },
  {  39,  12,  41,   2,  16,    7 },
  {  39,  12,  41,   2,  16,    7 },
  {  44,   5,  27,   1,   8,    7 },
  {  44,   5,  27,   1,   8,    7 },
  {  48,   3,  27,   2, -16,    7 },
  {  48,   3,  27,   2, -16,    7 },
  {  49,   3,  28,   2, -22,    7 },
  {  49,   3,  28,   2, -22,    7 } } };
#else
const Int g_pdpc_pred_param[5][2][35][7] =
{ { { {   27,   10,   27,   10,   29,    3,    0, },
{   22,    9,   22,    9,    0,    0,    0, },
{  -10,    7,   22,    1,   24,    1,    0, },
{  -10,    7,   22,    1,   24,    1,    0, },
{   -5,    4,   10,    1,   68,    3,    0, },
{   -5,    4,   10,    1,   68,    3,    0, },
{   -8,    3,    7,    2,   77,    2,    0, },
{   -8,    3,    7,    2,   77,    2,    0, },
{  -48,    1,    8,    6,   74,    2,    0, },
{  -48,    1,    8,    6,   74,    2,    0, },
{   20,    1,   25,   25,   63,    2,    0, },
{   20,    1,   25,   25,   63,    2,    0, },
{   14,   -1,    5,    9,   76,    3,    0, },
{   14,   -1,    5,    9,   76,    3,    0, },
{   10,    1,    1,    3,   77,    3,    0, },
{   10,    1,    1,    3,   77,    3,    0, },
{    6,    2,    2,    1,   75,    3,    0, },
{    6,    2,    2,    1,   75,    3,    0, },
{   -1,    2,   -1,    2,   37,    1,    0, },
{    2,    1,    6,    2,   75,    3,    0, },
{    2,    1,    6,    2,   75,    3,    0, },
{    1,    3,   10,    1,   77,    3,    0, },
{    1,    3,   10,    1,   77,    3,    0, },
{    5,    9,   14,   -1,   76,    3,    0, },
{    5,    9,   14,   -1,   76,    3,    0, },
{   25,   25,   20,    1,   63,    2,    0, },
{   25,   25,   20,    1,   63,    2,    0, },
{    8,    6,  -48,    1,   74,    2,    0, },
{    8,    6,  -48,    1,   74,    2,    0, },
{    7,    2,   -8,    3,   77,    2,    0, },
{    7,    2,   -8,    3,   77,    2,    0, },
{   10,    1,   -5,    4,   68,    3,    0, },
{   10,    1,   -5,    4,   68,    3,    0, },
{   22,    1,  -10,    7,   24,    1,    0, },
{   22,    1,  -10,    7,   24,    1,    0, },
},
{ {   33,    7,   33,    7,   30,    3,    0, },
{   25,    5,   25,    5,    0,    0,    0, },
{   10,    8,   29,    4,   11,    1,    0, },
{   10,    8,   29,    4,   11,    1,    0, },
{   17,    5,   20,    5,   52,    1,    0, },
{   17,    5,   20,    5,   52,    1,    0, },
{   21,    3,   18,    7,   70,    2,    0, },
{   21,    3,   18,    7,   70,    2,    0, },
{   20,    1,   18,   11,   63,    2,    0, },
{   20,    1,   18,   11,   63,    2,    0, },
{   16,    1,   30,   24,   56,    1,    0, },
{   16,    1,   30,   24,   56,    1,    0, },
{   15,    0,   15,   14,   67,    3,    0, },
{   15,    0,   15,   14,   67,    3,    0, },
{   15,    2,    9,    2,   62,    1,    0, },
{   15,    2,    9,    2,   62,    1,    0, },
{   11,    4,   10,    2,   40,    1,    0, },
{   11,    4,   10,    2,   40,    1,    0, },
{    4,    3,    4,    3,   22,    1,    0, },
{   10,    2,   11,    4,   40,    1,    0, },
{   10,    2,   11,    4,   40,    1,    0, },
{    9,    2,   15,    2,   62,    1,    0, },
{    9,    2,   15,    2,   62,    1,    0, },
{   15,   14,   15,    0,   67,    3,    0, },
{   15,   14,   15,    0,   67,    3,    0, },
{   30,   24,   16,    1,   56,    1,    0, },
{   30,   24,   16,    1,   56,    1,    0, },
{   18,   11,   20,    1,   63,    2,    0, },
{   18,   11,   20,    1,   63,    2,    0, },
{   18,    7,   21,    3,   70,    2,    0, },
{   18,    7,   21,    3,   70,    2,    0, },
{   20,    5,   17,    5,   52,    1,    0, },
{   20,    5,   17,    5,   52,    1,    0, },
{   29,    4,   10,    8,   11,    1,    0, },
{   29,    4,   10,    8,   11,    1,    0, },
},
},
{ { {   14,   10,   14,   10,   46,    3,    0, },
{   18,    9,   18,    9,    0,    0,    0, },
{  -14,    5,   12,    1,   20,    1,    0, },
{  -14,    5,   12,    1,   20,    1,    0, },
{   -5,    4,    7,    1,   73,    3,    0, },
{   -5,    4,    7,    1,   73,    3,    0, },
{   -6,    3,    7,    3,   80,    2,    0, },
{   -6,    3,    7,    3,   80,    2,    0, },
{  -48,    1,   11,    8,   74,    2,    0, },
{  -48,    1,   11,    8,   74,    2,    0, },
{    2,    1,   25,   25,   65,    3,    0, },
{    2,    1,   25,   25,   65,    3,    0, },
{    5,   -1,    5,    9,   77,    3,    0, },
{    5,   -1,    5,    9,   77,    3,    0, },
{    3,    0,    0,    2,   78,    3,    0, },
{    3,    0,    0,    2,   78,    3,    0, },
{    2,    1,    0,    0,   77,    3,    0, },
{    2,    1,    0,    0,   77,    3,    0, },
{   -2,    1,   -2,    2,   44,    1,    0, },
{    0,    0,    2,    1,   77,    3,    0, },
{    0,    0,    2,    1,   77,    3,    0, },
{    0,    2,    3,    0,   78,    3,    0, },
{    0,    2,    3,    0,   78,    3,    0, },
{    5,    9,    5,   -1,   77,    3,    0, },
{    5,    9,    5,   -1,   77,    3,    0, },
{   25,   25,    2,    1,   65,    3,    0, },
{   25,   25,    2,    1,   65,    3,    0, },
{   11,    8,  -48,    1,   74,    2,    0, },
{   11,    8,  -48,    1,   74,    2,    0, },
{    7,    3,   -6,    3,   80,    2,    0, },
{    7,    3,   -6,    3,   80,    2,    0, },
{    7,    1,   -5,    4,   73,    3,    0, },
{    7,    1,   -5,    4,   73,    3,    0, },
{   12,    1,  -14,    5,   20,    1,    0, },
{   12,    1,  -14,    5,   20,    1,    0, },
},
{ {   36,    7,   36,    7,   26,    3,    0, },
{   33,    8,   33,    8,    0,    0,    0, },
{   22,    7,   32,    6,   24,    3,    0, },
{   22,    7,   32,    6,   24,    3,    0, },
{   35,    4,   29,    8,   45,    2,    0, },
{   35,    4,   29,    8,   45,    2,    0, },
{   41,    3,   27,   12,   65,    3,    0, },
{   41,    3,   27,   12,   65,    3,    0, },
{   54,    1,   26,   16,   63,    2,    0, },
{   54,    1,   26,   16,   63,    2,    0, },
{   54,   -1,   34,   25,   52,    1,    0, },
{   54,   -1,   34,   25,   52,    1,    0, },
{   24,   -1,   21,   20,   62,    1,    0, },
{   24,   -1,   21,   20,   62,    1,    0, },
{   21,    3,   19,    3,   35,    1,    0, },
{   21,    3,   19,    3,   35,    1,    0, },
{   19,    4,   21,    3,   36,    2,    0, },
{   19,    4,   21,    3,   36,    2,    0, },
{   15,    6,   15,    6,   23,    2,    0, },
{   21,    3,   19,    4,   36,    2,    0, },
{   21,    3,   19,    4,   36,    2,    0, },
{   19,    3,   21,    3,   35,    1,    0, },
{   19,    3,   21,    3,   35,    1,    0, },
{   21,   20,   24,   -1,   62,    1,    0, },
{   21,   20,   24,   -1,   62,    1,    0, },
{   34,   25,   54,   -1,   52,    1,    0, },
{   34,   25,   54,   -1,   52,    1,    0, },
{   26,   16,   54,    1,   63,    2,    0, },
{   26,   16,   54,    1,   63,    2,    0, },
{   27,   12,   41,    3,   65,    3,    0, },
{   27,   12,   41,    3,   65,    3,    0, },
{   29,    8,   35,    4,   45,    2,    0, },
{   29,    8,   35,    4,   45,    2,    0, },
{   32,    6,   22,    7,   24,    3,    0, },
{   32,    6,   22,    7,   24,    3,    0, },
},
},
{ { {   23,   13,   23,   13,   14,    3,    0, },
{   23,   18,   23,   18,    0,    0,    0, },
{    0,    6,   35,    6,    3,    3,    0, },
{    0,    6,   35,    6,    3,    3,    0, },
{   -6,    3,   18,    4,   53,    2,    0, },
{   -6,    3,   18,    4,   53,    2,    0, },
{  -10,    2,    9,    3,   67,    3,    0, },
{  -10,    2,    9,    3,   67,    3,    0, },
{  -60,    1,   13,    9,   68,    1,    0, },
{  -60,    1,   13,    9,   68,    1,    0, },
{   55,    0,   33,   33,   56,    1,    0, },
{   55,    0,   33,   33,   56,    1,    0, },
{   34,    0,    7,   10,   57,    1,    0, },
{   34,    0,    7,   10,   57,    1,    0, },
{   14,    2,    1,    2,   55,    1,    0, },
{   14,    2,    1,    2,   55,    1,    0, },
{    6,    3,    0,    1,   46,    1,    0, },
{    6,    3,    0,    1,   46,    1,    0, },
{    7,    3,    7,    3,   18,    3,    0, },
{    0,    1,    6,    3,   46,    1,    0, },
{    0,    1,    6,    3,   46,    1,    0, },
{    1,    2,   14,    2,   55,    1,    0, },
{    1,    2,   14,    2,   55,    1,    0, },
{    7,   10,   34,    0,   57,    1,    0, },
{    7,   10,   34,    0,   57,    1,    0, },
{   33,   33,   55,    0,   56,    1,    0, },
{   33,   33,   55,    0,   56,    1,    0, },
{   13,    9,  -60,    1,   68,    1,    0, },
{   13,    9,  -60,    1,   68,    1,    0, },
{    9,    3,  -10,    2,   67,    3,    0, },
{    9,    3,  -10,    2,   67,    3,    0, },
{   18,    4,   -6,    3,   53,    2,    0, },
{   18,    4,   -6,    3,   53,    2,    0, },
{   35,    6,    0,    6,    3,    3,    0, },
{   35,    6,    0,    6,    3,    3,    0, },
},
{ {   45,    5,   45,    5,   -5,    3,    0, },
{   36,    8,   36,    8,    0,    0,    0, },
{   30,    6,   46,    6,  -15,    3,    0, },
{   30,    6,   46,    6,  -15,    3,    0, },
{   31,    5,   39,    8,   15,    3,    0, },
{   31,    5,   39,    8,   15,    3,    0, },
{   35,    3,   35,   11,   42,    3,    0, },
{   35,    3,   35,   11,   42,    3,    0, },
{   45,    1,   35,   19,   46,    3,    0, },
{   45,    1,   35,   19,   46,    3,    0, },
{   32,    0,   40,   32,   47,    3,    0, },
{   32,    0,   40,   32,   47,    3,    0, },
{   38,    0,   23,   13,   38,    2,    0, },
{   38,    0,   23,   13,   38,    2,    0, },
{   26,    2,   24,    0,   28,    3,    0, },
{   26,    2,   24,    0,   28,    3,    0, },
{   25,    2,   23,    0,   19,    3,    0, },
{   25,    2,   23,    0,   19,    3,    0, },
{   29,    1,   29,    1,   -7,    3,    0, },
{   24,    0,   25,    2,   19,    3,    0, },
{   24,    0,   25,    2,   19,    3,    0, },
{   24,    0,   26,    2,   28,    3,    0, },
{   24,    0,   26,    2,   28,    3,    0, },
{   23,   13,   38,    0,   38,    2,    0, },
{   23,   13,   38,    0,   38,    2,    0, },
{   40,   32,   32,    0,   47,    3,    0, },
{   40,   32,   32,    0,   47,    3,    0, },
{   35,   19,   45,    1,   46,    3,    0, },
{   35,   19,   45,    1,   46,    3,    0, },
{   35,   11,   35,    3,   42,    3,    0, },
{   35,   11,   35,    3,   42,    3,    0, },
{   39,    8,   31,    5,   15,    3,    0, },
{   39,    8,   31,    5,   15,    3,    0, },
{   46,    6,   30,    6,  -15,    3,    0, },
{   46,    6,   30,    6,  -15,    3,    0, },
},
},
{ { {    5,   21,    5,   21,   25,    5,    0, },
{   21,   32,   21,   32,    0,    0,    0, },
{    1,    4,   45,    7,   -4,    5,    0, },
{    1,    4,   45,    7,   -4,    5,    0, },
{    2,    2,   23,    5,   27,    5,    0, },
{    2,    2,   23,    5,   27,    5,    0, },
{   -3,    2,   21,    6,   44,    5,    0, },
{   -3,    2,   21,    6,   44,    5,    0, },
{    2,    2,   23,   15,   42,    3,    0, },
{    2,    2,   23,   15,   42,    3,    0, },
{   36,    0,   49,   49,   42,    1,    0, },
{   36,    0,   49,   49,   42,    1,    0, },
{   44,    1,   11,    6,   26,    3,    0, },
{   44,    1,   11,    6,   26,    3,    0, },
{   25,    2,   11,    2,   27,    5,    0, },
{   25,    2,   11,    2,   27,    5,    0, },
{    4,    3,    0,    4,   24,    5,    0, },
{    4,    3,    0,    4,   24,    5,    0, },
{    2,    3,    2,    3,   18,    5,    0, },
{    0,    4,    4,    3,   24,    5,    0, },
{    0,    4,    4,    3,   24,    5,    0, },
{   11,    2,   25,    2,   27,    5,    0, },
{   11,    2,   25,    2,   27,    5,    0, },
{   11,    6,   44,    1,   26,    3,    0, },
{   11,    6,   44,    1,   26,    3,    0, },
{   49,   49,   36,    0,   42,    1,    0, },
{   49,   49,   36,    0,   42,    1,    0, },
{   23,   15,    2,    2,   42,    3,    0, },
{   23,   15,    2,    2,   42,    3,    0, },
{   21,    6,   -3,    2,   44,    5,    0, },
{   21,    6,   -3,    2,   44,    5,    0, },
{   23,    5,    2,    2,   27,    5,    0, },
{   23,    5,    2,    2,   27,    5,    0, },
{   45,    7,    1,    4,   -4,    5,    0, },
{   45,    7,    1,    4,   -4,    5,    0, },
},
{ {   46,    6,   46,    6,   -3,    5,    0, },
{   44,    4,   44,    4,    0,    0,    0, },
{   33,    3,   52,    4,  -18,    5,    0, },
{   33,    3,   52,    4,  -18,    5,    0, },
{   38,    3,   50,    5,   -5,    5,    0, },
{   38,    3,   50,    5,   -5,    5,    0, },
{   40,    2,   47,    9,   16,    5,    0, },
{   40,    2,   47,    9,   16,    5,    0, },
{   48,    1,   45,   17,   22,    5,    0, },
{   48,    1,   45,   17,   22,    5,    0, },
{   45,   -1,   46,   30,   36,    5,    0, },
{   45,   -1,   46,   30,   36,    5,    0, },
{   41,    1,   37,   -1,   14,    5,    0, },
{   41,    1,   37,   -1,   14,    5,    0, },
{   35,    1,   39,   -2,    3,    5,    0, },
{   35,    1,   39,   -2,    3,    5,    0, },
{   41,   -1,   43,   -1,   -7,    5,    0, },
{   41,   -1,   43,   -1,   -7,    5,    0, },
{   32,    0,   32,    0,   -6,    5,    0, },
{   43,   -1,   41,   -1,   -7,    5,    0, },
{   43,   -1,   41,   -1,   -7,    5,    0, },
{   39,   -2,   35,    1,    3,    5,    0, },
{   39,   -2,   35,    1,    3,    5,    0, },
{   37,   -1,   41,    1,   14,    5,    0, },
{   37,   -1,   41,    1,   14,    5,    0, },
{   46,   30,   45,   -1,   36,    5,    0, },
{   46,   30,   45,   -1,   36,    5,    0, },
{   45,   17,   48,    1,   22,    5,    0, },
{   45,   17,   48,    1,   22,    5,    0, },
{   47,    9,   40,    2,   16,    5,    0, },
{   47,    9,   40,    2,   16,    5,    0, },
{   50,    5,   38,    3,   -5,    5,    0, },
{   50,    5,   38,    3,   -5,    5,    0, },
{   52,    4,   33,    3,  -18,    5,    0, },
{   52,    4,   33,    3,  -18,    5,    0, },
},
},
{ { {  -39,   32,  -39,   32,   44,    7,   10, },
{    6,   58,    6,   58,    0,    0,   10, },
{    0,    3,   55,   18,   -8,    7,   10, },
{    0,    3,   55,   18,   -8,    7,   10, },
{   -2,    2,   22,    5,   16,    7,   10, },
{   -2,    2,   22,    5,   16,    7,   10, },
{   -9,    1,   21,    5,   39,    7,   10, },
{   -9,    1,   21,    5,   39,    7,   10, },
{    3,    2,   14,    6,   18,    3,   10, },
{    3,    2,   14,    6,   18,    3,   10, },
{   10,    0,   63,   66,   49,    7,   10, },
{   10,    0,   63,   66,   49,    7,   10, },
{   38,    1,   12,    4,   25,    7,   10, },
{   38,    1,   12,    4,   25,    7,   10, },
{   23,    2,   14,    3,   26,    7,   10, },
{   23,    2,   14,    3,   26,    7,   10, },
{    6,    3,   14,    3,   11,    7,   10, },
{    6,    3,   14,    3,   11,    7,   10, },
{    3,    5,    3,    5,    4,    7,   10, },
{   14,    3,    6,    3,   11,    7,   10, },
{   14,    3,    6,    3,   11,    7,   10, },
{   14,    3,   23,    2,   26,    7,   10, },
{   14,    3,   23,    2,   26,    7,   10, },
{   12,    4,   38,    1,   25,    7,   10, },
{   12,    4,   38,    1,   25,    7,   10, },
{   63,   66,   10,    0,   49,    7,   10, },
{   63,   66,   10,    0,   49,    7,   10, },
{   14,    6,    3,    2,   18,    3,   10, },
{   14,    6,    3,    2,   18,    3,   10, },
{   21,    5,   -9,    1,   39,    7,   10, },
{   21,    5,   -9,    1,   39,    7,   10, },
{   22,    5,   -2,    2,   16,    7,   10, },
{   22,    5,   -2,    2,   16,    7,   10, },
{   55,   18,    0,    3,   -8,    7,   10, },
{   55,   18,    0,    3,   -8,    7,   10, },
},
{ {   42,    5,   42,    5,   -5,    7,   10, },
{   40,    3,   40,    3,    0,    0,   10, },
{   28,    2,   49,    3,  -22,    7,   10, },
{   28,    2,   49,    3,  -22,    7,   10, },
{   27,    2,   48,    3,  -16,    7,   10, },
{   27,    2,   48,    3,  -16,    7,   10, },
{   27,    1,   44,    5,    8,    7,   10, },
{   27,    1,   44,    5,    8,    7,   10, },
{   41,    2,   39,   12,   16,    7,   10, },
{   41,    2,   39,   12,   16,    7,   10, },
{   42,    0,   38,   21,   24,    7,   10, },
{   42,    0,   38,   21,   24,    7,   10, },
{   38,    0,   34,   -4,    5,    7,   10, },
{   38,    0,   34,   -4,    5,    7,   10, },
{   37,    1,   43,   -1,   -5,    7,   10, },
{   37,    1,   43,   -1,   -5,    7,   10, },
{   25,    0,   42,   -1,  -13,    7,   10, },
{   25,    0,   42,   -1,  -13,    7,   10, },
{   27,   -1,   27,   -1,  -11,    7,   10, },
{   42,   -1,   25,    0,  -13,    7,   10, },
{   42,   -1,   25,    0,  -13,    7,   10, },
{   43,   -1,   37,    1,   -5,    7,   10, },
{   43,   -1,   37,    1,   -5,    7,   10, },
{   34,   -4,   38,    0,    5,    7,   10, },
{   34,   -4,   38,    0,    5,    7,   10, },
{   38,   21,   42,    0,   24,    7,   10, },
{   38,   21,   42,    0,   24,    7,   10, },
{   39,   12,   41,    2,   16,    7,   10, },
{   39,   12,   41,    2,   16,    7,   10, },
{   44,    5,   27,    1,    8,    7,   10, },
{   44,    5,   27,    1,    8,    7,   10, },
{   48,    3,   27,    2,  -16,    7,   10, },
{   48,    3,   27,    2,  -16,    7,   10, },
{   49,    3,   28,    2,  -22,    7,   10, },
{   49,    3,   28,    2,  -22,    7,   10, },
},
},
};
#endif


#endif

#if IDCC_GENERALIZED_BI_PRED
const Char g_iGbiLog2WeightBase = 3;
const Char g_iGbiWeightBase     = ( 1 << g_iGbiLog2WeightBase );
#if IDCC_GENERALIZED_BI_PRED == 1
const Char g_aiGbiWeights     [GBI_NUM] = { 3, 4, 5 };
const Char g_aiGbiSearchOrder [GBI_NUM] = { GBI_DEFAULT, GBI_DEFAULT-1, GBI_DEFAULT+1 };
#elif IDCC_GENERALIZED_BI_PRED == 2
const Char g_aiGbiWeights     [GBI_NUM] = { 2, 3, 4, 5, 6 };
const Char g_aiGbiSearchOrder [GBI_NUM] = { GBI_DEFAULT, GBI_DEFAULT-2, GBI_DEFAULT+2, GBI_DEFAULT-1, GBI_DEFAULT+1 };
#else
const Char g_aiGbiWeights     [GBI_NUM] = { -2, 2, 3, 4, 5, 6, 10 };
const Char g_aiGbiSearchOrder [GBI_NUM] = { GBI_DEFAULT, GBI_DEFAULT-2, GBI_DEFAULT+2, GBI_DEFAULT-1, GBI_DEFAULT+1, GBI_DEFAULT-3, GBI_DEFAULT+3 };
#endif
      Char g_aiGbiCodingOrder [GBI_NUM];
      Char g_aiGbiParsingOrder[GBI_NUM];

Char getGbiWeight( UChar uhGbiIdx, UChar uhRefFrmList )
{
  // Weghts for the model: P0 + w * (P1 - P0) = (1-w) * P0 + w * P1
  // Retuning  1-w for P0 or w for P1
  return ( uhRefFrmList == REF_PIC_LIST_0 ? g_iGbiWeightBase - g_aiGbiWeights[uhGbiIdx] : g_aiGbiWeights[uhGbiIdx] );
}

#include "TComDataCU.h"
#include "TComPic.h"
Void resetGbiCodingOrder( Bool bRunDecoding, TComDataCU* pcCU )
{
  UChar uhWeightSetIdx = ( !pcCU->getSlice()->getMvdL1ZeroFlag() ? 0 : 1 );
  UInt  uiQp           = pcCU->getPic()->getSlice(0)->getSliceQp() < 0 ? 0 : ( pcCU->getPic()->getSlice(0)->getSliceQp() > 52 ? 52 : pcCU->getPic()->getSlice(0)->getSliceQp() );
  resetGbiCodingOrder( bRunDecoding, uhWeightSetIdx, uiQp );
}

Void resetGbiCodingOrder( Bool bRunDecoding, UChar uhWeightSetIdx, UInt uiQp )
{
  // Form parsing order
  if( uhWeightSetIdx == 0 )
  { // { GBI_DEFAULT, GBI_DEFAULT+1, GBI_DEFAULT-1, GBI_DEFAULT+2, GBI_DEFAULT-2, ... }
    g_aiGbiParsingOrder[0] = GBI_DEFAULT;
    for( UInt i = 1 ; i <= (GBI_NUM>>1) ; ++i )
    {
      g_aiGbiParsingOrder[2*i-1] = GBI_DEFAULT + (Char)i;
      g_aiGbiParsingOrder[2*i  ] = GBI_DEFAULT - (Char)i;
    }
  }
  else if( uhWeightSetIdx == 1 )
  { // { (GBI_DEFAULT+1), (GBI_DEFAULT+1)-1, (GBI_DEFAULT+1)+1, (GBI_DEFAULT+1)-2, (GBI_DEFAULT+1)+2, ... }
    g_aiGbiParsingOrder[0] = GBI_DEFAULT + 1;
    for( UInt i = 1 ; i < (GBI_NUM>>1) ; ++i )
    {
      g_aiGbiParsingOrder[2*i-1] = ( GBI_DEFAULT + 1 ) - (Char)i;
      g_aiGbiParsingOrder[2*i  ] = ( GBI_DEFAULT + 1 ) + (Char)i;
    }
    g_aiGbiParsingOrder[GBI_NUM-2] = ( GBI_DEFAULT + 1 ) - (GBI_NUM>>1);
    g_aiGbiParsingOrder[GBI_NUM-1] = ( GBI_DEFAULT + 1 ) - (GBI_NUM>>1) - 1;
  }
  else
  {
    assert( 0 );
  }

  // Form encoding order
  if( !bRunDecoding )
  {
    for( UInt i = 0 ; i < GBI_NUM ; ++i )
    {
      g_aiGbiCodingOrder[(UInt)g_aiGbiParsingOrder[i]] = i;
    }
  }
}

UInt deriveWeightIdxBits( UChar uhGbiIdx ) // Note: align this with TEncSbac::codeGbiIdx and TDecSbac::parseGbiIdx
{
  UInt uiBits = 1;
  UChar uhGbiCodingIdx = (UChar)g_aiGbiCodingOrder[uhGbiIdx];

  if( GBI_NUM > 2 && uhGbiCodingIdx != 0 )
  {
    UInt  uiPrefixNumBits = GBI_NUM - 2;
    UInt  uiStep          = 1;
    UChar uhPrefixSymbol  = uhGbiCodingIdx;

    // Truncated unary code
    UChar uhIdx = 1;
    for( UInt ui = 0 ; ui < uiPrefixNumBits ; ++ui )
    {
      if( uhPrefixSymbol == uhIdx )
      {
        ++uiBits;
        break;
      }
      else
      {
        ++uiBits;
        uhIdx += uiStep;
      }
    }
  }

  return uiBits;
}

Int (*g_apIntMultiplier[])(Pel) =  // NOTE: Make sure these operators cover all the weights specified in g_aiGbiWeights
{
  &integerScalor< -2>, &integerScalor< -1>,
  &integerScalor<  0>, &integerScalor<  1>, &integerScalor<  2>, &integerScalor<  3>, &integerScalor<  4>,
  &integerScalor<  5>, &integerScalor<  6>, &integerScalor<  7>, &integerScalor<  8>, &integerScalor<  9>,
  &integerScalor< 10>
};
template<Int n> Int integerScalor( Pel p ) { return p * n; }
#endif

//! \}