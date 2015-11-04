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

/** \file     TComRdCost.cpp
    \brief    RD cost computation class
*/

#include <math.h>
#include <assert.h>
#include "TComRom.h"
#include "TComRdCost.h"

//! \ingroup TLibCommon
//! \{

TComRdCost::TComRdCost()
{
  init();
}

TComRdCost::~TComRdCost()
{
#if !FIX203
  xUninit();
#endif
}

// Calculate RD functions
Double TComRdCost::calcRdCost( UInt uiBits, UInt uiDistortion, Bool bFlag, DFunc eDFunc )
{
  Double dRdCost = 0.0;
  Double dLambda = 0.0;
  
  switch ( eDFunc )
  {
    case DF_SSE:
      assert(0);
      break;
    case DF_SAD:
      dLambda = (Double)m_uiLambdaMotionSAD;
      break;
    case DF_DEFAULT:
      dLambda =         m_dLambda;
      break;
    case DF_SSE_FRAME:
      dLambda =         m_dFrameLambda;
      break;
    default:
      assert (0);
      break;
  }
  
  if (bFlag)
  {
    // Intra8x8, Intra4x4 Block only...
#if SEQUENCE_LEVEL_LOSSLESS
    dRdCost = (Double)(uiBits);
#else
    dRdCost = (((Double)uiDistortion) + ((Double)uiBits * dLambda));
#endif
  }
  else
  {
    if (eDFunc == DF_SAD)
    {
      dRdCost = ((Double)uiDistortion + (Double)((Int)(uiBits * dLambda+.5)>>16));
      dRdCost = (Double)(UInt)floor(dRdCost);
    }
    else
    {
#if SEQUENCE_LEVEL_LOSSLESS
      dRdCost = (Double)(uiBits);
#else
      dRdCost = ((Double)uiDistortion + (Double)((Int)(uiBits * dLambda+.5)));
      dRdCost = (Double)(UInt)floor(dRdCost);
#endif
    }
  }
  
  return dRdCost;
}

Double TComRdCost::calcRdCost64( UInt64 uiBits, UInt64 uiDistortion, Bool bFlag, DFunc eDFunc )
{
  Double dRdCost = 0.0;
  Double dLambda = 0.0;
  
  switch ( eDFunc )
  {
    case DF_SSE:
      assert(0);
      break;
    case DF_SAD:
      dLambda = (Double)m_uiLambdaMotionSAD;
      break;
    case DF_DEFAULT:
      dLambda =         m_dLambda;
      break;
    case DF_SSE_FRAME:
      dLambda =         m_dFrameLambda;
      break;
    default:
      assert (0);
      break;
  }
  
  if (bFlag)
  {
    // Intra8x8, Intra4x4 Block only...
#if SEQUENCE_LEVEL_LOSSLESS
    dRdCost = (Double)(uiBits);
#else
    dRdCost = (((Double)(Int64)uiDistortion) + ((Double)(Int64)uiBits * dLambda));
#endif
  }
  else
  {
    if (eDFunc == DF_SAD)
    {
      dRdCost = ((Double)(Int64)uiDistortion + (Double)((Int)((Int64)uiBits * dLambda+.5)>>16));
      dRdCost = (Double)(UInt)floor(dRdCost);
    }
    else
    {
#if SEQUENCE_LEVEL_LOSSLESS
      dRdCost = (Double)(uiBits);
#else
      dRdCost = ((Double)(Int64)uiDistortion + (Double)((Int)((Int64)uiBits * dLambda+.5)));
      dRdCost = (Double)(UInt)floor(dRdCost);
#endif
    }
  }
  
  return dRdCost;
}

Void TComRdCost::setLambda( Double dLambda )
{
  m_dLambda           = dLambda;
  m_sqrtLambda        = sqrt(m_dLambda);
  m_uiLambdaMotionSAD = (UInt)floor(65536.0 * m_sqrtLambda);
  m_uiLambdaMotionSSE = (UInt)floor(65536.0 * m_dLambda   );
}


// Initalize Function Pointer by [eDFunc]
Void TComRdCost::init()
{
#if QT_BT_STRUCTURE
  m_afpDistortFunc[DF_DEFAULT]  = NULL;                  // for DF_DEFAULT

  m_afpDistortFunc[DF_SSE]  = TComRdCost::xGetSSE;
  m_afpDistortFunc[DF_SSE4]  = TComRdCost::xGetSSE4;
  m_afpDistortFunc[DF_SSE8]  = TComRdCost::xGetSSE8;
  m_afpDistortFunc[DF_SSE16]  = TComRdCost::xGetSSE16;
  m_afpDistortFunc[DF_SSE32]  = TComRdCost::xGetSSE32;
  m_afpDistortFunc[DF_SSE64]  = TComRdCost::xGetSSE64;
  m_afpDistortFunc[DF_SSE128]  = TComRdCost::xGetSSE128;
  m_afpDistortFunc[DF_SSE256]  = TComRdCost::xGetSSE256; 
  m_afpDistortFunc[DF_SSE16N]  = TComRdCost::xGetSSE16N;

  m_afpDistortFunc[DF_SAD]  = TComRdCost::xGetSAD;
  m_afpDistortFunc[DF_SAD4]  = TComRdCost::xGetSAD4;
  m_afpDistortFunc[DF_SAD8] = TComRdCost::xGetSAD8;
  m_afpDistortFunc[DF_SAD16] = TComRdCost::xGetSAD16;
  m_afpDistortFunc[DF_SAD32] = TComRdCost::xGetSAD32;
  m_afpDistortFunc[DF_SAD64] = TComRdCost::xGetSAD64;
  m_afpDistortFunc[DF_SAD128] = TComRdCost::xGetSAD128;
  m_afpDistortFunc[DF_SAD256] = TComRdCost::xGetSAD256;
  m_afpDistortFunc[DF_SAD16N] = TComRdCost::xGetSAD16N;

  m_afpDistortFunc[DF_SADS] = TComRdCost::xGetSAD;
  m_afpDistortFunc[DF_SADS4] = TComRdCost::xGetSAD4;
  m_afpDistortFunc[DF_SADS8] = TComRdCost::xGetSAD8;
  m_afpDistortFunc[DF_SADS16] = TComRdCost::xGetSAD16;
  m_afpDistortFunc[DF_SADS32] = TComRdCost::xGetSAD32;
  m_afpDistortFunc[DF_SADS64] = TComRdCost::xGetSAD64;
  m_afpDistortFunc[DF_SADS128] = TComRdCost::xGetSAD128; 
  m_afpDistortFunc[DF_SADS256] = TComRdCost::xGetSAD256;
  m_afpDistortFunc[DF_SADS16N] = TComRdCost::xGetSAD16N;

  m_afpDistortFunc[DF_HADS] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS4] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS8] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS16] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS32] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS64] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS128] = TComRdCost::xGetHADs; 
  m_afpDistortFunc[DF_HADS256] = TComRdCost::xGetHADs; 
  m_afpDistortFunc[DF_HADS16N] = TComRdCost::xGetHADs;


  m_afpDistortFunc[DF_SAD12] = TComRdCost::xGetSAD12;
  m_afpDistortFunc[DF_SAD24] = TComRdCost::xGetSAD24;
  m_afpDistortFunc[DF_SAD48] = TComRdCost::xGetSAD48;
  m_afpDistortFunc[DF_SAD96] = TComRdCost::xGetSAD96; 
  m_afpDistortFunc[DF_SAD192] = TComRdCost::xGetSAD192;

  m_afpDistortFunc[DF_SADS12] = TComRdCost::xGetSAD12;
  m_afpDistortFunc[DF_SADS24] = TComRdCost::xGetSAD24;
  m_afpDistortFunc[DF_SADS48] = TComRdCost::xGetSAD48;
  m_afpDistortFunc[DF_SADS96] = TComRdCost::xGetSAD96; 
  m_afpDistortFunc[DF_SADS192] = TComRdCost::xGetSAD192; 

#else
  m_afpDistortFunc[0]  = NULL;                  // for DF_DEFAULT
  
  m_afpDistortFunc[1]  = TComRdCost::xGetSSE;
  m_afpDistortFunc[2]  = TComRdCost::xGetSSE4;
  m_afpDistortFunc[3]  = TComRdCost::xGetSSE8;
  m_afpDistortFunc[4]  = TComRdCost::xGetSSE16;
  m_afpDistortFunc[5]  = TComRdCost::xGetSSE32;
  m_afpDistortFunc[6]  = TComRdCost::xGetSSE64;
  m_afpDistortFunc[7]  = TComRdCost::xGetSSE16N;
  
  m_afpDistortFunc[8]  = TComRdCost::xGetSAD;
  m_afpDistortFunc[9]  = TComRdCost::xGetSAD4;
  m_afpDistortFunc[10] = TComRdCost::xGetSAD8;
  m_afpDistortFunc[11] = TComRdCost::xGetSAD16;
  m_afpDistortFunc[12] = TComRdCost::xGetSAD32;
  m_afpDistortFunc[13] = TComRdCost::xGetSAD64;
  m_afpDistortFunc[14] = TComRdCost::xGetSAD16N;
  
  m_afpDistortFunc[15] = TComRdCost::xGetSAD;
  m_afpDistortFunc[16] = TComRdCost::xGetSAD4;
  m_afpDistortFunc[17] = TComRdCost::xGetSAD8;
  m_afpDistortFunc[18] = TComRdCost::xGetSAD16;
  m_afpDistortFunc[19] = TComRdCost::xGetSAD32;
  m_afpDistortFunc[20] = TComRdCost::xGetSAD64;
  m_afpDistortFunc[21] = TComRdCost::xGetSAD16N;
  
#if AMP_SAD
  m_afpDistortFunc[43] = TComRdCost::xGetSAD12;
  m_afpDistortFunc[44] = TComRdCost::xGetSAD24;
  m_afpDistortFunc[45] = TComRdCost::xGetSAD48;

  m_afpDistortFunc[46] = TComRdCost::xGetSAD12;
  m_afpDistortFunc[47] = TComRdCost::xGetSAD24;
  m_afpDistortFunc[48] = TComRdCost::xGetSAD48;
#endif
  m_afpDistortFunc[22] = TComRdCost::xGetHADs;
  m_afpDistortFunc[23] = TComRdCost::xGetHADs;
  m_afpDistortFunc[24] = TComRdCost::xGetHADs;
  m_afpDistortFunc[25] = TComRdCost::xGetHADs;
  m_afpDistortFunc[26] = TComRdCost::xGetHADs;
  m_afpDistortFunc[27] = TComRdCost::xGetHADs;
  m_afpDistortFunc[28] = TComRdCost::xGetHADs;
#endif
  
#if !FIX203
  m_puiComponentCostOriginP = NULL;
  m_puiComponentCost        = NULL;
  m_puiVerCost              = NULL;
  m_puiHorCost              = NULL;
#endif
  m_uiCost                  = 0;
  m_iCostScale              = 0;
#if !FIX203
  m_iSearchLimit            = 0xdeaddead;
#endif
}

#if !FIX203
Void TComRdCost::initRateDistortionModel( Int iSubPelSearchLimit )
{
  // make it larger
  iSubPelSearchLimit += 4;
  iSubPelSearchLimit *= 8;
  
  if( m_iSearchLimit != iSubPelSearchLimit )
  {
    xUninit();
    
    m_iSearchLimit = iSubPelSearchLimit;
    
    m_puiComponentCostOriginP = new UInt[ 4 * iSubPelSearchLimit ];
    iSubPelSearchLimit *= 2;
    
    m_puiComponentCost = m_puiComponentCostOriginP + iSubPelSearchLimit;
    
    for( Int n = -iSubPelSearchLimit; n < iSubPelSearchLimit; n++)
    {
      m_puiComponentCost[n] = xGetComponentBits( n );
    }
  }
}

Void TComRdCost::xUninit()
{
  if( NULL != m_puiComponentCostOriginP )
  {
    delete [] m_puiComponentCostOriginP;
    m_puiComponentCostOriginP = NULL;
  }
}
#endif

UInt TComRdCost::xGetComponentBits( Int iVal )
{
  UInt uiLength = 1;
  UInt uiTemp   = ( iVal <= 0) ? (-iVal<<1)+1: (iVal<<1);
  
  assert ( uiTemp );
  
  while ( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  
  return uiLength;
}

Void TComRdCost::setDistParam( UInt uiBlkWidth, UInt uiBlkHeight, DFunc eDFunc, DistParam& rcDistParam )
{
  // set Block Width / Height
  rcDistParam.iCols    = uiBlkWidth;
  rcDistParam.iRows    = uiBlkHeight;
#if QT_BT_STRUCTURE
  rcDistParam.DistFunc = m_afpDistortFunc[eDFunc + g_aucConvertToBit[ rcDistParam.iCols ] + MIN_CU_LOG2 - 1 ];
#else
  rcDistParam.DistFunc = m_afpDistortFunc[eDFunc + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
#endif
  
  // initialize
  rcDistParam.iSubShift  = 0;
}

// Setting the Distortion Parameter for Inter (ME)
Void TComRdCost::setDistParam( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, DistParam& rcDistParam )
{
  // set Original & Curr Pointer / Stride
  rcDistParam.pOrg = pcPatternKey->getROIY();
  rcDistParam.pCur = piRefY;
  
  rcDistParam.iStrideOrg = pcPatternKey->getPatternLStride();
  rcDistParam.iStrideCur = iRefStride;

  // set Block Width / Height
  rcDistParam.iCols    = pcPatternKey->getROIYWidth();
  rcDistParam.iRows    = pcPatternKey->getROIYHeight();
#if QT_BT_STRUCTURE
  rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD + g_aucConvertToBit[ rcDistParam.iCols ] + MIN_CU_LOG2 - 1 ]; 

  if (rcDistParam.iCols == 12)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD12 ]; 
  }
  else if (rcDistParam.iCols == 24)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD24 ]; 
  }
  else if (rcDistParam.iCols == 48)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD48 ]; 
  }
  else if (rcDistParam.iCols == 96)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD96 ]; 
  }
  else if (rcDistParam.iCols == 192)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD192 ]; 
  }
#else
  rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];  
  
#if AMP_SAD
  if (rcDistParam.iCols == 12)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[43 ];
  }
  else if (rcDistParam.iCols == 24)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[44 ];
  }
  else if (rcDistParam.iCols == 48)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[45 ];
  }
#endif
#endif

  // initialize
  rcDistParam.iSubShift  = 0;
}

// Setting the Distortion Parameter for Inter (subpel ME with step)
Void TComRdCost::setDistParam( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, Int iStep, DistParam& rcDistParam, Bool bHADME )
{
  // set Original & Curr Pointer / Stride
  rcDistParam.pOrg = pcPatternKey->getROIY();
  rcDistParam.pCur = piRefY;
  
  rcDistParam.iStrideOrg = pcPatternKey->getPatternLStride();
  rcDistParam.iStrideCur = iRefStride * iStep;
  
  // set Step for interpolated buffer
  rcDistParam.iStep = iStep;
  
  // set Block Width / Height
  rcDistParam.iCols    = pcPatternKey->getROIYWidth();
  rcDistParam.iRows    = pcPatternKey->getROIYHeight();
  
  // set distortion function
  if ( !bHADME )
  {
#if QT_BT_STRUCTURE
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS + g_aucConvertToBit[ rcDistParam.iCols ] + MIN_CU_LOG2 - 1 ];
    if (rcDistParam.iCols == 12)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS12 ]; 
    }
    else if (rcDistParam.iCols == 24)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS24 ]; 
    }
    else if (rcDistParam.iCols == 48)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS48 ]; 
    }
    else if (rcDistParam.iCols == 96)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS96 ]; 
    }
    else if (rcDistParam.iCols == 192)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS192 ]; 
    }
  }
  else
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_HADS + g_aucConvertToBit[ rcDistParam.iCols ] + MIN_CU_LOG2 - 1 ]; 
  }
#else
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
#if AMP_SAD
    if (rcDistParam.iCols == 12)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[46 ];
    }
    else if (rcDistParam.iCols == 24)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[47 ];
    }
    else if (rcDistParam.iCols == 48)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[48 ];
    }
#endif
  }
  else
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_HADS + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
  }
#endif
  
  // initialize
  rcDistParam.iSubShift  = 0;
}

Void
TComRdCost::setDistParam( DistParam& rcDP, Int bitDepth, Pel* p1, Int iStride1, Pel* p2, Int iStride2, Int iWidth, Int iHeight, Bool bHadamard )
{
  rcDP.pOrg       = p1;
  rcDP.pCur       = p2;
  rcDP.iStrideOrg = iStride1;
  rcDP.iStrideCur = iStride2;
  rcDP.iCols      = iWidth;
  rcDP.iRows      = iHeight;
  rcDP.iStep      = 1;
  rcDP.iSubShift  = 0;
  rcDP.bitDepth   = bitDepth;
#if QT_BT_STRUCTURE
  rcDP.DistFunc   = m_afpDistortFunc[ ( bHadamard ? DF_HADS : DF_SADS ) + g_aucConvertToBit[ iWidth ] + MIN_CU_LOG2 - 1 ];  
#else
  rcDP.DistFunc   = m_afpDistortFunc[ ( bHadamard ? DF_HADS : DF_SADS ) + g_aucConvertToBit[ iWidth ] + 1 ];
#endif
}

UInt TComRdCost::calcHAD(Int bitDepth, Pel* pi0, Int iStride0, Pel* pi1, Int iStride1, Int iWidth, Int iHeight )
{
  UInt uiSum = 0;
  Int x, y;
  
#if QT_BT_STRUCTURE 
  if ( iWidth > iHeight && iHeight>=8) 
  {
    for ( y=0; y<iHeight; y+= 8 )
    {
      for ( x=0; x<iWidth; x+= 16 )
      {
        uiSum += xCalcHADs16x8( &pi0[x], &pi1[x], iStride0, iStride1 );
      }
      pi0 += iStride0*8;
      pi1 += iStride1*8;
    }
  }
  else if (  iWidth < iHeight && iWidth>=8) 
  {
    for ( y=0; y<iHeight; y+= 16 )
    {
      for ( x=0; x<iWidth; x+= 8 )
      {
        uiSum += xCalcHADs8x16(&pi0[x], &pi1[x], iStride0, iStride1 );
      }
      pi0 += iStride0*16;
      pi1 += iStride1*16;
    }
  }
  else if ( iWidth > iHeight && iHeight==4) 
  {
    for ( y=0; y<iHeight; y+= 4 )
    {
      for ( x=0; x<iWidth; x+= 8 )
      {
        uiSum += xCalcHADs8x4( &pi0[x], &pi1[x], iStride0, iStride1 );
      }
      pi0 += iStride0*4;
      pi1 += iStride1*4;
    }
  }
  else if (  iWidth < iHeight && iWidth==4) 
  {
    for ( y=0; y<iHeight; y+= 8 )
    {
      for ( x=0; x<iWidth; x+= 4 )
      {
        uiSum += xCalcHADs4x8(&pi0[x], &pi1[x], iStride0, iStride1 );
      }
      pi0 += iStride0*8;
      pi1 += iStride1*8;
    }
  }
  else if (iWidth == 2 || iHeight == 2)
  {
    for ( y=0; y<iHeight; y+= 2 )
    {
      for ( x=0; x<iWidth; x+= 2 )
      {
        uiSum += xCalcHADs2x2(&pi0[x], &pi1[x], iStride0, iStride1, 1 );
      }
      pi0 += iStride0*2;
      pi1 += iStride1*2;
    }
  }
  else
#endif
  if ( ( (iWidth % 8) == 0 ) && ( (iHeight % 8) == 0 ) )
  {
    for ( y=0; y<iHeight; y+= 8 )
    {
      for ( x=0; x<iWidth; x+= 8 )
      {
        uiSum += xCalcHADs8x8( &pi0[x], &pi1[x], iStride0, iStride1, 1 );
      }
      pi0 += iStride0*8;
      pi1 += iStride1*8;
    }
  }
  else
  {
    assert(iWidth % 4 == 0 && iHeight % 4 == 0);
    
    for ( y=0; y<iHeight; y+= 4 )
    {
      for ( x=0; x<iWidth; x+= 4 )
      {
        uiSum += xCalcHADs4x4( &pi0[x], &pi1[x], iStride0, iStride1, 1 );
      }
      pi0 += iStride0*4;
      pi1 += iStride1*4;
    }
  }
  
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(bitDepth-8);

}

UInt TComRdCost::getDistPart(Int bitDepth, Pel* piCur, Int iCurStride,  Pel* piOrg, Int iOrgStride, UInt uiBlkWidth, UInt uiBlkHeight, TextType eText, DFunc eDFunc)
{
  DistParam cDtParam;
  setDistParam( uiBlkWidth, uiBlkHeight, eDFunc, cDtParam );
  cDtParam.pOrg       = piOrg;
  cDtParam.pCur       = piCur;
  cDtParam.iStrideOrg = iOrgStride;
  cDtParam.iStrideCur = iCurStride;
  cDtParam.iStep      = 1;

  cDtParam.bApplyWeight = false;
  cDtParam.uiComp       = 255;    // just for assert: to be sure it was set before use, since only values 0,1 or 2 are allowed.
  cDtParam.bitDepth = bitDepth;

  if (eText == TEXT_CHROMA_U)
  {
   return ((Int) (m_cbDistortionWeight * cDtParam.DistFunc( &cDtParam )));
  }
  else if (eText == TEXT_CHROMA_V)
  {
   return ((Int) (m_crDistortionWeight * cDtParam.DistFunc( &cDtParam )));
  }
  else
  {
    return cDtParam.DistFunc( &cDtParam );
  }
}

// ====================================================================================================================
// Distortion functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// SAD
// --------------------------------------------------------------------------------------------------------------------

UInt TComRdCost::xGetSAD( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideCur = pcDtParam->iStrideCur;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-- )
  {
    for (Int n = 0; n < iCols; n++ )
    {
      uiSum += abs( piOrg[n] - piCur[n] );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetSAD4( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight ) 
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetSAD8( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg      = pcDtParam->pOrg;
  Pel* piCur      = pcDtParam->pCur;
  Int  iRows      = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetSAD16( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

#if AMP_SAD
UInt TComRdCost::xGetSAD12( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}
#endif

UInt TComRdCost::xGetSAD16N( DistParam* pcDtParam )
{
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    for (Int n = 0; n < iCols; n+=16 )
    {
      uiSum += abs( piOrg[n+ 0] - piCur[n+ 0] );
      uiSum += abs( piOrg[n+ 1] - piCur[n+ 1] );
      uiSum += abs( piOrg[n+ 2] - piCur[n+ 2] );
      uiSum += abs( piOrg[n+ 3] - piCur[n+ 3] );
      uiSum += abs( piOrg[n+ 4] - piCur[n+ 4] );
      uiSum += abs( piOrg[n+ 5] - piCur[n+ 5] );
      uiSum += abs( piOrg[n+ 6] - piCur[n+ 6] );
      uiSum += abs( piOrg[n+ 7] - piCur[n+ 7] );
      uiSum += abs( piOrg[n+ 8] - piCur[n+ 8] );
      uiSum += abs( piOrg[n+ 9] - piCur[n+ 9] );
      uiSum += abs( piOrg[n+10] - piCur[n+10] );
      uiSum += abs( piOrg[n+11] - piCur[n+11] );
      uiSum += abs( piOrg[n+12] - piCur[n+12] );
      uiSum += abs( piOrg[n+13] - piCur[n+13] );
      uiSum += abs( piOrg[n+14] - piCur[n+14] );
      uiSum += abs( piOrg[n+15] - piCur[n+15] );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetSAD32( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

#if AMP_SAD
UInt TComRdCost::xGetSAD24( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

#endif

UInt TComRdCost::xGetSAD64( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );
    uiSum += abs( piOrg[48] - piCur[48] );
    uiSum += abs( piOrg[49] - piCur[49] );
    uiSum += abs( piOrg[50] - piCur[50] );
    uiSum += abs( piOrg[51] - piCur[51] );
    uiSum += abs( piOrg[52] - piCur[52] );
    uiSum += abs( piOrg[53] - piCur[53] );
    uiSum += abs( piOrg[54] - piCur[54] );
    uiSum += abs( piOrg[55] - piCur[55] );
    uiSum += abs( piOrg[56] - piCur[56] );
    uiSum += abs( piOrg[57] - piCur[57] );
    uiSum += abs( piOrg[58] - piCur[58] );
    uiSum += abs( piOrg[59] - piCur[59] );
    uiSum += abs( piOrg[60] - piCur[60] );
    uiSum += abs( piOrg[61] - piCur[61] );
    uiSum += abs( piOrg[62] - piCur[62] );
    uiSum += abs( piOrg[63] - piCur[63] );
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

#if QT_BT_STRUCTURE
UInt TComRdCost::xGetSAD128( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  UInt uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    //
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );
    uiSum += abs( piOrg[48] - piCur[48] );
    uiSum += abs( piOrg[49] - piCur[49] );
    uiSum += abs( piOrg[50] - piCur[50] );
    uiSum += abs( piOrg[51] - piCur[51] );
    uiSum += abs( piOrg[52] - piCur[52] );
    uiSum += abs( piOrg[53] - piCur[53] );
    uiSum += abs( piOrg[54] - piCur[54] );
    uiSum += abs( piOrg[55] - piCur[55] );
    uiSum += abs( piOrg[56] - piCur[56] );
    uiSum += abs( piOrg[57] - piCur[57] );
    uiSum += abs( piOrg[58] - piCur[58] );
    uiSum += abs( piOrg[59] - piCur[59] );
    uiSum += abs( piOrg[60] - piCur[60] );
    uiSum += abs( piOrg[61] - piCur[61] );
    uiSum += abs( piOrg[62] - piCur[62] );
    uiSum += abs( piOrg[63] - piCur[63] );
    uiSum += abs( piOrg[64] - piCur[64] );
    uiSum += abs( piOrg[65] - piCur[65] );
    uiSum += abs( piOrg[66] - piCur[66] );
    uiSum += abs( piOrg[67] - piCur[67] );
    uiSum += abs( piOrg[68] - piCur[68] );
    uiSum += abs( piOrg[69] - piCur[69] );
    uiSum += abs( piOrg[70] - piCur[70] );
    uiSum += abs( piOrg[71] - piCur[71] );
    uiSum += abs( piOrg[72] - piCur[72] );
    uiSum += abs( piOrg[73] - piCur[73] );
    uiSum += abs( piOrg[74] - piCur[74] );
    uiSum += abs( piOrg[75] - piCur[75] );
    uiSum += abs( piOrg[76] - piCur[76] );
    uiSum += abs( piOrg[77] - piCur[77] );
    uiSum += abs( piOrg[78] - piCur[78] );
    uiSum += abs( piOrg[79] - piCur[79] );
    uiSum += abs( piOrg[80] - piCur[80] );
    uiSum += abs( piOrg[81] - piCur[81] );
    uiSum += abs( piOrg[82] - piCur[82] );
    uiSum += abs( piOrg[83] - piCur[83] );
    uiSum += abs( piOrg[84] - piCur[84] );
    uiSum += abs( piOrg[85] - piCur[85] );
    uiSum += abs( piOrg[86] - piCur[86] );
    uiSum += abs( piOrg[87] - piCur[87] );
    uiSum += abs( piOrg[88] - piCur[88] );
    uiSum += abs( piOrg[89] - piCur[89] );
    uiSum += abs( piOrg[90] - piCur[90] );
    uiSum += abs( piOrg[91] - piCur[91] );
    uiSum += abs( piOrg[92] - piCur[92] );
    uiSum += abs( piOrg[93] - piCur[93] );
    uiSum += abs( piOrg[94] - piCur[94] );
    uiSum += abs( piOrg[95] - piCur[95] );
    uiSum += abs( piOrg[96] - piCur[96] );
    uiSum += abs( piOrg[97] - piCur[97] );
    uiSum += abs( piOrg[98] - piCur[98] );
    uiSum += abs( piOrg[99] - piCur[99] );
    uiSum += abs( piOrg[100] - piCur[100] );
    uiSum += abs( piOrg[101] - piCur[101] );
    uiSum += abs( piOrg[102] - piCur[102] );
    uiSum += abs( piOrg[103] - piCur[103] );
    uiSum += abs( piOrg[104] - piCur[104] );
    uiSum += abs( piOrg[105] - piCur[105] );
    uiSum += abs( piOrg[106] - piCur[106] );
    uiSum += abs( piOrg[107] - piCur[107] );
    uiSum += abs( piOrg[108] - piCur[108] );
    uiSum += abs( piOrg[109] - piCur[109] );
    uiSum += abs( piOrg[110] - piCur[110] );
    uiSum += abs( piOrg[111] - piCur[111] );
    uiSum += abs( piOrg[112] - piCur[112] );
    uiSum += abs( piOrg[113] - piCur[113] );
    uiSum += abs( piOrg[114] - piCur[114] );
    uiSum += abs( piOrg[115] - piCur[115] );
    uiSum += abs( piOrg[116] - piCur[116] );
    uiSum += abs( piOrg[117] - piCur[117] );
    uiSum += abs( piOrg[118] - piCur[118] );
    uiSum += abs( piOrg[119] - piCur[119] );
    uiSum += abs( piOrg[120] - piCur[120] );
    uiSum += abs( piOrg[121] - piCur[121] );
    uiSum += abs( piOrg[122] - piCur[122] );
    uiSum += abs( piOrg[123] - piCur[123] );
    uiSum += abs( piOrg[124] - piCur[124] );
    uiSum += abs( piOrg[125] - piCur[125] );
    uiSum += abs( piOrg[126] - piCur[126] );
    uiSum += abs( piOrg[127] - piCur[127] );

    //

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}
UInt TComRdCost::xGetSAD256( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  UInt uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    //
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );
    uiSum += abs( piOrg[48] - piCur[48] );
    uiSum += abs( piOrg[49] - piCur[49] );
    uiSum += abs( piOrg[50] - piCur[50] );
    uiSum += abs( piOrg[51] - piCur[51] );
    uiSum += abs( piOrg[52] - piCur[52] );
    uiSum += abs( piOrg[53] - piCur[53] );
    uiSum += abs( piOrg[54] - piCur[54] );
    uiSum += abs( piOrg[55] - piCur[55] );
    uiSum += abs( piOrg[56] - piCur[56] );
    uiSum += abs( piOrg[57] - piCur[57] );
    uiSum += abs( piOrg[58] - piCur[58] );
    uiSum += abs( piOrg[59] - piCur[59] );
    uiSum += abs( piOrg[60] - piCur[60] );
    uiSum += abs( piOrg[61] - piCur[61] );
    uiSum += abs( piOrg[62] - piCur[62] );
    uiSum += abs( piOrg[63] - piCur[63] );
    uiSum += abs( piOrg[64] - piCur[64] );
    uiSum += abs( piOrg[65] - piCur[65] );
    uiSum += abs( piOrg[66] - piCur[66] );
    uiSum += abs( piOrg[67] - piCur[67] );
    uiSum += abs( piOrg[68] - piCur[68] );
    uiSum += abs( piOrg[69] - piCur[69] );
    uiSum += abs( piOrg[70] - piCur[70] );
    uiSum += abs( piOrg[71] - piCur[71] );
    uiSum += abs( piOrg[72] - piCur[72] );
    uiSum += abs( piOrg[73] - piCur[73] );
    uiSum += abs( piOrg[74] - piCur[74] );
    uiSum += abs( piOrg[75] - piCur[75] );
    uiSum += abs( piOrg[76] - piCur[76] );
    uiSum += abs( piOrg[77] - piCur[77] );
    uiSum += abs( piOrg[78] - piCur[78] );
    uiSum += abs( piOrg[79] - piCur[79] );
    uiSum += abs( piOrg[80] - piCur[80] );
    uiSum += abs( piOrg[81] - piCur[81] );
    uiSum += abs( piOrg[82] - piCur[82] );
    uiSum += abs( piOrg[83] - piCur[83] );
    uiSum += abs( piOrg[84] - piCur[84] );
    uiSum += abs( piOrg[85] - piCur[85] );
    uiSum += abs( piOrg[86] - piCur[86] );
    uiSum += abs( piOrg[87] - piCur[87] );
    uiSum += abs( piOrg[88] - piCur[88] );
    uiSum += abs( piOrg[89] - piCur[89] );
    uiSum += abs( piOrg[90] - piCur[90] );
    uiSum += abs( piOrg[91] - piCur[91] );
    uiSum += abs( piOrg[92] - piCur[92] );
    uiSum += abs( piOrg[93] - piCur[93] );
    uiSum += abs( piOrg[94] - piCur[94] );
    uiSum += abs( piOrg[95] - piCur[95] );
    uiSum += abs( piOrg[96] - piCur[96] );
    uiSum += abs( piOrg[97] - piCur[97] );
    uiSum += abs( piOrg[98] - piCur[98] );
    uiSum += abs( piOrg[99] - piCur[99] );
    uiSum += abs( piOrg[100] - piCur[100] );
    uiSum += abs( piOrg[101] - piCur[101] );
    uiSum += abs( piOrg[102] - piCur[102] );
    uiSum += abs( piOrg[103] - piCur[103] );
    uiSum += abs( piOrg[104] - piCur[104] );
    uiSum += abs( piOrg[105] - piCur[105] );
    uiSum += abs( piOrg[106] - piCur[106] );
    uiSum += abs( piOrg[107] - piCur[107] );
    uiSum += abs( piOrg[108] - piCur[108] );
    uiSum += abs( piOrg[109] - piCur[109] );
    uiSum += abs( piOrg[110] - piCur[110] );
    uiSum += abs( piOrg[111] - piCur[111] );
    uiSum += abs( piOrg[112] - piCur[112] );
    uiSum += abs( piOrg[113] - piCur[113] );
    uiSum += abs( piOrg[114] - piCur[114] );
    uiSum += abs( piOrg[115] - piCur[115] );
    uiSum += abs( piOrg[116] - piCur[116] );
    uiSum += abs( piOrg[117] - piCur[117] );
    uiSum += abs( piOrg[118] - piCur[118] );
    uiSum += abs( piOrg[119] - piCur[119] );
    uiSum += abs( piOrg[120] - piCur[120] );
    uiSum += abs( piOrg[121] - piCur[121] );
    uiSum += abs( piOrg[122] - piCur[122] );
    uiSum += abs( piOrg[123] - piCur[123] );
    uiSum += abs( piOrg[124] - piCur[124] );
    uiSum += abs( piOrg[125] - piCur[125] );
    uiSum += abs( piOrg[126] - piCur[126] );
    uiSum += abs( piOrg[127] - piCur[127] );
    uiSum += abs( piOrg[128] - piCur[128] );
    uiSum += abs( piOrg[129] - piCur[129] );
    uiSum += abs( piOrg[130] - piCur[130] );
    uiSum += abs( piOrg[131] - piCur[131] );
    uiSum += abs( piOrg[132] - piCur[132] );
    uiSum += abs( piOrg[133] - piCur[133] );
    uiSum += abs( piOrg[134] - piCur[134] );
    uiSum += abs( piOrg[135] - piCur[135] );
    uiSum += abs( piOrg[136] - piCur[136] );
    uiSum += abs( piOrg[137] - piCur[137] );
    uiSum += abs( piOrg[138] - piCur[138] );
    uiSum += abs( piOrg[139] - piCur[139] );
    uiSum += abs( piOrg[140] - piCur[140] );
    uiSum += abs( piOrg[141] - piCur[141] );
    uiSum += abs( piOrg[142] - piCur[142] );
    uiSum += abs( piOrg[143] - piCur[143] );
    uiSum += abs( piOrg[144] - piCur[144] );
    uiSum += abs( piOrg[145] - piCur[145] );
    uiSum += abs( piOrg[146] - piCur[146] );
    uiSum += abs( piOrg[147] - piCur[147] );
    uiSum += abs( piOrg[148] - piCur[148] );
    uiSum += abs( piOrg[149] - piCur[149] );
    uiSum += abs( piOrg[150] - piCur[150] );
    uiSum += abs( piOrg[151] - piCur[151] );
    uiSum += abs( piOrg[152] - piCur[152] );
    uiSum += abs( piOrg[153] - piCur[153] );
    uiSum += abs( piOrg[154] - piCur[154] );
    uiSum += abs( piOrg[155] - piCur[155] );
    uiSum += abs( piOrg[156] - piCur[156] );
    uiSum += abs( piOrg[157] - piCur[157] );
    uiSum += abs( piOrg[158] - piCur[158] );
    uiSum += abs( piOrg[159] - piCur[159] );
    uiSum += abs( piOrg[160] - piCur[160] );
    uiSum += abs( piOrg[161] - piCur[161] );
    uiSum += abs( piOrg[162] - piCur[162] );
    uiSum += abs( piOrg[163] - piCur[163] );
    uiSum += abs( piOrg[164] - piCur[164] );
    uiSum += abs( piOrg[165] - piCur[165] );
    uiSum += abs( piOrg[166] - piCur[166] );
    uiSum += abs( piOrg[167] - piCur[167] );
    uiSum += abs( piOrg[168] - piCur[168] );
    uiSum += abs( piOrg[169] - piCur[169] );
    uiSum += abs( piOrg[170] - piCur[170] );
    uiSum += abs( piOrg[171] - piCur[171] );
    uiSum += abs( piOrg[172] - piCur[172] );
    uiSum += abs( piOrg[173] - piCur[173] );
    uiSum += abs( piOrg[174] - piCur[174] );
    uiSum += abs( piOrg[175] - piCur[175] );
    uiSum += abs( piOrg[176] - piCur[176] );
    uiSum += abs( piOrg[177] - piCur[177] );
    uiSum += abs( piOrg[178] - piCur[178] );
    uiSum += abs( piOrg[179] - piCur[179] );
    uiSum += abs( piOrg[180] - piCur[180] );
    uiSum += abs( piOrg[181] - piCur[181] );
    uiSum += abs( piOrg[182] - piCur[182] );
    uiSum += abs( piOrg[183] - piCur[183] );
    uiSum += abs( piOrg[184] - piCur[184] );
    uiSum += abs( piOrg[185] - piCur[185] );
    uiSum += abs( piOrg[186] - piCur[186] );
    uiSum += abs( piOrg[187] - piCur[187] );
    uiSum += abs( piOrg[188] - piCur[188] );
    uiSum += abs( piOrg[189] - piCur[189] );
    uiSum += abs( piOrg[190] - piCur[190] );
    uiSum += abs( piOrg[191] - piCur[191] );
    uiSum += abs( piOrg[192] - piCur[192] );
    uiSum += abs( piOrg[193] - piCur[193] );
    uiSum += abs( piOrg[194] - piCur[194] );
    uiSum += abs( piOrg[195] - piCur[195] );
    uiSum += abs( piOrg[196] - piCur[196] );
    uiSum += abs( piOrg[197] - piCur[197] );
    uiSum += abs( piOrg[198] - piCur[198] );
    uiSum += abs( piOrg[199] - piCur[199] );
    uiSum += abs( piOrg[200] - piCur[200] );
    uiSum += abs( piOrg[201] - piCur[201] );
    uiSum += abs( piOrg[202] - piCur[202] );
    uiSum += abs( piOrg[203] - piCur[203] );
    uiSum += abs( piOrg[204] - piCur[204] );
    uiSum += abs( piOrg[205] - piCur[205] );
    uiSum += abs( piOrg[206] - piCur[206] );
    uiSum += abs( piOrg[207] - piCur[207] );
    uiSum += abs( piOrg[208] - piCur[208] );
    uiSum += abs( piOrg[209] - piCur[209] );
    uiSum += abs( piOrg[210] - piCur[210] );
    uiSum += abs( piOrg[211] - piCur[211] );
    uiSum += abs( piOrg[212] - piCur[212] );
    uiSum += abs( piOrg[213] - piCur[213] );
    uiSum += abs( piOrg[214] - piCur[214] );
    uiSum += abs( piOrg[215] - piCur[215] );
    uiSum += abs( piOrg[216] - piCur[216] );
    uiSum += abs( piOrg[217] - piCur[217] );
    uiSum += abs( piOrg[218] - piCur[218] );
    uiSum += abs( piOrg[219] - piCur[219] );
    uiSum += abs( piOrg[220] - piCur[220] );
    uiSum += abs( piOrg[221] - piCur[221] );
    uiSum += abs( piOrg[222] - piCur[222] );
    uiSum += abs( piOrg[223] - piCur[223] );
    uiSum += abs( piOrg[224] - piCur[224] );
    uiSum += abs( piOrg[225] - piCur[225] );
    uiSum += abs( piOrg[226] - piCur[226] );
    uiSum += abs( piOrg[227] - piCur[227] );
    uiSum += abs( piOrg[228] - piCur[228] );
    uiSum += abs( piOrg[229] - piCur[229] );
    uiSum += abs( piOrg[230] - piCur[230] );
    uiSum += abs( piOrg[231] - piCur[231] );
    uiSum += abs( piOrg[232] - piCur[232] );
    uiSum += abs( piOrg[233] - piCur[233] );
    uiSum += abs( piOrg[234] - piCur[234] );
    uiSum += abs( piOrg[235] - piCur[235] );
    uiSum += abs( piOrg[236] - piCur[236] );
    uiSum += abs( piOrg[237] - piCur[237] );
    uiSum += abs( piOrg[238] - piCur[238] );
    uiSum += abs( piOrg[239] - piCur[239] );
    uiSum += abs( piOrg[240] - piCur[240] );
    uiSum += abs( piOrg[241] - piCur[241] );
    uiSum += abs( piOrg[242] - piCur[242] );
    uiSum += abs( piOrg[243] - piCur[243] );
    uiSum += abs( piOrg[244] - piCur[244] );
    uiSum += abs( piOrg[245] - piCur[245] );
    uiSum += abs( piOrg[246] - piCur[246] );
    uiSum += abs( piOrg[247] - piCur[247] );
    uiSum += abs( piOrg[248] - piCur[248] );
    uiSum += abs( piOrg[249] - piCur[249] );
    uiSum += abs( piOrg[250] - piCur[250] );
    uiSum += abs( piOrg[251] - piCur[251] );
    uiSum += abs( piOrg[252] - piCur[252] );
    uiSum += abs( piOrg[253] - piCur[253] );
    uiSum += abs( piOrg[254] - piCur[254] );
    uiSum += abs( piOrg[255] - piCur[255] );

    //

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}
#endif

#if AMP_SAD
UInt TComRdCost::xGetSAD48( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;
  
  UInt uiSum = 0;
  
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}
#endif

#if QT_BT_STRUCTURE
UInt TComRdCost::xGetSAD96( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  UInt uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    //
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );
    uiSum += abs( piOrg[48] - piCur[48] );
    uiSum += abs( piOrg[49] - piCur[49] );
    uiSum += abs( piOrg[50] - piCur[50] );
    uiSum += abs( piOrg[51] - piCur[51] );
    uiSum += abs( piOrg[52] - piCur[52] );
    uiSum += abs( piOrg[53] - piCur[53] );
    uiSum += abs( piOrg[54] - piCur[54] );
    uiSum += abs( piOrg[55] - piCur[55] );
    uiSum += abs( piOrg[56] - piCur[56] );
    uiSum += abs( piOrg[57] - piCur[57] );
    uiSum += abs( piOrg[58] - piCur[58] );
    uiSum += abs( piOrg[59] - piCur[59] );
    uiSum += abs( piOrg[60] - piCur[60] );
    uiSum += abs( piOrg[61] - piCur[61] );
    uiSum += abs( piOrg[62] - piCur[62] );
    uiSum += abs( piOrg[63] - piCur[63] );
    uiSum += abs( piOrg[64] - piCur[64] );
    uiSum += abs( piOrg[65] - piCur[65] );
    uiSum += abs( piOrg[66] - piCur[66] );
    uiSum += abs( piOrg[67] - piCur[67] );
    uiSum += abs( piOrg[68] - piCur[68] );
    uiSum += abs( piOrg[69] - piCur[69] );
    uiSum += abs( piOrg[70] - piCur[70] );
    uiSum += abs( piOrg[71] - piCur[71] );
    uiSum += abs( piOrg[72] - piCur[72] );
    uiSum += abs( piOrg[73] - piCur[73] );
    uiSum += abs( piOrg[74] - piCur[74] );
    uiSum += abs( piOrg[75] - piCur[75] );
    uiSum += abs( piOrg[76] - piCur[76] );
    uiSum += abs( piOrg[77] - piCur[77] );
    uiSum += abs( piOrg[78] - piCur[78] );
    uiSum += abs( piOrg[79] - piCur[79] );
    uiSum += abs( piOrg[80] - piCur[80] );
    uiSum += abs( piOrg[81] - piCur[81] );
    uiSum += abs( piOrg[82] - piCur[82] );
    uiSum += abs( piOrg[83] - piCur[83] );
    uiSum += abs( piOrg[84] - piCur[84] );
    uiSum += abs( piOrg[85] - piCur[85] );
    uiSum += abs( piOrg[86] - piCur[86] );
    uiSum += abs( piOrg[87] - piCur[87] );
    uiSum += abs( piOrg[88] - piCur[88] );
    uiSum += abs( piOrg[89] - piCur[89] );
    uiSum += abs( piOrg[90] - piCur[90] );
    uiSum += abs( piOrg[91] - piCur[91] );
    uiSum += abs( piOrg[92] - piCur[92] );
    uiSum += abs( piOrg[93] - piCur[93] );
    uiSum += abs( piOrg[94] - piCur[94] );
    uiSum += abs( piOrg[95] - piCur[95] );

    //

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetSAD192( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSADw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  UInt uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    //
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );
    uiSum += abs( piOrg[48] - piCur[48] );
    uiSum += abs( piOrg[49] - piCur[49] );
    uiSum += abs( piOrg[50] - piCur[50] );
    uiSum += abs( piOrg[51] - piCur[51] );
    uiSum += abs( piOrg[52] - piCur[52] );
    uiSum += abs( piOrg[53] - piCur[53] );
    uiSum += abs( piOrg[54] - piCur[54] );
    uiSum += abs( piOrg[55] - piCur[55] );
    uiSum += abs( piOrg[56] - piCur[56] );
    uiSum += abs( piOrg[57] - piCur[57] );
    uiSum += abs( piOrg[58] - piCur[58] );
    uiSum += abs( piOrg[59] - piCur[59] );
    uiSum += abs( piOrg[60] - piCur[60] );
    uiSum += abs( piOrg[61] - piCur[61] );
    uiSum += abs( piOrg[62] - piCur[62] );
    uiSum += abs( piOrg[63] - piCur[63] );
    uiSum += abs( piOrg[64] - piCur[64] );
    uiSum += abs( piOrg[65] - piCur[65] );
    uiSum += abs( piOrg[66] - piCur[66] );
    uiSum += abs( piOrg[67] - piCur[67] );
    uiSum += abs( piOrg[68] - piCur[68] );
    uiSum += abs( piOrg[69] - piCur[69] );
    uiSum += abs( piOrg[70] - piCur[70] );
    uiSum += abs( piOrg[71] - piCur[71] );
    uiSum += abs( piOrg[72] - piCur[72] );
    uiSum += abs( piOrg[73] - piCur[73] );
    uiSum += abs( piOrg[74] - piCur[74] );
    uiSum += abs( piOrg[75] - piCur[75] );
    uiSum += abs( piOrg[76] - piCur[76] );
    uiSum += abs( piOrg[77] - piCur[77] );
    uiSum += abs( piOrg[78] - piCur[78] );
    uiSum += abs( piOrg[79] - piCur[79] );
    uiSum += abs( piOrg[80] - piCur[80] );
    uiSum += abs( piOrg[81] - piCur[81] );
    uiSum += abs( piOrg[82] - piCur[82] );
    uiSum += abs( piOrg[83] - piCur[83] );
    uiSum += abs( piOrg[84] - piCur[84] );
    uiSum += abs( piOrg[85] - piCur[85] );
    uiSum += abs( piOrg[86] - piCur[86] );
    uiSum += abs( piOrg[87] - piCur[87] );
    uiSum += abs( piOrg[88] - piCur[88] );
    uiSum += abs( piOrg[89] - piCur[89] );
    uiSum += abs( piOrg[90] - piCur[90] );
    uiSum += abs( piOrg[91] - piCur[91] );
    uiSum += abs( piOrg[92] - piCur[92] );
    uiSum += abs( piOrg[93] - piCur[93] );
    uiSum += abs( piOrg[94] - piCur[94] );
    uiSum += abs( piOrg[95] - piCur[95] );
    uiSum += abs( piOrg[96] - piCur[96] );
    uiSum += abs( piOrg[97] - piCur[97] );
    uiSum += abs( piOrg[98] - piCur[98] );
    uiSum += abs( piOrg[99] - piCur[99] );
    uiSum += abs( piOrg[100] - piCur[100] );
    uiSum += abs( piOrg[101] - piCur[101] );
    uiSum += abs( piOrg[102] - piCur[102] );
    uiSum += abs( piOrg[103] - piCur[103] );
    uiSum += abs( piOrg[104] - piCur[104] );
    uiSum += abs( piOrg[105] - piCur[105] );
    uiSum += abs( piOrg[106] - piCur[106] );
    uiSum += abs( piOrg[107] - piCur[107] );
    uiSum += abs( piOrg[108] - piCur[108] );
    uiSum += abs( piOrg[109] - piCur[109] );
    uiSum += abs( piOrg[110] - piCur[110] );
    uiSum += abs( piOrg[111] - piCur[111] );
    uiSum += abs( piOrg[112] - piCur[112] );
    uiSum += abs( piOrg[113] - piCur[113] );
    uiSum += abs( piOrg[114] - piCur[114] );
    uiSum += abs( piOrg[115] - piCur[115] );
    uiSum += abs( piOrg[116] - piCur[116] );
    uiSum += abs( piOrg[117] - piCur[117] );
    uiSum += abs( piOrg[118] - piCur[118] );
    uiSum += abs( piOrg[119] - piCur[119] );
    uiSum += abs( piOrg[120] - piCur[120] );
    uiSum += abs( piOrg[121] - piCur[121] );
    uiSum += abs( piOrg[122] - piCur[122] );
    uiSum += abs( piOrg[123] - piCur[123] );
    uiSum += abs( piOrg[124] - piCur[124] );
    uiSum += abs( piOrg[125] - piCur[125] );
    uiSum += abs( piOrg[126] - piCur[126] );
    uiSum += abs( piOrg[127] - piCur[127] );
    uiSum += abs( piOrg[128] - piCur[128] );
    uiSum += abs( piOrg[129] - piCur[129] );
    uiSum += abs( piOrg[130] - piCur[130] );
    uiSum += abs( piOrg[131] - piCur[131] );
    uiSum += abs( piOrg[132] - piCur[132] );
    uiSum += abs( piOrg[133] - piCur[133] );
    uiSum += abs( piOrg[134] - piCur[134] );
    uiSum += abs( piOrg[135] - piCur[135] );
    uiSum += abs( piOrg[136] - piCur[136] );
    uiSum += abs( piOrg[137] - piCur[137] );
    uiSum += abs( piOrg[138] - piCur[138] );
    uiSum += abs( piOrg[139] - piCur[139] );
    uiSum += abs( piOrg[140] - piCur[140] );
    uiSum += abs( piOrg[141] - piCur[141] );
    uiSum += abs( piOrg[142] - piCur[142] );
    uiSum += abs( piOrg[143] - piCur[143] );
    uiSum += abs( piOrg[144] - piCur[144] );
    uiSum += abs( piOrg[145] - piCur[145] );
    uiSum += abs( piOrg[146] - piCur[146] );
    uiSum += abs( piOrg[147] - piCur[147] );
    uiSum += abs( piOrg[148] - piCur[148] );
    uiSum += abs( piOrg[149] - piCur[149] );
    uiSum += abs( piOrg[150] - piCur[150] );
    uiSum += abs( piOrg[151] - piCur[151] );
    uiSum += abs( piOrg[152] - piCur[152] );
    uiSum += abs( piOrg[153] - piCur[153] );
    uiSum += abs( piOrg[154] - piCur[154] );
    uiSum += abs( piOrg[155] - piCur[155] );
    uiSum += abs( piOrg[156] - piCur[156] );
    uiSum += abs( piOrg[157] - piCur[157] );
    uiSum += abs( piOrg[158] - piCur[158] );
    uiSum += abs( piOrg[159] - piCur[159] );
    uiSum += abs( piOrg[160] - piCur[160] );
    uiSum += abs( piOrg[161] - piCur[161] );
    uiSum += abs( piOrg[162] - piCur[162] );
    uiSum += abs( piOrg[163] - piCur[163] );
    uiSum += abs( piOrg[164] - piCur[164] );
    uiSum += abs( piOrg[165] - piCur[165] );
    uiSum += abs( piOrg[166] - piCur[166] );
    uiSum += abs( piOrg[167] - piCur[167] );
    uiSum += abs( piOrg[168] - piCur[168] );
    uiSum += abs( piOrg[169] - piCur[169] );
    uiSum += abs( piOrg[170] - piCur[170] );
    uiSum += abs( piOrg[171] - piCur[171] );
    uiSum += abs( piOrg[172] - piCur[172] );
    uiSum += abs( piOrg[173] - piCur[173] );
    uiSum += abs( piOrg[174] - piCur[174] );
    uiSum += abs( piOrg[175] - piCur[175] );
    uiSum += abs( piOrg[176] - piCur[176] );
    uiSum += abs( piOrg[177] - piCur[177] );
    uiSum += abs( piOrg[178] - piCur[178] );
    uiSum += abs( piOrg[179] - piCur[179] );
    uiSum += abs( piOrg[180] - piCur[180] );
    uiSum += abs( piOrg[181] - piCur[181] );
    uiSum += abs( piOrg[182] - piCur[182] );
    uiSum += abs( piOrg[183] - piCur[183] );
    uiSum += abs( piOrg[184] - piCur[184] );
    uiSum += abs( piOrg[185] - piCur[185] );
    uiSum += abs( piOrg[186] - piCur[186] );
    uiSum += abs( piOrg[187] - piCur[187] );
    uiSum += abs( piOrg[188] - piCur[188] );
    uiSum += abs( piOrg[189] - piCur[189] );
    uiSum += abs( piOrg[190] - piCur[190] );
    uiSum += abs( piOrg[191] - piCur[191] );

    //

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}


#endif
// --------------------------------------------------------------------------------------------------------------------
// SSE
// --------------------------------------------------------------------------------------------------------------------

UInt TComRdCost::xGetSSE( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;
  
  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  
  Int iTemp;
  
  for( ; iRows != 0; iRows-- )
  {
    for (Int n = 0; n < iCols; n++ )
    {
      iTemp = piOrg[n  ] - piCur[n  ];
      uiSum += ( iTemp * iTemp ) >> uiShift;
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  return ( uiSum );
}

UInt TComRdCost::xGetSSE4( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 4 );
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;
  
  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  
  Int  iTemp;
  
  for( ; iRows != 0; iRows-- )
  {
    
    iTemp = piOrg[0] - piCur[0]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[1] - piCur[1]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[2] - piCur[2]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[3] - piCur[3]; uiSum += ( iTemp * iTemp ) >> uiShift;
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  return ( uiSum );
}

UInt TComRdCost::xGetSSE8( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 8 );
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;
  
  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  
  Int  iTemp;
  
  for( ; iRows != 0; iRows-- )
  {
    iTemp = piOrg[0] - piCur[0]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[1] - piCur[1]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[2] - piCur[2]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[3] - piCur[3]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[4] - piCur[4]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[5] - piCur[5]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[6] - piCur[6]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[7] - piCur[7]; uiSum += ( iTemp * iTemp ) >> uiShift;
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  return ( uiSum );
}

UInt TComRdCost::xGetSSE16( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 16 );
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;
  
  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  
  Int  iTemp;
  
  for( ; iRows != 0; iRows-- )
  {
    
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[10] - piCur[10]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[11] - piCur[11]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[12] - piCur[12]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[13] - piCur[13]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[14] - piCur[14]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[15] - piCur[15]; uiSum += ( iTemp * iTemp ) >> uiShift;
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  return ( uiSum );
}

UInt TComRdCost::xGetSSE16N( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;
  
  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  Int  iTemp;
  
  for( ; iRows != 0; iRows-- )
  {
    for (Int n = 0; n < iCols; n+=16 )
    {
      
      iTemp = piOrg[n+ 0] - piCur[n+ 0]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 1] - piCur[n+ 1]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 2] - piCur[n+ 2]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 3] - piCur[n+ 3]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 4] - piCur[n+ 4]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 5] - piCur[n+ 5]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 6] - piCur[n+ 6]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 7] - piCur[n+ 7]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 8] - piCur[n+ 8]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+ 9] - piCur[n+ 9]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+10] - piCur[n+10]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+11] - piCur[n+11]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+12] - piCur[n+12]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+13] - piCur[n+13]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+14] - piCur[n+14]; uiSum += ( iTemp * iTemp ) >> uiShift;
      iTemp = piOrg[n+15] - piCur[n+15]; uiSum += ( iTemp * iTemp ) >> uiShift;
      
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  return ( uiSum );
}

UInt TComRdCost::xGetSSE32( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 32 );
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;
  
  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  Int  iTemp;
  
  for( ; iRows != 0; iRows-- )
  {
    
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[10] - piCur[10]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[11] - piCur[11]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[12] - piCur[12]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[13] - piCur[13]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[14] - piCur[14]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[15] - piCur[15]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[16] - piCur[16]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[17] - piCur[17]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[18] - piCur[18]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[19] - piCur[19]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[20] - piCur[20]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[21] - piCur[21]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[22] - piCur[22]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[23] - piCur[23]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[24] - piCur[24]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[25] - piCur[25]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[26] - piCur[26]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[27] - piCur[27]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[28] - piCur[28]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[29] - piCur[29]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[30] - piCur[30]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[31] - piCur[31]; uiSum += ( iTemp * iTemp ) >> uiShift;
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  
  return ( uiSum );
}

UInt TComRdCost::xGetSSE64( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 64 );
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;
  
  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  Int  iTemp;
  
  for( ; iRows != 0; iRows-- )
  {
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[10] - piCur[10]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[11] - piCur[11]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[12] - piCur[12]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[13] - piCur[13]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[14] - piCur[14]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[15] - piCur[15]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[16] - piCur[16]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[17] - piCur[17]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[18] - piCur[18]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[19] - piCur[19]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[20] - piCur[20]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[21] - piCur[21]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[22] - piCur[22]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[23] - piCur[23]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[24] - piCur[24]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[25] - piCur[25]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[26] - piCur[26]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[27] - piCur[27]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[28] - piCur[28]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[29] - piCur[29]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[30] - piCur[30]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[31] - piCur[31]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[32] - piCur[32]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[33] - piCur[33]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[34] - piCur[34]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[35] - piCur[35]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[36] - piCur[36]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[37] - piCur[37]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[38] - piCur[38]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[39] - piCur[39]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[40] - piCur[40]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[41] - piCur[41]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[42] - piCur[42]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[43] - piCur[43]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[44] - piCur[44]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[45] - piCur[45]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[46] - piCur[46]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[47] - piCur[47]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[48] - piCur[48]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[49] - piCur[49]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[50] - piCur[50]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[51] - piCur[51]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[52] - piCur[52]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[53] - piCur[53]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[54] - piCur[54]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[55] - piCur[55]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[56] - piCur[56]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[57] - piCur[57]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[58] - piCur[58]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[59] - piCur[59]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[60] - piCur[60]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[61] - piCur[61]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[62] - piCur[62]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[63] - piCur[63]; uiSum += ( iTemp * iTemp ) >> uiShift;
    
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}


#if QT_BT_STRUCTURE
UInt TComRdCost::xGetSSE128( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 128 );
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;

  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    //
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 10] - piCur[ 10]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 11] - piCur[ 11]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 12] - piCur[ 12]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 13] - piCur[ 13]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 14] - piCur[ 14]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 15] - piCur[ 15]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 16] - piCur[ 16]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 17] - piCur[ 17]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 18] - piCur[ 18]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 19] - piCur[ 19]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 20] - piCur[ 20]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 21] - piCur[ 21]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 22] - piCur[ 22]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 23] - piCur[ 23]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 24] - piCur[ 24]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 25] - piCur[ 25]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 26] - piCur[ 26]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 27] - piCur[ 27]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 28] - piCur[ 28]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 29] - piCur[ 29]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 30] - piCur[ 30]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 31] - piCur[ 31]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 32] - piCur[ 32]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 33] - piCur[ 33]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 34] - piCur[ 34]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 35] - piCur[ 35]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 36] - piCur[ 36]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 37] - piCur[ 37]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 38] - piCur[ 38]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 39] - piCur[ 39]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 40] - piCur[ 40]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 41] - piCur[ 41]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 42] - piCur[ 42]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 43] - piCur[ 43]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 44] - piCur[ 44]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 45] - piCur[ 45]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 46] - piCur[ 46]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 47] - piCur[ 47]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 48] - piCur[ 48]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 49] - piCur[ 49]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 50] - piCur[ 50]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 51] - piCur[ 51]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 52] - piCur[ 52]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 53] - piCur[ 53]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 54] - piCur[ 54]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 55] - piCur[ 55]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 56] - piCur[ 56]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 57] - piCur[ 57]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 58] - piCur[ 58]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 59] - piCur[ 59]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 60] - piCur[ 60]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 61] - piCur[ 61]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 62] - piCur[ 62]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 63] - piCur[ 63]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 64] - piCur[ 64]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 65] - piCur[ 65]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 66] - piCur[ 66]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 67] - piCur[ 67]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 68] - piCur[ 68]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 69] - piCur[ 69]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 70] - piCur[ 70]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 71] - piCur[ 71]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 72] - piCur[ 72]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 73] - piCur[ 73]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 74] - piCur[ 74]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 75] - piCur[ 75]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 76] - piCur[ 76]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 77] - piCur[ 77]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 78] - piCur[ 78]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 79] - piCur[ 79]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 80] - piCur[ 80]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 81] - piCur[ 81]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 82] - piCur[ 82]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 83] - piCur[ 83]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 84] - piCur[ 84]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 85] - piCur[ 85]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 86] - piCur[ 86]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 87] - piCur[ 87]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 88] - piCur[ 88]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 89] - piCur[ 89]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 90] - piCur[ 90]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 91] - piCur[ 91]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 92] - piCur[ 92]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 93] - piCur[ 93]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 94] - piCur[ 94]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 95] - piCur[ 95]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 96] - piCur[ 96]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 97] - piCur[ 97]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 98] - piCur[ 98]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 99] - piCur[ 99]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 100] - piCur[ 100]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 101] - piCur[ 101]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 102] - piCur[ 102]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 103] - piCur[ 103]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 104] - piCur[ 104]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 105] - piCur[ 105]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 106] - piCur[ 106]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 107] - piCur[ 107]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 108] - piCur[ 108]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 109] - piCur[ 109]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 110] - piCur[ 110]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 111] - piCur[ 111]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 112] - piCur[ 112]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 113] - piCur[ 113]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 114] - piCur[ 114]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 115] - piCur[ 115]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 116] - piCur[ 116]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 117] - piCur[ 117]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 118] - piCur[ 118]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 119] - piCur[ 119]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 120] - piCur[ 120]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 121] - piCur[ 121]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 122] - piCur[ 122]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 123] - piCur[ 123]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 124] - piCur[ 124]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 125] - piCur[ 125]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 126] - piCur[ 126]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 127] - piCur[ 127]; uiSum += ( iTemp * iTemp ) >> uiShift;
    //
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

UInt TComRdCost::xGetSSE256( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 256 );
    return xGetSSEw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;

  UInt uiSum = 0;
  UInt uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);
  Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    //
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 10] - piCur[ 10]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 11] - piCur[ 11]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 12] - piCur[ 12]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 13] - piCur[ 13]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 14] - piCur[ 14]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 15] - piCur[ 15]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 16] - piCur[ 16]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 17] - piCur[ 17]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 18] - piCur[ 18]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 19] - piCur[ 19]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 20] - piCur[ 20]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 21] - piCur[ 21]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 22] - piCur[ 22]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 23] - piCur[ 23]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 24] - piCur[ 24]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 25] - piCur[ 25]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 26] - piCur[ 26]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 27] - piCur[ 27]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 28] - piCur[ 28]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 29] - piCur[ 29]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 30] - piCur[ 30]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 31] - piCur[ 31]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 32] - piCur[ 32]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 33] - piCur[ 33]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 34] - piCur[ 34]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 35] - piCur[ 35]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 36] - piCur[ 36]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 37] - piCur[ 37]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 38] - piCur[ 38]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 39] - piCur[ 39]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 40] - piCur[ 40]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 41] - piCur[ 41]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 42] - piCur[ 42]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 43] - piCur[ 43]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 44] - piCur[ 44]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 45] - piCur[ 45]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 46] - piCur[ 46]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 47] - piCur[ 47]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 48] - piCur[ 48]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 49] - piCur[ 49]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 50] - piCur[ 50]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 51] - piCur[ 51]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 52] - piCur[ 52]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 53] - piCur[ 53]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 54] - piCur[ 54]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 55] - piCur[ 55]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 56] - piCur[ 56]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 57] - piCur[ 57]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 58] - piCur[ 58]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 59] - piCur[ 59]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 60] - piCur[ 60]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 61] - piCur[ 61]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 62] - piCur[ 62]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 63] - piCur[ 63]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 64] - piCur[ 64]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 65] - piCur[ 65]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 66] - piCur[ 66]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 67] - piCur[ 67]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 68] - piCur[ 68]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 69] - piCur[ 69]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 70] - piCur[ 70]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 71] - piCur[ 71]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 72] - piCur[ 72]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 73] - piCur[ 73]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 74] - piCur[ 74]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 75] - piCur[ 75]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 76] - piCur[ 76]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 77] - piCur[ 77]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 78] - piCur[ 78]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 79] - piCur[ 79]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 80] - piCur[ 80]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 81] - piCur[ 81]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 82] - piCur[ 82]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 83] - piCur[ 83]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 84] - piCur[ 84]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 85] - piCur[ 85]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 86] - piCur[ 86]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 87] - piCur[ 87]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 88] - piCur[ 88]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 89] - piCur[ 89]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 90] - piCur[ 90]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 91] - piCur[ 91]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 92] - piCur[ 92]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 93] - piCur[ 93]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 94] - piCur[ 94]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 95] - piCur[ 95]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 96] - piCur[ 96]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 97] - piCur[ 97]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 98] - piCur[ 98]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 99] - piCur[ 99]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 100] - piCur[ 100]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 101] - piCur[ 101]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 102] - piCur[ 102]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 103] - piCur[ 103]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 104] - piCur[ 104]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 105] - piCur[ 105]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 106] - piCur[ 106]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 107] - piCur[ 107]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 108] - piCur[ 108]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 109] - piCur[ 109]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 110] - piCur[ 110]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 111] - piCur[ 111]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 112] - piCur[ 112]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 113] - piCur[ 113]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 114] - piCur[ 114]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 115] - piCur[ 115]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 116] - piCur[ 116]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 117] - piCur[ 117]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 118] - piCur[ 118]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 119] - piCur[ 119]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 120] - piCur[ 120]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 121] - piCur[ 121]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 122] - piCur[ 122]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 123] - piCur[ 123]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 124] - piCur[ 124]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 125] - piCur[ 125]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 126] - piCur[ 126]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 127] - piCur[ 127]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 128] - piCur[ 128]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 129] - piCur[ 129]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 130] - piCur[ 130]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 131] - piCur[ 131]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 132] - piCur[ 132]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 133] - piCur[ 133]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 134] - piCur[ 134]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 135] - piCur[ 135]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 136] - piCur[ 136]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 137] - piCur[ 137]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 138] - piCur[ 138]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 139] - piCur[ 139]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 140] - piCur[ 140]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 141] - piCur[ 141]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 142] - piCur[ 142]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 143] - piCur[ 143]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 144] - piCur[ 144]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 145] - piCur[ 145]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 146] - piCur[ 146]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 147] - piCur[ 147]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 148] - piCur[ 148]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 149] - piCur[ 149]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 150] - piCur[ 150]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 151] - piCur[ 151]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 152] - piCur[ 152]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 153] - piCur[ 153]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 154] - piCur[ 154]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 155] - piCur[ 155]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 156] - piCur[ 156]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 157] - piCur[ 157]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 158] - piCur[ 158]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 159] - piCur[ 159]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 160] - piCur[ 160]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 161] - piCur[ 161]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 162] - piCur[ 162]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 163] - piCur[ 163]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 164] - piCur[ 164]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 165] - piCur[ 165]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 166] - piCur[ 166]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 167] - piCur[ 167]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 168] - piCur[ 168]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 169] - piCur[ 169]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 170] - piCur[ 170]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 171] - piCur[ 171]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 172] - piCur[ 172]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 173] - piCur[ 173]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 174] - piCur[ 174]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 175] - piCur[ 175]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 176] - piCur[ 176]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 177] - piCur[ 177]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 178] - piCur[ 178]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 179] - piCur[ 179]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 180] - piCur[ 180]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 181] - piCur[ 181]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 182] - piCur[ 182]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 183] - piCur[ 183]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 184] - piCur[ 184]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 185] - piCur[ 185]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 186] - piCur[ 186]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 187] - piCur[ 187]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 188] - piCur[ 188]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 189] - piCur[ 189]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 190] - piCur[ 190]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 191] - piCur[ 191]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 192] - piCur[ 192]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 193] - piCur[ 193]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 194] - piCur[ 194]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 195] - piCur[ 195]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 196] - piCur[ 196]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 197] - piCur[ 197]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 198] - piCur[ 198]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 199] - piCur[ 199]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 200] - piCur[ 200]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 201] - piCur[ 201]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 202] - piCur[ 202]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 203] - piCur[ 203]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 204] - piCur[ 204]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 205] - piCur[ 205]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 206] - piCur[ 206]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 207] - piCur[ 207]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 208] - piCur[ 208]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 209] - piCur[ 209]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 210] - piCur[ 210]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 211] - piCur[ 211]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 212] - piCur[ 212]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 213] - piCur[ 213]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 214] - piCur[ 214]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 215] - piCur[ 215]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 216] - piCur[ 216]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 217] - piCur[ 217]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 218] - piCur[ 218]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 219] - piCur[ 219]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 220] - piCur[ 220]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 221] - piCur[ 221]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 222] - piCur[ 222]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 223] - piCur[ 223]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 224] - piCur[ 224]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 225] - piCur[ 225]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 226] - piCur[ 226]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 227] - piCur[ 227]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 228] - piCur[ 228]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 229] - piCur[ 229]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 230] - piCur[ 230]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 231] - piCur[ 231]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 232] - piCur[ 232]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 233] - piCur[ 233]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 234] - piCur[ 234]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 235] - piCur[ 235]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 236] - piCur[ 236]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 237] - piCur[ 237]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 238] - piCur[ 238]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 239] - piCur[ 239]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 240] - piCur[ 240]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 241] - piCur[ 241]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 242] - piCur[ 242]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 243] - piCur[ 243]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 244] - piCur[ 244]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 245] - piCur[ 245]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 246] - piCur[ 246]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 247] - piCur[ 247]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 248] - piCur[ 248]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 249] - piCur[ 249]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 250] - piCur[ 250]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 251] - piCur[ 251]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 252] - piCur[ 252]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 253] - piCur[ 253]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 254] - piCur[ 254]; uiSum += ( iTemp * iTemp ) >> uiShift;
    iTemp = piOrg[ 255] - piCur[ 255]; uiSum += ( iTemp * iTemp ) >> uiShift;

    //
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}


#endif
// --------------------------------------------------------------------------------------------------------------------
// HADAMARD with step (used in fractional search)
// --------------------------------------------------------------------------------------------------------------------

UInt TComRdCost::xCalcHADs2x2( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep )
{
  Int satd = 0, diff[4], m[4];
  assert( iStep == 1 );
  diff[0] = piOrg[0             ] - piCur[0];
  diff[1] = piOrg[1             ] - piCur[1];
  diff[2] = piOrg[iStrideOrg    ] - piCur[0 + iStrideCur];
  diff[3] = piOrg[iStrideOrg + 1] - piCur[1 + iStrideCur];
  m[0] = diff[0] + diff[2];
  m[1] = diff[1] + diff[3];
  m[2] = diff[0] - diff[2];
  m[3] = diff[1] - diff[3];
  
  satd += abs(m[0] + m[1]);
  satd += abs(m[0] - m[1]);
  satd += abs(m[2] + m[3]);
  satd += abs(m[2] - m[3]);
  
  return satd;
}

UInt TComRdCost::xCalcHADs4x4( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep )
{
  Int k, satd = 0, diff[16], m[16], d[16];
  
  assert( iStep == 1 );
  for( k = 0; k < 16; k+=4 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];
    
    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }
  
  /*===== hadamard transform =====*/
  m[ 0] = diff[ 0] + diff[12];
  m[ 1] = diff[ 1] + diff[13];
  m[ 2] = diff[ 2] + diff[14];
  m[ 3] = diff[ 3] + diff[15];
  m[ 4] = diff[ 4] + diff[ 8];
  m[ 5] = diff[ 5] + diff[ 9];
  m[ 6] = diff[ 6] + diff[10];
  m[ 7] = diff[ 7] + diff[11];
  m[ 8] = diff[ 4] - diff[ 8];
  m[ 9] = diff[ 5] - diff[ 9];
  m[10] = diff[ 6] - diff[10];
  m[11] = diff[ 7] - diff[11];
  m[12] = diff[ 0] - diff[12];
  m[13] = diff[ 1] - diff[13];
  m[14] = diff[ 2] - diff[14];
  m[15] = diff[ 3] - diff[15];
  
  d[ 0] = m[ 0] + m[ 4];
  d[ 1] = m[ 1] + m[ 5];
  d[ 2] = m[ 2] + m[ 6];
  d[ 3] = m[ 3] + m[ 7];
  d[ 4] = m[ 8] + m[12];
  d[ 5] = m[ 9] + m[13];
  d[ 6] = m[10] + m[14];
  d[ 7] = m[11] + m[15];
  d[ 8] = m[ 0] - m[ 4];
  d[ 9] = m[ 1] - m[ 5];
  d[10] = m[ 2] - m[ 6];
  d[11] = m[ 3] - m[ 7];
  d[12] = m[12] - m[ 8];
  d[13] = m[13] - m[ 9];
  d[14] = m[14] - m[10];
  d[15] = m[15] - m[11];
  
  m[ 0] = d[ 0] + d[ 3];
  m[ 1] = d[ 1] + d[ 2];
  m[ 2] = d[ 1] - d[ 2];
  m[ 3] = d[ 0] - d[ 3];
  m[ 4] = d[ 4] + d[ 7];
  m[ 5] = d[ 5] + d[ 6];
  m[ 6] = d[ 5] - d[ 6];
  m[ 7] = d[ 4] - d[ 7];
  m[ 8] = d[ 8] + d[11];
  m[ 9] = d[ 9] + d[10];
  m[10] = d[ 9] - d[10];
  m[11] = d[ 8] - d[11];
  m[12] = d[12] + d[15];
  m[13] = d[13] + d[14];
  m[14] = d[13] - d[14];
  m[15] = d[12] - d[15];
  
  d[ 0] = m[ 0] + m[ 1];
  d[ 1] = m[ 0] - m[ 1];
  d[ 2] = m[ 2] + m[ 3];
  d[ 3] = m[ 3] - m[ 2];
  d[ 4] = m[ 4] + m[ 5];
  d[ 5] = m[ 4] - m[ 5];
  d[ 6] = m[ 6] + m[ 7];
  d[ 7] = m[ 7] - m[ 6];
  d[ 8] = m[ 8] + m[ 9];
  d[ 9] = m[ 8] - m[ 9];
  d[10] = m[10] + m[11];
  d[11] = m[11] - m[10];
  d[12] = m[12] + m[13];
  d[13] = m[12] - m[13];
  d[14] = m[14] + m[15];
  d[15] = m[15] - m[14];
  
  for (k=0; k<16; ++k)
  {
    satd += abs(d[k]);
  }
  satd = ((satd+1)>>1);
  
  return satd;
}

UInt TComRdCost::xCalcHADs8x8( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep )
{
  Int k, i, j, jj, sad=0;
  Int diff[64], m1[8][8], m2[8][8], m3[8][8];
  assert( iStep == 1 );
  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];
    diff[k+4] = piOrg[4] - piCur[4];
    diff[k+5] = piOrg[5] - piCur[5];
    diff[k+6] = piOrg[6] - piCur[6];
    diff[k+7] = piOrg[7] - piCur[7];
    
    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }
  
  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];
    
    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];
    
    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }
  
  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];
    
    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];
    
    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }
  
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      sad += abs(m2[i][j]);
    }
  }
  
  sad=((sad+2)>>2);
  
  return sad;
}


#if QT_BT_STRUCTURE
UInt TComRdCost::xCalcHADs4x8( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur )
{
  Int k, i, j, jj, sad=0;
  Int diff[32], m1[8][4], m2[8][4];
  for( k = 0; k < 32; k += 4 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 2;
    m2[j][0] = diff[jj  ] + diff[jj+2];
    m2[j][1] = diff[jj+1] + diff[jj+3];
    m2[j][2] = diff[jj  ] - diff[jj+2];
    m2[j][3] = diff[jj+1] - diff[jj+3];

    m1[j][0] = m2[j][0] + m2[j][1];
    m1[j][1] = m2[j][0] - m2[j][1];
    m1[j][2] = m2[j][2] + m2[j][3];
    m1[j][3] = m2[j][2] - m2[j][3];
  }

  //vertical
  for (i=0; i<4; i++)
  {
    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 4; j++)
    {
      sad += abs(m2[i][j]);
    }
  }

  sad=(Int)(sad/sqrt(4.0*8)*2);

  return sad;
}

UInt TComRdCost::xCalcHADs8x4( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur )
{
  Int k, i, j, jj, sad=0;
  Int diff[32], m1[4][8], m2[4][8];
  for( k = 0; k < 32; k += 8 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];
    diff[k+4] = piOrg[4] - piCur[4];
    diff[k+5] = piOrg[5] - piCur[5];
    diff[k+6] = piOrg[6] - piCur[6];
    diff[k+7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 4; j++)
  {
    jj = j << 3;

    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i=0; i < 8; i++)
  {    
    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
  }

  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 8; j++)
    {
      sad += abs(m2[i][j]);
    }
  }

  sad=(Int)(sad/sqrt(4.0*8)*2);

  return sad;
}

UInt TComRdCost::xCalcHADs8x16( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur)
{
  Int k, i, j, jj, sad=0;
  Int diff[128], m1[16][8], m2[16][8];
  for( k = 0; k < 128; k += 8 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];
    diff[k+4] = piOrg[4] - piCur[4];
    diff[k+5] = piOrg[5] - piCur[5];
    diff[k+6] = piOrg[6] - piCur[6];
    diff[k+7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 16; j++)
  {
    jj = j << 3;

    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i=0; i < 8; i++)
  {    
    m1[0][i]  = m2[0][i] + m2[8][i];
    m1[1][i]  = m2[1][i] + m2[9][i];
    m1[2][i]  = m2[2][i] + m2[10][i];
    m1[3][i]  = m2[3][i] + m2[11][i];
    m1[4][i]  = m2[4][i] + m2[12][i];
    m1[5][i]  = m2[5][i] + m2[13][i];
    m1[6][i]  = m2[6][i] + m2[14][i];
    m1[7][i]  = m2[7][i] + m2[15][i];
    m1[8][i]  = m2[0][i] - m2[8][i];
    m1[9][i]  = m2[1][i] - m2[9][i];
    m1[10][i]  = m2[2][i] - m2[10][i];
    m1[11][i]  = m2[3][i] - m2[11][i];
    m1[12][i]  = m2[4][i] - m2[12][i];
    m1[13][i]  = m2[5][i] - m2[13][i];
    m1[14][i]  = m2[6][i] - m2[14][i];
    m1[15][i]  = m2[7][i] - m2[15][i];

    m2[0][i]  = m1[0][i]  + m1[4][i];
    m2[1][i]  = m1[1][i]  + m1[5][i];
    m2[2][i]  = m1[2][i]  + m1[6][i];
    m2[3][i]  = m1[3][i]  + m1[7][i];
    m2[4][i]  = m1[0][i]  - m1[4][i];
    m2[5][i]  = m1[1][i]  - m1[5][i];
    m2[6][i]  = m1[2][i]  - m1[6][i];
    m2[7][i]  = m1[3][i]  - m1[7][i];
    m2[8][i]  = m1[8][i]  + m1[12][i];
    m2[9][i]  = m1[9][i]  + m1[13][i];
    m2[10][i] = m1[10][i] + m1[14][i];
    m2[11][i] = m1[11][i] + m1[15][i];
    m2[12][i] = m1[8][i]  - m1[12][i];
    m2[13][i] = m1[9][i]  - m1[13][i];
    m2[14][i] = m1[10][i] - m1[14][i];
    m2[15][i] = m1[11][i] - m1[15][i];

    m1[0][i]  = m2[0][i]  + m2[2][i];
    m1[1][i]  = m2[1][i]  + m2[3][i];
    m1[2][i]  = m2[0][i]  - m2[2][i];
    m1[3][i]  = m2[1][i]  - m2[3][i];
    m1[4][i]  = m2[4][i]  + m2[6][i];
    m1[5][i]  = m2[5][i]  + m2[7][i];
    m1[6][i]  = m2[4][i]  - m2[6][i];
    m1[7][i]  = m2[5][i]  - m2[7][i];
    m1[8][i]  = m2[8][i]  + m2[10][i];
    m1[9][i]  = m2[9][i]  + m2[11][i];
    m1[10][i] = m2[8][i]  - m2[10][i];
    m1[11][i] = m2[9][i]  - m2[11][i];
    m1[12][i] = m2[12][i] + m2[14][i];
    m1[13][i] = m2[13][i] + m2[15][i];
    m1[14][i] = m2[12][i] - m2[14][i];
    m1[15][i] = m2[13][i] - m2[15][i];

    m2[0][i]  = m1[0][i]  + m1[1][i];
    m2[1][i]  = m1[0][i]  - m1[1][i];
    m2[2][i]  = m1[2][i]  + m1[3][i];
    m2[3][i]  = m1[2][i]  - m1[3][i];
    m2[4][i]  = m1[4][i]  + m1[5][i];
    m2[5][i]  = m1[4][i]  - m1[5][i];
    m2[6][i]  = m1[6][i]  + m1[7][i];
    m2[7][i]  = m1[6][i]  - m1[7][i];
    m2[8][i]  = m1[8][i]  + m1[9][i];
    m2[9][i]  = m1[8][i]  - m1[9][i];
    m2[10][i] = m1[10][i] + m1[11][i];
    m2[11][i] = m1[10][i] - m1[11][i];
    m2[12][i] = m1[12][i] + m1[13][i];
    m2[13][i] = m1[12][i] - m1[13][i];
    m2[14][i] = m1[14][i] + m1[15][i];
    m2[15][i] = m1[14][i] - m1[15][i];
  }

  for (i = 0; i < 16; i++)
  {
    for (j = 0; j < 8; j++)
    {
      sad += abs(m2[i][j]);
    }
  }

  sad=(Int)(sad/sqrt(16.0*8)*2);

  return sad;
}

UInt TComRdCost::xCalcHADs16x8( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur)
{
  Int k, i, j, jj, sad=0;
  Int diff[128], m1[8][16], m2[8][16];
  for( k = 0; k < 128; k += 16 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];
    diff[k+4] = piOrg[4] - piCur[4];
    diff[k+5] = piOrg[5] - piCur[5];
    diff[k+6] = piOrg[6] - piCur[6];
    diff[k+7] = piOrg[7] - piCur[7];

    diff[k+8]  = piOrg[8]  - piCur[8] ;
    diff[k+9]  = piOrg[9]  - piCur[9] ;
    diff[k+10] = piOrg[10] - piCur[10];
    diff[k+11] = piOrg[11] - piCur[11];
    diff[k+12] = piOrg[12] - piCur[12];
    diff[k+13] = piOrg[13] - piCur[13];
    diff[k+14] = piOrg[14] - piCur[14];
    diff[k+15] = piOrg[15] - piCur[15];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 4;

    m2[j][0]  = diff[jj  ] + diff[jj+8];
    m2[j][1]  = diff[jj+1] + diff[jj+9];
    m2[j][2]  = diff[jj+2] + diff[jj+10];
    m2[j][3]  = diff[jj+3] + diff[jj+11];
    m2[j][4]  = diff[jj+4] + diff[jj+12];
    m2[j][5]  = diff[jj+5] + diff[jj+13];
    m2[j][6]  = diff[jj+6] + diff[jj+14];
    m2[j][7]  = diff[jj+7] + diff[jj+15];
    m2[j][8]  = diff[jj  ] - diff[jj+8];
    m2[j][9]  = diff[jj+1] - diff[jj+9];
    m2[j][10] = diff[jj+2] - diff[jj+10];
    m2[j][11] = diff[jj+3] - diff[jj+11];
    m2[j][12] = diff[jj+4] - diff[jj+12];
    m2[j][13] = diff[jj+5] - diff[jj+13];
    m2[j][14] = diff[jj+6] - diff[jj+14];
    m2[j][15] = diff[jj+7] - diff[jj+15];

    m1[j][0]  = m2[j][0]  + m2[j][4];
    m1[j][1]  = m2[j][1]  + m2[j][5];
    m1[j][2]  = m2[j][2]  + m2[j][6];
    m1[j][3]  = m2[j][3]  + m2[j][7];
    m1[j][4]  = m2[j][0]  - m2[j][4];
    m1[j][5]  = m2[j][1]  - m2[j][5];
    m1[j][6]  = m2[j][2]  - m2[j][6];
    m1[j][7]  = m2[j][3]  - m2[j][7];
    m1[j][8]  = m2[j][8]  + m2[j][12];
    m1[j][9]  = m2[j][9]  + m2[j][13];
    m1[j][10] = m2[j][10] + m2[j][14];
    m1[j][11] = m2[j][11] + m2[j][15];
    m1[j][12] = m2[j][8]  - m2[j][12];
    m1[j][13] = m2[j][9]  - m2[j][13];
    m1[j][14] = m2[j][10] - m2[j][14];
    m1[j][15] = m2[j][11] - m2[j][15];

    m2[j][0]  = m1[j][0]  + m1[j][2];
    m2[j][1]  = m1[j][1]  + m1[j][3];
    m2[j][2]  = m1[j][0]  - m1[j][2];
    m2[j][3]  = m1[j][1]  - m1[j][3];
    m2[j][4]  = m1[j][4]  + m1[j][6];
    m2[j][5]  = m1[j][5]  + m1[j][7];
    m2[j][6]  = m1[j][4]  - m1[j][6];
    m2[j][7]  = m1[j][5]  - m1[j][7];
    m2[j][8]  = m1[j][8]  + m1[j][10];
    m2[j][9]  = m1[j][9]  + m1[j][11];
    m2[j][10] = m1[j][8]  - m1[j][10];
    m2[j][11] = m1[j][9]  - m1[j][11];
    m2[j][12] = m1[j][12] + m1[j][14];
    m2[j][13] = m1[j][13] + m1[j][15];
    m2[j][14] = m1[j][12] - m1[j][14];
    m2[j][15] = m1[j][13] - m1[j][15];

    m1[j][0]  = m2[j][0]  + m2[j][1];
    m1[j][1]  = m2[j][0]  - m2[j][1];
    m1[j][2]  = m2[j][2]  + m2[j][3];
    m1[j][3]  = m2[j][2]  - m2[j][3];
    m1[j][4]  = m2[j][4]  + m2[j][5];
    m1[j][5]  = m2[j][4]  - m2[j][5];
    m1[j][6]  = m2[j][6]  + m2[j][7];
    m1[j][7]  = m2[j][6]  - m2[j][7];
    m1[j][8]  = m2[j][8]  + m2[j][9];
    m1[j][9]  = m2[j][8]  - m2[j][9];
    m1[j][10] = m2[j][10] + m2[j][11];
    m1[j][11] = m2[j][10] - m2[j][11];
    m1[j][12] = m2[j][12] + m2[j][13];
    m1[j][13] = m2[j][12] - m2[j][13];
    m1[j][14] = m2[j][14] + m2[j][15];
    m1[j][15] = m2[j][14] - m2[j][15];
  }

  //vertical
  for (i=0; i < 16; i++)
  {    
    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 16; j++)
    {
      sad += abs(m2[i][j]);
    }
  }

  sad=(Int)(sad/sqrt(16.0*8)*2);

  return sad;
}
UInt TComRdCost::xCalcHADs16x4( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep )
{
  Int k, i, j, jj, sad=0;
  Int diff[64], m1[4][16], m2[4][16];
  assert( iStep == 1 );
  for( k = 0; k < 64; k += 16 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];
    diff[k+4] = piOrg[4] - piCur[4];
    diff[k+5] = piOrg[5] - piCur[5];
    diff[k+6] = piOrg[6] - piCur[6];
    diff[k+7] = piOrg[7] - piCur[7];

    diff[k+8]  = piOrg[8]  - piCur[8] ;
    diff[k+9]  = piOrg[9]  - piCur[9] ;
    diff[k+10] = piOrg[10] - piCur[10];
    diff[k+11] = piOrg[11] - piCur[11];
    diff[k+12] = piOrg[12] - piCur[12];
    diff[k+13] = piOrg[13] - piCur[13];
    diff[k+14] = piOrg[14] - piCur[14];
    diff[k+15] = piOrg[15] - piCur[15];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 4; j++)
  {
    jj = j << 4;

    m2[j][0]  = diff[jj  ] + diff[jj+8];
    m2[j][1]  = diff[jj+1] + diff[jj+9];
    m2[j][2]  = diff[jj+2] + diff[jj+10];
    m2[j][3]  = diff[jj+3] + diff[jj+11];
    m2[j][4]  = diff[jj+4] + diff[jj+12];
    m2[j][5]  = diff[jj+5] + diff[jj+13];
    m2[j][6]  = diff[jj+6] + diff[jj+14];
    m2[j][7]  = diff[jj+7] + diff[jj+15];
    m2[j][8]  = diff[jj  ] - diff[jj+8];
    m2[j][9]  = diff[jj+1] - diff[jj+9];
    m2[j][10] = diff[jj+2] - diff[jj+10];
    m2[j][11] = diff[jj+3] - diff[jj+11];
    m2[j][12] = diff[jj+4] - diff[jj+12];
    m2[j][13] = diff[jj+5] - diff[jj+13];
    m2[j][14] = diff[jj+6] - diff[jj+14];
    m2[j][15] = diff[jj+7] - diff[jj+15];

    m1[j][0]  = m2[j][0]  + m2[j][4];
    m1[j][1]  = m2[j][1]  + m2[j][5];
    m1[j][2]  = m2[j][2]  + m2[j][6];
    m1[j][3]  = m2[j][3]  + m2[j][7];
    m1[j][4]  = m2[j][0]  - m2[j][4];
    m1[j][5]  = m2[j][1]  - m2[j][5];
    m1[j][6]  = m2[j][2]  - m2[j][6];
    m1[j][7]  = m2[j][3]  - m2[j][7];
    m1[j][8]  = m2[j][8]  + m2[j][12];
    m1[j][9]  = m2[j][9]  + m2[j][13];
    m1[j][10] = m2[j][10] + m2[j][14];
    m1[j][11] = m2[j][11] + m2[j][15];
    m1[j][12] = m2[j][8]  - m2[j][12];
    m1[j][13] = m2[j][9]  - m2[j][13];
    m1[j][14] = m2[j][10] - m2[j][14];
    m1[j][15] = m2[j][11] - m2[j][15];

    m2[j][0]  = m1[j][0]  + m1[j][2];
    m2[j][1]  = m1[j][1]  + m1[j][3];
    m2[j][2]  = m1[j][0]  - m1[j][2];
    m2[j][3]  = m1[j][1]  - m1[j][3];
    m2[j][4]  = m1[j][4]  + m1[j][6];
    m2[j][5]  = m1[j][5]  + m1[j][7];
    m2[j][6]  = m1[j][4]  - m1[j][6];
    m2[j][7]  = m1[j][5]  - m1[j][7];
    m2[j][8]  = m1[j][8]  + m1[j][10];
    m2[j][9]  = m1[j][9]  + m1[j][11];
    m2[j][10] = m1[j][8]  - m1[j][10];
    m2[j][11] = m1[j][9]  - m1[j][11];
    m2[j][12] = m1[j][12] + m1[j][14];
    m2[j][13] = m1[j][13] + m1[j][15];
    m2[j][14] = m1[j][12] - m1[j][14];
    m2[j][15] = m1[j][13] - m1[j][15];

    m1[j][0]  = m2[j][0]  + m2[j][1];
    m1[j][1]  = m2[j][0]  - m2[j][1];
    m1[j][2]  = m2[j][2]  + m2[j][3];
    m1[j][3]  = m2[j][2]  - m2[j][3];
    m1[j][4]  = m2[j][4]  + m2[j][5];
    m1[j][5]  = m2[j][4]  - m2[j][5];
    m1[j][6]  = m2[j][6]  + m2[j][7];
    m1[j][7]  = m2[j][6]  - m2[j][7];
    m1[j][8]  = m2[j][8]  + m2[j][9];
    m1[j][9]  = m2[j][8]  - m2[j][9];
    m1[j][10] = m2[j][10] + m2[j][11];
    m1[j][11] = m2[j][10] - m2[j][11];
    m1[j][12] = m2[j][12] + m2[j][13];
    m1[j][13] = m2[j][12] - m2[j][13];
    m1[j][14] = m2[j][14] + m2[j][15];
    m1[j][15] = m2[j][14] - m2[j][15];
  }

  //vertical
  for (i=0; i < 16; i++)
  {    
    m2[0][i] = m1[0][i] + m1[2][i];
    m2[1][i] = m1[1][i] + m1[3][i];
    m2[2][i] = m1[0][i] - m1[2][i];
    m2[3][i] = m1[1][i] - m1[3][i];

    m1[0][i] = m2[0][i] + m2[1][i];
    m1[1][i] = m2[0][i] - m2[1][i];
    m1[2][i] = m2[2][i] + m2[3][i];
    m1[3][i] = m2[2][i] - m2[3][i];
  }

  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 16; j++)
    {
      sad += abs(m1[i][j]);
    }
  }

  sad=((sad+2)>>2);

  return sad;
}

UInt TComRdCost::xCalcHADs4x16( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep )
{
  Int k, i, j, jj, sad=0;
  Int diff[64], m1[16][4], m2[16][4], m3[16][4];
  assert( iStep == 1 );
  for( k = 0; k < 64; k += 4 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 16; j++)
  {
    jj = j << 2;
    m2[j][0] = diff[jj  ] + diff[jj+2];
    m2[j][1] = diff[jj+1] + diff[jj+3];
    m2[j][2] = diff[jj  ] - diff[jj+2];
    m2[j][3] = diff[jj+1] - diff[jj+3];

    m1[j][0] = m2[j][0] + m2[j][1];
    m1[j][1] = m2[j][0] - m2[j][1];
    m1[j][2] = m2[j][2] + m2[j][3];
    m1[j][3] = m2[j][2] - m2[j][3];
  }

  //vertical
  for (i=0; i < 4; i++)
  {
    m2[0][i]  = m1[0][i] + m1[8][i];
    m2[1][i]  = m1[1][i] + m1[9][i];
    m2[2][i]  = m1[2][i] + m1[10][i];
    m2[3][i]  = m1[3][i] + m1[11][i];
    m2[4][i]  = m1[4][i] + m1[12][i];
    m2[5][i]  = m1[5][i] + m1[13][i];
    m2[6][i]  = m1[6][i] + m1[14][i];
    m2[7][i]  = m1[7][i] + m1[15][i];
    m2[8][i]  = m1[0][i] - m1[8][i];
    m2[9][i]  = m1[1][i] - m1[9][i];
    m2[10][i] = m1[2][i] - m1[10][i];
    m2[11][i] = m1[3][i] - m1[11][i];
    m2[12][i] = m1[4][i] - m1[12][i];
    m2[13][i] = m1[5][i] - m1[13][i];
    m2[14][i] = m1[6][i] - m1[14][i];
    m2[15][i] = m1[7][i] - m1[15][i];

    m3[0][i]  = m2[0][i]  + m2[4][i];
    m3[1][i]  = m2[1][i]  + m2[5][i];
    m3[2][i]  = m2[2][i]  + m2[6][i];
    m3[3][i]  = m2[3][i]  + m2[7][i];
    m3[4][i]  = m2[0][i]  - m2[4][i];
    m3[5][i]  = m2[1][i]  - m2[5][i];
    m3[6][i]  = m2[2][i]  - m2[6][i];
    m3[7][i]  = m2[3][i]  - m2[7][i];
    m3[8][i]  = m2[8][i]  + m2[12][i];
    m3[9][i]  = m2[9][i]  + m2[13][i];
    m3[10][i] = m2[10][i] + m2[14][i];
    m3[11][i] = m2[11][i] + m2[15][i];
    m3[12][i] = m2[8][i]  - m2[12][i];
    m3[13][i] = m2[9][i]  - m2[13][i];
    m3[14][i] = m2[10][i] - m2[14][i];
    m3[15][i] = m2[11][i] - m2[15][i];

    m1[0][i]  = m3[0][i]  + m3[2][i];
    m1[1][i]  = m3[1][i]  + m3[3][i];
    m1[2][i]  = m3[0][i]  - m3[2][i];
    m1[3][i]  = m3[1][i]  - m3[3][i];
    m1[4][i]  = m3[4][i]  + m3[6][i];
    m1[5][i]  = m3[5][i]  + m3[7][i];
    m1[6][i]  = m3[4][i]  - m3[6][i];
    m1[7][i]  = m3[5][i]  - m3[7][i];
    m1[8][i]  = m3[8][i]  + m3[10][i];
    m1[9][i]  = m3[9][i]  + m3[11][i];
    m1[10][i] = m3[8][i]  - m3[10][i];
    m1[11][i] = m3[9][i]  - m3[11][i];
    m1[12][i] = m3[12][i] + m3[14][i];
    m1[13][i] = m3[13][i] + m3[15][i];
    m1[14][i] = m3[12][i] - m3[14][i];
    m1[15][i] = m3[13][i] - m3[15][i];

    m2[0][i]  = m1[0][i]  + m1[1][i];
    m2[1][i]  = m1[0][i]  - m1[1][i];
    m2[2][i]  = m1[2][i]  + m1[3][i];
    m2[3][i]  = m1[2][i]  - m1[3][i];
    m2[4][i]  = m1[4][i]  + m1[5][i];
    m2[5][i]  = m1[4][i]  - m1[5][i];
    m2[6][i]  = m1[6][i]  + m1[7][i];
    m2[7][i]  = m1[6][i]  - m1[7][i];
    m2[8][i]  = m1[8][i]  + m1[9][i];
    m2[9][i]  = m1[8][i]  - m1[9][i];
    m2[10][i] = m1[10][i] + m1[11][i];
    m2[11][i] = m1[10][i] - m1[11][i];
    m2[12][i] = m1[12][i] + m1[13][i];
    m2[13][i] = m1[12][i] - m1[13][i];
    m2[14][i] = m1[14][i] + m1[15][i];
    m2[15][i] = m1[14][i] - m1[15][i];
  }

  for (i = 0; i < 16; i++)
  {
    for (j = 0; j < 4; j++)
    {
      sad += abs(m2[i][j]);
    }
  }

  sad=((sad+2)>>2);

  return sad;
}
#endif

UInt TComRdCost::xGetHADs4( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetHADs4w( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideCur = pcDtParam->iStrideCur;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStep  = pcDtParam->iStep;
  Int  y;
  Int  iOffsetOrg = iStrideOrg<<2;
  Int  iOffsetCur = iStrideCur<<2;
  
  UInt uiSum = 0;
  
  for ( y=0; y<iRows; y+= 4 )
  {
    uiSum += xCalcHADs4x4( piOrg, piCur, iStrideOrg, iStrideCur, iStep );
    piOrg += iOffsetOrg;
    piCur += iOffsetCur;
  }
  
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetHADs8( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetHADs8w( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideCur = pcDtParam->iStrideCur;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStep  = pcDtParam->iStep;
  Int  y;
  
  UInt uiSum = 0;
  
  if ( iRows == 4 )
  {
    uiSum += xCalcHADs4x4( piOrg+0, piCur        , iStrideOrg, iStrideCur, iStep );
    uiSum += xCalcHADs4x4( piOrg+4, piCur+4*iStep, iStrideOrg, iStrideCur, iStep );
  }
  else
  {
    Int  iOffsetOrg = iStrideOrg<<3;
    Int  iOffsetCur = iStrideCur<<3;
    for ( y=0; y<iRows; y+= 8 )
    {
      uiSum += xCalcHADs8x8( piOrg, piCur, iStrideOrg, iStrideCur, iStep );
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

UInt TComRdCost::xGetHADs( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return xGetHADsw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideCur = pcDtParam->iStrideCur;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStep  = pcDtParam->iStep;
  
  Int  x, y;
  
  UInt uiSum = 0;
  
#if QT_BT_STRUCTURE
  if( ( iRows % 8 == 0) && (iCols % 8 == 0) && ( iRows == iCols ) )
#else
  if( ( iRows % 8 == 0) && (iCols % 8 == 0) )
#endif
  {
    Int  iOffsetOrg = iStrideOrg<<3;
    Int  iOffsetCur = iStrideCur<<3;
    for ( y=0; y<iRows; y+= 8 )
    {
      for ( x=0; x<iCols; x+= 8 )
      {
        uiSum += xCalcHADs8x8( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
#if QT_BT_STRUCTURE
  else if ( ( iCols > 8 ) && ( iCols > iRows ) ) 
  {
    Int  iOffsetOrg = iStrideOrg<<2;
    Int  iOffsetCur = iStrideCur<<2;
    for ( y=0; y<iRows; y+= 4 )
    {
      for ( x=0; x<iCols; x+= 16 )
      {
        uiSum += xCalcHADs16x4( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else if ( ( iRows > 8 ) && ( iCols < iRows ) ) 
  {
    Int  iOffsetOrg = iStrideOrg<<4;
    Int  iOffsetCur = iStrideCur<<4;
    for ( y=0; y<iRows; y+= 16 )
    {
      for ( x=0; x<iCols; x+= 4 )
      {
        uiSum += xCalcHADs4x16( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
#endif
  else if( ( iRows % 4 == 0) && (iCols % 4 == 0) )
  {
    Int  iOffsetOrg = iStrideOrg<<2;
    Int  iOffsetCur = iStrideCur<<2;
    
    for ( y=0; y<iRows; y+= 4 )
    {
      for ( x=0; x<iCols; x+= 4 )
      {
        uiSum += xCalcHADs4x4( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else if( ( iRows % 2 == 0) && (iCols % 2 == 0) )
  {
    Int  iOffsetOrg = iStrideOrg<<1;
    Int  iOffsetCur = iStrideCur<<1;
    for ( y=0; y<iRows; y+=2 )
    {
      for ( x=0; x<iCols; x+=2 )
      {
        uiSum += xCalcHADs2x2( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else
  {
    assert(false);
  }
  
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);
}

//! \}
