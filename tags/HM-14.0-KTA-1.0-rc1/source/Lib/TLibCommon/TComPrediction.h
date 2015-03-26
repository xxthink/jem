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

/** \file     TComPrediction.h
    \brief    prediction class (header)
*/

#ifndef __TCOMPREDICTION__
#define __TCOMPREDICTION__


// Include files
#include "TComPic.h"
#include "TComMotionInfo.h"
#include "TComPattern.h"
#include "TComTrQuant.h"
#include "TComInterpolationFilter.h"
#include "TComWeightPrediction.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
class TComPrediction : public TComWeightPrediction
{
protected:
  Int*      m_piYuvExt;
  Int       m_iYuvExtStride;
  Int       m_iYuvExtHeight;
  
  TComYuv   m_acYuvPred[2];
  TComYuv   m_cYuvPredTemp;
  TComYuv m_filteredBlock[4][4];
  TComYuv m_filteredBlockTmp[4];
  
  TComInterpolationFilter m_if;

#if QC_LMCHROMA
  UInt m_uiaLMShift[ 32 ];       // Table for multiplication to substitue of division operation
#endif
 
  Int*   m_pLumaRecBuffer;       ///< array for downsampled reconstructed luma sample 
  Int    m_iLumaRecStride;       ///< stride of #m_pLumaRecBuffer array

  Void xPredIntraAng            (Int bitDepth, Int* pSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter );
  Void xPredIntraPlanar         ( Int* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height );
  
  // motion compensation functions
  Void xPredInterUni            ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv*& rpcYuvPred, Bool bi=false          );
  Void xPredInterBi             ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight,                         TComYuv*& rpcYuvPred );
  Void xPredInterLumaBlk  ( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi );
  Void xPredInterChromaBlk( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi );
  Void xWeightedAverage         ( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv*& rpcYuvDst );
  
  Void xDCPredFiltering( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight );
  Bool xCheckIdenticalMotion    ( TComDataCU* pcCU, UInt PartAddr);
#if QC_SUB_PU_TMVP
  Bool xCheckTwoSPMotion ( TComDataCU* pcCU, UInt PartAddr0, UInt PartAddr1 );
  Void xGetSubPUAddrAndMerge(TComDataCU* pcCU, UInt uiPartAddr, Int iSPWidth, Int iSPHeight, Int iNumSPInOneLine, Int iNumSP, UInt* uiMergedSPW, UInt* uiMergedSPH, UInt* uiSPAddr );
#endif

#if QC_OBMC
  Void xSubblockOBMC ( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp );
  Void xSubtractOBMC ( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp );
  Void xSubBlockMotionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, Int uiPartAddr, Int iWidth, Int iHeight  );
#endif
public:
  TComPrediction();
  virtual ~TComPrediction();
  
  Void    initTempBuff();
  
  // inter
#if QC_OBMC
  Void subBlockOBMC               ( TComDataCU*  pcCU, UInt uiAbsPartIdx, TComYuv *pcYuvPred, TComYuv *pcYuvTmpPred1, TComYuv *pcYuvTmpPred2, Bool bOBMC4ME = false );
#endif
  Void motionCompensation         ( TComDataCU*  pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList = REF_PIC_LIST_X, Int iPartIdx = -1 );

  // motion vector prediction
  Void getMvPredAMVP              ( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred );
  
  // Angular Intra
  Void predIntraLumaAng           ( TComPattern* pcTComPattern, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft );
  Void predIntraChromaAng         ( Int* piSrc, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft );
  
  Pel  predIntraGetPredValDC      ( Int* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft );
  
  Int* getPredicBuf()             { return m_piYuvExt;      }
  Int  getPredicBufWidth()        { return m_iYuvExtStride; }
  Int  getPredicBufHeight()       { return m_iYuvExtHeight; }

#if QC_LMCHROMA
  Void predLMIntraChroma ( TComPattern* pcPattern, UInt uiChromaId, Pel* pPred, UInt uiPredStride, UInt uiCWidth, UInt uiCHeight );
  Void getLumaRecPixels  ( TComPattern* pcPattern, UInt uiCWidth, UInt uiCHeight, Bool bLeftPicBoundary );
  Void addCrossColorResi ( TComPattern* pcPattern, Pel* piPred, UInt uiPredStride, UInt uiWidth, UInt uiHeight, Pel* piResi, UInt uiResiStride );
  Void xGetLMParameters  ( TComPattern* pcPattern, UInt uiWidth, UInt uiHeight, Int iPredType, UInt uiChromaId, Int &a, Int &b, Int &iShift );
  Void xCalcLMParameters ( Int x, Int y, Int xx, Int xy, Int iCountShift, Int iPredType, Int &a, Int &b, Int &iShift );
#endif
};

//! \}

#endif // __TCOMPREDICTION__
