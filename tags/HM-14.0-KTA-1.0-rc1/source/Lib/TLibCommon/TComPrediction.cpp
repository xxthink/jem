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

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()
: m_pLumaRecBuffer(0)
, m_iLumaRecStride(0)
{
  m_piYuvExt = NULL;
}

TComPrediction::~TComPrediction()
{
  
  delete[] m_piYuvExt;

  m_acYuvPred[0].destroy();
  m_acYuvPred[1].destroy();

  m_cYuvPredTemp.destroy();

  if( m_pLumaRecBuffer )
  {
    delete [] m_pLumaRecBuffer;
  }
  
  Int i, j;
  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 4; j++)
    {
      m_filteredBlock[i][j].destroy();
    }
    m_filteredBlockTmp[i].destroy();
  }
}

Void TComPrediction::initTempBuff()
{
  if( m_piYuvExt == NULL )
  {
#if QC_LARGE_CTU
    Int extWidth  = g_uiMaxCUWidth + 16; 
    Int extHeight = g_uiMaxCUHeight + 1;
#else
    Int extWidth  = MAX_CU_SIZE + 16; 
    Int extHeight = MAX_CU_SIZE + 1;
#endif
    Int i, j;
    for (i = 0; i < 4; i++)
    {
      m_filteredBlockTmp[i].create(extWidth, extHeight + 7);
      for (j = 0; j < 4; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight);
      }
    }
#if QC_LARGE_CTU
    m_iYuvExtHeight  = ((g_uiMaxCUHeight + 2) << 4);
    m_iYuvExtStride = ((g_uiMaxCUWidth  + 8) << 4);
#else
    m_iYuvExtHeight  = ((MAX_CU_SIZE + 2) << 4);
    m_iYuvExtStride = ((MAX_CU_SIZE  + 8) << 4);
#endif
    m_piYuvExt = new Int[ m_iYuvExtStride * m_iYuvExtHeight ];

    // new structure
#if QC_LARGE_CTU
    m_acYuvPred[0] .create( g_uiMaxCUWidth, g_uiMaxCUHeight );
    m_acYuvPred[1] .create( g_uiMaxCUWidth, g_uiMaxCUHeight );

    m_cYuvPredTemp.create( g_uiMaxCUWidth, g_uiMaxCUHeight );
#else
    m_acYuvPred[0] .create( MAX_CU_SIZE, MAX_CU_SIZE );
    m_acYuvPred[1] .create( MAX_CU_SIZE, MAX_CU_SIZE );

    m_cYuvPredTemp.create( MAX_CU_SIZE, MAX_CU_SIZE );
#endif
  }

#if QC_LARGE_CTU
  if (m_iLumaRecStride != (g_uiMaxCUWidth>>1) + 1)
  {
    m_iLumaRecStride =  (g_uiMaxCUWidth>>1) + 1;
#else
  if (m_iLumaRecStride != (MAX_CU_SIZE>>1) + 1)
  {
    m_iLumaRecStride =  (MAX_CU_SIZE>>1) + 1;
#endif
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Int[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }

#if QC_LMCHROMA
  Int shift = g_bitDepthY + 4; //??????????
  for( Int i = 32; i < 64; i++ )
  {
    m_uiaLMShift[i-32] = ( ( 1 << shift ) + i/2 ) / i;
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
Pel TComPrediction::predIntraGetPredValDC( Int* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft )
{
  assert(iWidth > 0 && iHeight > 0);
  Int iInd, iSum = 0;
  Pel pDcVal;

  if (bAbove)
  {
    for (iInd = 0;iInd < iWidth;iInd++)
    {
      iSum += pSrc[iInd-iSrcStride];
    }
  }
  if (bLeft)
  {
    for (iInd = 0;iInd < iHeight;iInd++)
    {
      iSum += pSrc[iInd*iSrcStride-1];
    }
  }

  if (bAbove && bLeft)
  {
    pDcVal = (iSum + iWidth) / (iWidth + iHeight);
  }
  else if (bAbove)
  {
    pDcVal = (iSum + iWidth/2) / iWidth;
  }
  else if (bLeft)
  {
    pDcVal = (iSum + iHeight/2) / iHeight;
  }
  else
  {
    pDcVal = pSrc[-1]; // Default DC value already calculated and placed in the prediction array if no neighbors are available
  }
  
  return pDcVal;
}

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 * \param dirMode the intra prediction mode index
 * \param blkAboveAvailable boolean indication if the block above is available
 * \param blkLeftAvailable boolean indication if the block to the left is available
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
Void TComPrediction::xPredIntraAng(Int bitDepth, Int* pSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter )
{
  Int k,l;
  Int blkSize        = width;
  Pel* pDst          = rpDst;

  // Map the mode index to main prediction direction and angle
  assert( dirMode > 0 ); //no planar
  Bool modeDC        = dirMode < 2;
  Bool modeHor       = !modeDC && (dirMode < 18);
  Bool modeVer       = !modeDC && !modeHor;
  Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
  Int absAng         = abs(intraPredAngle);
  Int signAng        = intraPredAngle < 0 ? -1 : 1;

  // Set bitshifts and scale the angle parameter to block size
  Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
  Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
  Int invAngle       = invAngTable[absAng];
  absAng             = angTable[absAng];
  intraPredAngle     = signAng * absAng;

  // Do the DC prediction
  if (modeDC)
  {
    Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

    for (k=0;k<blkSize;k++)
    {
      for (l=0;l<blkSize;l++)
      {
        pDst[k*dstStride+l] = dcval;
      }
    }
  }

  // Do angular predictions
  else
  {
    Pel* refMain;
    Pel* refSide;
    Pel  refAbove[2*MAX_CU_SIZE+1];
    Pel  refLeft[2*MAX_CU_SIZE+1];

    // Initialise the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      for (k=0;k<blkSize+1;k++)
      {
        refAbove[k+blkSize-1] = pSrc[k-srcStride-1];
      }
      for (k=0;k<blkSize+1;k++)
      {
        refLeft[k+blkSize-1] = pSrc[(k-1)*srcStride-1];
      }
      refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
      refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)
      for (k=-1; k>blkSize*intraPredAngle>>5; k--)
      {
        invAngleSum += invAngle;
        refMain[k] = refSide[invAngleSum>>8];
      }
    }
    else
    {
      for (k=0;k<2*blkSize+1;k++)
      {
        refAbove[k] = pSrc[k-srcStride-1];
      }
      for (k=0;k<2*blkSize+1;k++)
      {
        refLeft[k] = pSrc[(k-1)*srcStride-1];
      }
      refMain = modeVer ? refAbove : refLeft;
      refSide = modeVer ? refLeft  : refAbove;
    }

    if (intraPredAngle == 0)
    {
      for (k=0;k<blkSize;k++)
      {
        for (l=0;l<blkSize;l++)
        {
          pDst[k*dstStride+l] = refMain[l+1];
        }
      }

      if ( bFilter )
      {
        for (k=0;k<blkSize;k++)
        {
          pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
        }
      }
    }
    else
    {
      Int deltaPos=0;
      Int deltaInt;
      Int deltaFract;
      Int refMainIndex;

      for (k=0;k<blkSize;k++)
      {
        deltaPos += intraPredAngle;
        deltaInt   = deltaPos >> 5;
        deltaFract = deltaPos & (32 - 1);

        if (deltaFract)
        {
          // Do linear filtering
          for (l=0;l<blkSize;l++)
          {
            refMainIndex        = l+deltaInt+1;
            pDst[k*dstStride+l] = (Pel) ( ((32-deltaFract)*refMain[refMainIndex]+deltaFract*refMain[refMainIndex+1]+16) >> 5 );
          }
        }
        else
        {
          // Just copy the integer samples
          for (l=0;l<blkSize;l++)
          {
            pDst[k*dstStride+l] = refMain[l+deltaInt+1];
          }
        }
      }
    }

    // Flip the block if this is the horizontal mode
    if (modeHor)
    {
      Pel  tmp;
      for (k=0;k<blkSize-1;k++)
      {
        for (l=k+1;l<blkSize;l++)
        {
          tmp                 = pDst[k*dstStride+l];
          pDst[k*dstStride+l] = pDst[l*dstStride+k];
          pDst[l*dstStride+k] = tmp;
        }
      }
    }
  }
}

Void TComPrediction::predIntraLumaAng(TComPattern* pcTComPattern, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft )
{
  Pel *pDst = piPred;
  Int *ptrSrc;

  assert( g_aucConvertToBit[ iWidth ] >= 0 ); //   4x  4
#if !QC_LARGE_CTU
  assert( g_aucConvertToBit[ iWidth ] <= 5 ); // 128x128
#endif
  assert( iWidth == iHeight  );

  ptrSrc = pcTComPattern->getPredictorPtr( uiDirMode, g_aucConvertToBit[ iWidth ] + 2, m_piYuvExt );

  // get starting pixel in block
  Int sw = 2 * iWidth + 1;

  // Create the prediction
  if ( uiDirMode == PLANAR_IDX )
  {
    xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
  }
  else
  {
    if ( (iWidth > 16) || (iHeight > 16) )
    {
      xPredIntraAng(g_bitDepthY, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false );
    }
    else
    {
      xPredIntraAng(g_bitDepthY, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, true );

      if( (uiDirMode == DC_IDX ) && bAbove && bLeft )
      {
        xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight);
      }
    }
  }
}

// Angular chroma
Void TComPrediction::predIntraChromaAng( Int* piSrc, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft )
{
  Pel *pDst = piPred;
  Int *ptrSrc = piSrc;

  // get starting pixel in block
  Int sw = 2 * iWidth + 1;

  if ( uiDirMode == PLANAR_IDX )
  {
    xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
  }
  else
  {
    // Create the prediction
    xPredIntraAng(g_bitDepthC, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false );
  }
}

/** Function for checking identical motion.
 * \param TComDataCU* pcCU
 * \param UInt PartAddr
 */
Bool TComPrediction::xCheckIdenticalMotion ( TComDataCU* pcCU, UInt PartAddr )
{
  if( pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred() )
  {
    if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)
    {
      Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
      Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
      if(RefPOCL0 == RefPOCL1 && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr))
      {
        return true;
      }
    }
  }
  return false;
}

#if QC_SUB_PU_TMVP
Void TComPrediction::xGetSubPUAddrAndMerge(TComDataCU* pcCU, UInt uiPartAddr, Int iSPWidth, Int iSPHeight, Int iNumSPInOneLine, Int iNumSP, UInt* uiMergedSPW, UInt* uiMergedSPH, UInt* uiSPAddr )
{
  for (Int i = 0; i < iNumSP; i++)
  {
    uiMergedSPW[i] = iSPWidth;
    uiMergedSPH[i] = iSPHeight;
    pcCU->getSPAbsPartIdx(uiPartAddr, iSPWidth, iSPHeight, i, iNumSPInOneLine, uiSPAddr[i]);
  }

  // horizontal sub-PU merge
  for (Int i=0; i<iNumSP; i++)
  {
    if (i % iNumSPInOneLine == iNumSPInOneLine - 1 || uiMergedSPW[i]==0 || uiMergedSPH[i]==0)
    {
      continue;
    }
    for (Int j=i+1; j<i+iNumSPInOneLine-i%iNumSPInOneLine; j++)
    {
      if (xCheckTwoSPMotion(pcCU, uiSPAddr[i], uiSPAddr[j]))
      {
        uiMergedSPW[i] += iSPWidth;
        uiMergedSPW[j] = uiMergedSPH[j] = 0;
      }
      else
      {
        break;
      }
    }
  }
  //vertical sub-PU merge
  for (Int i=0; i<iNumSP-iNumSPInOneLine; i++)
  {
    if (uiMergedSPW[i]==0 || uiMergedSPH[i]==0)
    {
      continue;
    }
    for (Int j=i+iNumSPInOneLine; j<iNumSP; j+=iNumSPInOneLine)
    {
      if (xCheckTwoSPMotion(pcCU, uiSPAddr[i], uiSPAddr[j]) && uiMergedSPW[i]==uiMergedSPW[j])
      {
        uiMergedSPH[i] += iSPHeight;
        uiMergedSPH[j] = uiMergedSPW[j] = 0;
      }
      else
      {
        break;
      }
    }
  }
}
Bool TComPrediction::xCheckTwoSPMotion ( TComDataCU* pcCU, UInt PartAddr0, UInt PartAddr1 )
{
  if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr1))
  {
    return false;
  }
  if( pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr1))
  {
    return false;
  }

  if (pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr0) >= 0)
  {
    if (pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr1))
    {
      return false;
    }
  }

  if (pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr0) >= 0)
  {
    if (pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr1))
    {
      return false;
    }
  }
  return true;
}
#endif

Void TComPrediction::motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList, Int iPartIdx )
{
  Int         iWidth;
  Int         iHeight;
  UInt        uiPartAddr;

  if ( iPartIdx >= 0 )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );
    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
#if QC_SUB_PU_TMVP
      if ( pcCU->getMergeType(uiPartAddr)== MGR_TYPE_SUBPU_TMVP )  
      {
        Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;

        pcCU->getSPPara(iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);

#if QC_LARGE_CTU
        UInt uiW[MAX_NUM_SPU_W*MAX_NUM_SPU_W], uiH[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
        UInt uiSPAddr[MAX_NUM_SPU_W*MAX_NUM_SPU_W];

#else
        UInt uiW[256], uiH[256];
        UInt uiSPAddr[256];
#endif

        xGetSubPUAddrAndMerge(pcCU, uiPartAddr, iSPWidth, iSPHeight, iNumSPInOneLine, iNumSP, uiW, uiH, uiSPAddr);

        //MC
        for (Int i = 0; i < iNumSP; i++)
        {
          if (uiW[i]==0 || uiH[i]==0)
          {
            continue;
          }

          if(xCheckIdenticalMotion( pcCU, uiSPAddr[i] )/*0*/)
          {
            xPredInterUni (pcCU, uiSPAddr[i], uiW[i], uiH[i], REF_PIC_LIST_0, pcYuvPred );
          }
          else
          {
            xPredInterBi  (pcCU, uiSPAddr[i], uiW[i], uiH[i], pcYuvPred);
          }
        }
      }
      else
      {
#endif
        if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
        {
          xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
        }
        else
        {
          xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
        }
#if QC_SUB_PU_TMVP
      }
#endif
    }
    return;
  }

  for ( iPartIdx = 0; iPartIdx < pcCU->getNumPartitions(); iPartIdx++ )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );

    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    { 
#if QC_SUB_PU_TMVP
      if (pcCU->getMergeType(uiPartAddr) == MGR_TYPE_SUBPU_TMVP)  
      {
        Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;

        pcCU->getSPPara(iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);

#if QC_LARGE_CTU
        UInt uiW[MAX_NUM_SPU_W*MAX_NUM_SPU_W], uiH[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
        UInt uiSPAddr[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
#else
        UInt uiW[256], uiH[256];
        UInt uiSPAddr[256];
#endif

        xGetSubPUAddrAndMerge(pcCU, uiPartAddr, iSPWidth, iSPHeight, iNumSPInOneLine, iNumSP, uiW, uiH, uiSPAddr);
        //MC
        for (Int i = 0; i < iNumSP; i++)
        {
          if (uiW[i]==0 || uiH[i]==0)
          {
            continue;
          }
          if( xCheckIdenticalMotion( pcCU, uiSPAddr[i] )/*0*/)
          {
            xPredInterUni (pcCU, uiSPAddr[i], uiW[i], uiH[i], REF_PIC_LIST_0, pcYuvPred );
          }
          else
          {
            xPredInterBi  (pcCU, uiSPAddr[i], uiW[i], uiH[i], pcYuvPred);
          }
        }
      }
      else
      {
#endif
        if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
        {
          xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
        }
        else
        {
          xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
        }
#if QC_SUB_PU_TMVP
      }
#endif
    }
  }
  return;
}

#if QC_OBMC
/** Function for sub-block based Overlapped Block Motion Compensation (OBMC).
 *
 * This function can:
 * 1. Perform sub-block OBMC for a CU.
 * 2. Before motion estimation, subtract (scaled) predictors generated by applying neighboring motions to current CU/PU from the original signal of current CU/PU,
 *    to make the motion estimation biased to OBMC.
 */
Void TComPrediction::subBlockOBMC( TComDataCU*  pcCU, UInt uiAbsPartIdx, TComYuv *pcYuvPred, TComYuv *pcYuvTmpPred1, TComYuv *pcYuvTmpPred2, Bool bOBMC4ME )
{
  if( !pcCU->getSlice()->getSPS()->getOBMC() || !pcCU->getOBMCFlag( uiAbsPartIdx ) )
  {
    return;
  }
  
  PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
  UInt uiWidth           = pcCU->getWidth ( uiAbsPartIdx );
  UInt uiHeight          = pcCU->getHeight( uiAbsPartIdx );
  UInt uiMinCUW          = pcCU->getPic()->getMinCUWidth();
  UInt uiOBMCBlkSize     = pcCU->getSlice()->getSPS()->getOBMCBlkSize();
  UInt uiMaxWidthInBlock = pcCU->getPic()->getNumPartInWidth();

  UInt uiHeightInBlock   = uiHeight / uiMinCUW;
  UInt uiWidthInBlock    = uiWidth / uiMinCUW;
  UInt uiStep            = uiOBMCBlkSize / uiMinCUW;
  UInt uiMaxCUDepth      = pcCU->getSlice()->getSPS()->getMaxCUDepth();
  UInt uiDepth           = uiMaxCUDepth - pcCU->getDepth( uiAbsPartIdx );

  UInt uiSubPartIdx      = 0;
  UInt uiZeroIdx         = pcCU->getZorderIdxInCU();
  UInt uiAbsPartIdxLCURaster = g_auiZscanToRaster[uiAbsPartIdx + uiZeroIdx];
  Bool bOBMCSimp             = ( uiWidth == 8 && ePartSize != SIZE_2Nx2N );

  Int  i1stPUWidth  = -1, i1stPUHeight = -1;
  UInt uiPartAddr   = 0;
#if QC_SUB_PU_TMVP
  Bool bATMVP       = pcCU->getMergeType( uiAbsPartIdx ) == MGR_TYPE_SUBPU_TMVP;
  Bool bNormal2Nx2N = (ePartSize == SIZE_2Nx2N && !bATMVP);
  Bool bSubMotion   = ePartSize == SIZE_NxN   || (ePartSize == SIZE_2Nx2N && bATMVP);
#else
  Bool bNormal2Nx2N = ePartSize == SIZE_2Nx2N;
  Bool bSubMotion   = ePartSize == SIZE_NxN;
#endif
  Bool bVerticalPU  = ( ePartSize == SIZE_2NxN || ePartSize == SIZE_2NxnU || ePartSize == SIZE_2NxnD );
  Bool bHorizonalPU = ( ePartSize == SIZE_Nx2N || ePartSize == SIZE_nLx2N || ePartSize == SIZE_nRx2N );
  Bool bAtmvpPU = false, bNormalTwoPUs = false;
  Bool bTwoPUs  = ( bVerticalPU || bHorizonalPU );
  Int  iNeigPredDir = 0, iCurPredDir = 0;

  if( bTwoPUs )
  {
#if QC_SUB_PU_TMVP
    pcCU->getPartIndexAndSize( 1, uiPartAddr, i1stPUWidth, i1stPUHeight );
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP);
#endif
    pcCU->getPartIndexAndSize( 0, uiPartAddr, i1stPUWidth, i1stPUHeight );
#if QC_SUB_PU_TMVP
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP );
#endif
    i1stPUWidth  /= pcCU->getPic()->getMinCUWidth();
    i1stPUHeight /= pcCU->getPic()->getMinCUWidth();    

    bNormalTwoPUs = !bAtmvpPU;
  }

  Bool bCurrMotStored = false, bDiffMot[4]= { false, false, false, false };
  TComMvField cCurMvField[2], cNeigMvField[2];

  Int maxDir = bNormal2Nx2N ? 2 : 4;
  for( Int iSubX = 0; iSubX < uiWidthInBlock; iSubX += uiStep )
  {
    for( Int iSubY = 0; iSubY < uiHeightInBlock; iSubY += uiStep )
    {
      if( bNormal2Nx2N && iSubX && iSubY )
      {
        continue;
      }
      Bool bCURBoundary = ( iSubX == uiWidthInBlock  - 1 );
      Bool bCUBBoundary = ( iSubY == uiHeightInBlock - 1 );

      bCurrMotStored    = false;
      uiSubPartIdx      = g_auiRasterToZscan[uiAbsPartIdxLCURaster + iSubX + iSubY*uiMaxWidthInBlock] - uiZeroIdx;

      for( Int iDir = 0; iDir < maxDir; iDir++ ) //iDir: 0 - above, 1 - left, 2 - below, 3 - right
      {
        if( ( iDir == 3 && bCURBoundary ) || ( iDir == 2 && bCUBBoundary ) )
        {
          continue;
        }
 
        Bool bVerPUBound  = false;
        Bool bHorPUBound  = false;

        if( bNormal2Nx2N ) //skip unnecessary check for CU boundary
        {
          if( ( iDir == 1 && !iSubY && iSubX ) || ( iDir == 0 && !iSubX && iSubY ) )
          {
            continue;
          }
        }
        else
        {
          Bool bCheckNeig = bSubMotion || ( iSubX == 0 && iDir == 1 ) || ( iSubY == 0 && iDir == 0 ); //CU boundary or NxN or 2nx2n_ATMVP
          if( !bCheckNeig && bTwoPUs )
          {
#if QC_SUB_PU_TMVP
            if( bAtmvpPU )
            {
              //sub-PU boundary
              bCheckNeig |= ( pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP );
            }
#endif
            if( !bCheckNeig )
            {
              //PU boundary
              bVerPUBound = bVerticalPU  && ( ( iDir == 2 && iSubY == i1stPUHeight - 1 ) || ( iDir == 0 && iSubY == i1stPUHeight ) );
              bHorPUBound = bHorizonalPU && ( ( iDir == 3 && iSubX == i1stPUWidth  - 1 ) || ( iDir == 1 && iSubX == i1stPUWidth ) );

              bCheckNeig  |= ( bVerPUBound || bHorPUBound );
            }
          }
          if( !bCheckNeig )
          {
            continue;
          }
        }
        
        Bool bCurSubBkFetched  = bNormalTwoPUs && ( ( bVerPUBound && iSubX ) || ( bHorPUBound && iSubY ) );
#if QC_SUB_PU_TMVP
        Bool bSubBlockOBMCSimp = bOBMCSimp || ( pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP );
#else
        Bool bSubBlockOBMCSimp = bOBMCSimp;
#endif
        if( ( bCurSubBkFetched && bDiffMot[iDir] ) || pcCU->getNeigMotion( uiSubPartIdx, cNeigMvField, iNeigPredDir, iDir, cCurMvField, iCurPredDir, uiZeroIdx, bCurrMotStored ) )
        {
          Bool bFeAllSubBkIn1Line = false; //Fetch all sub-blocks in one row/column
          if( !bCurSubBkFetched )
          {
            //store temporary motion information
            pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiSubPartIdx, uiMaxCUDepth);
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cNeigMvField[0], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cNeigMvField[1], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->setInterDirSubParts( iNeigPredDir, uiSubPartIdx, 0, uiMaxCUDepth );
            if( bNormalTwoPUs )
            {
              bFeAllSubBkIn1Line = ( bHorPUBound || bVerPUBound );
              if( bFeAllSubBkIn1Line )
              {
                bDiffMot[iDir] = true;
              }
            }
          }
          UInt uiSubBlockWidth  = ( bFeAllSubBkIn1Line && bVerticalPU  ) ? uiWidth  : uiOBMCBlkSize;
          UInt uiSubBlockHeight = ( bFeAllSubBkIn1Line && !bVerticalPU ) ? uiHeight : uiOBMCBlkSize;
          //motion compensation and OBMC
          if( !bCurSubBkFetched )
          {
            xSubBlockMotionCompensation( pcCU, bFeAllSubBkIn1Line ? pcYuvTmpPred2 : pcYuvTmpPred1, uiSubPartIdx, uiSubBlockWidth, uiSubBlockHeight );
          }

          if( bOBMC4ME )
          {
            xSubtractOBMC( pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiOBMCBlkSize, uiOBMCBlkSize, iDir, bSubBlockOBMCSimp );
          }
          else
          {
            xSubblockOBMC( pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiOBMCBlkSize, uiOBMCBlkSize, iDir, bSubBlockOBMCSimp );
          }
          //recover motion information
          if( !bCurSubBkFetched )
          {
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cCurMvField[0], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cCurMvField[1], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->setInterDirSubParts( iCurPredDir, uiSubPartIdx, 0, uiMaxCUDepth );
            pcCU->setPartSizeSubParts( ePartSize, uiSubPartIdx,      uiMaxCUDepth );
          }
        }
      }
    }
  }
}

// Function for (weighted) averaging predictors of current block and predictors generated by applying neighboring motions to current block.
Void TComPrediction::xSubblockOBMC( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp )
{
  Int iDstStride  = pcYuvPredDst->getWidth();
  Int iDstCStride = pcYuvPredDst->getCStride();
  Int iSrcStride  = pcYuvPredSrc->getWidth();
  Int iSrcCStride = pcYuvPredSrc->getCStride();

  Pel *pDst   = pcYuvPredDst->getLumaAddr( uiAbsPartIdx );
  Pel *pSrc   = pcYuvPredSrc->getLumaAddr( uiAbsPartIdx );

  Pel *pDstCb = pcYuvPredDst->getCbAddr( uiAbsPartIdx ); 
  Pel *pDstCr = pcYuvPredDst->getCrAddr( uiAbsPartIdx );
  Pel *pSrcCb = pcYuvPredSrc->getCbAddr( uiAbsPartIdx ); 
  Pel *pSrcCr = pcYuvPredSrc->getCrAddr( uiAbsPartIdx );

  Int iDstPtrOffset = iDstStride, iScrPtrOffset = iSrcStride, iDstPtrOffsetC = iDstCStride, iScrPtrOffsetC = iSrcCStride, ioffsetDst = 1, ioffsetSrc = 1, ioffsetDstC = 1, ioffsetSrcC = 1;

  if( iDir ) //0: above; 1:left; 2: below; 3:right
  {
    if( iDir == 1 )  
    {
      //luma
      iDstPtrOffset = iScrPtrOffset = iDstPtrOffsetC = iScrPtrOffsetC = 1;
      ioffsetDst = iDstStride; ioffsetSrc = iSrcStride; ioffsetDstC = iDstCStride; ioffsetSrcC = iSrcCStride;
      iWidth = iHeight;
    }
    else if( iDir == 2 )
    {
      Int iHMinus1 = iHeight - 1;
      pDst += iHMinus1*iDstStride;
      pSrc += iHMinus1*iSrcStride;
      iDstPtrOffset = -iDstStride; iScrPtrOffset = -iSrcStride; iDstPtrOffsetC = -iDstCStride; iScrPtrOffsetC = -iSrcCStride; ioffsetDst = ioffsetSrc = ioffsetDstC = ioffsetSrcC = 1;
      //chroma
      Int iCHMinus1   = (iHeight >> 1) - 1;
      Int iDstCOffset = iCHMinus1 * iDstCStride, iSrcCOffset = iCHMinus1 * iSrcCStride;
      pDstCb  += iDstCOffset;
      pDstCr  += iDstCOffset;
      pSrcCb  += iSrcCOffset;
      pSrcCr  += iSrcCOffset;
    }
    else
    {
      Int iWMinus1 = iWidth - 1 , iCWMinus1 = (iWidth>>1) - 1;
      //luma
      pDst += iWMinus1;
      pSrc += iWMinus1;
      iDstPtrOffset = iScrPtrOffset = iDstPtrOffsetC = iScrPtrOffsetC = -1;
      ioffsetDst = iDstStride; ioffsetSrc = iSrcStride; ioffsetDstC = iDstCStride; ioffsetSrcC = iSrcCStride;

      //chroma
      pDstCb  += iCWMinus1;
      pDstCr  += iCWMinus1;
      pSrcCr  += iCWMinus1;
      pSrcCb  += iCWMinus1;
      iWidth  = iHeight;
    }
  }

  //luma
  Pel *pDst1 = pDst  + iDstPtrOffset;
  Pel *pDst2 = pDst1 + iDstPtrOffset;
  Pel *pDst3 = pDst2 + iDstPtrOffset;
  Pel *pSrc1 = pSrc  + iScrPtrOffset;
  Pel *pSrc2 = pSrc1 + iScrPtrOffset;
  Pel *pSrc3 = pSrc2 + iScrPtrOffset;

  for( Int i = 0; i < iWidth; i++ )
  {
    *pDst  = ( (*pDst) * 3  + (*pSrc)  + 2 ) >> 2;
    *pDst1 = ( (*pDst1) * 7 + (*pSrc1) + 4 ) >> 3;
    pDst += ioffsetDst; pDst1 += ioffsetDst; pSrc += ioffsetSrc; pSrc1 += ioffsetSrc;

    if( !bOBMCSimp )
    {
      *pDst2 = ( (*pDst2) * 15 + (*pSrc2) +  8 ) >> 4;
      *pDst3 = ( (*pDst3) * 31 + (*pSrc3) + 16 ) >> 5;
      pDst2 += ioffsetDst; pDst3 += ioffsetDst; pSrc2 += ioffsetSrc; pSrc3 += ioffsetSrc;
    }
  }

  //chroma
  iWidth >>= 1;
  Pel *pDstCb1 = pDstCb + iDstPtrOffsetC; 
  Pel *pDstCr1 = pDstCr + iDstPtrOffsetC;
  Pel *pSrcCb1 = pSrcCb + iScrPtrOffsetC; 
  Pel *pSrcCr1 = pSrcCr + iScrPtrOffsetC; 

  for( Int i = 0; i < iWidth; i++ )
  {
    *pDstCb = ( (*pDstCb)* 3 + (*pSrcCb) + 2 ) >> 2;
    *pDstCr = ( (*pDstCr)* 3 + (*pSrcCr) + 2 ) >> 2;
    pDstCb += ioffsetDstC; pDstCr += ioffsetDstC; pSrcCb += ioffsetSrcC; pSrcCr += ioffsetSrcC;

    if( !bOBMCSimp )
    {
      *pDstCb1 = ( (*pDstCb1)* 7 + (*pSrcCb1) + 4 ) >> 3;
      *pDstCr1 = ( (*pDstCr1)* 7 + (*pSrcCr1) + 4 ) >> 3;
      pDstCb1 += ioffsetDstC; pDstCr1 += ioffsetDstC; pSrcCb1 += ioffsetSrcC; pSrcCr1 += ioffsetSrcC;
    }
  }
}

// Function for subtracting (scaled) predictors generated by applying neighboring motions to current block from the original signal of current block.
Void TComPrediction::xSubtractOBMC( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp )
{
  Int iDstStride = pcYuvPredDst->getWidth();
  Int iSrcStride = pcYuvPredSrc->getWidth();
  Pel *pDst      = pcYuvPredDst->getLumaAddr( uiAbsPartIdx );
  Pel *pSrc      = pcYuvPredSrc->getLumaAddr( uiAbsPartIdx );

  if( iDir == 0 ) //above
  {
    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 2 ) >> 2;
    }

    pDst += iDstStride;
    pSrc += iSrcStride;

    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 4 ) >> 3;
    }

    if( !bOBMCSimp )
    {
      pDst += iDstStride;
      pSrc += iSrcStride;

      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pDst[i] + 8 ) >> 4;
      }

      pDst += iDstStride;
      pSrc += iSrcStride;

      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pSrc[i] + 16 ) >> 5;
      }
    }
  }

  if( iDir == 1 ) //left
  {
    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 2 ) >> 2;
    }

    pDst++;
    pSrc++;

    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 4 ) >> 3;
    }

    if( !bOBMCSimp )
    {
      pDst++;
      pSrc++;

      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 8 ) >> 4;
      }
      pDst++;
      pSrc++;

      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 16 ) >> 5;
      }
    }
  }

  if( iDir == 2 ) //below
  {
    pDst += ( iHeight - 4 )*iDstStride;
    pSrc += ( iHeight - 4 )*iSrcStride;
    if( !bOBMCSimp )
    {
      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pSrc[i] + 16 ) >> 5;
      }

      pDst += iDstStride;
      pSrc += iSrcStride;

      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pSrc[i] + 8 ) >> 4;
      }
    }
    else
    {
      pDst += iDstStride;
      pSrc += iSrcStride;
    }

    pDst += iDstStride;
    pSrc += iSrcStride;

    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 4 ) >> 3;
    }

    pDst += iDstStride;
    pSrc += iSrcStride;

    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 2 ) >> 2;
    }
  }

  if( iDir == 3 ) //right
  {
    pDst += iWidth - 4;
    pSrc += iWidth - 4;
    if( !bOBMCSimp )
    {
      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 16 ) >> 5;
      }

      pDst++;
      pSrc++;

      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 8 ) >> 4;
      }
    }
    else
    {
      pDst++;
      pSrc++;
    }

    pDst++;
    pSrc++;

    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 4 ) >> 3;
    }
    pDst++;
    pSrc++;

    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 2 ) >> 2;
    }
  }
}

Void TComPrediction::xSubBlockMotionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, Int uiPartAddr, Int iWidth, Int iHeight )
{
  if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
  {
    xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
  }
  else
  {
    xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
  }
}
#endif

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv*& rpcYuvPred, Bool bi )
{
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );
  pcCU->clipMv(cMv);
  xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi );
  xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi );
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv*& rpcYuvPred )
{
  TComYuv* pcMbYuv;
  Int      iRefIdx[2] = {-1, -1};

  for ( Int iRefList = 0; iRefList < 2; iRefList++ )
  {
    RefPicList eRefPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[iRefList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );

    if ( iRefIdx[iRefList] < 0 )
    {
      continue;
    }

    assert( iRefIdx[iRefList] < pcCU->getSlice()->getNumRefIdx(eRefPicList) );

    pcMbYuv = &m_acYuvPred[iRefList];
    if( pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 )
    {
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
    }
    else
    {
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) || 
           ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE ) )
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
      }
      else
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv );
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE  )
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[0], &m_acYuvPred[1], iRefIdx[0], iRefIdx[1], uiPartAddr, iWidth, iHeight, rpcYuvPred );
  }  
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, rpcYuvPred ); 
  }
  else
  {
    xWeightedAverage( &m_acYuvPred[0], &m_acYuvPred[1], iRefIdx[0], iRefIdx[1], uiPartAddr, iWidth, iHeight, rpcYuvPred );
  }
}

/**
 * \brief Generate motion-compensated luma block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */
Void TComPrediction::xPredInterLumaBlk( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi )
{
  Int refStride = refPic->getStride();  
  Int refOffset = ( mv->getHor() >> 2 ) + ( mv->getVer() >> 2 ) * refStride;
  Pel *ref      = refPic->getLumaAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  
  Int dstStride = dstPic->getStride();
  Pel *dst      = dstPic->getLumaAddr( partAddr );
  
  Int xFrac = mv->getHor() & 0x3;
  Int yFrac = mv->getVer() & 0x3;

  if ( yFrac == 0 )
  {
    m_if.filterHorLuma( ref, refStride, dst, dstStride, width, height, xFrac,       !bi );
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVerLuma( ref, refStride, dst, dstStride, width, height, yFrac, true, !bi );
  }
  else
  {
    Int tmpStride = m_filteredBlockTmp[0].getStride();
    Short *tmp    = m_filteredBlockTmp[0].getLumaAddr();

    Int filterSize = NTAPS_LUMA;
    Int halfFilterSize = ( filterSize >> 1 );

    m_if.filterHorLuma(ref - (halfFilterSize-1)*refStride, refStride, tmp, tmpStride, width, height+filterSize-1, xFrac, false     );
    m_if.filterVerLuma(tmp + (halfFilterSize-1)*tmpStride, tmpStride, dst, dstStride, width, height,              yFrac, false, !bi);    
  }
}

/**
 * \brief Generate motion-compensated chroma block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */
Void TComPrediction::xPredInterChromaBlk( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi )
{
  Int     refStride  = refPic->getCStride();
  Int     dstStride  = dstPic->getCStride();
  
  Int     refOffset  = (mv->getHor() >> 3) + (mv->getVer() >> 3) * refStride;
  
  Pel*    refCb     = refPic->getCbAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  Pel*    refCr     = refPic->getCrAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  
  Pel* dstCb = dstPic->getCbAddr( partAddr );
  Pel* dstCr = dstPic->getCrAddr( partAddr );
  
  Int     xFrac  = mv->getHor() & 0x7;
  Int     yFrac  = mv->getVer() & 0x7;
  UInt    cxWidth  = width  >> 1;
  UInt    cxHeight = height >> 1;
  
  Int     extStride = m_filteredBlockTmp[0].getStride();
  Short*  extY      = m_filteredBlockTmp[0].getLumaAddr();


  if ( yFrac == 0 )
  {
    m_if.filterHorChroma(refCb, refStride, dstCb,  dstStride, cxWidth, cxHeight, xFrac, !bi);    
    m_if.filterHorChroma(refCr, refStride, dstCr,  dstStride, cxWidth, cxHeight, xFrac, !bi);    
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVerChroma(refCb, refStride, dstCb, dstStride, cxWidth, cxHeight, yFrac, true, !bi);    
    m_if.filterVerChroma(refCr, refStride, dstCr, dstStride, cxWidth, cxHeight, yFrac, true, !bi);    
  }
  else
  {
    Int filterSize = NTAPS_CHROMA;
    Int halfFilterSize = (filterSize>>1);

    m_if.filterHorChroma(refCb - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth, cxHeight+filterSize-1, xFrac, false);
    m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCb, dstStride, cxWidth, cxHeight  , yFrac, false, !bi);
    
    m_if.filterHorChroma(refCr - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth, cxHeight+filterSize-1, xFrac, false);
    m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCr, dstStride, cxWidth, cxHeight  , yFrac, false, !bi);    
  }
}

Void TComPrediction::xWeightedAverage( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv*& rpcYuvDst )
{
  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    rpcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )
  {
    pcYuvSrc0->copyPartToPartYuv( rpcYuvDst, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )
  {
    pcYuvSrc1->copyPartToPartYuv( rpcYuvDst, uiPartIdx, iWidth, iHeight );
  }
}

// AMVP
Void TComPrediction::getMvPredAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
  if( pcAMVPInfo->iN <= 1 )
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[0];

    pcCU->setMVPIdxSubParts( 0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }

  assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
  rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
  return;
}

/** Function for deriving planar intra prediction.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 *
 * This function derives the prediction samples for planar mode (intra coding).
 */
Void TComPrediction::xPredIntraPlanar( Int* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height )
{
  assert(width == height);

  Int k, l, bottomLeft, topRight;
  Int horPred;
  Int leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  UInt blkSize = width;
  UInt offset2D = width;
  UInt shift1D = g_aucConvertToBit[ width ] + 2;
  UInt shift2D = shift1D + 1;

  // Get left and above reference column and row
  for(k=0;k<blkSize+1;k++)
  {
    topRow[k] = pSrc[k-srcStride];
    leftColumn[k] = pSrc[k*srcStride-1];
  }

  // Prepare intermediate variables used in interpolation
  bottomLeft = leftColumn[blkSize];
  topRight   = topRow[blkSize];
  for (k=0;k<blkSize;k++)
  {
    bottomRow[k]   = bottomLeft - topRow[k];
    rightColumn[k] = topRight   - leftColumn[k];
    topRow[k]      <<= shift1D;
    leftColumn[k]  <<= shift1D;
  }

  // Generate prediction signal
  for (k=0;k<blkSize;k++)
  {
    horPred = leftColumn[k] + offset2D;
    for (l=0;l<blkSize;l++)
    {
      horPred += rightColumn[k];
      topRow[l] += bottomRow[l];
      rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
    }
  }
}

/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void TComPrediction::xDCPredFiltering( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight )
{
  Pel* pDst = rpDst;
  Int x, y, iDstStride2, iSrcStride2;

  // boundary pixels processing
  pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);

  for ( x = 1; x < iWidth; x++ )
  {
    pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
  }

  for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
  {
    pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
  }

  return;
}


#if QC_LMCHROMA
/** Function for deriving chroma LM intra prediction.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param piSrc pointer to reconstructed chroma sample array
 * \param pPred pointer for the prediction sample array
 * \param uiPredStride the stride of the prediction sample array
 * \param uiCWidth the width of the chroma block
 * \param uiCHeight the height of the chroma block
 *
 * This function derives the prediction samples for chroma LM mode (chroma intra coding)
 */
Void TComPrediction::predLMIntraChroma( TComPattern* pcPattern, UInt uiChromaId, Pel* pPred, UInt uiPredStride, UInt uiCWidth, UInt uiCHeight )
{
  // LLS parameters estimation -->
  Int a, b, iShift;
  xGetLMParameters( pcPattern, uiCWidth, uiCHeight, 0, uiChromaId, a, b, iShift );

  // get prediction -->
  Int  iLumaStride = m_iLumaRecStride;
  Int  *pLuma = m_pLumaRecBuffer + iLumaStride + 1;

  for( Int i = 0; i < uiCHeight; i++ )
  {
    for( Int j = 0; j < uiCWidth; j++ )
    {
      pPred[j] = ClipC( ( ( a * pLuma[j] ) >> iShift ) + b );
    }

    pPred += uiPredStride;
    pLuma += iLumaStride;
  }
  // <-- end of get prediction
}

/** Function for deriving downsampled luma sample of current chroma block and its above, left causal pixel
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param uiCWidth the width of the chroma block
 * \param uiCHeight the height of the chroma block
 * \param bLeftPicBoundary indication of the chroma block located on the left picture boundary
 *
 * This function derives downsampled luma sample of current chroma block and its above, left causal pixel
 */
Void TComPrediction::getLumaRecPixels( TComPattern* pcPattern, UInt uiCWidth, UInt uiCHeight, Bool bLeftPicBoundary)
{
  Pel* pRecSrc0 = pcPattern->getROIY();
  Int* pDst0 = m_pLumaRecBuffer + m_iLumaRecStride + 1;

  Int iRecSrcStride = pcPattern->getPatternLStride();
  Int iRecSrcStride2 = iRecSrcStride << 1;
  Int iDstStride = m_iLumaRecStride;

  UInt uiWidth  = 2 * uiCWidth;
  UInt uiHeight = 2 * uiCHeight;  
  Int* ptrSrc = pcPattern->getAdiOrgBuf( uiWidth, uiHeight, m_piYuvExt );
  Int iSrcStride = ( max( uiWidth, uiHeight ) << 1 ) + 1;

  iSrcStride = ( max( uiWidth, uiHeight ) << 1 ) + 3;
  Int iSrcStride2 = iSrcStride << 1;
  Int* pDst = pDst0 - 1 - iDstStride;  
  Int* piSrc = ptrSrc;

  // top row downsampled from ADI buffer
  pDst++;     
  piSrc += 3;
  for (Int i = 0; i < uiCWidth; i++)
  {
    pDst[i] = ( ((piSrc[2*i]              * 2 ) + piSrc[2*i - 1]              + piSrc[2*i + 1]             )
      + ((piSrc[2*i + iSrcStride] * 2 ) + piSrc[2*i - 1 + iSrcStride] + piSrc[2*i + 1 + iSrcStride])
      + 4) >> 3;
  }

  // left column downsampled from ADI buffer
  pDst = pDst0 - 1; 
  piSrc = ptrSrc + iSrcStride2;
  for (Int j = 0; j < uiCHeight; j++)
  {
    pDst[0] = (  (piSrc[1]              *2 + piSrc[0]          + piSrc[2]             ) 
      + (piSrc[1 + iSrcStride] *2 + piSrc[iSrcStride] + piSrc[2 + iSrcStride])
      + 4) >> 3;
    piSrc += iSrcStride2; 
    pDst += iDstStride;    
  }

  // inner part from reconstructed picture buffer
  for( Int j = 0; j < uiCHeight; j++ )
  {
    for (Int i = 0; i < uiCWidth; i++)
    {
      if(i==0 && bLeftPicBoundary)
      {
        pDst0[i] = ( pRecSrc0[2*i] + pRecSrc0[2*i + iRecSrcStride] + 1) >> 1;
      }
      else
      {
        pDst0[i] = ( pRecSrc0[2*i]              * 2 + pRecSrc0[2*i + 1]                 + pRecSrc0[2*i -1 ] 
        + pRecSrc0[2*i + iRecSrcStride]* 2 + pRecSrc0[2*i + 1 + iRecSrcStride] + pRecSrc0[2*i -1 + iRecSrcStride]
        + 4) >> 3;
      }
    }
    pDst0 += iDstStride;
    pRecSrc0 += iRecSrcStride2;
  }
}

/** Function for deriving LM parameter for predciton of Cr from Cb.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param uiWidth the width of the chroma block
 * \param uiHeight the height of the chroma block
 * \param a the weight of the linear prediction model
 * \param b the offset of the linear prediction model
 * \param iShift the shifting bits of of the linear prediction model
 *
 * This function derives the parameters of linear prediction model
 */
Void TComPrediction::addCrossColorResi( TComPattern* pcPattern, Pel* piPred, UInt uiPredStride, UInt uiWidth, UInt uiHeight, Pel* piResi, UInt uiResiStride )
{
  Int a, b, iShift;

  xGetLMParameters( pcPattern, uiWidth, uiHeight, 1, 1, a, b, iShift );

  Int offset = 1 << (iShift - 1);

  if (a >= 0)
  {
    return;
  }

  Pel*  pPred   = piPred;
  Pel*  pResi   = piResi;

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      pPred[ uiX ] = ClipC(pPred[ uiX ] + (( pResi[ uiX ] * a + offset) >> iShift ) );
    }
    pPred += uiPredStride;
    pResi += uiResiStride;
  }
}

/** Function for deriving the positon of first non-zero binary bit of a value
 * \param x input value
 *
 * This function derives the positon of first non-zero binary bit of a value
 */
Int GetFloorLog2( UInt x )
{
  int bits = -1;
  while( x > 0 )
  {
    bits ++;
    x >>= 1;
  }
  return bits;
}

/** Function for deriving the parameters of linear prediction model.
 * \param x, y, xx, yy sum of reference samples of source component, target component, square of source component and multiplication of source component and target component
 * \param iCountShift, count of reference samples
 * \param iPredType indication of the cross-componennt preidciton type, 0: chroma from luma, 1: Cr from Cb
 * \param a the weight of the linear prediction model
 * \param b the offset of the linear prediction model
 * \param iShift the shifting bits of of the linear prediction model
 *
 * This function derives the parameters of linear prediction model
 */

Void TComPrediction::xCalcLMParameters( Int x, Int y, Int xx, Int xy, Int iCountShift, Int iPredType, Int &a, Int &b, Int &iShift )
{
  Int avgX =  x  >> iCountShift;
  Int avgY =  y  >> iCountShift;

  Int RErrX = x & ( ( 1 << iCountShift ) - 1 );
  Int RErrY =  y & ( ( 1 << iCountShift ) - 1 );

  Int iB = 7;
  iShift = 13 - iB;

  UInt uiInternalBitDepth = g_bitDepthC; // need consider different bit depth later ????????????

  if( iCountShift == 0 )
  {
    a = 0;
    b = 1 << (uiInternalBitDepth - 1);
    iShift = 0;
  }
  else
  {
    Int a1 = xy - ( avgX*avgY << iCountShift ) - avgX*RErrY - avgY*RErrX;
    Int a2 = xx - ( avgX*avgX << iCountShift ) - 2*avgX*RErrX ;

    if ( iPredType == 1) // Cr residual predicted from Cb residual, Cr from Cb
    {
      a1 += -1*( xx >> (CR_FROM_CB_REG_COST_SHIFT + 1 ));
      a2 += xx >> CR_FROM_CB_REG_COST_SHIFT;
    }

    const Int iShiftA1 = uiInternalBitDepth - 2;
    const Int iShiftA2 = 5;
    const Int iAccuracyShift = uiInternalBitDepth + 4;

    Int iScaleShiftA2 = 0;
    Int iScaleShiftA1 = 0;
    Int a1s = a1;
    Int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : GetFloorLog2( abs( a1 ) ) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : GetFloorLog2( abs( a2 ) ) - iShiftA2;

    if( iScaleShiftA1 < 0 )
    {
      iScaleShiftA1 = 0;
    }
    
    if( iScaleShiftA2 < 0 )
    {
      iScaleShiftA2 = 0;
    }

    Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if (a2s >= 32)
    {
      UInt a2t = m_uiaLMShift[ a2s - 32 ] ;
      a2t = ClipC( a2t );  //????????????
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }
    
    if( iScaleShiftA < 0 )
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3(-( 1 << (15-iB) ), ( 1 << (15-iB )) - 1, a);
    a = a << iB;
   
    Short n = 0;
    if (a != 0)
    {
      n = GetFloorLog2(abs( a ) + ( (a < 0 ? -1 : 1) - 1)/2 ) - 5;
    }
    
    iShift =(iShift+iB)-n;
    a = a>>n;

    b =  avgY - ( (  a * avgX ) >> iShift );
  }   

}
/** Function for deriving LM parameter for predciton of Cr from Cb.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param uiWidth the width of the chroma block
 * \param uiHeight the height of the chroma block
 * \param a the weight of the linear prediction model
 * \param b the offset of the linear prediction model
 * \param iShift the shifting bits of of the linear prediction model
 *
 * This function derives the parameters of linear prediction model
 */
Void TComPrediction::xGetLMParameters( TComPattern* pcPattern, UInt uiWidth, UInt uiHeight, Int iPredType, UInt uiChromaId, Int &a, Int &b, Int &iShift )
{
  Int *pSrcColor0, *pCurChroma0; 
  Int iSrcStride, iCurStride;
  UInt uiInternalBitDepth = g_bitDepthC; 

  if (iPredType == 0) //chroma from luma
  {
    iSrcStride = m_iLumaRecStride;
    pSrcColor0 = m_pLumaRecBuffer + iSrcStride + 1;

    pCurChroma0  = ( uiChromaId > 0 ? pcPattern->getAdiCrBuf( uiWidth, uiHeight, m_piYuvExt ) : pcPattern->getAdiCbBuf( uiWidth, uiHeight, m_piYuvExt ) );
    iCurStride   = 2 * uiWidth+ 1;
    pCurChroma0 += iCurStride + 1;
  }
  else
  {
    assert (uiChromaId == 1);

    pSrcColor0   = pcPattern->getAdiCbBuf( uiWidth, uiHeight, getPredicBuf() );
    pCurChroma0  = pcPattern->getAdiCrBuf( uiWidth, uiHeight, getPredicBuf() ); 

    iSrcStride = 2 * uiWidth+ 1;
    iCurStride = 2 * uiWidth+ 1;

    pSrcColor0  += iSrcStride + 1;
    pCurChroma0 += iCurStride + 1;
  }

  Int x = 0, y = 0, xx = 0, xy = 0;
  Int i, j;
  Int iCountShift = 0;

  Int *pSrc = pSrcColor0 - iSrcStride;
  Int *pCur = pCurChroma0 - iCurStride;

  for( j = 0; j < uiWidth; j++ )
  {
    x += pSrc[j];
    y += pCur[j];
    xx += pSrc[j] * pSrc[j];
    xy += pSrc[j] * pCur[j];
  }

  pSrc  = pSrcColor0 - 1;
  pCur = pCurChroma0 - 1;

  for( i = 0; i < uiHeight; i++ )
  {
    x += pSrc[0];
    y += pCur[0];
    xx += pSrc[0] * pSrc[0];
    xy += pSrc[0] * pCur[0];

    pSrc += iSrcStride;
    pCur += iCurStride;
  }

  iCountShift = g_aucConvertToBit[ uiWidth ] + 3;

  Int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if( iTempShift > 0 )
  {
    x  = ( x +  ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    y  = ( y +  ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xx = ( xx + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xy = ( xy + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    iCountShift -= iTempShift;
  }

  // LLS parameters estimation -->
  xCalcLMParameters( x, y, xx, xy, iCountShift, iPredType, a, b, iShift );
}

#endif


//! \}
