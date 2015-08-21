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

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"
#include "TComPic.h"
#include "TComTU.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar TComPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
    10, //4x4
    7, //8x8
    1, //16x16
    0, //32x32
    10, //64x64
#if COM16_C806_LARGE_CTU
    0, //128x128
    0, //256x256
#endif
  },
  { // Chroma
    10, //4xn
    7, //8xn
    1, //16xn
    0, //32xn
    10, //64xn
#if COM16_C806_LARGE_CTU
    0, //128x128
    0, //256x256
#endif
  }
};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()
: m_pLumaRecBuffer(0)
, m_iLumaRecStride(0)
{
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  m_puiW = NULL;
  m_puiH = NULL;
  m_puiSPAddr = NULL;
#endif
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<2; buf++)
    {
      m_piYuvExt[ch][buf] = NULL;
    }
  }
}

TComPrediction::~TComPrediction()
{
  destroy();
}

Void TComPrediction::destroy()
{
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  if( m_puiW != NULL )
  {
    delete [] m_puiW;
    m_puiW = NULL;
  }
  if( m_puiH != NULL )
  {
    delete [] m_puiH;
    m_puiH = NULL;
  }
  if( m_puiSPAddr != NULL )
  {
    delete [] m_puiSPAddr;
    m_puiSPAddr = NULL;
  }
#endif
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
    {
      delete [] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = NULL;
    }
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acYuvPred[i].destroy();
  }

  m_cYuvPredTemp.destroy();

  if( m_pLumaRecBuffer )
  {
    delete [] m_pLumaRecBuffer;
    m_pLumaRecBuffer = 0;
  }
  m_iLumaRecStride = 0;

  for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
  {
    for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
    {
      m_filteredBlock[i][j].destroy();
    }
    m_filteredBlockTmp[i].destroy();
  }
}

Void TComPrediction::initTempBuff(ChromaFormat chromaFormatIDC)
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != NULL && m_cYuvPredTemp.getChromaFormat()!=chromaFormatIDC)
  {
    destroy();
  }

  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == NULL ) // check if first is null (in which case, nothing initialised yet)
  {
    Int extWidth  = MAX_CU_SIZE + 16;
    Int extHeight = MAX_CU_SIZE + 1;

    for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
    {
      m_filteredBlockTmp[i].create(extWidth, extHeight + 7, chromaFormatIDC);
      for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight, chromaFormatIDC);
      }
    }

    m_iYuvExtSize = (MAX_CU_SIZE*2+1) * (MAX_CU_SIZE*2+1);
    for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
    {
      for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[ m_iYuvExtSize ];
      }
    }

    // new structure
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acYuvPred[i] .create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
    }

    m_cYuvPredTemp.create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
  }


  if (m_iLumaRecStride != (MAX_CU_SIZE>>1) + 1)
  {
    m_iLumaRecStride =  (MAX_CU_SIZE>>1) + 1;
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Pel[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  m_puiW = new UInt[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH];
  m_puiH = new UInt[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH];
  m_puiSPAddr = new UInt[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH];
#endif

#if VCEG_AZ06_IC
  m_uiaICShift[0] = 0;
  for( Int i = 1; i < 64; i++ )
  {
    m_uiaICShift[i] = ( (1 << 15) + i/2 ) / i;
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel TComPrediction::predIntraGetPredValDC( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight)
{
  assert(iWidth > 0 && iHeight > 0);
  Int iInd, iSum = 0;
  Pel pDcVal;

  for (iInd = 0;iInd < iWidth;iInd++)
  {
    iSum += pSrc[iInd-iSrcStride];
  }
  for (iInd = 0;iInd < iHeight;iInd++)
  {
    iSum += pSrc[iInd*iSrcStride-1];
  }

  pDcVal = (iSum + iWidth) / (iWidth + iHeight);

  return pDcVal;
}

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param bitDepth           bit depth
 * \param pSrc               pointer to reconstructed sample array
 * \param srcStride          the stride of the reconstructed sample array
 * \param pTrueDst           reference to pointer for the prediction sample array
 * \param dstStrideTrue      the stride of the prediction sample array
 * \param uiWidth            the width of the block
 * \param uiHeight           the height of the block
 * \param channelType        type of pel array (luma/chroma)
 * \param format             chroma format
 * \param dirMode            the intra prediction mode index
 * \param blkAboveAvailable  boolean indication if the block above is available
 * \param blkLeftAvailable   boolean indication if the block to the left is available
 * \param bEnableEdgeFilters indication whether to enable edge filters
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
//NOTE: Bit-Limit - 25-bit source
Void TComPrediction::xPredIntraAng(       Int bitDepth,
                                    const Pel* pSrc,     Int srcStride,
                                          Pel* pTrueDst, Int dstStrideTrue,
                                          UInt uiWidth, UInt uiHeight, ChannelType channelType,
                                          UInt dirMode, const Bool bEnableEdgeFilters
                                  )
{
  Int width=Int(uiWidth);
  Int height=Int(uiHeight);

  // Map the mode index to main prediction direction and angle
  assert( dirMode != PLANAR_IDX ); //no planar
  const Bool modeDC        = dirMode==DC_IDX;

  // Do the DC prediction
  if (modeDC)
  {
    const Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height);

    for (Int y=height;y>0;y--, pTrueDst+=dstStrideTrue)
    {
      for (Int x=0; x<width;) // width is always a multiple of 4.
      {
        pTrueDst[x++] = dcval;
      }
    }
  }
  else // Do angular predictions
  {
    const Bool       bIsModeVer         = (dirMode >= 18);
    const Int        intraPredAngleMode = (bIsModeVer) ? (Int)dirMode - VER_IDX :  -((Int)dirMode - HOR_IDX);
    const Int        absAngMode         = abs(intraPredAngleMode);
    const Int        signAng            = intraPredAngleMode < 0 ? -1 : 1;
    const Bool       edgeFilter         = bEnableEdgeFilters && isLuma(channelType) && (width <= MAXIMUM_INTRA_FILTERED_WIDTH) && (height <= MAXIMUM_INTRA_FILTERED_HEIGHT);

    // Set bitshifts and scale the angle parameter to block size
    static const Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
    static const Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
    Int invAngle                    = invAngTable[absAngMode];
    Int absAng                      = angTable[absAngMode];
    Int intraPredAngle              = signAng * absAng;

    Pel* refMain;
    Pel* refSide;

    Pel  refAbove[2*MAX_CU_SIZE+1];
    Pel  refLeft[2*MAX_CU_SIZE+1];

    // Initialize the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      const Int refMainOffsetPreScale = (bIsModeVer ? height : width ) - 1;
      const Int refMainOffset         = height - 1;
      for (Int x=0;x<width+1;x++)
      {
        refAbove[x+refMainOffset] = pSrc[x-srcStride-1];
      }
      for (Int y=0;y<height+1;y++)
      {
        refLeft[y+refMainOffset] = pSrc[(y-1)*srcStride-1];
      }
      refMain = (bIsModeVer ? refAbove : refLeft)  + refMainOffset;
      refSide = (bIsModeVer ? refLeft  : refAbove) + refMainOffset;

      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)
      for (Int k=-1; k>(refMainOffsetPreScale+1)*intraPredAngle>>5; k--)
      {
        invAngleSum += invAngle;
        refMain[k] = refSide[invAngleSum>>8];
      }
    }
    else
    {
      for (Int x=0;x<2*width+1;x++)
      {
        refAbove[x] = pSrc[x-srcStride-1];
      }
      for (Int y=0;y<2*height+1;y++)
      {
        refLeft[y] = pSrc[(y-1)*srcStride-1];
      }
      refMain = bIsModeVer ? refAbove : refLeft ;
      refSide = bIsModeVer ? refLeft  : refAbove;
    }

    // swap width/height if we are doing a horizontal mode:
    Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
    const Int dstStride = bIsModeVer ? dstStrideTrue : MAX_CU_SIZE;
    Pel *pDst = bIsModeVer ? pTrueDst : tempArray;
    if (!bIsModeVer)
    {
      std::swap(width, height);
    }

    if (intraPredAngle == 0)  // pure vertical or pure horizontal
    {
      for (Int y=0;y<height;y++)
      {
        for (Int x=0;x<width;x++)
        {
          pDst[y*dstStride+x] = refMain[x+1];
        }
      }

      if (edgeFilter)
      {
        for (Int y=0;y<height;y++)
        {
          pDst[y*dstStride] = Clip3 (0, ((1 << bitDepth) - 1), pDst[y*dstStride] + (( refSide[y+1] - refSide[0] ) >> 1) );
        }
      }
    }
    else
    {
      Pel *pDsty=pDst;

      for (Int y=0, deltaPos=intraPredAngle; y<height; y++, deltaPos+=intraPredAngle, pDsty+=dstStride)
      {
        const Int deltaInt   = deltaPos >> 5;
        const Int deltaFract = deltaPos & (32 - 1);

        if (deltaFract)
        {
          // Do linear filtering
          const Pel *pRM=refMain+deltaInt+1;
          Int lastRefMainPel=*pRM++;
          for (Int x=0;x<width;pRM++,x++)
          {
            Int thisRefMainPel=*pRM;
            pDsty[x+0] = (Pel) ( ((32-deltaFract)*lastRefMainPel + deltaFract*thisRefMainPel +16) >> 5 );
            lastRefMainPel=thisRefMainPel;
          }
        }
        else
        {
          // Just copy the integer samples
          for (Int x=0;x<width; x++)
          {
            pDsty[x] = refMain[x+deltaInt+1];
          }
        }
      }
    }

    // Flip the block if this is the horizontal mode
    if (!bIsModeVer)
    {
      for (Int y=0; y<height; y++)
      {
        for (Int x=0; x<width; x++)
        {
          pTrueDst[x*dstStrideTrue] = pDst[x];
        }
        pTrueDst++;
        pDst+=dstStride;
      }
    }
  }
}

Void TComPrediction::predIntraAng( const ComponentID compID, UInt uiDirMode, Pel* piOrg /* Will be null for decoding */, UInt uiOrgStride, Pel* piPred, UInt uiStride, TComTU &rTu, const Bool bUseFilteredPredSamples, const Bool bUseLosslessDPCM )
{
  const ChannelType    channelType = toChannelType(compID);
  const TComRectangle &rect        = rTu.getRect(isLuma(compID) ? COMPONENT_Y : COMPONENT_Cb);
  const Int            iWidth      = rect.width;
  const Int            iHeight     = rect.height;

  assert( g_aucConvertToBit[ iWidth ] >= 0 ); //   4x  4
#if COM16_C806_LARGE_CTU
  assert( g_aucConvertToBit[ iWidth ] <= MAX_CU_DEPTH - 2 ); 
#else
  assert( g_aucConvertToBit[ iWidth ] <= 5 ); // 128x128
#endif
  //assert( iWidth == iHeight  );

        Pel *pDst = piPred;

  // get starting pixel in block
  const Int sw = (2 * iWidth + 1);

  if ( bUseLosslessDPCM )
  {
    const Pel *ptrSrc = getPredictorPtr( compID, false );
    // Sample Adaptive intra-Prediction (SAP)
    if (uiDirMode==HOR_IDX)
    {
      // left column filled with reference samples
      // remaining columns filled with piOrg data (if available).
      for(Int y=0; y<iHeight; y++)
      {
        piPred[y*uiStride+0] = ptrSrc[(y+1)*sw];
      }
      if (piOrg!=0)
      {
        piPred+=1; // miss off first column
        for(Int y=0; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, (iWidth-1)*sizeof(Pel));
        }
      }
    }
    else // VER_IDX
    {
      // top row filled with reference samples
      // remaining rows filled with piOrd data (if available)
      for(Int x=0; x<iWidth; x++)
      {
        piPred[x] = ptrSrc[x+1];
      }
      if (piOrg!=0)
      {
        piPred+=uiStride; // miss off the first row
        for(Int y=1; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, iWidth*sizeof(Pel));
        }
      }
    }
  }
  else
  {
    const Pel *ptrSrc = getPredictorPtr( compID, bUseFilteredPredSamples );

    if ( uiDirMode == PLANAR_IDX )
    {
      xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
    }
    else
    {
      // Create the prediction
            TComDataCU *const pcCU              = rTu.getCU();
      const UInt              uiAbsPartIdx      = rTu.GetAbsPartIdxTU();
      const Bool              enableEdgeFilters = !(pcCU->isRDPCMEnabled(uiAbsPartIdx) && pcCU->getCUTransquantBypass(uiAbsPartIdx));
#if O0043_BEST_EFFORT_DECODING
      const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getStreamBitDepth(channelType);
#else
      const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getBitDepth(channelType);
#endif
      xPredIntraAng( channelsBitDepthForPrediction, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType, uiDirMode, enableEdgeFilters );

      if( uiDirMode == DC_IDX )
      {
        xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType );
      }
    }
  }

}

/** Check for identical motion in both motion vector direction of a bi-directional predicted CU
  * \returns true, if motion vectors and reference pictures match
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
      if( pcCU->getSlice()->getPPS()->getUseWP()
#if VCEG_AZ06_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() 
#if VCEG_AZ06_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
      if ( pcCU->getMergeType(uiPartAddr))  
      {
        Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;

        pcCU->getSPPara(iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);

        xGetSubPUAddrAndMerge(pcCU, uiPartAddr, iSPWidth, iSPHeight, iNumSPInOneLine, iNumSP, m_puiW, m_puiH, m_puiSPAddr);

        //MC
        for (Int i = 0; i < iNumSP; i++)
        {
          if (m_puiW[i]==0 || m_puiH[i]==0)
          {
            continue;
          }

          if(xCheckIdenticalMotion( pcCU, m_puiSPAddr[i] ))
          {
            xPredInterUni (pcCU, m_puiSPAddr[i], m_puiW[i], m_puiH[i], REF_PIC_LIST_0, pcYuvPred );
          }
          else
          {
            xPredInterBi  (pcCU, m_puiSPAddr[i], m_puiW[i], m_puiH[i], pcYuvPred);
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
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
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
      if( pcCU->getSlice()->getPPS()->getUseWP()
#if VCEG_AZ06_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() 
#if VCEG_AZ06_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
      if (pcCU->getMergeType(uiPartAddr))  
      {
        Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;

        pcCU->getSPPara(iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);

        xGetSubPUAddrAndMerge(pcCU, uiPartAddr, iSPWidth, iSPHeight, iNumSPInOneLine, iNumSP, m_puiW, m_puiH, m_puiSPAddr);
      
        for (Int i = 0; i < iNumSP; i++)
        {
          if (m_puiW[i]==0 || m_puiH[i]==0)
          {
            continue;
          }
          if( xCheckIdenticalMotion( pcCU, m_puiSPAddr[i] ))
          {
            xPredInterUni (pcCU, m_puiSPAddr[i], m_puiW[i], m_puiH[i], REF_PIC_LIST_0, pcYuvPred );
          }
          else
          {
            xPredInterBi  (pcCU, m_puiSPAddr[i], m_puiW[i], m_puiH[i], pcYuvPred);
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
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
      }
#endif
    }
  }
  return;
}

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred, Bool bi )
{
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );
  pcCU->clipMv(cMv);

  for (UInt comp=COMPONENT_Y; comp<pcYuvPred->getNumberValidComponents(); comp++)
  {
    const ComponentID compID=ComponentID(comp);
#if VCEG_AZ06_IC
    Bool bICFlag = pcCU->getICFlag( uiPartAddr ) ;

    xPredInterBlk  ( compID, pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)), bICFlag );
#else
    xPredInterBlk  (compID,  pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)) );
#endif
  }
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvPred )
{
  TComYuv* pcMbYuv;
  Int      iRefIdx[NUM_REF_PIC_LIST_01] = {-1, -1};

  for ( UInt refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[refList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );

    if ( iRefIdx[refList] < 0 )
    {
      continue;
    }

    assert( iRefIdx[refList] < pcCU->getSlice()->getNumRefIdx(eRefPicList) );

    pcMbYuv = &m_acYuvPred[refList];
    if( pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 )
    {
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
    }
    else
    {
#if VCEG_AZ06_IC
      if ( ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) || 
        ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE ) ) 
        && !pcCU->getICFlag( uiPartAddr ) )
#else
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) ||
           ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE ) )
#endif
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
      }
      else
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv );
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE  
#if VCEG_AZ06_IC
    && !pcCU->getICFlag( uiPartAddr )
#endif
    )
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred );
  }
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE
#if VCEG_AZ06_IC
    && !pcCU->getICFlag( uiPartAddr )
#endif
    )
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[REF_PIC_LIST_0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
  }
  else
  {
    xWeightedAverage( &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred, pcCU->getSlice()->getSPS()->getBitDepths() );
  }
}

/**
 * \brief Generate motion-compensated block
 *
 * \param compID     Colour component ID
 * \param cu         Pointer to current CU
 * \param refPic     Pointer to reference picture
 * \param partAddr   Address of block within CU
 * \param mv         Motion vector
 * \param width      Width of block
 * \param height     Height of block
 * \param dstPic     Pointer to destination picture
 * \param bi         Flag indicating whether bipred is used
 * \param  bitDepth  Bit depth
 */


Void TComPrediction::xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth
#if VCEG_AZ06_IC
  , Bool bICFlag
#endif
  )
{
  Int     refStride  = refPic->getStride(compID);
  Int     dstStride  = dstPic->getStride(compID);
  Int shiftHor=(2+refPic->getComponentScaleX(compID));
  Int shiftVer=(2+refPic->getComponentScaleY(compID));

  Int     refOffset  = (mv->getHor() >> shiftHor) + (mv->getVer() >> shiftVer) * refStride;

  Pel*    ref     = refPic->getAddr(compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr ) + refOffset;

  Pel*    dst = dstPic->getAddr( compID, partAddr );

  Int     xFrac  = mv->getHor() & ((1<<shiftHor)-1);
  Int     yFrac  = mv->getVer() & ((1<<shiftVer)-1);
  UInt    cxWidth  = width  >> refPic->getComponentScaleX(compID);
  UInt    cxHeight = height >> refPic->getComponentScaleY(compID);

  const ChromaFormat chFmt = cu->getPic()->getChromaFormat();

  if ( yFrac == 0 )
  {
    m_if.filterHor(compID, ref, refStride, dst,  dstStride, cxWidth, cxHeight, xFrac, 
#if VCEG_AZ06_IC
      !bi || bICFlag,
#else
      !bi,
#endif
      chFmt, bitDepth);
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVer(compID, ref, refStride, dst, dstStride, cxWidth, cxHeight, yFrac, true, 
#if VCEG_AZ06_IC
      !bi || bICFlag,
#else
      !bi,
#endif
      chFmt, bitDepth);
  }
  else
  {
    Int   tmpStride = m_filteredBlockTmp[0].getStride(compID);
    Pel*  tmp       = m_filteredBlockTmp[0].getAddr(compID);

    const Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

    m_if.filterHor(compID, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, cxWidth, cxHeight+vFilterSize-1, xFrac, false,      chFmt, bitDepth);
    m_if.filterVer(compID, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dst, dstStride, cxWidth, cxHeight,               yFrac, false, 
#if VCEG_AZ06_IC
      !bi || bICFlag,
#else
      !bi,
#endif
      chFmt, bitDepth);
  }

#if VCEG_AZ06_IC
  if( bICFlag )
  {
    Int a, b, i, j;
    const Int iShift = m_ICConstShift;
    xGetLLSICPrediction( cu, mv, refPic, a, b, compID, bitDepth );
    
    dst = dstPic->getAddr( compID, partAddr );

    for ( i = 0; i < cxHeight; i++ )
    {
      for ( j = 0; j < cxWidth; j++ )
      {
        dst[j] = Clip3( 0, ( 1 << bitDepth ) - 1, ( ( a*dst[j] ) >> iShift ) + b );
      }
      dst += dstStride;
    }

    if(bi)
    {
      Pel *dst2      = dstPic->getAddr( compID, partAddr );
      Int shift = IF_INTERNAL_PREC - bitDepth;
      for (i = 0; i < cxHeight; i++)
      {
        for (j = 0; j < cxWidth; j++)
        {
          Short val = dst2[j] << shift;
          dst2[j] = val - (Short)IF_INTERNAL_OFFS;
        }
        dst2 += dstStride;
      }
    }
  }
#endif
}

#if VCEG_AZ06_IC
/** Function for deriving the position of first non-zero binary bit of a value
 * \param x input value
 *
 * This function derives the position of first non-zero binary bit of a value
 */
Int GetMSB( UInt x )
{
  Int iMSB = 0, bits = ( sizeof( Int ) << 3 ), y = 1;

  while( x > 1 )
  {
    bits >>= 1;
    y = x >> bits;

    if( y )
    {
      x = y;
      iMSB += bits;
    }
  }

  iMSB+=y;

  return iMSB;
}

/** Function for deriving LM illumination compensation.
 */
Void TComPrediction::xGetLLSICPrediction( TComDataCU* pcCU, TComMv *pMv, TComPicYuv *pRefPic, Int &a, Int &b, const ComponentID eComp, Int nBitDepth )
{
  TComPicYuv *pRecPic = pcCU->getPic()->getPicYuvRec();
  Pel *pRec = NULL, *pRef = NULL;
  UInt uiWidth, uiTmpPartIdx;
  Int iRecStride = pRecPic->getStride( eComp );
  Int iRefStride = pRefPic->getStride( eComp );
  Int iRefOffset, iRecOffset, iHor, iVer;
  Int shiftHor=(2+pRefPic->getComponentScaleX(eComp));
  Int shiftVer=(2+pRefPic->getComponentScaleY(eComp));

  iHor = ( pMv->getHor() + (1<<(shiftHor-1)) ) >> shiftHor;
  iVer = ( pMv->getVer() + (1<<(shiftVer-1)) ) >> shiftVer;
  uiWidth  = ( eComp == COMPONENT_Y ) ? pcCU->getWidth( 0 )  : ( pcCU->getWidth( 0 )  >> 1 );
  Int j, iCountShift = 0;

  // LLS parameters estimation -->

  Int x = 0, y = 0, xx = 0, xy = 0;
  Int precShift = std::max( 0, ( nBitDepth - 12 ) );
  Int iTmpRec, iTmpRef;
  Int iRefStep, iRecStep;
  UInt uiStep = 2;//uiWidth > 8 ? 2 : 1;
  TComDataCU* pNeigCu = NULL;
  TComMv cMv;
  Int iMaxNumMinus1 = 30 - 2*min( nBitDepth, 12 ) - 1;
  while( uiWidth/uiStep > ( 1 << iMaxNumMinus1 ) ) //make sure log2(2*uiWidth/uiStep) + 2*min(g_bitDepthY, 12) <= 30
  {
    uiStep <<= 1;
  }

  for( Int iDir = 0; iDir < 2; iDir++ ) //iDir: 0 - above, 1 - left
  {
    if( !iDir )
    {
      pNeigCu = pcCU->getPUAbove( uiTmpPartIdx, pcCU->getZorderIdxInCtu() );
    }
    else
    {
      pNeigCu =  pcCU->getPULeft( uiTmpPartIdx, pcCU->getZorderIdxInCtu() );
    }

    if( pNeigCu == NULL )
    {
      continue;
    }

    cMv.setHor( iHor << shiftHor ); cMv.setVer( iVer << shiftVer );
    pNeigCu->clipMv( cMv );

    if( iDir )
    {
      iRefOffset = ( cMv.getHor() >> shiftHor ) + ( cMv.getVer() >> shiftVer ) * iRefStride - 1;
      iRecOffset = -1;
      iRefStep   = iRefStride*uiStep;
      iRecStep   = iRecStride*uiStep;
    }
    else
    {
      iRefOffset = ( cMv.getHor() >> shiftHor ) + ( cMv.getVer() >> shiftVer ) * iRefStride - iRefStride;
      iRecOffset = -iRecStride;
      iRefStep   = uiStep;
      iRecStep   = uiStep;
    }

    pRef = pRefPic->getAddr( eComp, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() ) + iRefOffset;
    pRec = pRecPic->getAddr( eComp, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() ) + iRecOffset;

    for( j = 0; j < uiWidth; j+=uiStep )
    {
      iTmpRef = pRef[0] >> precShift;
      iTmpRec = pRec[0] >> precShift;

      x  += iTmpRef;
      y  += iTmpRec;
      xx += iTmpRef*iTmpRef;
      xy += iTmpRef*iTmpRec;

      pRef += iRefStep;
      pRec += iRecStep;
    }

    iCountShift += ( iCountShift ? 1 : g_aucConvertToBit[ uiWidth/uiStep ] + 2 );
  }

  if( iCountShift == 0 )
  {
    a = ( 1 << m_ICConstShift );
    b = 0;
    return;
  }

  xy += xx >> m_ICRegCostShift;
  xx += xx >> m_ICRegCostShift;

  Int  iCropShift = max( 0, ( nBitDepth - precShift + iCountShift ) - 15 );
  Int  x1 = x, y1 = y;

  x  >>= iCropShift;
  y  >>= iCropShift;
  xy >>= ( iCropShift << 1 );
  xx >>= ( iCropShift << 1 );

  Int a1 = ( xy << iCountShift ) - ( y * x );
  Int a2 = ( xx << iCountShift ) - ( x * x );

  x = x1 << precShift;
  y = y1 << precShift;

  const Int iShift = m_ICConstShift;
  const Int iShiftA2 = 6;
  const Int iAccuracyShift = 15;
  Int iScaleShiftA2 = 0;
  Int iScaleShiftA1 = 0;
  Int a1s = a1;
  Int a2s = a2;

  iScaleShiftA2 = GetMSB( abs( a2 ) ) - iShiftA2;
  iScaleShiftA1 = iScaleShiftA2 - m_ICShiftDiff;

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

  a2s = Clip3( 0 , 63, a2s );
  Int64 aI64 = ( (Int64) a1s * m_uiaICShift[ a2s ] ) >> iScaleShiftA;
  a = (Int) aI64;
  a = Clip3( 0, 1 << ( iShift + 2 ), a );
  b = (  y - ( ( a * x ) >> iShift ) + ( 1 << ( iCountShift - 1 ) ) ) >> iCountShift;
  Int iOffset = 1 << ( nBitDepth - 1 );
  b = Clip3( -iOffset, iOffset - 1, b );
}
#endif

Void TComPrediction::xWeightedAverage( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv* pcYuvDst, const BitDepths &clipBitDepths )
{
  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    pcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight, clipBitDepths );
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )
  {
    pcYuvSrc0->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )
  {
    pcYuvSrc1->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
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
 * \param pSrc        pointer to reconstructed sample array
 * \param srcStride   the stride of the reconstructed sample array
 * \param rpDst       reference to pointer for the prediction sample array
 * \param dstStride   the stride of the prediction sample array
 * \param width       the width of the block
 * \param height      the height of the block
 * \param channelType type of pel array (luma, chroma)
 * \param format      chroma format
 *
 * This function derives the prediction samples for planar mode (intra coding).
 */
//NOTE: Bit-Limit - 24-bit source
Void TComPrediction::xPredIntraPlanar( const Pel* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height )
{
  assert(width <= height);

  Int leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  UInt shift1Dhor = g_aucConvertToBit[ width ] + 2;
  UInt shift1Dver = g_aucConvertToBit[ height ] + 2;

  // Get left and above reference column and row
  for(Int k=0;k<width+1;k++)
  {
    topRow[k] = pSrc[k-srcStride];
  }

  for (Int k=0; k < height+1; k++)
  {
    leftColumn[k] = pSrc[k*srcStride-1];
  }

  // Prepare intermediate variables used in interpolation
  Int bottomLeft = leftColumn[height];
  Int topRight   = topRow[width];

  for(Int k=0;k<width;k++)
  {
    bottomRow[k]  = bottomLeft - topRow[k];
    topRow[k]     <<= shift1Dver;
  }

  for(Int k=0;k<height;k++)
  {
    rightColumn[k]  = topRight - leftColumn[k];
    leftColumn[k]   <<= shift1Dhor;
  }

  const UInt topRowShift = 0;

  // Generate prediction signal
  for (Int y=0;y<height;y++)
  {
    Int horPred = leftColumn[y] + width;
    for (Int x=0;x<width;x++)
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      Int vertPred = ((topRow[x] + topRowShift)>>topRowShift);
      rpDst[y*dstStride+x] = ( horPred + vertPred ) >> (shift1Dhor+1);
    }
  }
}

/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param pDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 * \param channelType type of pel array (luma, chroma)
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void TComPrediction::xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel* pDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType )
{
  Int x, y, iDstStride2, iSrcStride2;

  if (isLuma(channelType) && (iWidth <= MAXIMUM_INTRA_FILTERED_WIDTH) && (iHeight <= MAXIMUM_INTRA_FILTERED_HEIGHT))
  {
    //top-left
    pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);

    //top row (vertical filter)
    for ( x = 1; x < iWidth; x++ )
    {
      pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
    }

    //left column (horizontal filter)
    for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
    {
      pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
    }
  }

  return;
}

/* Static member function */
Bool TComPrediction::UseDPCMForFirstPassIntraEstimation(TComTU &rTu, const UInt uiDirMode)
{
  return (rTu.getCU()->isRDPCMEnabled(rTu.GetAbsPartIdxTU()) ) &&
          rTu.getCU()->getCUTransquantBypass(rTu.GetAbsPartIdxTU()) &&
          (uiDirMode==HOR_IDX || uiDirMode==VER_IDX);
}

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
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

#if COM16_C806_OBMC
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
  UInt uiChromaOBMCWidth = 0, uiChromaOBMCHeight = 0;
  UInt uiMaxWidthInBlock = pcCU->getPic()->getNumPartInCtuWidth();

  UInt uiHeightInBlock   = uiHeight / uiMinCUW;
  UInt uiWidthInBlock    = uiWidth / uiMinCUW;
  UInt uiStep            = uiOBMCBlkSize / uiMinCUW;
  UInt uiMaxCUDepth      = pcCU->getSlice()->getSPS()->getMaxTotalCUDepth();
  UInt uiDepth           = uiMaxCUDepth - pcCU->getDepth( uiAbsPartIdx );

  UInt uiSubPartIdx      = 0;
  UInt uiZeroIdx         = pcCU->getZorderIdxInCtu();
  UInt uiAbsPartIdxLCURaster = g_auiZscanToRaster[uiAbsPartIdx + uiZeroIdx];
  Bool bOBMCSimp             = ( uiWidth == 8 && ePartSize != SIZE_2Nx2N );

  Int  i1stPUWidth  = -1, i1stPUHeight = -1;
  UInt uiPartAddr   = 0;
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  Bool bATMVP       = (pcCU->getMergeType( uiAbsPartIdx ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiAbsPartIdx ) == MGR_TYPE_SUBPU_TMVP_EXT);
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

  switch( pcCU->getSlice()->getSPS()->getChromaFormatIdc() )
  {
  case CHROMA_400:
    uiChromaOBMCWidth = uiChromaOBMCHeight = 0;                              break;
  case CHROMA_420:
    uiChromaOBMCWidth = uiChromaOBMCHeight = uiOBMCBlkSize/2;                break;
  case CHROMA_422:
    uiChromaOBMCWidth = uiOBMCBlkSize/2; uiChromaOBMCHeight = uiOBMCBlkSize; break;
  case CHROMA_444:
    uiChromaOBMCWidth = uiChromaOBMCHeight = uiOBMCBlkSize;                  break;
  default:
    break;
  }

  if( bTwoPUs )
  {
    pcCU->getPartIndexAndSize( 1, uiPartAddr, i1stPUWidth, i1stPUHeight );
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP_EXT);
#endif
    pcCU->getPartIndexAndSize( 0, uiPartAddr, i1stPUWidth, i1stPUHeight );
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP_EXT);
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
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
            if( bAtmvpPU )
            {
              //sub-PU boundary
              bCheckNeig |= (pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP_EXT);
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

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
        Bool bSubBlockOBMCSimp = (bOBMCSimp || (( pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP_EXT) && ( 1 << pcCU->getSlice()->getSPS()->getSubPUTLog2Size() ) == 4 ));
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
            xSubblockOBMC( COMPONENT_Y, pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiOBMCBlkSize, uiOBMCBlkSize, iDir, bSubBlockOBMCSimp );
            xSubblockOBMC( COMPONENT_Cb, pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiChromaOBMCWidth, uiChromaOBMCHeight, iDir, bSubBlockOBMCSimp );
            xSubblockOBMC( COMPONENT_Cr, pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiChromaOBMCWidth, uiChromaOBMCHeight, iDir, bSubBlockOBMCSimp );
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
Void TComPrediction::xSubblockOBMC( const ComponentID eComp, TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp )
{

  if( iWidth == 0 || iHeight == 0 )
  {
    return;
  }

  Int iDstStride  = pcYuvPredDst->getStride( eComp );
  Int iSrcStride  = pcYuvPredSrc->getStride( eComp );

  Pel *pDst   = pcYuvPredDst->getAddr( eComp, uiAbsPartIdx );
  Pel *pSrc   = pcYuvPredSrc->getAddr( eComp, uiAbsPartIdx );

  Int iDstPtrOffset = iDstStride, iScrPtrOffset = iSrcStride, ioffsetDst = 1, ioffsetSrc = 1;

  if( iDir ) //0: above; 1:left; 2: below; 3:right
  {
    if( iDir == 1 )  
    {
      iDstPtrOffset = iScrPtrOffset = 1;
      ioffsetDst = iDstStride; ioffsetSrc = iSrcStride;
      iWidth = iHeight;
    }
    else if( iDir == 2 )
    {
      Int iHMinus1 = iHeight - 1;
      pDst += iHMinus1*iDstStride;
      pSrc += iHMinus1*iSrcStride;
      iDstPtrOffset = -iDstStride; iScrPtrOffset = -iSrcStride; ioffsetDst = ioffsetSrc = 1;
    }
    else
    {
      Int iWMinus1 = iWidth - 1;
      pDst += iWMinus1;
      pSrc += iWMinus1;
      iDstPtrOffset = iScrPtrOffset = -1;
      ioffsetDst = iDstStride; ioffsetSrc = iSrcStride;
      iWidth  = iHeight;
    }
  }

  Pel *pDst1 = pDst  + iDstPtrOffset;
  Pel *pDst2 = pDst1 + iDstPtrOffset;
  Pel *pDst3 = pDst2 + iDstPtrOffset;
  Pel *pSrc1 = pSrc  + iScrPtrOffset;
  Pel *pSrc2 = pSrc1 + iScrPtrOffset;
  Pel *pSrc3 = pSrc2 + iScrPtrOffset;

  for( Int i = 0; i < iWidth; i++ )
  {
    *pDst  = ( (*pDst) * 3  + (*pSrc)  + 2 ) >> 2;
    if( !bOBMCSimp || eComp == COMPONENT_Y )
    {
      *pDst1 = ( (*pDst1) * 7 + (*pSrc1) + 4 ) >> 3;
    }
    pDst += ioffsetDst; pDst1 += ioffsetDst; pSrc += ioffsetSrc; pSrc1 += ioffsetSrc;

    if( !bOBMCSimp && ( eComp == COMPONENT_Y ) )
    {
      *pDst2 = ( (*pDst2) * 15 + (*pSrc2) +  8 ) >> 4;
      *pDst3 = ( (*pDst3) * 31 + (*pSrc3) + 16 ) >> 5;
      pDst2 += ioffsetDst; pDst3 += ioffsetDst; pSrc2 += ioffsetSrc; pSrc3 += ioffsetSrc;
    }
  }
}

// Function for subtracting (scaled) predictors generated by applying neighboring motions to current block from the original signal of current block.
Void TComPrediction::xSubtractOBMC( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp )
{
  Int iDstStride = pcYuvPredDst->getWidth( COMPONENT_Y );
  Int iSrcStride = pcYuvPredSrc->getWidth( COMPONENT_Y );
  Pel *pDst      = pcYuvPredDst->getAddr( COMPONENT_Y, uiAbsPartIdx );
  Pel *pSrc      = pcYuvPredSrc->getAddr( COMPONENT_Y, uiAbsPartIdx );

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

//! \}
