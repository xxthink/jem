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

/** \file     TDecCu.cpp
\brief    CU decoder class
*/

#include "TDecCu.h"

//! \ingroup TLibDecoder
//! \{
// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TDecCu::TDecCu()
{
#if !QT_BT_STRUCTURE
  m_ppcYuvResi = NULL;
  m_ppcYuvReco = NULL;
  m_ppcCU      = NULL;
#endif
}

TDecCu::~TDecCu()
{
}

Void TDecCu::init( TDecEntropy* pcEntropyDecoder, TComTrQuant* pcTrQuant, TComPrediction* pcPrediction)
{
  m_pcEntropyDecoder  = pcEntropyDecoder;
  m_pcTrQuant         = pcTrQuant;
  m_pcPrediction      = pcPrediction;
}

/**
\param    uiMaxDepth    total number of allowable depth
\param    uiMaxWidth    largest CU width
\param    uiMaxHeight   largest CU height
*/
Void TDecCu::create( UInt uiMaxDepth, UInt uiMaxWidth, UInt uiMaxHeight )
{
  m_uiMaxDepth = uiMaxDepth+1;

#if !QT_BT_STRUCTURE
  m_ppcYuvResi = new TComYuv*[m_uiMaxDepth-1];
  m_ppcYuvReco = new TComYuv*[m_uiMaxDepth-1];
  m_ppcCU      = new TComDataCU*[m_uiMaxDepth-1];
#endif

  UInt uiNumPartitions;
  for ( UInt ui = 0; ui < m_uiMaxDepth-1; ui++ )
  {
    uiNumPartitions = 1<<( ( m_uiMaxDepth - ui - 1 )<<1 );
#if !QT_BT_STRUCTURE
    UInt uiWidth  = uiMaxWidth  >> ui;
    UInt uiHeight = uiMaxHeight >> ui;

    m_ppcYuvResi[ui] = new TComYuv;    m_ppcYuvResi[ui]->create( uiWidth, uiHeight );
    m_ppcYuvReco[ui] = new TComYuv;    m_ppcYuvReco[ui]->create( uiWidth, uiHeight );
    m_ppcCU     [ui] = new TComDataCU; m_ppcCU     [ui]->create( uiNumPartitions, uiWidth, uiHeight, true, uiMaxWidth >> (m_uiMaxDepth - 1) );
#endif
  }

#if QT_BT_STRUCTURE
  UInt uiNumWidthIdx = g_aucConvertToBit[uiMaxWidth] + 1;
  UInt uiNumHeightIdx = g_aucConvertToBit[uiMaxHeight] + 1;

  m_pppcCUPU      = new TComDataCU**[uiNumWidthIdx];

  m_pppcYuvResiPU = new TComYuv**[uiNumWidthIdx];
  m_pppcYuvRecoPU = new TComYuv**[uiNumWidthIdx];

  for (UInt wIdx=0; wIdx<uiNumWidthIdx; wIdx++)
  {
    m_pppcCUPU[wIdx] = new TComDataCU* [uiNumHeightIdx];
    m_pppcYuvResiPU[wIdx] = new TComYuv*[uiNumHeightIdx];
    m_pppcYuvRecoPU[wIdx] = new TComYuv*[uiNumHeightIdx];
    for (UInt hIdx=0; hIdx<uiNumHeightIdx; hIdx++)
    {
      uiNumPartitions = 1<<(wIdx+hIdx);
      UInt uiWidth = 1<<(wIdx+MIN_CU_LOG2);
      UInt uiHeight = 1<<(hIdx+MIN_CU_LOG2);

      m_pppcCUPU[wIdx][hIdx]      = new TComDataCU; m_pppcCUPU[wIdx][hIdx]->create( 1<<( ( m_uiMaxDepth - 1 )<<1 ), uiMaxWidth, uiMaxHeight, true, uiMaxWidth >> (m_uiMaxDepth - 1), uiWidth, uiHeight );
      m_pppcYuvResiPU[wIdx][hIdx] = new TComYuv;    m_pppcYuvResiPU[wIdx][hIdx]->create( uiWidth, uiHeight );
      m_pppcYuvRecoPU[wIdx][hIdx] = new TComYuv;    m_pppcYuvRecoPU[wIdx][hIdx]->create( uiWidth, uiHeight );

    }
  }
#endif

  m_bDecodeDQP = false;

  // initialize partition order.
  UInt* piTmp = &g_auiZscanToRaster[0];
  initZscanToRaster(m_uiMaxDepth, 1, 0, piTmp);
  initRasterToZscan( uiMaxWidth, uiMaxHeight, m_uiMaxDepth );

  // initialize conversion matrix from partition index to pel
  initRasterToPelXY( uiMaxWidth, uiMaxHeight, m_uiMaxDepth );
}

Void TDecCu::destroy()
{
  for ( UInt ui = 0; ui < m_uiMaxDepth-1; ui++ )
  {
#if !QT_BT_STRUCTURE
    m_ppcYuvResi[ui]->destroy(); delete m_ppcYuvResi[ui]; m_ppcYuvResi[ui] = NULL;
    m_ppcYuvReco[ui]->destroy(); delete m_ppcYuvReco[ui]; m_ppcYuvReco[ui] = NULL;
    m_ppcCU     [ui]->destroy(); delete m_ppcCU     [ui]; m_ppcCU     [ui] = NULL;
#endif
  }

#if !QT_BT_STRUCTURE
  delete [] m_ppcYuvResi; m_ppcYuvResi = NULL;
  delete [] m_ppcYuvReco; m_ppcYuvReco = NULL;
  delete [] m_ppcCU     ; m_ppcCU      = NULL;
#endif

#if QT_BT_STRUCTURE
  UInt uiMaxWidth = g_uiMaxCUWidth;
  UInt uiMaxHeight = g_uiMaxCUHeight;
  UInt uiNumWidthIdx = g_aucConvertToBit[uiMaxWidth] + 1;
  UInt uiNumHeightIdx = g_aucConvertToBit[uiMaxHeight] + 1;

  for (UInt wIdx=0; wIdx<uiNumWidthIdx; wIdx++)
  {
    for (UInt hIdx=0; hIdx<uiNumHeightIdx; hIdx++)
    {
      m_pppcCUPU[wIdx][hIdx]->destroy(); delete m_pppcCUPU[wIdx][hIdx] ; m_pppcCUPU[wIdx][hIdx] = NULL;
      m_pppcYuvResiPU[wIdx][hIdx]->destroy(); delete m_pppcYuvResiPU[wIdx][hIdx]; m_pppcYuvResiPU[wIdx][hIdx] = NULL;
      m_pppcYuvRecoPU[wIdx][hIdx]->destroy(); delete m_pppcYuvRecoPU[wIdx][hIdx]; m_pppcYuvRecoPU[wIdx][hIdx] = NULL;
    }
    delete m_pppcCUPU[wIdx]; m_pppcCUPU[wIdx] = NULL;
    delete m_pppcYuvResiPU[wIdx]; m_pppcYuvResiPU[wIdx] = NULL;
    delete m_pppcYuvRecoPU[wIdx]; m_pppcYuvRecoPU[wIdx] = NULL;
  }
  delete m_pppcCUPU; m_pppcCUPU = NULL;
  delete m_pppcYuvResiPU; m_pppcYuvResiPU = NULL;
  delete m_pppcYuvRecoPU; m_pppcYuvRecoPU = NULL;
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param    pcCU        pointer of CU data
\param    ruiIsLast   last data?
*/
Void TDecCu::decodeCU( TComDataCU* pcCU, UInt& ruiIsLast )
{
  if ( pcCU->getSlice()->getPPS()->getUseDQP() )
  {
    setdQPFlag(true);
  }
#if QT_BT_STRUCTURE
  pcCU->getPic()->setCodedBlkInCTU(false, 0, 0, MAX_NUM_SPU_W, MAX_NUM_SPU_W);
  pcCU->getPic()->setCodedAreaInCTU(0);

  // start from the top level CU
  UInt uiLastIdx=0;
  UInt uiLastDepth=0;
  pcCU->setTextType(TEXT_LUMA);
  xDecodeCU( pcCU, 0, 0, g_uiMaxCUWidth, g_uiMaxCUHeight, uiLastIdx, uiLastDepth);
  UInt uiDummy = 0;

  if (pcCU->getSlice()->isIntra())
  {
    pcCU->getPic()->setCodedBlkInCTU(false, 0, 0, MAX_NUM_SPU_W, MAX_NUM_SPU_W);
    pcCU->getPic()->setCodedAreaInCTU(0);
    pcCU->setTextType(TEXT_CHROMA);
    xDecodeCU( pcCU, 0, 0, g_uiMaxCUWidth, g_uiMaxCUHeight, uiDummy, uiDummy); 
    pcCU->setTextType(TEXT_LUMA);
  }
  xFinishDecodeCU( pcCU, uiLastIdx, uiLastDepth, ruiIsLast ); 
#else
  xDecodeCU( pcCU, 0, 0, ruiIsLast);
#endif
}

/** \param    pcCU        pointer of CU data
*/
Void TDecCu::decompressCU( TComDataCU* pcCU )
{
#if QT_BT_STRUCTURE
  pcCU->setTextType(TEXT_LUMA);
  pcCU->getPic()->setCodedBlkInCTU(false, 0, 0, MAX_NUM_SPU_W, MAX_NUM_SPU_W);
  pcCU->getPic()->setCodedAreaInCTU(0);
  xDecompressCU( pcCU, 0,  0, g_uiMaxCUWidth, g_uiMaxCUHeight ); 
  if (pcCU->getSlice()->isIntra())
  {
    pcCU->setTextType(TEXT_CHROMA);
    pcCU->getPic()->setCodedBlkInCTU(false, 0, 0, MAX_NUM_SPU_W, MAX_NUM_SPU_W);
    pcCU->getPic()->setCodedAreaInCTU(0);
    xDecompressCU( pcCU, 0,  0, g_uiMaxCUWidth, g_uiMaxCUHeight ); 
  }
#else
  xDecompressCU( pcCU, 0,  0 );
#endif
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**decode end-of-slice flag
* \param pcCU
* \param uiAbsPartIdx 
* \param uiDepth 
* \returns Bool
*/
Bool TDecCu::xDecodeSliceEnd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth)
{
  UInt uiIsLast;
  TComPic* pcPic = pcCU->getPic();
  TComSlice * pcSlice = pcPic->getSlice(pcPic->getCurrSliceIdx());
  UInt uiCurNumParts    = pcPic->getNumPartInCU() >> (uiDepth<<1);
  UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
  UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
  UInt uiGranularityWidth = g_uiMaxCUWidth;
  UInt uiPosX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiPosY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

  if(((uiPosX+pcCU->getWidth(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosX+pcCU->getWidth(uiAbsPartIdx)==uiWidth))
    &&((uiPosY+pcCU->getHeight(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosY+pcCU->getHeight(uiAbsPartIdx)==uiHeight)))
  {
    m_pcEntropyDecoder->decodeTerminatingBit( uiIsLast );
  }
  else
  {
    uiIsLast=0;
  }

  if(uiIsLast) 
  {
    if(pcSlice->isNextSliceSegment()&&!pcSlice->isNextSlice()) 
    {
      pcSlice->setSliceSegmentCurEndCUAddr(pcCU->getSCUAddr()+uiAbsPartIdx+uiCurNumParts);
    }
    else 
    {
      pcSlice->setSliceCurEndCUAddr(pcCU->getSCUAddr()+uiAbsPartIdx+uiCurNumParts);
      pcSlice->setSliceSegmentCurEndCUAddr(pcCU->getSCUAddr()+uiAbsPartIdx+uiCurNumParts);
    }
  }

  return uiIsLast>0;
}

/** decode CU block recursively
* \param pcCU
* \param uiAbsPartIdx 
* \param uiDepth 
* \returns Void
*/

#if QT_BT_STRUCTURE
Void TDecCu::xDecodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight, UInt& ruiLastIdx, UInt& ruiLastDepth, UInt uiSplitConstrain )
#else
Void TDecCu::xDecodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt& ruiIsLast)
#endif
{
  TComPic* pcPic = pcCU->getPic();
  UInt uiCurNumParts    = pcPic->getNumPartInCU() >> (uiDepth<<1);
  UInt uiQNumParts      = uiCurNumParts>>2;

  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;

  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  Bool bStartInCU = pcCU->getSCUAddr()+uiAbsPartIdx+uiCurNumParts>pcSlice->getSliceSegmentCurStartCUAddr()&&pcCU->getSCUAddr()+uiAbsPartIdx<pcSlice->getSliceSegmentCurStartCUAddr();

#if BT_RMV_REDUNDANT
  pcCU->setSplitConstrain( uiSplitConstrain );
  Bool bQTreeValid = false;
#endif

#if QT_BT_STRUCTURE
  if ((g_uiMaxCUWidth>>uiDepth)==uiWidth && (g_uiMaxCUHeight>>uiDepth)==uiHeight)
  {
    assert(uiWidth==uiHeight);
#endif
    if((!bStartInCU) && ( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
    {
      m_pcEntropyDecoder->decodeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
    }
    else
    {
      bBoundary = true;
    }

#if BT_RMV_REDUNDANT
    bQTreeValid = true;

    UInt uiMinQTSize = pcCU->getSlice()->isIntra() ? (pcCU->getTextType()==TEXT_LUMA?MIN_QT_SIZE:MIN_QT_SIZE_C): pcCU->getSlice()->getMinQTSize();

    if ((g_uiMaxCUWidth>>uiDepth) <= uiMinQTSize )  
    {
      bQTreeValid = false;
    }
#endif

#if QT_BT_STRUCTURE
    if( ( uiDepth < pcCU->getDepth( uiAbsPartIdx )  ) || bBoundary )
#else
    if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth ) ) || bBoundary )
#endif
    {
      UInt uiIdx = uiAbsPartIdx;
      if( (g_uiMaxCUWidth>>uiDepth) == pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
      {
        setdQPFlag(true);
        pcCU->setQPSubParts( pcCU->getRefQP(uiAbsPartIdx), uiAbsPartIdx, uiDepth ); // set QP to default QP
      }

      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
      {
        uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
        uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];

        Bool bSubInSlice = pcCU->getSCUAddr()+uiIdx+uiQNumParts>pcSlice->getSliceSegmentCurStartCUAddr();
        if ( bSubInSlice )
        {
#if QT_BT_STRUCTURE
          if (( uiLPelX < pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
          {
            xDecodeCU( pcCU, uiIdx, uiDepth+1, uiWidth>>1, uiHeight>>1, ruiLastIdx, ruiLastDepth );
          }
#else
          if ( !ruiIsLast && ( uiLPelX < pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
          {
            xDecodeCU( pcCU, uiIdx, uiDepth+1, ruiIsLast );
          }
#endif
          else
          {
            pcCU->setOutsideCUPart( uiIdx, uiDepth+1 );
#if QT_BT_STRUCTURE
            pcCU->getPic()->addCodedAreaInCTU(uiWidth*uiHeight>>2);
#endif
          }
        }

        uiIdx += uiQNumParts;
      }
      if( (g_uiMaxCUWidth>>uiDepth) == pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
      {
        if ( getdQPFlag() )
        {
          UInt uiQPSrcPartIdx;
          if ( pcPic->getCU( pcCU->getAddr() )->getSliceSegmentStartCU(uiAbsPartIdx) != pcSlice->getSliceSegmentCurStartCUAddr() )
          {
            uiQPSrcPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU();
          }
          else
          {
            uiQPSrcPartIdx = uiAbsPartIdx;
          }
          pcCU->setQPSubParts( pcCU->getRefQP( uiQPSrcPartIdx ), uiAbsPartIdx, uiDepth ); // set QP to default QP
        }
      }
      return;
    }

#if QT_BT_STRUCTURE
    ruiLastIdx = uiAbsPartIdx;
    ruiLastDepth = uiDepth;
#endif
    if( (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
    {
      setdQPFlag(true);
      pcCU->setQPSubParts( pcCU->getRefQP(uiAbsPartIdx), uiAbsPartIdx, uiDepth ); // set QP to default QP
    }

    if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyDecoder->decodeCUTransquantBypassFlag( pcCU, uiAbsPartIdx, uiDepth );
    }

#if QT_BT_STRUCTURE
  }

#if BT_RMV_REDUNDANT  
  Bool bBTHorRmvEnable = false;
  Bool bBTVerRmvEnable = false;
  if (pcCU->getSlice()->getSliceType() != I_SLICE)
  {
    bBTHorRmvEnable = true;
    bBTVerRmvEnable = bQTreeValid;
  }
#endif

  UInt uiMaxBTD = pcCU->getSlice()->isIntra() ? (pcCU->getTextType()==TEXT_LUMA?MAX_BT_DEPTH:MAX_BT_DEPTH_C): MAX_BT_DEPTH_INTER;
  UInt uiMaxBTSize = pcCU->getTextType()==TEXT_LUMA ? pcCU->getSlice()->getMaxBTSize(): MAX_BT_SIZE_C;
  UInt uiMinBTSize = pcCU->getSlice()->isIntra() ? (pcCU->getTextType()==TEXT_LUMA?MIN_BT_SIZE:MIN_BT_SIZE_C): MIN_BT_SIZE_INTER;
  UInt uiBTDepth = pcCU->getBTDepth(uiAbsPartIdx, uiWidth, uiHeight);

  if ( (uiHeight>=2*uiMinBTSize || uiWidth>=2*uiMinBTSize) 
    && uiWidth<=uiMaxBTSize && uiHeight<=uiMaxBTSize && uiBTDepth<uiMaxBTD) 
  {
    m_pcEntropyDecoder->decodeBTSplitMode(pcCU, uiAbsPartIdx, uiWidth, uiHeight);

#if BT_RMV_REDUNDANT
    uiSplitConstrain = 0;
#endif
    if (pcCU->getBTSplitModeForBTDepth(uiAbsPartIdx, uiBTDepth)==1)
    {
      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 2; uiPartUnitIdx++ )
      {
        if (uiPartUnitIdx==1)
        {
          uiAbsPartIdx = g_auiRasterToZscan[g_auiZscanToRaster[uiAbsPartIdx] 
          + (uiHeight>>1)/pcCU->getPic()->getMinCUHeight()*pcCU->getPic()->getNumPartInWidth()];
        }
#if BT_RMV_REDUNDANT
        xDecodeCU( pcCU, uiAbsPartIdx, uiDepth, uiWidth, uiHeight>>1, ruiLastIdx, ruiLastDepth, uiSplitConstrain );
        if (pcCU->getBTSplitModeForBTDepth(uiAbsPartIdx, uiBTDepth+1) == 2 && bBTHorRmvEnable && uiPartUnitIdx==0)
        {
          uiSplitConstrain = 2;
        }
#else
        xDecodeCU( pcCU, uiAbsPartIdx, uiDepth, uiWidth, uiHeight>>1, ruiLastIdx, ruiLastDepth );
#endif
      }
      return;
    }
    else if (pcCU->getBTSplitModeForBTDepth(uiAbsPartIdx, uiBTDepth)==2)
    {
      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 2; uiPartUnitIdx++ )
      {
        if (uiPartUnitIdx==1)
        {
          uiAbsPartIdx = g_auiRasterToZscan[g_auiZscanToRaster[uiAbsPartIdx] 
          + (uiWidth>>1)/pcCU->getPic()->getMinCUWidth()];
        }
#if BT_RMV_REDUNDANT
        xDecodeCU( pcCU, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight, ruiLastIdx, ruiLastDepth, uiSplitConstrain );
        if (pcCU->getBTSplitModeForBTDepth(uiAbsPartIdx, uiBTDepth+1) == 1 && bBTVerRmvEnable && uiPartUnitIdx==0)
        {
          uiSplitConstrain = 1;
        }
#else
        xDecodeCU( pcCU, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight, ruiLastIdx, ruiLastDepth );
#endif
      }
      return;
    }  
  }

  pcCU->setSizeSubParts( uiWidth, uiHeight, uiAbsPartIdx, uiDepth );

  UInt uiBlkX = g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ] >> MIN_CU_LOG2;
  UInt uiBlkY = g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ] >> MIN_CU_LOG2;
  pcCU->getPic()->setCodedBlkInCTU(true, uiBlkX, uiBlkY, uiWidth>> MIN_CU_LOG2, uiHeight>> MIN_CU_LOG2);

  pcCU->getPic()->addCodedAreaInCTU(uiWidth*uiHeight);

  UInt uiWidthIdx = g_aucConvertToBit[uiWidth];
  UInt uiHeightIdx = g_aucConvertToBit[uiHeight];
#endif

#if QT_BT_STRUCTURE
  if (pcCU->getTextType()==TEXT_LUMA)
  {
#endif
    // decode CU mode and the partition size
    if( !pcCU->getSlice()->isIntra())
    {
      m_pcEntropyDecoder->decodeSkipFlag( pcCU, uiAbsPartIdx, uiDepth );
    }

    if( pcCU->isSkipped(uiAbsPartIdx) )
    {
#if QT_BT_STRUCTURE
      m_pppcCUPU[uiWidthIdx][uiHeightIdx]->copyInterPredInfoFrom( pcCU, uiAbsPartIdx, REF_PIC_LIST_0 );
      m_pppcCUPU[uiWidthIdx][uiHeightIdx]->copyInterPredInfoFrom( pcCU, uiAbsPartIdx, REF_PIC_LIST_1 );
#else
      m_ppcCU[uiDepth]->copyInterPredInfoFrom( pcCU, uiAbsPartIdx, REF_PIC_LIST_0 );
      m_ppcCU[uiDepth]->copyInterPredInfoFrom( pcCU, uiAbsPartIdx, REF_PIC_LIST_1 );
#endif
      TComMvField cMvFieldNeighbours[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
      UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
      Int numValidMergeCand = 0;
#if QT_BT_STRUCTURE
      for( UInt ui = 0; ui < m_pppcCUPU[uiWidthIdx][uiHeightIdx]->getSlice()->getMaxNumMergeCand(); ++ui )
#else
      for( UInt ui = 0; ui < m_ppcCU[uiDepth]->getSlice()->getMaxNumMergeCand(); ++ui )
#endif
      {
        uhInterDirNeighbours[ui] = 0;
      }
      m_pcEntropyDecoder->decodeMergeIndex( pcCU, 0, uiAbsPartIdx, uiDepth );
      UInt uiMergeIndex = pcCU->getMergeIndex(uiAbsPartIdx);
#if QT_BT_STRUCTURE
      m_pppcCUPU[uiWidthIdx][uiHeightIdx]->getInterMergeCandidates( 0, 0, cMvFieldNeighbours, uhInterDirNeighbours, 
        numValidMergeCand,        
        uiMergeIndex );
      pcCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeIndex], uiAbsPartIdx);
#else
      m_ppcCU[uiDepth]->getInterMergeCandidates( 0, 0, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand, uiMergeIndex );
      pcCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeIndex], uiAbsPartIdx, 0, uiDepth );
#endif

      TComMv cTmpMv( 0, 0 );
      for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
      {        
        if ( pcCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
        {
#if QT_BT_STRUCTURE
          pcCU->setMVPIdxSubParts( 0, RefPicList( uiRefListIdx ), uiAbsPartIdx);
          pcCU->setMVPNumSubParts( 0, RefPicList( uiRefListIdx ), uiAbsPartIdx);
#else
          pcCU->setMVPIdxSubParts( 0, RefPicList( uiRefListIdx ), uiAbsPartIdx, 0, uiDepth);
          pcCU->setMVPNumSubParts( 0, RefPicList( uiRefListIdx ), uiAbsPartIdx, 0, uiDepth);
#endif
          pcCU->getCUMvField( RefPicList( uiRefListIdx ) )->setAllMvd( cTmpMv, SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
          pcCU->getCUMvField( RefPicList( uiRefListIdx ) )->setAllMvField( cMvFieldNeighbours[ 2*uiMergeIndex + uiRefListIdx ], SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
        }
      }
#if QT_BT_STRUCTURE
      if(  pcCU->getSlice()->getPPS()->getUseDQP())
      {
        pcCU->setQPSubParts( getdQPFlag()?pcCU->getRefQP(uiAbsPartIdx):pcCU->getCodedQP(), uiAbsPartIdx, uiDepth ); // set QP
      }
#else
      xFinishDecodeCU( pcCU, uiAbsPartIdx, uiDepth, ruiIsLast );
#endif
      return;
    }

    m_pcEntropyDecoder->decodePredMode( pcCU, uiAbsPartIdx, uiDepth );
#if !QT_BT_STRUCTURE
    m_pcEntropyDecoder->decodePartSize( pcCU, uiAbsPartIdx, uiDepth );
#else
  }
  else
  {
    pcCU->setPredModeSubParts( MODE_INTRA, uiAbsPartIdx );
  }
#endif

#if QT_BT_STRUCTURE
  if (pcCU->isIntra( uiAbsPartIdx ))
#else
  if (pcCU->isIntra( uiAbsPartIdx ) && pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )
#endif
  {
    m_pcEntropyDecoder->decodeIPCMInfo( pcCU, uiAbsPartIdx, uiDepth );

    if(pcCU->getIPCMFlag(uiAbsPartIdx))
    {
#if QT_BT_STRUCTURE
      if(  pcCU->getSlice()->getPPS()->getUseDQP())
      {
        pcCU->setQPSubParts( getdQPFlag()?pcCU->getRefQP(uiAbsPartIdx):pcCU->getCodedQP(), uiAbsPartIdx, uiDepth ); // set QP
      }
#else
      xFinishDecodeCU( pcCU, uiAbsPartIdx, uiDepth, ruiIsLast );
#endif
      return;
    }
  }

#if !QT_BT_STRUCTURE
  UInt uiCurrWidth      = pcCU->getWidth ( uiAbsPartIdx );
  UInt uiCurrHeight     = pcCU->getHeight( uiAbsPartIdx );
#endif

  // prediction mode ( Intra : direction mode, Inter : Mv, reference idx )
#if QT_BT_STRUCTURE
  m_pcEntropyDecoder->decodePredInfo( pcCU, uiAbsPartIdx, uiDepth, m_pppcCUPU[uiWidthIdx][uiHeightIdx]);
#else
  m_pcEntropyDecoder->decodePredInfo( pcCU, uiAbsPartIdx, uiDepth, m_ppcCU[uiDepth]);
#endif

  // Coefficient decoding
  Bool bCodeDQP = getdQPFlag();
#if QT_BT_STRUCTURE
  m_pcEntropyDecoder->decodeCoeff( pcCU, uiAbsPartIdx, uiDepth, uiWidth, uiHeight, bCodeDQP );
#else
  m_pcEntropyDecoder->decodeCoeff( pcCU, uiAbsPartIdx, uiDepth, uiCurrWidth, uiCurrHeight, bCodeDQP );
#endif
  setdQPFlag( bCodeDQP );
#if QT_BT_STRUCTURE
  if(  pcCU->getSlice()->getPPS()->getUseDQP())
  {
    pcCU->setQPSubParts( getdQPFlag()?pcCU->getRefQP(uiAbsPartIdx):pcCU->getCodedQP(), uiAbsPartIdx, uiDepth ); // set QP
  }
#else
  xFinishDecodeCU( pcCU, uiAbsPartIdx, uiDepth, ruiIsLast );
#endif
}

Void TDecCu::xFinishDecodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt& ruiIsLast)
{
#if !QT_BT_STRUCTURE
  if(  pcCU->getSlice()->getPPS()->getUseDQP())
  {
    pcCU->setQPSubParts( getdQPFlag()?pcCU->getRefQP(uiAbsPartIdx):pcCU->getCodedQP(), uiAbsPartIdx, uiDepth ); // set QP
  }
#endif

  ruiIsLast = xDecodeSliceEnd( pcCU, uiAbsPartIdx, uiDepth);
}

#if QT_BT_STRUCTURE
Void TDecCu::xDecompressCU( TComDataCU* pcCU, UInt uiAbsPartIdx,  UInt uiDepth, UInt uiWidth, UInt uiHeight )
#else
Void TDecCu::xDecompressCU( TComDataCU* pcCU, UInt uiAbsPartIdx,  UInt uiDepth )
#endif
{
  TComPic* pcPic = pcCU->getPic();

  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;

  UInt uiCurNumParts    = pcPic->getNumPartInCU() >> (uiDepth<<1);
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  Bool bStartInCU = pcCU->getSCUAddr()+uiAbsPartIdx+uiCurNumParts>pcSlice->getSliceSegmentCurStartCUAddr()&&pcCU->getSCUAddr()+uiAbsPartIdx<pcSlice->getSliceSegmentCurStartCUAddr();
#if QT_BT_STRUCTURE
  if ((g_uiMaxCUWidth>>uiDepth)==uiWidth && (g_uiMaxCUHeight>>uiDepth)==uiHeight)
  {
    assert(uiWidth==uiHeight);
#endif
    if(bStartInCU||( uiRPelX >= pcSlice->getSPS()->getPicWidthInLumaSamples() ) || ( uiBPelY >= pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
    {
      bBoundary = true;
    }

#if QT_BT_STRUCTURE
    if( (  uiDepth < pcCU->getDepth( uiAbsPartIdx ) )  || bBoundary )
#else
    if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth ) ) || bBoundary )
#endif
    {
      UInt uiNextDepth = uiDepth + 1;
      UInt uiQNumParts = pcCU->getTotalNumPart() >> (uiNextDepth<<1);
      UInt uiIdx = uiAbsPartIdx;
      for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++ )
      {
        uiLPelX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
        uiTPelY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];

        Bool binSlice = (pcCU->getSCUAddr()+uiIdx+uiQNumParts>pcSlice->getSliceSegmentCurStartCUAddr())&&(pcCU->getSCUAddr()+uiIdx<pcSlice->getSliceSegmentCurEndCUAddr());
        if(binSlice&&( uiLPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
        {
#if QT_BT_STRUCTURE
          xDecompressCU(pcCU, uiIdx, uiNextDepth, uiWidth>>1, uiHeight>>1 );
#else
          xDecompressCU(pcCU, uiIdx, uiNextDepth );
#endif
        }
#if QT_BT_STRUCTURE
        else
        {
          pcCU->getPic()->addCodedAreaInCTU(uiWidth*uiHeight>>2);
        }
#endif

        uiIdx += uiQNumParts;
      }
      return;
    }
#if QT_BT_STRUCTURE
  }
  UInt uiBTDepth = pcCU->getBTDepth(uiAbsPartIdx, uiWidth, uiHeight);

  if (pcCU->getBTSplitModeForBTDepth(uiAbsPartIdx, uiBTDepth)==1)
  {
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 2; uiPartUnitIdx++ )
    {
      if (uiPartUnitIdx==1)
      {
        uiAbsPartIdx = g_auiRasterToZscan[g_auiZscanToRaster[uiAbsPartIdx] 
        + (uiHeight>>1)/pcCU->getPic()->getMinCUHeight()*pcCU->getPic()->getNumPartInWidth()];
      }
      xDecompressCU( pcCU, uiAbsPartIdx, uiDepth, uiWidth, uiHeight>>1 );
    }
    return;
  }
  else if (pcCU->getBTSplitModeForBTDepth(uiAbsPartIdx, uiBTDepth)==2)
  {
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 2; uiPartUnitIdx++ )
    {
      if (uiPartUnitIdx==1)
      {
        uiAbsPartIdx = g_auiRasterToZscan[g_auiZscanToRaster[uiAbsPartIdx] 
        + (uiWidth>>1)/pcCU->getPic()->getMinCUWidth()];
      }
      xDecompressCU( pcCU, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight );
    }
    return;
  }

  UInt uiWidthIdx = g_aucConvertToBit[uiWidth];
  UInt uiHeightIdx = g_aucConvertToBit[uiHeight];
  m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx]->clear();

  m_pppcCUPU[uiWidthIdx][uiHeightIdx]->copySubCU(pcCU, uiAbsPartIdx, uiDepth);

  switch (m_pppcCUPU[uiWidthIdx][uiHeightIdx]->getPredictionMode(0))
  {
  case MODE_INTER:
    xReconInter( m_pppcCUPU[uiWidthIdx][uiHeightIdx], uiDepth );
    break;
  case MODE_INTRA:
    xReconIntraQT( m_pppcCUPU[uiWidthIdx][uiHeightIdx], uiDepth );
    break;
  default:
    assert(0);
    break;
  }

  UInt uiBlkX = g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ] >> MIN_CU_LOG2;
  UInt uiBlkY = g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ] >> MIN_CU_LOG2;
  pcCU->getPic()->setCodedBlkInCTU(true, uiBlkX, uiBlkY, uiWidth>> MIN_CU_LOG2, uiHeight>> MIN_CU_LOG2);
  pcCU->getPic()->addCodedAreaInCTU(uiWidth*uiHeight);
#else

    // Residual reconstruction
    m_ppcYuvResi[uiDepth]->clear();

    m_ppcCU[uiDepth]->copySubCU( pcCU, uiAbsPartIdx, uiDepth );

    switch( m_ppcCU[uiDepth]->getPredictionMode(0) )
    {
    case MODE_INTER:
      xReconInter( m_ppcCU[uiDepth], uiDepth );
      break;
    case MODE_INTRA:
      xReconIntraQT( m_ppcCU[uiDepth], uiDepth );
      break;
    default:
      assert(0);
      break;
    }
    if ( m_ppcCU[uiDepth]->isLosslessCoded(0) && (m_ppcCU[uiDepth]->getIPCMFlag(0) == false))
    {
      xFillPCMBuffer(m_ppcCU[uiDepth], uiDepth);
    }

    xCopyToPic( m_ppcCU[uiDepth], pcPic, uiAbsPartIdx, uiDepth );
#endif
  }

  Void TDecCu::xReconInter( TComDataCU* pcCU, UInt uiDepth )
  {

    // inter prediction
#if QT_BT_STRUCTURE
    UInt uiWidthIdx = g_aucConvertToBit[pcCU->getWidth(0)];
    UInt uiHeightIdx = g_aucConvertToBit[pcCU->getHeight(0)];
    m_pcPrediction->motionCompensation( pcCU, m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx] );
#else
    m_pcPrediction->motionCompensation( pcCU, m_ppcYuvReco[uiDepth] );
#endif

    // inter recon
    xDecodeInterTexture( pcCU, 0, uiDepth );

    // clip for only non-zero cbp case
    if  ( ( pcCU->getCbf( 0, TEXT_LUMA ) ) || ( pcCU->getCbf( 0, TEXT_CHROMA_U ) ) || ( pcCU->getCbf(0, TEXT_CHROMA_V ) ) )
    {
#if QT_BT_STRUCTURE
      m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->addClip( m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx], m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx], 0, pcCU->getWidth( 0 ), pcCU->getHeight(0) );
#else
      m_ppcYuvReco[uiDepth]->addClip( m_ppcYuvReco[uiDepth], m_ppcYuvResi[uiDepth], 0, pcCU->getWidth( 0 ) );
#endif
    }
#if QT_BT_STRUCTURE
    m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->copyToPicYuv(pcCU->getPic()->getPicYuvRec(), pcCU->getAddr(), pcCU->getZorderIdxInCU());
#else
    else
    {
      m_ppcYuvReco[uiDepth]->copyPartToPartYuv( m_ppcYuvReco[uiDepth],0, pcCU->getWidth( 0 ),pcCU->getHeight( 0 ));
    }
#endif
  }

  Void
    TDecCu::xIntraRecLumaBlk( TComDataCU* pcCU,
#if !QT_BT_STRUCTURE
    UInt        uiTrDepth,
    UInt        uiAbsPartIdx,
#endif
    TComYuv*    pcRecoYuv,
    TComYuv*    pcPredYuv, 
    TComYuv*    pcResiYuv )
  {
#if QT_BT_STRUCTURE
    UInt uiWidth = pcCU->getWidth(0);
    UInt uiHeight = pcCU->getHeight(0);

    UInt    uiStride          = pcRecoYuv->getStride  ();
    Pel*  piReco            = pcRecoYuv->getLumaAddr( 0 );
    Pel*  piPred            = pcPredYuv->getLumaAddr( 0 );
    Pel*  piResi            = pcResiYuv->getLumaAddr( 0 );
    Pel*    piRecIPred;
#else
    UInt    uiWidth           = pcCU     ->getWidth   ( 0 ) >> uiTrDepth;
    UInt    uiHeight          = pcCU     ->getHeight  ( 0 ) >> uiTrDepth;
    UInt    uiStride          = pcRecoYuv->getStride  ();
    Pel*    piReco            = pcRecoYuv->getLumaAddr( uiAbsPartIdx );
    Pel*    piPred            = pcPredYuv->getLumaAddr( uiAbsPartIdx );
    Pel*    piResi            = pcResiYuv->getLumaAddr( uiAbsPartIdx );
#endif

#if QT_BT_STRUCTURE
    TCoeff* pcCoeff           = pcCU->getCoeffY();

    UInt    uiLumaPredMode    = pcCU->getLumaIntraDir     ( 0 );

    UInt    uiZOrder          = pcCU->getZorderIdxInCU() ;
#else
    UInt    uiNumCoeffInc     = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
    TCoeff* pcCoeff           = pcCU->getCoeffY() + ( uiNumCoeffInc * uiAbsPartIdx );

    UInt    uiLumaPredMode    = pcCU->getLumaIntraDir     ( uiAbsPartIdx );

    UInt    uiZOrder          = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
#endif
#if QT_BT_STRUCTURE
    piRecIPred        = pcCU->getPic()->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), uiZOrder );
#else
    Pel*    piRecIPred        = pcCU->getPic()->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), uiZOrder );
#endif
    UInt    uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getStride  ();
#if QT_BT_STRUCTURE
    Bool    useTransformSkip  = pcCU->getTransformSkip(0, TEXT_LUMA);
#else
    Bool    useTransformSkip  = pcCU->getTransformSkip(uiAbsPartIdx, TEXT_LUMA);
#endif
    //===== init availability pattern =====
    Bool  bAboveAvail = false;
    Bool  bLeftAvail  = false;

#if QT_BT_STRUCTURE
    pcCU->getPattern()->initPattern   ( pcCU, 0, 0,0,0 );
    pcCU->getPattern()->initAdiPattern( pcCU, 0, uiWidth, uiHeight, 
      m_pcPrediction->getPredicBuf       (),
      m_pcPrediction->getPredicBufWidth  (),
      m_pcPrediction->getPredicBufHeight ());
    bAboveAvail = true;
    bLeftAvail = true;
#else
    pcCU->getPattern()->initPattern   ( pcCU, uiTrDepth, uiAbsPartIdx );
    pcCU->getPattern()->initAdiPattern( pcCU, uiAbsPartIdx, uiTrDepth, 
      m_pcPrediction->getPredicBuf       (),
      m_pcPrediction->getPredicBufWidth  (),
      m_pcPrediction->getPredicBufHeight (),
      bAboveAvail, bLeftAvail );
#endif

    //===== get prediction signal =====
#if QT_BT_STRUCTURE
    m_pcPrediction->predIntraLumaAng( pcCU->getPattern(), uiLumaPredMode, piPred, uiStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail );
    if ( pcCU->getCbf( 0, TEXT_LUMA ) )
#else
    m_pcPrediction->predIntraLumaAng( pcCU->getPattern(), uiLumaPredMode, piPred, uiStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail );
    if ( pcCU->getCbf( uiAbsPartIdx, TEXT_LUMA, uiTrDepth ) )
#endif
    {
      //===== inverse transform =====
      m_pcTrQuant->setQPforQuant  ( pcCU->getQP(0), TEXT_LUMA, pcCU->getSlice()->getSPS()->getQpBDOffsetY(), 0 );

#if QT_BT_STRUCTURE
      Int scalingListType = (pcCU->isIntra(0) ? 0 : 3) + g_eTTable[(Int)TEXT_LUMA];
#else
      Int scalingListType = (pcCU->isIntra(uiAbsPartIdx) ? 0 : 3) + g_eTTable[(Int)TEXT_LUMA];
#endif
      assert(scalingListType < SCALING_LIST_NUM);

#if QT_BT_STRUCTURE
#if ITSKIP
      assert(uiWidth > pcCU->m_puiLastX[0]);
      m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(0), TEXT_LUMA, pcCU->getLumaIntraDir( 0 ), piResi, uiStride, pcCoeff, uiWidth, uiHeight, scalingListType, useTransformSkip, uiWidth - pcCU->m_puiLastX[0] - 1, uiHeight-pcCU->m_puiLastY[0]-1 );
#else
      m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_LUMA, pcCU->getLumaIntraDir( uiAbsPartIdx ), piResi, uiStride, pcCoeff, uiTrWidth, uiTrHeight, scalingListType, useTransformSkip );
#endif
#else
      m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_LUMA, pcCU->getLumaIntraDir( uiAbsPartIdx ), piResi, uiStride, pcCoeff, uiWidth, uiHeight, scalingListType, useTransformSkip );
#endif


      //===== reconstruction =====
      Pel* pPred      = piPred;
      Pel* pResi      = piResi;
      Pel* pReco      = piReco;
      Pel* pRecIPred  = piRecIPred;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pReco    [ uiX ] = ClipY( pPred[ uiX ] + pResi[ uiX ] );
          pRecIPred[ uiX ] = pReco[ uiX ];
        }
        pPred     += uiStride;
        pResi     += uiStride;
        pReco     += uiStride;
        pRecIPred += uiRecIPredStride;
      }
    }
    else
    {
      //===== reconstruction =====
      Pel* pPred      = piPred;
      Pel* pReco      = piReco;
      Pel* pRecIPred  = piRecIPred;
      for ( Int y = 0; y < uiHeight; y++ )
      {
        for ( Int x = 0; x < uiWidth; x++ )
        {
          pReco    [ x ] = pPred[ x ];
          pRecIPred[ x ] = pReco[ x ];
        }
        pPred     += uiStride;
        pReco     += uiStride;
        pRecIPred += uiRecIPredStride;
      }
    }
  }


  Void
    TDecCu::xIntraRecChromaBlk( TComDataCU* pcCU,
#if !QT_BT_STRUCTURE
    UInt        uiTrDepth,
    UInt        uiAbsPartIdx,
#endif
    TComYuv*    pcRecoYuv,
    TComYuv*    pcPredYuv, 
    TComYuv*    pcResiYuv,
    UInt        uiChromaId )
  {
#if !QT_BT_STRUCTURE
    UInt uiFullDepth  = pcCU->getDepth( 0 ) + uiTrDepth;
    UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;

    if( uiLog2TrSize == 2 )
    {
      assert( uiTrDepth > 0 );
      uiTrDepth--;
      UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth ) << 1 );
      Bool bFirstQ = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
      if( !bFirstQ )
      {
        return;
      }
    }
#endif

    TextType  eText             = ( uiChromaId > 0 ? TEXT_CHROMA_V : TEXT_CHROMA_U );
#if QT_BT_STRUCTURE
    UInt      uiWidth           = pcCU->getWidth(0)>>1;
    UInt      uiHeight          = pcCU->getHeight(0)>>1;
#else
    UInt      uiWidth           = pcCU     ->getWidth   ( 0 ) >> ( uiTrDepth + 1 );
    UInt      uiHeight          = pcCU     ->getHeight  ( 0 ) >> ( uiTrDepth + 1 );
#endif
    UInt      uiStride          = pcRecoYuv->getCStride ();
#if QT_BT_STRUCTURE
    Pel*      piReco            = ( uiChromaId > 0 ? pcRecoYuv->getCrAddr( 0 ) : pcRecoYuv->getCbAddr( 0 ) );
    Pel*      piPred            = ( uiChromaId > 0 ? pcPredYuv->getCrAddr( 0 ) : pcPredYuv->getCbAddr( 0 ) );
    Pel*      piResi            = ( uiChromaId > 0 ? pcResiYuv->getCrAddr( 0 ) : pcResiYuv->getCbAddr( 0 ) );
#else
    Pel*      piReco            = ( uiChromaId > 0 ? pcRecoYuv->getCrAddr( uiAbsPartIdx ) : pcRecoYuv->getCbAddr( uiAbsPartIdx ) );
    Pel*      piPred            = ( uiChromaId > 0 ? pcPredYuv->getCrAddr( uiAbsPartIdx ) : pcPredYuv->getCbAddr( uiAbsPartIdx ) );
    Pel*      piResi            = ( uiChromaId > 0 ? pcResiYuv->getCrAddr( uiAbsPartIdx ) : pcResiYuv->getCbAddr( uiAbsPartIdx ) );
#endif

#if QT_BT_STRUCTURE
    TCoeff*   pcCoeff           =  uiChromaId > 0 ? pcCU->getCoeffCr() : pcCU->getCoeffCb()  ;
#else
    UInt      uiNumCoeffInc     = ( ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 ) ) >> 2;
    TCoeff*   pcCoeff           = ( uiChromaId > 0 ? pcCU->getCoeffCr() : pcCU->getCoeffCb() ) + ( uiNumCoeffInc * uiAbsPartIdx );
#endif

    UInt      uiChromaPredMode  = pcCU->getChromaIntraDir( 0 );

#if QT_BT_STRUCTURE
    UInt      uiZOrder          = pcCU->getZorderIdxInCU();
#else
    UInt      uiZOrder          = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
#endif
    Pel*      piRecIPred        = ( uiChromaId > 0 ? pcCU->getPic()->getPicYuvRec()->getCrAddr( pcCU->getAddr(), uiZOrder ) : pcCU->getPic()->getPicYuvRec()->getCbAddr( pcCU->getAddr(), uiZOrder ) );
    UInt      uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getCStride();
#if QT_BT_STRUCTURE
    Bool      useTransformSkipChroma = pcCU->getTransformSkip(0, eText);
#else
    Bool      useTransformSkipChroma = pcCU->getTransformSkip(uiAbsPartIdx,eText);
#endif
    //===== init availability pattern =====
    Bool  bAboveAvail = false;
    Bool  bLeftAvail  = false;
#if QT_BT_STRUCTURE
    pcCU->getPattern()->initPattern         ( pcCU, 0, 0, 0,0 );
#else
    pcCU->getPattern()->initPattern         ( pcCU, uiTrDepth, uiAbsPartIdx );
#endif

#if QT_BT_STRUCTURE
    pcCU->getPattern()->initAdiPatternChroma( pcCU, 0, 0,
#else
    pcCU->getPattern()->initAdiPatternChroma( pcCU, uiAbsPartIdx, uiTrDepth,
#endif
      m_pcPrediction->getPredicBuf       (),
      m_pcPrediction->getPredicBufWidth  (),
      m_pcPrediction->getPredicBufHeight (),
      bAboveAvail, bLeftAvail );
    Int* pPatChroma   = ( uiChromaId > 0 ? pcCU->getPattern()->getAdiCrBuf( uiWidth, uiHeight, m_pcPrediction->getPredicBuf() ) : pcCU->getPattern()->getAdiCbBuf( uiWidth, uiHeight, m_pcPrediction->getPredicBuf() ) );

    //===== get prediction signal =====
    {
      if( uiChromaPredMode == DM_CHROMA_IDX )
      {
#if QT_BT_STRUCTURE
        uiChromaPredMode = pcCU->getPic()->getCU(pcCU->getAddr())->getLumaIntraDir( pcCU->getZorderIdxInCU() );
#else
        uiChromaPredMode = pcCU->getLumaIntraDir( 0 );
#endif
      }
      m_pcPrediction->predIntraChromaAng( pPatChroma, uiChromaPredMode, piPred, uiStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail );  
    }

#if QT_BT_STRUCTURE
    if ( pcCU->getCbf( 0, eText ) )
#else
    if ( pcCU->getCbf( uiAbsPartIdx, eText, uiTrDepth ) )
#endif
    {
      //===== inverse transform =====
      Int curChromaQpOffset;
      if(eText == TEXT_CHROMA_U)
      {
        curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
      }
      else
      {
        curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
      }
      m_pcTrQuant->setQPforQuant  ( pcCU->getQP(0), eText, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

#if QT_BT_STRUCTURE
      Int scalingListType = (pcCU->isIntra(0) ? 0 : 3) + g_eTTable[(Int)eText];
#else
      Int scalingListType = (pcCU->isIntra(uiAbsPartIdx) ? 0 : 3) + g_eTTable[(Int)eText];
#endif
      assert(scalingListType < SCALING_LIST_NUM);
#if ITSKIP
      Int lastX = eText==TEXT_CHROMA_U ? pcCU->m_puiLastXCb[0]: pcCU->m_puiLastXCr[0];
      Int lastY = eText==TEXT_CHROMA_U ? pcCU->m_puiLastYCb[0]: pcCU->m_puiLastYCr[0];
      m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(0), eText, REG_DCT, piResi, uiStride, pcCoeff, uiWidth, uiHeight, scalingListType, useTransformSkipChroma, uiWidth-lastX-1, uiHeight-lastY-1 );
#else
      m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), eText, REG_DCT, piResi, uiStride, pcCoeff, uiWidth, uiHeight, scalingListType, useTransformSkipChroma );
#endif

      //===== reconstruction =====
      Pel* pPred      = piPred;
      Pel* pResi      = piResi;
      Pel* pReco      = piReco;
      Pel* pRecIPred  = piRecIPred;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pReco    [ uiX ] = ClipC( pPred[ uiX ] + pResi[ uiX ] );
          pRecIPred[ uiX ] = pReco[ uiX ];
        }
        pPred     += uiStride;
        pResi     += uiStride;
        pReco     += uiStride;
        pRecIPred += uiRecIPredStride;
      }
    }
    else
    {
      //===== reconstruction =====
      Pel* pPred      = piPred;
      Pel* pReco      = piReco;
      Pel* pRecIPred  = piRecIPred;
      for ( Int y = 0; y < uiHeight; y++ )
      {
        for ( Int x = 0; x < uiWidth; x++ )
        {
          pReco    [ x ] = pPred[ x ];
          pRecIPred[ x ] = pReco[ x ];
        }
        pPred     += uiStride;
        pReco     += uiStride;
        pRecIPred += uiRecIPredStride;
      }    
    }
  }


  Void
    TDecCu::xReconIntraQT( TComDataCU* pcCU, UInt uiDepth )
  {
    UInt  uiNumPart     = pcCU->getNumPartInter();
#if QT_BT_STRUCTURE
    UInt uiWidthIdx = g_aucConvertToBit[pcCU->getWidth(0)];
    UInt uiHeightIdx = g_aucConvertToBit[pcCU->getHeight(0)];
#else
    UInt  uiNumQParts     = pcCU->getTotalNumPart() >> 2;
#endif

    if (pcCU->getIPCMFlag(0))
    {
      xReconPCM( pcCU, uiDepth );
      return;
    }

#if QT_BT_STRUCTURE
    if (pcCU->getTextType()==TEXT_LUMA)
    {
#endif
      for( UInt uiPU = 0; uiPU < uiNumPart; uiPU++ )
      {
#if QT_BT_STRUCTURE
        xIntraLumaRecQT( pcCU, m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]
        , m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx], m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx] );
#else
        xIntraLumaRecQT( pcCU, uiInitTrDepth, uiPU * uiNumQParts, m_ppcYuvReco[uiDepth], m_ppcYuvReco[uiDepth], m_ppcYuvResi[uiDepth] );
#endif
      }  
#if QT_BT_STRUCTURE
    }
#endif

#if QT_BT_STRUCTURE
    if (pcCU->getTextType()!=TEXT_LUMA || !pcCU->getSlice()->isIntra())
    {
#endif
      for( UInt uiPU = 0; uiPU < uiNumPart; uiPU++ )
      {
#if QT_BT_STRUCTURE
        xIntraChromaRecQT( pcCU, m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx], m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx], m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx], TEXT_CHROMA_U );
        xIntraChromaRecQT( pcCU, m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx], m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx], m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx], TEXT_CHROMA_V );
#else
        xIntraChromaRecQT( pcCU, uiInitTrDepth, uiPU * uiNumQParts, m_ppcYuvReco[uiDepth], m_ppcYuvReco[uiDepth], m_ppcYuvResi[uiDepth] );
#endif
      }
#if QT_BT_STRUCTURE
    }
#endif

  }

  /** Function for deriving recontructed PU/CU Luma sample with QTree structure
  * \param pcCU pointer of current CU
  * \param uiTrDepth current tranform split depth
  * \param uiAbsPartIdx  part index
  * \param pcRecoYuv pointer to reconstructed sample arrays
  * \param pcPredYuv pointer to prediction sample arrays
  * \param pcResiYuv pointer to residue sample arrays
  * 
  \ This function dervies recontructed PU/CU Luma sample with recursive QTree structure
  */
  Void
    TDecCu::xIntraLumaRecQT( TComDataCU* pcCU,
#if !QT_BT_STRUCTURE
    UInt        uiTrDepth,
    UInt        uiAbsPartIdx,
#endif
    TComYuv*    pcRecoYuv,
    TComYuv*    pcPredYuv, 
    TComYuv*    pcResiYuv )
  {
#if QT_BT_STRUCTURE
    xIntraRecLumaBlk  ( pcCU, pcRecoYuv, pcPredYuv, pcResiYuv );
#else
    UInt uiFullDepth  = pcCU->getDepth(0) + uiTrDepth;
    UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
    if( uiTrMode == uiTrDepth )
    {
      xIntraRecLumaBlk  ( pcCU, uiTrDepth, uiAbsPartIdx, pcRecoYuv, pcPredYuv, pcResiYuv );
    }
    else
    {
      UInt uiNumQPart  = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
      for( UInt uiPart = 0; uiPart < 4; uiPart++ )
      {
        xIntraLumaRecQT( pcCU, uiTrDepth + 1, uiAbsPartIdx + uiPart * uiNumQPart, pcRecoYuv, pcPredYuv, pcResiYuv );
      }
    }
#endif
  }

  /** Function for deriving recontructed PU/CU chroma samples with QTree structure
  * \param pcCU pointer of current CU
  * \param uiTrDepth current tranform split depth
  * \param uiAbsPartIdx  part index
  * \param pcRecoYuv pointer to reconstructed sample arrays
  * \param pcPredYuv pointer to prediction sample arrays
  * \param pcResiYuv pointer to residue sample arrays
  * 
  \ This function dervies recontructed PU/CU chroma samples with QTree recursive structure
  */
  Void
    TDecCu::xIntraChromaRecQT( TComDataCU* pcCU,
#if !QT_BT_STRUCTURE
    UInt uiTrDepth, 
    UInt uiAbsPartIdx,
#endif
    TComYuv*    pcRecoYuv,
    TComYuv*    pcPredYuv, 
    TComYuv*    pcResiYuv 
#if QT_BT_STRUCTURE
    ,TextType   eType
#endif
    )
  {
#if !QT_BT_STRUCTURE
    UInt uiFullDepth  = pcCU->getDepth(0) + uiTrDepth;
    UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
    if( uiTrMode == uiTrDepth )
    {
#endif
#if QT_BT_STRUCTURE
      xIntraRecChromaBlk( pcCU, pcRecoYuv, pcPredYuv, pcResiYuv, eType==TEXT_CHROMA_U ? 0: 1 );
#else
      xIntraRecChromaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcRecoYuv, pcPredYuv, pcResiYuv, 0 );
      xIntraRecChromaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcRecoYuv, pcPredYuv, pcResiYuv, 1 );
#endif
#if !QT_BT_STRUCTURE
    }
    else
    {
      UInt uiNumQPart  = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
      for( UInt uiPart = 0; uiPart < 4; uiPart++ )
      {
        xIntraChromaRecQT( pcCU, uiTrDepth + 1, uiAbsPartIdx + uiPart * uiNumQPart, pcRecoYuv, pcPredYuv, pcResiYuv );
      }
    }
#endif
  }

#if !QT_BT_STRUCTURE
  Void TDecCu::xCopyToPic( TComDataCU* pcCU, TComPic* pcPic, UInt uiZorderIdx, UInt uiDepth )
  {
    UInt uiCUAddr = pcCU->getAddr();

    m_ppcYuvReco[uiDepth]->copyToPicYuv  ( pcPic->getPicYuvRec (), uiCUAddr, uiZorderIdx );

    return;
  }
#endif

  Void TDecCu::xDecodeInterTexture ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
  {
    UInt    uiWidth    = pcCU->getWidth ( uiAbsPartIdx );
    UInt    uiHeight   = pcCU->getHeight( uiAbsPartIdx );
#if QT_BT_STRUCTURE
    UInt uiWidthIdx = g_aucConvertToBit[uiWidth];
    UInt uiHeightIdx = g_aucConvertToBit[uiHeight];
#endif
    TCoeff* piCoeff;

    Pel*    pResi;
#if QT_BT_STRUCTURE
    UInt    trMode = 0;
#else
    UInt    trMode = pcCU->getTransformIdx( uiAbsPartIdx );
#endif

    // Y
    piCoeff = pcCU->getCoeffY();
#if QT_BT_STRUCTURE
    pResi = m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx]->getLumaAddr();
#else
    pResi = m_ppcYuvResi[uiDepth]->getLumaAddr();
#endif

    m_pcTrQuant->setQPforQuant( pcCU->getQP( uiAbsPartIdx ), TEXT_LUMA, pcCU->getSlice()->getSPS()->getQpBDOffsetY(), 0 );

#if QT_BT_STRUCTURE
    m_pcTrQuant->invRecurTransformNxN ( pcCU, 0, TEXT_LUMA, pResi, 0, m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx]->getStride(), uiWidth, uiHeight, trMode, 0, piCoeff );
#else
    m_pcTrQuant->invRecurTransformNxN ( pcCU, 0, TEXT_LUMA, pResi, 0, m_ppcYuvResi[uiDepth]->getStride(), uiWidth, uiHeight, trMode, 0, piCoeff );
#endif

    // Cb and Cr
    Int curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
    m_pcTrQuant->setQPforQuant( pcCU->getQP( uiAbsPartIdx ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

    uiWidth  >>= 1;
    uiHeight >>= 1;
#if QT_BT_STRUCTURE
    piCoeff = pcCU->getCoeffCb(); pResi = m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx]->getCbAddr();
    m_pcTrQuant->invRecurTransformNxN ( pcCU, 0, TEXT_CHROMA_U, pResi, 0, m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx]->getCStride(), uiWidth, uiHeight, trMode, 0, piCoeff );
#else
    piCoeff = pcCU->getCoeffCb(); pResi = m_ppcYuvResi[uiDepth]->getCbAddr();
    m_pcTrQuant->invRecurTransformNxN ( pcCU, 0, TEXT_CHROMA_U, pResi, 0, m_ppcYuvResi[uiDepth]->getCStride(), uiWidth, uiHeight, trMode, 0, piCoeff );
#endif

    curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
    m_pcTrQuant->setQPforQuant( pcCU->getQP( uiAbsPartIdx ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

#if QT_BT_STRUCTURE
    piCoeff = pcCU->getCoeffCr(); pResi = m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx]->getCrAddr();
    m_pcTrQuant->invRecurTransformNxN ( pcCU, 0, TEXT_CHROMA_V, pResi, 0, m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx]->getCStride(), uiWidth, uiHeight, trMode, 0, piCoeff );
#else
    piCoeff = pcCU->getCoeffCr(); pResi = m_ppcYuvResi[uiDepth]->getCrAddr();
    m_pcTrQuant->invRecurTransformNxN ( pcCU, 0, TEXT_CHROMA_V, pResi, 0, m_ppcYuvResi[uiDepth]->getCStride(), uiWidth, uiHeight, trMode, 0, piCoeff );
#endif
  }

  /** Function for deriving reconstructed luma/chroma samples of a PCM mode CU.
  * \param pcCU pointer to current CU
  * \param uiPartIdx part index
  * \param piPCM pointer to PCM code arrays
  * \param piReco pointer to reconstructed sample arrays
  * \param uiStride stride of reconstructed sample arrays
  * \param uiWidth CU width
  * \param uiHeight CU height
  * \param ttText texture component type
  * \returns Void
  */
  Void TDecCu::xDecodePCMTexture( TComDataCU* pcCU, UInt uiPartIdx, Pel *piPCM, Pel* piReco, UInt uiStride, UInt uiWidth, UInt uiHeight, TextType ttText)
  {
    UInt uiX, uiY;
    Pel* piPicReco;
    UInt uiPicStride;
    UInt uiPcmLeftShiftBit; 

    if( ttText == TEXT_LUMA )
    {
      uiPicStride   = pcCU->getPic()->getPicYuvRec()->getStride();
      piPicReco = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiPartIdx);
      uiPcmLeftShiftBit = g_bitDepthY - pcCU->getSlice()->getSPS()->getPCMBitDepthLuma();
    }
    else
    {
      uiPicStride = pcCU->getPic()->getPicYuvRec()->getCStride();

      if( ttText == TEXT_CHROMA_U )
      {
        piPicReco = pcCU->getPic()->getPicYuvRec()->getCbAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiPartIdx);
      }
      else
      {
        piPicReco = pcCU->getPic()->getPicYuvRec()->getCrAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiPartIdx);
      }
      uiPcmLeftShiftBit = g_bitDepthC - pcCU->getSlice()->getSPS()->getPCMBitDepthChroma();
    }

    for( uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( uiX = 0; uiX < uiWidth; uiX++ )
      {
        piReco[uiX] = (piPCM[uiX] << uiPcmLeftShiftBit);
        piPicReco[uiX] = piReco[uiX];
      }
      piPCM += uiWidth;
      piReco += uiStride;
      piPicReco += uiPicStride;
    }
  }

  /** Function for reconstructing a PCM mode CU.
  * \param pcCU pointer to current CU
  * \param uiDepth CU Depth
  * \returns Void
  */
  Void TDecCu::xReconPCM( TComDataCU* pcCU, UInt uiDepth )
  {
    // Luma
    UInt uiWidth  = (g_uiMaxCUWidth >> uiDepth);
    UInt uiHeight = (g_uiMaxCUHeight >> uiDepth);
#if QT_BT_STRUCTURE //modifying  just for compiling success
    UInt uiWidthIdx = g_aucConvertToBit[uiWidth];
    UInt uiHeightIdx = g_aucConvertToBit[uiHeight];
#endif

    Pel* piPcmY = pcCU->getPCMSampleY();
#if QT_BT_STRUCTURE
    Pel* piRecoY = m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->getLumaAddr(0, uiWidth);

    UInt uiStride = m_pppcYuvResiPU[uiWidthIdx][uiHeightIdx]->getStride();
#else
    Pel* piRecoY = m_ppcYuvReco[uiDepth]->getLumaAddr(0, uiWidth);

    UInt uiStride = m_ppcYuvResi[uiDepth]->getStride();
#endif

    xDecodePCMTexture( pcCU, 0, piPcmY, piRecoY, uiStride, uiWidth, uiHeight, TEXT_LUMA);

    // Cb and Cr
    UInt uiCWidth  = (uiWidth>>1);
    UInt uiCHeight = (uiHeight>>1);

    Pel* piPcmCb = pcCU->getPCMSampleCb();
    Pel* piPcmCr = pcCU->getPCMSampleCr();
#if QT_BT_STRUCTURE
    Pel* pRecoCb = m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->getCbAddr();
    Pel* pRecoCr = m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->getCrAddr();

    UInt uiCStride = m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->getCStride();
#else
    Pel* pRecoCb = m_ppcYuvReco[uiDepth]->getCbAddr();
    Pel* pRecoCr = m_ppcYuvReco[uiDepth]->getCrAddr();

    UInt uiCStride = m_ppcYuvReco[uiDepth]->getCStride();
#endif

    xDecodePCMTexture( pcCU, 0, piPcmCb, pRecoCb, uiCStride, uiCWidth, uiCHeight, TEXT_CHROMA_U);
    xDecodePCMTexture( pcCU, 0, piPcmCr, pRecoCr, uiCStride, uiCWidth, uiCHeight, TEXT_CHROMA_V);
  }

  /** Function for filling the PCM buffer of a CU using its reconstructed sample array 
  * \param pcCU pointer to current CU
  * \param uiDepth CU Depth
  * \returns Void
  */
  Void TDecCu::xFillPCMBuffer(TComDataCU* pCU, UInt depth)
  {
    // Luma
    UInt width  = (g_uiMaxCUWidth >> depth);
    UInt height = (g_uiMaxCUHeight >> depth);
#if QT_BT_STRUCTURE //modifying  just for compiling success
    UInt uiWidthIdx = g_aucConvertToBit[width];
    UInt uiHeightIdx = g_aucConvertToBit[height];
#endif

    Pel* pPcmY = pCU->getPCMSampleY();
#if QT_BT_STRUCTURE
    Pel* pRecoY = m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->getLumaAddr(0, width);

    UInt stride = m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->getStride();
#else
    Pel* pRecoY = m_ppcYuvReco[depth]->getLumaAddr(0, width);

    UInt stride = m_ppcYuvReco[depth]->getStride();
#endif

    for(Int y = 0; y < height; y++ )
    {
      for(Int x = 0; x < width; x++ )
      {
        pPcmY[x] = pRecoY[x];
      }
      pPcmY += width;
      pRecoY += stride;
    }

    // Cb and Cr
    UInt widthC  = (width>>1);
    UInt heightC = (height>>1);

    Pel* pPcmCb = pCU->getPCMSampleCb();
    Pel* pPcmCr = pCU->getPCMSampleCr();
#if QT_BT_STRUCTURE
    Pel* pRecoCb = m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->getCbAddr();
    Pel* pRecoCr = m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->getCrAddr();

    UInt strideC = m_pppcYuvRecoPU[uiWidthIdx][uiHeightIdx]->getCStride();
#else
    Pel* pRecoCb = m_ppcYuvReco[depth]->getCbAddr();
    Pel* pRecoCr = m_ppcYuvReco[depth]->getCrAddr();

    UInt strideC = m_ppcYuvReco[depth]->getCStride();
#endif

    for(Int y = 0; y < heightC; y++ )
    {
      for(Int x = 0; x < widthC; x++ )
      {
        pPcmCb[x] = pRecoCb[x];
        pPcmCr[x] = pRecoCr[x];
      }
      pPcmCr += widthC;
      pPcmCb += widthC;
      pRecoCb += strideC;
      pRecoCr += strideC;
    }

  }

//! \}
