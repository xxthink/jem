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

/** \file     TComPic.cpp
    \brief    picture class
*/

#include "TComPic.h"
#include "SEI.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComPic::TComPic()
: m_uiTLayer                              (0)
, m_bUsedByCurr                           (false)
, m_bIsLongTerm                           (false)
, m_pcPicYuvPred                          (NULL)
, m_pcPicYuvResi                          (NULL)
, m_bReconstructed                        (false)
, m_bNeededForOutput                      (false)
, m_uiCurrSliceIdx                        (0)
, m_bCheckLTMSB                           (false)
{
  for(UInt i=0; i<NUM_PIC_YUV; i++)
  {
    m_apcPicYuv[i]      = NULL;
  }
#if INTER_KLT
  m_apcQuaPicYuv[0][0] = NULL; m_apcQuaPicYuv[0][1] = NULL; m_apcQuaPicYuv[0][2] = NULL; m_apcQuaPicYuv[0][3] = NULL;
  m_apcQuaPicYuv[1][0] = NULL; m_apcQuaPicYuv[1][1] = NULL; m_apcQuaPicYuv[1][2] = NULL; m_apcQuaPicYuv[1][3] = NULL;
  m_apcQuaPicYuv[2][0] = NULL; m_apcQuaPicYuv[2][1] = NULL; m_apcQuaPicYuv[2][2] = NULL; m_apcQuaPicYuv[2][3] = NULL;
  m_apcQuaPicYuv[3][0] = NULL; m_apcQuaPicYuv[3][1] = NULL; m_apcQuaPicYuv[3][2] = NULL; m_apcQuaPicYuv[3][3] = NULL;
#endif
}

TComPic::~TComPic()
{
}

Void TComPic::create( const TComSPS &sps, const TComPPS &pps, const Bool bIsVirtual)
{
  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
  const Int          iWidth          = sps.getPicWidthInLumaSamples();
  const Int          iHeight         = sps.getPicHeightInLumaSamples();
  const UInt         uiMaxCuWidth    = sps.getMaxCUWidth();
  const UInt         uiMaxCuHeight   = sps.getMaxCUHeight();
  const UInt         uiMaxDepth      = sps.getMaxTotalCUDepth();

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  m_iNumCuInWidth                    = iWidth / uiMaxCuWidth;
  m_iNumCuInWidth                   += ( iWidth % uiMaxCuWidth ) ? 1 : 0;
  m_iBaseUnitWidth                   = uiMaxCuWidth  >> uiMaxDepth;
  m_iBaseUnitHeight                  = uiMaxCuHeight >> uiMaxDepth;
#endif

  m_picSym.create( sps, pps, uiMaxDepth );
  if (!bIsVirtual)
  {
    m_apcPicYuv[PIC_YUV_ORG    ]   = new TComPicYuv;  m_apcPicYuv[PIC_YUV_ORG     ]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
    m_apcPicYuv[PIC_YUV_TRUE_ORG]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_TRUE_ORG]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
  }
  m_apcPicYuv[PIC_YUV_REC]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_REC]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );

#if INTER_KLT
  for (UInt uiRow = 0; uiRow < 4; uiRow++)
  {
      for (UInt uiCol = 0; uiCol < 4; uiCol++)
      {
          if (uiRow == 0 && uiCol == 0)
          {
              m_apcQuaPicYuv[uiRow][uiCol] = m_apcPicYuv[PIC_YUV_REC];
          }
          else
          {
              m_apcQuaPicYuv[uiRow][uiCol] = new TComPicYuv;
              m_apcQuaPicYuv[uiRow][uiCol]->create(iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true);
          }
      }
  }
#endif 

  // there are no SEI messages associated with this picture initially
  if (m_SEIs.size() > 0)
  {
    deleteSEIs (m_SEIs);
  }
  m_bUsedByCurr = false;
}

Void TComPic::destroy()
{
  m_picSym.destroy();

  for(UInt i=0; i<NUM_PIC_YUV; i++)
  {
    if (m_apcPicYuv[i])
    {
      m_apcPicYuv[i]->destroy();
      delete m_apcPicYuv[i];
      m_apcPicYuv[i]  = NULL;
    }
  }

#if INTER_KLT
  for (UInt uiRow = 0; uiRow < 4; uiRow++)
  {
      for (UInt uiCol = 0; uiCol < 4; uiCol++)
      {
          if (uiRow == 0 && uiCol == 0)
          {
              continue;
          }
          if (m_apcQuaPicYuv[uiRow][uiCol])
          {
              m_apcQuaPicYuv[uiRow][uiCol]->destroy();
              delete m_apcQuaPicYuv[uiRow][uiCol];
              m_apcQuaPicYuv[uiRow][uiCol] = NULL;
          }
      }
  }
#endif
  deleteSEIs(m_SEIs);
}

Void TComPic::compressMotion()
{
  TComPicSym* pPicSym = getPicSym();
  for ( UInt uiCUAddr = 0; uiCUAddr < pPicSym->getNumberOfCtusInFrame(); uiCUAddr++ )
  {
    TComDataCU* pCtu = pPicSym->getCtu(uiCUAddr);
    pCtu->compressMV();
  }
}

Bool  TComPic::getSAOMergeAvailability(Int currAddr, Int mergeAddr)
{
  Bool mergeCtbInSliceSeg = (mergeAddr >= getPicSym()->getCtuTsToRsAddrMap(getCtu(currAddr)->getSlice()->getSliceCurStartCtuTsAddr()));
  Bool mergeCtbInTile     = (getPicSym()->getTileIdxMap(mergeAddr) == getPicSym()->getTileIdxMap(currAddr));
  return (mergeCtbInSliceSeg && mergeCtbInTile);
}

UInt TComPic::getSubstreamForCtuAddr(const UInt ctuAddr, const Bool bAddressInRaster, TComSlice *pcSlice)
{
  UInt subStrm;
  const bool bWPPEnabled=pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();
  const TComPicSym &picSym            = *(getPicSym());

  if ((bWPPEnabled && picSym.getFrameHeightInCtus()>1) || (picSym.getNumTiles()>1)) // wavefronts, and possibly tiles being used.
  {
    if (bWPPEnabled)
    {
      const UInt ctuRsAddr                = bAddressInRaster?ctuAddr : picSym.getCtuTsToRsAddrMap(ctuAddr);
      const UInt frameWidthInCtus         = picSym.getFrameWidthInCtus();
      const UInt tileIndex                = picSym.getTileIdxMap(ctuRsAddr);
      const UInt numTileColumns           = (picSym.getNumTileColumnsMinus1()+1);
      const TComTile *pTile               = picSym.getTComTile(tileIndex);
      const UInt firstCtuRsAddrOfTile     = pTile->getFirstCtuRsAddr();
      const UInt tileYInCtus              = firstCtuRsAddrOfTile / frameWidthInCtus;
      // independent tiles => substreams are "per tile"
      const UInt ctuLine                  = ctuRsAddr / frameWidthInCtus;
      const UInt startingSubstreamForTile =(tileYInCtus*numTileColumns) + (pTile->getTileHeightInCtus()*(tileIndex%numTileColumns));
      subStrm = startingSubstreamForTile + (ctuLine - tileYInCtus);
    }
    else
    {
      const UInt ctuRsAddr                = bAddressInRaster?ctuAddr : picSym.getCtuTsToRsAddrMap(ctuAddr);
      const UInt tileIndex                = picSym.getTileIdxMap(ctuRsAddr);
      subStrm=tileIndex;
    }
  }
  else
  {
    // dependent tiles => substreams are "per frame".
    subStrm = 0;
  }
  return subStrm;
}
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
Void TComPic::getCUAddrAndPartIdx( Int iX, Int iY, Int& riCuAddr, Int& riAbsZorderIdx )
{
  Int iMaxCUWidth   = (Int) ( getPicSym()->getSPS().getMaxCUWidth()  );
  Int iMaxCuHeight  = (Int) ( getPicSym()->getSPS().getMaxCUHeight() );

  Int iCuX            = iX / iMaxCUWidth;
  Int iCuY            = iY / iMaxCuHeight;
  Int iBaseX          = ( iX - iCuX * iMaxCUWidth  ) / m_iBaseUnitWidth;
  Int iBaseY          = ( iY - iCuY * iMaxCuHeight ) / m_iBaseUnitHeight;
  Int iCuSizeInBases  = iMaxCUWidth                  / m_iBaseUnitWidth;
  riCuAddr            = iCuY   * m_iNumCuInWidth + iCuX;
  Int iRastPartIdx    = iBaseY * iCuSizeInBases  + iBaseX;
  riAbsZorderIdx      = g_auiRasterToZscan[ iRastPartIdx ];
}
#endif

//! \}
