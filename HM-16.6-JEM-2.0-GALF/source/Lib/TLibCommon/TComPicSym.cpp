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

/** \file     TComPicSym.cpp
    \brief    picture symbol class
*/

#include "TComPicSym.h"
#include "TComSampleAdaptiveOffset.h"
#include "TComSlice.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComPicSym::TComPicSym()
:m_frameWidthInCtus(0)
,m_frameHeightInCtus(0)
,m_uiMinCUWidth(0)
,m_uiMinCUHeight(0)
,m_uhTotalDepth(0)
,m_numPartitionsInCtu(0)
,m_numPartInCtuWidth(0)
,m_numPartInCtuHeight(0)
,m_numCtusInFrame(0)
,m_apSlices()
,m_pictureCtuArray(NULL)
,m_numTileColumnsMinus1(0)
,m_numTileRowsMinus1(0)
,m_ctuTsToRsAddrMap(NULL)
,m_puiTileIdxMap(NULL)
,m_ctuRsToTsAddrMap(NULL)
,m_saoBlkParams(NULL)
#if ADAPTIVE_QP_SELECTION
,m_pParentARLBuffer(NULL)
#endif
{}


Void TComPicSym::create  ( const TComSPS &sps, const TComPPS &pps, UInt uiMaxDepth )
{
  UInt i;
  m_sps = sps;
  m_pps = pps;

  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
  const Int iPicWidth      = sps.getPicWidthInLumaSamples();
  const Int iPicHeight     = sps.getPicHeightInLumaSamples();
  const UInt uiMaxCuWidth  = sps.getMaxCUWidth();
  const UInt uiMaxCuHeight = sps.getMaxCUHeight();

  m_uhTotalDepth       = uiMaxDepth;
  m_numPartitionsInCtu = 1<<(m_uhTotalDepth<<1);

  m_uiMinCUWidth       = uiMaxCuWidth  >> m_uhTotalDepth;
  m_uiMinCUHeight      = uiMaxCuHeight >> m_uhTotalDepth;

  m_numPartInCtuWidth  = uiMaxCuWidth  / m_uiMinCUWidth;  // equivalent to 1<<m_uhTotalDepth
  m_numPartInCtuHeight = uiMaxCuHeight / m_uiMinCUHeight; // equivalent to 1<<m_uhTotalDepth

  m_frameWidthInCtus   = ( iPicWidth %uiMaxCuWidth  ) ? iPicWidth /uiMaxCuWidth  + 1 : iPicWidth /uiMaxCuWidth;
  m_frameHeightInCtus  = ( iPicHeight%uiMaxCuHeight ) ? iPicHeight/uiMaxCuHeight + 1 : iPicHeight/uiMaxCuHeight;

  m_numCtusInFrame     = m_frameWidthInCtus * m_frameHeightInCtus;
  m_pictureCtuArray    = new TComDataCU*[m_numCtusInFrame];

  clearSliceBuffer();
  allocateNewSlice();

#if ADAPTIVE_QP_SELECTION
  if (m_pParentARLBuffer == NULL)
  {
     m_pParentARLBuffer = new TCoeff[uiMaxCuWidth*uiMaxCuHeight*MAX_NUM_COMPONENT];
  }
#endif

  for ( i=0; i<m_numCtusInFrame ; i++ )
  {
    m_pictureCtuArray[i] = new TComDataCU;
    m_pictureCtuArray[i]->create( chromaFormatIDC, m_numPartitionsInCtu, uiMaxCuWidth, uiMaxCuHeight, false, uiMaxCuWidth >> m_uhTotalDepth
#if ADAPTIVE_QP_SELECTION
      , m_pParentARLBuffer
#endif
      );
  }

  m_ctuTsToRsAddrMap = new UInt[m_numCtusInFrame+1];
  m_puiTileIdxMap    = new UInt[m_numCtusInFrame];
  m_ctuRsToTsAddrMap = new UInt[m_numCtusInFrame+1];

  for( i=0; i<m_numCtusInFrame; i++ )
  {
    m_ctuTsToRsAddrMap[i] = i;
    m_ctuRsToTsAddrMap[i] = i;
  }

  m_saoBlkParams = new SAOBlkParam[m_numCtusInFrame];


  xInitTiles();
  xInitCtuTsRsAddrMaps();

}

Void TComPicSym::destroy()
{
  clearSliceBuffer();

  for (Int i = 0; i < m_numCtusInFrame; i++)
  {
    m_pictureCtuArray[i]->destroy();
    delete m_pictureCtuArray[i];
    m_pictureCtuArray[i] = NULL;
  }
  delete [] m_pictureCtuArray;
  m_pictureCtuArray = NULL;

  delete [] m_ctuTsToRsAddrMap;
  m_ctuTsToRsAddrMap = NULL;

  delete [] m_puiTileIdxMap;
  m_puiTileIdxMap = NULL;

  delete [] m_ctuRsToTsAddrMap;
  m_ctuRsToTsAddrMap = NULL;

  if(m_saoBlkParams)
  {
    delete[] m_saoBlkParams; m_saoBlkParams = NULL;
  }

#if ADAPTIVE_QP_SELECTION
  delete [] m_pParentARLBuffer;
  m_pParentARLBuffer = NULL;
#endif
}

Void TComPicSym::allocateNewSlice()
{
  m_apSlices.push_back(new TComSlice);
  m_apSlices.back()->setPPS(&m_pps);
  m_apSlices.back()->setSPS(&m_sps);
  if (m_apSlices.size()>=2)
  {
    m_apSlices.back()->copySliceInfo( m_apSlices[m_apSlices.size()-2] );
    m_apSlices.back()->initSlice();
  }
}

Void TComPicSym::clearSliceBuffer()
{
  for (UInt i = 0; i < UInt(m_apSlices.size()); i++)
  {
    delete m_apSlices[i];
  }
  m_apSlices.clear();
}

Void TComPicSym::xInitCtuTsRsAddrMaps()
{
  //generate the Coding Order Map and Inverse Coding Order Map
  for(Int ctuTsAddr=0, ctuRsAddr=0; ctuTsAddr<getNumberOfCtusInFrame(); ctuTsAddr++, ctuRsAddr = xCalculateNextCtuRSAddr(ctuRsAddr))
  {
    setCtuTsToRsAddrMap(ctuTsAddr, ctuRsAddr);
    setCtuRsToTsAddrMap(ctuRsAddr, ctuTsAddr);
  }
  setCtuTsToRsAddrMap(getNumberOfCtusInFrame(), getNumberOfCtusInFrame());
  setCtuRsToTsAddrMap(getNumberOfCtusInFrame(), getNumberOfCtusInFrame());
}

Void TComPicSym::xInitTiles()
{
  //set NumColumnsMinus1 and NumRowsMinus1
  setNumTileColumnsMinus1( m_pps.getNumTileColumnsMinus1() );
  setNumTileRowsMinus1(    m_pps.getNumTileRowsMinus1()    );

  const Int numCols = m_pps.getNumTileColumnsMinus1() + 1;
  const Int numRows = m_pps.getNumTileRowsMinus1() + 1;
  const Int numTiles = numRows * numCols;

  // allocate memory for tile parameters
  m_tileParameters.resize(numTiles);

  if( m_pps.getTileUniformSpacingFlag() )
  {
    //set width and height for each (uniform) tile
    for(Int row=0; row < numRows; row++)
    {
      for(Int col=0; col < numCols; col++)
      {
        const Int tileIdx = row * numCols + col;
        m_tileParameters[tileIdx].setTileWidthInCtus(  (col+1)*getFrameWidthInCtus( )/numCols - (col*getFrameWidthInCtus( ))/numCols );
        m_tileParameters[tileIdx].setTileHeightInCtus( (row+1)*getFrameHeightInCtus()/numRows - (row*getFrameHeightInCtus())/numRows );
      }
    }
  }
  else
  {
    //set the width for each tile
    for(Int row=0; row < numRows; row++)
    {
      Int cumulativeTileWidth = 0;
      for(Int col=0; col < getNumTileColumnsMinus1(); col++)
      {
        m_tileParameters[row * numCols + col].setTileWidthInCtus( m_pps.getTileColumnWidth(col) );
        cumulativeTileWidth += m_pps.getTileColumnWidth(col);
      }
      m_tileParameters[row * numCols + getNumTileColumnsMinus1()].setTileWidthInCtus( getFrameWidthInCtus()-cumulativeTileWidth );
    }

    //set the height for each tile
    for(Int col=0; col < numCols; col++)
    {
      Int cumulativeTileHeight = 0;
      for(Int row=0; row < getNumTileRowsMinus1(); row++)
      {
        m_tileParameters[row * numCols + col].setTileHeightInCtus( m_pps.getTileRowHeight(row) );
        cumulativeTileHeight += m_pps.getTileRowHeight(row);
      }
      m_tileParameters[getNumTileRowsMinus1() * numCols + col].setTileHeightInCtus( getFrameHeightInCtus()-cumulativeTileHeight );
    }
  }

  // Tile size check
  Int minWidth  = 1;
  Int minHeight = 1;
  const Int profileIdc = m_sps.getPTL()->getGeneralPTL()->getProfileIdc();
  if (  profileIdc == Profile::MAIN || profileIdc == Profile::MAIN10) //TODO: add more profiles to the tile-size check...
  {
    if (m_pps.getTilesEnabledFlag())
    {
      minHeight = 64  / m_sps.getMaxCUHeight();
      minWidth  = 256 / m_sps.getMaxCUWidth();
    }
  }
  for(Int row=0; row < numRows; row++)
  {
    for(Int col=0; col < numCols; col++)
    {
      const Int tileIdx = row * numCols + col;
      assert (m_tileParameters[tileIdx].getTileWidthInCtus() >= minWidth);
      assert (m_tileParameters[tileIdx].getTileHeightInCtus() >= minHeight);
    }
  }

  //initialize each tile of the current picture
  for( Int row=0; row < numRows; row++ )
  {
    for( Int col=0; col < numCols; col++ )
    {
      const Int tileIdx = row * numCols + col;

      //initialize the RightEdgePosInCU for each tile
      Int rightEdgePosInCTU = 0;
      for( Int i=0; i <= col; i++ )
      {
        rightEdgePosInCTU += m_tileParameters[row * numCols + i].getTileWidthInCtus();
      }
      m_tileParameters[tileIdx].setRightEdgePosInCtus(rightEdgePosInCTU-1);

      //initialize the BottomEdgePosInCU for each tile
      Int bottomEdgePosInCTU = 0;
      for( Int i=0; i <= row; i++ )
      {
        bottomEdgePosInCTU += m_tileParameters[i * numCols + col].getTileHeightInCtus();
      }
      m_tileParameters[tileIdx].setBottomEdgePosInCtus(bottomEdgePosInCTU-1);

      //initialize the FirstCUAddr for each tile
      m_tileParameters[tileIdx].setFirstCtuRsAddr( (m_tileParameters[tileIdx].getBottomEdgePosInCtus() - m_tileParameters[tileIdx].getTileHeightInCtus() + 1) * getFrameWidthInCtus() +
                                                    m_tileParameters[tileIdx].getRightEdgePosInCtus()  - m_tileParameters[tileIdx].getTileWidthInCtus()  + 1);
    }
  }

  Int  columnIdx = 0;
  Int  rowIdx = 0;

  //initialize the TileIdxMap
  for( Int i=0; i<m_numCtusInFrame; i++)
  {
    for( Int col=0; col < numCols; col++)
    {
      if(i % getFrameWidthInCtus() <= m_tileParameters[col].getRightEdgePosInCtus())
      {
        columnIdx = col;
        break;
      }
    }
    for(Int row=0; row < numRows; row++)
    {
      if(i / getFrameWidthInCtus() <= m_tileParameters[row*numCols].getBottomEdgePosInCtus())
      {
        rowIdx = row;
        break;
      }
    }
    m_puiTileIdxMap[i] = rowIdx * numCols + columnIdx;
  }
}
UInt TComPicSym::xCalculateNextCtuRSAddr( UInt currCtuRsAddr )
{
  UInt  nextCtuRsAddr;

  //get the tile index for the current CTU
  const UInt uiTileIdx = getTileIdxMap(currCtuRsAddr);

  //get the raster scan address for the next CTU
  if( currCtuRsAddr % m_frameWidthInCtus == getTComTile(uiTileIdx)->getRightEdgePosInCtus() && currCtuRsAddr / m_frameWidthInCtus == getTComTile(uiTileIdx)->getBottomEdgePosInCtus() )
  //the current CTU is the last CTU of the tile
  {
    if(uiTileIdx+1 == getNumTiles())
    {
      nextCtuRsAddr = m_numCtusInFrame;
    }
    else
    {
      nextCtuRsAddr = getTComTile(uiTileIdx+1)->getFirstCtuRsAddr();
    }
  }
  else //the current CTU is not the last CTU of the tile
  {
    if( currCtuRsAddr % m_frameWidthInCtus == getTComTile(uiTileIdx)->getRightEdgePosInCtus() )  //the current CTU is on the rightmost edge of the tile
    {
      nextCtuRsAddr = currCtuRsAddr + m_frameWidthInCtus - getTComTile(uiTileIdx)->getTileWidthInCtus() + 1;
    }
    else
    {
      nextCtuRsAddr = currCtuRsAddr + 1;
    }
  }

  return nextCtuRsAddr;
}

Void TComPicSym::deriveLoopFilterBoundaryAvailibility(Int ctuRsAddr,
                                                      Bool& isLeftAvail,
                                                      Bool& isRightAvail,
                                                      Bool& isAboveAvail,
                                                      Bool& isBelowAvail,
                                                      Bool& isAboveLeftAvail,
                                                      Bool& isAboveRightAvail,
                                                      Bool& isBelowLeftAvail,
                                                      Bool& isBelowRightAvail
                                                      )
{

  isLeftAvail      = (ctuRsAddr % m_frameWidthInCtus != 0);
  isRightAvail     = (ctuRsAddr % m_frameWidthInCtus != m_frameWidthInCtus-1);
  isAboveAvail     = (ctuRsAddr >= m_frameWidthInCtus );
  isBelowAvail     = (ctuRsAddr <  m_numCtusInFrame - m_frameWidthInCtus);
  isAboveLeftAvail = (isAboveAvail && isLeftAvail);
  isAboveRightAvail= (isAboveAvail && isRightAvail);
  isBelowLeftAvail = (isBelowAvail && isLeftAvail);
  isBelowRightAvail= (isBelowAvail && isRightAvail);

  Bool isLoopFiltAcrossTilePPS = getCtu(ctuRsAddr)->getSlice()->getPPS()->getLoopFilterAcrossTilesEnabledFlag();

  {
    TComDataCU* ctuCurr  = getCtu(ctuRsAddr);
    TComDataCU* ctuLeft  = isLeftAvail ?getCtu(ctuRsAddr-1):NULL;
    TComDataCU* ctuRight = isRightAvail?getCtu(ctuRsAddr+1):NULL;
    TComDataCU* ctuAbove = isAboveAvail?getCtu(ctuRsAddr-m_frameWidthInCtus):NULL;
    TComDataCU* ctuBelow = isBelowAvail?getCtu(ctuRsAddr+m_frameWidthInCtus):NULL;
    TComDataCU* ctuAboveLeft  = isAboveLeftAvail ? getCtu(ctuRsAddr-m_frameWidthInCtus-1):NULL;
    TComDataCU* ctuAboveRight = isAboveRightAvail? getCtu(ctuRsAddr-m_frameWidthInCtus+1):NULL;
    TComDataCU* ctuBelowLeft  = isBelowLeftAvail ? getCtu(ctuRsAddr+m_frameWidthInCtus-1):NULL;
    TComDataCU* ctuBelowRight = isBelowRightAvail? getCtu(ctuRsAddr+m_frameWidthInCtus+1):NULL;

    {
      //left
      if(ctuLeft != NULL)
      {
        isLeftAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuLeft->getSlice()->getSliceCurStartCtuTsAddr())?ctuCurr->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }
      //above
      if(ctuAbove != NULL)
      {
        isAboveAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuAbove->getSlice()->getSliceCurStartCtuTsAddr())?ctuCurr->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }
      //right
      if(ctuRight != NULL)
      {
        isRightAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuRight->getSlice()->getSliceCurStartCtuTsAddr())?ctuRight->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }
      //below
      if(ctuBelow != NULL)
      {
        isBelowAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuBelow->getSlice()->getSliceCurStartCtuTsAddr())?ctuBelow->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }
      //above-left
      if(ctuAboveLeft != NULL)
      {
        isAboveLeftAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuAboveLeft->getSlice()->getSliceCurStartCtuTsAddr())?ctuCurr->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }
      //below-right
      if(ctuBelowRight != NULL)
      {
        isBelowRightAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuBelowRight->getSlice()->getSliceCurStartCtuTsAddr())?ctuBelowRight->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }

      //above-right
      if(ctuAboveRight != NULL)
      {
        Int curSliceStartTsAddr  = ctuCurr->getSlice()->getSliceCurStartCtuTsAddr();
        Int aboveRightSliceStartTsAddr = ctuAboveRight->getSlice()->getSliceCurStartCtuTsAddr();

        isAboveRightAvail = (curSliceStartTsAddr == aboveRightSliceStartTsAddr)?(true):
          (
          (curSliceStartTsAddr > aboveRightSliceStartTsAddr)?(ctuCurr->getSlice()->getLFCrossSliceBoundaryFlag())
          :(ctuAboveRight->getSlice()->getLFCrossSliceBoundaryFlag())
          );
      }
      //below-left
      if(ctuBelowLeft != NULL)
      {
        Int curSliceStartTsAddr       = ctuCurr->getSlice()->getSliceCurStartCtuTsAddr();
        Int belowLeftSliceStartTsAddr = ctuBelowLeft->getSlice()->getSliceCurStartCtuTsAddr();

        isBelowLeftAvail = (curSliceStartTsAddr == belowLeftSliceStartTsAddr)?(true):
          (
          (curSliceStartTsAddr > belowLeftSliceStartTsAddr)?(ctuCurr->getSlice()->getLFCrossSliceBoundaryFlag())
          :(ctuBelowLeft->getSlice()->getLFCrossSliceBoundaryFlag())
          );
      }
    }

    if(!isLoopFiltAcrossTilePPS)
    {
      isLeftAvail      = (!isLeftAvail      ) ?false:(getTileIdxMap( ctuLeft->getCtuRsAddr()         ) == getTileIdxMap( ctuRsAddr ));
      isAboveAvail     = (!isAboveAvail     ) ?false:(getTileIdxMap( ctuAbove->getCtuRsAddr()        ) == getTileIdxMap( ctuRsAddr ));
      isRightAvail     = (!isRightAvail     ) ?false:(getTileIdxMap( ctuRight->getCtuRsAddr()        ) == getTileIdxMap( ctuRsAddr ));
      isBelowAvail     = (!isBelowAvail     ) ?false:(getTileIdxMap( ctuBelow->getCtuRsAddr()        ) == getTileIdxMap( ctuRsAddr ));
      isAboveLeftAvail = (!isAboveLeftAvail ) ?false:(getTileIdxMap( ctuAboveLeft->getCtuRsAddr()    ) == getTileIdxMap( ctuRsAddr ));
      isAboveRightAvail= (!isAboveRightAvail) ?false:(getTileIdxMap( ctuAboveRight->getCtuRsAddr()   ) == getTileIdxMap( ctuRsAddr ));
      isBelowLeftAvail = (!isBelowLeftAvail ) ?false:(getTileIdxMap( ctuBelowLeft->getCtuRsAddr()    ) == getTileIdxMap( ctuRsAddr ));
      isBelowRightAvail= (!isBelowRightAvail) ?false:(getTileIdxMap( ctuBelowRight->getCtuRsAddr()   ) == getTileIdxMap( ctuRsAddr ));
    }
  }

}

#if VCEG_AZ07_FRUC_MERGE
Void TComPicSym::initFRUCMVP()
{
  const Int nBlkPosMask = getSPS().getMaxCUWidth() - 1;
  const Int nCUSizeLog2 = g_aucConvertToBit[getSPS().getMaxCUWidth()] + 2;
  const Int nWidthInNumSPU = 1 << ( nCUSizeLog2 - 2 );
  assert( MAX_CU_DEPTH == g_aucConvertToBit[MAX_CU_SIZE] + 2 && MIN_PU_SIZE == 4 );
  TComSlice * pCurSlice = getSlice( 0 );
  assert( !pCurSlice->isIntra() );
  // reset MV prediction
  for( UInt uiCUAddr = 0 ; uiCUAddr < getNumberOfCtusInFrame() ; uiCUAddr++ )
  {
    TComDataCU * pCurPicCU = getCtu( uiCUAddr );
    pCurPicCU->getFRUCUniLateralMVField( REF_PIC_LIST_0 )->clearMvField();
    pCurPicCU->getFRUCUniLateralMVField( REF_PIC_LIST_1 )->clearMvField();
  }

  // get MV from all reference
  Int nCurPOC = pCurSlice->getPOC();
  for( Int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
  {
    RefPicList eRefPicList = ( RefPicList )nRefPicList;
    for( Int nRefIdx = 0 ; nRefIdx < pCurSlice->getNumRefIdx( eRefPicList ) ; nRefIdx++ )
    {
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
      Int nOffset = 1 << ( VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE + 1 );
#endif
      Int nTargetRefIdx = 0;
      Int nTargetRefPOC = pCurSlice->getRefPOC( eRefPicList , nTargetRefIdx );
      Int nCurRefIdx = nRefIdx; 
      Int nCurRefPOC = pCurSlice->getRefPOC( eRefPicList , nCurRefIdx );
      Int nColPOC = pCurSlice->getRefPOC( eRefPicList , nRefIdx );
      TComPic * pColPic = pCurSlice->getRefPic( eRefPicList , nRefIdx );
      assert( getNumberOfCtusInFrame() == pColPic->getNumberOfCtusInFrame() );
      for( UInt uiColPicCUAddr = 0 ; uiColPicCUAddr < pColPic->getNumberOfCtusInFrame() ; uiColPicCUAddr++ )
      {
        TComDataCU * pColPicCU = pColPic->getCtu( uiColPicCUAddr );
        for( UInt uiAbsPartIdxColPicCU = 0 ; uiAbsPartIdxColPicCU < pColPic->getNumPartitionsInCtu() ; uiAbsPartIdxColPicCU++ )
        {
          Int xColPic = pColPicCU->getCUPelX() + g_auiRasterToPelX[g_auiZscanToRaster[uiAbsPartIdxColPicCU]];
          Int yColPic = pColPicCU->getCUPelY() + g_auiRasterToPelY[g_auiZscanToRaster[uiAbsPartIdxColPicCU]];
          TComCUMvField * pColPicCUMVField = pColPicCU->getCUMvField( eRefPicList );
          if( pColPicCUMVField->getRefIdx( uiAbsPartIdxColPicCU ) >= 0 )
          {
            TComSlice * pColSlice = pColPic->getSlice( 0 );
            Int nColRefPOC = pColSlice->getRefPOC( eRefPicList , pColPicCUMVField->getRefIdx( uiAbsPartIdxColPicCU ) );
            TComMv mvColPic = pColPicCUMVField->getMv( uiAbsPartIdxColPicCU );
            TComMv mv2CurRefPic = pColPicCU->scaleMV( mvColPic , nCurPOC , nCurRefPOC , nColPOC , nColRefPOC );
            // map the position into current picture, note that MV must be Qual-pel
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
            Int xCurPic = xColPic + ( MIN_PU_SIZE >> 1 ) - ( ( mv2CurRefPic.getHor() + nOffset ) >> (2+VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE) ) ; 
            Int yCurPic = yColPic + ( MIN_PU_SIZE >> 1 ) - ( ( mv2CurRefPic.getVer() + nOffset ) >> (2+VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE) ) ;
#else
            Int xCurPic = xColPic - ( ( mv2CurRefPic.getHor() + 2 ) >> 2 ) + ( MIN_PU_SIZE >> 1); 
            Int yCurPic = yColPic - ( ( mv2CurRefPic.getVer() + 2 ) >> 2 ) + ( MIN_PU_SIZE >> 1 );
#endif
            if( 0 <= xCurPic && xCurPic < pCurSlice->getSPS()->getPicWidthInLumaSamples()
              && 0 <= yCurPic && yCurPic < pCurSlice->getSPS()->getPicHeightInLumaSamples() )
            {
              UInt uiCurPicCUAddr = ( yCurPic >> nCUSizeLog2 ) * getFrameWidthInCtus() + ( xCurPic >> nCUSizeLog2 );
              assert( MIN_PU_SIZE == 4 );
              UInt uiAbsPartIdxCurPicCU = g_auiRasterToZscan[( ( yCurPic & nBlkPosMask ) >> 2 ) * nWidthInNumSPU + ( ( xCurPic & nBlkPosMask ) >> 2 )];
              TComCUMvField * pCurPicFRUCCUMVField = getCtu( uiCurPicCUAddr )->getFRUCUniLateralMVField( eRefPicList );
              if( pCurPicFRUCCUMVField->getRefIdx( uiAbsPartIdxCurPicCU ) < 0 )
              {
                // further scale to the target reference picture
                TComMv mv2TargetPic = nCurRefIdx == nTargetRefIdx ? mv2CurRefPic : pColPicCU->scaleMV( mvColPic , nCurPOC , nTargetRefPOC , nColPOC , nColRefPOC );
                pCurPicFRUCCUMVField->setMv( mv2TargetPic , uiAbsPartIdxCurPicCU );
                pCurPicFRUCCUMVField->setRefIdx( nTargetRefIdx , uiAbsPartIdxCurPicCU );
              }            
            }
          }
        }
      }
    }
  }
}
#endif

TComTile::TComTile()
: m_tileWidthInCtus     (0)
, m_tileHeightInCtus    (0)
, m_rightEdgePosInCtus  (0)
, m_bottomEdgePosInCtus (0)
, m_firstCtuRsAddr      (0)
{
}

TComTile::~TComTile()
{
}
//! \}