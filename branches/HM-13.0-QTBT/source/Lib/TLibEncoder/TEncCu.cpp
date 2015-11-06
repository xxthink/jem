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

/** \file     TEncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include <stdio.h>
#include "TEncTop.h"
#include "TEncCu.h"
#include "TEncAnalyze.h"

#include <cmath>
#include <algorithm>
using namespace std;

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

/**
 \param    uiTotalDepth  total number of allowable depth
 \param    uiMaxWidth    largest CU width
 \param    uiMaxHeight   largest CU height
 */
Void TEncCu::create(UChar uhTotalDepth, UInt uiMaxWidth, UInt uiMaxHeight)
{
  Int i;
  
  m_uhTotalDepth   = uhTotalDepth + 1;
#if !QT_BT_STRUCTURE
  m_ppcBestCU      = new TComDataCU*[m_uhTotalDepth-1];
  m_ppcTempCU      = new TComDataCU*[m_uhTotalDepth-1];
    
  m_ppcPredYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcResiYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcRecoYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcPredYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcResiYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcRecoYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcOrigYuv     = new TComYuv*[m_uhTotalDepth-1];
#endif
  
  UInt uiNumPartitions;
  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
    uiNumPartitions = 1<<( ( m_uhTotalDepth - i - 1 )<<1 );
#if !QT_BT_STRUCTURE
    UInt uiWidth  = uiMaxWidth  >> i;
    UInt uiHeight = uiMaxHeight >> i;
    
    m_ppcBestCU[i] = new TComDataCU; m_ppcBestCU[i]->create( uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );
    m_ppcTempCU[i] = new TComDataCU; m_ppcTempCU[i]->create( uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );
    
    m_ppcPredYuvBest[i] = new TComYuv; m_ppcPredYuvBest[i]->create(uiWidth, uiHeight);
    m_ppcResiYuvBest[i] = new TComYuv; m_ppcResiYuvBest[i]->create(uiWidth, uiHeight);
    m_ppcRecoYuvBest[i] = new TComYuv; m_ppcRecoYuvBest[i]->create(uiWidth, uiHeight);
    
    m_ppcPredYuvTemp[i] = new TComYuv; m_ppcPredYuvTemp[i]->create(uiWidth, uiHeight);
    m_ppcResiYuvTemp[i] = new TComYuv; m_ppcResiYuvTemp[i]->create(uiWidth, uiHeight);
    m_ppcRecoYuvTemp[i] = new TComYuv; m_ppcRecoYuvTemp[i]->create(uiWidth, uiHeight);
    
    m_ppcOrigYuv    [i] = new TComYuv; m_ppcOrigYuv    [i]->create(uiWidth, uiHeight);
#endif
  }
  
#if QT_BT_STRUCTURE
  UInt uiNumWidthIdx = g_aucConvertToBit[uiMaxWidth] + 1;
  UInt uiNumHeightIdx = g_aucConvertToBit[uiMaxHeight] + 1;

  m_ppcBestCUPU      = new TComDataCU**[uiNumWidthIdx];
  m_ppcTempCUPU      = new TComDataCU**[uiNumWidthIdx];

  m_ppcPredYuvBestPU = new TComYuv**[uiNumWidthIdx];
  m_ppcResiYuvBestPU = new TComYuv**[uiNumWidthIdx];
  m_ppcRecoYuvBestPU = new TComYuv**[uiNumWidthIdx];
  m_ppcPredYuvTempPU = new TComYuv**[uiNumWidthIdx];
  m_ppcResiYuvTempPU = new TComYuv**[uiNumWidthIdx];
  m_ppcRecoYuvTempPU = new TComYuv**[uiNumWidthIdx];
  m_ppcOrigYuvPU     = new TComYuv**[uiNumWidthIdx];

  for (UInt wIdx=0; wIdx<uiNumWidthIdx; wIdx++)
  {
    m_ppcBestCUPU[wIdx]      = new TComDataCU*[uiNumHeightIdx];
    m_ppcTempCUPU[wIdx]      = new TComDataCU*[uiNumHeightIdx];

    m_ppcPredYuvBestPU[wIdx] = new TComYuv*[uiNumHeightIdx];
    m_ppcResiYuvBestPU[wIdx] = new TComYuv*[uiNumHeightIdx];
    m_ppcRecoYuvBestPU[wIdx] = new TComYuv*[uiNumHeightIdx];
    m_ppcPredYuvTempPU[wIdx] = new TComYuv*[uiNumHeightIdx];
    m_ppcResiYuvTempPU[wIdx] = new TComYuv*[uiNumHeightIdx];
    m_ppcRecoYuvTempPU[wIdx] = new TComYuv*[uiNumHeightIdx];
    m_ppcOrigYuvPU[wIdx]     = new TComYuv*[uiNumHeightIdx];

    for (UInt hIdx=0; hIdx<uiNumHeightIdx; hIdx++)
    {
      uiNumPartitions = 1<<(wIdx+hIdx);
      UInt uiWidth = 1<<(wIdx+MIN_CU_LOG2);
      UInt uiHeight = 1<<(hIdx+MIN_CU_LOG2);

      m_ppcBestCUPU[wIdx][hIdx]      = new TComDataCU; m_ppcBestCUPU[wIdx][hIdx]->create( 1<<( ( m_uhTotalDepth - 1 )<<1 ), uiMaxWidth, uiMaxHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1), uiWidth, uiHeight );
      m_ppcTempCUPU[wIdx][hIdx]      = new TComDataCU; m_ppcTempCUPU[wIdx][hIdx]->create( 1<<( ( m_uhTotalDepth - 1 )<<1 ), uiMaxWidth, uiMaxHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1), uiWidth, uiHeight );

      m_ppcPredYuvBestPU[wIdx][hIdx] = new TComYuv; m_ppcPredYuvBestPU[wIdx][hIdx]->create(uiWidth, uiHeight);
      m_ppcResiYuvBestPU[wIdx][hIdx] = new TComYuv; m_ppcResiYuvBestPU[wIdx][hIdx]->create(uiWidth, uiHeight);
      m_ppcRecoYuvBestPU[wIdx][hIdx] = new TComYuv; m_ppcRecoYuvBestPU[wIdx][hIdx]->create(uiWidth, uiHeight);
      m_ppcPredYuvTempPU[wIdx][hIdx] = new TComYuv; m_ppcPredYuvTempPU[wIdx][hIdx]->create(uiWidth, uiHeight);
      m_ppcResiYuvTempPU[wIdx][hIdx] = new TComYuv; m_ppcResiYuvTempPU[wIdx][hIdx]->create(uiWidth, uiHeight);
      m_ppcRecoYuvTempPU[wIdx][hIdx] = new TComYuv; m_ppcRecoYuvTempPU[wIdx][hIdx]->create(uiWidth, uiHeight);
      m_ppcOrigYuvPU[wIdx][hIdx]     = new TComYuv; m_ppcOrigYuvPU[wIdx][hIdx]->create(uiWidth, uiHeight);
    }
  }
#endif
  m_bEncodeDQP = false;

  // initialize partition order.
  UInt* piTmp = &g_auiZscanToRaster[0];
  initZscanToRaster( m_uhTotalDepth, 1, 0, piTmp);
  initRasterToZscan( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );
  
  // initialize conversion matrix from partition index to pel
  initRasterToPelXY( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );
}

Void TEncCu::destroy()
{
  
#if QT_BT_STRUCTURE
  UInt uiMaxWidth = g_uiMaxCUWidth;
  UInt uiMaxHeight = g_uiMaxCUHeight;

  UInt uiNumWidthIdx = g_aucConvertToBit[uiMaxWidth] + 1;
  UInt uiNumHeightIdx = g_aucConvertToBit[uiMaxHeight] + 1;

  for (UInt wIdx=0; wIdx<uiNumWidthIdx; wIdx++)
  {
    for (UInt hIdx=0; hIdx<uiNumHeightIdx; hIdx++)
    {
      if (m_ppcBestCUPU[wIdx][hIdx])
      {
        m_ppcBestCUPU[wIdx][hIdx]->destroy(); delete m_ppcBestCUPU[wIdx][hIdx]; m_ppcBestCUPU[wIdx][hIdx] = NULL; 
      }
      if (m_ppcTempCUPU[wIdx][hIdx])
      {
        m_ppcTempCUPU[wIdx][hIdx]->destroy(); delete m_ppcTempCUPU[wIdx][hIdx]; m_ppcTempCUPU[wIdx][hIdx] = NULL; 
      }

      if(m_ppcPredYuvBestPU[wIdx][hIdx])
      {
        m_ppcPredYuvBestPU[wIdx][hIdx]->destroy(); delete m_ppcPredYuvBestPU[wIdx][hIdx]; m_ppcPredYuvBestPU[wIdx][hIdx] = NULL;
      }
      if(m_ppcResiYuvBestPU[wIdx][hIdx])
      {
        m_ppcResiYuvBestPU[wIdx][hIdx]->destroy(); delete m_ppcResiYuvBestPU[wIdx][hIdx]; m_ppcResiYuvBestPU[wIdx][hIdx] = NULL;
      }
      if(m_ppcRecoYuvBestPU[wIdx][hIdx])
      {
        m_ppcRecoYuvBestPU[wIdx][hIdx]->destroy(); delete m_ppcRecoYuvBestPU[wIdx][hIdx]; m_ppcRecoYuvBestPU[wIdx][hIdx] = NULL;
      }
      if(m_ppcPredYuvTempPU[wIdx][hIdx])
      {
        m_ppcPredYuvTempPU[wIdx][hIdx]->destroy(); delete m_ppcPredYuvTempPU[wIdx][hIdx]; m_ppcPredYuvTempPU[wIdx][hIdx] = NULL;
      }
      if(m_ppcResiYuvTempPU[wIdx][hIdx])
      {
        m_ppcResiYuvTempPU[wIdx][hIdx]->destroy(); delete m_ppcResiYuvTempPU[wIdx][hIdx]; m_ppcResiYuvTempPU[wIdx][hIdx] = NULL;
      }
      if(m_ppcRecoYuvTempPU[wIdx][hIdx])
      {
        m_ppcRecoYuvTempPU[wIdx][hIdx]->destroy(); delete m_ppcRecoYuvTempPU[wIdx][hIdx]; m_ppcRecoYuvTempPU[wIdx][hIdx] = NULL;
      }
      if(m_ppcOrigYuvPU[wIdx][hIdx])
      {
        m_ppcOrigYuvPU[wIdx][hIdx]->destroy();     delete m_ppcOrigYuvPU[wIdx][hIdx];     m_ppcOrigYuvPU[wIdx][hIdx] = NULL;
      }
    }
    if (m_ppcBestCUPU[wIdx])
    {
      delete [] m_ppcBestCUPU[wIdx]; m_ppcBestCUPU[wIdx] = NULL; 
    }
    if (m_ppcTempCUPU[wIdx])
    {
      delete [] m_ppcTempCUPU[wIdx]; m_ppcTempCUPU[wIdx] = NULL; 
    }

    if(m_ppcPredYuvBestPU[wIdx])
    {
      delete [] m_ppcPredYuvBestPU[wIdx]; m_ppcPredYuvBestPU[wIdx] = NULL;
    }
    if(m_ppcResiYuvBestPU[wIdx])
    {
      delete [] m_ppcResiYuvBestPU[wIdx]; m_ppcResiYuvBestPU[wIdx] = NULL;
    }
    if(m_ppcRecoYuvBestPU[wIdx])
    {
      delete [] m_ppcRecoYuvBestPU[wIdx]; m_ppcRecoYuvBestPU[wIdx] = NULL;
    }
    if(m_ppcPredYuvTempPU[wIdx])
    {
      delete [] m_ppcPredYuvTempPU[wIdx]; m_ppcPredYuvTempPU[wIdx] = NULL;
    }
    if(m_ppcResiYuvTempPU[wIdx])
    {
      delete [] m_ppcResiYuvTempPU[wIdx]; m_ppcResiYuvTempPU[wIdx] = NULL;
    }
    if(m_ppcRecoYuvTempPU[wIdx])
    {
      delete [] m_ppcRecoYuvTempPU[wIdx]; m_ppcRecoYuvTempPU[wIdx] = NULL;
    }
    if(m_ppcOrigYuvPU[wIdx])
    {
      delete [] m_ppcOrigYuvPU[wIdx];     m_ppcOrigYuvPU[wIdx] = NULL;
    }
  }
  if (m_ppcBestCUPU)
  {
    delete [] m_ppcBestCUPU; m_ppcBestCUPU = NULL; 
  }
  if (m_ppcTempCUPU)
  {
    delete [] m_ppcTempCUPU; m_ppcTempCUPU = NULL; 
  }

  if(m_ppcPredYuvBestPU)
  {
    delete [] m_ppcPredYuvBestPU; m_ppcPredYuvBestPU = NULL;
  }
  if(m_ppcResiYuvBestPU)
  {
    delete [] m_ppcResiYuvBestPU; m_ppcResiYuvBestPU = NULL;
  }
  if(m_ppcRecoYuvBestPU)
  {
    delete [] m_ppcRecoYuvBestPU; m_ppcRecoYuvBestPU = NULL;
  }
  if(m_ppcPredYuvTempPU)
  {
    delete [] m_ppcPredYuvTempPU; m_ppcPredYuvTempPU = NULL;
  }
  if(m_ppcResiYuvTempPU)
  {
    delete [] m_ppcResiYuvTempPU; m_ppcResiYuvTempPU = NULL;
  }
  if(m_ppcRecoYuvTempPU)
  {
    delete [] m_ppcRecoYuvTempPU; m_ppcRecoYuvTempPU = NULL;
  }
  if(m_ppcOrigYuvPU)
  {
    delete [] m_ppcOrigYuvPU;     m_ppcOrigYuvPU = NULL;
  }  
#else

  Int i;
  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
    if(m_ppcBestCU[i])
    {
      m_ppcBestCU[i]->destroy();      delete m_ppcBestCU[i];      m_ppcBestCU[i] = NULL;
    }
    if(m_ppcTempCU[i])
    {
      m_ppcTempCU[i]->destroy();      delete m_ppcTempCU[i];      m_ppcTempCU[i] = NULL;
    }
    if(m_ppcPredYuvBest[i])
    {
      m_ppcPredYuvBest[i]->destroy(); delete m_ppcPredYuvBest[i]; m_ppcPredYuvBest[i] = NULL;
    }
    if(m_ppcResiYuvBest[i])
    {
      m_ppcResiYuvBest[i]->destroy(); delete m_ppcResiYuvBest[i]; m_ppcResiYuvBest[i] = NULL;
    }
    if(m_ppcRecoYuvBest[i])
    {
      m_ppcRecoYuvBest[i]->destroy(); delete m_ppcRecoYuvBest[i]; m_ppcRecoYuvBest[i] = NULL;
    }
    if(m_ppcPredYuvTemp[i])
    {
      m_ppcPredYuvTemp[i]->destroy(); delete m_ppcPredYuvTemp[i]; m_ppcPredYuvTemp[i] = NULL;
    }
    if(m_ppcResiYuvTemp[i])
    {
      m_ppcResiYuvTemp[i]->destroy(); delete m_ppcResiYuvTemp[i]; m_ppcResiYuvTemp[i] = NULL;
    }
    if(m_ppcRecoYuvTemp[i])
    {
      m_ppcRecoYuvTemp[i]->destroy(); delete m_ppcRecoYuvTemp[i]; m_ppcRecoYuvTemp[i] = NULL;
    }
    if(m_ppcOrigYuv[i])
    {
      m_ppcOrigYuv[i]->destroy();     delete m_ppcOrigYuv[i];     m_ppcOrigYuv[i] = NULL;
    }
  }
  if(m_ppcBestCU)
  {
    delete [] m_ppcBestCU;
    m_ppcBestCU = NULL;
  }
  if(m_ppcTempCU)
  {
    delete [] m_ppcTempCU;
    m_ppcTempCU = NULL;
  }
  
  if(m_ppcPredYuvBest)
  {
    delete [] m_ppcPredYuvBest;
    m_ppcPredYuvBest = NULL;
  }
  if(m_ppcResiYuvBest)
  {
    delete [] m_ppcResiYuvBest;
    m_ppcResiYuvBest = NULL;
  }
  if(m_ppcRecoYuvBest)
  {
    delete [] m_ppcRecoYuvBest;
    m_ppcRecoYuvBest = NULL;
  }
  if(m_ppcPredYuvTemp)
  {
    delete [] m_ppcPredYuvTemp;
    m_ppcPredYuvTemp = NULL;
  }
  if(m_ppcResiYuvTemp)
  {
    delete [] m_ppcResiYuvTemp;
    m_ppcResiYuvTemp = NULL;
  }
  if(m_ppcRecoYuvTemp)
  {
    delete [] m_ppcRecoYuvTemp;
    m_ppcRecoYuvTemp = NULL;
  }
  if(m_ppcOrigYuv)
  {
    delete [] m_ppcOrigYuv;
    m_ppcOrigYuv = NULL;
  }
#endif
}

/** \param    pcEncTop      pointer of encoder class
 */
Void TEncCu::init( TEncTop* pcEncTop )
{
  m_pcEncCfg           = pcEncTop;
  m_pcPredSearch       = pcEncTop->getPredSearch();
  m_pcTrQuant          = pcEncTop->getTrQuant();
  m_pcBitCounter       = pcEncTop->getBitCounter();
  m_pcRdCost           = pcEncTop->getRdCost();
  
  m_pcEntropyCoder     = pcEncTop->getEntropyCoder();
  m_pcCavlcCoder       = pcEncTop->getCavlcCoder();
  m_pcSbacCoder       = pcEncTop->getSbacCoder();
  m_pcBinCABAC         = pcEncTop->getBinCABAC();
  
  m_pppcRDSbacCoder   = pcEncTop->getRDSbacCoder();
  m_pcRDGoOnSbacCoder = pcEncTop->getRDGoOnSbacCoder();
  
  m_pcRateCtrl        = pcEncTop->getRateCtrl();
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  rpcCU pointer of CU data class
 */
Void TEncCu::compressCU( TComDataCU*& rpcCU )
{
#if QT_BT_STRUCTURE
  rpcCU->getPic()->setCodedAreaInCTU(0);
  if (!rpcCU->getSlice()->isIntra())
  {
    rpcCU->getPic()->clearAllIntMv();
    rpcCU->getPic()->clearAllSkiped();
    rpcCU->getPic()->clearAllInter();
    rpcCU->getPic()->clearAllIntra();
  }

  UInt uiWidthIdx = g_aucConvertToBit[g_uiMaxCUWidth];
  UInt uiHeightIdx = g_aucConvertToBit[g_uiMaxCUHeight];

  m_ppcBestCUPU[uiWidthIdx][uiHeightIdx]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
  m_ppcTempCUPU[uiWidthIdx][uiHeightIdx]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
  m_ppcBestCUPU[uiWidthIdx][uiHeightIdx]->setTextType(TEXT_LUMA);
  m_ppcTempCUPU[uiWidthIdx][uiHeightIdx]->setTextType(TEXT_LUMA);
  xCompressCU(m_ppcBestCUPU[uiWidthIdx][uiHeightIdx], m_ppcTempCUPU[uiWidthIdx][uiHeightIdx], 0
    , m_ppcBestCUPU[uiWidthIdx][uiHeightIdx]->getWidth(0), m_ppcBestCUPU[uiWidthIdx][uiHeightIdx]->getHeight(0), 0); 
  
  if (rpcCU->getSlice()->isIntra())
  {
    rpcCU->getPic()->setCodedAreaInCTU(0);
    m_ppcBestCUPU[uiWidthIdx][uiHeightIdx]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
    m_ppcTempCUPU[uiWidthIdx][uiHeightIdx]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
    m_ppcBestCUPU[uiWidthIdx][uiHeightIdx]->setTextType(TEXT_CHROMA);
    m_ppcTempCUPU[uiWidthIdx][uiHeightIdx]->setTextType(TEXT_CHROMA);
    xCompressCU(m_ppcBestCUPU[uiWidthIdx][uiHeightIdx], m_ppcTempCUPU[uiWidthIdx][uiHeightIdx], 0
      , m_ppcBestCUPU[uiWidthIdx][uiHeightIdx]->getWidth(0), m_ppcBestCUPU[uiWidthIdx][uiHeightIdx]->getHeight(0), 0); 
  }
#else
  // initialize CU data
  m_ppcBestCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
  m_ppcTempCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );

  // analysis of CU
  xCompressCU( m_ppcBestCU[0], m_ppcTempCU[0], 0 );
#endif

#if ADAPTIVE_QP_SELECTION
  if( m_pcEncCfg->getUseAdaptQpSelect() )
  {
    if(rpcCU->getSlice()->getSliceType()!=I_SLICE) //IIII
    {
      xLcuCollectARLStats( rpcCU);
    }
  }
#endif
}
/** \param  pcCU  pointer of CU data class
 */
Void TEncCu::encodeCU ( TComDataCU* pcCU )
{
  if ( pcCU->getSlice()->getPPS()->getUseDQP() )
  {
    setdQPFlag(true);
  }

  // Encode CU data
#if QT_BT_STRUCTURE
  pcCU->getPic()->setCodedAreaInCTU(0);
  UInt uiLastIdx=0;
  UInt uiLastDepth=0;
  pcCU->setTextType(TEXT_LUMA); 
  xEncodeCU( pcCU, 0, 0, g_uiMaxCUWidth, g_uiMaxCUHeight, uiLastIdx, uiLastDepth ); 
  UInt uiDummy = 0;
  if (pcCU->getSlice()->isIntra())
  {
    pcCU->setTextType(TEXT_CHROMA);
    pcCU->getPic()->setCodedAreaInCTU(0);
    xEncodeCU( pcCU, 0, 0, g_uiMaxCUWidth, g_uiMaxCUHeight, uiDummy, uiDummy ); 
    pcCU->setTextType(TEXT_LUMA);
  }
  finishCU(pcCU,uiLastIdx,uiLastDepth); 
#else
  xEncodeCU( pcCU, 0, 0 );
#endif
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Derive small set of test modes for AMP encoder speed-up
 *\param   rpcBestCU
 *\param   eParentPartSize
 *\param   bTestAMP_Hor
 *\param   bTestAMP_Ver
 *\param   bTestMergeAMP_Hor
 *\param   bTestMergeAMP_Ver
 *\returns Void 
*/
#if !QT_BT_STRUCTURE
#if AMP_ENC_SPEEDUP
#if AMP_MRG
Void TEncCu::deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver)
#else
Void TEncCu::deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver)
#endif
{
  if ( rpcBestCU->getPartitionSize(0) == SIZE_2NxN )
  {
    bTestAMP_Hor = true;
  }
  else if ( rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
  {
    bTestAMP_Ver = true;
  }
  else if ( rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->getMergeFlag(0) == false && rpcBestCU->isSkipped(0) == false )
  {
    bTestAMP_Hor = true;          
    bTestAMP_Ver = true;          
  }

#if AMP_MRG
  //! Utilizing the partition size of parent PU    
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  { 
    bTestMergeAMP_Hor = true;
    bTestMergeAMP_Ver = true;
  }

  if ( eParentPartSize == SIZE_NONE ) //! if parent is intra
  {
    if ( rpcBestCU->getPartitionSize(0) == SIZE_2NxN )
    {
      bTestMergeAMP_Hor = true;
    }
    else if ( rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
    {
      bTestMergeAMP_Ver = true;
    }
  }

  if ( rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->isSkipped(0) == false )
  {
    bTestMergeAMP_Hor = true;          
    bTestMergeAMP_Ver = true;          
  }

  if ( rpcBestCU->getWidth(0) == 64 )
  { 
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }    
#else
  //! Utilizing the partition size of parent PU        
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  { 
    bTestAMP_Hor = true;
    bTestAMP_Ver = true;
  }

  if ( eParentPartSize == SIZE_2Nx2N )
  { 
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }      
#endif
}
#endif
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Compress a CU block recursively with enabling sub-LCU-level delta QP
 *\param   rpcBestCU
 *\param   rpcTempCU
 *\param   uiDepth
 *\returns Void
 *
 *- for loop of QP value to compress the current CU with all possible QP
*/
#if AMP_ENC_SPEEDUP
#if QT_BT_STRUCTURE
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, UInt uiWidth, UInt uiHeight, UInt uiBTSplitMode, UInt uiSplitConstrain )
#else
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize )
#endif
#else
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
#endif
{

#if BT_RMV_REDUNDANT
  rpcBestCU->setSplitConstrain( uiSplitConstrain );
  rpcTempCU->setSplitConstrain( uiSplitConstrain );

  Bool bQTreeValid = false;
#endif
  TComPic* pcPic = rpcBestCU->getPic();

#if QT_BT_STRUCTURE
  assert(uiWidth == rpcTempCU->getWidth(0) && uiHeight == rpcTempCU->getHeight(0));
  UInt uiWidthIdx = g_aucConvertToBit[rpcTempCU->getWidth(0)];
  UInt uiHeightIdx = g_aucConvertToBit[rpcTempCU->getHeight(0)];
  m_ppcOrigYuvPU[uiWidthIdx][uiHeightIdx]->copyFromPicYuv(pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU() );

  UInt uiPelXInCTU = rpcBestCU->getCUPelX() - rpcBestCU->getPic()->getCU(rpcBestCU->getAddr())->getCUPelX();
  UInt uiPelYInCTU = rpcBestCU->getCUPelY() - rpcBestCU->getPic()->getCU(rpcBestCU->getAddr())->getCUPelY();
  rpcBestCU->getPic()->setCodedBlkInCTU(false, uiPelXInCTU>> MIN_CU_LOG2, uiPelYInCTU>> MIN_CU_LOG2, uiWidth>> MIN_CU_LOG2, uiHeight>> MIN_CU_LOG2 );  
#else
  // get Original YUV data from picture
  m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU() );
#endif

  // variable for Early CU determination
  Bool    bSubBranch = true;

  // variable for Cbf fast mode PU decision
  Bool    doNotBlockPu = true;
  Bool earlyDetectionSkipMode = false;

  Bool bBoundary = false;
  UInt uiLPelX   = rpcBestCU->getCUPelX();
  UInt uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;
  UInt uiTPelY   = rpcBestCU->getCUPelY();
  UInt uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;

  Int iBaseQP = xComputeQP( rpcBestCU, uiDepth );
  Int iMinQP;
  Int iMaxQP;
  Bool isAddLowestQP = false;

  if( (g_uiMaxCUWidth>>uiDepth) >= rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP+idQP );
  }
  else
  {
    iMinQP = rpcTempCU->getQP(0);
    iMaxQP = rpcTempCU->getQP(0);
  }

  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }

  // transquant-bypass (TQB) processing loop variable initialisation ---

  const Int lowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.

  if ( (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) )
  {
    isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.
    iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs
    if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
    {
      iMaxQP = iMinQP;
    }
  }
#if NEIGHBOR_FAST
  UChar ucMinDepth , ucMaxDepth ;
  rpcTempCU->getMaxMinCUDepth( ucMinDepth , ucMaxDepth , rpcTempCU->getZorderIdxInCU() );
#endif
  // If slice start or slice end is within this cu...
  TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurStartCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart();
  Bool bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurEndCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart());
  Bool bInsidePicture = ( uiRPelX < rpcBestCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < rpcBestCU->getSlice()->getSPS()->getPicHeightInLumaSamples() );
  // We need to split, so don't try these modes.
#if NEIGHBOR_FAST
  assert(uiDepth <= ucMaxDepth);
  if(!bSliceEnd && !bSliceStart && bInsidePicture && ucMinDepth <= uiDepth )
#else
  if(!bSliceEnd && !bSliceStart && bInsidePicture )
#endif
  {
    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
    {
      const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);

      if (bIsLosslessMode)
      {
        iQP = lowestQP;
      }

      rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

      // do inter modes, SKIP and 2Nx2N
      if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
      {
#if QT_BT_STRUCTURE
        assert( rpcBestCU->getTextType()==TEXT_LUMA);
#endif
        // 2Nx2N
        if(m_pcEncCfg->getUseEarlySkipDetection())
        {
          xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N
        }
        // SKIP
        xCheckRDCostMerge2Nx2N( rpcBestCU, rpcTempCU, &earlyDetectionSkipMode );//by Merge for inter_2Nx2N        
        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

#if QT_BT_STRUCTURE
        if (!m_pcEncCfg->getUseEarlySkipDetection() && !rpcBestCU->getPic()->getSkiped(rpcBestCU->getZorderIdxInCU(), uiWidth, uiHeight)
          && !rpcBestCU->getPic()->getIntra(rpcBestCU->getZorderIdxInCU(), uiWidth, uiHeight)
          )
#else
        if(!m_pcEncCfg->getUseEarlySkipDetection())
#endif
        {
          // 2Nx2N, NxN
          xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          if(m_pcEncCfg->getUseCbfFastMode())
          {
            doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
          }
        }
      }

      if (bIsLosslessMode)
      {
        iQP = iMinQP;
      }
    }

    if(!earlyDetectionSkipMode)
    {
      for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
      {
        const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);

        if (bIsLosslessMode)
        {
          iQP = lowestQP;
        }
        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

#if !QT_BT_STRUCTURE
        // do inter modes, NxN, 2NxN, and Nx2N
        if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
        {
          // 2Nx2N, NxN
          if(!( (rpcBestCU->getWidth(0)==8) && (rpcBestCU->getHeight(0)==8) ))
          {
            if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && doNotBlockPu)
            {
              xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_NxN   );
              rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            }
          }

          // 2NxN, Nx2N
          if(doNotBlockPu)
          {
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_Nx2N  );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }
          if(doNotBlockPu)
          {
            xCheckRDCostInter      ( rpcBestCU, rpcTempCU, SIZE_2NxN  );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxN)
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }

#if 1
          //! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
          if( pcPic->getSlice(0)->getSPS()->getAMPAcc(uiDepth) )
          {
#if AMP_ENC_SPEEDUP        
            Bool bTestAMP_Hor = false, bTestAMP_Ver = false;

#if AMP_MRG
            Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;

            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);
#else
            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Hor )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Hor ) 
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU, true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD, true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N, true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N, true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
              }
            }
#endif

#else
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

#endif
          }    
#endif
        }

#endif
        // do normal intra modes
        // speedup for inter frames
#if QT_BT_STRUCTURE
        if( rpcBestCU->getSlice()->getSliceType() == I_SLICE || 
          ((rpcBestCU->getCbf( 0, TEXT_LUMA     ) != 0 || rpcBestCU->getCbf( 0, TEXT_CHROMA_U     ) != 0 || rpcBestCU->getCbf( 0, TEXT_CHROMA_V     ) != 0 )
          && !rpcBestCU->getPic()->getInter(rpcBestCU->getZorderIdxInCU(), uiWidth, uiHeight)          
          && rpcBestCU->getTextType()==TEXT_LUMA)  ||
          (rpcBestCU->getTextType()!=TEXT_LUMA)  ) // avoid very complex intra if it is unlikely
#else
        if( rpcBestCU->getSlice()->getSliceType() == I_SLICE || 
          rpcBestCU->getCbf( 0, TEXT_LUMA     ) != 0   ||
          rpcBestCU->getCbf( 0, TEXT_CHROMA_U ) != 0   ||
          rpcBestCU->getCbf( 0, TEXT_CHROMA_V ) != 0     ) // avoid very complex intra if it is unlikely
#endif
        {
#if PBINTRA_FAST
          rpcTempCU->getInterHAD() = MAX_UINT;
          if (rpcBestCU->getPredictionMode(0)==MODE_INTER && !rpcBestCU->getSlice()->isIntra())
          {
            m_pcPredSearch->motionCompensation ( rpcTempCU, m_ppcPredYuvTempPU[uiWidthIdx][uiHeightIdx] );
            // use hadamard transform here
            UInt uiSad = m_pcRdCost->calcHAD(g_bitDepthY, m_ppcOrigYuvPU[uiWidthIdx][uiHeightIdx]->getLumaAddr(), m_ppcOrigYuvPU[uiWidthIdx][uiHeightIdx]->getStride()
              , m_ppcPredYuvTempPU[uiWidthIdx][uiHeightIdx]->getLumaAddr(), m_ppcPredYuvTempPU[uiWidthIdx][uiHeightIdx]->getStride()
              , rpcTempCU->getWidth(0), rpcTempCU->getHeight(0) );
            rpcTempCU->getInterHAD() = uiSad;
          }
#endif
          xCheckRDCostIntra( rpcBestCU, rpcTempCU, SIZE_2Nx2N );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
#if !QT_BT_STRUCTURE
          if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
          {
            if( rpcTempCU->getWidth(0) > ( 1 << rpcTempCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() ) )
            {
              xCheckRDCostIntra( rpcBestCU, rpcTempCU, SIZE_NxN   );
              rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            }
          }
#endif
        }

        // test PCM
        if(pcPic->getSlice(0)->getSPS()->getUsePCM()
          && rpcTempCU->getWidth(0) <= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MaxSize())
          && rpcTempCU->getWidth(0) >= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MinSize()) )
        {
          UInt uiRawBits = (2 * g_bitDepthY + g_bitDepthC) * rpcBestCU->getWidth(0) * rpcBestCU->getHeight(0) / 2;
          UInt uiBestBits = rpcBestCU->getTotalBits();
          if((uiBestBits > uiRawBits) || (rpcBestCU->getTotalCost() > m_pcRdCost->calcRdCost(uiRawBits, 0)))
          {
            xCheckIntraPCM (rpcBestCU, rpcTempCU);
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          }
        }
        if (bIsLosslessMode)
        {
          iQP = iMinQP;
        }
      }
    }

#if QT_BT_STRUCTURE 
    if (!rpcBestCU->isIntra(0))
    {
      m_pcEntropyCoder->resetBits();
      if (uiBTSplitMode==0)
      {
        assert(g_uiMaxCUWidth>>uiDepth==uiWidth && uiWidth==uiHeight);
        m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );
      }

      UInt uiMaxBTD = rpcTempCU->getSlice()->isIntra() ? (rpcTempCU->getTextType()==TEXT_LUMA?MAX_BT_DEPTH:MAX_BT_DEPTH_C): MAX_BT_DEPTH_INTER;
      UInt uiMaxBTSize = rpcTempCU->getTextType()==TEXT_LUMA ? rpcTempCU->getSlice()->getMaxBTSize(): MAX_BT_SIZE_C;
      UInt uiMinBTSize = rpcTempCU->getSlice()->isIntra() ? (rpcTempCU->getTextType()==TEXT_LUMA?MIN_BT_SIZE:MIN_BT_SIZE_C): MIN_BT_SIZE_INTER;

      if (rpcBestCU->getWidth(0)<=uiMaxBTSize && rpcBestCU->getHeight(0)<=uiMaxBTSize 
        && (rpcBestCU->getWidth(0)>=2*uiMinBTSize || rpcBestCU->getHeight(0)>=2*uiMinBTSize) 
        && rpcBestCU->getBTDepth(0)<uiMaxBTD)
      {
        m_pcEntropyCoder->encodeBTSplitMode(rpcBestCU, 0, rpcBestCU->getWidth(0), rpcBestCU->getHeight(0), true);
      } 

      rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
      rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      rpcBestCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion() );
    }
#else
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );
    rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
    rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
    rpcBestCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion() );
#endif

    // Early CU determination
    if( m_pcEncCfg->getUseEarlyCU() && rpcBestCU->isSkipped(0) )
    {
      bSubBranch = false;
    }
#if NEIGHBOR_FAST
    else if( uiDepth >= ucMaxDepth )
    {
      bSubBranch = false;
    }
#endif
    else
    {
      bSubBranch = true;
    }
  }
  else if(!(bSliceEnd && bInsidePicture))
  {
    bBoundary = true;
  }

  // copy orginal YUV samples to PCM buffer
  if( rpcBestCU->isLosslessCoded(0) && (rpcBestCU->getIPCMFlag(0) == false))
  {
#if QT_BT_STRUCTURE
    xFillPCMBuffer(rpcBestCU, m_ppcOrigYuvPU[uiWidthIdx][uiHeightIdx]);
#else
    xFillPCMBuffer(rpcBestCU, m_ppcOrigYuv[uiDepth]);
#endif
  }
  if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP+idQP );
  }
  else if( (g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    iMinQP = iBaseQP;
    iMaxQP = iBaseQP;
  }
  else
  {
    Int iStartQP;
    if( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
    {
      iStartQP = rpcTempCU->getQP(0);
    }
    else
    {
      UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
      iStartQP = rpcTempCU->getQP(uiCurSliceStartPartIdx);
    }
    iMinQP = iStartQP;
    iMaxQP = iStartQP;
  }
  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }

  if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
  {
    iMaxQP = iMinQP; // If all blocks are forced into using transquant bypass, do not loop here.
  }

  for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
  {
    const Bool bIsLosslessMode = false; // False at this level. Next level down may set it to true.
    rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

#if BT_RMV_REDUNDANT
    bQTreeValid = false;
    if( uiBTSplitMode == 0 )
    {
      if (rpcTempCU->getSlice()->isIntra())
      {
        UInt uiMinQTSize = rpcBestCU->getSlice()->isIntra() ? (rpcBestCU->getTextType()==TEXT_LUMA?MIN_QT_SIZE:MIN_QT_SIZE_C): rpcBestCU->getSlice()->getMinQTSize();              
        bQTreeValid = ( uiWidth>=2*uiMinQTSize );
      }
      else
      {
        bQTreeValid = (uiWidth>=2*rpcBestCU->getSlice()->getMinQTSize() );
      }
    }

    Bool bBTHorRmvEnable = false;
    Bool bBTVerRmvEnable = false;
    if (rpcBestCU->getSlice()->getSliceType() != I_SLICE)
    {
      bBTHorRmvEnable = true;
      bBTVerRmvEnable = bQTreeValid;
    }
#endif

#if QT_BT_STRUCTURE
    Bool bQTSplit = bSubBranch && uiBTSplitMode==0;
    if (rpcTempCU->getSlice()->isIntra())
    {
      UInt uiMinQTSize = rpcBestCU->getSlice()->isIntra() ? (rpcBestCU->getTextType()==TEXT_LUMA?MIN_QT_SIZE:MIN_QT_SIZE_C): rpcBestCU->getSlice()->getMinQTSize();              
      bQTSplit = bQTSplit && ( uiWidth>=2*uiMinQTSize || bBoundary);
    }
    else
    {
      bQTSplit = bQTSplit && (uiWidth>=2*rpcBestCU->getSlice()->getMinQTSize() || bBoundary);
    }

    if (!bBoundary && rpcBestCU->isSkipped(0))
    {
      rpcBestCU->getPic()->setSkiped(rpcBestCU->getZorderIdxInCU(), uiWidth, uiHeight, true);
    }
    if (!bBoundary && rpcBestCU->getPredictionMode(0)==MODE_INTER)
    {
      rpcBestCU->getPic()->setInter(rpcBestCU->getZorderIdxInCU(), uiWidth, uiHeight, true);
    }
    else if (!bBoundary && rpcBestCU->getPredictionMode(0)==MODE_INTRA)
    {
      rpcBestCU->getPic()->setIntra(rpcBestCU->getZorderIdxInCU(), uiWidth, uiHeight, true);
    }

    UInt uiQTWidth = g_uiMaxCUWidth>>uiDepth;
    UInt uiQTHeight = g_uiMaxCUHeight>>uiDepth;
    UInt uiBTDepth = g_aucConvertToBit[uiQTWidth]-g_aucConvertToBit[uiWidth] + g_aucConvertToBit[uiQTHeight]-g_aucConvertToBit[uiHeight];

    UInt uiMaxBTD = rpcTempCU->getSlice()->isIntra() ? (rpcTempCU->getTextType()==TEXT_LUMA?MAX_BT_DEPTH:MAX_BT_DEPTH_C): MAX_BT_DEPTH_INTER;
    UInt uiMaxBTSize = rpcTempCU->getTextType()==TEXT_LUMA ? rpcTempCU->getSlice()->getMaxBTSize(): MAX_BT_SIZE_C;
    UInt uiMinBTSize = rpcTempCU->getSlice()->isIntra() ? (rpcTempCU->getTextType()==TEXT_LUMA?MIN_BT_SIZE:MIN_BT_SIZE_C): MIN_BT_SIZE_INTER;

    Bool bTestHorSplit = (!bBoundary && uiHeight>=2*uiMinBTSize 
      && uiWidth<=uiMaxBTSize && uiHeight<=uiMaxBTSize && uiBTDepth<uiMaxBTD
      );

    Bool bTestVerSplit = (!bBoundary && uiWidth>=2*uiMinBTSize 
      && uiWidth<=uiMaxBTSize && uiHeight<=uiMaxBTSize && uiBTDepth<uiMaxBTD
      );

    //for encoder speedup
    if (rpcBestCU->getSkipFlag(0) && (bTestHorSplit || bTestVerSplit) && uiBTDepth>=SKIP_DEPTH)
    { 
      bTestHorSplit = bTestVerSplit = bQTSplit = false;
    }

#if BT_RMV_REDUNDANT
    if( uiSplitConstrain == 1 )
    {
        bTestHorSplit = false;
    }
    if( uiSplitConstrain == 2 )
    {
        bTestVerSplit = false;
    }
#endif

    if (bTestHorSplit) 
    {
      UChar       uhNextDepth         = uiDepth;
      UInt uiPUWidthIdx = g_aucConvertToBit[uiWidth]; 
      UInt uiPUHeightIdx = g_aucConvertToBit[uiHeight>>1];
      TComDataCU* pcSubBestPartCU     = m_ppcBestCUPU[uiPUWidthIdx][uiPUHeightIdx];
      TComDataCU* pcSubTempPartCU     = m_ppcTempCUPU[uiPUWidthIdx][uiPUHeightIdx];
      rpcTempCU->setBTSplitModeSubParts(1, 0, uiWidth, uiHeight);
#if BT_RMV_REDUNDANT
      uiSplitConstrain = 0;
#endif

      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 2; uiPartUnitIdx++ )
      {
        pcSubBestPartCU->initSubPU( rpcTempCU, uiPartUnitIdx, uiDepth, uiWidth, uiHeight>>1, 1, iQP );           
        pcSubTempPartCU->initSubPU( rpcTempCU, uiPartUnitIdx, uiDepth, uiWidth, uiHeight>>1, 1, iQP );           // clear sub partition datas or init.

        Bool bInSlice = pcSubBestPartCU->getSCUAddr()+pcSubBestPartCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&pcSubBestPartCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();
        if(bInSlice && ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
        {
          if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
          {
            m_pppcRDSbacCoder[uiWidthIdx][uiHeightIdx-1][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiWidthIdx][uiHeightIdx][CI_CURR_BEST]);
          }
          else
          {
            m_pppcRDSbacCoder[uiWidthIdx][uiHeightIdx-1][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiWidthIdx][uiHeightIdx-1][CI_NEXT_BEST]);
          }
#if BT_RMV_REDUNDANT
          xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uiDepth, uiWidth, uiHeight>>1, pcSubBestPartCU->getBTSplitMode(0), uiSplitConstrain );
          
          if( uiPartUnitIdx == 0 && pcSubBestPartCU->getBTSplitModeForBTDepth(0, uiBTDepth+1) == 2 && bBTHorRmvEnable )
          {
              uiSplitConstrain = 2;
          }
#else
          if (uiBTDepth >= (uiMaxBTD >= uiTDDepth ? uiMaxBTD-uiTDDepth : 0))
          {
            compressCUTopDown(pcSubBestPartCU, pcSubTempPartCU, uiDepth, uiWidth, uiHeight>>1, pcSubBestPartCU->getBTSplitMode(0), true);
          }
          else
          {
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uiDepth, uiWidth, uiHeight>>1, pcSubBestPartCU->getBTSplitMode(0), SIZE_NONE );
          }
#endif

          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth, uiWidth, uiHeight>>1 );         // Keep best part data to current temporary data.
          xCopyYuv2Tmp( pcSubBestPartCU->getZorderIdxInCU()-rpcTempCU->getZorderIdxInCU(), uiWidth, uiHeight, 1 );
        }
        else if (bInSlice)  
        {
          assert(0);
        }
      }

      m_pcEntropyCoder->resetBits();
      if (g_uiMaxCUWidth>>uiDepth==uiWidth && uiWidth==uiHeight )
      {
        m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );
      }
      m_pcEntropyCoder->encodeBTSplitMode(rpcTempCU, 0, uiWidth, uiHeight, true);

      rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
      rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();

      rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

      m_pppcRDSbacCoder[uiWidthIdx][uiHeightIdx-1][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiWidthIdx][uiHeightIdx][CI_TEMP_BEST]);

      Bool isEndOfSlice        = rpcBestCU->getSlice()->getSliceMode()==FIXED_NUMBER_OF_BYTES
        && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceArgument()<<3);
      Bool isEndOfSliceSegment = rpcBestCU->getSlice()->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
        && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceSegmentArgument()<<3);
      if(isEndOfSlice||isEndOfSliceSegment)
      {
        rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost()+1;
      }
      xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth, uiWidth, uiHeight);
      rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode, uiWidth, uiHeight, uiBTSplitMode );

      rpcBestCU->getPic()->setCodedBlkInCTU(false, uiPelXInCTU>> MIN_CU_LOG2, uiPelYInCTU>> MIN_CU_LOG2, uiWidth>> MIN_CU_LOG2, uiHeight>> MIN_CU_LOG2 );  
      rpcBestCU->getPic()->addCodedAreaInCTU(-(Int)uiWidth*uiHeight);
    }

    //for encoder speedup
    if (bTestHorSplit && rpcBestCU->isSkipped(0) && rpcBestCU->getBTDepth(0)==uiBTDepth && uiBTDepth>=SKIPHORNOVERQT_DEPTH_TH)
    {
      bTestVerSplit = bQTSplit = false;
    }

    //vertical split
    if (bTestVerSplit) 
    {
      UChar       uhNextDepth         = uiDepth;
      UInt uiPUWidthIdx = g_aucConvertToBit[uiWidth>>1]; 
      UInt uiPUHeightIdx = g_aucConvertToBit[uiHeight];
      TComDataCU* pcSubBestPartCU     = m_ppcBestCUPU[uiPUWidthIdx][uiPUHeightIdx];
      TComDataCU* pcSubTempPartCU     = m_ppcTempCUPU[uiPUWidthIdx][uiPUHeightIdx];
      rpcTempCU->setBTSplitModeSubParts(2, 0, uiWidth, uiHeight);
#if BT_RMV_REDUNDANT
      uiSplitConstrain = 0;
#endif
      
      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 2; uiPartUnitIdx++ )
      {
        pcSubBestPartCU->initSubPU( rpcTempCU, uiPartUnitIdx, uiDepth, uiWidth>>1, uiHeight, 2, iQP );           
        pcSubTempPartCU->initSubPU( rpcTempCU, uiPartUnitIdx, uiDepth, uiWidth>>1, uiHeight, 2, iQP );           // clear sub partition datas or init.

        Bool bInSlice = pcSubBestPartCU->getSCUAddr()+pcSubBestPartCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&pcSubBestPartCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();
        if(bInSlice && ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
        {
          if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
          {
            m_pppcRDSbacCoder[uiWidthIdx-1][uiHeightIdx][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiWidthIdx][uiHeightIdx][CI_CURR_BEST]);
          }
          else
          {
            m_pppcRDSbacCoder[uiWidthIdx-1][uiHeightIdx][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiWidthIdx-1][uiHeightIdx][CI_NEXT_BEST]);
          }

#if BT_RMV_REDUNDANT
          xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uiDepth, uiWidth>>1, uiHeight, pcSubBestPartCU->getBTSplitMode(0), uiSplitConstrain );
          
          if( uiPartUnitIdx == 0 && pcSubBestPartCU->getBTSplitModeForBTDepth(0, uiBTDepth+1) == 1 && bBTVerRmvEnable )
          {
              uiSplitConstrain = 1;
          }
#else
          if (uiBTDepth >= (uiMaxBTD >= uiTDDepth ? uiMaxBTD-uiTDDepth : 0))
          {
            compressCUTopDown(pcSubBestPartCU, pcSubTempPartCU, uiDepth, uiWidth>>1, uiHeight, pcSubBestPartCU->getBTSplitMode(0), true);
          }
          else
          {
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uiDepth, uiWidth>>1, uiHeight, pcSubBestPartCU->getBTSplitMode(0), SIZE_NONE );
          }
#endif

          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth, uiWidth>>1, uiHeight );         // Keep best part data to current temporary data.
          xCopyYuv2Tmp( pcSubBestPartCU->getZorderIdxInCU()-rpcTempCU->getZorderIdxInCU(), uiWidth, uiHeight, 2 );
        }
        else if (bInSlice)  
        {
          assert(0);
        }
      }

      m_pcEntropyCoder->resetBits();
      if (g_uiMaxCUWidth>>uiDepth==uiWidth && uiWidth==uiHeight )
      {
        m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );
      }
      m_pcEntropyCoder->encodeBTSplitMode(rpcTempCU, 0, uiWidth, uiHeight, true);

      rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
      rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

      m_pppcRDSbacCoder[uiWidthIdx-1][uiHeightIdx][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiWidthIdx][uiHeightIdx][CI_TEMP_BEST]);

      Bool isEndOfSlice        = rpcBestCU->getSlice()->getSliceMode()==FIXED_NUMBER_OF_BYTES
        && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceArgument()<<3);
      Bool isEndOfSliceSegment = rpcBestCU->getSlice()->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
        && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceSegmentArgument()<<3);
      if(isEndOfSlice||isEndOfSliceSegment)
      {
        rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost()+1;
      }
      xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth, uiWidth, uiHeight);
      rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode, uiWidth, uiHeight, uiBTSplitMode );  
      rpcBestCU->getPic()->setCodedBlkInCTU(false, uiPelXInCTU>> MIN_CU_LOG2, uiPelYInCTU>> MIN_CU_LOG2, uiWidth>> MIN_CU_LOG2, uiHeight>> MIN_CU_LOG2 );
      rpcBestCU->getPic()->addCodedAreaInCTU(-(Int)uiWidth*uiHeight);
    }

    UInt uiZorderBR = g_auiRasterToZscan[((uiHeight>> MIN_CU_LOG2)-1) * (g_uiMaxCUWidth>> MIN_CU_LOG2) + (uiWidth>> MIN_CU_LOG2)-1];  //bottom-right part.

    if (bQTSplit && ((rpcBestCU->getBTDepth(0)==0 && uiMaxBTD>=(rpcBestCU->getSlice()->isIntra() ? 3: 2) )
      || (rpcBestCU->getBTDepth(0)==1 && rpcBestCU->getBTDepth(uiZorderBR)==1 && uiMaxBTD>=(rpcBestCU->getSlice()->isIntra()? 4: 3))) 
      && bTestHorSplit && bTestVerSplit ) 
    {
      bQTSplit = false;
    }
#endif

    // further split
#if QT_BT_STRUCTURE
    if( bQTSplit)
#else
    if( bSubBranch && uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth )
#endif
    {
      UChar       uhNextDepth         = uiDepth+1;
#if QT_BT_STRUCTURE
      UInt uiCUWidthIdx = g_aucConvertToBit[uiWidth>>1]; 
      UInt uiCUHeightIdx = g_aucConvertToBit[uiHeight>>1];
      TComDataCU* pcSubBestPartCU     = m_ppcBestCUPU[uiCUWidthIdx][uiCUHeightIdx];
      TComDataCU* pcSubTempPartCU     = m_ppcTempCUPU[uiCUWidthIdx][uiCUHeightIdx];
#else
      TComDataCU* pcSubBestPartCU     = m_ppcBestCU[uhNextDepth];
      TComDataCU* pcSubTempPartCU     = m_ppcTempCU[uhNextDepth];
#endif

      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
      {
        pcSubBestPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
        pcSubTempPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.

        Bool bInSlice = pcSubBestPartCU->getSCUAddr()+pcSubBestPartCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&pcSubBestPartCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();
        if(bInSlice && ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
        {
#if QT_BT_STRUCTURE
          if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
          {
            m_pppcRDSbacCoder[uiCUWidthIdx][uiCUHeightIdx][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiWidthIdx][uiHeightIdx][CI_CURR_BEST]);
          }
          else
          {
            m_pppcRDSbacCoder[uiCUWidthIdx][uiCUHeightIdx][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiCUWidthIdx][uiCUHeightIdx][CI_NEXT_BEST]);
          }
#else
          if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
          {
            m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
          }
          else
          {
            m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]);
          }
#endif

#if AMP_ENC_SPEEDUP
          if ( rpcBestCU->isIntra(0) )
          {
#if QT_BT_STRUCTURE
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, uiWidth>>1, uiHeight>>1, 0 );            
#else
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, SIZE_NONE );
#endif
          }
          else
          {
#if QT_BT_STRUCTURE
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, uiWidth>>1, uiHeight>>1, 0 );
#else
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, rpcBestCU->getPartitionSize(0) );
#endif
          }
#else
          xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
#endif

#if QT_BT_STRUCTURE
          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth, uiWidth>>1, uiHeight>>1 );         // Keep best part data to current temporary data.
          assert(pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx == pcSubBestPartCU->getZorderIdxInCU()-rpcTempCU->getZorderIdxInCU());       
          xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uiWidth, uiHeight );
#else
          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.
          xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );
#endif
        }
        else if (bInSlice)
        {
#if QT_BT_STRUCTURE
          pcSubBestPartCU->copyToPic( uhNextDepth, uiWidth>>1, uiHeight>>1 );
          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth, uiWidth>>1, uiHeight>>1 );

          rpcBestCU->getPic()->addCodedAreaInCTU(uiWidth*uiHeight>>2);
#else
          pcSubBestPartCU->copyToPic( uhNextDepth );
          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );
#endif
        }
      }

      if( !bBoundary )
      {
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );

        rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
        rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      }
      rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

      if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() && rpcTempCU->getSlice()->getPPS()->getUseDQP())
      {
        Bool hasResidual = false;
        for( UInt uiBlkIdx = 0; uiBlkIdx < rpcTempCU->getTotalNumPart(); uiBlkIdx ++)
        {
          if( ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(uiBlkIdx+rpcTempCU->getZorderIdxInCU()) == rpcTempCU->getSlice()->getSliceSegmentCurStartCUAddr() ) && 
              ( rpcTempCU->getCbf( uiBlkIdx, TEXT_LUMA ) || rpcTempCU->getCbf( uiBlkIdx, TEXT_CHROMA_U ) || rpcTempCU->getCbf( uiBlkIdx, TEXT_CHROMA_V ) ) )
          {
            hasResidual = true;
            break;
          }
        }

        UInt uiTargetPartIdx;
        if ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) != pcSlice->getSliceSegmentCurStartCUAddr() )
        {
          uiTargetPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
        }
        else
        {
          uiTargetPartIdx = 0;
        }
        if ( hasResidual )
        {
#if !RDO_WITHOUT_DQP_BITS
          m_pcEntropyCoder->resetBits();
          m_pcEntropyCoder->encodeQP( rpcTempCU, uiTargetPartIdx, false );
          rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
          rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
          rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
#endif

          Bool foundNonZeroCbf = false;
          rpcTempCU->setQPSubCUs( rpcTempCU->getRefQP( uiTargetPartIdx ), rpcTempCU, 0, uiDepth, foundNonZeroCbf );
          assert( foundNonZeroCbf );
        }
        else
        {
          rpcTempCU->setQPSubParts( rpcTempCU->getRefQP( uiTargetPartIdx ), 0, uiDepth ); // set QP to default QP
        }
      }

#if QT_BT_STRUCTURE
      m_pppcRDSbacCoder[uiCUWidthIdx][uiCUHeightIdx][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiWidthIdx][uiHeightIdx][CI_TEMP_BEST]);
#else
      m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
#endif

      Bool isEndOfSlice        = rpcBestCU->getSlice()->getSliceMode()==FIXED_NUMBER_OF_BYTES
                                 && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceArgument()<<3);
      Bool isEndOfSliceSegment = rpcBestCU->getSlice()->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
                                 && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceSegmentArgument()<<3);
      if(isEndOfSlice||isEndOfSliceSegment)
      {
        rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost()+1;
      }
#if QT_BT_STRUCTURE
      xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth, uiWidth, uiHeight);

      rpcBestCU->getPic()->setCodedBlkInCTU(false, uiPelXInCTU>>MIN_CU_LOG2, uiPelYInCTU>>MIN_CU_LOG2, uiWidth>>MIN_CU_LOG2, uiHeight>>MIN_CU_LOG2 );  
      rpcBestCU->getPic()->addCodedAreaInCTU(-(Int)uiWidth*uiHeight);
#else
      xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth);                                  // RD compare current larger prediction
#endif
    }                                                                                  // with sub partitioned prediction.
  }

#if QT_BT_STRUCTURE  
  rpcBestCU->copyToPic(uiDepth, uiWidth, uiHeight); 
  if (rpcBestCU->getTextType()==TEXT_LUMA)
  {
    xCopyYuv2Pic( rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, uiWidth, uiHeight, rpcBestCU, uiLPelX, uiTPelY );   // Copy Yuv data to picture Yuv
  }
  else 
  {
    xCopyYuv2PicSep( rpcBestCU->getTextType(), rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, uiWidth, uiHeight, rpcBestCU, uiLPelX, uiTPelY );   // Copy Yuv data to picture Yuv
  }
#else
  rpcBestCU->copyToPic(uiDepth);                                                     // Copy Best data to Picture for next partition prediction.

  xCopyYuv2Pic( rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, rpcBestCU, uiLPelX, uiTPelY );   // Copy Yuv data to picture Yuv
#endif
#if QT_BT_STRUCTURE
  rpcBestCU->getPic()->setCodedBlkInCTU(true, uiPelXInCTU>>MIN_CU_LOG2, uiPelYInCTU>>MIN_CU_LOG2, uiWidth>>MIN_CU_LOG2, uiHeight>>MIN_CU_LOG2 );  
  rpcBestCU->getPic()->addCodedAreaInCTU(uiWidth*uiHeight);
#endif
  if( bBoundary ||(bSliceEnd && bInsidePicture))
  {
    return;
  }

#if !QT_BT_STRUCTURE
  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  assert( rpcBestCU->getPartitionSize ( 0 ) != SIZE_NONE  );
  assert( rpcBestCU->getPredictionMode( 0 ) != MODE_NONE  );
  assert( rpcBestCU->getTotalCost     (   ) != MAX_DOUBLE );
#endif
}

/** finish encoding a cu and handle end-of-slice conditions
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth 
 * \returns Void
 */
Void TEncCu::finishCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  TComPic* pcPic = pcCU->getPic();
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());

  //Calculate end address
  UInt uiCUAddr = pcCU->getSCUAddr()+uiAbsPartIdx;

  UInt uiInternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) % pcPic->getNumPartInCU();
  UInt uiExternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) / pcPic->getNumPartInCU();
  UInt uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
  UInt uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
  UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
  UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
  while(uiPosX>=uiWidth||uiPosY>=uiHeight)
  {
    uiInternalAddress--;
    uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
    uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
  }
  uiInternalAddress++;
  if(uiInternalAddress==pcCU->getPic()->getNumPartInCU())
  {
    uiInternalAddress = 0;
    uiExternalAddress = pcPic->getPicSym()->getCUOrderMap(pcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
  }
  UInt uiRealEndAddress = pcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*pcPic->getNumPartInCU()+uiInternalAddress);

  // Encode slice finish
  Bool bTerminateSlice = false;
  if (uiCUAddr+(pcCU->getPic()->getNumPartInCU()>>(uiDepth<<1)) == uiRealEndAddress)
  {
    bTerminateSlice = true;
  }
  UInt uiGranularityWidth = g_uiMaxCUWidth;
  uiPosX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  uiPosY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  Bool granularityBoundary=((uiPosX+pcCU->getWidth(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosX+pcCU->getWidth(uiAbsPartIdx)==uiWidth))
    &&((uiPosY+pcCU->getHeight(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosY+pcCU->getHeight(uiAbsPartIdx)==uiHeight));
  
  if(granularityBoundary)
  {
    // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
    if (!bTerminateSlice)
      m_pcEntropyCoder->encodeTerminatingBit( bTerminateSlice ? 1 : 0 );
  }
  
  Int numberOfWrittenBits = 0;
  if (m_pcBitCounter)
  {
    numberOfWrittenBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  
  // Calculate slice end IF this CU puts us over slice bit size.
  UInt iGranularitySize = pcCU->getPic()->getNumPartInCU();
  Int iGranularityEnd = ((pcCU->getSCUAddr()+uiAbsPartIdx)/iGranularitySize)*iGranularitySize;
  if(iGranularityEnd<=pcSlice->getSliceSegmentCurStartCUAddr()) 
  {
    iGranularityEnd+=max(iGranularitySize,(pcCU->getPic()->getNumPartInCU()>>(uiDepth<<1)));
  }
  // Set slice end parameter
  if(pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES&&!pcSlice->getFinalized()&&pcSlice->getSliceBits()+numberOfWrittenBits>pcSlice->getSliceArgument()<<3) 
  {
    pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
    pcSlice->setSliceCurEndCUAddr(iGranularityEnd);
    return;
  }
  // Set dependent slice end parameter
  if(pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES&&!pcSlice->getFinalized()&&pcSlice->getSliceSegmentBits()+numberOfWrittenBits > pcSlice->getSliceSegmentArgument()<<3) 
  {
    pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
    return;
  }
  if(granularityBoundary)
  {
    pcSlice->setSliceBits( (UInt)(pcSlice->getSliceBits() + numberOfWrittenBits) );
    pcSlice->setSliceSegmentBits(pcSlice->getSliceSegmentBits()+numberOfWrittenBits);
    if (m_pcBitCounter)
    {
      m_pcEntropyCoder->resetBits();      
    }
  }
}

/** Compute QP for each CU
 * \param pcCU Target CU
 * \param uiDepth CU depth
 * \returns quantization parameter
 */
Int TEncCu::xComputeQP( TComDataCU* pcCU, UInt uiDepth )
{
  Int iBaseQp = pcCU->getSlice()->getSliceQp();
  Int iQpOffset = 0;
  if ( m_pcEncCfg->getUseAdaptiveQP() )
  {
    TEncPic* pcEPic = dynamic_cast<TEncPic*>( pcCU->getPic() );
    UInt uiAQDepth = min( uiDepth, pcEPic->getMaxAQDepth()-1 );
    TEncPicQPAdaptationLayer* pcAQLayer = pcEPic->getAQLayer( uiAQDepth );
    UInt uiAQUPosX = pcCU->getCUPelX() / pcAQLayer->getAQPartWidth();
    UInt uiAQUPosY = pcCU->getCUPelY() / pcAQLayer->getAQPartHeight();
    UInt uiAQUStride = pcAQLayer->getAQPartStride();
    TEncQPAdaptationUnit* acAQU = pcAQLayer->getQPAdaptationUnit();

    Double dMaxQScale = pow(2.0, m_pcEncCfg->getQPAdaptationRange()/6.0);
    Double dAvgAct = pcAQLayer->getAvgActivity();
    Double dCUAct = acAQU[uiAQUPosY * uiAQUStride + uiAQUPosX].getActivity();
    Double dNormAct = (dMaxQScale*dCUAct + dAvgAct) / (dCUAct + dMaxQScale*dAvgAct);
    Double dQpOffset = log(dNormAct) / log(2.0) * 6.0;
    iQpOffset = Int(floor( dQpOffset + 0.49999 ));
  }
  return Clip3(-pcCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQp+iQpOffset );
}

/** encode a CU block recursively
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth 
 * \returns Void
 */
#if QT_BT_STRUCTURE
Void TEncCu::xEncodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight, UInt& ruiLastIdx, UInt& ruiLastDepth, UInt uiSplitConstrain )
#else
Void TEncCu::xEncodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
#endif
{
#if BT_RMV_REDUNDANT
  pcCU->setSplitConstrain( uiSplitConstrain );
  Bool bQTreeValid = false;
#endif
  TComPic* pcPic = pcCU->getPic();
  
  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
  
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  // If slice start is within this cu...
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
    pcSlice->getSliceSegmentCurStartCUAddr() < pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcPic->getNumPartInCU() >> (uiDepth<<1) );
  // We need to split, so don't try these modes.
#if QT_BT_STRUCTURE
  if ((g_uiMaxCUWidth>>uiDepth)==uiWidth && (g_uiMaxCUHeight>>uiDepth)==uiHeight)
  {
    assert(uiWidth==uiHeight);
#endif
    if(!bSliceStart&&( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
    {
      m_pcEntropyCoder->encodeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
    }
    else
    {
      bBoundary = true;
    }
  
#if BT_RMV_REDUNDANT
    bQTreeValid = true;
    UInt uiMinQTSize = pcCU->getSlice()->isIntra() ? (pcCU->getTextType()==TEXT_LUMA?MIN_QT_SIZE:MIN_QT_SIZE_C): pcCU->getSlice()->getMinQTSize();
    if ((g_uiMaxCUWidth>>uiDepth) <= uiMinQTSize)
    {
      bQTreeValid = false;
    }
#endif

#if QT_BT_STRUCTURE
    if( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) || bBoundary )
#else
    if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || bBoundary )
#endif
    {
      UInt uiQNumParts = ( pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
      if( (g_uiMaxCUWidth>>uiDepth) == pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
      {
        setdQPFlag(true);
      }
      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
      {
        uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
        uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
        Bool bInSlice = pcCU->getSCUAddr()+uiAbsPartIdx+uiQNumParts>pcSlice->getSliceSegmentCurStartCUAddr()&&pcCU->getSCUAddr()+uiAbsPartIdx<pcSlice->getSliceSegmentCurEndCUAddr();
        if(bInSlice&&( uiLPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
        {
#if QT_BT_STRUCTURE
          xEncodeCU( pcCU, uiAbsPartIdx, uiDepth+1, uiWidth>>1, uiHeight>>1, ruiLastIdx, ruiLastDepth );
#else
          xEncodeCU( pcCU, uiAbsPartIdx, uiDepth+1 );
#endif
        }
#if QT_BT_STRUCTURE
        else
        {
          pcCU->getPic()->addCodedAreaInCTU(uiWidth*uiHeight>>2);
        }
#endif
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
    }
    if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag( pcCU, uiAbsPartIdx );
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
#if BT_RMV_REDUNDANT
    uiSplitConstrain = 0;
#endif
    m_pcEntropyCoder->encodeBTSplitMode(pcCU, uiAbsPartIdx, uiWidth, uiHeight);
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
        xEncodeCU( pcCU, uiAbsPartIdx, uiDepth, uiWidth, uiHeight>>1, ruiLastIdx, ruiLastDepth, uiSplitConstrain );
        if (pcCU->getBTSplitModeForBTDepth(uiAbsPartIdx, uiBTDepth+1) == 2 && bBTHorRmvEnable && uiPartUnitIdx==0)
        {
          uiSplitConstrain = 2;
        }
#else
        xEncodeCU( pcCU, uiAbsPartIdx, uiDepth, uiWidth, uiHeight>>1, ruiLastIdx, ruiLastDepth );
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
        xEncodeCU( pcCU, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight, ruiLastIdx, ruiLastDepth, uiSplitConstrain );
        if (pcCU->getBTSplitModeForBTDepth(uiAbsPartIdx, uiBTDepth+1) == 1 && bBTVerRmvEnable && uiPartUnitIdx==0)
        {
          uiSplitConstrain = 1;
        }
#else
        xEncodeCU( pcCU, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight, ruiLastIdx, ruiLastDepth );
#endif
      }
      return;
    }
  }  
  pcCU->getPic()->addCodedAreaInCTU(uiWidth*uiHeight);
#endif

#if AMAX_BT
  if (!pcCU->getSlice()->isIntra())
  {
    g_uiBlkSize[pcCU->getSlice()->getDepth()] += uiWidth*uiHeight;
    g_uiNumBlk[pcCU->getSlice()->getDepth()]++;
  }
#endif

#if QT_BT_STRUCTURE
  if (pcCU->getTextType()==TEXT_LUMA)
  {
#endif
    if( !pcCU->getSlice()->isIntra() )
    {
      m_pcEntropyCoder->encodeSkipFlag( pcCU, uiAbsPartIdx );
    }

    if( pcCU->isSkipped( uiAbsPartIdx ) )
    {
      m_pcEntropyCoder->encodeMergeIndex( pcCU, uiAbsPartIdx );
#if !QT_BT_STRUCTURE
      finishCU(pcCU,uiAbsPartIdx,uiDepth);
#endif
      return;
    }
    m_pcEntropyCoder->encodePredMode( pcCU, uiAbsPartIdx );
#if !QT_BT_STRUCTURE
    m_pcEntropyCoder->encodePartSize( pcCU, uiAbsPartIdx, uiDepth );
#endif
#if QT_BT_STRUCTURE
  }
  else
  {
    assert(pcCU->getPredictionMode(uiAbsPartIdx)==MODE_INTRA);
  }
#endif

#if QT_BT_STRUCTURE
  if (pcCU->isIntra( uiAbsPartIdx ))
#else
  if (pcCU->isIntra( uiAbsPartIdx ) && pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )
#endif
  {
    m_pcEntropyCoder->encodeIPCMInfo( pcCU, uiAbsPartIdx );

    if(pcCU->getIPCMFlag(uiAbsPartIdx))
    {
      // Encode slice finish
#if !QT_BT_STRUCTURE
      finishCU(pcCU,uiAbsPartIdx,uiDepth);
#endif
      return;
    }
  }

  // prediction Info ( Intra : direction mode, Inter : Mv, reference idx )
  m_pcEntropyCoder->encodePredInfo( pcCU, uiAbsPartIdx );
  
  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  m_pcEntropyCoder->encodeCoeff( pcCU, uiAbsPartIdx, uiDepth, pcCU->getWidth (uiAbsPartIdx), pcCU->getHeight(uiAbsPartIdx), bCodeDQP );
  setdQPFlag( bCodeDQP );

#if !QT_BT_STRUCTURE
  // --- write terminating bit ---
  finishCU(pcCU,uiAbsPartIdx,uiDepth);
#endif
}

Int xCalcHADs8x8_ISlice(Pel *piOrg, Int iStrideOrg) 
{
  Int k, i, j, jj;
  Int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = piOrg[0] ;
    diff[k+1] = piOrg[1] ;
    diff[k+2] = piOrg[2] ;
    diff[k+3] = piOrg[3] ;
    diff[k+4] = piOrg[4] ;
    diff[k+5] = piOrg[5] ;
    diff[k+6] = piOrg[6] ;
    diff[k+7] = piOrg[7] ;
 
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
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad =(iSumHad+2)>>2;
  return(iSumHad);
}

Int  TEncCu::updateLCUDataISlice(TComDataCU* pcCU, Int LCUIdx, Int width, Int height)
{
  Int  xBl, yBl; 
  const Int iBlkSize = 8;

  Pel* pOrgInit   = pcCU->getPic()->getPicYuvOrg()->getLumaAddr(pcCU->getAddr(), 0);
  Int  iStrideOrig = pcCU->getPic()->getPicYuvOrg()->getStride();
  Pel  *pOrg;

  Int iSumHad = 0;
  for ( yBl=0; (yBl+iBlkSize)<=height; yBl+= iBlkSize)
  {
    for ( xBl=0; (xBl+iBlkSize)<=width; xBl+= iBlkSize)
    {
      pOrg = pOrgInit + iStrideOrig*yBl + xBl; 
      iSumHad += xCalcHADs8x8_ISlice(pOrg, iStrideOrig);
    }
  }
  return(iSumHad);
}

/** check RD costs for a CU block encoded with merge
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckRDCostMerge2Nx2N( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Bool *earlyDetectionSkipMode )
{
  assert( rpcTempCU->getSlice()->getSliceType() != I_SLICE );
  TComMvField  cMvFieldNeighbours[ 2 * MRG_MAX_NUM_CANDS ]; // double length for mv of both lists
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
  Int numValidMergeCand = 0;
  const Bool bTransquantBypassFlag = rpcTempCU->getCUTransquantBypass(0);
#if QT_BT_STRUCTURE
  UInt uiWIdx = g_aucConvertToBit[rpcBestCU->getWidth(0)];
  UInt uiHIdx = g_aucConvertToBit[rpcBestCU->getHeight(0)];
#endif

  for( UInt ui = 0; ui < rpcTempCU->getSlice()->getMaxNumMergeCand(); ++ui )
  {
    uhInterDirNeighbours[ui] = 0;
  }
  UChar uhDepth = rpcTempCU->getDepth( 0 );
#if !QT_BT_STRUCTURE
  rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
#endif
  rpcTempCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );
  
  Int mergeCandBuffer[MRG_MAX_NUM_CANDS];
  for( UInt ui = 0; ui < numValidMergeCand; ++ui )
  {
    mergeCandBuffer[ui] = 0;
  }

  Bool bestIsSkip = false;
#if MRG_FAST
  bestIsSkip = rpcBestCU->getPic()->getSkiped(rpcBestCU->getZorderIdxInCU(), rpcBestCU->getWidth(0), rpcBestCU->getHeight(0));
#endif

  UInt iteration;
  if ( rpcTempCU->isLosslessCoded(0))
  {
    iteration = 1;
  }
  else 
  {
    iteration = 2;
  }

#if MRG_FAST
  UInt uiNumMrgSATDCand = NUM_MRG_SATD_CAND;
  UInt uiRdModeList[NUM_MRG_SATD_CAND];
  Double CandCostList[NUM_MRG_SATD_CAND];
  for (UInt i=0; i<NUM_MRG_SATD_CAND; i++)
  {
    uiRdModeList[i] = i;
    CandCostList[i] = MAX_DOUBLE;
  }

  if (!bestIsSkip)
  {
    for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
    {
      // set MC parameters
      rpcTempCU->setPredModeSubParts( MODE_INTER, 0 ); // interprets depth relative to LCU level
      rpcTempCU->setCUTransquantBypassSubParts( bTransquantBypassFlag,     0, uhDepth );
      rpcTempCU->setMergeFlagSubParts( true, 0 ); // interprets depth relative to LCU level
      rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0 ); // interprets depth relative to LCU level
      rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0); // interprets depth relative to LCU level
      rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
      rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level

      m_pcPredSearch->motionCompensation ( rpcTempCU, m_ppcPredYuvTempPU[uiWIdx][uiHIdx] );
      // use hadamard transform here
      UInt uiSad = m_pcRdCost->calcHAD(g_bitDepthY, m_ppcOrigYuvPU[uiWIdx][uiHIdx]->getLumaAddr(), m_ppcOrigYuvPU[uiWIdx][uiHIdx]->getStride()
        , m_ppcPredYuvTempPU[uiWIdx][uiHIdx]->getLumaAddr(), m_ppcPredYuvTempPU[uiWIdx][uiHIdx]->getStride()
        , rpcTempCU->getWidth(0), rpcTempCU->getHeight(0) );

      UInt uiBitsCand = uiMergeCand + 1;                                         
      if (uiMergeCand == rpcTempCU->getSlice()->getMaxNumMergeCand() -1)
      {
        uiBitsCand--;
      }   
      Double cost      = (Double)uiSad + (Double)uiBitsCand * m_pcRdCost->getSqrtLambda();

      TEncSearch::updateCandList( uiMergeCand, cost, uiNumMrgSATDCand, uiRdModeList, CandCostList );
    }
    for (UInt i=1; i<uiNumMrgSATDCand; i++)
    {
      if (CandCostList[i] > MRG_FAST_RATIO*CandCostList[0])
      {
        uiNumMrgSATDCand = i;
        break;
      }
    }
  }

  for( UInt uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; ++uiMrgHADIdx )
  {
    UInt uiMergeCand = uiRdModeList[uiMrgHADIdx];
#else
  for( UInt uiNoResidual = 0; uiNoResidual < iteration; ++uiNoResidual )
  {
    for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
    {
      if(!(uiNoResidual==1 && mergeCandBuffer[uiMergeCand]==1))
      {
        if( !(bestIsSkip && uiNoResidual == 0) )
        {
#endif
          // set MC parameters
#if QT_BT_STRUCTURE
          rpcTempCU->setPredModeSubParts( MODE_INTER, 0 ); // interprets depth relative to LCU level
#else
          rpcTempCU->setPredModeSubParts( MODE_INTER, 0, uhDepth ); // interprets depth relative to LCU level
#endif
          rpcTempCU->setCUTransquantBypassSubParts( bTransquantBypassFlag,     0, uhDepth );
#if QT_BT_STRUCTURE
          rpcTempCU->setMergeFlagSubParts( true, 0 ); // interprets depth relative to LCU level
          rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0 ); // interprets depth relative to LCU level
          rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0 ); // interprets depth relative to LCU level
#else
          rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setMergeFlagSubParts( true, 0, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth ); // interprets depth relative to LCU level
#endif
          rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
          rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level

          // do MC
#if QT_BT_STRUCTURE
          m_pcPredSearch->motionCompensation ( rpcTempCU, m_ppcPredYuvTempPU[uiWIdx][uiHIdx] );
          m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU,
            m_ppcOrigYuvPU    [uiWIdx][uiHIdx],
            m_ppcPredYuvTempPU[uiWIdx][uiHIdx],
            m_ppcResiYuvTempPU[uiWIdx][uiHIdx],
            m_ppcResiYuvBestPU[uiWIdx][uiHIdx],
            m_ppcRecoYuvTempPU[uiWIdx][uiHIdx],
            bestIsSkip? true: false);

          TComYuv* pYuv = m_ppcPredYuvTempPU[uiWIdx][uiHIdx];
          if ( rpcTempCU->getQtRootCbf(0) == 0 )
#else
          m_pcPredSearch->motionCompensation ( rpcTempCU, m_ppcPredYuvTemp[uhDepth] );
          // estimate residual and encode everything
          m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU,
                                                    m_ppcOrigYuv    [uhDepth],
                                                    m_ppcPredYuvTemp[uhDepth],
                                                    m_ppcResiYuvTemp[uhDepth],
                                                    m_ppcResiYuvBest[uhDepth],
                                                    m_ppcRecoYuvTemp[uhDepth],
                                                    (uiNoResidual? true:false));
          
          
          if ( uiNoResidual == 0 && rpcTempCU->getQtRootCbf(0) == 0 )
#endif
          {
            // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
            mergeCandBuffer[uiMergeCand] = 1;
          }

#if QT_BT_STRUCTURE
          rpcTempCU->setSkipFlagSubParts( rpcTempCU->getQtRootCbf(0) == 0, 0 );
#else
          rpcTempCU->setSkipFlagSubParts( rpcTempCU->getQtRootCbf(0) == 0, 0, uhDepth );
#endif
          Int orgQP = rpcTempCU->getQP( 0 );
          xCheckDQP( rpcTempCU );
          xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
          rpcTempCU->initEstData( uhDepth, orgQP, bTransquantBypassFlag );

#if MRG_FAST
          if (mergeCandBuffer[uiMergeCand] == 0)
          {
            //try skip mode

            // set MC parameters
            rpcTempCU->setPredModeSubParts( MODE_INTER, 0 ); // interprets depth relative to LCU level
            rpcTempCU->setCUTransquantBypassSubParts( bTransquantBypassFlag,     0, uhDepth );
            rpcTempCU->setMergeFlagSubParts( true, 0); // interprets depth relative to LCU level
            rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0 ); // interprets depth relative to LCU level
            rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0 ); // interprets depth relative to LCU level
            rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
            rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level

            //load motion compensation pred
            pYuv->copyPartToYuv(m_ppcPredYuvTempPU[uiWIdx][uiHIdx], 0);

            m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU,
              m_ppcOrigYuvPU    [uiWIdx][uiHIdx],
              m_ppcPredYuvTempPU[uiWIdx][uiHIdx],
              m_ppcResiYuvTempPU[uiWIdx][uiHIdx],
              m_ppcResiYuvBestPU[uiWIdx][uiHIdx],
              m_ppcRecoYuvTempPU[uiWIdx][uiHIdx],
              true);

            rpcTempCU->setSkipFlagSubParts( rpcTempCU->getQtRootCbf(0) == 0, 0 );
            xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
            rpcTempCU->initEstData( uhDepth, orgQP, bTransquantBypassFlag );
          }
          if (rpcBestCU->getQtRootCbf(0) == 0)
          {
            bestIsSkip = true;
          }
#else
          if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
          {
            bestIsSkip = rpcBestCU->getQtRootCbf(0) == 0;
          }
        }
      }
#endif
    }

#if !MRG_FAST
    if(uiNoResidual == 0 && m_pcEncCfg->getUseEarlySkipDetection())
    {
      if(rpcBestCU->getQtRootCbf( 0 ) == 0)
      {
        if( rpcBestCU->getMergeFlag( 0 ))
        {
          *earlyDetectionSkipMode = true;
        }
        else
        {
          Int absoulte_MV=0;
          for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if ( rpcBestCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              TComCUMvField* pcCUMvField = rpcBestCU->getCUMvField(RefPicList( uiRefListIdx ));
              Int iHor = pcCUMvField->getMvd( 0 ).getAbsHor();
              Int iVer = pcCUMvField->getMvd( 0 ).getAbsVer();
              absoulte_MV+=iHor+iVer;
            }
          }
          
          if(absoulte_MV == 0)
          {
            *earlyDetectionSkipMode = true;
          }
        }
      }
    }
  }
#endif
}


#if AMP_MRG
Void TEncCu::xCheckRDCostInter( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize, Bool bUseMRG)
#else
Void TEncCu::xCheckRDCostInter( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize )
#endif
{
  UChar uhDepth = rpcTempCU->getDepth( 0 );
  
  rpcTempCU->setDepthSubParts( uhDepth, 0 );
  
#if QT_BT_STRUCTURE
  rpcTempCU->setSkipFlagSubParts( false, 0 );
  rpcTempCU->setPredModeSubParts  ( MODE_INTER, 0 );
#else
  rpcTempCU->setSkipFlagSubParts( false, 0, uhDepth );
  rpcTempCU->setPartSizeSubParts  ( ePartSize,  0, uhDepth );
  rpcTempCU->setPredModeSubParts  ( MODE_INTER, 0, uhDepth );
#endif
  
#if QT_BT_STRUCTURE
  UInt uiWIdx = g_aucConvertToBit[rpcBestCU->getWidth(0)];
  UInt uiHIdx = g_aucConvertToBit[rpcBestCU->getHeight(0)];
#endif

#if AMP_MRG
  rpcTempCU->setMergeAMP (true);
#if QT_BT_STRUCTURE
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuvPU[uiWIdx][uiHIdx], m_ppcPredYuvTempPU[uiWIdx][uiHIdx], m_ppcResiYuvTempPU[uiWIdx][uiHIdx], m_ppcRecoYuvTempPU[uiWIdx][uiHIdx], false, bUseMRG );
#else
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth], false, bUseMRG );
#endif
#else  
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] );
#endif

#if AMP_MRG
  if ( !rpcTempCU->getMergeAMP() )
  {
    return;
  }
#endif

#if QT_BT_STRUCTURE
  m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU, m_ppcOrigYuvPU[uiWIdx][uiHIdx], m_ppcPredYuvTempPU[uiWIdx][uiHIdx], m_ppcResiYuvTempPU[uiWIdx][uiHIdx], m_ppcResiYuvBestPU[uiWIdx][uiHIdx], m_ppcRecoYuvTempPU[uiWIdx][uiHIdx], false );
#else
  m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false );
#endif
  rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

  xCheckDQP( rpcTempCU );
  xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
}

Void TEncCu::xCheckRDCostIntra( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize eSize )
{
#if QT_BT_STRUCTURE
  UInt uiWIdx = g_aucConvertToBit[rpcBestCU->getWidth(0)];
  UInt uiHIdx = g_aucConvertToBit[rpcBestCU->getHeight(0)];
#endif
  UInt uiDepth = rpcTempCU->getDepth( 0 );
  
#if QT_BT_STRUCTURE
  rpcTempCU->setSkipFlagSubParts( false, 0 );
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0 );
#else
  rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );
  rpcTempCU->setPartSizeSubParts( eSize, 0, uiDepth );
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
#endif
  
  Bool bSeparateLumaChroma = true; // choose estimation mode
  UInt uiPreCalcDistC      = 0;
#if QT_BT_STRUCTURE
  if (rpcTempCU->getTextType()==TEXT_LUMA)
  {
    m_pcPredSearch  ->estIntraPredQT      ( rpcTempCU, m_ppcOrigYuvPU[uiWIdx][uiHIdx], m_ppcPredYuvTempPU[uiWIdx][uiHIdx], m_ppcResiYuvTempPU[uiWIdx][uiHIdx], m_ppcRecoYuvTempPU[uiWIdx][uiHIdx], uiPreCalcDistC, bSeparateLumaChroma );
#if PBINTRA_FAST
    if (rpcTempCU->getTotalDistortion()==MAX_UINT)
    {
      return;
    }
#endif
    // move from outside to here, cxcTBD
    if (!rpcTempCU->getSlice()->isIntra())
    {
      m_ppcRecoYuvTempPU[uiWIdx][uiHIdx]->copyToPicLuma(rpcTempCU->getPic()->getPicYuvRec(), rpcTempCU->getAddr(), rpcTempCU->getZorderIdxInCU() );
    }
  }
  if (!(rpcTempCU->getSlice()->isIntra() && rpcTempCU->getTextType()==TEXT_LUMA))
  {
    m_pcPredSearch  ->estIntraPredChromaQT(rpcTempCU, m_ppcOrigYuvPU[uiWIdx][uiHIdx], m_ppcPredYuvTempPU[uiWIdx][uiHIdx], m_ppcResiYuvTempPU[uiWIdx][uiHIdx], m_ppcRecoYuvTempPU[uiWIdx][uiHIdx], 0 );
  }
#else
  if( !bSeparateLumaChroma )
  {
    m_pcPredSearch->preestChromaPredMode( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth] );
  }
  m_pcPredSearch  ->estIntraPredQT      ( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], uiPreCalcDistC, bSeparateLumaChroma );

  m_ppcRecoYuvTemp[uiDepth]->copyToPicLuma(rpcTempCU->getPic()->getPicYuvRec(), rpcTempCU->getAddr(), rpcTempCU->getZorderIdxInCU() );
  
  m_pcPredSearch  ->estIntraPredChromaQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], uiPreCalcDistC );
#endif
  
  m_pcEntropyCoder->resetBits();
  if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );
  }
#if QT_BT_STRUCTURE
  if (rpcTempCU->getTextType()==TEXT_LUMA)
  {
#endif
  m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePredMode( rpcTempCU, 0,          true );
#if !QT_BT_STRUCTURE
  m_pcEntropyCoder->encodePartSize( rpcTempCU, 0, uiDepth, true );
#else
  }
#endif
  m_pcEntropyCoder->encodePredInfo( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodeIPCMInfo(rpcTempCU, 0, true );

  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  m_pcEntropyCoder->encodeCoeff( rpcTempCU, 0, uiDepth, rpcTempCU->getWidth (0), rpcTempCU->getHeight(0), bCodeDQP );
#if QT_BT_STRUCTURE
  if ((g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getWidth(0) && uiWIdx==uiHIdx )  
  {
    m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );
  }

  UInt uiMaxBTD = rpcTempCU->getSlice()->isIntra() ? (rpcTempCU->getTextType()==TEXT_LUMA?MAX_BT_DEPTH:MAX_BT_DEPTH_C): MAX_BT_DEPTH_INTER;
  UInt uiMinBTSize = rpcTempCU->getSlice()->isIntra() ? (rpcTempCU->getTextType()==TEXT_LUMA?MIN_BT_SIZE:MIN_BT_SIZE_C): MIN_BT_SIZE_INTER;
  UInt uiMaxBTSize = rpcTempCU->getTextType()==TEXT_LUMA ? rpcTempCU->getSlice()->getMaxBTSize(): MAX_BT_SIZE_C;
  if (rpcTempCU->getWidth(0)<=uiMaxBTSize && rpcTempCU->getHeight(0)<=uiMaxBTSize 
    && (rpcTempCU->getWidth(0)>=2*uiMinBTSize || rpcTempCU->getHeight(0)>=2*uiMinBTSize))
  {
    UInt uiBTDepth = rpcTempCU->getBTDepth(0, rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
    assert(uiBTDepth == rpcTempCU->getBTDepth(0));
    if (uiBTDepth<uiMaxBTD)
    {
      m_pcEntropyCoder->encodeBTSplitMode(rpcTempCU, 0, rpcTempCU->getWidth(0), rpcTempCU->getHeight(0), true);
    }
  }  
#endif
  setdQPFlag( bCodeDQP );

#if QT_BT_STRUCTURE
  m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiWIdx][uiHIdx][CI_TEMP_BEST]);
#else
  m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
#endif
  
  rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
  rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
  
  xCheckDQP( rpcTempCU );
  xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth);
}

/** Check R-D costs for a CU with PCM mode. 
 * \param rpcBestCU pointer to best mode CU data structure
 * \param rpcTempCU pointer to testing mode CU data structure
 * \returns Void
 * 
 * \note Current PCM implementation encodes sample values in a lossless way. The distortion of PCM mode CUs are zero. PCM mode is selected if the best mode yields bits greater than that of PCM mode.
 */
Void TEncCu::xCheckIntraPCM( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU )
{
#if QT_BT_STRUCTURE
  UInt uiWIdx = g_aucConvertToBit[rpcBestCU->getWidth(0)];
  UInt uiHIdx = g_aucConvertToBit[rpcBestCU->getHeight(0)];
#endif
  UInt uiDepth = rpcTempCU->getDepth( 0 );

#if QT_BT_STRUCTURE
  rpcTempCU->setSkipFlagSubParts( false, 0 );
#else
  rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );
#endif

  rpcTempCU->setIPCMFlag(0, true);
  rpcTempCU->setIPCMFlagSubParts (true, 0, rpcTempCU->getDepth(0));
#if QT_BT_STRUCTURE
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0 );
#else
  rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
  rpcTempCU->setTrIdxSubParts ( 0, 0, uiDepth );
#endif

#if QT_BT_STRUCTURE
  m_pcPredSearch->IPCMSearch( rpcTempCU, m_ppcOrigYuvPU[uiWIdx][uiHIdx], m_ppcPredYuvTempPU[uiWIdx][uiHIdx], m_ppcResiYuvTempPU[uiWIdx][uiHIdx], m_ppcRecoYuvTempPU[uiWIdx][uiHIdx]);
  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiWIdx][uiHIdx][CI_CURR_BEST]);
#else
  m_pcPredSearch->IPCMSearch( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth]);
  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
#endif

  m_pcEntropyCoder->resetBits();
  if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );
  }
  m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePredMode ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePartSize ( rpcTempCU, 0, uiDepth, true );
  m_pcEntropyCoder->encodeIPCMInfo ( rpcTempCU, 0, true );

#if QT_BT_STRUCTURE
  m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiWIdx][uiHIdx][CI_TEMP_BEST]);
#else
  m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
#endif

  rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
  rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

  xCheckDQP( rpcTempCU );
  xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth );
}

/** check whether current try is the best with identifying the depth of current try
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
#if QT_BT_STRUCTURE
Void TEncCu::xCheckBestMode( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, UInt uiWidth, UInt uiHeight )
#else
Void TEncCu::xCheckBestMode( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
#endif
{
  if( rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() )
  {
    TComYuv* pcYuv;
    // Change Information data
    TComDataCU* pcCU = rpcBestCU;
    rpcBestCU = rpcTempCU;
    rpcTempCU = pcCU;

#if QT_BT_STRUCTURE
    if (uiWidth==0 || uiHeight==0)
    {
      uiWidth = rpcTempCU->getWidth(0);
      uiHeight = rpcTempCU->getHeight(0);
    }
    UInt uiWIdx = g_aucConvertToBit[uiWidth]; 
    UInt uiHIdx = g_aucConvertToBit[uiHeight];
    // Change Prediction data
    pcYuv = m_ppcPredYuvBestPU[uiWIdx][uiHIdx];
    m_ppcPredYuvBestPU[uiWIdx][uiHIdx] = m_ppcPredYuvTempPU[uiWIdx][uiHIdx];
    m_ppcPredYuvTempPU[uiWIdx][uiHIdx] = pcYuv;

    // Change Reconstruction data
    pcYuv = m_ppcRecoYuvBestPU[uiWIdx][uiHIdx];
    m_ppcRecoYuvBestPU[uiWIdx][uiHIdx] = m_ppcRecoYuvTempPU[uiWIdx][uiHIdx];
    m_ppcRecoYuvTempPU[uiWIdx][uiHIdx] = pcYuv;
#else
    // Change Prediction data
    pcYuv = m_ppcPredYuvBest[uiDepth];
    m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
    m_ppcPredYuvTemp[uiDepth] = pcYuv;

    // Change Reconstruction data
    pcYuv = m_ppcRecoYuvBest[uiDepth];
    m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];
    m_ppcRecoYuvTemp[uiDepth] = pcYuv;
#endif

    pcYuv = NULL;
    pcCU  = NULL;

    // store temp best CI for next CU coding
#if QT_BT_STRUCTURE
    m_pppcRDSbacCoder[uiWIdx][uiHIdx][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiWIdx][uiHIdx][CI_NEXT_BEST]);
#else
    m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);
#endif
  }
}

Void TEncCu::xCheckDQP( TComDataCU* pcCU )
{
  UInt uiDepth = pcCU->getDepth( 0 );

  if( pcCU->getSlice()->getPPS()->getUseDQP() && (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
#if QT_BT_STRUCTURE
    if ( pcCU->getCbf( 0, TEXT_LUMA ) || pcCU->getCbf( 0, TEXT_CHROMA_U ) || pcCU->getCbf( 0, TEXT_CHROMA_V ) )
#else
    if ( pcCU->getCbf( 0, TEXT_LUMA, 0 ) || pcCU->getCbf( 0, TEXT_CHROMA_U, 0 ) || pcCU->getCbf( 0, TEXT_CHROMA_V, 0 ) )
#endif
    {
#if !RDO_WITHOUT_DQP_BITS
      m_pcEntropyCoder->resetBits();
      m_pcEntropyCoder->encodeQP( pcCU, 0, false );
      pcCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
      pcCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      pcCU->getTotalCost() = m_pcRdCost->calcRdCost( pcCU->getTotalBits(), pcCU->getTotalDistortion() );
#endif
    }
    else
    {
      pcCU->setQPSubParts( pcCU->getRefQP( 0 ), 0, uiDepth ); // set QP to default QP
    }
  }
}

Void TEncCu::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->iN = pSrc->iN;
  for (Int i = 0; i < pSrc->iN; i++)
  {
    pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];
  }
}

#if QT_BT_STRUCTURE
Void TEncCu::xCopyYuv2PicSep(TextType eType, TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, UInt uiSrcWidth, UInt uiSrcHeight, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY )
{
  if ((g_uiMaxCUWidth >> uiSrcDepth) > uiSrcWidth || (g_uiMaxCUWidth >> uiSrcDepth)>uiSrcHeight)
  {
    //PU level
    UInt uiWIdx = g_aucConvertToBit[uiSrcWidth];
    UInt uiHIdx = g_aucConvertToBit[uiSrcHeight];
    if (eType == TEXT_LUMA)
    {
      m_ppcRecoYuvBestPU[uiWIdx][uiHIdx]->copyToPicLuma( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, 0, 0);
    }
    else
    {
      m_ppcRecoYuvBestPU[uiWIdx][uiHIdx]->copyToPicChroma( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, 0, 0);
    }
  }
  else
  {
    //CU level
    UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
    UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
    TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
    Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
      pcSlice->getSliceSegmentCurStartCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
    Bool bSliceEnd   = pcSlice->getSliceSegmentCurEndCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
      pcSlice->getSliceSegmentCurEndCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
    if(!bSliceEnd && !bSliceStart && ( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
    {
      UInt uiAbsPartIdxInRaster = g_auiZscanToRaster[uiAbsPartIdx];
      UInt uiSrcBlkWidth = rpcPic->getNumPartInWidth() >> (uiSrcDepth);
      UInt uiBlkWidth    = rpcPic->getNumPartInWidth() >> (uiDepth);
      UInt uiPartIdxX = ( ( uiAbsPartIdxInRaster % rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
      UInt uiPartIdxY = ( ( uiAbsPartIdxInRaster / rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
      UInt uiPartIdx = uiPartIdxY * ( uiSrcBlkWidth / uiBlkWidth ) + uiPartIdxX;

      UInt uiWIdx = g_aucConvertToBit[uiSrcWidth];
      UInt uiHIdx = g_aucConvertToBit[uiSrcHeight];
      if (eType == TEXT_LUMA)
      {
        assert(0);
        m_ppcRecoYuvBestPU[uiWIdx][uiHIdx]->copyToPicLuma( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);
      }
      else 
      {
        m_ppcRecoYuvBestPU[uiWIdx][uiHIdx]->copyToPicChroma( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);
      }
    }
    else
    {
      UInt uiQNumParts = ( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) )>>2;

      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
      {
        UInt uiSubCULPelX   = uiLPelX + ( g_uiMaxCUWidth >>(uiDepth+1) )*( uiPartUnitIdx &  1 );
        UInt uiSubCUTPelY   = uiTPelY + ( g_uiMaxCUHeight>>(uiDepth+1) )*( uiPartUnitIdx >> 1 );

        Bool bInSlice = rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+uiQNumParts > pcSlice->getSliceSegmentCurStartCUAddr() && 
          rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx < pcSlice->getSliceSegmentCurEndCUAddr();
        if(bInSlice&&( uiSubCULPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiSubCUTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
        {
          xCopyYuv2PicSep( eType, rpcPic, uiCUAddr, uiAbsPartIdx, uiDepth+1, uiSrcDepth, uiSrcWidth, uiSrcHeight, pcCU, uiSubCULPelX, uiSubCUTPelY );   // Copy Yuv data to picture Yuv
        }
      }
    }
  }
}
#endif

#if QT_BT_STRUCTURE
Void TEncCu::xCopyYuv2Pic(TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, UInt uiSrcWidth, UInt uiSrcHeight, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY )
#else
Void TEncCu::xCopyYuv2Pic(TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY )
#endif
{
#if QT_BT_STRUCTURE
  if ((g_uiMaxCUWidth >> uiSrcDepth) > uiSrcWidth || (g_uiMaxCUWidth >> uiSrcDepth)>uiSrcHeight)
  {
    //PU level
    UInt uiWIdx = g_aucConvertToBit[uiSrcWidth];
    UInt uiHIdx = g_aucConvertToBit[uiSrcHeight];
    m_ppcRecoYuvBestPU[uiWIdx][uiHIdx]->copyToPicYuv( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, 0, 0);
  }
  else
  {
    //CU level
#endif
    UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
    UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
    TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
    Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
      pcSlice->getSliceSegmentCurStartCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
    Bool bSliceEnd   = pcSlice->getSliceSegmentCurEndCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
      pcSlice->getSliceSegmentCurEndCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
    if(!bSliceEnd && !bSliceStart && ( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
    {
      UInt uiAbsPartIdxInRaster = g_auiZscanToRaster[uiAbsPartIdx];
      UInt uiSrcBlkWidth = rpcPic->getNumPartInWidth() >> (uiSrcDepth);
      UInt uiBlkWidth    = rpcPic->getNumPartInWidth() >> (uiDepth);
      UInt uiPartIdxX = ( ( uiAbsPartIdxInRaster % rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
      UInt uiPartIdxY = ( ( uiAbsPartIdxInRaster / rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
      UInt uiPartIdx = uiPartIdxY * ( uiSrcBlkWidth / uiBlkWidth ) + uiPartIdxX;
#if QT_BT_STRUCTURE
      UInt uiWIdx = g_aucConvertToBit[uiSrcWidth];
      UInt uiHIdx = g_aucConvertToBit[uiSrcHeight];
      m_ppcRecoYuvBestPU[uiWIdx][uiHIdx]->copyToPicYuv( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);
#else
      m_ppcRecoYuvBest[uiSrcDepth]->copyToPicYuv( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);
#endif
    }
    else
    {
      UInt uiQNumParts = ( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) )>>2;

      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
      {
        UInt uiSubCULPelX   = uiLPelX + ( g_uiMaxCUWidth >>(uiDepth+1) )*( uiPartUnitIdx &  1 );
        UInt uiSubCUTPelY   = uiTPelY + ( g_uiMaxCUHeight>>(uiDepth+1) )*( uiPartUnitIdx >> 1 );

        Bool bInSlice = rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+uiQNumParts > pcSlice->getSliceSegmentCurStartCUAddr() && 
          rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx < pcSlice->getSliceSegmentCurEndCUAddr();
        if(bInSlice&&( uiSubCULPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiSubCUTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
        {
#if QT_BT_STRUCTURE
          xCopyYuv2Pic( rpcPic, uiCUAddr, uiAbsPartIdx, uiDepth+1, uiSrcDepth, uiSrcWidth, uiSrcHeight, pcCU, uiSubCULPelX, uiSubCUTPelY );   // Copy Yuv data to picture Yuv
#else
          xCopyYuv2Pic( rpcPic, uiCUAddr, uiAbsPartIdx, uiDepth+1, uiSrcDepth, pcCU, uiSubCULPelX, uiSubCUTPelY );   // Copy Yuv data to picture Yuv
#endif
        }
      }
    }
#if QT_BT_STRUCTURE
  }
#endif
}

#if QT_BT_STRUCTURE
Void TEncCu::xCopyBestYuvFrom( UInt uiPartUnitIdx, UInt uiWidth, UInt uiHeight, UInt uiSplitMethod )
{
  UInt uiWIdx = g_aucConvertToBit[uiWidth];
  UInt uiHIdx = g_aucConvertToBit[uiHeight];
  UInt uiNextWIdx = uiWIdx - ((uiSplitMethod & 1)==0 ? 1: 0);
  UInt uiNextHIdx = uiHIdx - ((uiSplitMethod & 2)==0 ? 1: 0);
  m_ppcRecoYuvBestPU[uiWIdx][uiHIdx]->copyPartToYuv( m_ppcRecoYuvBestPU[uiNextWIdx][uiNextHIdx], uiPartUnitIdx );
}
#endif

#if QT_BT_STRUCTURE //uiSplitMethod: 0: quadtree; 1: hor; 2: ver
Void TEncCu::xCopyYuv2Tmp( UInt uiPartUnitIdx, UInt uiWidth, UInt uiHeight, UInt uiSplitMethod )
#else
Void TEncCu::xCopyYuv2Tmp( UInt uiPartUnitIdx, UInt uiNextDepth )
#endif
{
#if QT_BT_STRUCTURE 
  UInt uiWIdx = g_aucConvertToBit[uiWidth];
  UInt uiHIdx = g_aucConvertToBit[uiHeight];
  UInt uiNextWIdx = uiWIdx - ((uiSplitMethod & 1)==0 ? 1: 0);
  UInt uiNextHIdx = uiHIdx - ((uiSplitMethod & 2)==0 ? 1: 0);
  m_ppcRecoYuvBestPU[uiNextWIdx][uiNextHIdx]->copyToPartYuv( m_ppcRecoYuvTempPU[uiWIdx][uiHIdx], uiPartUnitIdx );
#else
  UInt uiCurrDepth = uiNextDepth - 1;
  m_ppcRecoYuvBest[uiNextDepth]->copyToPartYuv( m_ppcRecoYuvTemp[uiCurrDepth], uiPartUnitIdx );
#endif
}

/** Function for filling the PCM buffer of a CU using its original sample array 
 * \param pcCU pointer to current CU
 * \param pcOrgYuv pointer to original sample array
 * \returns Void
 */
Void TEncCu::xFillPCMBuffer     ( TComDataCU*& pCU, TComYuv* pOrgYuv )
{

  UInt   width        = pCU->getWidth(0);
  UInt   height       = pCU->getHeight(0);

  Pel*   pSrcY = pOrgYuv->getLumaAddr(0, width); 
  Pel*   pDstY = pCU->getPCMSampleY();
  UInt   srcStride = pOrgYuv->getStride();

  for(Int y = 0; y < height; y++ )
  {
    for(Int x = 0; x < width; x++ )
    {
      pDstY[x] = pSrcY[x];
    }
    pDstY += width;
    pSrcY += srcStride;
  }

  Pel* pSrcCb       = pOrgYuv->getCbAddr();
  Pel* pSrcCr       = pOrgYuv->getCrAddr();;

  Pel* pDstCb       = pCU->getPCMSampleCb();
  Pel* pDstCr       = pCU->getPCMSampleCr();;

  UInt srcStrideC = pOrgYuv->getCStride();
  UInt heightC   = height >> 1;
  UInt widthC    = width  >> 1;

  for(Int y = 0; y < heightC; y++ )
  {
    for(Int x = 0; x < widthC; x++ )
    {
      pDstCb[x] = pSrcCb[x];
      pDstCr[x] = pSrcCr[x];
    }
    pDstCb += widthC;
    pDstCr += widthC;
    pSrcCb += srcStrideC;
    pSrcCr += srcStrideC;
  }
}

#if ADAPTIVE_QP_SELECTION
/** Collect ARL statistics from one block
  */
Int TEncCu::xTuCollectARLStats(TCoeff* rpcCoeff, Int* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples )
{
  for( Int n = 0; n < NumCoeffInCU; n++ )
  {
    Int u = abs( rpcCoeff[ n ] );
    Int absc = rpcArlCoeff[ n ];

    if( u != 0 )
    {
      if( u < LEVEL_RANGE )
      {
        cSum[ u ] += ( Double )absc;
        numSamples[ u ]++;
      }
      else 
      {
        cSum[ LEVEL_RANGE ] += ( Double )absc - ( Double )( u << ARL_C_PRECISION );
        numSamples[ LEVEL_RANGE ]++;
      }
    }
  }

  return 0;
}

/** Collect ARL statistics from one LCU
 * \param pcCU
 */
Void TEncCu::xLcuCollectARLStats(TComDataCU* rpcCU )
{
  Double cSum[ LEVEL_RANGE + 1 ];     //: the sum of DCT coefficients corresponding to datatype and quantization output
  UInt numSamples[ LEVEL_RANGE + 1 ]; //: the number of coefficients corresponding to datatype and quantization output

  TCoeff* pCoeffY = rpcCU->getCoeffY();
  Int* pArlCoeffY = rpcCU->getArlCoeffY();

  UInt uiMinCUWidth = g_uiMaxCUWidth >> g_uiMaxCUDepth;
  UInt uiMinNumCoeffInCU = 1 << uiMinCUWidth;

  memset( cSum, 0, sizeof( Double )*(LEVEL_RANGE+1) );
  memset( numSamples, 0, sizeof( UInt )*(LEVEL_RANGE+1) );

  // Collect stats to cSum[][] and numSamples[][]
  for(Int i = 0; i < rpcCU->getTotalNumPart(); i ++ )
  {
#if !QT_BT_STRUCTURE
    UInt uiTrIdx = rpcCU->getTransformIdx(i);
#endif

    if(rpcCU->getPredictionMode(i) == MODE_INTER)
#if QT_BT_STRUCTURE
    if( rpcCU->getCbf( i, TEXT_LUMA ) )
#else
    if( rpcCU->getCbf( i, TEXT_LUMA, uiTrIdx ) )
#endif
    {
      xTuCollectARLStats(pCoeffY, pArlCoeffY, uiMinNumCoeffInCU, cSum, numSamples);
    }//Note that only InterY is processed. QP rounding is based on InterY data only.
   
    pCoeffY  += uiMinNumCoeffInCU;
    pArlCoeffY  += uiMinNumCoeffInCU;
  }

  for(Int u=1; u<LEVEL_RANGE;u++)
  {
    m_pcTrQuant->getSliceSumC()[u] += cSum[ u ] ;
    m_pcTrQuant->getSliceNSamples()[u] += numSamples[ u ] ;
  }
  m_pcTrQuant->getSliceSumC()[LEVEL_RANGE] += cSum[ LEVEL_RANGE ] ;
  m_pcTrQuant->getSliceNSamples()[LEVEL_RANGE] += numSamples[ LEVEL_RANGE ] ;
}
#endif
//! \}
