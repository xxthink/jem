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

/** \file     TEncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#ifndef __TENCCU__
#define __TENCCU__

// Include files
#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComYuv.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComBitCounter.h"
#include "TLibCommon/TComDataCU.h"

#include "TEncEntropy.h"
#include "TEncSearch.h"
#include "TEncRateCtrl.h"
//! \ingroup TLibEncoder
//! \{

class TEncTop;
class TEncSbac;
class TEncCavlc;
class TEncSlice;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU encoder class
class TEncCu
{
private:
  
#if !QT_BT_STRUCTURE
  TComDataCU**            m_ppcBestCU;      ///< Best CUs in each depth
  TComDataCU**            m_ppcTempCU;      ///< Temporary CUs in each depth
#endif
  UChar                   m_uhTotalDepth;
  
#if QT_BT_STRUCTURE
  TComDataCU***            m_ppcBestCUPU;      ///< Best CUs in each depth
  TComDataCU***            m_ppcTempCUPU;      ///< Temporary CUs in each depth
  
  TComYuv***               m_ppcPredYuvBestPU; ///< Best Prediction Yuv for each depth
  TComYuv***               m_ppcResiYuvBestPU; ///< Best Residual Yuv for each depth
  TComYuv***               m_ppcRecoYuvBestPU; ///< Best Reconstruction Yuv for each depth
  TComYuv***               m_ppcPredYuvTempPU; ///< Temporary Prediction Yuv for each depth
  TComYuv***               m_ppcResiYuvTempPU; ///< Temporary Residual Yuv for each depth
  TComYuv***               m_ppcRecoYuvTempPU; ///< Temporary Reconstruction Yuv for each depth
  TComYuv***               m_ppcOrigYuvPU;     ///< Original Yuv for each depth
#else
  TComYuv**               m_ppcPredYuvBest; ///< Best Prediction Yuv for each depth
  TComYuv**               m_ppcResiYuvBest; ///< Best Residual Yuv for each depth
  TComYuv**               m_ppcRecoYuvBest; ///< Best Reconstruction Yuv for each depth
  TComYuv**               m_ppcPredYuvTemp; ///< Temporary Prediction Yuv for each depth
  TComYuv**               m_ppcResiYuvTemp; ///< Temporary Residual Yuv for each depth
  TComYuv**               m_ppcRecoYuvTemp; ///< Temporary Reconstruction Yuv for each depth
  TComYuv**               m_ppcOrigYuv;     ///< Original Yuv for each depth
#endif

  //  Data : encoder control
  Bool                    m_bEncodeDQP;
  
  //  Access channel
  TEncCfg*                m_pcEncCfg;
  TEncSearch*             m_pcPredSearch;
  TComTrQuant*            m_pcTrQuant;
  TComBitCounter*         m_pcBitCounter;
  TComRdCost*             m_pcRdCost;
  
  TEncEntropy*            m_pcEntropyCoder;
  TEncCavlc*              m_pcCavlcCoder;
  TEncSbac*               m_pcSbacCoder;
  TEncBinCABAC*           m_pcBinCABAC;
  
  // SBAC RD
#if QT_BT_STRUCTURE
  TEncSbac****            m_pppcRDSbacCoder;
#else
  TEncSbac***             m_pppcRDSbacCoder;
#endif
  TEncSbac*               m_pcRDGoOnSbacCoder;
  TEncRateCtrl*           m_pcRateCtrl;
public:
  /// copy parameters from encoder class
  Void  init                ( TEncTop* pcEncTop );
  
  /// create internal buffers
  Void  create              ( UChar uhTotalDepth, UInt iMaxWidth, UInt iMaxHeight );
  
  /// destroy internal buffers
  Void  destroy             ();
  
  /// CU analysis function
  Void  compressCU          ( TComDataCU*&  rpcCU );
  /// CU encoding function
  Void  encodeCU            ( TComDataCU*    pcCU );
  
  Void setBitCounter        ( TComBitCounter* pcBitCounter ) { m_pcBitCounter = pcBitCounter; }
  Int   updateLCUDataISlice ( TComDataCU* pcCU, Int LCUIdx, Int width, Int height );
protected:
  Void  finishCU            ( TComDataCU*  pcCU, UInt uiAbsPartIdx,           UInt uiDepth        );
#if AMP_ENC_SPEEDUP
#if QT_BT_STRUCTURE
  Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, UInt uiWidth, UInt uiHeight, UInt uiBTSplitMode, UInt uiSplitConstrain = 0 );
#else
  Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize = SIZE_NONE );
#endif
#else
  Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth        );
#endif
#if QT_BT_STRUCTURE
  Void  xEncodeCU           ( TComDataCU*  pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight, UInt& ruiLastIdx, UInt& ruiLastDepth, UInt uiSplitConstrain = 0 );
#else
  Void  xEncodeCU           ( TComDataCU*  pcCU, UInt uiAbsPartIdx,           UInt uiDepth        );
#endif
  
  Int   xComputeQP          ( TComDataCU* pcCU, UInt uiDepth );
#if QT_BT_STRUCTURE
  Void  xCheckBestMode      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, UInt uiWidth=0, UInt uiHeight=0        );
#else
  Void  xCheckBestMode      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth        );
#endif
  
  Void  xCheckRDCostMerge2Nx2N( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Bool *earlyDetectionSkipMode);

#if AMP_MRG
  Void  xCheckRDCostInter   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize, Bool bUseMRG = false  );
#else
  Void  xCheckRDCostInter   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
#endif
  Void  xCheckRDCostIntra   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
  Void  xCheckDQP           ( TComDataCU*  pcCU );
  
  Void  xCheckIntraPCM      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU                      );
  Void  xCopyAMVPInfo       ( AMVPInfo* pSrc, AMVPInfo* pDst );
#if QT_BT_STRUCTURE
  Void  xCopyYuv2Pic        (TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, UInt uiSrcWidth, UInt uiSrcHeight, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY );
#else
  Void  xCopyYuv2Pic        (TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY );
#endif
#if QT_BT_STRUCTURE
  Void  xCopyYuv2PicSep     (TextType eType, TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, UInt uiSrcWidth, UInt uiSrcHeight, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY );
  Void  xCopyYuv2Tmp        ( UInt uhPartUnitIdx, UInt uiWidth, UInt uiHeight, UInt uiSplitMethod=0 );
  Void  xCopyBestYuvFrom    ( UInt uhPartUnitIdx, UInt uiWidth, UInt uiHeight, UInt uiSplitMethod=0 );
#else
  Void  xCopyYuv2Tmp        ( UInt uhPartUnitIdx, UInt uiDepth );
#endif

  Bool getdQPFlag           ()                        { return m_bEncodeDQP;        }
  Void setdQPFlag           ( Bool b )                { m_bEncodeDQP = b;           }

#if ADAPTIVE_QP_SELECTION
  // Adaptive reconstruction level (ARL) statistics collection functions
  Void xLcuCollectARLStats(TComDataCU* rpcCU);
  Int  xTuCollectARLStats(TCoeff* rpcCoeff, Int* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples );
#endif

#if !QT_BT_STRUCTURE
#if AMP_ENC_SPEEDUP 
#if AMP_MRG
  Void deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver);
#else
  Void deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver);
#endif
#endif
#endif

  Void  xFillPCMBuffer     ( TComDataCU*& pCU, TComYuv* pOrgYuv ); 
};

//! \}

#endif // __TENCMB__
