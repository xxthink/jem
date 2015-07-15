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
#if QC_IMV
#include <set>
#endif
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
  
  TComDataCU**            m_ppcBestCU;      ///< Best CUs in each depth
  TComDataCU**            m_ppcTempCU;      ///< Temporary CUs in each depth
#if QC_IMV
  TComDataCU**            m_ppcTempCUIMVCache[NUMBER_OF_PART_SIZES]; 
#endif
#if QC_OBMC
  TComDataCU**            m_ppcTempCUWoOBMC;      ///< Temporary CUs in each depth
#endif
#if QC_FRUC_MERGE
  TComDataCU**            m_ppcFRUCBufferCU;      
#endif
  UChar                   m_uhTotalDepth;
  
  TComYuv**               m_ppcPredYuvBest; ///< Best Prediction Yuv for each depth
  TComYuv**               m_ppcResiYuvBest; ///< Best Residual Yuv for each depth
  TComYuv**               m_ppcRecoYuvBest; ///< Best Reconstruction Yuv for each depth
  TComYuv**               m_ppcPredYuvTemp; ///< Temporary Prediction Yuv for each depth
  TComYuv**               m_ppcResiYuvTemp; ///< Temporary Residual Yuv for each depth
  TComYuv**               m_ppcRecoYuvTemp; ///< Temporary Reconstruction Yuv for each depth
  TComYuv**               m_ppcOrigYuv;     ///< Original Yuv for each depth
#if QC_OBMC
  TComYuv**               m_ppcTmpYuv1;     ///< Temporary Yuv used for OBMC
  TComYuv**               m_ppcTmpYuv2;     ///< Temporary Yuv used for OBMC
  TComYuv**               m_ppcPredYuvWoOBMC; ///< Temporary Prediction Yuv for each depth
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
  TEncSbac***             m_pppcRDSbacCoder;
  TEncSbac*               m_pcRDGoOnSbacCoder;
  TEncRateCtrl*           m_pcRateCtrl;

#if QC_SUB_PU_TMVP
#if QC_SUB_PU_TMVP_EXT
  TComMvField  * m_pMvFieldSP[2];
  UChar        * m_phInterDirSP[2];
#else
  TComMvField  * m_pMvFieldSP;
  UChar        * m_phInterDirSP;
#endif
#endif
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
  Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, PartSize eParentPartSize = SIZE_NONE );
#else
  Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth        );
#endif
  Void  xEncodeCU           ( TComDataCU*  pcCU, UInt uiAbsPartIdx,           UInt uiDepth        );
  
  Int   xComputeQP          ( TComDataCU* pcCU, UInt uiDepth );
  Void  xCheckBestMode      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth        );
  
  Void  xCheckRDCostMerge2Nx2N( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, Bool *earlyDetectionSkipMode);
#if QC_FRUC_MERGE
  Void  xCheckRDCostMerge2Nx2NFRUC( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU , Bool *earlyDetectionSkipMode );
#endif

#if AMP_MRG
  Void  xCheckRDCostInter   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize, Bool bUseMRG = false 
#if QC_IMV
    , Bool bIMV = false , TComDataCU * pcCUInfo2Reuse = NULL 
#endif
    );
#else
  Void  xCheckRDCostInter   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
#endif
  Void  xCheckRDCostIntra   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
  Void  xCheckDQP           ( TComDataCU*  pcCU );
  
  Void  xCheckIntraPCM      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU                      );
  Void  xCopyAMVPInfo       ( AMVPInfo* pSrc, AMVPInfo* pDst );
  Void  xCopyYuv2Pic        (TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY );
  Void  xCopyYuv2Tmp        ( UInt uhPartUnitIdx, UInt uiDepth );

  Bool getdQPFlag           ()                        { return m_bEncodeDQP;        }
  Void setdQPFlag           ( Bool b )                { m_bEncodeDQP = b;           }

#if ADAPTIVE_QP_SELECTION
  // Adaptive reconstruction level (ARL) statistics collection functions
  Void xLcuCollectARLStats(TComDataCU* rpcCU);
  Int  xTuCollectARLStats(TCoeff* rpcCoeff, Int* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples );
#endif

#if AMP_ENC_SPEEDUP 
#if AMP_MRG
  Void deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver);
#else
  Void deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver);
#endif
#endif
  Void  xFillPCMBuffer     ( TComDataCU*& pCU, TComYuv* pOrgYuv ); 

#if QC_IMV
private:
  typedef struct
  {
    Double    dRDCost;
    Bool      bUseMrg;
    PartSize  eInterPartSize;
    TComDataCU * pcCUMode;
  }SModeCand;

  class cmpModeCand
  {
  public:
    bool operator()(const SModeCand & r1 , const SModeCand & r2 ) const
    {
      return r1.dRDCost < r2.dRDCost;
    }
  };
  std::set <SModeCand, cmpModeCand> m_setInterCand;
#endif
};

//! \}

#endif // __TENCMB__
