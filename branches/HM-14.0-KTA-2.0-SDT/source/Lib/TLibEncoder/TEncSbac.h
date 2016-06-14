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

/** \file     TEncSbac.h
    \brief    Context-adaptive entropy encoder class (header)
*/

#ifndef __TENCSBAC__
#define __TENCSBAC__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TLibCommon/TComBitStream.h"
#include "TLibCommon/ContextTables.h"
#include "TLibCommon/ContextModel.h"
#include "TLibCommon/ContextModel3DBuffer.h"
#include "TEncEntropy.h"
#include "TEncBinCoder.h"
#include "TEncBinCoderCABAC.h"
#if FAST_BIT_EST
#include "TEncBinCoderCABACCounter.h"
#endif

class TEncTop;

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// SBAC encoder class
class TEncSbac : public TEncEntropyIf
{
public:
  TEncSbac();
  virtual ~TEncSbac();
  
  Void  init                   ( TEncBinIf* p )  { m_pcBinIf = p; }
  Void  uninit                 ()                { m_pcBinIf = 0; }

  //  Virtual list
  Void  resetEntropy           ();
  Void  determineCabacInitIdx  ();
  Void  setBitstream           ( TComBitIf* p )  { m_pcBitIf = p; m_pcBinIf->init( p ); }
  Void  setSlice               ( TComSlice* p )  { m_pcSlice = p;                       }
  // SBAC RD
  Void  resetCoeffCost         ()                { m_uiCoeffCost = 0;  }
  UInt  getCoeffCost           ()                { return  m_uiCoeffCost;  }
  
  Void  load                   ( TEncSbac* pScr  );
  Void  loadIntraDirModeLuma   ( TEncSbac* pScr  );
  Void  store                  ( TEncSbac* pDest );
  Void  loadContexts           ( TEncSbac* pScr  );
  Void  resetBits              ()                { m_pcBinIf->resetBits(); m_pcBitIf->resetBits(); }
  UInt  getNumberOfWrittenBits ()                { return m_pcBinIf->getNumWrittenBits(); }
  //--SBAC RD

  Void  codeVPS                 ( TComVPS* pcVPS );
  Void  codeSPS                 ( TComSPS* pcSPS     );
  Void  codePPS                 ( TComPPS* pcPPS     );
  Void  codeSliceHeader         ( TComSlice* pcSlice );
  Void  codeTilesWPPEntryPoint( TComSlice* pSlice );
  Void  codeTerminatingBit      ( UInt uilsLast      );
  Void  codeSliceFinish         ();
  Void  codeSaoMaxUvlc    ( UInt code, UInt maxSymbol );
  Void  codeSaoMerge  ( UInt  uiCode );
  Void  codeSaoTypeIdx    ( UInt  uiCode);
  Void  codeSaoUflc       ( UInt uiLength, UInt  uiCode );
  Void  codeSAOSign       ( UInt  uiCode);  //<! code SAO offset sign
  Void  codeScalingList      ( TComScalingList* /*scalingList*/     ){ assert (0);  return;};

  Void codeSAOOffsetParam(Int compIdx, SAOOffset& ctbParam, Bool sliceEnabled);
  Void codeSAOBlkParam(SAOBlkParam& saoBlkParam
                    , Bool* sliceEnabled
                    , Bool leftMergeAvail
                    , Bool aboveMergeAvail
                    , Bool onlyEstMergeInfo = false
                    );
#if QC_AC_ADAPT_WDOW
  Int  getCtxNumber()     {return m_numContextModels;}
  Void codeCtxUpdateInfo  (TComSlice* pcSlice,  TComStats* apcStats) ;
  Void xUpdateWindowSize (SliceType eSliceType, Int iQPIdx);
#if INIT_PREVFRAME
  Void  loadContextsFromPrev (TComStats* apcStats, SliceType eSliceType, Int iQPIdx, Bool bFromGloble, Int iQPIdxRst =-1, Bool bAfterLastISlice= false);
#endif
#endif

private:
  Void  xWriteUnarySymbol    ( UInt uiSymbol, ContextModel* pcSCModel, Int iOffset );
  Void  xWriteUnaryMaxSymbol ( UInt uiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol );
  Void  xWriteEpExGolomb     ( UInt uiSymbol, UInt uiCount );
#if QC_CTX_RESIDUALCODING
  Void  xWriteGoRiceExGolomb     ( UInt uiSymbol, UInt &ruiGoRiceParam );
#else
  Void  xWriteCoefRemainExGolomb ( UInt symbol, UInt &rParam );
#endif
  Void  xCopyFrom            ( TEncSbac* pSrc );
  Void  xCopyContextsFrom    ( TEncSbac* pSrc );  
  
  Void codeDFFlag( UInt /*uiCode*/, const Char* /*pSymbolName*/ )       {printf("Not supported in codeDFFlag()\n"); assert(0); exit(1);};
  Void codeDFSvlc( Int /*iCode*/, const Char* /*pSymbolName*/ )         {printf("Not supported in codeDFSvlc()\n"); assert(0); exit(1);};

protected:
  TComBitIf*    m_pcBitIf;
  TComSlice*    m_pcSlice;
  TEncBinIf*    m_pcBinIf;
  //SBAC RD
  UInt          m_uiCoeffCost;

  //--Adaptive loop filter
  
public:
  Void codeCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeSkipFlag      ( TComDataCU* pcCU, UInt uiAbsPartIdx );
#if ROT_TR 
    Void codeROTIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx,UInt uiDepth  );
#endif
#if CU_LEVEL_MPI
     Void codeMPIIdx     ( TComDataCU* pcCU, UInt uiAbsPartIdx );
#endif
#if QC_IMV
  Void codeiMVFlag      ( TComDataCU* pcCU, UInt uiAbsPartIdx );
#endif
#if QC_OBMC
  Void codeOBMCFlag      ( TComDataCU* pcCU, UInt uiAbsPartIdx );
#endif
#if QC_IC
  Void codeICFlag        ( TComDataCU* pcCU, UInt uiAbsPartIdx );
#endif 
  Void codeMergeFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeMergeIndex    ( TComDataCU* pcCU, UInt uiAbsPartIdx );
#if QC_FRUC_MERGE
  Void codeFRUCMgrMode  ( TComDataCU* pcCU, UInt uiAbsPartIdx , UInt uiPUIdx );
#endif
  Void codeSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  Void codeMVPIdx        ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList );
  
#if QC_EMT
  Void codeEmtTuIdx      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  Void codeEmtCuFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRootCbf );
#endif

  Void codePartSize      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  Void codePredMode      ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeIPCMInfo      ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeTransformSubdivFlag ( UInt uiSymbol, UInt uiCtx );
  Void codeQtCbf               ( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth );
  Void codeQtRootCbf           ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeQtCbfZero           ( TComDataCU* pcCU, TextType eType, UInt uiTrDepth );
  Void codeQtRootCbfZero       ( TComDataCU* pcCU );
  Void codeIntraDirLumaAng     ( TComDataCU* pcCU, UInt absPartIdx, Bool isMultiple
#if QC_USE_65ANG_MODES
    , Int* piModes = NULL, Int  iAboveLeftCase = -1
#endif
    );
  
  Void codeIntraDirChroma      ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeInterDir            ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeRefFrmIdx           ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList );
  Void codeMvd                 ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList );
  
  Void codeDeltaQP             ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  
  Void codeLastSignificantXY ( UInt uiPosX, UInt uiPosY, Int width, Int height, TextType eTType, UInt uiScanIdx );
  Void codeCoeffNxN            ( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType 
#if ROT_TR   || CU_LEVEL_MPI 
    ,  Int& bCbfCU
#endif
    );
  void codeTransformSkipFlags ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt width, UInt height, TextType eTType );

#if KLT_COMMON
  Void codeKLTFlags           ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt width, UInt height, TextType eTType );
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // for RD-optimizatioon
  // -------------------------------------------------------------------------------------------------------------------
  
  Void estBit               (estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, TextType eTType);
  Void estCBFBit                     ( estBitsSbacStruct* pcEstBitsSbac );
  Void estSignificantCoeffGroupMapBit( estBitsSbacStruct* pcEstBitsSbac, TextType eTType );
  Void estSignificantMapBit          ( estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, TextType eTType );
  Void estSignificantCoefficientsBit ( estBitsSbacStruct* pcEstBitsSbac, TextType eTType );
  
  Void updateContextTables           ( SliceType eSliceType, Int iQp, Bool bExecuteFinish=true  );
  Void updateContextTables           ( SliceType eSliceType, Int iQp  ) { this->updateContextTables( eSliceType, iQp, true); };
  
  TEncBinIf* getEncBinIf()  { return m_pcBinIf; }

#if ALF_HM3_QC_REFACTOR
  Bool  getAlfCtrl             ()                         { return m_bAlfCtrl;          }
  UInt  getMaxAlfCtrlDepth     ()                         { return m_uiMaxAlfCtrlDepth; }
  Void  setAlfCtrl             ( Bool bAlfCtrl          ) { m_bAlfCtrl          = bAlfCtrl;          }
  Void  setMaxAlfCtrlDepth     ( UInt uiMaxAlfCtrlDepth ) { m_uiMaxAlfCtrlDepth = uiMaxAlfCtrlDepth; }
  Void  codeAlfFlag       ( UInt uiCode );
  Void  codeAlfUvlc       ( UInt uiCode );
  Void  codeAlfSvlc       ( Int  uiCode );
  Void  codeAlfCtrlDepth  ();
  Void codeAlfFlagNum        ( UInt uiCode, UInt minValue );
  Void codeAlfCtrlFlag       ( UInt uiSymbol );
  Void codeAlfCtrlFlag   ( TComDataCU* pcCU, UInt uiAbsPartIdx );
#endif
private:
  UInt                 m_uiLastQp;
  
  ContextModel         m_contextModels[MAX_NUM_CTX_MOD];
  Int                  m_numContextModels;
  ContextModel3DBuffer m_cCUSplitFlagSCModel;
  ContextModel3DBuffer m_cCUSkipFlagSCModel;
#if ROT_TR
    ContextModel3DBuffer m_cROTidxSCModel;
#endif
#if CU_LEVEL_MPI
    ContextModel3DBuffer m_cMPIIdxSCModel;
#endif
#if QC_IMV
  ContextModel3DBuffer m_cCUiMVFlagSCModel;
#endif
#if QC_OBMC
  ContextModel3DBuffer m_cCUOBMCFlagSCModel;
#endif
#if QC_IC
  ContextModel3DBuffer m_cCUICFlagSCModel;
#endif
  ContextModel3DBuffer m_cCUMergeFlagExtSCModel;
  ContextModel3DBuffer m_cCUMergeIdxExtSCModel;
#if QC_FRUC_MERGE
  ContextModel3DBuffer m_cCUFRUCMgrModeSCModel;
  ContextModel3DBuffer m_cCUFRUCMESCModel;
#endif
#if QC_EMT
  ContextModel3DBuffer m_cEmtTuIdxSCModel;
  ContextModel3DBuffer m_cEmtCuFlagSCModel;
#endif
  ContextModel3DBuffer m_cCUPartSizeSCModel;
  ContextModel3DBuffer m_cCUPredModeSCModel;
  ContextModel3DBuffer m_cCUIntraPredSCModel;
  ContextModel3DBuffer m_cCUChromaPredSCModel;
  ContextModel3DBuffer m_cCUDeltaQpSCModel;
  ContextModel3DBuffer m_cCUInterDirSCModel;
  ContextModel3DBuffer m_cCURefPicSCModel;
  ContextModel3DBuffer m_cCUMvdSCModel;
  ContextModel3DBuffer m_cCUQtCbfSCModel;
  ContextModel3DBuffer m_cCUTransSubdivFlagSCModel;
  ContextModel3DBuffer m_cCUQtRootCbfSCModel;
  
  ContextModel3DBuffer m_cCUSigCoeffGroupSCModel;
  ContextModel3DBuffer m_cCUSigSCModel;
  ContextModel3DBuffer m_cCuCtxLastX;
  ContextModel3DBuffer m_cCuCtxLastY;
  ContextModel3DBuffer m_cCUOneSCModel;
#if !QC_CTX_RESIDUALCODING
  ContextModel3DBuffer m_cCUAbsSCModel;
#endif

  ContextModel3DBuffer m_cMVPIdxSCModel;
  
  ContextModel3DBuffer m_cSaoMergeSCModel;
  ContextModel3DBuffer m_cSaoTypeIdxSCModel;
  ContextModel3DBuffer m_cTransformSkipSCModel;
#if KLT_COMMON
  ContextModel3DBuffer m_cKLTFlagSCModel;
#endif
  ContextModel3DBuffer m_CUTransquantBypassFlagSCModel;

#if ALF_HM3_QC_REFACTOR
  Bool          m_bAlfCtrl;
  UInt          m_uiMaxAlfCtrlDepth;
  ContextModel3DBuffer m_cCUAlfCtrlFlagSCModel;
  ContextModel3DBuffer m_cALFFlagSCModel;
  ContextModel3DBuffer m_cALFUvlcSCModel;
  ContextModel3DBuffer m_cALFSvlcSCModel;
#endif


#if QC_AC_ADAPT_WDOW
public:
  TComStats* m_pcStats;
  TComStats* getStatesHandle () {return m_pcStats;}
  Void setStatesHandle ( TComStats* pcStats) {m_pcStats = pcStats ;}
  ContextModel*  getContextModel() {return m_contextModels;}
  Int getContextModelNum()         {return m_numContextModels;}
  TEncBinIf* getBinIf  ()          {return m_pcBinIf;         }
#if ALF_HM3_QC_REFACTOR
  ContextModel3DBuffer*  getAlfCtrlFlagSCModel() {return &m_cCUAlfCtrlFlagSCModel;}
  ContextModel3DBuffer*  getAlfFlagSCModel    () {return &m_cALFFlagSCModel;      }
  ContextModel3DBuffer*  getAlfUvlcSCModel    () {return &m_cALFUvlcSCModel;      }  
  ContextModel3DBuffer*  getAlfSvlcSCModel    () {return &m_cALFSvlcSCModel;      }
#endif
#endif
};

//! \}

#endif // !defined(AFX_TENCSBAC_H__DDA7CDC4_EDE3_4015_9D32_2156249C82AA__INCLUDED_)