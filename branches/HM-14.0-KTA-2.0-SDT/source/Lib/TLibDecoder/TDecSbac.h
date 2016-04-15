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

/** \file     TDecSbac.h
    \brief    SBAC decoder class (header)
*/

#ifndef __TDECSBAC__
#define __TDECSBAC__


#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include "TDecEntropy.h"
#include "TDecBinCoder.h"
#include "TLibCommon/ContextTables.h"
#include "TLibCommon/ContextModel.h"
#include "TLibCommon/ContextModel3DBuffer.h"

//! \ingroup TLibDecoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// SBAC decoder class
class TDecSbac : public TDecEntropyIf
{
public:
  TDecSbac();
  virtual ~TDecSbac();
#if QC_AC_ADAPT_WDOW
  TComStats* m_pcStats;
  TComStats* getStatesHandle () {return m_pcStats;}
  Void setStatesHandle ( TComStats* pcStats) {m_pcStats = pcStats ;}
  Int  getCtxNumber    ()                    { return m_numContextModels; }
#endif

  Void  init                      ( TDecBinIf* p )    { m_pcTDecBinIf = p; }
  Void  uninit                    (              )    { m_pcTDecBinIf = 0; }
  
  Void load                          ( TDecSbac* pScr );
  Void loadContexts                  ( TDecSbac* pScr );
  Void xCopyFrom           ( TDecSbac* pSrc );
  Void xCopyContextsFrom       ( TDecSbac* pSrc );

  Void  resetEntropy (TComSlice* pSlice );
  Void  setBitstream              ( TComInputBitstream* p  ) { m_pcBitstream = p; m_pcTDecBinIf->init( p ); }
  Void  parseVPS                  ( TComVPS* /*pcVPS*/ ) {}
  Void  parseSPS                  ( TComSPS* /*pcSPS*/ ) {}
  Void  parsePPS                  ( TComPPS* /*pcPPS*/ ) {}

#if QC_AC_ADAPT_WDOW
  Void parseCtxUpdateInfo         (TComSlice*& rpcSlice,  TComStats* apcStats )   {};
  Void xUpdateWindowSize         (SliceType eSliceType, Int uiQPIdx);  
#endif

  Void  parseSliceHeader          ( TComSlice*& /*rpcSlice*/, ParameterSetManagerDecoder* /*parameterSetManager*/) {}
  Void  parseTerminatingBit       ( UInt& ruiBit );
  Void  parseMVPIdx               ( Int& riMVPIdx          );
  Void  parseSaoMaxUvlc           ( UInt& val, UInt maxSymbol );
  Void  parseSaoMerge         ( UInt&  ruiVal   );
  Void  parseSaoTypeIdx           ( UInt&  ruiVal  );
  Void  parseSaoUflc              ( UInt uiLength, UInt& ruiVal     );
  Void parseSAOBlkParam (SAOBlkParam& saoBlkParam, Bool* sliceEnabled, Bool leftMergeAvail, Bool aboveMergeAvail);
  Void parseSaoSign(UInt& val);
private:
  Void  xReadUnarySymbol    ( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset );
  Void  xReadUnaryMaxSymbol ( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol );
  Void  xReadEpExGolomb     ( UInt& ruiSymbol, UInt uiCount );
#if QC_CTX_RESIDUALCODING
  Void  xReadGoRiceExGolomb     ( UInt &ruiSymbol, UInt &ruiGoRiceParam );
#else
  Void  xReadCoefRemainExGolomb ( UInt &rSymbol, UInt &rParam );
#endif
private:
  TComInputBitstream* m_pcBitstream;
  TDecBinIf*        m_pcTDecBinIf;
 
public:
  
  Void parseSkipFlag      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
#if ROT_TR
  Void parseROTIdx     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
#endif
#if CU_LEVEL_MPI
  Void parseMPIIdx     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
#endif
#if QC_IMV
  Void parseiMVFlag      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
#endif
#if QC_OBMC
  Void parseOBMCFlag      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
#endif
#if QC_IC
  Void parseICFlag        ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
#endif
  Void parseCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  Void parseSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  Void parseMergeFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx );
  Void parseMergeIndex    ( TComDataCU* pcCU, UInt& ruiMergeIndex );
#if QC_FRUC_MERGE
  Void parseFRUCMgrMode  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx );
#endif
#if QC_EMT
  Void parseEmtTuIdx      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  Void parseEmtCuFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRootCbf );
#endif
  Void parsePartSize      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  Void parsePredMode      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  
  Void parseIntraDirLumaAng( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  
  Void parseIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  
  Void parseInterDir      ( TComDataCU* pcCU, UInt& ruiInterDir, UInt uiAbsPartIdx );
  Void parseRefFrmIdx     ( TComDataCU* pcCU, Int& riRefFrmIdx, RefPicList eRefList );
  Void parseMvd           ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList );
  
  Void parseTransformSubdivFlag( UInt& ruiSubdivFlag, UInt uiLog2TransformBlockSize );
  Void parseQtCbf         ( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth, UInt uiDepth );
  Void parseQtRootCbf     ( UInt uiAbsPartIdx, UInt& uiQtRootCbf );
  
  Void parseDeltaQP       ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  
  Void parseIPCMInfo      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth);

  Void parseLastSignificantXY( UInt& uiPosLastX, UInt& uiPosLastY, Int width, Int height, TextType eTType, UInt uiScanIdx );
  Void parseCoeffNxN      ( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType 
#if ROT_TR
    , Bool& bCbfCU
#endif
    );
  Void parseTransformSkipFlags ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt width, UInt height, UInt uiDepth, TextType eTType);

#if KLT_COMMON
  Void parseKLTFlags      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt width, UInt height, UInt uiDepth, TextType eTType );
#endif

  Void updateContextTables( SliceType eSliceType, Int iQp );

  Void  parseScalingList ( TComScalingList* /*scalingList*/ ) {}

#if ALF_HM3_QC_REFACTOR
  Void  setAlfCtrl                ( Bool bAlfCtrl          ) { m_bAlfCtrl = bAlfCtrl;                   }
  Void  setMaxAlfCtrlDepth        ( UInt uiMaxAlfCtrlDepth ) { m_uiMaxAlfCtrlDepth = uiMaxAlfCtrlDepth; }
  Void  parseAlfFlag              ( UInt& ruiVal           );
  Void  parseAlfUvlc              ( UInt& ruiVal           );
  Void  parseAlfSvlc              ( Int&  riVal            );
  Void  parseAlfCtrlDepth         ( UInt& ruiAlfCtrlDepth  );
  Void parseAlfCtrlFlag   ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  Void parseAlfFlagNum    ( UInt& ruiVal, UInt minValue, UInt depth );
  Void parseAlfCtrlFlag   ( UInt &ruiAlfCtrlFlag );
#endif

#if INIT_PREVFRAME
  Void loadContextsFromPrev (TComStats* apcStats, SliceType eSliceType, Int iQPIdx, Bool bFromGloble, Int iQPIdxRst =-1, Bool bAfterLastISlice= false );
#endif

private:
  UInt m_uiLastDQpNonZero;
  UInt m_uiLastQp;
  
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
  Bool m_bAlfCtrl;
  UInt m_uiMaxAlfCtrlDepth;
  ContextModel3DBuffer m_cCUAlfCtrlFlagSCModel;
  ContextModel3DBuffer m_cALFFlagSCModel;
  ContextModel3DBuffer m_cALFUvlcSCModel;
  ContextModel3DBuffer m_cALFSvlcSCModel;
#endif
};

//! \}
#endif // !defined(AFX_TDECSBAC_H__CFCAAA19_8110_47F4_9A16_810C4B5499D5__INCLUDED_)