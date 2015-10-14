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

/** \file     TDecSbac.cpp
    \brief    Context-adaptive entropy decoder class
*/

#include "TDecSbac.h"

//! \ingroup TLibDecoder
//! \{

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TDecSbac::TDecSbac() 
// new structure here
: m_pcBitstream               ( 0 )
, m_pcTDecBinIf               ( NULL )
, m_numContextModels          ( 0 )
, m_cCUSplitFlagSCModel       ( 1,             1,               NUM_SPLIT_FLAG_CTX            , m_contextModels + m_numContextModels, m_numContextModels )
, m_cCUSkipFlagSCModel        ( 1,             1,               NUM_SKIP_FLAG_CTX             , m_contextModels + m_numContextModels, m_numContextModels)
#if ROT_TR
, m_cROTidxSCModel           ( 1,             1,               NUM_ROT_TR_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
#endif
#if CU_LEVEL_MPI
, m_cMPIIdxSCModel           ( 1,             1,               NUM_MPI_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
#endif
#if QC_IMV
, m_cCUiMVFlagSCModel         ( 1,             1,               NUM_IMV_FLAG_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
#endif
#if QC_OBMC
, m_cCUOBMCFlagSCModel        ( 1,             1,               NUM_OBMC_FLAG_CTX             , m_contextModels + m_numContextModels, m_numContextModels)
#endif
#if QC_IC
, m_cCUICFlagSCModel          ( 1,             1,               NUM_IC_FLAG_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
#endif
, m_cCUMergeFlagExtSCModel    ( 1,             1,               NUM_MERGE_FLAG_EXT_CTX        , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeIdxExtSCModel     ( 1,             1,               NUM_MERGE_IDX_EXT_CTX         , m_contextModels + m_numContextModels, m_numContextModels)
#if QC_FRUC_MERGE
, m_cCUFRUCMgrModeSCModel     ( 1,             1,               NUM_FRUCMGRMODE_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUFRUCMESCModel          ( 1,             1,               NUM_FRUCME_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
#endif
#if QC_EMT
, m_cEmtTuIdxSCModel          ( 1,             1,               NUM_EMT_TU_IDX_CTX            , m_contextModels + m_numContextModels, m_numContextModels)
, m_cEmtCuFlagSCModel         ( 1,             1,               NUM_EMT_CU_FLAG_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
#endif
, m_cCUPartSizeSCModel        ( 1,             1,               NUM_PART_SIZE_CTX             , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPredModeSCModel        ( 1,             1,               NUM_PRED_MODE_CTX             , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUIntraPredSCModel       ( 1,             1,               NUM_ADI_CTX                   , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUChromaPredSCModel      ( 1,             1,               NUM_CHROMA_PRED_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUDeltaQpSCModel         ( 1,             1,               NUM_DELTA_QP_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUInterDirSCModel        ( 1,             1,               NUM_INTER_DIR_CTX             , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCURefPicSCModel          ( 1,             1,               NUM_REF_NO_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMvdSCModel             ( 1,             1,               NUM_MV_RES_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUQtCbfSCModel           ( 1,             2,               NUM_QT_CBF_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUTransSubdivFlagSCModel ( 1,             1,               NUM_TRANS_SUBDIV_FLAG_CTX     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUQtRootCbfSCModel       ( 1,             1,               NUM_QT_ROOT_CBF_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSigCoeffGroupSCModel   ( 1,             2,               NUM_SIG_CG_FLAG_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSigSCModel             ( 1,             1,               NUM_SIG_FLAG_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCuCtxLastX               ( 1,             2,               NUM_CTX_LAST_FLAG_XY          , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCuCtxLastY               ( 1,             2,               NUM_CTX_LAST_FLAG_XY          , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUOneSCModel             ( 1,             1,               NUM_ONE_FLAG_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
#if !QC_CTX_RESIDUALCODING
, m_cCUAbsSCModel             ( 1,             1,               NUM_ABS_FLAG_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
#endif
, m_cMVPIdxSCModel            ( 1,             1,               NUM_MVP_IDX_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoMergeSCModel      ( 1,             1,               NUM_SAO_MERGE_FLAG_CTX   , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoTypeIdxSCModel        ( 1,             1,               NUM_SAO_TYPE_IDX_CTX          , m_contextModels + m_numContextModels, m_numContextModels)
, m_cTransformSkipSCModel     ( 1,             2,               NUM_TRANSFORMSKIP_FLAG_CTX    , m_contextModels + m_numContextModels, m_numContextModels)
#if KLT_COMMON
, m_cKLTFlagSCModel           ( 1,             2,               NUM_KLT_FLAG_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
#endif
, m_CUTransquantBypassFlagSCModel( 1,          1,               NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX, m_contextModels + m_numContextModels, m_numContextModels)
#if ALF_HM3_QC_REFACTOR
, m_bAlfCtrl                  ( false )
, m_uiMaxAlfCtrlDepth         ( 0 )
, m_cCUAlfCtrlFlagSCModel     ( 1,             1,               NUM_ALF_CTRL_FLAG_CTX         , m_contextModels + m_numContextModels, m_numContextModels)
, m_cALFFlagSCModel           ( 1,             1,               NUM_ALF_FLAG_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
, m_cALFUvlcSCModel           ( 1,             1,               NUM_ALF_UVLC_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
, m_cALFSvlcSCModel           ( 1,             1,               NUM_ALF_SVLC_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
#endif
{
  assert( m_numContextModels <= MAX_NUM_CTX_MOD );
}

TDecSbac::~TDecSbac()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecSbac::resetEntropy(TComSlice* pSlice)
{
  SliceType sliceType  = pSlice->getSliceType();
  Int       qp         = pSlice->getSliceQp();

  if (pSlice->getPPS()->getCabacInitPresentFlag() && pSlice->getCabacInitFlag())
  {
    switch (sliceType)
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE; 
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE; 
      break;
    default     :           // should not occur
      assert(0);
    }
  }

  m_cCUSplitFlagSCModel.initBuffer       ( sliceType, qp, (UChar*)INIT_SPLIT_FLAG );
  m_cCUSkipFlagSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_SKIP_FLAG );
#if ROT_TR
  m_cROTidxSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_ROT_TR_IDX );
#endif
#if CU_LEVEL_MPI
  m_cMPIIdxSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_MPIIdx_FLAG );
#endif
#if QC_IMV
  m_cCUiMVFlagSCModel.initBuffer         ( sliceType, qp, (UChar*)INIT_IMV_FLAG );
#endif
#if QC_OBMC
  m_cCUOBMCFlagSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_OBMC_FLAG );
#endif
#if QC_IC
  m_cCUICFlagSCModel.initBuffer          ( sliceType, qp, (UChar*)INIT_IC_FLAG );
#endif  
  m_cCUMergeFlagExtSCModel.initBuffer    ( sliceType, qp, (UChar*)INIT_MERGE_FLAG_EXT );
  m_cCUMergeIdxExtSCModel.initBuffer     ( sliceType, qp, (UChar*)INIT_MERGE_IDX_EXT );
#if QC_FRUC_MERGE
  m_cCUFRUCMgrModeSCModel.initBuffer     ( sliceType, qp, (UChar*)INIT_FRUCMGRMODEBIN1 );
  m_cCUFRUCMESCModel.initBuffer          ( sliceType, qp, (UChar*)INIT_FRUCMGRMODEBIN2 );
#endif
#if QC_EMT
  m_cEmtTuIdxSCModel.initBuffer          ( sliceType, qp, (UChar*)INIT_EMT_TU_IDX );
  m_cEmtCuFlagSCModel.initBuffer         ( sliceType, qp, (UChar*)INIT_EMT_CU_FLAG );
#endif
  m_cCUPartSizeSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_PART_SIZE );
  m_cCUPredModeSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_PRED_MODE );
  m_cCUIntraPredSCModel.initBuffer       ( sliceType, qp, (UChar*)INIT_INTRA_PRED_MODE );
  m_cCUChromaPredSCModel.initBuffer      ( sliceType, qp, (UChar*)INIT_CHROMA_PRED_MODE );
  m_cCUInterDirSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_INTER_DIR );
  m_cCUMvdSCModel.initBuffer             ( sliceType, qp, (UChar*)INIT_MVD );
  m_cCURefPicSCModel.initBuffer          ( sliceType, qp, (UChar*)INIT_REF_PIC );
  m_cCUDeltaQpSCModel.initBuffer         ( sliceType, qp, (UChar*)INIT_DQP );
  m_cCUQtCbfSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_QT_CBF );
  m_cCUQtRootCbfSCModel.initBuffer       ( sliceType, qp, (UChar*)INIT_QT_ROOT_CBF );
  m_cCUSigCoeffGroupSCModel.initBuffer   ( sliceType, qp, (UChar*)INIT_SIG_CG_FLAG );
  m_cCUSigSCModel.initBuffer             ( sliceType, qp, (UChar*)INIT_SIG_FLAG );
  m_cCuCtxLastX.initBuffer               ( sliceType, qp, (UChar*)INIT_LAST );
  m_cCuCtxLastY.initBuffer               ( sliceType, qp, (UChar*)INIT_LAST );
  m_cCUOneSCModel.initBuffer             ( sliceType, qp, (UChar*)INIT_ONE_FLAG );
#if !QC_CTX_RESIDUALCODING
  m_cCUAbsSCModel.initBuffer             ( sliceType, qp, (UChar*)INIT_ABS_FLAG );
#endif
  m_cMVPIdxSCModel.initBuffer            ( sliceType, qp, (UChar*)INIT_MVP_IDX );
  m_cSaoMergeSCModel.initBuffer      ( sliceType, qp, (UChar*)INIT_SAO_MERGE_FLAG );
  m_cSaoTypeIdxSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_SAO_TYPE_IDX );

  m_cCUTransSubdivFlagSCModel.initBuffer ( sliceType, qp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
  m_cTransformSkipSCModel.initBuffer     ( sliceType, qp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
#if KLT_COMMON
  m_cKLTFlagSCModel.initBuffer(sliceType, qp, (UChar*)INIT_KLT_FLAG);
#endif
  m_CUTransquantBypassFlagSCModel.initBuffer( sliceType, qp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );
#if ALF_HM3_QC_REFACTOR
  m_cCUAlfCtrlFlagSCModel.initBuffer     ( sliceType, qp, (UChar*)INIT_ALF_CTRL_FLAG );
  m_cALFFlagSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_ALF_FLAG );
  m_cALFUvlcSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_ALF_UVLC );
  m_cALFSvlcSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_ALF_SVLC );
#endif
  m_uiLastDQpNonZero  = 0;
  
  // new structure
  m_uiLastQp          = qp;
  
  m_pcTDecBinIf->start();

#if QC_AC_ADAPT_WDOW
  Int iQPIdx = pSlice->getCtxMapQPIdx();
  if( iQPIdx != -1)
  {
    xUpdateWindowSize (pSlice->getSliceType(), iQPIdx); 
  }
#endif
}

#if QC_AC_ADAPT_WDOW
Void TDecSbac::xUpdateWindowSize (SliceType eSliceType, Int uiQPIdx)
{
  Int iCtxNr = getCtxNumber();
  for(UInt i=0; i< iCtxNr; i++)
  {
    m_contextModels[i].setWindowSize(m_pcStats->m_uiCtxCodeIdx[eSliceType][uiQPIdx][i]);
  }
}
#endif
/** The function does the following: Read out terminate bit. Flush CABAC. Byte-align for next tile.
 *  Intialize CABAC states. Start CABAC.
 */
Void TDecSbac::updateContextTables( SliceType eSliceType, Int iQp )
{
  UInt uiBit;
  m_pcTDecBinIf->decodeBinTrm(uiBit);
  assert(uiBit); // end_of_sub_stream_one_bit must be equal to 1
  m_pcTDecBinIf->finish();  
  m_pcBitstream->readOutTrailingBits();
  m_cCUSplitFlagSCModel.initBuffer       ( eSliceType, iQp, (UChar*)INIT_SPLIT_FLAG );
  m_cCUSkipFlagSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_SKIP_FLAG );
#if ROT_TR
  m_cROTidxSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_ROT_TR_IDX );
#endif
#if CU_LEVEL_MPI
  m_cMPIIdxSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_MPIIdx_FLAG );
#endif
#if QC_IMV
  m_cCUiMVFlagSCModel.initBuffer         ( eSliceType, iQp, (UChar*)INIT_IMV_FLAG );
#endif
#if QC_OBMC
  m_cCUOBMCFlagSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_OBMC_FLAG );
#endif
#if QC_IC
  m_cCUICFlagSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_IC_FLAG );
#endif
  m_cCUMergeFlagExtSCModel.initBuffer    ( eSliceType, iQp, (UChar*)INIT_MERGE_FLAG_EXT );
  m_cCUMergeIdxExtSCModel.initBuffer     ( eSliceType, iQp, (UChar*)INIT_MERGE_IDX_EXT );
#if QC_FRUC_MERGE
  m_cCUFRUCMgrModeSCModel.initBuffer     ( eSliceType, iQp, (UChar*)INIT_FRUCMGRMODEBIN1 );
  m_cCUFRUCMESCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_FRUCMGRMODEBIN2 );
#endif
#if QC_EMT
  m_cEmtTuIdxSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_EMT_TU_IDX );
  m_cEmtCuFlagSCModel.initBuffer         ( eSliceType, iQp, (UChar*)INIT_EMT_CU_FLAG );
#endif
  m_cCUPartSizeSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_PART_SIZE );
  m_cCUPredModeSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_PRED_MODE );
  m_cCUIntraPredSCModel.initBuffer       ( eSliceType, iQp, (UChar*)INIT_INTRA_PRED_MODE );
  m_cCUChromaPredSCModel.initBuffer      ( eSliceType, iQp, (UChar*)INIT_CHROMA_PRED_MODE );
  m_cCUInterDirSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_INTER_DIR );
  m_cCUMvdSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_MVD );
  m_cCURefPicSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_REF_PIC );
  m_cCUDeltaQpSCModel.initBuffer         ( eSliceType, iQp, (UChar*)INIT_DQP );
  m_cCUQtCbfSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_QT_CBF );
  m_cCUQtRootCbfSCModel.initBuffer       ( eSliceType, iQp, (UChar*)INIT_QT_ROOT_CBF );
  m_cCUSigCoeffGroupSCModel.initBuffer   ( eSliceType, iQp, (UChar*)INIT_SIG_CG_FLAG );
  m_cCUSigSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_SIG_FLAG );
  m_cCuCtxLastX.initBuffer               ( eSliceType, iQp, (UChar*)INIT_LAST );
  m_cCuCtxLastY.initBuffer               ( eSliceType, iQp, (UChar*)INIT_LAST );
  m_cCUOneSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_ONE_FLAG );
#if !QC_CTX_RESIDUALCODING
  m_cCUAbsSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_ABS_FLAG );
#endif
  m_cMVPIdxSCModel.initBuffer            ( eSliceType, iQp, (UChar*)INIT_MVP_IDX );
  m_cSaoMergeSCModel.initBuffer      ( eSliceType, iQp, (UChar*)INIT_SAO_MERGE_FLAG );
  m_cSaoTypeIdxSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_SAO_TYPE_IDX );
  m_cCUTransSubdivFlagSCModel.initBuffer ( eSliceType, iQp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
  m_cTransformSkipSCModel.initBuffer     ( eSliceType, iQp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
#if KLT_COMMON
  m_cKLTFlagSCModel.initBuffer(eSliceType, iQp, (UChar*)INIT_KLT_FLAG);
#endif
  m_CUTransquantBypassFlagSCModel.initBuffer( eSliceType, iQp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );
  m_pcTDecBinIf->start();
}

Void TDecSbac::parseTerminatingBit( UInt& ruiBit )
{
  m_pcTDecBinIf->decodeBinTrm( ruiBit );
  if ( ruiBit )
  {
    m_pcTDecBinIf->finish();
  }
}


Void TDecSbac::xReadUnaryMaxSymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol )
{
  if (uiMaxSymbol == 0)
  {
    ruiSymbol = 0;
    return;
  }
  
  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] );
  
  if( ruiSymbol == 0 || uiMaxSymbol == 1 )
  {
    return;
  }
  
  UInt uiSymbol = 0;
  UInt uiCont;
  
  do
  {
    m_pcTDecBinIf->decodeBin( uiCont, pcSCModel[ iOffset ] );
    uiSymbol++;
  }
  while( uiCont && ( uiSymbol < uiMaxSymbol - 1 ) );
  
  if( uiCont && ( uiSymbol == uiMaxSymbol - 1 ) )
  {
    uiSymbol++;
  }
  
  ruiSymbol = uiSymbol;
}

Void TDecSbac::xReadEpExGolomb( UInt& ruiSymbol, UInt uiCount )
{
  UInt uiSymbol = 0;
  UInt uiBit = 1;
  
  while( uiBit )
  {
    m_pcTDecBinIf->decodeBinEP( uiBit );
    uiSymbol += uiBit << uiCount++;
  }
  
  if ( --uiCount )
  {
    UInt bins;
    m_pcTDecBinIf->decodeBinsEP( bins, uiCount );
    uiSymbol += bins;
  }
  
  ruiSymbol = uiSymbol;
}

Void TDecSbac::xReadUnarySymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset )
{
  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] );
  
  if( !ruiSymbol )
  {
    return;
  }
  
  UInt uiSymbol = 0;
  UInt uiCont;
  
  do
  {
    m_pcTDecBinIf->decodeBin( uiCont, pcSCModel[ iOffset ] );
    uiSymbol++;
  }
  while( uiCont );
  
  ruiSymbol = uiSymbol;
}

#if QC_CTX_RESIDUALCODING
/** Parsing of coeff_abs_level_minus3
 * \param ruiSymbol reference to coeff_abs_level_minus3
 * \param ruiGoRiceParam reference to Rice parameter
 * \returns Void
 */
Void TDecSbac::xReadGoRiceExGolomb( UInt &ruiSymbol, UInt &ruiGoRiceParam )
{
  Bool bExGolomb    = false;
  UInt uiCodeWord   = 0;
  UInt uiQuotient   = 0;
  UInt uiRemainder  = 0;
  UInt uiMaxVlc     = g_auiGoRiceRange[ ruiGoRiceParam ];
  UInt uiMaxPreLen  = g_auiGoRicePrefixLen[ ruiGoRiceParam ];

  do
  {
    uiQuotient++;
    m_pcTDecBinIf->decodeBinEP( uiCodeWord );
  }
  while( uiCodeWord && uiQuotient < uiMaxPreLen );

  uiCodeWord  = 1 - uiCodeWord;
  uiQuotient -= uiCodeWord;

  if ( ruiGoRiceParam > 0 )
  {
    m_pcTDecBinIf->decodeBinsEP( uiRemainder, ruiGoRiceParam );    
  }

  ruiSymbol      = uiRemainder + ( uiQuotient << ruiGoRiceParam );
  bExGolomb      = ruiSymbol == ( uiMaxVlc + 1 );

  if( bExGolomb )
  {
    xReadEpExGolomb( uiCodeWord, 0 );
    ruiSymbol += uiCodeWord;
  }
  return;
}
#else
/** Parsing of coeff_abs_level_remaing
 * \param ruiSymbol reference to coeff_abs_level_remaing
 * \param ruiParam reference to parameter
 * \returns Void
 */
Void TDecSbac::xReadCoefRemainExGolomb ( UInt &rSymbol, UInt &rParam )
{

  UInt prefix   = 0;
  UInt codeWord = 0;
  do
  {
    prefix++;
    m_pcTDecBinIf->decodeBinEP( codeWord );
  }
  while( codeWord);
  codeWord  = 1 - codeWord;
  prefix -= codeWord;
  codeWord=0;
  if (prefix < COEF_REMAIN_BIN_REDUCTION )
  {
    m_pcTDecBinIf->decodeBinsEP(codeWord,rParam);
    rSymbol = (prefix<<rParam) + codeWord;
  }
  else
  {
    m_pcTDecBinIf->decodeBinsEP(codeWord,prefix-COEF_REMAIN_BIN_REDUCTION+rParam);
    rSymbol = (((1<<(prefix-COEF_REMAIN_BIN_REDUCTION))+COEF_REMAIN_BIN_REDUCTION-1)<<rParam)+codeWord;
  }
}
#endif

/** Parse I_PCM information. 
 * \param pcCU
 * \param uiAbsPartIdx 
 * \param uiDepth
 * \returns Void
 *
 * If I_PCM flag indicates that the CU is I_PCM, parse its PCM alignment bits and codes. 
 */
Void TDecSbac::parseIPCMInfo ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

  m_pcTDecBinIf->decodeBinTrm(uiSymbol);

  if (uiSymbol)
  {
    Bool bIpcmFlag = true;

    pcCU->setPartSizeSubParts  ( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts      ( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );
    pcCU->setTrIdxSubParts     ( 0, uiAbsPartIdx, uiDepth );
    pcCU->setIPCMFlagSubParts  ( bIpcmFlag, uiAbsPartIdx, uiDepth );

    UInt uiMinCoeffSize = pcCU->getPic()->getMinCUWidth()*pcCU->getPic()->getMinCUHeight();
    UInt uiLumaOffset   = uiMinCoeffSize*uiAbsPartIdx;
    UInt uiChromaOffset = uiLumaOffset>>2;

    Pel* piPCMSample;
    UInt uiWidth;
    UInt uiHeight;
    UInt uiSampleBits;
    UInt uiX, uiY;

    piPCMSample = pcCU->getPCMSampleY() + uiLumaOffset;
    uiWidth = pcCU->getWidth(uiAbsPartIdx);
    uiHeight = pcCU->getHeight(uiAbsPartIdx);
    uiSampleBits = pcCU->getSlice()->getSPS()->getPCMBitDepthLuma();

    for(uiY = 0; uiY < uiHeight; uiY++)
    {
      for(uiX = 0; uiX < uiWidth; uiX++)
      {
        UInt uiSample;
        m_pcTDecBinIf->xReadPCMCode(uiSampleBits, uiSample);
        piPCMSample[uiX] = uiSample;
      }
      piPCMSample += uiWidth;
    }

    piPCMSample = pcCU->getPCMSampleCb() + uiChromaOffset;
    uiWidth = pcCU->getWidth(uiAbsPartIdx)/2;
    uiHeight = pcCU->getHeight(uiAbsPartIdx)/2;
    uiSampleBits = pcCU->getSlice()->getSPS()->getPCMBitDepthChroma();

    for(uiY = 0; uiY < uiHeight; uiY++)
    {
      for(uiX = 0; uiX < uiWidth; uiX++)
      {
        UInt uiSample;
        m_pcTDecBinIf->xReadPCMCode(uiSampleBits, uiSample);
        piPCMSample[uiX] = uiSample;
      }
      piPCMSample += uiWidth;
    }

    piPCMSample = pcCU->getPCMSampleCr() + uiChromaOffset;
    uiWidth = pcCU->getWidth(uiAbsPartIdx)/2;
    uiHeight = pcCU->getHeight(uiAbsPartIdx)/2;
    uiSampleBits = pcCU->getSlice()->getSPS()->getPCMBitDepthChroma();

    for(uiY = 0; uiY < uiHeight; uiY++)
    {
      for(uiX = 0; uiX < uiWidth; uiX++)
      {
        UInt uiSample;
        m_pcTDecBinIf->xReadPCMCode(uiSampleBits, uiSample);
        piPCMSample[uiX] = uiSample;
      }
      piPCMSample += uiWidth;
    }

    m_pcTDecBinIf->start();
  }
}

Void TDecSbac::parseCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_CUTransquantBypassFlagSCModel.get( 0, 0, 0 ) );
  pcCU->setCUTransquantBypassSubParts(uiSymbol ? true : false, uiAbsPartIdx, uiDepth);
}

/** parse skip flag
 * \param pcCU
 * \param uiAbsPartIdx 
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parseSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( pcCU->getSlice()->isIntra() )
  {
    return;
  }
  
  UInt uiSymbol = 0;
  UInt uiCtxSkip = pcCU->getCtxSkipFlag( uiAbsPartIdx );
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUSkipFlagSCModel.get( 0, 0, uiCtxSkip ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tSkipFlag" );
  DTRACE_CABAC_T( "\tuiCtxSkip: ");
  DTRACE_CABAC_V( uiCtxSkip );
  DTRACE_CABAC_T( "\tuiSymbol: ");
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\n");
  
  if( uiSymbol )
  {
    pcCU->setSkipFlagSubParts( true,        uiAbsPartIdx, uiDepth );
    pcCU->setPredModeSubParts( MODE_INTER,  uiAbsPartIdx, uiDepth );
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );
    pcCU->setMergeFlagSubParts( true , uiAbsPartIdx, 0, uiDepth );
  }
}

#if QC_IMV
Void TDecSbac::parseiMVFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( pcCU->getSlice()->isIntra() )
  {
    return;
  }
  else if( pcCU->getMergeFlag( uiAbsPartIdx ) && pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCtxiMV = pcCU->getCtxiMVFlag( uiAbsPartIdx );
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUiMVFlagSCModel.get( 0, 0, uiCtxiMV ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tiMVFlag" );
  DTRACE_CABAC_T( "\tuiCtxiMV: ");
  DTRACE_CABAC_V( uiCtxiMV );
  DTRACE_CABAC_T( "\tuiSymbol: ");
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\n");

  if( uiSymbol )
  {
    pcCU->setiMVFlagSubParts( true,        uiAbsPartIdx, uiDepth );
  }
}
#endif

#if QC_OBMC
Void TDecSbac::parseOBMCFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  pcCU->setOBMCFlagSubParts( true, uiAbsPartIdx, uiDepth );

  if ( !pcCU->getSlice()->getSPS()->getOBMC() || !pcCU->isOBMCFlagCoded( uiAbsPartIdx ) )
  {
    return;
  }

  UInt uiSymbol = 0;

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUOBMCFlagSCModel.get( 0, 0, 0 ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tOBMCFlag" );
  DTRACE_CABAC_T( "\tuiSymbol: ");
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\n");

  pcCU->setOBMCFlagSubParts( uiSymbol ? true : false, uiAbsPartIdx, uiDepth );
}
#endif

#if QC_IC
/** parse illumination compensation flag
 * \param pcCU
 * \param uiAbsPartIdx 
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parseICFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{ 
  UInt uiSymbol = 0;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUICFlagSCModel.get( 0, 0, 0 ) );
 
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tICFlag" );
  DTRACE_CABAC_T( "\tuiSymbol: ");
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\n");

  pcCU->setICFlagSubParts( uiSymbol ? true : false , uiAbsPartIdx, uiDepth );
}
#endif
#if ROT_TR
Void TDecSbac::parseROTIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{ 
  Int iNumberOfPassesROT = 1;
  if( pcCU->isIntra(uiAbsPartIdx)
#if CU_LEVEL_MPI
    && pcCU->getMPIIdx(uiAbsPartIdx) ==0
#endif    
    )  iNumberOfPassesROT = 4;
  if (iNumberOfPassesROT>1) // for only 1 pass no signaling is needed 
  {
      UInt uiSymbol0 = 0;
      UInt uiSymbol1 = 0;
      m_pcTDecBinIf->decodeBin( uiSymbol0, m_cROTidxSCModel.get(0,0, uiDepth ) );
      m_pcTDecBinIf->decodeBin( uiSymbol1, m_cROTidxSCModel.get(0,0, uiDepth ) );
      pcCU->setROTIdxSubParts( (uiSymbol0<<1) +uiSymbol1, uiAbsPartIdx,  uiDepth ); 
  }
  else
  {
       pcCU->setROTIdxSubParts( 0, uiAbsPartIdx,  uiDepth ); 
  }
}
#endif
#if CU_LEVEL_MPI
Void TDecSbac::parseMPIIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{  
  if (pcCU->getPredictionMode(uiAbsPartIdx)==MODE_INTER)
  {
     pcCU->setMPIIdxSubParts( 0, uiAbsPartIdx,  uiDepth ); 
     return;
  }
  Int iNumberOfPassesMPI = 1;
    if( pcCU->getSlice()->getSliceType() == I_SLICE) iNumberOfPassesMPI = MPI_DICT_SIZE_INTRA;
    else iNumberOfPassesMPI = MPI_DICT_SIZE_INTER;
  if (iNumberOfPassesMPI>1) // for only 1 pass no signaling is needed 
  {
    if (iNumberOfPassesMPI>2)  // 3 or 4
    {
      UInt uiSymbol0 = 0;
      UInt uiSymbol1 = 0;
      m_pcTDecBinIf->decodeBin( uiSymbol0, m_cMPIIdxSCModel.get(0,0, 0 ) );
      m_pcTDecBinIf->decodeBin( uiSymbol1, m_cMPIIdxSCModel.get(0,0, 1 ) );
      pcCU->setMPIIdxSubParts( (uiSymbol0<<1) +uiSymbol1, uiAbsPartIdx,  uiDepth );
    }
    else  //iNumberOfPassesMPI==2
    {
      UInt uiSymbol = 0;
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cMPIIdxSCModel.get(0,0, 0 ) );
      pcCU->setMPIIdxSubParts( uiSymbol, uiAbsPartIdx,  uiDepth );  
    }
  }
  else
  {
       pcCU->setMPIIdxSubParts( 0, uiAbsPartIdx,  uiDepth ); 
  }
}
#endif
/** parse merge flag
 * \param pcCU
 * \param uiAbsPartIdx 
 * \param uiDepth
 * \param uiPUIdx
 * \returns Void
 */
Void TDecSbac::parseMergeFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, *m_cCUMergeFlagExtSCModel.get( 0 ) );
  pcCU->setMergeFlagSubParts( uiSymbol ? true : false, uiAbsPartIdx, uiPUIdx, uiDepth );

  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tMergeFlag: " );
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\tAddress: " );
  DTRACE_CABAC_V( pcCU->getAddr() );
  DTRACE_CABAC_T( "\tuiAbsPartIdx: " );
  DTRACE_CABAC_V( uiAbsPartIdx );
  DTRACE_CABAC_T( "\n" );
}

Void TDecSbac::parseMergeIndex ( TComDataCU* pcCU, UInt& ruiMergeIndex )
{
  UInt uiUnaryIdx = 0;
  UInt uiNumCand = pcCU->getSlice()->getMaxNumMergeCand();
  if ( uiNumCand > 1 )
  {
    for( ; uiUnaryIdx < uiNumCand - 1; ++uiUnaryIdx )
    {
      UInt uiSymbol = 0;
#if GEN_MRG_IMPROVEMENT
#if QC_SUB_PU_TMVP_EXT
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMergeIdxExtSCModel.get( 0, 0, (uiUnaryIdx>NUM_MERGE_IDX_EXT_CTX-1? NUM_MERGE_IDX_EXT_CTX-1:uiUnaryIdx) ) );
#else
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMergeIdxExtSCModel.get( 0, 0, (uiUnaryIdx>3?3:uiUnaryIdx) ) );
#endif
#else
      if ( uiUnaryIdx==0 )
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMergeIdxExtSCModel.get( 0, 0, 0 ) );
      }
      else
      {
        m_pcTDecBinIf->decodeBinEP( uiSymbol );
      }
#endif
      if( uiSymbol == 0 )
      {
        break;
      }
    }
  }
  ruiMergeIndex = uiUnaryIdx;

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseMergeIndex()" )
  DTRACE_CABAC_T( "\tuiMRGIdx= " )
  DTRACE_CABAC_V( ruiMergeIndex )
  DTRACE_CABAC_T( "\n" )
}



#if QC_FRUC_MERGE
Void TDecSbac::parseFRUCMgrMode ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx )
{
  if( !pcCU->getSlice()->getSPS()->getUseFRUCMgrMode() )
    return;

  UChar uhFRUCMode = QC_FRUC_MERGE_OFF;
  UInt uiFirstBin = 0;
  UInt uiSecondBin = 0;
  m_pcTDecBinIf->decodeBin( uiFirstBin, m_cCUFRUCMgrModeSCModel.get( 0 , 0 , pcCU->getCtxFRUCMgrMode( uiAbsPartIdx ) ) );
  if( uiFirstBin )
  {
    if( pcCU->getSlice()->isInterP() )
    {
      uhFRUCMode = QC_FRUC_MERGE_TEMPLATE;
    }
    else
    {
      m_pcTDecBinIf->decodeBin( uiSecondBin , m_cCUFRUCMESCModel.get( 0 , 0 , pcCU->getCtxFRUCME( uiAbsPartIdx ) ) );
      uhFRUCMode = uiSecondBin ? QC_FRUC_MERGE_BILATERALMV : QC_FRUC_MERGE_TEMPLATE;
    }
  }

  pcCU->setFRUCMgrModeSubParts( uhFRUCMode , uiAbsPartIdx, uiPUIdx, uiDepth );
}
#endif

Void TDecSbac::parseMVPIdx      ( Int& riMVPIdx )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol(uiSymbol, m_cMVPIdxSCModel.get(0), 1, AMVP_MAX_NUM_CANDS-1);
  riMVPIdx = uiSymbol;
}

Void TDecSbac::parseSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
  {
    pcCU->setDepthSubParts( uiDepth, uiAbsPartIdx );
    return;
  }
  
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUSplitFlagSCModel.get( 0, 0, pcCU->getCtxSplitFlag( uiAbsPartIdx, uiDepth ) ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tSplitFlag\n" )
  pcCU->setDepthSubParts( uiDepth + uiSymbol, uiAbsPartIdx );
  
  return;
}

#if QC_EMT
/** parse TU-level transform index
 * \param pcCU
 * \param uiAbsPartIdx 
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parseEmtTuIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UChar trMode = 0;

#if QC_EMT_INTRA
  if ( pcCU->isIntra( uiAbsPartIdx ) && pcCU->getWidth(uiAbsPartIdx) <= QC_EMT_INTRA_MAX_CU  )
  {
    UInt uiSymbol1 = 0, uiSymbol2 = 0;
    m_pcTDecBinIf->decodeBin( uiSymbol1, m_cEmtTuIdxSCModel.get(0, 0, 0) );
    m_pcTDecBinIf->decodeBin( uiSymbol2, m_cEmtTuIdxSCModel.get(0, 0, 1) );
    trMode = (uiSymbol2 << 1) | uiSymbol1; 
  }
#endif

#if QC_EMT_INTER
  if( !pcCU->isIntra( uiAbsPartIdx ) && pcCU->getWidth(uiAbsPartIdx) <= QC_EMT_INTER_MAX_CU )
  {
    UInt uiSymbol1 = 0, uiSymbol2 = 0;
    m_pcTDecBinIf->decodeBin( uiSymbol1, m_cEmtTuIdxSCModel.get(0, 0, 2) );
    m_pcTDecBinIf->decodeBin( uiSymbol2, m_cEmtTuIdxSCModel.get(0, 0, 3) );
    trMode = (uiSymbol2 << 1) | uiSymbol1; 
  }
#endif

  pcCU->setEmtTuIdxSubParts( trMode, uiAbsPartIdx, uiDepth );
}

/** parse CU-level EMT enable flag
 * \param pcCU
 * \param uiAbsPartIdx 
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parseEmtCuFlag  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bParseCuFlag )
{
  assert( uiDepth < NUM_EMT_CU_FLAG_CTX );
  pcCU->setEmtCuFlagSubParts( 0, uiAbsPartIdx, uiDepth );
  pcCU->setEmtTuIdxSubParts( DCT2_EMT, uiAbsPartIdx, uiDepth );

#if QC_EMT_INTRA
  if ( pcCU->isIntra( uiAbsPartIdx ) && bParseCuFlag && pcCU->getWidth(uiAbsPartIdx) <= QC_EMT_INTRA_MAX_CU && pcCU->getSlice()->getSPS()->getUseIntraEMT() )
  {
    UInt tuOptTrFlag = 0;
    m_pcTDecBinIf->decodeBin( tuOptTrFlag, m_cEmtCuFlagSCModel.get(0, 0, uiDepth) );
    pcCU->setEmtCuFlagSubParts( tuOptTrFlag, uiAbsPartIdx, uiDepth );
  }
#endif

#if QC_EMT_INTER
  if( !pcCU->isIntra( uiAbsPartIdx ) && bParseCuFlag && pcCU->getWidth(uiAbsPartIdx) <= QC_EMT_INTER_MAX_CU && pcCU->getSlice()->getSPS()->getUseInterEMT() )
  {
    UInt tuOptTrFlag = 0;
  m_pcTDecBinIf->decodeBin( tuOptTrFlag, m_cEmtCuFlagSCModel.get(0, 0, uiDepth));
    pcCU->setEmtCuFlagSubParts( tuOptTrFlag, uiAbsPartIdx, uiDepth );
  }
#endif
}
#endif

/** parse partition size
 * \param pcCU
 * \param uiAbsPartIdx 
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parsePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol, uiMode = 0;
  PartSize eMode;
  
  if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    uiSymbol = 1;
    if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 0) );
    }
    eMode = uiSymbol ? SIZE_2Nx2N : SIZE_NxN;
    UInt uiTrLevel = 0;    
    UInt uiWidthInBit  = g_aucConvertToBit[pcCU->getWidth(uiAbsPartIdx)]+2;
    UInt uiTrSizeInBit = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxTrSize()]+2;
    uiTrLevel          = uiWidthInBit >= uiTrSizeInBit ? uiWidthInBit - uiTrSizeInBit : 0;
    if( eMode == SIZE_NxN )
    {
      pcCU->setTrIdxSubParts( 1+uiTrLevel, uiAbsPartIdx, uiDepth );
    }
    else
    {
      pcCU->setTrIdxSubParts( uiTrLevel, uiAbsPartIdx, uiDepth );
    }
  }
  else
  {
    UInt uiMaxNumBits = 2;
#if QC_HEVC_MOTION_CONSTRAINT_REMOVAL && !QC_DISABLE_4X4_PU
    if ( (pcCU->getSlice()->getSPS()->getAtmvpEnableFlag() && (uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth)) ||
        (!pcCU->getSlice()->getSPS()->getAtmvpEnableFlag() && (uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && !( (g_uiMaxCUWidth>>uiDepth) == 8 && (g_uiMaxCUHeight>>uiDepth) == 8 ) ) ))
#else
    if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && !( (g_uiMaxCUWidth>>uiDepth) == 8 && (g_uiMaxCUHeight>>uiDepth) == 8 ) )
#endif
    {
      uiMaxNumBits ++;
    }
    for ( UInt ui = 0; ui < uiMaxNumBits; ui++ )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, ui) );
      if ( uiSymbol )
      {
        break;
      }
      uiMode++;
    }
    eMode = (PartSize) uiMode;
    if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
    {
      if (eMode == SIZE_2NxN)
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 3 ));
        if (uiSymbol == 0)
        {
          m_pcTDecBinIf->decodeBinEP(uiSymbol);
          eMode = (uiSymbol == 0? SIZE_2NxnU : SIZE_2NxnD);
        }
      }
      else if (eMode == SIZE_Nx2N)
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 3 ));
        if (uiSymbol == 0)
        {
          m_pcTDecBinIf->decodeBinEP(uiSymbol);
          eMode = (uiSymbol == 0? SIZE_nLx2N : SIZE_nRx2N);
        }
      }
    }
  }
  pcCU->setPartSizeSubParts( eMode, uiAbsPartIdx, uiDepth );
  pcCU->setSizeSubParts( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );
}

/** parse prediction mode
 * \param pcCU
 * \param uiAbsPartIdx 
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parsePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( pcCU->getSlice()->isIntra() )
  {
    pcCU->setPredModeSubParts( MODE_INTRA, uiAbsPartIdx, uiDepth );
    return;
  }
  
  UInt uiSymbol;
  Int  iPredMode = MODE_INTER;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPredModeSCModel.get( 0, 0, 0 ) );
  iPredMode += uiSymbol;
  pcCU->setPredModeSubParts( (PredMode)iPredMode, uiAbsPartIdx, uiDepth );
}

Void TDecSbac::parseIntraDirLumaAng  ( TComDataCU* pcCU, UInt absPartIdx, UInt depth )
{
  PartSize mode = pcCU->getPartitionSize( absPartIdx );
  UInt partNum = mode==SIZE_NxN?4:1;
  UInt partOffset = ( pcCU->getPic()->getNumPartInCU() >> ( pcCU->getDepth(absPartIdx) << 1 ) ) >> 2;
  UInt mpmPred[4],symbol;
  Int j,intraPredMode;    

#if QC_USE_65ANG_MODES
  const UInt uiContextMPM0[4] = { 2, 3, 1, 2 };
  const UInt uiContextMPM1[4] = { 4, 5, 5, 6 };
  const UInt uiContextMPM2[4] = { 7, 7, 8, 7 };
  UInt uiWidth = pcCU->getWidth(absPartIdx)>>(mode==SIZE_2Nx2N ? 0 : 1);
  Bool bUseExtIntraAngModes = pcCU->getUseExtIntraAngModes(uiWidth);
#endif

  if (mode==SIZE_NxN)
  {
    depth++;
  }
  for (j=0;j<partNum;j++)
  {
    m_pcTDecBinIf->decodeBin( symbol, m_cCUIntraPredSCModel.get( 0, 0, 0) );
    mpmPred[j] = symbol;
  }
  for (j=0;j<partNum;j++)
  {
#if QC_USE_65ANG_MODES
    Int preds[6] = {-1, -1, -1, -1, -1, -1};
    Int iLeftAboveCase=0;
#else
    Int preds[3] = {-1, -1, -1};
#endif
    Int predNum = pcCU->getIntraDirLumaPredictor(absPartIdx+partOffset*j, preds
#if QC_USE_65ANG_MODES
      , iLeftAboveCase
#endif
      );  
    if (mpmPred[j])
    {
#if QC_USE_65ANG_MODES
      if(bUseExtIntraAngModes)
        m_pcTDecBinIf->decodeBin( symbol, m_cCUIntraPredSCModel.get( 0, 0, uiContextMPM0[iLeftAboveCase]) );
      else
#endif
      m_pcTDecBinIf->decodeBinEP( symbol );
      if (symbol)
      {
#if QC_USE_65ANG_MODES
        if(bUseExtIntraAngModes)
          m_pcTDecBinIf->decodeBin( symbol, m_cCUIntraPredSCModel.get( 0, 0, uiContextMPM1[iLeftAboveCase]) );
        else
#endif
        m_pcTDecBinIf->decodeBinEP( symbol );
#if QC_USE_65ANG_MODES
        if( symbol && bUseExtIntraAngModes )
        {
          m_pcTDecBinIf->decodeBin( symbol, m_cCUIntraPredSCModel.get( 0, 0, uiContextMPM2[iLeftAboveCase]) );
          if( symbol )
          {
            m_pcTDecBinIf->decodeBinEP( symbol );
            if( symbol )
            {
              m_pcTDecBinIf->decodeBinEP( symbol );
              symbol++;
            }
            symbol++;
          }
          symbol++;
        }
#endif
        symbol++;
      }
      intraPredMode = preds[symbol];
    }
    else
    {
#if QC_USE_65ANG_MODES
      if(bUseExtIntraAngModes)
      {
        const UInt uiNumIntraModes = NUM_INTRA_MODE;
        const UInt shift = 6;
        m_pcTDecBinIf->decodeBinsEP( symbol, shift-2 );
        symbol <<= 2;
        if( symbol<(uiNumIntraModes-8) )
        {
          UInt symbol0;
          m_pcTDecBinIf->decodeBinsEP( symbol0, 2 );
          symbol += symbol0;
        }
      }
      else
#endif
      m_pcTDecBinIf->decodeBinsEP( symbol, 5 );
      intraPredMode = symbol;

      //postponed sorting of MPMs (only in remaining branch)
#if QC_USE_65ANG_MODES
      std::sort(preds, preds+(bUseExtIntraAngModes?6:3));
#else
      if (preds[0] > preds[1])
      { 
        std::swap(preds[0], preds[1]); 
      }
      if (preds[0] > preds[2])
      {
        std::swap(preds[0], preds[2]);
      }
      if (preds[1] > preds[2])
      {
        std::swap(preds[1], preds[2]);
      }
#endif
      for ( Int i = 0; i < predNum; i++ )
      {
#if QC_USE_65ANG_MODES
        if(!bUseExtIntraAngModes)
        {
          preds[i] = MAP67TO35(preds[i]);
        }
#endif
        intraPredMode += ( intraPredMode >= preds[i] );
      }
#if QC_USE_65ANG_MODES
      if( !bUseExtIntraAngModes )
      {
        intraPredMode = MAP35TO67(intraPredMode);
      }
#endif
    }
    pcCU->setLumaIntraDirSubParts( (UChar)intraPredMode, absPartIdx+partOffset*j, depth );
  }
}

Void TDecSbac::parseIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUChromaPredSCModel.get( 0, 0, 0 ) );

  if( uiSymbol == 0 )
  {
    uiSymbol = DM_CHROMA_IDX;
  } 
  else 
  {
#if QC_LMCHROMA
    if( pcCU->getSlice()->getSPS()->getUseLMChroma() )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUChromaPredSCModel.get( 0, 0, 1 ) );
    }
    else
    {
      uiSymbol = 1;
    }

    if( uiSymbol == 0 )
    {
      uiSymbol = LM_CHROMA_IDX;
    } 
    else
#endif
    {
      UInt uiIPredMode;
      m_pcTDecBinIf->decodeBinsEP( uiIPredMode, 2 );
      UInt uiAllowedChromaDir[ NUM_CHROMA_MODE ];
      pcCU->getAllowedChromaDir( uiAbsPartIdx, uiAllowedChromaDir );
      uiSymbol = uiAllowedChromaDir[ uiIPredMode ];
    }
  }
  pcCU->setChromIntraDirSubParts( uiSymbol, uiAbsPartIdx, uiDepth );
  return;
}

Void TDecSbac::parseInterDir( TComDataCU* pcCU, UInt& ruiInterDir, UInt uiAbsPartIdx )
{
  UInt uiSymbol;
  const UInt uiCtx = pcCU->getCtxInterDir( uiAbsPartIdx );
  ContextModel *pCtx = m_cCUInterDirSCModel.get( 0 );
  uiSymbol = 0;
#if QC_HEVC_MOTION_CONSTRAINT_REMOVAL
  if (pcCU->getSlice()->getSPS()->getAtmvpEnableFlag() || pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2Nx2N || pcCU->getHeight(uiAbsPartIdx) != 8 )
#else
  if (pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2Nx2N || pcCU->getHeight(uiAbsPartIdx) != 8 )
#endif
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, *( pCtx + uiCtx ) );
  }

  if( uiSymbol )
  {
    uiSymbol = 2;
  }
  else
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, *( pCtx + 4 ) );
    assert(uiSymbol == 0 || uiSymbol == 1);
  }

  uiSymbol++;
  ruiInterDir = uiSymbol;
  return;
}

Void TDecSbac::parseRefFrmIdx( TComDataCU* pcCU, Int& riRefFrmIdx, RefPicList eRefList )
{
  UInt uiSymbol;
  {
    ContextModel *pCtx = m_cCURefPicSCModel.get( 0 );
    m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx );

    if( uiSymbol )
    {
      UInt uiRefNum = pcCU->getSlice()->getNumRefIdx( eRefList ) - 2;
      pCtx++;
      UInt ui;
      for( ui = 0; ui < uiRefNum; ++ui )
      {
        if( ui == 0 )
        {
          m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx );
        }
        else
        {
          m_pcTDecBinIf->decodeBinEP( uiSymbol );
        }
        if( uiSymbol == 0 )
        {
          break;
        }
      }
      uiSymbol = ui + 1;
    }
    riRefFrmIdx = uiSymbol;
  }

  return;
}

Void TDecSbac::parseMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{
  UInt uiSymbol;
  UInt uiHorAbs;
  UInt uiVerAbs;
  UInt uiHorSign = 0;
  UInt uiVerSign = 0;
  ContextModel *pCtx = m_cCUMvdSCModel.get( 0 );

  if(pcCU->getSlice()->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pcCU->getInterDir(uiAbsPartIdx)==3)
  {
    uiHorAbs=0;
    uiVerAbs=0;
  }
  else
  {
    m_pcTDecBinIf->decodeBin( uiHorAbs, *pCtx );
    m_pcTDecBinIf->decodeBin( uiVerAbs, *pCtx );

    const Bool bHorAbsGr0 = uiHorAbs != 0;
    const Bool bVerAbsGr0 = uiVerAbs != 0;
    pCtx++;

    if( bHorAbsGr0 )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx );
      uiHorAbs += uiSymbol;
    }

    if( bVerAbsGr0 )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx );
      uiVerAbs += uiSymbol;
    }

    if( bHorAbsGr0 )
    {
      if( 2 == uiHorAbs )
      {
        xReadEpExGolomb( uiSymbol, 1 );
        uiHorAbs += uiSymbol;
      }

      m_pcTDecBinIf->decodeBinEP( uiHorSign );
    }

    if( bVerAbsGr0 )
    {
      if( 2 == uiVerAbs )
      {
        xReadEpExGolomb( uiSymbol, 1 );
        uiVerAbs += uiSymbol;
      }

      m_pcTDecBinIf->decodeBinEP( uiVerSign );
    }

  }

#if QC_MV_STORE_PRECISION_BIT
  TComMv cMv( uiHorSign ? -Int( uiHorAbs ): uiHorAbs, uiVerSign ? -Int( uiVerAbs ) : uiVerAbs );
  cMv <<= ( QC_MV_STORE_PRECISION_BIT - QC_MV_SIGNAL_PRECISION_BIT );
#else
  const TComMv cMv( uiHorSign ? -Int( uiHorAbs ): uiHorAbs, uiVerSign ? -Int( uiVerAbs ) : uiVerAbs );
#endif
  pcCU->getCUMvField( eRefList )->setAllMvd( cMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiDepth, uiPartIdx );
  return;
}


Void TDecSbac::parseTransformSubdivFlag( UInt& ruiSubdivFlag, UInt uiLog2TransformBlockSize )
{
  m_pcTDecBinIf->decodeBin( ruiSubdivFlag, m_cCUTransSubdivFlagSCModel.get( 0, 0, uiLog2TransformBlockSize ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseTransformSubdivFlag()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( ruiSubdivFlag )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiLog2TransformBlockSize )
  DTRACE_CABAC_T( "\n" )
}

Void TDecSbac::parseQtRootCbf( UInt uiAbsPartIdx, UInt& uiQtRootCbf )
{
  UInt uiSymbol;
  const UInt uiCtx = 0;
  m_pcTDecBinIf->decodeBin( uiSymbol , m_cCUQtRootCbfSCModel.get( 0, 0, uiCtx ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtRootCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiSymbol )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )
  
  uiQtRootCbf = uiSymbol;
}

Void TDecSbac::parseDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  Int qp;
  UInt uiDQp;
  Int  iDQp;
  
  UInt uiSymbol;

  xReadUnaryMaxSymbol (uiDQp,  &m_cCUDeltaQpSCModel.get( 0, 0, 0 ), 1, CU_DQP_TU_CMAX);

  if( uiDQp >= CU_DQP_TU_CMAX)
  {
    xReadEpExGolomb( uiSymbol, CU_DQP_EG_k );
    uiDQp+=uiSymbol;
  }

  if ( uiDQp > 0 )
  {
    UInt uiSign;
    Int qpBdOffsetY = pcCU->getSlice()->getSPS()->getQpBDOffsetY();
    m_pcTDecBinIf->decodeBinEP(uiSign);
    iDQp = uiDQp;
    if(uiSign)
    {
      iDQp = -iDQp;
    }
    qp = (((Int) pcCU->getRefQP( uiAbsPartIdx ) + iDQp + 52 + 2*qpBdOffsetY )%(52+qpBdOffsetY)) - qpBdOffsetY;
  }
  else 
  {
    qp = pcCU->getRefQP(uiAbsPartIdx);
  }
  pcCU->setQPSubParts(qp, uiAbsPartIdx, uiDepth);  
  pcCU->setCodedQP(qp);
}

Void TDecSbac::parseQtCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth, UInt uiDepth )
{
  UInt uiSymbol;
  const UInt uiCtx = pcCU->getCtxQtCbf( eType, uiTrDepth );
  m_pcTDecBinIf->decodeBin( uiSymbol , m_cCUQtCbfSCModel.get( 0, eType ? TEXT_CHROMA: eType, uiCtx ) );
  
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiSymbol )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tetype=" )
  DTRACE_CABAC_V( eType )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )
  
  pcCU->setCbfSubParts( uiSymbol << uiTrDepth, eType, uiAbsPartIdx, uiDepth );
}

void TDecSbac::parseTransformSkipFlags (TComDataCU* pcCU, UInt uiAbsPartIdx, UInt width, UInt height, UInt uiDepth, TextType eTType)
{
  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    return;
  }
  if(width != 4 || height != 4)
  {
    return;
  }
  
  UInt useTransformSkip;
  m_pcTDecBinIf->decodeBin( useTransformSkip , m_cTransformSkipSCModel.get( 0, eTType? TEXT_CHROMA: TEXT_LUMA, 0 ) );
  if(eTType!= TEXT_LUMA)
  {
    const UInt uiLog2TrafoSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()] + 2 - uiDepth;
    if(uiLog2TrafoSize == 2) 
    { 
      uiDepth --;
    }
  }
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T("\tparseTransformSkip()");
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( useTransformSkip )
  DTRACE_CABAC_T( "\tAddr=" )
  DTRACE_CABAC_V( pcCU->getAddr() )
  DTRACE_CABAC_T( "\tetype=" )
  DTRACE_CABAC_V( eTType )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )
  pcCU->setTransformSkipSubParts( useTransformSkip, eTType, uiAbsPartIdx, uiDepth);
}

#if KLT_COMMON
void TDecSbac::parseKLTFlags(TComDataCU* pcCU, UInt uiAbsPartIdx, UInt width, UInt height, UInt uiDepth, TextType eTType)
{
  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    return;
  }
  UInt uiMaxTrWidth = g_uiDepth2Width[USE_MORE_BLOCKSIZE_DEPTH_MAX - 1];
  UInt uiMinTrWidth = g_uiDepth2Width[USE_MORE_BLOCKSIZE_DEPTH_MIN - 1];
  Bool checkKLTY = ((width == height) && (width <= uiMaxTrWidth) && (width >= uiMinTrWidth) && (eTType == TEXT_LUMA));
  if (checkKLTY == false)
  {
    return;
  }
  UInt useKLTFlag = 0;

  m_pcTDecBinIf->decodeBin(useKLTFlag, m_cKLTFlagSCModel.get(0, eTType ? TEXT_CHROMA : TEXT_LUMA, 0));

  if (eTType != TEXT_LUMA)
  {
    const UInt uiLog2TrafoSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()] + 2 - uiDepth;
    if (uiLog2TrafoSize == 2)
    {
      uiDepth--;
    }
  }
  DTRACE_CABAC_VL(g_nSymbolCounter++)
  DTRACE_CABAC_T("\tparseKLTFlag()");
  DTRACE_CABAC_T("\tsymbol=")
  DTRACE_CABAC_V(useKLTFlag)
  DTRACE_CABAC_T("\tAddr=")
  DTRACE_CABAC_V(pcCU->getAddr())
  DTRACE_CABAC_T("\tetype=")
  DTRACE_CABAC_V(eTType)
  DTRACE_CABAC_T("\tuiAbsPartIdx=")
  DTRACE_CABAC_V(uiAbsPartIdx)
  DTRACE_CABAC_T("\n")

  pcCU->setKLTFlagSubParts(useKLTFlag, eTType, uiAbsPartIdx, uiDepth);
}
#endif

/** Parse (X,Y) position of the last significant coefficient
 * \param uiPosLastX reference to X component of last coefficient
 * \param uiPosLastY reference to Y component of last coefficient
 * \param width  Block width
 * \param height Block height
 * \param eTType plane type / luminance or chrominance
 * \param uiScanIdx scan type (zig-zag, hor, ver)
 *
 * This method decodes the X and Y component within a block of the last significant coefficient.
 */
Void TDecSbac::parseLastSignificantXY( UInt& uiPosLastX, UInt& uiPosLastY, Int width, Int height, TextType eTType, UInt uiScanIdx )
{
  UInt uiLast;
  ContextModel *pCtxX = m_cCuCtxLastX.get( 0, eTType );
  ContextModel *pCtxY = m_cCuCtxLastY.get( 0, eTType );
#if QC_CTX_RESIDUALCODING && !QC_T64
// posX
  Int widthCtx = eTType ? 4 : width;
  const UInt *puiCtxIdxX = g_uiLastCtx + ( g_aucConvertToBit[ widthCtx ] * ( g_aucConvertToBit[ widthCtx ] + 3 ) );

  for( uiPosLastX = 0; uiPosLastX < g_uiGroupIdx[ width - 1 ]; uiPosLastX++ )
  {
    if ( eTType  )
    {
      m_pcTDecBinIf->decodeBin( uiLast, *( pCtxX + (uiPosLastX>>g_aucConvertToBit[ width ])  ) );
    }
    else
    {
      m_pcTDecBinIf->decodeBin( uiLast, *( pCtxX + puiCtxIdxX[ uiPosLastX ] ) );
    }

    if( !uiLast )
    {
      break;
    }
  }

  // posY

  Int heightCtx = eTType? 4 : height;
  const UInt *puiCtxIdxY = g_uiLastCtx + ( g_aucConvertToBit[ heightCtx ] * ( g_aucConvertToBit[ heightCtx ] + 3 ) );

  for( uiPosLastY = 0; uiPosLastY < g_uiGroupIdx[ height - 1 ]; uiPosLastY++ )
  {
    if (eTType)
    {
      m_pcTDecBinIf->decodeBin( uiLast, *( pCtxY + (uiPosLastY>>g_aucConvertToBit[ height ]) ) );
    }
    else
    {
      m_pcTDecBinIf->decodeBin( uiLast, *( pCtxY + puiCtxIdxY[ uiPosLastY ] ) );
    }

    if( !uiLast )
    {
      break;
    }
  }
#else
  Int blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY;
  blkSizeOffsetX = eTType ? 0: (g_aucConvertToBit[ width ] *3 + ((g_aucConvertToBit[ width ] +1)>>2));
  blkSizeOffsetY = eTType ? 0: (g_aucConvertToBit[ height ]*3 + ((g_aucConvertToBit[ height ]+1)>>2));
  shiftX= eTType ? g_aucConvertToBit[ width  ] :((g_aucConvertToBit[ width  ]+3)>>2);
  shiftY= eTType ? g_aucConvertToBit[ height ] :((g_aucConvertToBit[ height ]+3)>>2);
  // posX
  for( uiPosLastX = 0; uiPosLastX < g_uiGroupIdx[ width - 1 ]; uiPosLastX++ )
  {
    m_pcTDecBinIf->decodeBin( uiLast, *( pCtxX + blkSizeOffsetX + (uiPosLastX >>shiftX) ) );
    if( !uiLast )
    {
      break;
    }
  }

  // posY
  for( uiPosLastY = 0; uiPosLastY < g_uiGroupIdx[ height - 1 ]; uiPosLastY++ )
  {
    m_pcTDecBinIf->decodeBin( uiLast, *( pCtxY + blkSizeOffsetY + (uiPosLastY >>shiftY)) );
    if( !uiLast )
    {
      break;
    }
  }
#endif
  if ( uiPosLastX > 3 )
  {
    UInt uiTemp  = 0;
    UInt uiCount = ( uiPosLastX - 2 ) >> 1;
    for ( Int i = uiCount - 1; i >= 0; i-- )
    {
      m_pcTDecBinIf->decodeBinEP( uiLast );
      uiTemp += uiLast << i;
    }
    uiPosLastX = g_uiMinInGroup[ uiPosLastX ] + uiTemp;
  }
  if ( uiPosLastY > 3 )
  {
    UInt uiTemp  = 0;
    UInt uiCount = ( uiPosLastY - 2 ) >> 1;
    for ( Int i = uiCount - 1; i >= 0; i-- )
    {
      m_pcTDecBinIf->decodeBinEP( uiLast );
      uiTemp += uiLast << i;
    }
    uiPosLastY = g_uiMinInGroup[ uiPosLastY ] + uiTemp;
  }
  
  if( uiScanIdx == SCAN_VER )
  {
    swap( uiPosLastX, uiPosLastY );
  }
}
#if QC_CTX_RESIDUALCODING
Void TDecSbac::parseCoeffNxN( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType 
#if ROT_TR
    , Bool& bCbfCU
#endif
    )
{
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
    DTRACE_CABAC_T( "\tparseCoeffNxN()\teType=" )
    DTRACE_CABAC_V( eTType )
    DTRACE_CABAC_T( "\twidth=" )
    DTRACE_CABAC_V( uiWidth )
    DTRACE_CABAC_T( "\theight=" )
    DTRACE_CABAC_V( uiHeight )
    DTRACE_CABAC_T( "\tdepth=" )
    DTRACE_CABAC_V( uiDepth )
    DTRACE_CABAC_T( "\tabspartidx=" )
    DTRACE_CABAC_V( uiAbsPartIdx )
    DTRACE_CABAC_T( "\ttoCU-X=" )
    DTRACE_CABAC_V( pcCU->getCUPelX() )
    DTRACE_CABAC_T( "\ttoCU-Y=" )
    DTRACE_CABAC_V( pcCU->getCUPelY() )
    DTRACE_CABAC_T( "\tCU-addr=" )
    DTRACE_CABAC_V(  pcCU->getAddr() )
    DTRACE_CABAC_T( "\tinCU-X=" )
    DTRACE_CABAC_V( g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ] )
    DTRACE_CABAC_T( "\tinCU-Y=" )
    DTRACE_CABAC_V( g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ] )
    DTRACE_CABAC_T( "\tpredmode=" )
    DTRACE_CABAC_V(  pcCU->getPredictionMode( uiAbsPartIdx ) )
    DTRACE_CABAC_T( "\n" )

#if QC_EMT_INTRA
    UInt uiNumSig=0;
#endif

  if( uiWidth > pcCU->getSlice()->getSPS()->getMaxTrSize() )
  {
    uiWidth  = pcCU->getSlice()->getSPS()->getMaxTrSize();
    uiHeight = pcCU->getSlice()->getSPS()->getMaxTrSize();
  }
  if(pcCU->getSlice()->getPPS()->getUseTransformSkip())
  {
    parseTransformSkipFlags( pcCU, uiAbsPartIdx, uiWidth, uiHeight, uiDepth, eTType);
  }
#if KLT_COMMON
  UInt uiMaxTrWidth = g_uiDepth2Width[USE_MORE_BLOCKSIZE_DEPTH_MAX - 1];
  UInt uiMinTrWidth = g_uiDepth2Width[USE_MORE_BLOCKSIZE_DEPTH_MIN - 1];
  Bool bCheckKLTFlag = (eTType == TEXT_LUMA) && (uiWidth == uiHeight) && (uiWidth <= uiMaxTrWidth) && (uiWidth >= uiMinTrWidth);
  if (bCheckKLTFlag && pcCU->getSlice()->getPPS()->getUseTransformSkip())
  {
    UInt useTransformSkip = pcCU->getTransformSkip(uiAbsPartIdx, eTType);
    bCheckKLTFlag &= !useTransformSkip;
  }

#if INTER_KLT && !INTRA_KLT //only inter
  bCheckKLTFlag &= (!pcCU->isIntra(uiAbsPartIdx));
#endif
#if !INTER_KLT && INTRA_KLT //only intra
  bCheckKLTFlag &= (pcCU->isIntra(uiAbsPartIdx));
#endif
#if !INTER_KLT && !INTRA_KLT //none
  bCheckKLTFlag = false;
#endif

  if (bCheckKLTFlag)
  {
    parseKLTFlags(pcCU, uiAbsPartIdx, uiWidth, uiHeight, uiDepth, eTType);
  }
#endif

  eTType = eTType == TEXT_LUMA ? TEXT_LUMA : ( eTType == TEXT_NONE ? TEXT_NONE : TEXT_CHROMA );

  //----- parse significance map -----
  const UInt  uiLog2BlockSize   = g_aucConvertToBit[ uiWidth ] + 2;
  const UInt  uiMaxNumCoeff     = uiWidth * uiHeight;
  const UInt  uiMaxNumCoeffM1   = uiMaxNumCoeff - 1;
  UInt uiScanIdx = pcCU->getCoefScanIdx(uiAbsPartIdx, uiWidth, eTType==TEXT_LUMA, pcCU->isIntra(uiAbsPartIdx));

  //===== decode last significant =====
  UInt uiPosLastX, uiPosLastY;
  parseLastSignificantXY( uiPosLastX, uiPosLastY, uiWidth, uiHeight, eTType, uiScanIdx );
  UInt uiBlkPosLast      = uiPosLastX + (uiPosLastY<<uiLog2BlockSize);
  pcCoef[ uiBlkPosLast ] = 1;

  //===== decode significance flags =====
  UInt uiScanPosLast;
  const UInt *scan   = g_auiSigLastScan[ uiScanIdx ][ uiLog2BlockSize-1 ];
  for( uiScanPosLast = 0; uiScanPosLast < uiMaxNumCoeffM1; uiScanPosLast++ )
  {
    UInt uiBlkPos = scan[ uiScanPosLast ];
    if( uiBlkPosLast == uiBlkPos )
    {
      break;
    }
  }

  UInt iOffsetonTU = (g_aucConvertToBit[ uiWidth ]>2? 2:g_aucConvertToBit[ uiWidth ])*NUM_SIG_FLAG_CTX_LUMA_TU;
  ContextModel * const baseCoeffGroupCtx = m_cCUSigCoeffGroupSCModel.get( 0, eTType );
  ContextModel * const baseCtx = (eTType==TEXT_LUMA) ? m_cCUSigSCModel.get( 0, 0 ) + iOffsetonTU : m_cCUSigSCModel.get( 0, 0 ) + NUM_SIG_FLAG_CTX_LUMA;
  ContextModel * const greXCtx = (eTType==TEXT_LUMA) ? m_cCUOneSCModel.get( 0, 0 ) : m_cCUOneSCModel.get( 0, 0 ) + NUM_ONE_FLAG_CTX_LUMA;

  const Int  iLastScanSet      = uiScanPosLast >> LOG2_SCAN_SET_SIZE;
  UInt uiGoRiceParam           = 0;

  Bool beValid; 
  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    beValid = false;
  }
  else 
  {
    beValid = pcCU->getSlice()->getPPS()->getSignHideFlag() > 0;
  }
  UInt absSum = 0;

  UInt uiSigCoeffGroupFlag[ MLS_GRP_NUM ];
  ::memset( uiSigCoeffGroupFlag, 0, sizeof(UInt) * MLS_GRP_NUM );
  const UInt uiNumBlkSide = uiWidth >> (MLS_CG_SIZE >> 1);
  const UInt * scanCG;
  {
    scanCG = g_auiSigLastScan[ uiScanIdx ][ uiLog2BlockSize > 3 ? uiLog2BlockSize-2-1 : 0  ];    
    if( uiLog2BlockSize == 3 )
    {
      scanCG = g_sigLastScan8x8[ uiScanIdx ];
    }
    else if( uiLog2BlockSize == 5 )
    {
      scanCG = g_sigLastScanCG32x32;
    }
#if QC_T64
    else if( uiLog2BlockSize == 6 )
    {
      scanCG = g_sigLastScanCG64x64;
    }
#endif
  }

  Int  iScanPosSig             = (Int) uiScanPosLast;
  Bool bHor8x8 = uiWidth == 8 && uiHeight == 8 && uiScanIdx == SCAN_HOR;
  Bool bVer8x8 = uiWidth == 8 && uiHeight == 8 && uiScanIdx == SCAN_VER;
  Bool bNonZig8x8 = bHor8x8 || bVer8x8; 

  for( Int iSubSet = iLastScanSet; iSubSet >= 0; iSubSet-- )
  {
    Int  iSubPos     = iSubSet << LOG2_SCAN_SET_SIZE;
    uiGoRiceParam    = 0;
    Int numNonZero   = 0;
    UInt ctxG1       = 0;
    UInt ctxG2       = 0;

    Int lastNZPosInCG = -1, firstNZPosInCG = SCAN_SET_SIZE;
    Int pos[SCAN_SET_SIZE];
    if( iScanPosSig == (Int) uiScanPosLast )
    {
      lastNZPosInCG  = iScanPosSig;
      firstNZPosInCG = iScanPosSig;
      iScanPosSig--;
      pos[ numNonZero ] = uiBlkPosLast;
      numNonZero = 1;
    }

    // decode significant_coeffgroup_flag
    Int iCGBlkPos = scanCG[ iSubSet ];
    Int iCGPosY   = iCGBlkPos / uiNumBlkSide;
    Int iCGPosX   = iCGBlkPos - (iCGPosY * uiNumBlkSide);
    if(bNonZig8x8)
    {
      iCGPosY = (bHor8x8 ? iCGBlkPos : 0);
      iCGPosX = (bVer8x8 ? iCGBlkPos : 0);
    }
    if( iSubSet == iLastScanSet || iSubSet == 0)
    {
      uiSigCoeffGroupFlag[ iCGBlkPos ] = 1;
    }
#if QC_T64
    else if( uiWidth>=64 && ( iCGPosY>=(uiNumBlkSide/2) || iCGPosX>=(uiNumBlkSide/2) ) )
    {
      uiSigCoeffGroupFlag[ iCGBlkPos ] = 0;
    }
#endif
    else
    {
      UInt uiSigCoeffGroup;
      UInt uiCtxSig  = TComTrQuant::getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, iCGPosX, iCGPosY, uiScanIdx, uiWidth, uiHeight );
      m_pcTDecBinIf->decodeBin( uiSigCoeffGroup, baseCoeffGroupCtx[ uiCtxSig ] );
      uiSigCoeffGroupFlag[ iCGBlkPos ] = uiSigCoeffGroup;
    }

    // decode significant_coeff_flag
    UInt uiBlkPos, uiPosY, uiPosX, uiSig;
    for( ; iScanPosSig >= iSubPos; iScanPosSig-- )
    {
      uiBlkPos  = scan[ iScanPosSig ];
      if( uiSigCoeffGroupFlag[ iCGBlkPos ] )
      {
        uiSig     = 0;
        if( iScanPosSig > iSubPos || iSubSet == 0  || numNonZero )
        {
          uiPosY    = uiBlkPos >> uiLog2BlockSize;
          uiPosX    = uiBlkPos - ( uiPosY << uiLog2BlockSize );
          Int uiCtxSig  = TComTrQuant::getGrtZeroCtxInc( pcCoef, uiPosX, uiPosY, uiWidth, uiHeight, eTType );
          m_pcTDecBinIf->decodeBin( uiSig, baseCtx[ uiCtxSig ] );
        }
        else
        {
          uiSig = 1;
        }
        pcCoef[ uiBlkPos ] = uiSig;
        if( uiSig )
        {
          pos[ numNonZero ] = uiBlkPos;
          numNonZero ++;
          if( lastNZPosInCG == -1 )
          {
            lastNZPosInCG = iScanPosSig;
          }
          firstNZPosInCG = iScanPosSig;
        }
      }
      else
      {
        pcCoef[ uiBlkPos ] = 0;
      }
    }

    if( numNonZero )
    {
#if ROT_TR 
  bCbfCU = true;
#endif
      //.................start...................../
      //bit-plane decoding 2nd/3rd/remaining bins
      Int numC1Flag = min(numNonZero, C1FLAG_NUMBER);
      Int firstC2FlagIdx = -1;
      UInt greO = 0;

      for( Int idx = 0; idx < numC1Flag; idx++ )
      {
        uiBlkPos  = pos[idx];
        if( idx || iSubSet != iLastScanSet)
        {
          uiPosY    = uiBlkPos >> uiLog2BlockSize;
          uiPosX    = uiBlkPos - ( uiPosY << uiLog2BlockSize );
          ctxG1 = TComTrQuant::getGrtOneCtxInc( pcCoef, uiPosX, uiPosY, uiWidth, uiHeight, eTType );
        }
        m_pcTDecBinIf->decodeBin( greO, greXCtx[ ctxG1 ] );
        pcCoef[ uiBlkPos ] += greO;

        if(greO)
        {
          if (firstC2FlagIdx == -1)
          {
            firstC2FlagIdx = idx;
          }
        }
      }

      if(firstC2FlagIdx != -1)
      {
        uiBlkPos  = pos[firstC2FlagIdx]; 
        if( firstC2FlagIdx || iSubSet != iLastScanSet )
        {
          uiPosY    = uiBlkPos >> uiLog2BlockSize;
          uiPosX    = uiBlkPos - ( uiPosY << uiLog2BlockSize );
          ctxG2 = TComTrQuant::getGrtTwoCtxInc( pcCoef, uiPosX, uiPosY, uiWidth, uiHeight, eTType );
        }
        m_pcTDecBinIf->decodeBin( greO, greXCtx[ ctxG2 ] );
        pcCoef[ uiBlkPos ] += greO;
      }

      Int iFirstCoeff2 = 1;
      if (firstC2FlagIdx!= -1 || numNonZero > C1FLAG_NUMBER)
      {
        for( Int idx = 0; idx < numNonZero; idx++ )      
        {
          uiBlkPos  = pos[ idx ];
          const Int baseLevel =  ( idx < C1FLAG_NUMBER ) ? (2 + iFirstCoeff2) : 1;
          if( pcCoef[ uiBlkPos ] == baseLevel )
          {
            uiPosY    = uiBlkPos >> uiLog2BlockSize;
            uiPosX    = uiBlkPos - ( uiPosY << uiLog2BlockSize );    
            uiGoRiceParam = TComTrQuant::getRemainCoeffCtxInc( pcCoef, uiPosX, uiPosY, uiWidth, uiHeight );
            xReadGoRiceExGolomb( uiSig, uiGoRiceParam );
            pcCoef[ uiBlkPos ] = (uiSig+baseLevel);
          }
          if(pcCoef[ uiBlkPos ] >= 2)
          {
            iFirstCoeff2 = 0;
          }
        }
      }
      //.................end...................../
      Bool signHidden = ( lastNZPosInCG - firstNZPosInCG >= SBH_THRESHOLD );
      absSum = 0;
      UInt coeffSigns;
      if ( signHidden && beValid )
      {
        m_pcTDecBinIf->decodeBinsEP( coeffSigns, numNonZero-1 );
        coeffSigns <<= 32 - (numNonZero-1);
      }
      else
      {
        m_pcTDecBinIf->decodeBinsEP( coeffSigns, numNonZero );
        coeffSigns <<= 32 - numNonZero;
      }

      for( Int idx = 0; idx < numNonZero; idx++ )
      {
        Int blkPos = pos[ idx ];
        // Signs applied later.
        absSum += pcCoef[ blkPos ];

        if ( idx == numNonZero-1 && signHidden && beValid )
        {
          // Infer sign of 1st element.
          if (absSum&0x1)
            pcCoef[ blkPos ] = -pcCoef[ blkPos ];
        }
        else
        {
          Int sign = static_cast<Int>( coeffSigns ) >> 31;
          pcCoef[ blkPos ] = ( pcCoef[ blkPos ] ^ sign ) - sign;
          coeffSigns <<= 1;
        }
      }
    }
#if QC_EMT_INTRA
    uiNumSig += numNonZero;
#endif
  }

#if QC_EMT
  UInt useTransformSkip = pcCU->getTransformSkip( uiAbsPartIdx,eTType);
  if (!useTransformSkip && eTType == TEXT_LUMA )
  {
#if QC_EMT_INTRA
    if ( pcCU->getEmtCuFlag( uiAbsPartIdx ) && pcCU->isIntra( uiAbsPartIdx ) )
    {
      if( uiNumSig>g_iEmtSigNumThr )
      {
        parseEmtTuIdx( pcCU, uiAbsPartIdx, uiDepth ); 
      }
      else
      {
        pcCU->setEmtTuIdxSubParts( 0, uiAbsPartIdx, uiDepth );
      }
    }
#endif

#if QC_EMT_INTER
    if ( pcCU->getEmtCuFlag( uiAbsPartIdx ) && !pcCU->isIntra( uiAbsPartIdx ) )
    {
      parseEmtTuIdx( pcCU, uiAbsPartIdx, uiDepth ); 
    }
#endif
  }
#endif
  return;
}
#else
Void TDecSbac::parseCoeffNxN( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType 
#if ROT_TR
    , Bool& bCbfCU
#endif
    )
{
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseCoeffNxN()\teType=" )
  DTRACE_CABAC_V( eTType )
  DTRACE_CABAC_T( "\twidth=" )
  DTRACE_CABAC_V( uiWidth )
  DTRACE_CABAC_T( "\theight=" )
  DTRACE_CABAC_V( uiHeight )
  DTRACE_CABAC_T( "\tdepth=" )
  DTRACE_CABAC_V( uiDepth )
  DTRACE_CABAC_T( "\tabspartidx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\ttoCU-X=" )
  DTRACE_CABAC_V( pcCU->getCUPelX() )
  DTRACE_CABAC_T( "\ttoCU-Y=" )
  DTRACE_CABAC_V( pcCU->getCUPelY() )
  DTRACE_CABAC_T( "\tCU-addr=" )
  DTRACE_CABAC_V(  pcCU->getAddr() )
  DTRACE_CABAC_T( "\tinCU-X=" )
  DTRACE_CABAC_V( g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ] )
  DTRACE_CABAC_T( "\tinCU-Y=" )
  DTRACE_CABAC_V( g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ] )
  DTRACE_CABAC_T( "\tpredmode=" )
  DTRACE_CABAC_V(  pcCU->getPredictionMode( uiAbsPartIdx ) )
  DTRACE_CABAC_T( "\n" )
  
#if QC_EMT
  UInt uiNumSig=0;
#endif

  if( uiWidth > pcCU->getSlice()->getSPS()->getMaxTrSize() )
  {
    uiWidth  = pcCU->getSlice()->getSPS()->getMaxTrSize();
    uiHeight = pcCU->getSlice()->getSPS()->getMaxTrSize();
  }
  if(pcCU->getSlice()->getPPS()->getUseTransformSkip())
  {
    parseTransformSkipFlags( pcCU, uiAbsPartIdx, uiWidth, uiHeight, uiDepth, eTType);
  }

  eTType = eTType == TEXT_LUMA ? TEXT_LUMA : ( eTType == TEXT_NONE ? TEXT_NONE : TEXT_CHROMA );
  
  //----- parse significance map -----
  const UInt  uiLog2BlockSize   = g_aucConvertToBit[ uiWidth ] + 2;
  const UInt  uiMaxNumCoeff     = uiWidth * uiHeight;
  const UInt  uiMaxNumCoeffM1   = uiMaxNumCoeff - 1;
  UInt uiScanIdx = pcCU->getCoefScanIdx(uiAbsPartIdx, uiWidth, eTType==TEXT_LUMA, pcCU->isIntra(uiAbsPartIdx));
  
  //===== decode last significant =====
  UInt uiPosLastX, uiPosLastY;
  parseLastSignificantXY( uiPosLastX, uiPosLastY, uiWidth, uiHeight, eTType, uiScanIdx );
  UInt uiBlkPosLast      = uiPosLastX + (uiPosLastY<<uiLog2BlockSize);
  pcCoef[ uiBlkPosLast ] = 1;

  //===== decode significance flags =====
  UInt uiScanPosLast;
  const UInt *scan   = g_auiSigLastScan[ uiScanIdx ][ uiLog2BlockSize-1 ];
  for( uiScanPosLast = 0; uiScanPosLast < uiMaxNumCoeffM1; uiScanPosLast++ )
  {
    UInt uiBlkPos = scan[ uiScanPosLast ];
    if( uiBlkPosLast == uiBlkPos )
    {
      break;
    }
  }

  ContextModel * const baseCoeffGroupCtx = m_cCUSigCoeffGroupSCModel.get( 0, eTType );
  ContextModel * const baseCtx = (eTType==TEXT_LUMA) ? m_cCUSigSCModel.get( 0, 0 ) : m_cCUSigSCModel.get( 0, 0 ) + NUM_SIG_FLAG_CTX_LUMA;

  const Int  iLastScanSet      = uiScanPosLast >> LOG2_SCAN_SET_SIZE;
  UInt c1 = 1;
  UInt uiGoRiceParam           = 0;

  Bool beValid; 
  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    beValid = false;
  }
  else 
  {
    beValid = pcCU->getSlice()->getPPS()->getSignHideFlag() > 0;
  }
  UInt absSum = 0;

  UInt uiSigCoeffGroupFlag[ MLS_GRP_NUM ];
  ::memset( uiSigCoeffGroupFlag, 0, sizeof(UInt) * MLS_GRP_NUM );
  const UInt uiNumBlkSide = uiWidth >> (MLS_CG_SIZE >> 1);
  const UInt * scanCG;
  {
    scanCG = g_auiSigLastScan[ uiScanIdx ][ uiLog2BlockSize > 3 ? uiLog2BlockSize-2-1 : 0  ];    
    if( uiLog2BlockSize == 3 )
    {
      scanCG = g_sigLastScan8x8[ uiScanIdx ];
    }
    else if( uiLog2BlockSize == 5 )
    {
      scanCG = g_sigLastScanCG32x32;
    }
#if QC_T64
    else if( uiLog2BlockSize == 6 )
    {
      scanCG = g_sigLastScanCG64x64;
    }
#endif
  }

  Int  iScanPosSig             = (Int) uiScanPosLast;
  for( Int iSubSet = iLastScanSet; iSubSet >= 0; iSubSet-- )
  {
    Int  iSubPos     = iSubSet << LOG2_SCAN_SET_SIZE;
    uiGoRiceParam    = 0;
    Int numNonZero = 0;
    
    Int lastNZPosInCG = -1, firstNZPosInCG = SCAN_SET_SIZE;

    Int pos[SCAN_SET_SIZE];
    if( iScanPosSig == (Int) uiScanPosLast )
    {
      lastNZPosInCG  = iScanPosSig;
      firstNZPosInCG = iScanPosSig;
      iScanPosSig--;
      pos[ numNonZero ] = uiBlkPosLast;
      numNonZero = 1;
    }

    // decode significant_coeffgroup_flag
    Int iCGBlkPos = scanCG[ iSubSet ];
    Int iCGPosY   = iCGBlkPos / uiNumBlkSide;
    Int iCGPosX   = iCGBlkPos - (iCGPosY * uiNumBlkSide);
    if( iSubSet == iLastScanSet || iSubSet == 0)
    {
      uiSigCoeffGroupFlag[ iCGBlkPos ] = 1;
    }
#if QC_T64
    else if( uiWidth>=64 && ( iCGPosY>=(uiNumBlkSide/2) || iCGPosX>=(uiNumBlkSide/2) ) )
    {
      uiSigCoeffGroupFlag[ iCGBlkPos ] = 0;
    }
#endif
    else
    {
      UInt uiSigCoeffGroup;
      UInt uiCtxSig  = TComTrQuant::getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, iCGPosX, iCGPosY, uiWidth, uiHeight );
      m_pcTDecBinIf->decodeBin( uiSigCoeffGroup, baseCoeffGroupCtx[ uiCtxSig ] );
      uiSigCoeffGroupFlag[ iCGBlkPos ] = uiSigCoeffGroup;
    }

    // decode significant_coeff_flag
    Int patternSigCtx = TComTrQuant::calcPatternSigCtx( uiSigCoeffGroupFlag, iCGPosX, iCGPosY, uiWidth, uiHeight );
    UInt uiBlkPos, uiPosY, uiPosX, uiSig, uiCtxSig;
    for( ; iScanPosSig >= iSubPos; iScanPosSig-- )
    {
      uiBlkPos  = scan[ iScanPosSig ];
      uiPosY    = uiBlkPos >> uiLog2BlockSize;
      uiPosX    = uiBlkPos - ( uiPosY << uiLog2BlockSize );
      uiSig     = 0;
      
      if( uiSigCoeffGroupFlag[ iCGBlkPos ] )
      {
        if( iScanPosSig > iSubPos || iSubSet == 0  || numNonZero )
        {
          uiCtxSig  = TComTrQuant::getSigCtxInc( patternSigCtx, uiScanIdx, uiPosX, uiPosY, uiLog2BlockSize, eTType );
          m_pcTDecBinIf->decodeBin( uiSig, baseCtx[ uiCtxSig ] );
        }
        else
        {
          uiSig = 1;
        }
      }
      pcCoef[ uiBlkPos ] = uiSig;
      if( uiSig )
      {
        pos[ numNonZero ] = uiBlkPos;
        numNonZero ++;
        if( lastNZPosInCG == -1 )
        {
          lastNZPosInCG = iScanPosSig;
        }
        firstNZPosInCG = iScanPosSig;
      }
    }
    
    if( numNonZero )
    {
      Bool signHidden = ( lastNZPosInCG - firstNZPosInCG >= SBH_THRESHOLD );
      absSum = 0;
      UInt uiCtxSet    = (iSubSet > 0 && eTType==TEXT_LUMA) ? 2 : 0;
      UInt uiBin;
      if( c1 == 0 )
      {
        uiCtxSet++;
      }
      c1 = 1;
      ContextModel *baseCtxMod = ( eTType==TEXT_LUMA ) ? m_cCUOneSCModel.get( 0, 0 ) + 4 * uiCtxSet : m_cCUOneSCModel.get( 0, 0 ) + NUM_ONE_FLAG_CTX_LUMA + 4 * uiCtxSet;
      Int absCoeff[SCAN_SET_SIZE];
#if ROT_TR 
  bCbfCU = true;
#endif
      for ( Int i = 0; i < numNonZero; i++) absCoeff[i] = 1;   
      Int numC1Flag = min(numNonZero, C1FLAG_NUMBER);
      Int firstC2FlagIdx = -1;

      for( Int idx = 0; idx < numC1Flag; idx++ )
      {
        m_pcTDecBinIf->decodeBin( uiBin, baseCtxMod[c1] );
        if( uiBin == 1 )
        {
          c1 = 0;
          if (firstC2FlagIdx == -1)
          {
            firstC2FlagIdx = idx;
          }
        }
        else if( (c1 < 3) && (c1 > 0) )
        {
          c1++;
        }
        absCoeff[ idx ] = uiBin + 1;
      }
      
      if (c1 == 0)
      {
        baseCtxMod = ( eTType==TEXT_LUMA ) ? m_cCUAbsSCModel.get( 0, 0 ) + uiCtxSet : m_cCUAbsSCModel.get( 0, 0 ) + NUM_ABS_FLAG_CTX_LUMA + uiCtxSet;
        if ( firstC2FlagIdx != -1)
        {
          m_pcTDecBinIf->decodeBin( uiBin, baseCtxMod[0] ); 
          absCoeff[ firstC2FlagIdx ] = uiBin + 2;
        }
      }

      UInt coeffSigns;
      if ( signHidden && beValid )
      {
        m_pcTDecBinIf->decodeBinsEP( coeffSigns, numNonZero-1 );
        coeffSigns <<= 32 - (numNonZero-1);
      }
      else
      {
        m_pcTDecBinIf->decodeBinsEP( coeffSigns, numNonZero );
        coeffSigns <<= 32 - numNonZero;
      }
      
      Int iFirstCoeff2 = 1;    
      if (c1 == 0 || numNonZero > C1FLAG_NUMBER)
      {
        for( Int idx = 0; idx < numNonZero; idx++ )
        {
          UInt baseLevel  = (idx < C1FLAG_NUMBER)? (2 + iFirstCoeff2) : 1;

          if( absCoeff[ idx ] == baseLevel)
          {
            UInt uiLevel;
            xReadCoefRemainExGolomb( uiLevel, uiGoRiceParam );
            absCoeff[ idx ] = uiLevel + baseLevel;
            if(absCoeff[idx]>3*(1<<uiGoRiceParam))
            {
              uiGoRiceParam = min<UInt>(uiGoRiceParam+ 1, 4);
            }
          }

          if(absCoeff[ idx ] >= 2)  
          {
            iFirstCoeff2 = 0;
          }
        }
      }

      for( Int idx = 0; idx < numNonZero; idx++ )
      {
        Int blkPos = pos[ idx ];
        // Signs applied later.
        pcCoef[ blkPos ] = absCoeff[ idx ];
        absSum += absCoeff[ idx ];

        if ( idx == numNonZero-1 && signHidden && beValid )
        {
          // Infer sign of 1st element.
          if (absSum&0x1)
          {
            pcCoef[ blkPos ] = -pcCoef[ blkPos ];
          }
        }
        else
        {
          Int sign = static_cast<Int>( coeffSigns ) >> 31;
          pcCoef[ blkPos ] = ( pcCoef[ blkPos ] ^ sign ) - sign;
          coeffSigns <<= 1;
        }
      }
    }
#if QC_EMT
    uiNumSig += numNonZero;
#endif
  }
  
#if QC_EMT
  if (!pcCU->getTransformSkip( uiAbsPartIdx,eTType) && eTType == TEXT_LUMA )
  {
#if QC_EMT_INTRA
    if ( pcCU->getEmtCuFlag( uiAbsPartIdx ) && pcCU->isIntra( uiAbsPartIdx ) )
    {
      if( uiNumSig>g_iEmtSigNumThr )
      {
        parseEmtTuIdx( pcCU, uiAbsPartIdx, uiDepth ); 
      }
      else
      {
        pcCU->setEmtTuIdxSubParts( 0, uiAbsPartIdx, uiDepth );
      }
    }
#endif
#if QC_EMT_INTER
    if ( pcCU->getEmtCuFlag( uiAbsPartIdx ) && !pcCU->isIntra( uiAbsPartIdx ) )
    {
      parseEmtTuIdx( pcCU, uiAbsPartIdx, uiDepth ); 
    }
#endif
  }
#endif

  return;
}
#endif



Void TDecSbac::parseSaoMaxUvlc ( UInt& val, UInt maxSymbol )
{
  if (maxSymbol == 0)
  {
    val = 0;
    return;
  }

  UInt code;
  Int  i;
  m_pcTDecBinIf->decodeBinEP( code );
  if ( code == 0 )
  {
    val = 0;
    return;
  }

  i=1;
  while (1)
  {
    m_pcTDecBinIf->decodeBinEP( code );
    if ( code == 0 )
    {
      break;
    }
    i++;
    if (i == maxSymbol) 
    {
      break;
    }
  }

  val = i;
}
Void TDecSbac::parseSaoUflc (UInt uiLength, UInt&  riVal)
{
  m_pcTDecBinIf->decodeBinsEP ( riVal, uiLength );
}
Void TDecSbac::parseSaoMerge (UInt&  ruiVal)
{
  UInt uiCode;
  m_pcTDecBinIf->decodeBin( uiCode, m_cSaoMergeSCModel.get( 0, 0, 0 ) );
  ruiVal = (Int)uiCode;
}
Void TDecSbac::parseSaoTypeIdx (UInt&  ruiVal)
{
  UInt uiCode;
  m_pcTDecBinIf->decodeBin( uiCode, m_cSaoTypeIdxSCModel.get( 0, 0, 0 ) );
  if (uiCode == 0) 
  {
    ruiVal = 0;
  }
  else
  {
    m_pcTDecBinIf->decodeBinEP( uiCode ); 
    if (uiCode == 0)
    {
      ruiVal = 1;
    }
    else
    {
      ruiVal = 2;
    }
  }
}

Void TDecSbac::parseSaoSign(UInt& val)
{
  m_pcTDecBinIf->decodeBinEP ( val ); 
}

Void TDecSbac::parseSAOBlkParam (SAOBlkParam& saoBlkParam
                                , Bool* sliceEnabled
                                , Bool leftMergeAvail
                                , Bool aboveMergeAvail
                                )
{
  UInt uiSymbol;

  Bool isLeftMerge = false;
  Bool isAboveMerge= false;

  if(leftMergeAvail)
  {
    parseSaoMerge(uiSymbol); //sao_merge_left_flag
    isLeftMerge = (uiSymbol?true:false);
  }

  if( aboveMergeAvail && !isLeftMerge)
  {
    parseSaoMerge(uiSymbol); //sao_merge_up_flag
    isAboveMerge = (uiSymbol?true:false);
  }

  if(isLeftMerge || isAboveMerge) //merge mode
  {
    saoBlkParam[SAO_Y].modeIdc = saoBlkParam[SAO_Cb].modeIdc = saoBlkParam[SAO_Cr].modeIdc = SAO_MODE_MERGE;
    saoBlkParam[SAO_Y].typeIdc = saoBlkParam[SAO_Cb].typeIdc = saoBlkParam[SAO_Cr].typeIdc = (isLeftMerge)?SAO_MERGE_LEFT:SAO_MERGE_ABOVE;
  }
  else //new or off mode
  {    
    for(Int compIdx=0; compIdx < NUM_SAO_COMPONENTS; compIdx++)
    {
      SAOOffset& ctbParam = saoBlkParam[compIdx];

      if(!sliceEnabled[compIdx])
      {
        //off
        ctbParam.modeIdc = SAO_MODE_OFF;
        continue;
      }

      //type
      if(compIdx == SAO_Y || compIdx == SAO_Cb)
      {
        parseSaoTypeIdx(uiSymbol); //sao_type_idx_luma or sao_type_idx_chroma

        assert(uiSymbol ==0 || uiSymbol ==1 || uiSymbol ==2);

        if(uiSymbol ==0) //OFF
        {
          ctbParam.modeIdc = SAO_MODE_OFF;
        }
        else if(uiSymbol == 1) //BO
        {
          ctbParam.modeIdc = SAO_MODE_NEW;
          ctbParam.typeIdc = SAO_TYPE_START_BO;
        }
        else //2, EO
        {
          ctbParam.modeIdc = SAO_MODE_NEW;
          ctbParam.typeIdc = SAO_TYPE_START_EO;
        }

      }
      else //Cr, follow Cb SAO type
      {
        ctbParam.modeIdc = saoBlkParam[SAO_Cb].modeIdc;
        ctbParam.typeIdc = saoBlkParam[SAO_Cb].typeIdc;
      }

      if(ctbParam.modeIdc == SAO_MODE_NEW)
      {
        Int offset[4];
        for(Int i=0; i< 4; i++)
        {
          parseSaoMaxUvlc(uiSymbol,  g_saoMaxOffsetQVal[compIdx] ); //sao_offset_abs
          offset[i] = (Int)uiSymbol;
        }

        if(ctbParam.typeIdc == SAO_TYPE_START_BO)
        {
          for(Int i=0; i< 4; i++)
          {
            if(offset[i] != 0)
            {
              parseSaoSign(uiSymbol); //sao_offset_sign
              if(uiSymbol)
              {
                offset[i] = -offset[i];
              }
            }
          }
          parseSaoUflc(NUM_SAO_BO_CLASSES_LOG2, uiSymbol ); //sao_band_position
          ctbParam.typeAuxInfo = uiSymbol;
        
          for(Int i=0; i<4; i++)
          {
            ctbParam.offset[(ctbParam.typeAuxInfo+i)%MAX_NUM_SAO_CLASSES] = offset[i];
          }      
        
        }
        else //EO
        {
          ctbParam.typeAuxInfo = 0;

          if(compIdx == SAO_Y || compIdx == SAO_Cb)
          {
            parseSaoUflc(NUM_SAO_EO_TYPES_LOG2, uiSymbol ); //sao_eo_class_luma or sao_eo_class_chroma
            ctbParam.typeIdc += uiSymbol;
          }
          else
          {
            ctbParam.typeIdc = saoBlkParam[SAO_Cb].typeIdc;
          }
          ctbParam.offset[SAO_CLASS_EO_FULL_VALLEY] = offset[0];
          ctbParam.offset[SAO_CLASS_EO_HALF_VALLEY] = offset[1];
          ctbParam.offset[SAO_CLASS_EO_PLAIN      ] = 0;
          ctbParam.offset[SAO_CLASS_EO_HALF_PEAK  ] = -offset[2];
          ctbParam.offset[SAO_CLASS_EO_FULL_PEAK  ] = -offset[3];
        }
      }
    }
  }
}

/**
 - Initialize our contexts from the nominated source.
 .
 \param pSrc Contexts to be copied.
 */
Void TDecSbac::xCopyContextsFrom( TDecSbac* pSrc )
{
  memcpy(m_contextModels, pSrc->m_contextModels, m_numContextModels*sizeof(m_contextModels[0]));
}

Void TDecSbac::xCopyFrom( TDecSbac* pSrc )
{
  m_pcTDecBinIf->copyState( pSrc->m_pcTDecBinIf );

  m_uiLastQp           = pSrc->m_uiLastQp;
  xCopyContextsFrom( pSrc );

}

Void TDecSbac::load ( TDecSbac* pScr )
{
  xCopyFrom(pScr);
}

Void TDecSbac::loadContexts ( TDecSbac* pScr )
{
  xCopyContextsFrom(pScr);
}

#if ALF_HM3_QC_REFACTOR
Void TDecSbac::parseAlfCtrlDepth( UInt& ruiAlfCtrlDepth )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol( uiSymbol, m_cALFUvlcSCModel.get( 0 ), 1, g_uiMaxCUDepth - 1 );
  ruiAlfCtrlDepth = uiSymbol;
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tAlfCtrlDepth\n" )
}

Void TDecSbac::parseAlfCtrlFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( !m_bAlfCtrl )
  {
    return;
  }

  if( uiDepth > m_uiMaxAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth( uiAbsPartIdx, m_uiMaxAlfCtrlDepth ) )
  {
    return;
  }

  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUAlfCtrlFlagSCModel.get( 0, 0, pcCU->getCtxAlfCtrlFlag( uiAbsPartIdx ) ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tAlfCtrlFlag\n" )

  if( uiDepth > m_uiMaxAlfCtrlDepth )
  {
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, m_uiMaxAlfCtrlDepth );
  }
  else
  {
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, uiDepth );
  }
}
Void TDecSbac::parseAlfFlag (UInt& ruiVal)
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cALFFlagSCModel.get( 0, 0, 0 ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tAlfFlag\n" )

  ruiVal = uiSymbol;
}

Void TDecSbac::parseAlfFlagNum( UInt& ruiVal, UInt minValue, UInt depth )
{
  UInt uiLength = 0;
  UInt maxValue = (minValue << (depth*2));
  UInt temp = maxValue - minValue;
  for(UInt i=0; i<32; i++)
  {
    if(temp&0x1)
    {
      uiLength = i+1;
    }
    temp = (temp >> 1);
  }
  ruiVal = 0;
  UInt uiBit;
  if(uiLength)
  {
    while( uiLength-- )
    {
      m_pcTDecBinIf->decodeBinEP( uiBit );
      ruiVal += uiBit << uiLength;
    }
  }
  else
  {
    ruiVal = 0;
  }
  ruiVal += minValue;
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tAlfFlagNum\n" )
}

Void TDecSbac::parseAlfCtrlFlag( UInt &ruiAlfCtrlFlag )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUAlfCtrlFlagSCModel.get( 0, 0, 0 ) );
  ruiAlfCtrlFlag = uiSymbol;
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tAlfCtrlFlag\n" )
}

Void TDecSbac::parseAlfUvlc (UInt& ruiVal)
{
  UInt uiCode;
  Int  i;

  m_pcTDecBinIf->decodeBin( uiCode, m_cALFUvlcSCModel.get( 0, 0, 0 ) );
  if ( uiCode == 0 )
  {
    ruiVal = 0;
    return;
  }

  i=1;
  while (1)
  {
    m_pcTDecBinIf->decodeBin( uiCode, m_cALFUvlcSCModel.get( 0, 0, 1 ) );
    if ( uiCode == 0 ) break;
    i++;
  }

  ruiVal = i;
}

Void TDecSbac::parseAlfSvlc (Int&  riVal)
{
  UInt uiCode;
  Int  iSign;
  Int  i;

  m_pcTDecBinIf->decodeBin( uiCode, m_cALFSvlcSCModel.get( 0, 0, 0 ) );

  if ( uiCode == 0 )
  {
    riVal = 0;
    return;
  }

  // read sign
  m_pcTDecBinIf->decodeBin( uiCode, m_cALFSvlcSCModel.get( 0, 0, 1 ) );

  if ( uiCode == 0 ) iSign =  1;
  else               iSign = -1;

  // read magnitude
  i=1;
  while (1)
  {
    m_pcTDecBinIf->decodeBin( uiCode, m_cALFSvlcSCModel.get( 0, 0, 2 ) );
    if ( uiCode == 0 ) break;
    i++;
  }

  riVal = i*iSign;
}
#endif
#if INIT_PREVFRAME
Void TDecSbac::loadContextsFromPrev (TComStats* apcStats, SliceType eSliceType, Int iQPIdx, Bool bFromGloble, Int iQPIdxRst, Bool bAfterLastISlice )
{
  if(bFromGloble)
  {
    if(iQPIdx==-1 || (bAfterLastISlice && !apcStats->aaQPUsed[eSliceType][iQPIdxRst].uiResetInit))
    {
      return;
    }
    Int iCtxNr = getCtxNumber() - NUM_ALF_CTX;
    for(UInt i = 0; i < iCtxNr; i++)
    {
      m_contextModels[i].setState(apcStats->m_uiCtxProbIdx[eSliceType][iQPIdx][0][i]);
    }
  }
  else
  {
    Int iCtxNr = getCtxNumber() - NUM_ALF_CTX;
    for(UInt i = 0; i < iCtxNr; i++)
    {
      apcStats->m_uiCtxProbIdx[eSliceType][iQPIdx][0][i] = m_contextModels[i].getState();
    }
  }
}
#endif
//! \}
