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

/** \file     TEncSearch.cpp
 \brief    encoder search class
 */

#include "TLibCommon/TypeDef.h"
#include "TLibCommon/TComRom.h"
#include "TLibCommon/TComMotionInfo.h"
#include "TEncSearch.h"
#include <math.h>

//! \ingroup TLibEncoder
//! \{
#if KLT_COMMON
extern short **g_ppsEigenVector[USE_MORE_BLOCKSIZE_DEPTH_MAX];
#endif

#if QC_EMT_INTRA_FAST
#define QC_EMT_INTRA_SKIP_CHECK_4x4_THR          1.47  // Skip checking   4x4 Intra modes using the R-D cost in the DCT2-pass 
#define QC_EMT_INTRA_SKIP_CHECK_8x8_THR          1.28  // Skip checking   8x8 Intra modes using the R-D cost in the DCT2-pass 
#define QC_EMT_INTRA_SKIP_CHECK_16x16_THR        1.12  // Skip checking 16x16 Intra modes using the R-D cost in the DCT2-pass 
#define QC_EMT_INTRA_SKIP_CHECK_32x32_THR        1.06  // Skip checking 32x32 Intra modes using the R-D cost in the DCT2-pass 
#endif
#if QC_EMT_INTER_FAST
#define QC_EMT_INTER_SKIP_CHECK_EMT_THR          1.1  // Skip checking EMT transforms
#endif

static const TComMv s_acMvRefineH[9] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 )  // 8
};

static const TComMv s_acMvRefineQ[9] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 )  // 8
};

static const UInt s_auiDFilter[9] =
{
  0, 1, 0,
  2, 3, 2,
  0, 1, 0
};

TEncSearch::TEncSearch()
{
  m_ppcQTTempCoeffY  = NULL;
  m_ppcQTTempCoeffCb = NULL;
  m_ppcQTTempCoeffCr = NULL;
  m_pcQTTempCoeffY   = NULL;
  m_pcQTTempCoeffCb  = NULL;
  m_pcQTTempCoeffCr  = NULL;
#if ADAPTIVE_QP_SELECTION
  m_ppcQTTempArlCoeffY  = NULL;
  m_ppcQTTempArlCoeffCb = NULL;
  m_ppcQTTempArlCoeffCr = NULL;
  m_pcQTTempArlCoeffY   = NULL;
  m_pcQTTempArlCoeffCb  = NULL;
  m_pcQTTempArlCoeffCr  = NULL;
#endif
#if QC_EMT
  m_puhQTTempEmtTuIdx   = NULL;
  m_puhQTTempEmtCuFlag  = NULL;
#endif
  m_puhQTTempTrIdx   = NULL;
  m_puhQTTempCbf[0] = m_puhQTTempCbf[1] = m_puhQTTempCbf[2] = NULL;
  m_pcQTTempTComYuv  = NULL;
#if INTER_KLT
  m_pcQTTempTComYuvRec = NULL;
#endif
  m_pcEncCfg = NULL;
  m_pcEntropyCoder = NULL;
  m_pTempPel = NULL;
  m_pSharedPredTransformSkip[0] = m_pSharedPredTransformSkip[1] = m_pSharedPredTransformSkip[2] = NULL;
  m_pcQTTempTUCoeffY   = NULL;
  m_pcQTTempTUCoeffCb  = NULL;
  m_pcQTTempTUCoeffCr  = NULL;
#if ADAPTIVE_QP_SELECTION
  m_ppcQTTempTUArlCoeffY  = NULL;
  m_ppcQTTempTUArlCoeffCb = NULL;
  m_ppcQTTempTUArlCoeffCr = NULL;
#endif
  m_puhQTTempTransformSkipFlag[0] = NULL;
  m_puhQTTempTransformSkipFlag[1] = NULL;
  m_puhQTTempTransformSkipFlag[2] = NULL;
#if KLT_COMMON
  m_puhQTTempKLTFlag[0] = NULL;
  m_puhQTTempKLTFlag[1] = NULL;
  m_puhQTTempKLTFlag[2] = NULL;
#endif
  setWpScalingDistParam( NULL, -1, REF_PIC_LIST_X );

#if QC_SUB_PU_TMVP
#if QC_SUB_PU_TMVP_EXT
  m_pMvFieldSP[0] = new TComMvField[MAX_NUM_SPU_W*MAX_NUM_SPU_W*2];
  m_pMvFieldSP[1] = new TComMvField[MAX_NUM_SPU_W*MAX_NUM_SPU_W*2];
  m_phInterDirSP[0] = new UChar[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
  m_phInterDirSP[1] = new UChar[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
#else
  m_pMvFieldSP = new TComMvField[MAX_NUM_SPU_W*MAX_NUM_SPU_W*2];
  m_phInterDirSP = new UChar[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
  assert( m_pMvFieldSP != NULL && m_phInterDirSP != NULL );
#endif
#endif
#if QC_FRUC_MERGE
  m_pMvFieldFRUC = new TComMvField[MAX_NUM_SPU_W*MAX_NUM_SPU_W*2];
  m_phInterDirFRUC = new UChar[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
  m_phFRUCRefineDist[0] = new UChar[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
  m_phFRUCRefineDist[1] = new UChar[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
  m_phFRUCSBlkRefineDist[0] = new UChar[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
  m_phFRUCSBlkRefineDist[1] = new UChar[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
  assert( m_pMvFieldFRUC != NULL && m_phInterDirFRUC != NULL );
#endif
}

TEncSearch::~TEncSearch()
{
  if ( m_pTempPel )
  {
    delete [] m_pTempPel;
    m_pTempPel = NULL;
  }
  
  if ( m_pcEncCfg )
  {
    const UInt uiNumLayersAllocated = m_pcEncCfg->getQuadtreeTULog2MaxSize()-m_pcEncCfg->getQuadtreeTULog2MinSize()+1;
    for( UInt ui = 0; ui < uiNumLayersAllocated; ++ui )
    {
      delete[] m_ppcQTTempCoeffY[ui];
      delete[] m_ppcQTTempCoeffCb[ui];
      delete[] m_ppcQTTempCoeffCr[ui];
#if ADAPTIVE_QP_SELECTION
      delete[] m_ppcQTTempArlCoeffY[ui];
      delete[] m_ppcQTTempArlCoeffCb[ui];
      delete[] m_ppcQTTempArlCoeffCr[ui];
#endif
      m_pcQTTempTComYuv[ui].destroy();
#if INTER_KLT
    m_pcQTTempTComYuvRec[ui].destroy();
#endif
    }
  }
  delete[] m_ppcQTTempCoeffY;
  delete[] m_ppcQTTempCoeffCb;
  delete[] m_ppcQTTempCoeffCr;
  delete[] m_pcQTTempCoeffY;
  delete[] m_pcQTTempCoeffCb;
  delete[] m_pcQTTempCoeffCr;
#if ADAPTIVE_QP_SELECTION
  delete[] m_ppcQTTempArlCoeffY;
  delete[] m_ppcQTTempArlCoeffCb;
  delete[] m_ppcQTTempArlCoeffCr;
  delete[] m_pcQTTempArlCoeffY;
  delete[] m_pcQTTempArlCoeffCb;
  delete[] m_pcQTTempArlCoeffCr;
#endif
#if QC_EMT
  delete[] m_puhQTTempEmtTuIdx;
  delete[] m_puhQTTempEmtCuFlag;
#endif
  delete[] m_puhQTTempTrIdx;
  delete[] m_puhQTTempCbf[0];
  delete[] m_puhQTTempCbf[1];
  delete[] m_puhQTTempCbf[2];
  delete[] m_pcQTTempTComYuv;
#if INTER_KLT
  delete[] m_pcQTTempTComYuvRec;
#endif
  delete[] m_pSharedPredTransformSkip[0];
  delete[] m_pSharedPredTransformSkip[1];
  delete[] m_pSharedPredTransformSkip[2];
  delete[] m_pcQTTempTUCoeffY;
  delete[] m_pcQTTempTUCoeffCb;
  delete[] m_pcQTTempTUCoeffCr;
#if ADAPTIVE_QP_SELECTION
  delete[] m_ppcQTTempTUArlCoeffY;
  delete[] m_ppcQTTempTUArlCoeffCb;
  delete[] m_ppcQTTempTUArlCoeffCr;
#endif
  delete[] m_puhQTTempTransformSkipFlag[0];
  delete[] m_puhQTTempTransformSkipFlag[1];
  delete[] m_puhQTTempTransformSkipFlag[2];
#if KLT_COMMON
  delete[] m_puhQTTempKLTFlag[0];
  delete[] m_puhQTTempKLTFlag[1];
  delete[] m_puhQTTempKLTFlag[2];
#endif
#if QC_LMCHROMA
  m_pcQTTempResiTComYuv.destroy();
#endif
  m_pcQTTempTransformSkipTComYuv.destroy();
  m_tmpYuvPred.destroy();

#if QC_SUB_PU_TMVP
#if QC_SUB_PU_TMVP_EXT
  for (UInt ui=0;ui<2;ui++)
  {
    if( m_pMvFieldSP[ui] != NULL )
    {
      delete [] m_pMvFieldSP[ui];
      m_pMvFieldSP[ui] = NULL;
    }
    if( m_phInterDirSP[ui] != NULL )
    {
      delete [] m_phInterDirSP[ui];
      m_phInterDirSP[ui] = NULL;
    }
  }
#else
  if( m_pMvFieldSP != NULL )
  {
    delete [] m_pMvFieldSP;
    m_pMvFieldSP = NULL;
  }
  if( m_phInterDirSP != NULL )
  {
    delete [] m_phInterDirSP;
    m_phInterDirSP = NULL;
  }
#endif
#endif
#if QC_FRUC_MERGE
  if( m_pMvFieldFRUC != NULL )
  {
    delete [] m_pMvFieldFRUC;
    m_pMvFieldFRUC = NULL;
  }
  if( m_phInterDirFRUC != NULL )
  {
    delete [] m_phInterDirFRUC;
    m_phInterDirFRUC = NULL;
  }
  if( m_phFRUCRefineDist[0] != NULL )
  {
    delete [] m_phFRUCRefineDist[0];
    m_phFRUCRefineDist[0] = NULL;
  }
  if( m_phFRUCRefineDist[1] != NULL )
  {
    delete [] m_phFRUCRefineDist[1];
    m_phFRUCRefineDist[1] = NULL;
  }
  if( m_phFRUCSBlkRefineDist[0] != NULL )
  {
    delete [] m_phFRUCSBlkRefineDist[0];
    m_phFRUCSBlkRefineDist[0] = NULL;
  }
  if( m_phFRUCSBlkRefineDist[1] != NULL )
  {
    delete [] m_phFRUCSBlkRefineDist[1];
    m_phFRUCSBlkRefineDist[1] = NULL;
  }
#endif
}

void TEncSearch::init(TEncCfg*      pcEncCfg,
                      TComTrQuant*  pcTrQuant,
                      Int           iSearchRange,
                      Int           bipredSearchRange,
                      Int           iFastSearch,
                      Int           iMaxDeltaQP,
                      TEncEntropy*  pcEntropyCoder,
                      TComRdCost*   pcRdCost,
                      TEncSbac*** pppcRDSbacCoder,
                      TEncSbac*   pcRDGoOnSbacCoder
                      )
{
  m_pcEncCfg             = pcEncCfg;
  m_pcTrQuant            = pcTrQuant;
  m_iSearchRange         = iSearchRange;
  m_bipredSearchRange    = bipredSearchRange;
  m_iFastSearch          = iFastSearch;
  m_iMaxDeltaQP          = iMaxDeltaQP;
  m_pcEntropyCoder       = pcEntropyCoder;
  m_pcRdCost             = pcRdCost;
  
  m_pppcRDSbacCoder     = pppcRDSbacCoder;
  m_pcRDGoOnSbacCoder   = pcRDGoOnSbacCoder;

  for (Int iDir = 0; iDir < 2; iDir++)
  {
    for (Int iRefIdx = 0; iRefIdx < 33; iRefIdx++)
    {
      m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange;
    }
  }
  
  m_puiDFilter = s_auiDFilter + 4;
  
  // initialize motion cost
#if !FIX203
  m_pcRdCost->initRateDistortionModel( m_iSearchRange << 2 );
#endif
  
  for( Int iNum = 0; iNum < AMVP_MAX_NUM_CANDS+1; iNum++)
  {
    for( Int iIdx = 0; iIdx < AMVP_MAX_NUM_CANDS; iIdx++)
    {
      if (iIdx < iNum)
        m_auiMVPIdxCost[iIdx][iNum] = xGetMvpIdxBits(iIdx, iNum);
      else
        m_auiMVPIdxCost[iIdx][iNum] = MAX_INT;
    }
  }
  
  initTempBuff();
  
  m_pTempPel = new Pel[g_uiMaxCUWidth*g_uiMaxCUHeight];
  
  const UInt uiNumLayersToAllocate = pcEncCfg->getQuadtreeTULog2MaxSize()-pcEncCfg->getQuadtreeTULog2MinSize()+1;
  m_ppcQTTempCoeffY  = new TCoeff*[uiNumLayersToAllocate];
  m_ppcQTTempCoeffCb = new TCoeff*[uiNumLayersToAllocate];
  m_ppcQTTempCoeffCr = new TCoeff*[uiNumLayersToAllocate];
  m_pcQTTempCoeffY   = new TCoeff [g_uiMaxCUWidth*g_uiMaxCUHeight   ];
  m_pcQTTempCoeffCb  = new TCoeff [g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
  m_pcQTTempCoeffCr  = new TCoeff [g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
#if ADAPTIVE_QP_SELECTION
  m_ppcQTTempArlCoeffY  = new Int*[uiNumLayersToAllocate];
  m_ppcQTTempArlCoeffCb = new Int*[uiNumLayersToAllocate];
  m_ppcQTTempArlCoeffCr = new Int*[uiNumLayersToAllocate];
  m_pcQTTempArlCoeffY   = new Int [g_uiMaxCUWidth*g_uiMaxCUHeight   ];
  m_pcQTTempArlCoeffCb  = new Int [g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
  m_pcQTTempArlCoeffCr  = new Int [g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
#endif
  
  const UInt uiNumPartitions = 1<<(g_uiMaxCUDepth<<1);
#if QC_EMT
  m_puhQTTempEmtTuIdx  = new UChar  [uiNumPartitions];
  m_puhQTTempEmtCuFlag = new UChar  [uiNumPartitions];
#endif
  m_puhQTTempTrIdx   = new UChar  [uiNumPartitions];
  m_puhQTTempCbf[0]  = new UChar  [uiNumPartitions];
  m_puhQTTempCbf[1]  = new UChar  [uiNumPartitions];
  m_puhQTTempCbf[2]  = new UChar  [uiNumPartitions];
  m_pcQTTempTComYuv  = new TComYuv[uiNumLayersToAllocate];
#if INTER_KLT
  m_pcQTTempTComYuvRec = new TComYuv[uiNumLayersToAllocate];
#endif
  for( UInt ui = 0; ui < uiNumLayersToAllocate; ++ui )
  {
    m_ppcQTTempCoeffY[ui]  = new TCoeff[g_uiMaxCUWidth*g_uiMaxCUHeight   ];
    m_ppcQTTempCoeffCb[ui] = new TCoeff[g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
    m_ppcQTTempCoeffCr[ui] = new TCoeff[g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempArlCoeffY[ui]  = new Int[g_uiMaxCUWidth*g_uiMaxCUHeight   ];
    m_ppcQTTempArlCoeffCb[ui] = new Int[g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
    m_ppcQTTempArlCoeffCr[ui] = new Int[g_uiMaxCUWidth*g_uiMaxCUHeight>>2];
#endif
    m_pcQTTempTComYuv[ui].create( g_uiMaxCUWidth, g_uiMaxCUHeight );
#if INTER_KLT
  m_pcQTTempTComYuvRec[ui].create(g_uiMaxCUWidth, g_uiMaxCUHeight);
#endif
  }
  m_pSharedPredTransformSkip[0] = new Pel[MAX_TS_WIDTH*MAX_TS_HEIGHT];
  m_pSharedPredTransformSkip[1] = new Pel[MAX_TS_WIDTH*MAX_TS_HEIGHT];
  m_pSharedPredTransformSkip[2] = new Pel[MAX_TS_WIDTH*MAX_TS_HEIGHT];
  m_pcQTTempTUCoeffY  = new TCoeff[MAX_TS_WIDTH*MAX_TS_HEIGHT];
  m_pcQTTempTUCoeffCb = new TCoeff[MAX_TS_WIDTH*MAX_TS_HEIGHT];
  m_pcQTTempTUCoeffCr = new TCoeff[MAX_TS_WIDTH*MAX_TS_HEIGHT];
#if ADAPTIVE_QP_SELECTION
  m_ppcQTTempTUArlCoeffY  = new Int[MAX_TS_WIDTH*MAX_TS_HEIGHT];
  m_ppcQTTempTUArlCoeffCb = new Int[MAX_TS_WIDTH*MAX_TS_HEIGHT];
  m_ppcQTTempTUArlCoeffCr = new Int[MAX_TS_WIDTH*MAX_TS_HEIGHT];
#endif
  m_pcQTTempTransformSkipTComYuv.create( g_uiMaxCUWidth, g_uiMaxCUHeight );

#if QC_LMCHROMA
  m_pcQTTempResiTComYuv.create( g_uiMaxCUWidth, g_uiMaxCUHeight );
#endif

  m_puhQTTempTransformSkipFlag[0] = new UChar  [uiNumPartitions];
  m_puhQTTempTransformSkipFlag[1] = new UChar  [uiNumPartitions];
  m_puhQTTempTransformSkipFlag[2] = new UChar  [uiNumPartitions];
#if KLT_COMMON
  m_puhQTTempKLTFlag[0] = new UChar[uiNumPartitions];
  m_puhQTTempKLTFlag[1] = new UChar[uiNumPartitions];
  m_puhQTTempKLTFlag[2] = new UChar[uiNumPartitions];
#endif
#if QC_LARGE_CTU
  m_tmpYuvPred.create(g_uiMaxCUWidth, g_uiMaxCUHeight);
#else
  m_tmpYuvPred.create(MAX_CU_SIZE, MAX_CU_SIZE);
#endif
}

#if FASTME_SMOOTHER_MV
#define FIRSTSEARCHSTOP     1
#else
#define FIRSTSEARCHSTOP     0
#endif

#define TZ_SEARCH_CONFIGURATION                                                                                 \
const Int  iRaster                  = 5;  /* TZ soll von aussen ?ergeben werden */                            \
const Bool bTestOtherPredictedMV    = 0;                                                                      \
const Bool bTestZeroVector          = 1;                                                                      \
const Bool bTestZeroVectorStart     = 0;                                                                      \
const Bool bTestZeroVectorStop      = 0;                                                                      \
const Bool bFirstSearchDiamond      = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bFirstSearchStop         = FIRSTSEARCHSTOP;                                                        \
const UInt uiFirstSearchRounds      = 3;  /* first search stop X rounds after best match (must be >=1) */     \
const Bool bEnableRasterSearch      = 1;                                                                      \
const Bool bAlwaysRasterSearch      = 0;  /* ===== 1: BETTER but factor 2 slower ===== */                     \
const Bool bRasterRefinementEnable  = 0;  /* enable either raster refinement or star refinement */            \
const Bool bRasterRefinementDiamond = 0;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bStarRefinementEnable    = 1;  /* enable either star refinement or raster refinement */            \
const Bool bStarRefinementDiamond   = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bStarRefinementStop      = 0;                                                                      \
const UInt uiStarRefinementRounds   = 2;  /* star refinement stop X rounds after best match (must be >=1) */  \


__inline Void TEncSearch::xTZSearchHelp( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, const Int iSearchX, const Int iSearchY, const UChar ucPointNr, const UInt uiDistance )
{
  UInt  uiSad;
  
  Pel*  piRefSrch;
  
  piRefSrch = rcStruct.piRefY + iSearchY * rcStruct.iYStride + iSearchX;
#if QC_IC
  m_cDistParam.bMRFlag = pcPatternKey->getMRFlag();
#endif  
  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefSrch, rcStruct.iYStride,  m_cDistParam );
  
  // fast encoder decision: use subsampled SAD when rows > 8 for integer ME
  if ( m_pcEncCfg->getUseFastEnc() )
  {
    if ( m_cDistParam.iRows > 8 )
    {
      m_cDistParam.iSubShift = 1;
    }
  }

  setDistParamComp(0);  // Y component

  // distortion
  m_cDistParam.bitDepth = g_bitDepthY;
  uiSad = m_cDistParam.DistFunc( &m_cDistParam );
  
  // motion cost
#if QC_IMV
  uiSad += m_pcRdCost->getCost( iSearchX, iSearchY, pcPatternKey->getImvFlag() );
#else
  uiSad += m_pcRdCost->getCost( iSearchX, iSearchY );
#endif
  
  if( uiSad < rcStruct.uiBestSad )
  {
    rcStruct.uiBestSad      = uiSad;
    rcStruct.iBestX         = iSearchX;
    rcStruct.iBestY         = iSearchY;
    rcStruct.uiBestDistance = uiDistance;
    rcStruct.uiBestRound    = 0;
    rcStruct.ucPointNr      = ucPointNr;
  }
}

__inline Void TEncSearch::xTZ2PointSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  Int iStartX = rcStruct.iBestX;
  Int iStartY = rcStruct.iBestY;
  switch( rcStruct.ucPointNr )
  {
    case 1:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY, 0, 2 );
      }
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY - 1, 0, 2 );
      }
    }
      break;
    case 2:
    {
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        if ( (iStartX - 1) >= iSrchRngHorLeft )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
        }
        if ( (iStartX + 1) <= iSrchRngHorRight )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
        }
      }
    }
      break;
    case 3:
    {
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY - 1, 0, 2 );
      }
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY, 0, 2 );
      }
    }
      break;
    case 4:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        if ( (iStartY + 1) <= iSrchRngVerBottom )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
        }
        if ( (iStartY - 1) >= iSrchRngVerTop )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
        }
      }
    }
      break;
    case 5:
    {
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        if ( (iStartY - 1) >= iSrchRngVerTop )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
        }
        if ( (iStartY + 1) <= iSrchRngVerBottom )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
        }
      }
    }
      break;
    case 6:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY , 0, 2 );
      }
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY + 1, 0, 2 );
      }
    }
      break;
    case 7:
    {
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        if ( (iStartX - 1) >= iSrchRngHorLeft )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
        }
        if ( (iStartX + 1) <= iSrchRngHorRight )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
        }
      }
    }
      break;
    case 8:
    {
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY, 0, 2 );
      }
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY + 1, 0, 2 );
      }
    }
      break;
    default:
    {
      assert( false );
    }
      break;
  } // switch( rcStruct.ucPointNr )
}

__inline Void TEncSearch::xTZ8PointSquareSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  assert( iDist != 0 );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;
  
  if ( iTop >= iSrchRngVerTop ) // check top
  {
    if ( iLeft >= iSrchRngHorLeft ) // check top left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iTop, 1, iDist );
    }
    // top middle
    xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
    
    if ( iRight <= iSrchRngHorRight ) // check top right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iTop, 3, iDist );
    }
  } // check top
  if ( iLeft >= iSrchRngHorLeft ) // check middle left
  {
    xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
  }
  if ( iRight <= iSrchRngHorRight ) // check middle right
  {
    xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
  }
  if ( iBottom <= iSrchRngVerBottom ) // check bottom
  {
    if ( iLeft >= iSrchRngHorLeft ) // check bottom left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iBottom, 6, iDist );
    }
    // check bottom middle
    xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
    
    if ( iRight <= iSrchRngHorRight ) // check bottom right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iBottom, 8, iDist );
    }
  } // check bottom
}

__inline Void TEncSearch::xTZ8PointDiamondSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  assert ( iDist != 0 );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;
  
  if ( iDist == 1 ) // iDist == 1
  {
    if ( iTop >= iSrchRngVerTop ) // check top
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
    }
    if ( iLeft >= iSrchRngHorLeft ) // check middle left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
    }
    if ( iRight <= iSrchRngHorRight ) // check middle right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
    }
    if ( iBottom <= iSrchRngVerBottom ) // check bottom
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
    }
  }
  else // if (iDist != 1)
  {
    if ( iDist <= 8 )
    {
      const Int iTop_2      = iStartY - (iDist>>1);
      const Int iBottom_2   = iStartY + (iDist>>1);
      const Int iLeft_2     = iStartX - (iDist>>1);
      const Int iRight_2    = iStartX + (iDist>>1);
      
      if (  iTop >= iSrchRngVerTop && iLeft >= iSrchRngHorLeft &&
          iRight <= iSrchRngHorRight && iBottom <= iSrchRngVerBottom ) // check border
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX,  iTop,      2, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2,  iTop_2,    1, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iTop_2,    3, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft,    iStartY,   4, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight,   iStartY,   5, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2,  iBottom_2, 6, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iBottom_2, 8, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX,  iBottom,   7, iDist    );
      }
      else // check border
      {
        if ( iTop >= iSrchRngVerTop ) // check top
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
        }
        if ( iTop_2 >= iSrchRngVerTop ) // check half top
        {
          if ( iLeft_2 >= iSrchRngHorLeft ) // check half left
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2, iTop_2, 1, (iDist>>1) );
          }
          if ( iRight_2 <= iSrchRngHorRight ) // check half right
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iTop_2, 3, (iDist>>1) );
          }
        } // check half top
        if ( iLeft >= iSrchRngHorLeft ) // check left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
        }
        if ( iRight <= iSrchRngHorRight ) // check right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
        }
        if ( iBottom_2 <= iSrchRngVerBottom ) // check half bottom
        {
          if ( iLeft_2 >= iSrchRngHorLeft ) // check half left
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2, iBottom_2, 6, (iDist>>1) );
          }
          if ( iRight_2 <= iSrchRngHorRight ) // check half right
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iBottom_2, 8, (iDist>>1) );
          }
        } // check half bottom
        if ( iBottom <= iSrchRngVerBottom ) // check bottom
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
        }
      } // check border
    }
    else // iDist > 8
    {
      if ( iTop >= iSrchRngVerTop && iLeft >= iSrchRngHorLeft &&
          iRight <= iSrchRngHorRight && iBottom <= iSrchRngVerBottom ) // check border
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop,    0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft,   iStartY, 0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight,  iStartY, 0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 0, iDist );
        for ( Int index = 1; index < 4; index++ )
        {
          Int iPosYT = iTop    + ((iDist>>2) * index);
          Int iPosYB = iBottom - ((iDist>>2) * index);
          Int iPosXL = iStartX - ((iDist>>2) * index);
          Int iPosXR = iStartX + ((iDist>>2) * index);
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYT, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYT, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYB, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYB, 0, iDist );
        }
      }
      else // check border
      {
        if ( iTop >= iSrchRngVerTop ) // check top
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 0, iDist );
        }
        if ( iLeft >= iSrchRngHorLeft ) // check left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 0, iDist );
        }
        if ( iRight <= iSrchRngHorRight ) // check right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 0, iDist );
        }
        if ( iBottom <= iSrchRngVerBottom ) // check bottom
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 0, iDist );
        }
        for ( Int index = 1; index < 4; index++ )
        {
          Int iPosYT = iTop    + ((iDist>>2) * index);
          Int iPosYB = iBottom - ((iDist>>2) * index);
          Int iPosXL = iStartX - ((iDist>>2) * index);
          Int iPosXR = iStartX + ((iDist>>2) * index);
          
          if ( iPosYT >= iSrchRngVerTop ) // check top
          {
            if ( iPosXL >= iSrchRngHorLeft ) // check left
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYT, 0, iDist );
            }
            if ( iPosXR <= iSrchRngHorRight ) // check right
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYT, 0, iDist );
            }
          } // check top
          if ( iPosYB <= iSrchRngVerBottom ) // check bottom
          {
            if ( iPosXL >= iSrchRngHorLeft ) // check left
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYB, 0, iDist );
            }
            if ( iPosXR <= iSrchRngHorRight ) // check right
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYB, 0, iDist );
            }
          } // check bottom
        } // for ...
      } // check border
    } // iDist <= 8
  } // iDist == 1
}

//<--

UInt TEncSearch::xPatternRefinement( TComPattern* pcPatternKey,
                                    TComMv baseRefMv,
                                    Int iFrac, TComMv& rcMvFrac 
                                    )
{
  UInt  uiDist;
  UInt  uiDistBest  = MAX_UINT;
  UInt  uiDirecBest = 0;
  
  Pel*  piRefPos;
  Int iRefStride = m_filteredBlock[0][0].getStride();
  m_pcRdCost->setDistParam( pcPatternKey, m_filteredBlock[0][0].getLumaAddr(), iRefStride, 1, m_cDistParam, m_pcEncCfg->getUseHADME() );
  
  const TComMv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);
  
  for (UInt i = 0; i < 9; i++)
  {
#if QC_IMV
    if( pcPatternKey->getImvFlag() && ( i > 0 || iFrac == 1 ) )
    {
      continue;
    }
#endif
    TComMv cMvTest = pcMvRefine[i];
    cMvTest += baseRefMv;
    
    Int horVal = cMvTest.getHor() * iFrac;
    Int verVal = cMvTest.getVer() * iFrac;
    piRefPos = m_filteredBlock[ verVal & 3 ][ horVal & 3 ].getLumaAddr();
    if ( horVal == 2 && ( verVal & 1 ) == 0 )
    {
      piRefPos += 1;
    }
    if ( ( horVal & 1 ) == 0 && verVal == 2 )
    {
      piRefPos += iRefStride;
    }
    cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;

    setDistParamComp(0);  // Y component
#if QC_IC
    m_cDistParam.bMRFlag = pcPatternKey->getMRFlag();
#endif
    m_cDistParam.pCur = piRefPos;
    m_cDistParam.bitDepth = g_bitDepthY;
    uiDist = m_cDistParam.DistFunc( &m_cDistParam );
    uiDist += m_pcRdCost->getCost( cMvTest.getHor(), cMvTest.getVer() 
#if QC_IMV
      , pcPatternKey->getImvFlag()
#endif
      );
    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
    }
  }
  
  rcMvFrac = pcMvRefine[uiDirecBest];
  
  return uiDistBest;
}

Void
TEncSearch::xEncSubdivCbfQT( TComDataCU*  pcCU,
                            UInt         uiTrDepth,
                            UInt         uiAbsPartIdx,
                            Bool         bLuma,
                            Bool         bChroma )
{
  UInt  uiFullDepth     = pcCU->getDepth(0) + uiTrDepth;
  UInt  uiTrMode        = pcCU->getTransformIdx( uiAbsPartIdx );
  UInt  uiSubdiv        = ( uiTrMode > uiTrDepth ? 1 : 0 );
  UInt  uiLog2TrafoSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()] + 2 - uiFullDepth;

  if( pcCU->getPredictionMode(0) == MODE_INTRA && pcCU->getPartitionSize(0) == SIZE_NxN && uiTrDepth == 0 )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2TrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2TrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    assert( !uiSubdiv );
  }
  else if( uiLog2TrafoSize == pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
  {
    assert( !uiSubdiv );
  }
  else
  {
    assert( uiLog2TrafoSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
    if( bLuma )
    {
#if QC_T64
      m_pcEntropyCoder->encodeTransformSubdivFlag( uiSubdiv, pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrafoSize );
#else
      m_pcEntropyCoder->encodeTransformSubdivFlag( uiSubdiv, 5 - uiLog2TrafoSize );
#endif
    }
  }
  
  if ( bChroma )
  {
    if( uiLog2TrafoSize > 2 )
    {
      if( uiTrDepth==0 || pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepth-1 ) )
      {
        m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepth );
      }
      if( uiTrDepth==0 || pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepth-1 ) )
      {
        m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepth );
      }
    }
  }

  if( uiSubdiv )
  {
    UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );

#if QC_EMT
    if( 0==uiTrDepth )
    {
      m_pcEntropyCoder->encodeEmtCuFlag( pcCU, uiAbsPartIdx, pcCU->getDepth( 0 ), true );
    }
#endif

    for( UInt uiPart = 0; uiPart < 4; uiPart++ )
    {
      xEncSubdivCbfQT( pcCU, uiTrDepth + 1, uiAbsPartIdx + uiPart * uiQPartNum, bLuma, bChroma );
    }
    return;
  }
  
#if QC_EMT
  if( 0==uiTrDepth )
  {
    m_pcEntropyCoder->encodeEmtCuFlag( pcCU, uiAbsPartIdx, pcCU->getDepth( 0 ), pcCU->getCbf( uiAbsPartIdx, TEXT_LUMA, uiTrMode ) ? true : false );
  }
#endif

  //===== Cbfs =====
  if( bLuma )
  {
    m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_LUMA,     uiTrMode );
  }
}

Void
TEncSearch::xEncCoeffQT( TComDataCU*  pcCU,
                        UInt         uiTrDepth,
                        UInt         uiAbsPartIdx,
                        TextType     eTextType,
                        Bool         bRealCoeff )
{
  UInt  uiFullDepth     = pcCU->getDepth(0) + uiTrDepth;
  UInt  uiTrMode        = pcCU->getTransformIdx( uiAbsPartIdx );
  UInt  uiSubdiv        = ( uiTrMode > uiTrDepth ? 1 : 0 );
  UInt  uiLog2TrafoSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()] + 2 - uiFullDepth;
  UInt  uiChroma        = ( eTextType != TEXT_LUMA ? 1 : 0 );
  
  if( uiSubdiv )
  {
    UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    for( UInt uiPart = 0; uiPart < 4; uiPart++ )
    {
      xEncCoeffQT( pcCU, uiTrDepth + 1, uiAbsPartIdx + uiPart * uiQPartNum, eTextType, bRealCoeff );
    }
    return;
  }
  
  if( eTextType != TEXT_LUMA && uiLog2TrafoSize == 2 )
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
  
  //===== coefficients =====
  UInt    uiWidth         = pcCU->getWidth  ( 0 ) >> ( uiTrDepth + uiChroma );
  UInt    uiHeight        = pcCU->getHeight ( 0 ) >> ( uiTrDepth + uiChroma );
  UInt    uiCoeffOffset   = ( pcCU->getPic()->getMinCUWidth() * pcCU->getPic()->getMinCUHeight() * uiAbsPartIdx ) >> ( uiChroma << 1 );
  UInt    uiQTLayer       = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrafoSize;
  TCoeff* pcCoeff         = 0;
  switch( eTextType )
  {
    case TEXT_LUMA:     pcCoeff = ( bRealCoeff ? pcCU->getCoeffY () : m_ppcQTTempCoeffY [uiQTLayer] );  break;
    case TEXT_CHROMA_U: pcCoeff = ( bRealCoeff ? pcCU->getCoeffCb() : m_ppcQTTempCoeffCb[uiQTLayer] );  break;
    case TEXT_CHROMA_V: pcCoeff = ( bRealCoeff ? pcCU->getCoeffCr() : m_ppcQTTempCoeffCr[uiQTLayer] );  break;
    default:            assert(0);
  }
  pcCoeff += uiCoeffOffset;
  
  m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeff, uiAbsPartIdx, uiWidth, uiHeight, uiFullDepth, eTextType );
}


Void
TEncSearch::xEncIntraHeader( TComDataCU*  pcCU,
                            UInt         uiTrDepth,
                            UInt         uiAbsPartIdx,
                            Bool         bLuma,
                            Bool         bChroma )
{
  if( bLuma )
  {
    // CU header
    if( uiAbsPartIdx == 0 )
    {
      if( !pcCU->getSlice()->isIntra() )
      {
        if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
        {
          m_pcEntropyCoder->encodeCUTransquantBypassFlag( pcCU, 0, true );
        }
        m_pcEntropyCoder->encodeSkipFlag( pcCU, 0, true );
        m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
#if CU_LEVEL_MPI 
   m_pcEntropyCoder->encodeMPIIdx( pcCU, 0, true );
#endif
    }
      
      m_pcEntropyCoder  ->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );

      if (pcCU->isIntra(0) && pcCU->getPartitionSize(0) == SIZE_2Nx2N )
      {
        m_pcEntropyCoder->encodeIPCMInfo( pcCU, 0, true );

        if ( pcCU->getIPCMFlag (0))
        {
          return;
        }
      }
    }
    // luma prediction mode
    if( pcCU->getPartitionSize(0) == SIZE_2Nx2N )
    {
      if( uiAbsPartIdx == 0 )
      {
        m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, 0 );
      }
    }
    else
    {
      UInt uiQNumParts = pcCU->getTotalNumPart() >> 2;
      if( uiTrDepth == 0 )
      {
        assert( uiAbsPartIdx == 0 );
        for( UInt uiPart = 0; uiPart < 4; uiPart++ )
        {
          m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiPart * uiQNumParts );
        }
      }
      else if( ( uiAbsPartIdx % uiQNumParts ) == 0 )
      {
        m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiAbsPartIdx );
      }
    }
  }
  if( bChroma )
  {
    // chroma prediction mode
    if( uiAbsPartIdx == 0 )
    {
      m_pcEntropyCoder->encodeIntraDirModeChroma( pcCU, 0, true );
    }
  }
}


UInt
TEncSearch::xGetIntraBitsQT( TComDataCU*  pcCU,
                            UInt         uiTrDepth,
                            UInt         uiAbsPartIdx,
                            Bool         bLuma,
                            Bool         bChroma,
                            Bool         bRealCoeff /* just for test */ )
{
  m_pcEntropyCoder->resetBits();
  xEncIntraHeader ( pcCU, uiTrDepth, uiAbsPartIdx, bLuma, bChroma );
  xEncSubdivCbfQT ( pcCU, uiTrDepth, uiAbsPartIdx, bLuma, bChroma );

  if( bLuma )
  {
    xEncCoeffQT   ( pcCU, uiTrDepth, uiAbsPartIdx, TEXT_LUMA,      bRealCoeff );
  }
  if( bChroma )
  {
    xEncCoeffQT   ( pcCU, uiTrDepth, uiAbsPartIdx, TEXT_CHROMA_U,  bRealCoeff );
    xEncCoeffQT   ( pcCU, uiTrDepth, uiAbsPartIdx, TEXT_CHROMA_V,  bRealCoeff );
  }
  UInt   uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  return uiBits;
}

UInt
TEncSearch::xGetIntraBitsQTChroma( TComDataCU*  pcCU,
                                  UInt         uiTrDepth,
                                  UInt         uiAbsPartIdx,
                                  UInt         uiChromaId,
                                  Bool         bRealCoeff /* just for test */ )
{
  m_pcEntropyCoder->resetBits();
  if( uiChromaId == TEXT_CHROMA_U)
  {
    xEncCoeffQT   ( pcCU, uiTrDepth, uiAbsPartIdx, TEXT_CHROMA_U,  bRealCoeff );
  }
  else if(uiChromaId == TEXT_CHROMA_V)
  {
    xEncCoeffQT   ( pcCU, uiTrDepth, uiAbsPartIdx, TEXT_CHROMA_V,  bRealCoeff );
  }

  UInt   uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  return uiBits;
}

Void
TEncSearch::xIntraCodingLumaBlk( TComDataCU* pcCU,
                                UInt        uiTrDepth,
                                UInt        uiAbsPartIdx,
                                TComYuv*    pcOrgYuv, 
                                TComYuv*    pcPredYuv, 
                                TComYuv*    pcResiYuv, 
                                UInt&       ruiDist,
#if QC_EMT_INTRA
                                UInt&       uiSigNum,
#endif
                                Int        default0Save1Load2 )
{
#if QC_EMT_INTRA
  UChar   ucTrIdx           = pcCU->getEmtCuFlag(uiAbsPartIdx) ? pcCU->getEmtTuIdx(uiAbsPartIdx) : ( pcCU->getSlice()->getSPS()->getUseIntraEMT() ? DCT2_EMT : DCT2_HEVC );
#endif
  UInt    uiLumaPredMode    = pcCU     ->getLumaIntraDir     ( uiAbsPartIdx );
  UInt    uiFullDepth       = pcCU     ->getDepth   ( 0 )  + uiTrDepth;
  UInt    uiWidth           = pcCU     ->getWidth   ( 0 ) >> uiTrDepth;
  UInt    uiHeight          = pcCU     ->getHeight  ( 0 ) >> uiTrDepth;
  UInt    uiStride          = pcOrgYuv ->getStride  ();
  Pel*    piOrg             = pcOrgYuv ->getLumaAddr( uiAbsPartIdx );
  Pel*    piPred            = pcPredYuv->getLumaAddr( uiAbsPartIdx );
  Pel*    piResi            = pcResiYuv->getLumaAddr( uiAbsPartIdx );
  Pel*    piReco            = pcPredYuv->getLumaAddr( uiAbsPartIdx );
  
  UInt    uiLog2TrSize      = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
  UInt    uiQTLayer         = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
  UInt    uiNumCoeffPerInc  = pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
  TCoeff* pcCoeff           = m_ppcQTTempCoeffY[ uiQTLayer ] + uiNumCoeffPerInc * uiAbsPartIdx;
#if ADAPTIVE_QP_SELECTION
  Int*    pcArlCoeff        = m_ppcQTTempArlCoeffY[ uiQTLayer ] + uiNumCoeffPerInc * uiAbsPartIdx;
#endif
  Pel*    piRecQt           = m_pcQTTempTComYuv[ uiQTLayer ].getLumaAddr( uiAbsPartIdx );
  UInt    uiRecQtStride     = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ();
  
  UInt    uiZOrder          = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
  Pel*    piRecIPred        = pcCU->getPic()->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), uiZOrder );
  UInt    uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getStride  ();
  Bool    useTransformSkip  = pcCU->getTransformSkip(uiAbsPartIdx, TEXT_LUMA);
  //===== init availability pattern =====
  Bool  bAboveAvail = false;
  Bool  bLeftAvail  = false;
#if QC_USE_65ANG_MODES
  UInt  uiPUWidth = pcCU->getWidth(uiAbsPartIdx) >> ( pcCU->getPartitionSize(uiAbsPartIdx)==SIZE_NxN ? 1 : 0);
#endif
  if(default0Save1Load2 != 2)
  {
    pcCU->getPattern()->initPattern   ( pcCU, uiTrDepth, uiAbsPartIdx );
    pcCU->getPattern()->initAdiPattern( pcCU, uiAbsPartIdx, uiTrDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail );
    //===== get prediction signal =====
    predIntraLumaAng( pcCU->getPattern(), uiLumaPredMode, piPred, uiStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail 
#if QC_INTRA_4TAP_FILTER
    , pcCU->getSlice()->getSPS()->getUse4TapIntraFilter()
#endif
#if INTRA_BOUNDARY_FILTER
    , pcCU->getSlice()->getSPS()->getUseBoundaryFilter()
#endif
#if QC_USE_65ANG_MODES
    , pcCU->getUseExtIntraAngModes(uiPUWidth)
#endif
#if CU_LEVEL_MPI
,  pcCU,  uiAbsPartIdx
#endif
      );
    // save prediction 
    if(default0Save1Load2 == 1)
    {
      Pel*  pPred   = piPred;
      Pel*  pPredBuf = m_pSharedPredTransformSkip[0];
      Int k = 0;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pPredBuf[ k ++ ] = pPred[ uiX ];
        }
        pPred += uiStride;
      }
    }
  }
  else 
  {
    // load prediction
    Pel*  pPred   = piPred;
    Pel*  pPredBuf = m_pSharedPredTransformSkip[0];
    Int k = 0;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pPred[ uiX ] = pPredBuf[ k ++ ];
      }
      pPred += uiStride;
    }
  }
  //===== get residual signal =====
  {
    // get residual
    Pel*  pOrg    = piOrg;
    Pel*  pPred   = piPred;
    Pel*  pResi   = piResi;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pResi[ uiX ] = pOrg[ uiX ] - pPred[ uiX ];
      }
      pOrg  += uiStride;
      pResi += uiStride;
      pPred += uiStride;
    }
  }
  
  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
#if QC_EMT_INTRA
  if( ( useTransformSkip? m_pcEncCfg->getUseRDOQTS():m_pcEncCfg->getUseRDOQ() ) && !(ucTrIdx>=1 && ucTrIdx<=3) )
#else
  if( useTransformSkip? m_pcEncCfg->getUseRDOQTS():m_pcEncCfg->getUseRDOQ())
#endif
  {
    m_pcEntropyCoder->estimateBit( m_pcTrQuant->m_pcEstBitsSbac, uiWidth, uiWidth, TEXT_LUMA );
  }
  //--- transform and quantization ---
  UInt uiAbsSum = 0;
  pcCU       ->setTrIdxSubParts ( uiTrDepth, uiAbsPartIdx, uiFullDepth );

  m_pcTrQuant->setQPforQuant    ( pcCU->getQP( 0 ), TEXT_LUMA, pcCU->getSlice()->getSPS()->getQpBDOffsetY(), 0 );

#if RDOQ_CHROMA_LAMBDA 
  m_pcTrQuant->selectLambda     (TEXT_LUMA);  
#endif

  m_pcTrQuant->transformNxN     ( pcCU, piResi, uiStride, pcCoeff, 
#if ADAPTIVE_QP_SELECTION
    pcArlCoeff, 
#endif
    uiWidth, uiHeight, uiAbsSum, TEXT_LUMA, uiAbsPartIdx,useTransformSkip 
#if QC_EMT_INTRA
    , ucTrIdx
#endif
    );
  
#if QC_EMT_INTRA
  if( ucTrIdx!=DCT2_EMT && ucTrIdx!=DCT2_HEVC )
  {
    uiSigNum = 0;
    for( UInt uiX = 0; uiX < uiHeight*uiWidth; uiX++ )
    {
      if( pcCoeff[uiX] )
      {
        uiSigNum ++;
        if( uiSigNum>g_iEmtSigNumThr )
        {
          break;
        }
      }
    }

    if( ucTrIdx!=0 && uiSigNum<=g_iEmtSigNumThr && !useTransformSkip )
    {
      return;
    }
  }
#endif

  //--- set coded block flag ---
  pcCU->setCbfSubParts          ( ( uiAbsSum ? 1 : 0 ) << uiTrDepth, TEXT_LUMA, uiAbsPartIdx, uiFullDepth );
  //--- inverse transform ---
  if( uiAbsSum )
  {
    Int scalingListType = 0 + g_eTTable[(Int)TEXT_LUMA];
    assert(scalingListType < SCALING_LIST_NUM);
    m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_LUMA,pcCU->getLumaIntraDir( uiAbsPartIdx ), piResi, uiStride, pcCoeff, uiWidth, uiHeight, scalingListType, useTransformSkip 
#if QC_EMT_INTRA
    , ucTrIdx
#endif
#if ROT_TR 
   , pcCU->getROTIdx(uiAbsPartIdx)
#endif
#if QC_USE_65ANG_MODES
    , pcCU->getUseExtIntraAngModes(uiPUWidth)
#endif
      );
  }
  else
  {
    Pel* pResi = piResi;
    memset( pcCoeff, 0, sizeof( TCoeff ) * uiWidth * uiHeight );
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      memset( pResi, 0, sizeof( Pel ) * uiWidth );
      pResi += uiStride;
    }
  }
  
  //===== reconstruction =====
  {
    Pel* pPred      = piPred;
    Pel* pResi      = piResi;
    Pel* pReco      = piReco;
    Pel* pRecQt     = piRecQt;
    Pel* pRecIPred  = piRecIPred;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pReco    [ uiX ] = ClipY( pPred[ uiX ] + pResi[ uiX ] );
        pRecQt   [ uiX ] = pReco[ uiX ];
        pRecIPred[ uiX ] = pReco[ uiX ];
      }
      pPred     += uiStride;
      pResi     += uiStride;
      pReco     += uiStride;
      pRecQt    += uiRecQtStride;
      pRecIPred += uiRecIPredStride;
    }
  }
  
  //===== update distortion =====
  ruiDist += m_pcRdCost->getDistPart(g_bitDepthY, piReco, uiStride, piOrg, uiStride, uiWidth, uiHeight );
}

#if INTRA_KLT
Bool
TEncSearch::xIntraCodingLumaBlkTM(  TComDataCU* pcCU,
                  UInt        uiTrDepth,
                  UInt        uiAbsPartIdx,
                  TComYuv*    pcOrgYuv,
                  TComYuv*    pcPredYuv,
                  TComYuv*    pcResiYuv,
                  UInt&       ruiDist,
                  Int        genPred0genPredAndtrainKLT1)
{
#if QC_EMT_INTRA
  UChar   ucTrIdx = DCT2_HEVC;
#endif 
  // UInt    uiLumaPredMode = pcCU->getLumaIntraDir(uiAbsPartIdx);
  UInt    uiFullDepth = pcCU->getDepth(0) + uiTrDepth;
  UInt    uiWidth = pcCU->getWidth(0) >> uiTrDepth;
  UInt    uiHeight = pcCU->getHeight(0) >> uiTrDepth;
  UInt    uiStride = pcOrgYuv->getStride();
  Pel*    piOrg = pcOrgYuv->getLumaAddr(uiAbsPartIdx);
  Pel*    piPred = pcPredYuv->getLumaAddr(uiAbsPartIdx);
  Pel*    piResi = pcResiYuv->getLumaAddr(uiAbsPartIdx);
  Pel*    piReco = pcPredYuv->getLumaAddr(uiAbsPartIdx);

  UInt    uiLog2TrSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth] + 2;
  UInt    uiQTLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
  UInt    uiNumCoeffPerInc = pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> (pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1);
  TCoeff* pcCoeff = m_ppcQTTempCoeffY[uiQTLayer] + uiNumCoeffPerInc * uiAbsPartIdx;
#if ADAPTIVE_QP_SELECTION
  Int*    pcArlCoeff = m_ppcQTTempArlCoeffY[uiQTLayer] + uiNumCoeffPerInc * uiAbsPartIdx;
#endif
  Pel*    piRecQt = m_pcQTTempTComYuv[uiQTLayer].getLumaAddr(uiAbsPartIdx);
  UInt    uiRecQtStride = m_pcQTTempTComYuv[uiQTLayer].getStride();

  UInt    uiZOrder = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
  Pel*    piRecIPred = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), uiZOrder);
  UInt    uiRecIPredStride = pcCU->getPic()->getPicYuvRec()->getStride();
  Bool    useTransformSkip = pcCU->getTransformSkip(uiAbsPartIdx, TEXT_LUMA);
  //===== init availability pattern =====
  // Bool  bAboveAvail = false;
  // Bool  bLeftAvail = false;

#if QC_USE_65ANG_MODES
  UInt  uiPUWidth = pcCU->getWidth(uiAbsPartIdx) >> (pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_NxN ? 1 : 0);
#endif

  // Pel *piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiAbsPartIdx);
  UInt uiBlkSize = uiWidth;
  UInt uiTarDepth = g_aucConvertToBit[uiBlkSize];
  // Pel   *pCurrStart = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), uiZOrder);
  // UInt  uiPicStride = pcCU->getPic()->getStride();

  UInt uiTempSize = g_uiDepth2IntraTempSize[uiTarDepth];
  m_pcTrQuant->getTargetTemplate(pcCU, uiAbsPartIdx, uiAbsPartIdx, pcPredYuv, uiBlkSize, uiTempSize);
  m_pcTrQuant->candidateSearchIntra(pcCU, uiAbsPartIdx, uiBlkSize, uiTempSize);

  Int foundCandiNum;
  Bool useKLT = false;
  Bool bSuccessful = m_pcTrQuant->generateTMPrediction(piPred, uiStride, uiBlkSize, uiTempSize, genPred0genPredAndtrainKLT1, foundCandiNum);
  if (bSuccessful == false || foundCandiNum < 1)
  {
    return false;
  }
  if (1 == genPred0genPredAndtrainKLT1 && bSuccessful)
  {
    useKLT = m_pcTrQuant->calcKLTIntra(piPred, uiStride, uiBlkSize, uiTempSize);
  }
  //===== get residual signal =====
  {
    // get residual
    Pel*  pOrg = piOrg;
    Pel*  pPred = piPred;
    Pel*  pResi = piResi;
    for (UInt uiY = 0; uiY < uiHeight; uiY++)
    {
      for (UInt uiX = 0; uiX < uiWidth; uiX++)
      {
        pResi[uiX] = pOrg[uiX] - pPred[uiX];
      }
      pOrg += uiStride;
      pResi += uiStride;
      pPred += uiStride;
    }
  }

  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
#if QC_EMT_INTRA
  if ((useTransformSkip ? m_pcEncCfg->getUseRDOQTS() : m_pcEncCfg->getUseRDOQ()) && !(ucTrIdx >= 1 && ucTrIdx <= 3))
#else
  if (useTransformSkip ? m_pcEncCfg->getUseRDOQTS() : m_pcEncCfg->getUseRDOQ())
#endif
  {
    m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, uiWidth, uiWidth, TEXT_LUMA);
  }
  //--- transform and quantization ---
  UInt uiAbsSum = 0;
  pcCU->setTrIdxSubParts(uiTrDepth, uiAbsPartIdx, uiFullDepth);

  m_pcTrQuant->setQPforQuant(pcCU->getQP(0), TEXT_LUMA, pcCU->getSlice()->getSPS()->getQpBDOffsetY(), 0);

#if RDOQ_CHROMA_LAMBDA 
  m_pcTrQuant->selectLambda(TEXT_LUMA);
#endif

  UChar ucROTIdx = 0;
  m_pcTrQuant->transformNxN(pcCU, piResi, uiStride, pcCoeff,
#if ADAPTIVE_QP_SELECTION
    pcArlCoeff,
#endif
    uiWidth, uiHeight, uiAbsSum, TEXT_LUMA, uiAbsPartIdx, useTransformSkip
#if QC_EMT_INTRA
    , ucTrIdx
#endif
#if ROT_TR 
    , ucROTIdx
#endif
    , useKLT
    );

  //--- set coded block flag ---
  pcCU->setCbfSubParts((uiAbsSum ? 1 : 0) << uiTrDepth, TEXT_LUMA, uiAbsPartIdx, uiFullDepth);
  //--- inverse transform ---
  if (uiAbsSum)
  {
    UInt *scan = 0;
    if (useKLT)
    {
      const UInt  uiLog2BlockSize = g_aucConvertToBit[uiWidth] + 2;
      UInt scanIdx = pcCU->getCoefScanIdx(uiAbsPartIdx, uiWidth, TEXT_LUMA == TEXT_LUMA, pcCU->isIntra(uiAbsPartIdx));
      scan = g_auiSigLastScan[scanIdx][uiLog2BlockSize - 1];
      recoverOrderCoeff(pcCoeff, scan, uiWidth, uiHeight);
#if ADAPTIVE_QP_SELECTION
      recoverOrderCoeff(pcArlCoeff, scan, uiWidth, uiHeight);
#endif 
    }
    Int scalingListType = 0 + g_eTTable[(Int)TEXT_LUMA];
    assert(scalingListType < SCALING_LIST_NUM);

    m_pcTrQuant->invtransformNxN(pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_LUMA, pcCU->getLumaIntraDir(uiAbsPartIdx), piResi, uiStride, pcCoeff, uiWidth, uiHeight, scalingListType, useTransformSkip
#if QC_EMT_INTRA
      , ucTrIdx
#endif
#if ROT_TR 
      , pcCU->getROTIdx(uiAbsPartIdx)
#endif
#if QC_USE_65ANG_MODES
      , pcCU->getUseExtIntraAngModes(uiPUWidth)
#endif
      ,
      useKLT
      );

    if (useKLT)
    {
      reOrderCoeff(pcCoeff, scan, uiWidth, uiHeight);
#if ADAPTIVE_QP_SELECTION
      reOrderCoeff(pcArlCoeff, scan, uiWidth, uiHeight);
#endif 
    }
  }
  else
  {
    Pel* pResi = piResi;
    memset(pcCoeff, 0, sizeof(TCoeff)* uiWidth * uiHeight);
    for (UInt uiY = 0; uiY < uiHeight; uiY++)
    {
      memset(pResi, 0, sizeof(Pel)* uiWidth);
      pResi += uiStride;
    }
  }

  //===== reconstruction =====
  {
    Pel* pPred = piPred;
    Pel* pResi = piResi;
    Pel* pReco = piReco;
    Pel* pRecQt = piRecQt;
    Pel* pRecIPred = piRecIPred;
    for (UInt uiY = 0; uiY < uiHeight; uiY++)
    {
      for (UInt uiX = 0; uiX < uiWidth; uiX++)
      {
        pReco[uiX] = ClipY(pPred[uiX] + pResi[uiX]);
        pRecQt[uiX] = pReco[uiX];
        pRecIPred[uiX] = pReco[uiX];
      }
      pPred += uiStride;
      pResi += uiStride;
      pReco += uiStride;
      pRecQt += uiRecQtStride;
      pRecIPred += uiRecIPredStride;
    }
  }

  //===== update distortion =====
  ruiDist += m_pcRdCost->getDistPart(g_bitDepthY, piReco, uiStride, piOrg, uiStride, uiWidth, uiHeight);
  return bSuccessful;
}
#endif

Void
TEncSearch::xIntraCodingChromaBlk( TComDataCU* pcCU,
                                  UInt        uiTrDepth,
                                  UInt        uiAbsPartIdx,
                                  TComYuv*    pcOrgYuv, 
                                  TComYuv*    pcPredYuv, 
                                  TComYuv*    pcResiYuv, 
                                  UInt&       ruiDist,
                                  UInt        uiChromaId,
                                  Int        default0Save1Load2 )
{
  UInt uiOrgTrDepth = uiTrDepth;
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
  
  TextType  eText             = ( uiChromaId > 0 ? TEXT_CHROMA_V : TEXT_CHROMA_U );
  UInt      uiChromaPredMode  = pcCU     ->getChromaIntraDir( uiAbsPartIdx );
  UInt      uiWidth           = pcCU     ->getWidth   ( 0 ) >> ( uiTrDepth + 1 );
  UInt      uiHeight          = pcCU     ->getHeight  ( 0 ) >> ( uiTrDepth + 1 );
  UInt      uiStride          = pcOrgYuv ->getCStride ();
  Pel*      piOrg             = ( uiChromaId > 0 ? pcOrgYuv ->getCrAddr( uiAbsPartIdx ) : pcOrgYuv ->getCbAddr( uiAbsPartIdx ) );
  Pel*      piPred            = ( uiChromaId > 0 ? pcPredYuv->getCrAddr( uiAbsPartIdx ) : pcPredYuv->getCbAddr( uiAbsPartIdx ) );
  Pel*      piResi            = ( uiChromaId > 0 ? pcResiYuv->getCrAddr( uiAbsPartIdx ) : pcResiYuv->getCbAddr( uiAbsPartIdx ) );
  Pel*      piReco            = ( uiChromaId > 0 ? pcPredYuv->getCrAddr( uiAbsPartIdx ) : pcPredYuv->getCbAddr( uiAbsPartIdx ) );
  
  UInt      uiQTLayer         = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
  UInt      uiNumCoeffPerInc  = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 ) ) >> 2;
  TCoeff*   pcCoeff           = ( uiChromaId > 0 ? m_ppcQTTempCoeffCr[ uiQTLayer ] : m_ppcQTTempCoeffCb[ uiQTLayer ] ) + uiNumCoeffPerInc * uiAbsPartIdx;
#if ADAPTIVE_QP_SELECTION
  Int*      pcArlCoeff        = ( uiChromaId > 0 ? m_ppcQTTempArlCoeffCr[ uiQTLayer ] : m_ppcQTTempArlCoeffCb[ uiQTLayer ] ) + uiNumCoeffPerInc * uiAbsPartIdx;
#endif
  Pel*      piRecQt           = ( uiChromaId > 0 ? m_pcQTTempTComYuv[ uiQTLayer ].getCrAddr( uiAbsPartIdx ) : m_pcQTTempTComYuv[ uiQTLayer ].getCbAddr( uiAbsPartIdx ) );
  UInt      uiRecQtStride     = m_pcQTTempTComYuv[ uiQTLayer ].getCStride();
  
  UInt      uiZOrder          = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
  Pel*      piRecIPred        = ( uiChromaId > 0 ? pcCU->getPic()->getPicYuvRec()->getCrAddr( pcCU->getAddr(), uiZOrder ) : pcCU->getPic()->getPicYuvRec()->getCbAddr( pcCU->getAddr(), uiZOrder ) );
  UInt      uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getCStride();
  Bool      useTransformSkipChroma       = pcCU->getTransformSkip(uiAbsPartIdx, eText);
  //===== update chroma mode =====
  if( uiChromaPredMode == DM_CHROMA_IDX )
  {
    uiChromaPredMode          = pcCU->getLumaIntraDir( 0 );
  }
  
  //===== init availability pattern =====
  Bool  bAboveAvail = false;
  Bool  bLeftAvail  = false;
  if( default0Save1Load2 != 2 )
  {
    pcCU->getPattern()->initPattern         ( pcCU, uiTrDepth, uiAbsPartIdx );

#if QC_LMCHROMA
    if( uiChromaPredMode == LM_CHROMA_IDX && uiChromaId == 0 )
    {
      Bool bLeftAvailLM = ( (pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ]) != 0 );
      pcCU->getPattern()->initAdiPattern( pcCU, uiAbsPartIdx, uiTrDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail, true );      
      getLumaRecPixels( pcCU->getPattern(), uiWidth, uiHeight, !bLeftAvailLM );
    }
#endif

    pcCU->getPattern()->initAdiPatternChroma( pcCU, uiAbsPartIdx, uiTrDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail );
    Int*  pPatChroma  = ( uiChromaId > 0 ? pcCU->getPattern()->getAdiCrBuf( uiWidth, uiHeight, m_piYuvExt ) : pcCU->getPattern()->getAdiCbBuf( uiWidth, uiHeight, m_piYuvExt ) );

    //===== get prediction signal =====
#if QC_LMCHROMA
    if( uiChromaPredMode == LM_CHROMA_IDX )
    {
      predLMIntraChroma( pcCU->getPattern(), uiChromaId, piPred, uiStride, uiWidth, uiHeight );
    }
    else
#endif
    {
      predIntraChromaAng( pPatChroma, uiChromaPredMode, piPred, uiStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail 
#if QC_INTRA_4TAP_FILTER
        , pcCU->getSlice()->getSPS()->getUse4TapIntraFilter()
#endif
#if CU_LEVEL_MPI
,  pcCU,  uiAbsPartIdx, uiChromaId
#endif
        );

#if QC_LMCHROMA
      if( uiChromaId == 1 && pcCU->getSlice()->getSPS()->getUseLMChroma())
      {
        addCrossColorResi( pcCU->getPattern(), piPred, uiStride, uiWidth, uiHeight, pcResiYuv->getCbAddr( uiAbsPartIdx ), pcResiYuv->getCStride() );
      }
#endif
    }
    // save prediction 
    if( default0Save1Load2 == 1 )
    {
      Pel*  pPred   = piPred;
      Pel*  pPredBuf = m_pSharedPredTransformSkip[1 + uiChromaId];
      Int k = 0;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pPredBuf[ k ++ ] = pPred[ uiX ];
        }
        pPred += uiStride;
      }
    }
  }
  else
  {
    // load prediction 
    Pel*  pPred   = piPred;
    Pel*  pPredBuf = m_pSharedPredTransformSkip[1 + uiChromaId];
    Int k = 0;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pPred[ uiX ] = pPredBuf[ k ++ ];
      }
      pPred += uiStride;
    }
  }
  //===== get residual signal =====
  {
    // get residual
    Pel*  pOrg    = piOrg;
    Pel*  pPred   = piPred;
    Pel*  pResi   = piResi;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pResi[ uiX ] = pOrg[ uiX ] - pPred[ uiX ];
      }
      pOrg  += uiStride;
      pResi += uiStride;
      pPred += uiStride;
    }
  }
  
  //===== transform and quantization =====
  {
    //--- init rate estimation arrays for RDOQ ---
    if( useTransformSkipChroma? m_pcEncCfg->getUseRDOQTS():m_pcEncCfg->getUseRDOQ())
    {
      m_pcEntropyCoder->estimateBit( m_pcTrQuant->m_pcEstBitsSbac, uiWidth, uiWidth, eText );
    }
    //--- transform and quantization ---
    UInt uiAbsSum = 0;

    Int curChromaQpOffset;
    if(eText == TEXT_CHROMA_U)
    {
      curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
    }
    else
    {
      curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
    }
    m_pcTrQuant->setQPforQuant     ( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

#if RDOQ_CHROMA_LAMBDA 
    m_pcTrQuant->selectLambda(eText);

#if CR_FROM_CB_LAMBDA_ADJUSTMENT
    if ( uiChromaPredMode != LM_CHROMA_IDX && pcCU->getSlice()->getSPS()->getUseLMChroma() )
    {
      if (uiChromaId == 0)
      {
        m_pcTrQuant->setLambda ( m_pcTrQuant->getlambda() * 15/16);
      }
      else
      {
        m_pcTrQuant->setLambda ( m_pcTrQuant->getlambda() * 16/15);
      }
    }
#endif
#endif
    m_pcTrQuant->transformNxN      ( pcCU, piResi, uiStride, pcCoeff, 
#if ADAPTIVE_QP_SELECTION
                                     pcArlCoeff, 
#endif
                                     uiWidth, uiHeight, uiAbsSum, eText, uiAbsPartIdx, useTransformSkipChroma );
    //--- set coded block flag ---
    pcCU->setCbfSubParts           ( ( uiAbsSum ? 1 : 0 ) << uiOrgTrDepth, eText, uiAbsPartIdx, pcCU->getDepth(0) + uiTrDepth );
    //--- inverse transform ---
    if( uiAbsSum )
    {
      Int scalingListType = 0 + g_eTTable[(Int)eText];
      assert(scalingListType < SCALING_LIST_NUM);
      m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_CHROMA, REG_DCT, piResi, uiStride, pcCoeff, uiWidth, uiHeight, scalingListType, useTransformSkipChroma 
#if QC_EMT_INTRA
        , pcCU->getSlice()->getSPS()->getUseIntraEMT() ? DCT2_EMT : DCT2_HEVC
#endif
#if ROT_TR 
   , pcCU->getROTIdx(uiAbsPartIdx)
#endif
        );
    }
    else
    {
      Pel* pResi = piResi;
      memset( pcCoeff, 0, sizeof( TCoeff ) * uiWidth * uiHeight );
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        memset( pResi, 0, sizeof( Pel ) * uiWidth );
        pResi += uiStride;
      }
    }
  }
  
  //===== reconstruction =====
  {
    Pel* pPred      = piPred;
    Pel* pResi      = piResi;
    Pel* pReco      = piReco;
    Pel* pRecQt     = piRecQt;
    Pel* pRecIPred  = piRecIPred;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pReco    [ uiX ] = ClipC( pPred[ uiX ] + pResi[ uiX ] );
        pRecQt   [ uiX ] = pReco[ uiX ];
        pRecIPred[ uiX ] = pReco[ uiX ];
      }
      pPred     += uiStride;
      pResi     += uiStride;
      pReco     += uiStride;
      pRecQt    += uiRecQtStride;
      pRecIPred += uiRecIPredStride;
    }
  }
  
  //===== update distortion =====
  ruiDist += m_pcRdCost->getDistPart(g_bitDepthC, piReco, uiStride, piOrg, uiStride, uiWidth, uiHeight, eText );
}



Void 
TEncSearch::xRecurIntraCodingQT( TComDataCU*  pcCU, 
                                UInt         uiTrDepth,
                                UInt         uiAbsPartIdx, 
                                Bool         bLumaOnly,
                                TComYuv*     pcOrgYuv, 
                                TComYuv*     pcPredYuv, 
                                TComYuv*     pcResiYuv, 
                                UInt&        ruiDistY,
                                UInt&        ruiDistC,
#if HHI_RQT_INTRA_SPEEDUP
                                Bool         bCheckFirst,
#endif
                                Double&      dRDCost )
{
  UInt    uiFullDepth   = pcCU->getDepth( 0 ) +  uiTrDepth;
  UInt    uiLog2TrSize  = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
  Bool    bCheckFull    = ( uiLog2TrSize  <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
  Bool    bCheckSplit   = ( uiLog2TrSize  >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
#if QC_EMT_INTRA 
  UInt    uiSigNum;
#endif

#if HHI_RQT_INTRA_SPEEDUP
  Int maxTuSize = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize();
  Int isIntraSlice = (pcCU->getSlice()->getSliceType() == I_SLICE);
  // don't check split if TU size is less or equal to max TU size
  Bool noSplitIntraMaxTuSize = bCheckFull;
  if(m_pcEncCfg->getRDpenalty() && ! isIntraSlice)
  {
    // in addition don't check split if TU size is less or equal to 16x16 TU size for non-intra slice
    noSplitIntraMaxTuSize = ( uiLog2TrSize  <= min(maxTuSize,4) );

    // if maximum RD-penalty don't check TU size 32x32 
    if(m_pcEncCfg->getRDpenalty()==2)
    {
      bCheckFull    = ( uiLog2TrSize  <= min(maxTuSize,4));
    }
  }
  if( bCheckFirst && noSplitIntraMaxTuSize )
  {
    bCheckSplit = false;
  }
#else
  Int maxTuSize = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize();
  Int isIntraSlice = (pcCU->getSlice()->getSliceType() == I_SLICE);
  // if maximum RD-penalty don't check TU size 32x32 
  if((m_pcEncCfg->getRDpenalty()==2)  && !isIntraSlice)
  {
    bCheckFull    = ( uiLog2TrSize  <= min(maxTuSize,4));
  }
#endif

#if QC_EMT_INTRA && HHI_RQT_INTRA_SPEEDUP
  // Re-use the selected transform indexes in the previous call of xRecurIntraCodingQT
  UInt    uiInitTrDepth     = pcCU->getPartitionSize(0) == SIZE_2Nx2N ? 0 : 1;
  UChar   ucSavedEmtTrIdx   = 0;
  Bool    bCheckInitTrDepth = false;
  static UInt uiInitAbsPartIdx;
  if ( uiTrDepth==uiInitTrDepth )
  {
    uiInitAbsPartIdx = uiAbsPartIdx;
  }
  if ( !bCheckFirst && uiTrDepth==uiInitTrDepth )
  {
    ucSavedEmtTrIdx   = m_puhQTTempEmtTuIdx[uiAbsPartIdx-uiInitAbsPartIdx];
    bCheckInitTrDepth = true;
  }
#endif


  Double  dSingleCost   = MAX_DOUBLE;
  UInt    uiSingleDistY = 0;
  UInt    uiSingleDistC = 0;
  UInt    uiSingleCbfY  = 0;
  UInt    uiSingleCbfU  = 0;
  UInt    uiSingleCbfV  = 0;
  Bool    checkTransformSkip  = pcCU->getSlice()->getPPS()->getUseTransformSkip();
  UInt    widthTransformSkip  = pcCU->getWidth ( 0 ) >> uiTrDepth;
  UInt    heightTransformSkip = pcCU->getHeight( 0 ) >> uiTrDepth;
  Int     bestModeId    = 0;
  Int     bestModeIdUV[2] = {0, 0};
#if QC_EMT_INTRA
  UChar   bestTrIdx     = 0;
  UChar   nNumTrCands   = pcCU->getEmtCuFlag(uiAbsPartIdx) ? 4 : 1;
#if QC_EMT_INTRA_FAST
  Bool    bAllIntra     = (m_pcEncCfg->getIntraPeriod()==1);
 #endif
#endif

  checkTransformSkip         &= (widthTransformSkip == 4 && heightTransformSkip == 4);
  checkTransformSkip         &= (!pcCU->getCUTransquantBypass(0));

#if INTRA_KLT
  Int  bestTMKLT = 0;
  Bool checkTM = (bCheckFirst == false);
  if (checkTM)
  {
    UInt uiMaxTrWidth = g_uiDepth2Width[USE_MORE_BLOCKSIZE_DEPTH_MAX - 1];
    UInt uiMinTrWidth = g_uiDepth2Width[USE_MORE_BLOCKSIZE_DEPTH_MIN - 1];
    UInt    uiWidth = pcCU->getWidth(0) >> uiTrDepth;
    UInt    uiHeight = pcCU->getHeight(0) >> uiTrDepth;
    Bool bCheckKLTFlag = (uiWidth == uiHeight) && (uiWidth <= uiMaxTrWidth) && (uiWidth >= uiMinTrWidth);
    checkTM &= bCheckKLTFlag; 
#if CU_LEVEL_MPI //for speed up only
    Int iMPIidx = pcCU->getMPIIdx(uiAbsPartIdx);
    checkTM = checkTM & (iMPIidx == 0);
#endif
#if ROT_TR //for speed up only
    Int iRotidx = pcCU->getROTIdx(uiAbsPartIdx);
    checkTM = checkTM & (iRotidx == 0);
#endif
  }
#endif

  if ( m_pcEncCfg->getUseTransformSkipFast() )
  {
    checkTransformSkip       &= (pcCU->getPartitionSize(uiAbsPartIdx)==SIZE_NxN);
  }
  if( bCheckFull )
  {
    if(checkTransformSkip == true)
    {
      //----- store original entropy coding status -----
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );

      UInt   singleDistYTmp     = 0;
      UInt   singleDistCTmp     = 0;
      UInt   singleCbfYTmp      = 0;
      UInt   singleCbfUTmp      = 0;
      UInt   singleCbfVTmp      = 0;
      Double singleCostTmp      = 0;
      Int    default0Save1Load2 = 0;
      Int    firstCheckId       = 0;

      UInt   uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + (uiTrDepth - 1) ) << 1 );
      Bool   bFirstQ = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
#if INTRA_KLT
    pcCU->setKLTFlagSubParts(0, TEXT_LUMA, uiAbsPartIdx, uiFullDepth);
#endif
      for(Int modeId = firstCheckId; modeId < 2; modeId ++)
      {
#if QC_EMT_INTRA
#if HHI_RQT_INTRA_SPEEDUP
        UChar numTrIdxCands = ((modeId == firstCheckId && !bCheckInitTrDepth) ? nNumTrCands : 1 );
#else
        UChar numTrIdxCands = ((modeId == firstCheckId) ? nNumTrCands : 1 );
#endif

        for (UChar ucTrIdx = 0; ucTrIdx < numTrIdxCands; ucTrIdx++)
        {
#if QC_EMT_INTRA_FAST
          // Skip checking other transform candidates if zero CBF is encountered
          if( ucTrIdx && !uiSingleCbfY && bAllIntra && m_pcEncCfg->getUseFastIntraEMT() )
          {
            continue;
          }
#endif
#endif
        singleDistYTmp = 0;
        singleDistCTmp = 0;
        pcCU ->setTransformSkipSubParts ( modeId, TEXT_LUMA, uiAbsPartIdx, uiFullDepth ); 
#if QC_EMT_INTRA
        if (modeId == firstCheckId && ucTrIdx == 0)
#else
        if (modeId == firstCheckId)
#endif
        {
          default0Save1Load2 = 1;
        }
        else
        {
          default0Save1Load2 = 2;
        }
#if QC_EMT_INTRA
#if HHI_RQT_INTRA_SPEEDUP
        pcCU->setEmtTuIdxSubParts( bCheckInitTrDepth?ucSavedEmtTrIdx:ucTrIdx, uiAbsPartIdx, uiFullDepth );
#else
        pcCU->setEmtTuIdxSubParts( txIdx, uiAbsPartIdx, uiFullDepth );
#endif
#endif
        //----- code luma block with given intra prediction mode and store Cbf-----
        xIntraCodingLumaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, singleDistYTmp
#if QC_EMT_INTRA
          , uiSigNum
#endif
          , default0Save1Load2); 
        singleCbfYTmp = pcCU->getCbf( uiAbsPartIdx, TEXT_LUMA, uiTrDepth );
        //----- code chroma blocks with given intra prediction mode and store Cbf-----
        if( !bLumaOnly )
        {
          if(bFirstQ)
          {
            pcCU ->setTransformSkipSubParts ( modeId, TEXT_CHROMA_U, uiAbsPartIdx, uiFullDepth); 
            pcCU ->setTransformSkipSubParts ( modeId, TEXT_CHROMA_V, uiAbsPartIdx, uiFullDepth); 
          }
          xIntraCodingChromaBlk ( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, singleDistCTmp, 0, default0Save1Load2); 
          xIntraCodingChromaBlk ( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, singleDistCTmp, 1, default0Save1Load2); 
          singleCbfUTmp = pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepth );
          singleCbfVTmp = pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepth );
        }
        //----- determine rate and r-d cost -----
#if QC_EMT_INTRA
        if( (modeId == 1 && singleCbfYTmp == 0) || (modeId==0 && ucTrIdx && ucTrIdx!=DCT2_EMT && uiSigNum<=g_iEmtSigNumThr) )
#else
        if(modeId == 1 && singleCbfYTmp == 0)
#endif
        {
          //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
          singleCostTmp = MAX_DOUBLE; 
        }
        else
        {
          UInt uiSingleBits = xGetIntraBitsQT( pcCU, uiTrDepth, uiAbsPartIdx, true, !bLumaOnly, false );
          singleCostTmp     = m_pcRdCost->calcRdCost( uiSingleBits, singleDistYTmp + singleDistCTmp );
        }

        if(singleCostTmp < dSingleCost)
        {
          dSingleCost   = singleCostTmp;
          uiSingleDistY = singleDistYTmp;
          uiSingleDistC = singleDistCTmp;
          uiSingleCbfY  = singleCbfYTmp;
          uiSingleCbfU  = singleCbfUTmp;
          uiSingleCbfV  = singleCbfVTmp;
          bestModeId    = modeId;
#if QC_EMT_INTRA
          bestTrIdx     = ucTrIdx;
#endif
          if(bestModeId == firstCheckId)
          {
            xStoreIntraResultQT(pcCU, uiTrDepth, uiAbsPartIdx,bLumaOnly );
            m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
          }
        }
        if(modeId == firstCheckId)
        {
          m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
        }
#if QC_EMT_INTRA
        }
#endif
      }

      pcCU ->setTransformSkipSubParts ( bestModeId, TEXT_LUMA, uiAbsPartIdx, uiFullDepth ); 
#if QC_EMT_INTRA
      pcCU->setEmtTuIdxSubParts ( bestTrIdx, uiAbsPartIdx, uiFullDepth );
#endif

      if(bestModeId == firstCheckId)
      {
        xLoadIntraResultQT(pcCU, uiTrDepth, uiAbsPartIdx,bLumaOnly );
        pcCU->setCbfSubParts  ( uiSingleCbfY << uiTrDepth, TEXT_LUMA, uiAbsPartIdx, uiFullDepth );
        if( !bLumaOnly )
        {
          if(bFirstQ)
          {
            pcCU->setCbfSubParts( uiSingleCbfU << uiTrDepth, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth( 0 ) + uiTrDepth - 1 );
            pcCU->setCbfSubParts( uiSingleCbfV << uiTrDepth, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth( 0 ) + uiTrDepth - 1 );
          }
        }
        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
      }

      if( !bLumaOnly )
      {
        bestModeIdUV[0] = bestModeIdUV[1] = bestModeId;
        if(bFirstQ && bestModeId == 1)
        {
          //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
          if(uiSingleCbfU == 0)
          {
            pcCU ->setTransformSkipSubParts ( 0, TEXT_CHROMA_U, uiAbsPartIdx, uiFullDepth); 
            bestModeIdUV[0] = 0;
          }
          if(uiSingleCbfV == 0)
          {
            pcCU ->setTransformSkipSubParts ( 0, TEXT_CHROMA_V, uiAbsPartIdx, uiFullDepth); 
            bestModeIdUV[1] = 0;
          }
        }
      }
    }
    else
    {
#if QC_EMT_INTRA
#if HHI_RQT_INTRA_SPEEDUP
      UChar numTrIdxCands = ( !bCheckInitTrDepth ) ? nNumTrCands : 1;
#else
      UChar numTrIdxCands = nNumTrCands;
#endif

      for (UChar ucTrIdx = 0; ucTrIdx < numTrIdxCands; ucTrIdx++)
      {
        UInt   singleDistYTmp     = 0;
        UInt   singleDistCTmp     = 0;
        UInt   singleCbfYTmp      = 0;
        UInt   singleCbfUTmp      = 0;
        UInt   singleCbfVTmp      = 0;
        Double singleCostTmp      = 0;
        Int    default0Save1Load2 = 0;
        Bool   bSaveEmtResults    = ucTrIdx<(numTrIdxCands-1);
#endif
      pcCU ->setTransformSkipSubParts ( 0, TEXT_LUMA, uiAbsPartIdx, uiFullDepth ); 
#if INTRA_KLT
    pcCU->setKLTFlagSubParts ( 0, TEXT_LUMA, uiAbsPartIdx, uiFullDepth);
#endif
#if QC_EMT_INTRA_FAST
        // Skip checking other transform candidates if zero CBF is encountered
        if ( ucTrIdx && !uiSingleCbfY && bAllIntra && m_pcEncCfg->getUseFastIntraEMT() )
        {
          continue;
        }
#endif

      //----- store original entropy coding status -----
#if INTRA_KLT
#if QC_EMT_INTRA
    if (bCheckSplit || bSaveEmtResults || checkTM)
#else
    if( bCheckSplit || checkTM)
#endif
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }
#else
#if QC_EMT_INTRA
      if( bCheckSplit || bSaveEmtResults )
#else
      if( bCheckSplit )
#endif
      {
        m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
      }
#endif

#if QC_EMT_INTRA
      default0Save1Load2 = numTrIdxCands>1 ? ( bSaveEmtResults ? 1 : 2 ) : 0;
#endif

      //----- code luma block with given intra prediction mode and store Cbf-----
#if QC_EMT_INTRA
#if HHI_RQT_INTRA_SPEEDUP
      pcCU->setEmtTuIdxSubParts( bCheckInitTrDepth?ucSavedEmtTrIdx:ucTrIdx, uiAbsPartIdx, uiFullDepth );
#else
      pcCU->setEmtTuIdxSubParts( trIdx, uiAbsPartIdx, uiFullDepth );
#endif
      xIntraCodingLumaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, singleDistYTmp, uiSigNum, default0Save1Load2 ); 
#else
      dSingleCost   = 0.0;
      xIntraCodingLumaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, uiSingleDistY ); 
#endif

#if INTRA_KLT
#if QC_EMT_INTRA
      if( bCheckSplit || bSaveEmtResults || checkTM)
#else
    if( bCheckSplit || checkTM)
#endif
#else
#if QC_EMT_INTRA
    if (bCheckSplit || bSaveEmtResults)
#else
    if (bCheckSplit)
#endif
#endif
      {
#if QC_EMT_INTRA
        singleCbfYTmp = pcCU->getCbf( uiAbsPartIdx, TEXT_LUMA, uiTrDepth );
#else
        uiSingleCbfY = pcCU->getCbf( uiAbsPartIdx, TEXT_LUMA, uiTrDepth );
#endif
      }
      //----- code chroma blocks with given intra prediction mode and store Cbf-----
      if( !bLumaOnly )
      {
        pcCU ->setTransformSkipSubParts ( 0, TEXT_CHROMA_U, uiAbsPartIdx, uiFullDepth ); 
        pcCU ->setTransformSkipSubParts ( 0, TEXT_CHROMA_V, uiAbsPartIdx, uiFullDepth ); 
        xIntraCodingChromaBlk ( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, uiSingleDistC, 0 ); 
        xIntraCodingChromaBlk ( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, uiSingleDistC, 1 ); 
#if INTRA_KLT
    if( checkTM)
#else
        if( bCheckSplit )
#endif
        {
          uiSingleCbfU = pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepth );
          uiSingleCbfV = pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepth );
        }
      }
      //----- determine rate and r-d cost -----
#if QC_EMT_INTRA
      if( ucTrIdx && ucTrIdx!=DCT2_EMT && uiSigNum<=g_iEmtSigNumThr )
      {
        singleCostTmp = MAX_DOUBLE;
      }
      else
      {
#endif
      UInt uiSingleBits = xGetIntraBitsQT( pcCU, uiTrDepth, uiAbsPartIdx, true, !bLumaOnly, false );
      if(m_pcEncCfg->getRDpenalty() && (uiLog2TrSize==5) && !isIntraSlice)
      {
        uiSingleBits=uiSingleBits*4; 
      }
#if QC_EMT_INTRA
      singleCostTmp     = m_pcRdCost->calcRdCost( uiSingleBits, singleDistYTmp + uiSingleDistC );
      }
#else
      dSingleCost       = m_pcRdCost->calcRdCost( uiSingleBits, uiSingleDistY + uiSingleDistC );
#endif

#if QC_EMT_INTRA
      if(singleCostTmp < dSingleCost)
      {
        dSingleCost   = singleCostTmp;
        uiSingleDistY = singleDistYTmp;
        uiSingleDistC = singleDistCTmp;
        uiSingleCbfY  = singleCbfYTmp;
        uiSingleCbfU  = singleCbfUTmp;
        uiSingleCbfV  = singleCbfVTmp;
        bestTrIdx     = ucTrIdx;

#if QC_EMT_INTRA_FAST
        if( bSaveEmtResults && ( uiSingleCbfY || !bAllIntra || !m_pcEncCfg->getUseFastIntraEMT() ) )
#else
        if( bSaveEmtResults )
#endif
        {
          xStoreIntraResultQT(pcCU, uiTrDepth, uiAbsPartIdx, bLumaOnly );
          m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
        }
      }
#if QC_EMT_INTRA_FAST
      if( bSaveEmtResults && ( uiSingleCbfY || !bAllIntra || !m_pcEncCfg->getUseFastIntraEMT() ) )
#else
      if( bSaveEmtResults )
#endif
      {
        m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
      }
      }

      pcCU->setEmtTuIdxSubParts( bestTrIdx, uiAbsPartIdx, uiFullDepth );

#if QC_EMT_INTRA_FAST
      if( bestTrIdx < (numTrIdxCands-1) && ( uiSingleCbfY || !bAllIntra || !m_pcEncCfg->getUseFastIntraEMT() ) )
#else
      if( bestTrIdx < (numTrIdxCands-1) )
#endif
      {
        UInt   uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + (uiTrDepth - 1) ) << 1 );
        Bool   bFirstQ = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
        xLoadIntraResultQT(pcCU, uiTrDepth, uiAbsPartIdx,bLumaOnly );
        pcCU->setCbfSubParts  ( uiSingleCbfY << uiTrDepth, TEXT_LUMA, uiAbsPartIdx, uiFullDepth );
        if( !bLumaOnly )
        {
          if(bFirstQ)
          {
            pcCU->setCbfSubParts( uiSingleCbfU << uiTrDepth, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth( 0 ) + uiTrDepth - 1 );
            pcCU->setCbfSubParts( uiSingleCbfV << uiTrDepth, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth( 0 ) + uiTrDepth - 1 );
          }
        }
        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
      }
#endif
    }

#if INTRA_KLT
  if (checkTM && uiSingleCbfY)
  {
    Int genPred0genPredAndtrainKLT1 = GENPRED0GENPREDANDTRAINKLT1;
    //backup the former results
    Double  dSingleCostBackUp = dSingleCost;
    UInt    uiSingleCbfYBackUp = uiSingleCbfY;
    UInt    uiSingleCbfUBackUp = uiSingleCbfU;
    UInt    uiSingleCbfVBackUp = uiSingleCbfV;
    UInt    uiSingleDistYBackUp = uiSingleDistY;
    UInt    uiSingleDistCBackUp = uiSingleDistC;

    uiSingleDistY = 0; 
    uiSingleDistC = 0;
    dSingleCost = 0.0;
    uiSingleCbfY = 0;
    uiSingleCbfU = 0;
    uiSingleCbfV = 0;

    pcCU->setKLTFlagSubParts(1, TEXT_LUMA, uiAbsPartIdx, uiFullDepth);
#if QC_EMT_INTRA
    pcCU->setEmtTuIdxSubParts(0, uiAbsPartIdx, uiFullDepth);
#endif
    xStoreIntraResultQT(pcCU, uiTrDepth, uiAbsPartIdx, bLumaOnly);
    m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiFullDepth][CI_TEMP_BEST]);

    //template matching prediction and transform
    pcCU->setTransformSkipSubParts(0, TEXT_LUMA, uiAbsPartIdx, uiFullDepth);
    //----- store original entropy coding status -----
    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiFullDepth][CI_QT_TRAFO_ROOT]);
    //----- code luma block with given intra prediction mode and store Cbf-----
    Bool bSuccessful;
    bSuccessful = xIntraCodingLumaBlkTM(pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, uiSingleDistY, genPred0genPredAndtrainKLT1);
    uiSingleCbfY = pcCU->getCbf(uiAbsPartIdx, TEXT_LUMA, uiTrDepth);

    if (bSuccessful == true && uiSingleCbfY) //assume only cbf being nonzero, can use TM mode
    {
      //----- code chroma blocks with given intra prediction mode and store Cbf-----
      if (!bLumaOnly)
      {
        pcCU->setTransformSkipSubParts(0, TEXT_CHROMA_U, uiAbsPartIdx, uiFullDepth);
        pcCU->setTransformSkipSubParts(0, TEXT_CHROMA_V, uiAbsPartIdx, uiFullDepth);
        xIntraCodingChromaBlk(pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, uiSingleDistC, 0);
        xIntraCodingChromaBlk(pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, uiSingleDistC, 1);
        if (bCheckSplit)
        {
          uiSingleCbfU = pcCU->getCbf(uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepth);
          uiSingleCbfV = pcCU->getCbf(uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepth);
        }
      }
      //----- determine rate and r-d cost -----
      UInt uiSingleBits = xGetIntraBitsQT(pcCU, uiTrDepth, uiAbsPartIdx, true, !bLumaOnly, false);
      if (m_pcEncCfg->getRDpenalty() && (uiLog2TrSize == 5) && !isIntraSlice)
      {
        uiSingleBits = uiSingleBits * 4;
      }
      dSingleCost = m_pcRdCost->calcRdCost(uiSingleBits, uiSingleDistY + uiSingleDistC);
    }
    else
    {
      dSingleCost = MAX_DOUBLE;
    }
    if (dSingleCostBackUp <= dSingleCost)
    {
      dSingleCost = dSingleCostBackUp;
      uiSingleCbfY = uiSingleCbfYBackUp;
      uiSingleCbfU = uiSingleCbfUBackUp;
      uiSingleCbfV = uiSingleCbfVBackUp;
      uiSingleDistY = uiSingleDistYBackUp;
      uiSingleDistC = uiSingleDistCBackUp;
      pcCU->setKLTFlagSubParts(0, TEXT_LUMA, uiAbsPartIdx, uiFullDepth);
      pcCU->setEmtTuIdxSubParts(bestTrIdx, uiAbsPartIdx, uiFullDepth);
      pcCU->setTransformSkipSubParts(bestModeId, TEXT_LUMA, uiAbsPartIdx, uiFullDepth);
      if (!bLumaOnly)
      {
        pcCU->setTransformSkipSubParts(bestModeIdUV[0], TEXT_CHROMA_U, uiAbsPartIdx, uiFullDepth);
        pcCU->setTransformSkipSubParts(bestModeIdUV[1], TEXT_CHROMA_V, uiAbsPartIdx, uiFullDepth);
      }
      xLoadIntraResultQT(pcCU, uiTrDepth, uiAbsPartIdx, bLumaOnly);
      pcCU->setCbfSubParts(uiSingleCbfY << uiTrDepth, TEXT_LUMA, uiAbsPartIdx, uiFullDepth); //?????????
      if (!bLumaOnly)
      {
        UInt   uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ((pcCU->getDepth(0) + (uiTrDepth - 1)) << 1);
        Bool   bFirstQ = ((uiAbsPartIdx % uiQPDiv) == 0);
        if (bFirstQ)
        {
          pcCU->setCbfSubParts(uiSingleCbfU << uiTrDepth, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0) + uiTrDepth - 1);
          pcCU->setCbfSubParts(uiSingleCbfV << uiTrDepth, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0) + uiTrDepth - 1);
        }
      }
      m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiFullDepth][CI_TEMP_BEST]);
    }
    else
    {
      pcCU->setEmtTuIdxSubParts(0, uiAbsPartIdx, uiFullDepth);
      bestTMKLT = 1;
      bestModeId = 0;
      bestTrIdx = 0; 
    }
  }
#endif
  }
  
  if( bCheckSplit )
  {
    //----- store full entropy coding status, load original entropy coding status -----
    if( bCheckFull )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    else
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }

    //----- code splitted block -----
    Double  dSplitCost      = 0.0;
    UInt    uiSplitDistY    = 0;
    UInt    uiSplitDistC    = 0;
    UInt    uiQPartsDiv     = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    UInt    uiAbsPartIdxSub = uiAbsPartIdx;

    UInt    uiSplitCbfY = 0;
    UInt    uiSplitCbfU = 0;
    UInt    uiSplitCbfV = 0;

#if QC_EMT
    Bool    bSplitSelected = true;
#endif

    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiAbsPartIdxSub += uiQPartsDiv )
    {
#if HHI_RQT_INTRA_SPEEDUP
      xRecurIntraCodingQT( pcCU, uiTrDepth + 1, uiAbsPartIdxSub, bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, uiSplitDistY, uiSplitDistC, bCheckFirst, dSplitCost );
#else
      xRecurIntraCodingQT( pcCU, uiTrDepth + 1, uiAbsPartIdxSub, bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, uiSplitDistY, uiSplitDistC, dSplitCost );
#endif

#if QC_EMT
      if( dSplitCost>dSingleCost )
      {
        bSplitSelected = false;
        break;
      }
#endif

      uiSplitCbfY |= pcCU->getCbf( uiAbsPartIdxSub, TEXT_LUMA, uiTrDepth + 1 );
      if(!bLumaOnly)
      {
        uiSplitCbfU |= pcCU->getCbf( uiAbsPartIdxSub, TEXT_CHROMA_U, uiTrDepth + 1 );
        uiSplitCbfV |= pcCU->getCbf( uiAbsPartIdxSub, TEXT_CHROMA_V, uiTrDepth + 1 );
      }
    }

#if QC_EMT
    if( bSplitSelected )
    {
#endif
    for( UInt uiOffs = 0; uiOffs < 4 * uiQPartsDiv; uiOffs++ )
    {
      pcCU->getCbf( TEXT_LUMA )[ uiAbsPartIdx + uiOffs ] |= ( uiSplitCbfY << uiTrDepth );
    }
    if( !bLumaOnly )
    {
      for( UInt uiOffs = 0; uiOffs < 4 * uiQPartsDiv; uiOffs++ )
      {
        pcCU->getCbf( TEXT_CHROMA_U )[ uiAbsPartIdx + uiOffs ] |= ( uiSplitCbfU << uiTrDepth );
        pcCU->getCbf( TEXT_CHROMA_V )[ uiAbsPartIdx + uiOffs ] |= ( uiSplitCbfV << uiTrDepth );
      }
    }
    //----- restore context states -----
    m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );

    //----- determine rate and r-d cost -----
    UInt uiSplitBits = xGetIntraBitsQT( pcCU, uiTrDepth, uiAbsPartIdx, true, !bLumaOnly, false );
    dSplitCost       = m_pcRdCost->calcRdCost( uiSplitBits, uiSplitDistY + uiSplitDistC );
#if QC_EMT
    }
#endif

    //===== compare and set best =====
    if( dSplitCost < dSingleCost )
    {
      //--- update cost ---
      ruiDistY += uiSplitDistY;
      ruiDistC += uiSplitDistC;
      dRDCost  += dSplitCost;
      return;
    }
    //----- set entropy coding status -----
    m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );
    
    //--- set transform index and Cbf values ---
    pcCU->setTrIdxSubParts( uiTrDepth, uiAbsPartIdx, uiFullDepth );
    pcCU->setCbfSubParts  ( uiSingleCbfY << uiTrDepth, TEXT_LUMA, uiAbsPartIdx, uiFullDepth );
    pcCU ->setTransformSkipSubParts  ( bestModeId, TEXT_LUMA, uiAbsPartIdx, uiFullDepth ); 
#if QC_EMT_INTRA
    pcCU->setEmtTuIdxSubParts( bestTrIdx, uiAbsPartIdx, uiFullDepth );
#endif
#if INTRA_KLT
  pcCU->setKLTFlagSubParts(bestTMKLT, TEXT_LUMA, uiAbsPartIdx, uiFullDepth);
#endif
    if( !bLumaOnly )
    {
      pcCU->setCbfSubParts( uiSingleCbfU << uiTrDepth, TEXT_CHROMA_U, uiAbsPartIdx, uiFullDepth );
      pcCU->setCbfSubParts( uiSingleCbfV << uiTrDepth, TEXT_CHROMA_V, uiAbsPartIdx, uiFullDepth );
      pcCU->setTransformSkipSubParts ( bestModeIdUV[0], TEXT_CHROMA_U, uiAbsPartIdx, uiFullDepth); 
      pcCU->setTransformSkipSubParts ( bestModeIdUV[1], TEXT_CHROMA_V, uiAbsPartIdx, uiFullDepth); 
    }
    
    //--- set reconstruction for next intra prediction blocks ---
    UInt  uiWidth     = pcCU->getWidth ( 0 ) >> uiTrDepth;
    UInt  uiHeight    = pcCU->getHeight( 0 ) >> uiTrDepth;
    UInt  uiQTLayer   = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    UInt  uiZOrder    = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
    Pel*  piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getLumaAddr( uiAbsPartIdx );
    UInt  uiSrcStride = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ();
    Pel*  piDes       = pcCU->getPic()->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), uiZOrder );
    UInt  uiDesStride = pcCU->getPic()->getPicYuvRec()->getStride  ();
    for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        piDes[ uiX ] = piSrc[ uiX ];
      }
    }
    if( !bLumaOnly )
    {
      uiWidth   >>= 1;
      uiHeight  >>= 1;
      piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getCbAddr  ( uiAbsPartIdx );
      uiSrcStride = m_pcQTTempTComYuv[ uiQTLayer ].getCStride ();
      piDes       = pcCU->getPic()->getPicYuvRec()->getCbAddr ( pcCU->getAddr(), uiZOrder );
      uiDesStride = pcCU->getPic()->getPicYuvRec()->getCStride();
      for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          piDes[ uiX ] = piSrc[ uiX ];
        }
      }
      piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getCrAddr  ( uiAbsPartIdx );
      piDes       = pcCU->getPic()->getPicYuvRec()->getCrAddr ( pcCU->getAddr(), uiZOrder );
      for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          piDes[ uiX ] = piSrc[ uiX ];
        }
      }
    }
  }
  ruiDistY += uiSingleDistY;
  ruiDistC += uiSingleDistC;
  dRDCost  += dSingleCost;
}


Void
TEncSearch::xSetIntraResultQT( TComDataCU* pcCU,
                              UInt        uiTrDepth,
                              UInt        uiAbsPartIdx,
                              Bool        bLumaOnly,
                              TComYuv*    pcRecoYuv )
{
  UInt uiFullDepth  = pcCU->getDepth(0) + uiTrDepth;
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    
    Bool bSkipChroma  = false;
    Bool bChromaSame  = false;
    if( !bLumaOnly && uiLog2TrSize == 2 )
    {
      assert( uiTrDepth > 0 );
      UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth - 1 ) << 1 );
      bSkipChroma  = ( ( uiAbsPartIdx % uiQPDiv ) != 0 );
      bChromaSame  = true;
    }
    
    //===== copy transform coefficients =====
    UInt uiNumCoeffY    = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( uiFullDepth << 1 );
    UInt uiNumCoeffIncY = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
    TCoeff* pcCoeffSrcY = m_ppcQTTempCoeffY [ uiQTLayer ] + ( uiNumCoeffIncY * uiAbsPartIdx );
    TCoeff* pcCoeffDstY = pcCU->getCoeffY ()              + ( uiNumCoeffIncY * uiAbsPartIdx );
    ::memcpy( pcCoeffDstY, pcCoeffSrcY, sizeof( TCoeff ) * uiNumCoeffY );
#if ADAPTIVE_QP_SELECTION
    Int* pcArlCoeffSrcY = m_ppcQTTempArlCoeffY [ uiQTLayer ] + ( uiNumCoeffIncY * uiAbsPartIdx );
    Int* pcArlCoeffDstY = pcCU->getArlCoeffY ()              + ( uiNumCoeffIncY * uiAbsPartIdx );
    ::memcpy( pcArlCoeffDstY, pcArlCoeffSrcY, sizeof( Int ) * uiNumCoeffY );
#endif
    if( !bLumaOnly && !bSkipChroma )
    {
      UInt uiNumCoeffC    = ( bChromaSame ? uiNumCoeffY    : uiNumCoeffY    >> 2 );
      UInt uiNumCoeffIncC = uiNumCoeffIncY >> 2;
      TCoeff* pcCoeffSrcU = m_ppcQTTempCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcCoeffSrcV = m_ppcQTTempCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcCoeffDstU = pcCU->getCoeffCb()              + ( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcCoeffDstV = pcCU->getCoeffCr()              + ( uiNumCoeffIncC * uiAbsPartIdx );
      ::memcpy( pcCoeffDstU, pcCoeffSrcU, sizeof( TCoeff ) * uiNumCoeffC );
      ::memcpy( pcCoeffDstV, pcCoeffSrcV, sizeof( TCoeff ) * uiNumCoeffC );
#if ADAPTIVE_QP_SELECTION
      Int* pcArlCoeffSrcU = m_ppcQTTempArlCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      Int* pcArlCoeffSrcV = m_ppcQTTempArlCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      Int* pcArlCoeffDstU = pcCU->getArlCoeffCb()              + ( uiNumCoeffIncC * uiAbsPartIdx );
      Int* pcArlCoeffDstV = pcCU->getArlCoeffCr()              + ( uiNumCoeffIncC * uiAbsPartIdx );
      ::memcpy( pcArlCoeffDstU, pcArlCoeffSrcU, sizeof( Int ) * uiNumCoeffC );
      ::memcpy( pcArlCoeffDstV, pcArlCoeffSrcV, sizeof( Int ) * uiNumCoeffC );
#endif
    }
    
    //===== copy reconstruction =====
    m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartLuma( pcRecoYuv, uiAbsPartIdx, 1 << uiLog2TrSize, 1 << uiLog2TrSize );
    if( !bLumaOnly && !bSkipChroma )
    {
      UInt uiLog2TrSizeChroma = ( bChromaSame ? uiLog2TrSize : uiLog2TrSize - 1 );
      m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartChroma( pcRecoYuv, uiAbsPartIdx, 1 << uiLog2TrSizeChroma, 1 << uiLog2TrSizeChroma );
    }
  }
  else
  {
    UInt uiNumQPart  = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    for( UInt uiPart = 0; uiPart < 4; uiPart++ )
    {
      xSetIntraResultQT( pcCU, uiTrDepth + 1, uiAbsPartIdx + uiPart * uiNumQPart, bLumaOnly, pcRecoYuv );
    }
  }
}

Void
TEncSearch::xStoreIntraResultQT( TComDataCU* pcCU,
                                UInt        uiTrDepth,
                                UInt        uiAbsPartIdx,
                                Bool        bLumaOnly )
{
  UInt uiFullDepth  = pcCU->getDepth(0) + uiTrDepth;
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  assert(  uiTrMode == uiTrDepth );
  UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
  UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

  Bool bSkipChroma  = false;
  Bool bChromaSame  = false;
  if( !bLumaOnly && uiLog2TrSize == 2 )
  {
    assert( uiTrDepth > 0 );
    UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth - 1 ) << 1 );
    bSkipChroma  = ( ( uiAbsPartIdx % uiQPDiv ) != 0 );
    bChromaSame  = true;
  }

  //===== copy transform coefficients =====
  UInt uiNumCoeffY    = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( uiFullDepth << 1 );
  UInt uiNumCoeffIncY = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
  TCoeff* pcCoeffSrcY = m_ppcQTTempCoeffY [ uiQTLayer ] + ( uiNumCoeffIncY * uiAbsPartIdx );
  TCoeff* pcCoeffDstY = m_pcQTTempTUCoeffY;

  ::memcpy( pcCoeffDstY, pcCoeffSrcY, sizeof( TCoeff ) * uiNumCoeffY );
#if ADAPTIVE_QP_SELECTION
  Int* pcArlCoeffSrcY = m_ppcQTTempArlCoeffY [ uiQTLayer ] + ( uiNumCoeffIncY * uiAbsPartIdx );
  Int* pcArlCoeffDstY = m_ppcQTTempTUArlCoeffY;
  ::memcpy( pcArlCoeffDstY, pcArlCoeffSrcY, sizeof( Int ) * uiNumCoeffY );
#endif
  if( !bLumaOnly && !bSkipChroma )
  {
    UInt uiNumCoeffC    = ( bChromaSame ? uiNumCoeffY    : uiNumCoeffY    >> 2 );
    UInt uiNumCoeffIncC = uiNumCoeffIncY >> 2;
    TCoeff* pcCoeffSrcU = m_ppcQTTempCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    TCoeff* pcCoeffSrcV = m_ppcQTTempCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    TCoeff* pcCoeffDstU = m_pcQTTempTUCoeffCb;
    TCoeff* pcCoeffDstV = m_pcQTTempTUCoeffCr;
    ::memcpy( pcCoeffDstU, pcCoeffSrcU, sizeof( TCoeff ) * uiNumCoeffC );
    ::memcpy( pcCoeffDstV, pcCoeffSrcV, sizeof( TCoeff ) * uiNumCoeffC );
#if ADAPTIVE_QP_SELECTION
    Int* pcArlCoeffSrcU = m_ppcQTTempArlCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    Int* pcArlCoeffSrcV = m_ppcQTTempArlCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    Int* pcArlCoeffDstU = m_ppcQTTempTUArlCoeffCb;
    Int* pcArlCoeffDstV = m_ppcQTTempTUArlCoeffCr;
    ::memcpy( pcArlCoeffDstU, pcArlCoeffSrcU, sizeof( Int ) * uiNumCoeffC );
    ::memcpy( pcArlCoeffDstV, pcArlCoeffSrcV, sizeof( Int ) * uiNumCoeffC );
#endif
  }

  //===== copy reconstruction =====
  m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartLuma( &m_pcQTTempTransformSkipTComYuv, uiAbsPartIdx, 1 << uiLog2TrSize, 1 << uiLog2TrSize );

  if( !bLumaOnly && !bSkipChroma )
  {
    UInt uiLog2TrSizeChroma = ( bChromaSame ? uiLog2TrSize : uiLog2TrSize - 1 );
    m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartChroma( &m_pcQTTempTransformSkipTComYuv, uiAbsPartIdx, 1 << uiLog2TrSizeChroma, 1 << uiLog2TrSizeChroma );
  }
}

Void
TEncSearch::xLoadIntraResultQT( TComDataCU* pcCU,
                               UInt        uiTrDepth,
                               UInt        uiAbsPartIdx,
                               Bool        bLumaOnly )
{
  UInt uiFullDepth  = pcCU->getDepth(0) + uiTrDepth;
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  assert(  uiTrMode == uiTrDepth );
  UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
  UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

  Bool bSkipChroma  = false;
  Bool bChromaSame  = false;
  if( !bLumaOnly && uiLog2TrSize == 2 )
  {
    assert( uiTrDepth > 0 );
    UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth - 1 ) << 1 );
    bSkipChroma  = ( ( uiAbsPartIdx % uiQPDiv ) != 0 );
    bChromaSame  = true;
  }

  //===== copy transform coefficients =====
  UInt uiNumCoeffY    = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( uiFullDepth << 1 );
  UInt uiNumCoeffIncY = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
  TCoeff* pcCoeffDstY = m_ppcQTTempCoeffY [ uiQTLayer ] + ( uiNumCoeffIncY * uiAbsPartIdx );
  TCoeff* pcCoeffSrcY = m_pcQTTempTUCoeffY;

  ::memcpy( pcCoeffDstY, pcCoeffSrcY, sizeof( TCoeff ) * uiNumCoeffY );
#if ADAPTIVE_QP_SELECTION
  Int* pcArlCoeffDstY = m_ppcQTTempArlCoeffY [ uiQTLayer ] + ( uiNumCoeffIncY * uiAbsPartIdx );
  Int* pcArlCoeffSrcY = m_ppcQTTempTUArlCoeffY;
  ::memcpy( pcArlCoeffDstY, pcArlCoeffSrcY, sizeof( Int ) * uiNumCoeffY );
#endif
  if( !bLumaOnly && !bSkipChroma )
  {
    UInt uiNumCoeffC    = ( bChromaSame ? uiNumCoeffY    : uiNumCoeffY    >> 2 );
    UInt uiNumCoeffIncC = uiNumCoeffIncY >> 2;
    TCoeff* pcCoeffDstU = m_ppcQTTempCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    TCoeff* pcCoeffDstV = m_ppcQTTempCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    TCoeff* pcCoeffSrcU = m_pcQTTempTUCoeffCb;
    TCoeff* pcCoeffSrcV = m_pcQTTempTUCoeffCr;
    ::memcpy( pcCoeffDstU, pcCoeffSrcU, sizeof( TCoeff ) * uiNumCoeffC );
    ::memcpy( pcCoeffDstV, pcCoeffSrcV, sizeof( TCoeff ) * uiNumCoeffC );
#if ADAPTIVE_QP_SELECTION
    Int* pcArlCoeffDstU = m_ppcQTTempArlCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    Int* pcArlCoeffDstV = m_ppcQTTempArlCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    Int* pcArlCoeffSrcU = m_ppcQTTempTUArlCoeffCb;
    Int* pcArlCoeffSrcV = m_ppcQTTempTUArlCoeffCr;
    ::memcpy( pcArlCoeffDstU, pcArlCoeffSrcU, sizeof( Int ) * uiNumCoeffC );
    ::memcpy( pcArlCoeffDstV, pcArlCoeffSrcV, sizeof( Int ) * uiNumCoeffC );
#endif
  }

  //===== copy reconstruction =====
  m_pcQTTempTransformSkipTComYuv.copyPartToPartLuma( &m_pcQTTempTComYuv[ uiQTLayer ] , uiAbsPartIdx, 1 << uiLog2TrSize, 1 << uiLog2TrSize );

  if( !bLumaOnly && !bSkipChroma )
  {
    UInt uiLog2TrSizeChroma = ( bChromaSame ? uiLog2TrSize : uiLog2TrSize - 1 );
    m_pcQTTempTransformSkipTComYuv.copyPartToPartChroma( &m_pcQTTempTComYuv[ uiQTLayer ], uiAbsPartIdx, 1 << uiLog2TrSizeChroma, 1 << uiLog2TrSizeChroma );
  }

  UInt    uiZOrder          = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
  Pel*    piRecIPred        = pcCU->getPic()->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), uiZOrder );
  UInt    uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getStride  ();
  Pel*    piRecQt           = m_pcQTTempTComYuv[ uiQTLayer ].getLumaAddr( uiAbsPartIdx );
  UInt    uiRecQtStride     = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ();
  UInt    uiWidth           = pcCU     ->getWidth   ( 0 ) >> uiTrDepth;
  UInt    uiHeight          = pcCU     ->getHeight  ( 0 ) >> uiTrDepth;
  Pel* pRecQt     = piRecQt;
  Pel* pRecIPred  = piRecIPred;
  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      pRecIPred[ uiX ] = pRecQt   [ uiX ];
    }
    pRecQt    += uiRecQtStride;
    pRecIPred += uiRecIPredStride;
  }

  if( !bLumaOnly && !bSkipChroma )
  {
    piRecIPred = pcCU->getPic()->getPicYuvRec()->getCbAddr( pcCU->getAddr(), uiZOrder );
    piRecQt    = m_pcQTTempTComYuv[ uiQTLayer ].getCbAddr( uiAbsPartIdx );
    pRecQt     = piRecQt;
    pRecIPred  = piRecIPred;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pRecIPred[ uiX ] = pRecQt[ uiX ];
      }
      pRecQt    += uiRecQtStride;
      pRecIPred += uiRecIPredStride;
    }

    piRecIPred = pcCU->getPic()->getPicYuvRec()->getCrAddr( pcCU->getAddr(), uiZOrder );
    piRecQt    = m_pcQTTempTComYuv[ uiQTLayer ].getCrAddr( uiAbsPartIdx );
    pRecQt     = piRecQt;
    pRecIPred  = piRecIPred;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pRecIPred[ uiX ] = pRecQt[ uiX ];
      }
      pRecQt    += uiRecQtStride;
      pRecIPred += uiRecIPredStride;
    }
  }
}

Void
TEncSearch::xStoreIntraResultChromaQT( TComDataCU* pcCU,
#if QC_LMCHROMA
                                       TComYuv*     pcResiYuv,
#endif
                                       UInt        uiTrDepth,
                                       UInt        uiAbsPartIdx,
                                       UInt        stateU0V1Both2 )
{
  UInt uiFullDepth = pcCU->getDepth(0) + uiTrDepth;
  UInt uiTrMode    = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    Bool bChromaSame = false;
    if( uiLog2TrSize == 2 )
    {
      assert( uiTrDepth > 0 );
      uiTrDepth --;
      UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth) << 1 );
      if( ( uiAbsPartIdx % uiQPDiv ) != 0 )
      {
        return;
      }
      bChromaSame = true;
    }

    //===== copy transform coefficients =====
    UInt uiNumCoeffC    = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( uiFullDepth << 1 );
    if( !bChromaSame )
    {
      uiNumCoeffC     >>= 2;
    }
    UInt uiNumCoeffIncC = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 ) + 2 );
    if(stateU0V1Both2 == 0 || stateU0V1Both2 == 2)
    {
      TCoeff* pcCoeffSrcU = m_ppcQTTempCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcCoeffDstU = m_pcQTTempTUCoeffCb;
      ::memcpy( pcCoeffDstU, pcCoeffSrcU, sizeof( TCoeff ) * uiNumCoeffC );

#if ADAPTIVE_QP_SELECTION    
      Int* pcArlCoeffSrcU = m_ppcQTTempArlCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      Int* pcArlCoeffDstU = m_ppcQTTempTUArlCoeffCb;
      ::memcpy( pcArlCoeffDstU, pcArlCoeffSrcU, sizeof( Int ) * uiNumCoeffC );
#endif
    }
    if(stateU0V1Both2 == 1 || stateU0V1Both2 == 2)
    {
      TCoeff* pcCoeffSrcV = m_ppcQTTempCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcCoeffDstV = m_pcQTTempTUCoeffCr;
      ::memcpy( pcCoeffDstV, pcCoeffSrcV, sizeof( TCoeff ) * uiNumCoeffC );
#if ADAPTIVE_QP_SELECTION    
      Int* pcArlCoeffSrcV = m_ppcQTTempArlCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      Int* pcArlCoeffDstV = m_ppcQTTempTUArlCoeffCr;
      ::memcpy( pcArlCoeffDstV, pcArlCoeffSrcV, sizeof( Int ) * uiNumCoeffC );
#endif
    }

    //===== copy reconstruction =====
    UInt uiLog2TrSizeChroma = ( bChromaSame ? uiLog2TrSize : uiLog2TrSize - 1 );
    m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartChroma(&m_pcQTTempTransformSkipTComYuv, uiAbsPartIdx, 1 << uiLog2TrSizeChroma, 1 << uiLog2TrSizeChroma, stateU0V1Both2 );

    //===== copy residue =====
#if QC_LMCHROMA
    pcResiYuv->copyPartToPartChroma( &m_pcQTTempResiTComYuv, uiAbsPartIdx,  1 << uiLog2TrSizeChroma, 1 << uiLog2TrSizeChroma, 0);
#endif
  }
}


Void
TEncSearch::xLoadIntraResultChromaQT( TComDataCU* pcCU,
#if QC_LMCHROMA
                                      TComYuv*     pcResiYuv,
#endif
                                      UInt        uiTrDepth,
                                      UInt        uiAbsPartIdx,
                                      UInt        stateU0V1Both2 )
{
  UInt uiFullDepth = pcCU->getDepth(0) + uiTrDepth;
  UInt uiTrMode    = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    Bool bChromaSame = false;
    if( uiLog2TrSize == 2 )
    {
      assert( uiTrDepth > 0 );
      uiTrDepth --;
      UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth ) << 1 );
      if( ( uiAbsPartIdx % uiQPDiv ) != 0 )
      {
        return;
      }
      bChromaSame = true;
    }

    //===== copy transform coefficients =====
    UInt uiNumCoeffC = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( uiFullDepth << 1 );
    if( !bChromaSame )
    {
      uiNumCoeffC >>= 2;
    }
    UInt uiNumCoeffIncC = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 ) + 2 );

    if(stateU0V1Both2 ==0 || stateU0V1Both2 == 2)
    {
      TCoeff* pcCoeffDstU = m_ppcQTTempCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcCoeffSrcU = m_pcQTTempTUCoeffCb;
      ::memcpy( pcCoeffDstU, pcCoeffSrcU, sizeof( TCoeff ) * uiNumCoeffC );
#if ADAPTIVE_QP_SELECTION    
      Int* pcArlCoeffDstU = m_ppcQTTempArlCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      Int* pcArlCoeffSrcU = m_ppcQTTempTUArlCoeffCb;
      ::memcpy( pcArlCoeffDstU, pcArlCoeffSrcU, sizeof( Int ) * uiNumCoeffC );
#endif
    }
    if(stateU0V1Both2 ==1 || stateU0V1Both2 == 2)
    {
      TCoeff* pcCoeffDstV = m_ppcQTTempCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcCoeffSrcV = m_pcQTTempTUCoeffCr;
      ::memcpy( pcCoeffDstV, pcCoeffSrcV, sizeof( TCoeff ) * uiNumCoeffC );
#if ADAPTIVE_QP_SELECTION    
      Int* pcArlCoeffDstV = m_ppcQTTempArlCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
      Int* pcArlCoeffSrcV = m_ppcQTTempTUArlCoeffCr;       
      ::memcpy( pcArlCoeffDstV, pcArlCoeffSrcV, sizeof( Int ) * uiNumCoeffC );
#endif
    }

    //===== copy reconstruction =====
    UInt uiLog2TrSizeChroma = ( bChromaSame ? uiLog2TrSize : uiLog2TrSize - 1 );
    m_pcQTTempTransformSkipTComYuv.copyPartToPartChroma( &m_pcQTTempTComYuv[ uiQTLayer ], uiAbsPartIdx, 1 << uiLog2TrSizeChroma, 1 << uiLog2TrSizeChroma, stateU0V1Both2);

    //===== copy residue =====
#if QC_LMCHROMA
    m_pcQTTempResiTComYuv.copyPartToPartChroma( pcResiYuv, uiAbsPartIdx,  1 << uiLog2TrSizeChroma, 1 << uiLog2TrSizeChroma, 0 );
#endif

    UInt    uiZOrder          = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
    UInt    uiWidth           = pcCU->getWidth   ( 0 ) >> (uiTrDepth + 1);
    UInt    uiHeight          = pcCU->getHeight  ( 0 ) >> (uiTrDepth + 1);
    UInt    uiRecQtStride     = m_pcQTTempTComYuv[ uiQTLayer ].getCStride  ();
    UInt    uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getCStride  ();

    if(stateU0V1Both2 ==0 || stateU0V1Both2 == 2)
    {
      Pel* piRecIPred = pcCU->getPic()->getPicYuvRec()->getCbAddr( pcCU->getAddr(), uiZOrder );
      Pel* piRecQt    = m_pcQTTempTComYuv[ uiQTLayer ].getCbAddr( uiAbsPartIdx );
      Pel* pRecQt     = piRecQt;
      Pel* pRecIPred  = piRecIPred;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pRecIPred[ uiX ] = pRecQt[ uiX ];
        }
        pRecQt    += uiRecQtStride;
        pRecIPred += uiRecIPredStride;
      }
    }
    if(stateU0V1Both2 == 1 || stateU0V1Both2 == 2)
    {
      Pel* piRecIPred = pcCU->getPic()->getPicYuvRec()->getCrAddr( pcCU->getAddr(), uiZOrder );
      Pel* piRecQt    = m_pcQTTempTComYuv[ uiQTLayer ].getCrAddr( uiAbsPartIdx );
      Pel* pRecQt     = piRecQt;
      Pel* pRecIPred  = piRecIPred;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pRecIPred[ uiX ] = pRecQt[ uiX ];
        }
        pRecQt    += uiRecQtStride;
        pRecIPred += uiRecIPredStride;
      }
    }
  }
}

Void 
TEncSearch::xRecurIntraChromaCodingQT( TComDataCU*  pcCU, 
                                      UInt         uiTrDepth,
                                      UInt         uiAbsPartIdx, 
                                      TComYuv*     pcOrgYuv, 
                                      TComYuv*     pcPredYuv, 
                                      TComYuv*     pcResiYuv, 
                                      UInt&        ruiDist )
{
  UInt uiFullDepth = pcCU->getDepth( 0 ) +  uiTrDepth;
  UInt uiTrMode    = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    Bool checkTransformSkip = pcCU->getSlice()->getPPS()->getUseTransformSkip();
    UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;

    UInt actualTrDepth = uiTrDepth;
    if( uiLog2TrSize == 2 )
    {
      assert( uiTrDepth > 0 );
      actualTrDepth--;
      UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + actualTrDepth) << 1 );
      Bool bFirstQ = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
      if( !bFirstQ )
      {
        return;
      }
    }

    checkTransformSkip &= (uiLog2TrSize <= 3);

    if ( m_pcEncCfg->getUseTransformSkipFast() )
    {

      checkTransformSkip &= (uiLog2TrSize < 3);

      if (checkTransformSkip)
      {
        Int nbLumaSkip = 0;
        for(UInt absPartIdxSub = uiAbsPartIdx; absPartIdxSub < uiAbsPartIdx + 4; absPartIdxSub ++)
        {
          nbLumaSkip += pcCU->getTransformSkip(absPartIdxSub, TEXT_LUMA);
        }
        checkTransformSkip &= (nbLumaSkip > 0);
      }
    }

    if(checkTransformSkip)
    {
      //use RDO to decide whether Cr/Cb takes TS
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiFullDepth][CI_QT_TRAFO_ROOT] );
      
      for(Int chromaId = 0; chromaId < 2; chromaId ++)
      {
        Double  dSingleCost    = MAX_DOUBLE;
        Int     bestModeId     = 0;
        UInt    singleDistC    = 0;
        UInt    singleCbfC     = 0;
        UInt    singleDistCTmp = 0;
        Double  singleCostTmp  = 0;
        UInt    singleCbfCTmp  = 0;
        
        Int     default0Save1Load2 = 0;
        Int     firstCheckId       = 0;
        
        for(Int chromaModeId = firstCheckId; chromaModeId < 2; chromaModeId ++)
        {
          pcCU->setTransformSkipSubParts ( chromaModeId, (TextType)(chromaId + 2), uiAbsPartIdx, pcCU->getDepth( 0 ) +  actualTrDepth);
          if(chromaModeId == firstCheckId)
          {
            default0Save1Load2 = 1;
          }
          else
          {
            default0Save1Load2 = 2;
          }
          singleDistCTmp = 0;
          xIntraCodingChromaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, singleDistCTmp, chromaId ,default0Save1Load2);
          singleCbfCTmp = pcCU->getCbf( uiAbsPartIdx, (TextType)(chromaId + 2), uiTrDepth);
          
          if(chromaModeId == 1 && singleCbfCTmp == 0)
          {
            //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
            singleCostTmp = MAX_DOUBLE;
          }
          else
          {
            UInt bitsTmp = xGetIntraBitsQTChroma( pcCU,uiTrDepth, uiAbsPartIdx,chromaId + 2, false );
            singleCostTmp  = m_pcRdCost->calcRdCost( bitsTmp, singleDistCTmp);
          }
          
          if(singleCostTmp < dSingleCost)
          {
            dSingleCost = singleCostTmp;
            singleDistC = singleDistCTmp;
            bestModeId  = chromaModeId;
            singleCbfC  = singleCbfCTmp;
            
            if(bestModeId == firstCheckId)
            {
#if QC_LMCHROMA
              xStoreIntraResultChromaQT(pcCU, pcResiYuv, uiTrDepth, uiAbsPartIdx, chromaId);
#else
              xStoreIntraResultChromaQT(pcCU, uiTrDepth, uiAbsPartIdx,chromaId);
#endif
              m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
            }
          }
          if(chromaModeId == firstCheckId)
          {
            m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
          }
        }
        
        if(bestModeId == firstCheckId)
        {
#if QC_LMCHROMA
          xLoadIntraResultChromaQT(pcCU, pcResiYuv, uiTrDepth, uiAbsPartIdx, chromaId );
#else
          xLoadIntraResultChromaQT(pcCU, uiTrDepth, uiAbsPartIdx,chromaId);
#endif
          pcCU->setCbfSubParts ( singleCbfC << uiTrDepth, (TextType)(chromaId + 2), uiAbsPartIdx, pcCU->getDepth(0) + actualTrDepth );
          m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
        }
        pcCU ->setTransformSkipSubParts( bestModeId, (TextType)(chromaId + 2), uiAbsPartIdx, pcCU->getDepth( 0 ) +  actualTrDepth );
        ruiDist += singleDistC;
        
        if(chromaId == 0)
        {
          m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiFullDepth][CI_QT_TRAFO_ROOT] );
        }
      }
    }
    else
    {
      pcCU ->setTransformSkipSubParts( 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth( 0 ) +  actualTrDepth );
      pcCU ->setTransformSkipSubParts( 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth( 0 ) +  actualTrDepth ); 
      xIntraCodingChromaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, ruiDist, 0 ); 
      xIntraCodingChromaBlk( pcCU, uiTrDepth, uiAbsPartIdx, pcOrgYuv, pcPredYuv, pcResiYuv, ruiDist, 1 ); 
    }
  }
  else
  {
    UInt uiSplitCbfU     = 0;
    UInt uiSplitCbfV     = 0;
    UInt uiQPartsDiv     = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    UInt uiAbsPartIdxSub = uiAbsPartIdx;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiAbsPartIdxSub += uiQPartsDiv )
    {
      xRecurIntraChromaCodingQT( pcCU, uiTrDepth + 1, uiAbsPartIdxSub, pcOrgYuv, pcPredYuv, pcResiYuv, ruiDist );
      uiSplitCbfU |= pcCU->getCbf( uiAbsPartIdxSub, TEXT_CHROMA_U, uiTrDepth + 1 );
      uiSplitCbfV |= pcCU->getCbf( uiAbsPartIdxSub, TEXT_CHROMA_V, uiTrDepth + 1 );
    }
    for( UInt uiOffs = 0; uiOffs < 4 * uiQPartsDiv; uiOffs++ )
    {
      pcCU->getCbf( TEXT_CHROMA_U )[ uiAbsPartIdx + uiOffs ] |= ( uiSplitCbfU << uiTrDepth );
      pcCU->getCbf( TEXT_CHROMA_V )[ uiAbsPartIdx + uiOffs ] |= ( uiSplitCbfV << uiTrDepth );
    }
  }
}

Void
TEncSearch::xSetIntraResultChromaQT( TComDataCU* pcCU,
                                    UInt        uiTrDepth,
                                    UInt        uiAbsPartIdx,
                                    TComYuv*    pcRecoYuv )
{
  UInt uiFullDepth  = pcCU->getDepth(0) + uiTrDepth;
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiFullDepth ] + 2;
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    
    Bool bChromaSame  = false;
    if( uiLog2TrSize == 2 )
    {
      assert( uiTrDepth > 0 );
      UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrDepth - 1 ) << 1 );
      if( ( uiAbsPartIdx % uiQPDiv ) != 0 )
      {
        return;
      }
      bChromaSame     = true;
    }
    
    //===== copy transform coefficients =====
    UInt uiNumCoeffC    = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( uiFullDepth << 1 );
    if( !bChromaSame )
    {
      uiNumCoeffC     >>= 2;
    }
    UInt uiNumCoeffIncC = ( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 ) + 2 );
    TCoeff* pcCoeffSrcU = m_ppcQTTempCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    TCoeff* pcCoeffSrcV = m_ppcQTTempCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    TCoeff* pcCoeffDstU = pcCU->getCoeffCb()              + ( uiNumCoeffIncC * uiAbsPartIdx );
    TCoeff* pcCoeffDstV = pcCU->getCoeffCr()              + ( uiNumCoeffIncC * uiAbsPartIdx );
    ::memcpy( pcCoeffDstU, pcCoeffSrcU, sizeof( TCoeff ) * uiNumCoeffC );
    ::memcpy( pcCoeffDstV, pcCoeffSrcV, sizeof( TCoeff ) * uiNumCoeffC );
#if ADAPTIVE_QP_SELECTION    
    Int* pcArlCoeffSrcU = m_ppcQTTempArlCoeffCb[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    Int* pcArlCoeffSrcV = m_ppcQTTempArlCoeffCr[ uiQTLayer ] + ( uiNumCoeffIncC * uiAbsPartIdx );
    Int* pcArlCoeffDstU = pcCU->getArlCoeffCb()              + ( uiNumCoeffIncC * uiAbsPartIdx );
    Int* pcArlCoeffDstV = pcCU->getArlCoeffCr()              + ( uiNumCoeffIncC * uiAbsPartIdx );
    ::memcpy( pcArlCoeffDstU, pcArlCoeffSrcU, sizeof( Int ) * uiNumCoeffC );
    ::memcpy( pcArlCoeffDstV, pcArlCoeffSrcV, sizeof( Int ) * uiNumCoeffC );
#endif
    
    //===== copy reconstruction =====
    UInt uiLog2TrSizeChroma = ( bChromaSame ? uiLog2TrSize : uiLog2TrSize - 1 );
    m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartChroma( pcRecoYuv, uiAbsPartIdx, 1 << uiLog2TrSizeChroma, 1 << uiLog2TrSizeChroma );
  }
  else
  {
    UInt uiNumQPart  = pcCU->getPic()->getNumPartInCU() >> ( ( uiFullDepth + 1 ) << 1 );
    for( UInt uiPart = 0; uiPart < 4; uiPart++ )
    {
      xSetIntraResultChromaQT( pcCU, uiTrDepth + 1, uiAbsPartIdx + uiPart * uiNumQPart, pcRecoYuv );
    }
  }
}


Void 
TEncSearch::preestChromaPredMode( TComDataCU* pcCU, 
                                 TComYuv*    pcOrgYuv, 
                                 TComYuv*    pcPredYuv )
{
  UInt  uiWidth     = pcCU->getWidth ( 0 ) >> 1;
  UInt  uiHeight    = pcCU->getHeight( 0 ) >> 1;
  UInt  uiStride    = pcOrgYuv ->getCStride();
  Pel*  piOrgU      = pcOrgYuv ->getCbAddr ( 0 );
  Pel*  piOrgV      = pcOrgYuv ->getCrAddr ( 0 );
  Pel*  piPredU     = pcPredYuv->getCbAddr ( 0 );
  Pel*  piPredV     = pcPredYuv->getCrAddr ( 0 );
  
  //===== init pattern =====
  Bool  bAboveAvail = false;
  Bool  bLeftAvail  = false;
  pcCU->getPattern()->initPattern         ( pcCU, 0, 0 );
  pcCU->getPattern()->initAdiPatternChroma( pcCU, 0, 0, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail );
  Int*  pPatChromaU = pcCU->getPattern()->getAdiCbBuf( uiWidth, uiHeight, m_piYuvExt );
  Int*  pPatChromaV = pcCU->getPattern()->getAdiCrBuf( uiWidth, uiHeight, m_piYuvExt );
  
  //===== get best prediction modes (using SAD) =====
  UInt  uiMinMode   = 0;
  UInt  uiMaxMode   = 4;
  UInt  uiBestMode  = MAX_UINT;
  UInt  uiMinSAD    = MAX_UINT;
  for( UInt uiMode  = uiMinMode; uiMode < uiMaxMode; uiMode++ )
  {
    //--- get prediction ---
    predIntraChromaAng( pPatChromaU, uiMode, piPredU, uiStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail 
#if QC_INTRA_4TAP_FILTER
      , pcCU->getSlice()->getSPS()->getUse4TapIntraFilter()
#endif
#if CU_LEVEL_MPI
,  pcCU,  0, 0
#endif
      );
    predIntraChromaAng( pPatChromaV, uiMode, piPredV, uiStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail 
#if QC_INTRA_4TAP_FILTER
      , pcCU->getSlice()->getSPS()->getUse4TapIntraFilter()
#endif
#if CU_LEVEL_MPI
,  pcCU,  0, 1
#endif
      );
    
    //--- get SAD ---
    UInt  uiSAD  = m_pcRdCost->calcHAD(g_bitDepthC, piOrgU, uiStride, piPredU, uiStride, uiWidth, uiHeight );
    uiSAD       += m_pcRdCost->calcHAD(g_bitDepthC, piOrgV, uiStride, piPredV, uiStride, uiWidth, uiHeight );
    //--- check ---
    if( uiSAD < uiMinSAD )
    {
      uiMinSAD   = uiSAD;
      uiBestMode = uiMode;
    }
  }
  
  //===== set chroma pred mode =====
  pcCU->setChromIntraDirSubParts( uiBestMode, 0, pcCU->getDepth( 0 ) );
}

Void 
TEncSearch::estIntraPredQT( TComDataCU* pcCU, 
                           TComYuv*    pcOrgYuv, 
                           TComYuv*    pcPredYuv, 
                           TComYuv*    pcResiYuv, 
                           TComYuv*    pcRecoYuv,
                           UInt&       ruiDistC,
                           Bool        bLumaOnly )
{
  UInt    uiDepth        = pcCU->getDepth(0);
  UInt    uiNumPU        = pcCU->getNumPartitions();
  UInt    uiInitTrDepth  = pcCU->getPartitionSize(0) == SIZE_2Nx2N ? 0 : 1;
  UInt    uiWidth        = pcCU->getWidth (0) >> uiInitTrDepth;
  UInt    uiHeight       = pcCU->getHeight(0) >> uiInitTrDepth;
  UInt    uiQNumParts    = pcCU->getTotalNumPart() >> 2;
  UInt    uiWidthBit     = pcCU->getIntraSizeIdx(0);
  UInt    uiOverallDistY = 0;
  UInt    uiOverallDistC = 0;
  UInt    CandNum;
  Double  CandCostList[ FAST_UDI_MAX_RDMODE_NUM ];
#if QC_USE_65ANG_MODES
  Bool    bUseExtIntraAngModes = pcCU->getUseExtIntraAngModes(uiWidth);
#endif

  //===== set QP and clear Cbf =====
  if ( pcCU->getSlice()->getPPS()->getUseDQP() == true)
  {
    pcCU->setQPSubParts( pcCU->getQP(0), 0, uiDepth );
  }
  else
  {
    pcCU->setQPSubParts( pcCU->getSlice()->getSliceQp(), 0, uiDepth );
  }
  
#if QC_EMT_INTRA
#if QC_EMT_INTRA_FAST
  static Double dBestModeCostStore[4]; // RD cost of the best mode for each PU using DCT2
  static Double dModeCostStore[4][35]; // RD cost of each mode for each PU using DCT2
#endif
  static UInt   uiSavedRdModeList[4][35], uiSavedNumRdModes[4];

  // Marking EMT usage for faster EMT
  // 0: EMT not applicable for current CU (pcCU->getWidth(0) <= QC_EMT_INTRA_MAX_CU)
  // 1: EMT can be applied for current CU, and DCT2 is being checked
  // 2: EMT is being checked for current CU. Stored results of DCT2 can be utilized for speedup
  UChar ucEmtUsageFlag = ( pcCU->getWidth(0) <= QC_EMT_INTRA_MAX_CU ? ( pcCU->getEmtCuFlag(0)==1 ? 2 : 1 ) : 0 );
  Bool  bAllIntra = (m_pcEncCfg->getIntraPeriod()==1);

  if( pcCU->getPartitionSize(0) == SIZE_NxN && !bAllIntra )
  {
    ucEmtUsageFlag = 0;
  }
#endif

  //===== loop over partitions =====
  UInt uiPartOffset = 0;
  for( UInt uiPU = 0; uiPU < uiNumPU; uiPU++, uiPartOffset += uiQNumParts )
  {
#if QC_EMT_INTRA
    UInt uiRdModeList[FAST_UDI_MAX_RDMODE_NUM];
    Int numModesForFullRD = g_aucIntraModeNumFast[ uiWidthBit ]
#if QC_USE_65ANG_MODES
    - (bUseExtIntraAngModes ? 1 : 0)
#endif
    ;
    if( ucEmtUsageFlag != 2 )
    {
#endif
    //===== init pattern for luma prediction =====
    Bool bAboveAvail = false;
    Bool bLeftAvail  = false;
    pcCU->getPattern()->initPattern   ( pcCU, uiInitTrDepth, uiPartOffset );
    pcCU->getPattern()->initAdiPattern( pcCU, uiPartOffset, uiInitTrDepth, m_piYuvExt, m_iYuvExtStride, m_iYuvExtHeight, bAboveAvail, bLeftAvail );
    
    //===== determine set of modes to be tested (using prediction signal only) =====
#if QC_USE_65ANG_MODES
    Int numModesAvailable     = 67; //total number of Intra modes
#else
    Int numModesAvailable     = 35; //total number of Intra modes
#endif
    Pel* piOrg         = pcOrgYuv ->getLumaAddr( uiPU, uiWidth );
    Pel* piPred        = pcPredYuv->getLumaAddr( uiPU, uiWidth );
    UInt uiStride      = pcPredYuv->getStride();
#if !QC_EMT_INTRA
    UInt uiRdModeList[FAST_UDI_MAX_RDMODE_NUM];
    Int numModesForFullRD = g_aucIntraModeNumFast[ uiWidthBit ];
#endif

    Bool doFastSearch = (numModesForFullRD != numModesAvailable);
    if (doFastSearch)
    {
      assert(numModesForFullRD < numModesAvailable);

      for( Int i=0; i < numModesForFullRD; i++ ) 
      {
        CandCostList[ i ] = MAX_DOUBLE;
      }
      CandNum = 0;
      
#if QC_USE_65ANG_MODES
      Int uiPreds[6] = {-1, -1, -1, -1, -1, -1};
      Int iAboveLeftCase=0, iMode=-1;
      Int numCand = pcCU->getIntraDirLumaPredictor( uiPartOffset, uiPreds, iAboveLeftCase, &iMode ); // Pre-calculate the MPMs, so avoid redundant MPM calculations during the SATD loop
      assert( iMode >= 0 );
      if( bUseExtIntraAngModes )
      {
        iMode = min(iMode+(bUseExtIntraAngModes?1:0), 6);
      }

      Char bSatdChecked[NUM_INTRA_MODE];
      memset( bSatdChecked, 0, NUM_INTRA_MODE*sizeof(Char) );
#endif

      for( Int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++ )
      {
        UInt uiMode = modeIdx;

#if QC_USE_65ANG_MODES
        if( uiMode>DC_IDX && (uiMode&1) )
        {
          // Skip checking extended Angular modes in the first round of SATD
          continue;
        }
        bSatdChecked[uiMode] = true;
#endif
        predIntraLumaAng( pcCU->getPattern(), uiMode, piPred, uiStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail 
#if QC_INTRA_4TAP_FILTER
          , pcCU->getSlice()->getSPS()->getUse4TapIntraFilter()
#endif
#if INTRA_BOUNDARY_FILTER
          , pcCU->getSlice()->getSPS()->getUseBoundaryFilter()
#endif
#if QC_USE_65ANG_MODES
          , pcCU->getUseExtIntraAngModes(uiWidth)
#endif
#if CU_LEVEL_MPI
,  pcCU,  uiPartOffset
#endif
          );
        
        // use hadamard transform here
        UInt uiSad = m_pcRdCost->calcHAD(g_bitDepthY, piOrg, uiStride, piPred, uiStride, uiWidth, uiHeight );

        UInt   iModeBits = xModeBitsIntra( pcCU, uiMode, uiPU, uiPartOffset, uiDepth, uiInitTrDepth 
#if QC_USE_65ANG_MODES
          , uiPreds, iAboveLeftCase
#endif
          );
        Double cost      = (Double)uiSad + (Double)iModeBits * m_pcRdCost->getSqrtLambda();
        
        CandNum += xUpdateCandList( uiMode, cost, numModesForFullRD, uiRdModeList, CandCostList );
      }

#if QC_USE_65ANG_MODES
      if( bUseExtIntraAngModes )
      {
        UInt uiParentCandList[FAST_UDI_MAX_RDMODE_NUM];
        memcpy( uiParentCandList, uiRdModeList, sizeof(UInt)*numModesForFullRD );

        // Second round of SATD for extended Angular modes
        for( Int modeIdx = 0; modeIdx < numModesForFullRD; modeIdx++ )
        {
          UInt uiParentMode = uiParentCandList[modeIdx];
          if( uiParentMode>2 && uiParentMode<(NUM_INTRA_MODE-2) )
          {
            for( Int subModeIdx = -1; subModeIdx <= 1; subModeIdx+=2 )
            {
              UInt uiMode = uiParentMode + subModeIdx;
              if( !bSatdChecked[uiMode] )
              {
                predIntraLumaAng( pcCU->getPattern(), uiMode, piPred, uiStride, uiWidth, uiHeight, bAboveAvail, bLeftAvail 
#if QC_INTRA_4TAP_FILTER
                  , pcCU->getSlice()->getSPS()->getUse4TapIntraFilter()
#endif
#if INTRA_BOUNDARY_FILTER
                  , pcCU->getSlice()->getSPS()->getUseBoundaryFilter()
#endif
                  , pcCU->getUseExtIntraAngModes(uiWidth)
#if CU_LEVEL_MPI
,  pcCU,  uiPartOffset
#endif
                  );

                UInt uiSad = m_pcRdCost->calcHAD(g_bitDepthY, piOrg, uiStride, piPred, uiStride, uiWidth, uiHeight );
                UInt   iModeBits = xModeBitsIntra( pcCU, uiMode, uiPU, uiPartOffset, uiDepth, uiInitTrDepth, uiPreds, iAboveLeftCase );
                Double cost      = (Double)uiSad + (Double)iModeBits * m_pcRdCost->getSqrtLambda();
                CandNum += xUpdateCandList( uiMode, cost, numModesForFullRD, uiRdModeList, CandCostList );

                bSatdChecked[uiMode] = true; // Mark as checked
              }
            }
          }
        }
      }
#endif

#if FAST_UDI_USE_MPM
#if !QC_USE_65ANG_MODES
      Int uiPreds[3] = {-1, -1, -1};
      Int iMode = -1;
      Int numCand = pcCU->getIntraDirLumaPredictor( uiPartOffset, uiPreds, &iMode );
#endif
      if( iMode >= 0 )
      {
        numCand = iMode;
      }
      
      for( Int j=0; j < numCand; j++)
      {
        Bool mostProbableModeIncluded = false;
        Int mostProbableMode = uiPreds[j];
        
        for( Int i=0; i < numModesForFullRD; i++)
        {
          mostProbableModeIncluded |= (mostProbableMode == uiRdModeList[i]);
        }
        if (!mostProbableModeIncluded)
        {
          uiRdModeList[numModesForFullRD++] = mostProbableMode;
        }
      }
#endif // FAST_UDI_USE_MPM
    }
    else
    {
      for( Int i=0; i < numModesForFullRD; i++)
      {
        uiRdModeList[i] = i;
      }
    }
#if QC_EMT_INTRA
    if( ucEmtUsageFlag==1 )
    {
      // Store the modes to be checked with RD
      uiSavedNumRdModes[uiPU] = numModesForFullRD;
      ::memcpy( uiSavedRdModeList[uiPU], uiRdModeList, numModesForFullRD*sizeof(UInt) );
    }
    }
    else if( ucEmtUsageFlag==2 )
    {
#if QC_EMT_INTRA_FAST
      if( bAllIntra && m_pcEncCfg->getUseFastIntraEMT() )
      {
        double dThrFastMode;
        
        switch(uiWidth)
        {
        case  4: dThrFastMode =   QC_EMT_INTRA_SKIP_CHECK_4x4_THR; break;
        case  8: dThrFastMode =   QC_EMT_INTRA_SKIP_CHECK_8x8_THR; break;
        case 16: dThrFastMode = QC_EMT_INTRA_SKIP_CHECK_16x16_THR; break;
        case 32: dThrFastMode = QC_EMT_INTRA_SKIP_CHECK_32x32_THR; break;
        default: dThrFastMode = QC_EMT_INTRA_SKIP_CHECK_32x32_THR; break;
        }

        numModesForFullRD=0;

        // Skip checking the modes with much larger R-D cost than the best mode
        for( Int i=0; i < uiSavedNumRdModes[uiPU]; i++)
        {
#if QC_USE_65ANG_MODES
          if( dModeCostStore[uiPU][i] <= dThrFastMode * dBestModeCostStore[uiPU] )
#else
          if( dModeCostStore[uiPU][i] < dThrFastMode * dBestModeCostStore[uiPU] )
#endif
          {
            uiRdModeList[numModesForFullRD++] = uiSavedRdModeList[uiPU][i];
          }
        }
      }
      else
#endif
      {
        // Restore the modes to be checked with RD
        numModesForFullRD = uiSavedNumRdModes[uiPU];
        ::memcpy( uiRdModeList, uiSavedRdModeList[uiPU], numModesForFullRD*sizeof(UInt) );
      }
    }
#endif

    //===== check modes (using r-d costs) =====
#if HHI_RQT_INTRA_SPEEDUP_MOD
    UInt   uiSecondBestMode  = MAX_UINT;
    Double dSecondBestPUCost = MAX_DOUBLE;
#endif
    
    UInt    uiBestPUMode  = 0;
    UInt    uiBestPUDistY = 0;
    UInt    uiBestPUDistC = 0;
    Double  dBestPUCost   = MAX_DOUBLE;
    for( UInt uiMode = 0; uiMode < numModesForFullRD; uiMode++ )
    {
      // set luma prediction mode
      UInt uiOrgMode = uiRdModeList[uiMode];
      
      pcCU->setLumaIntraDirSubParts ( uiOrgMode, uiPartOffset, uiDepth + uiInitTrDepth );
      
      // set context models
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );
      
      // determine residual for partition
      UInt   uiPUDistY = 0;
      UInt   uiPUDistC = 0;
      Double dPUCost   = 0.0;
#if HHI_RQT_INTRA_SPEEDUP
      xRecurIntraCodingQT( pcCU, uiInitTrDepth, uiPartOffset, bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, true, dPUCost );
#else
      xRecurIntraCodingQT( pcCU, uiInitTrDepth, uiPartOffset, bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost );
#endif
      
#if QC_EMT_INTRA_FAST
      if ( 1==ucEmtUsageFlag )
      {
        dModeCostStore[uiPU][uiMode] = dPUCost;
      }
#endif

      // check r-d cost
      if( dPUCost < dBestPUCost )
      {
#if HHI_RQT_INTRA_SPEEDUP_MOD
        uiSecondBestMode  = uiBestPUMode;
        dSecondBestPUCost = dBestPUCost;
#endif
        uiBestPUMode  = uiOrgMode;
        uiBestPUDistY = uiPUDistY;
        uiBestPUDistC = uiPUDistC;
        dBestPUCost   = dPUCost;
        
#if QC_EMT_INTRA_FAST
        if ( 1==ucEmtUsageFlag )
        {
          dBestModeCostStore[uiPU] = dPUCost;
        }
#endif

        xSetIntraResultQT( pcCU, uiInitTrDepth, uiPartOffset, bLumaOnly, pcRecoYuv );
        
        UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth(0) + uiInitTrDepth ) << 1 );
#if QC_EMT
        ::memcpy( m_puhQTTempEmtTuIdx,   pcCU->getEmtTuIdx()+uiPartOffset,  uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempEmtCuFlag,  pcCU->getEmtCuFlag()+uiPartOffset, uiQPartNum * sizeof( UChar ) );
#endif
        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx()       + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempCbf[0], pcCU->getCbf( TEXT_LUMA     ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempCbf[1], pcCU->getCbf( TEXT_CHROMA_U ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempCbf[2], pcCU->getCbf( TEXT_CHROMA_V ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempTransformSkipFlag[0], pcCU->getTransformSkip(TEXT_LUMA)     + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempTransformSkipFlag[1], pcCU->getTransformSkip(TEXT_CHROMA_U) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempTransformSkipFlag[2], pcCU->getTransformSkip(TEXT_CHROMA_V) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
#if INTRA_KLT
    ::memcpy( m_puhQTTempKLTFlag[0], pcCU->getKLTFlag(TEXT_LUMA) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
    ::memcpy( m_puhQTTempKLTFlag[1], pcCU->getKLTFlag(TEXT_CHROMA_U) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
    ::memcpy( m_puhQTTempKLTFlag[2], pcCU->getKLTFlag(TEXT_CHROMA_V) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
#endif
      }
#if HHI_RQT_INTRA_SPEEDUP_MOD
      else if( dPUCost < dSecondBestPUCost )
      {
        uiSecondBestMode  = uiOrgMode;
        dSecondBestPUCost = dPUCost;
      }
#endif
    } // Mode loop
    
#if HHI_RQT_INTRA_SPEEDUP
#if HHI_RQT_INTRA_SPEEDUP_MOD
    for( UInt ui =0; ui < 2; ++ui )
#endif
    {
#if HHI_RQT_INTRA_SPEEDUP_MOD
      UInt uiOrgMode   = ui ? uiSecondBestMode  : uiBestPUMode;
      if( uiOrgMode == MAX_UINT )
      {
        break;
      }
#else
      UInt uiOrgMode = uiBestPUMode;
#endif
      
      pcCU->setLumaIntraDirSubParts ( uiOrgMode, uiPartOffset, uiDepth + uiInitTrDepth );
      
      // set context models
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );
      
      // determine residual for partition
      UInt   uiPUDistY = 0;
      UInt   uiPUDistC = 0;
      Double dPUCost   = 0.0;
      xRecurIntraCodingQT( pcCU, uiInitTrDepth, uiPartOffset, bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, false, dPUCost );
      
      // check r-d cost
      if( dPUCost < dBestPUCost )
      {
        uiBestPUMode  = uiOrgMode;
        uiBestPUDistY = uiPUDistY;
        uiBestPUDistC = uiPUDistC;
        dBestPUCost   = dPUCost;
        
        xSetIntraResultQT( pcCU, uiInitTrDepth, uiPartOffset, bLumaOnly, pcRecoYuv );
        
        UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth(0) + uiInitTrDepth ) << 1 );
#if QC_EMT 
        ::memcpy( m_puhQTTempEmtTuIdx,   pcCU->getEmtTuIdx()+uiPartOffset,  uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempEmtCuFlag,  pcCU->getEmtCuFlag()+uiPartOffset, uiQPartNum * sizeof( UChar ) );
#endif
        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx()       + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempCbf[0], pcCU->getCbf( TEXT_LUMA     ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempCbf[1], pcCU->getCbf( TEXT_CHROMA_U ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempCbf[2], pcCU->getCbf( TEXT_CHROMA_V ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempTransformSkipFlag[0], pcCU->getTransformSkip(TEXT_LUMA)     + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempTransformSkipFlag[1], pcCU->getTransformSkip(TEXT_CHROMA_U) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempTransformSkipFlag[2], pcCU->getTransformSkip(TEXT_CHROMA_V) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
#if INTRA_KLT
    ::memcpy( m_puhQTTempKLTFlag[0], pcCU->getKLTFlag(TEXT_LUMA) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
    ::memcpy( m_puhQTTempKLTFlag[1], pcCU->getKLTFlag(TEXT_CHROMA_U) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
    ::memcpy( m_puhQTTempKLTFlag[2], pcCU->getKLTFlag(TEXT_CHROMA_V) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
#endif
      }
    } // Mode loop
#endif
    
    //--- update overall distortion ---
    uiOverallDistY += uiBestPUDistY;
    uiOverallDistC += uiBestPUDistC;
    
    //--- update transform index and cbf ---
    UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth(0) + uiInitTrDepth ) << 1 );
#if QC_EMT
    ::memcpy( pcCU->getEmtTuIdx()      + uiPartOffset,  m_puhQTTempEmtTuIdx,  uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getEmtCuFlag()     + uiPartOffset, m_puhQTTempEmtCuFlag,  uiQPartNum * sizeof( UChar ) );
#endif
    ::memcpy( pcCU->getTransformIdx()       + uiPartOffset, m_puhQTTempTrIdx,  uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getCbf( TEXT_LUMA     ) + uiPartOffset, m_puhQTTempCbf[0], uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getCbf( TEXT_CHROMA_U ) + uiPartOffset, m_puhQTTempCbf[1], uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getCbf( TEXT_CHROMA_V ) + uiPartOffset, m_puhQTTempCbf[2], uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getTransformSkip(TEXT_LUMA)     + uiPartOffset, m_puhQTTempTransformSkipFlag[0], uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getTransformSkip(TEXT_CHROMA_U) + uiPartOffset, m_puhQTTempTransformSkipFlag[1], uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getTransformSkip(TEXT_CHROMA_V) + uiPartOffset, m_puhQTTempTransformSkipFlag[2], uiQPartNum * sizeof( UChar ) );
#if INTRA_KLT
  ::memcpy( pcCU->getKLTFlag(TEXT_LUMA) + uiPartOffset, m_puhQTTempKLTFlag[0], uiQPartNum * sizeof( UChar ) );
  ::memcpy( pcCU->getKLTFlag(TEXT_CHROMA_U) + uiPartOffset, m_puhQTTempKLTFlag[1], uiQPartNum * sizeof( UChar ) );
  ::memcpy( pcCU->getKLTFlag(TEXT_CHROMA_V) + uiPartOffset, m_puhQTTempKLTFlag[2], uiQPartNum * sizeof( UChar ) );
#endif
    //--- set reconstruction for next intra prediction blocks ---
    if( uiPU != uiNumPU - 1 )
    {
      Bool bSkipChroma  = false;
      Bool bChromaSame  = false;
      UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> ( pcCU->getDepth(0) + uiInitTrDepth ) ] + 2;
      if( !bLumaOnly && uiLog2TrSize == 2 )
      {
        assert( uiInitTrDepth  > 0 );
        bSkipChroma  = ( uiPU != 0 );
        bChromaSame  = true;
      }
      
      UInt    uiCompWidth   = pcCU->getWidth ( 0 ) >> uiInitTrDepth;
      UInt    uiCompHeight  = pcCU->getHeight( 0 ) >> uiInitTrDepth;
      UInt    uiZOrder      = pcCU->getZorderIdxInCU() + uiPartOffset;
      Pel*    piDes         = pcCU->getPic()->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), uiZOrder );
      UInt    uiDesStride   = pcCU->getPic()->getPicYuvRec()->getStride();
      Pel*    piSrc         = pcRecoYuv->getLumaAddr( uiPartOffset );
      UInt    uiSrcStride   = pcRecoYuv->getStride();
      for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
        {
          piDes[ uiX ] = piSrc[ uiX ];
        }
      }
      if( !bLumaOnly && !bSkipChroma )
      {
        if( !bChromaSame )
        {
          uiCompWidth   >>= 1;
          uiCompHeight  >>= 1;
        }
        piDes         = pcCU->getPic()->getPicYuvRec()->getCbAddr( pcCU->getAddr(), uiZOrder );
        uiDesStride   = pcCU->getPic()->getPicYuvRec()->getCStride();
        piSrc         = pcRecoYuv->getCbAddr( uiPartOffset );
        uiSrcStride   = pcRecoYuv->getCStride();
        for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
        {
          for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
          {
            piDes[ uiX ] = piSrc[ uiX ];
          }
        }
        piDes         = pcCU->getPic()->getPicYuvRec()->getCrAddr( pcCU->getAddr(), uiZOrder );
        piSrc         = pcRecoYuv->getCrAddr( uiPartOffset );
        for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
        {
          for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
          {
            piDes[ uiX ] = piSrc[ uiX ];
          }
        }
      }
    }
    
    //=== update PU data ====
    pcCU->setLumaIntraDirSubParts     ( uiBestPUMode, uiPartOffset, uiDepth + uiInitTrDepth );
    pcCU->copyToPic                   ( uiDepth, uiPU, uiInitTrDepth );
  } // PU loop
  
  
  if( uiNumPU > 1 )
  { // set Cbf for all blocks
    UInt uiCombCbfY = 0;
    UInt uiCombCbfU = 0;
    UInt uiCombCbfV = 0;
    UInt uiPartIdx  = 0;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiPartIdx += uiQNumParts )
    {
      uiCombCbfY |= pcCU->getCbf( uiPartIdx, TEXT_LUMA,     1 );
      uiCombCbfU |= pcCU->getCbf( uiPartIdx, TEXT_CHROMA_U, 1 );
      uiCombCbfV |= pcCU->getCbf( uiPartIdx, TEXT_CHROMA_V, 1 );
    }
    for( UInt uiOffs = 0; uiOffs < 4 * uiQNumParts; uiOffs++ )
    {
      pcCU->getCbf( TEXT_LUMA     )[ uiOffs ] |= uiCombCbfY;
      pcCU->getCbf( TEXT_CHROMA_U )[ uiOffs ] |= uiCombCbfU;
      pcCU->getCbf( TEXT_CHROMA_V )[ uiOffs ] |= uiCombCbfV;
    }
  }
  
  //===== reset context models =====
  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
  
  //===== set distortion (rate and r-d costs are determined later) =====
  ruiDistC                   = uiOverallDistC;
  pcCU->getTotalDistortion() = uiOverallDistY + uiOverallDistC;
}



Void 
TEncSearch::estIntraPredChromaQT( TComDataCU* pcCU, 
                                 TComYuv*    pcOrgYuv, 
                                 TComYuv*    pcPredYuv, 
                                 TComYuv*    pcResiYuv, 
                                 TComYuv*    pcRecoYuv,
                                 UInt        uiPreCalcDistC )
{
  UInt    uiDepth     = pcCU->getDepth(0);
  UInt    uiBestMode  = 0;
  UInt    uiBestDist  = 0;
  Double  dBestCost   = MAX_DOUBLE;
  
  //----- init mode list -----
  UInt  uiMinMode = 0;
  UInt  uiModeList[ NUM_CHROMA_MODE ];
  pcCU->getAllowedChromaDir( 0, uiModeList );
  UInt  uiMaxMode = NUM_CHROMA_MODE;

  //----- check chroma modes -----
  for( UInt uiMode = uiMinMode; uiMode < uiMaxMode; uiMode++ )
  {
#if QC_LMCHROMA
    if ( !pcCU->getSlice()->getSPS()->getUseLMChroma() && uiModeList[uiMode] == LM_CHROMA_IDX )
    {
      continue;
    }
#endif
    //----- restore context models -----
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );
    
    //----- chroma coding -----
    UInt    uiDist = 0;
    pcCU->setChromIntraDirSubParts  ( uiModeList[uiMode], 0, uiDepth );
    xRecurIntraChromaCodingQT       ( pcCU,   0, 0, pcOrgYuv, pcPredYuv, pcResiYuv, uiDist );
    if( pcCU->getSlice()->getPPS()->getUseTransformSkip() )
    {
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );
    }
    UInt    uiBits = xGetIntraBitsQT( pcCU,   0, 0, false, true, false );
    Double  dCost  = m_pcRdCost->calcRdCost( uiBits, uiDist );

    //----- compare -----
    if( dCost < dBestCost )
    {
      dBestCost   = dCost;
      uiBestDist  = uiDist;
      uiBestMode  = uiModeList[uiMode];
      UInt  uiQPN = pcCU->getPic()->getNumPartInCU() >> ( uiDepth << 1 );
      xSetIntraResultChromaQT( pcCU, 0, 0, pcRecoYuv );
      ::memcpy( m_puhQTTempCbf[1], pcCU->getCbf( TEXT_CHROMA_U ), uiQPN * sizeof( UChar ) );
      ::memcpy( m_puhQTTempCbf[2], pcCU->getCbf( TEXT_CHROMA_V ), uiQPN * sizeof( UChar ) );
      ::memcpy( m_puhQTTempTransformSkipFlag[1], pcCU->getTransformSkip( TEXT_CHROMA_U ), uiQPN * sizeof( UChar ) );
      ::memcpy( m_puhQTTempTransformSkipFlag[2], pcCU->getTransformSkip( TEXT_CHROMA_V ), uiQPN * sizeof( UChar ) );
    }
  }
  
  //----- set data -----
  UInt  uiQPN = pcCU->getPic()->getNumPartInCU() >> ( uiDepth << 1 );
  ::memcpy( pcCU->getCbf( TEXT_CHROMA_U ), m_puhQTTempCbf[1], uiQPN * sizeof( UChar ) );
  ::memcpy( pcCU->getCbf( TEXT_CHROMA_V ), m_puhQTTempCbf[2], uiQPN * sizeof( UChar ) );
  ::memcpy( pcCU->getTransformSkip( TEXT_CHROMA_U ), m_puhQTTempTransformSkipFlag[1], uiQPN * sizeof( UChar ) );
  ::memcpy( pcCU->getTransformSkip( TEXT_CHROMA_V ), m_puhQTTempTransformSkipFlag[2], uiQPN * sizeof( UChar ) );
  pcCU->setChromIntraDirSubParts( uiBestMode, 0, uiDepth );
  pcCU->getTotalDistortion      () += uiBestDist - uiPreCalcDistC;
  
  //----- restore context models -----
  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );
}

/** Function for encoding and reconstructing luma/chroma samples of a PCM mode CU.
 * \param pcCU pointer to current CU
 * \param uiAbsPartIdx part index
 * \param piOrg pointer to original sample arrays
 * \param piPCM pointer to PCM code arrays
 * \param piPred pointer to prediction signal arrays
 * \param piResi pointer to residual signal arrays
 * \param piReco pointer to reconstructed sample arrays
 * \param uiStride stride of the original/prediction/residual sample arrays
 * \param uiWidth block width
 * \param uiHeight block height
 * \param ttText texture component type
 * \returns Void
 */
Void TEncSearch::xEncPCM (TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* piOrg, Pel* piPCM, Pel* piPred, Pel* piResi, Pel* piReco, UInt uiStride, UInt uiWidth, UInt uiHeight, TextType eText )
{
  UInt uiX, uiY;
  UInt uiReconStride;
  Pel* pOrg  = piOrg;
  Pel* pPCM  = piPCM;
  Pel* pPred = piPred;
  Pel* pResi = piResi;
  Pel* pReco = piReco;
  Pel* pRecoPic;
  Int shiftPcm;

  if( eText == TEXT_LUMA)
  {
    uiReconStride = pcCU->getPic()->getPicYuvRec()->getStride();
    pRecoPic      = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
    shiftPcm = g_bitDepthY - pcCU->getSlice()->getSPS()->getPCMBitDepthLuma();
  }
  else
  {
    uiReconStride = pcCU->getPic()->getPicYuvRec()->getCStride();

    if( eText == TEXT_CHROMA_U )
    {
      pRecoPic = pcCU->getPic()->getPicYuvRec()->getCbAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
    }
    else
    {
      pRecoPic = pcCU->getPic()->getPicYuvRec()->getCrAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
    }
    shiftPcm = g_bitDepthC - pcCU->getSlice()->getSPS()->getPCMBitDepthChroma();
  }

  // Reset pred and residual
  for( uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( uiX = 0; uiX < uiWidth; uiX++ )
    {
      pPred[uiX] = 0;
      pResi[uiX] = 0;
    }
    pPred += uiStride;
    pResi += uiStride;
  }

  // Encode
  for( uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( uiX = 0; uiX < uiWidth; uiX++ )
    {
      pPCM[uiX] = pOrg[uiX]>> shiftPcm;
    }
    pPCM += uiWidth;
    pOrg += uiStride;
  }

  pPCM  = piPCM;

  // Reconstruction
  for( uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( uiX = 0; uiX < uiWidth; uiX++ )
    {
      pReco   [uiX] = pPCM[uiX]<< shiftPcm;
      pRecoPic[uiX] = pReco[uiX];
    }
    pPCM += uiWidth;
    pReco += uiStride;
    pRecoPic += uiReconStride;
  }
}

/**  Function for PCM mode estimation.
 * \param pcCU
 * \param pcOrgYuv
 * \param rpcPredYuv
 * \param rpcResiYuv
 * \param rpcRecoYuv
 * \returns Void
 */
Void TEncSearch::IPCMSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv )
{
  UInt   uiDepth        = pcCU->getDepth(0);
  UInt   uiWidth        = pcCU->getWidth(0);
  UInt   uiHeight       = pcCU->getHeight(0);
  UInt   uiStride       = rpcPredYuv->getStride();
  UInt   uiStrideC      = rpcPredYuv->getCStride();
  UInt   uiWidthC       = uiWidth  >> 1;
  UInt   uiHeightC      = uiHeight >> 1;
  UInt   uiDistortion = 0;
  UInt   uiBits;

  Double dCost;

  Pel*    pOrig;
  Pel*    pResi;
  Pel*    pReco;
  Pel*    pPred;
  Pel*    pPCM;

  UInt uiAbsPartIdx = 0;

  UInt uiMinCoeffSize = pcCU->getPic()->getMinCUWidth()*pcCU->getPic()->getMinCUHeight();
  UInt uiLumaOffset   = uiMinCoeffSize*uiAbsPartIdx;
  UInt uiChromaOffset = uiLumaOffset>>2;

  // Luminance
  pOrig    = pcOrgYuv->getLumaAddr(0, uiWidth);
  pResi    = rpcResiYuv->getLumaAddr(0, uiWidth);
  pPred    = rpcPredYuv->getLumaAddr(0, uiWidth);
  pReco    = rpcRecoYuv->getLumaAddr(0, uiWidth);
  pPCM     = pcCU->getPCMSampleY() + uiLumaOffset;

  xEncPCM ( pcCU, 0, pOrig, pPCM, pPred, pResi, pReco, uiStride, uiWidth, uiHeight, TEXT_LUMA );

  // Chroma U
  pOrig    = pcOrgYuv->getCbAddr();
  pResi    = rpcResiYuv->getCbAddr();
  pPred    = rpcPredYuv->getCbAddr();
  pReco    = rpcRecoYuv->getCbAddr();
  pPCM     = pcCU->getPCMSampleCb() + uiChromaOffset;

  xEncPCM ( pcCU, 0, pOrig, pPCM, pPred, pResi, pReco, uiStrideC, uiWidthC, uiHeightC, TEXT_CHROMA_U );

  // Chroma V
  pOrig    = pcOrgYuv->getCrAddr();
  pResi    = rpcResiYuv->getCrAddr();
  pPred    = rpcPredYuv->getCrAddr();
  pReco    = rpcRecoYuv->getCrAddr();
  pPCM     = pcCU->getPCMSampleCr() + uiChromaOffset;

  xEncPCM ( pcCU, 0, pOrig, pPCM, pPred, pResi, pReco, uiStrideC, uiWidthC, uiHeightC, TEXT_CHROMA_V );

  m_pcEntropyCoder->resetBits();
  xEncIntraHeader ( pcCU, uiDepth, uiAbsPartIdx, true, false);
  uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();

  dCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

  pcCU->getTotalBits()       = uiBits;
  pcCU->getTotalCost()       = dCost;
  pcCU->getTotalDistortion() = uiDistortion;

  pcCU->copyToPic(uiDepth, 0, 0);
}

Void TEncSearch::xGetInterPredictionError( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, UInt& ruiErr, Bool bHadamard )
{
  motionCompensation( pcCU, &m_tmpYuvPred, REF_PIC_LIST_X, iPartIdx );

  UInt uiAbsPartIdx = 0;
  Int iWidth = 0;
  Int iHeight = 0;
  pcCU->getPartIndexAndSize( iPartIdx, uiAbsPartIdx, iWidth, iHeight );

  DistParam cDistParam;

  cDistParam.bApplyWeight = false;

  m_pcRdCost->setDistParam( cDistParam, g_bitDepthY,
                            pcYuvOrg->getLumaAddr( uiAbsPartIdx ), pcYuvOrg->getStride(), 
                            m_tmpYuvPred .getLumaAddr( uiAbsPartIdx ), m_tmpYuvPred .getStride(), 
                            iWidth, iHeight, m_pcEncCfg->getUseHADME() );
  ruiErr = cDistParam.DistFunc( &cDistParam );
}

/** estimation of best merge coding
 * \param pcCU
 * \param pcYuvOrg
 * \param iPUIdx
 * \param uiInterDir
 * \param pacMvField
 * \param uiMergeIndex
 * \param ruiCost
 * \param ruiBits
 * \param puhNeighCands
 * \param bValid 
 * \returns Void
 */
Void TEncSearch::xMergeEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPUIdx, UInt& uiInterDir, TComMvField* pacMvField, UInt& uiMergeIndex, UInt& ruiCost, TComMvField* cMvFieldNeighbours, UChar* uhInterDirNeighbours, Int& numValidMergeCand 
#if QC_SUB_PU_TMVP
                                  , UChar*          pMergeTypeNeighbor 
#if QC_SUB_PU_TMVP_EXT
                                  , TComMvField*    pcMvFieldSP[2]
                                  , UChar*          puhInterDirSP[2]
#else
                                  , TComMvField*    pcMvFieldSP
                                  , UChar*          puhInterDirSP
#endif
#endif
  )
{
  UInt uiAbsPartIdx = 0;
  Int iWidth = 0;
  Int iHeight = 0; 
#if QC_IC
  Bool abICFlag[MRG_MAX_NUM_CANDS];
#endif
  pcCU->getPartIndexAndSize( iPUIdx, uiAbsPartIdx, iWidth, iHeight );
  UInt uiDepth = pcCU->getDepth( uiAbsPartIdx );
  PartSize partSize = pcCU->getPartitionSize( 0 );
  if ( pcCU->getSlice()->getPPS()->getLog2ParallelMergeLevelMinus2() && partSize != SIZE_2Nx2N && pcCU->getWidth( 0 ) <= 8 )
  {
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );
    if ( iPUIdx == 0 )
    {
      pcCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand 
#if QC_IC
          , abICFlag
#endif
#if QC_SUB_PU_TMVP
          , pMergeTypeNeighbor
          , pcMvFieldSP
          , puhInterDirSP
#endif
        );
    }
    pcCU->setPartSizeSubParts( partSize, 0, uiDepth );
  }
  else
  {
    pcCU->getInterMergeCandidates( uiAbsPartIdx, iPUIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand 
#if QC_IC
          , abICFlag
#endif
#if QC_SUB_PU_TMVP
          , pMergeTypeNeighbor
          , pcMvFieldSP
          , puhInterDirSP
#endif
      );
  }
  xRestrictBipredMergeCand( pcCU, iPUIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );

  ruiCost = MAX_UINT;
  for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
  {
    UInt uiCostCand = MAX_UINT;
    UInt uiBitsCand = 0;
    
    PartSize ePartSize = pcCU->getPartitionSize( 0 );
#if QC_SUB_PU_TMVP
    pcCU->setMergeTypeSubParts( pMergeTypeNeighbor[uiMergeCand], uiAbsPartIdx, iPUIdx, pcCU->getDepth( uiAbsPartIdx ));  
#if QC_SUB_PU_TMVP_EXT
    if( pMergeTypeNeighbor[uiMergeCand]==MGR_TYPE_SUBPU_TMVP || pMergeTypeNeighbor[uiMergeCand]==MGR_TYPE_SUBPU_TMVP_EXT)
#else
    if( pMergeTypeNeighbor[uiMergeCand]==MGR_TYPE_SUBPU_TMVP )
#endif
    {

      UInt uiSPAddr;
      Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;
#if QC_SUB_PU_TMVP_EXT
      UInt uiSPListIndex = pMergeTypeNeighbor[uiMergeCand]==MGR_TYPE_SUBPU_TMVP?0:1;
#endif
      pcCU->getSPPara(iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);

      for (Int iPartitionIdx = 0; iPartitionIdx < iNumSP; iPartitionIdx++)
      {
        pcCU->getSPAbsPartIdx(uiAbsPartIdx, iSPWidth, iSPHeight, iPartitionIdx, iNumSPInOneLine, uiSPAddr);
#if QC_SUB_PU_TMVP_EXT
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setMvFieldSP(pcCU, uiSPAddr, m_pMvFieldSP[uiSPListIndex][2*iPartitionIdx], iSPWidth, iSPHeight);
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setMvFieldSP(pcCU, uiSPAddr, m_pMvFieldSP[uiSPListIndex][2*iPartitionIdx + 1], iSPWidth, iSPHeight);
#else
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setMvFieldSP(pcCU, uiSPAddr, pcMvFieldSP[2*iPartitionIdx], iSPWidth, iSPHeight);
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setMvFieldSP(pcCU, uiSPAddr, pcMvFieldSP[2*iPartitionIdx + 1], iSPWidth, iSPHeight);
#endif
      }
    }
    else 
    {
#endif
    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], ePartSize, uiAbsPartIdx, 0, iPUIdx );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], ePartSize, uiAbsPartIdx, 0, iPUIdx );
#if QC_SUB_PU_TMVP
    }
#endif


    xGetInterPredictionError( pcCU, pcYuvOrg, iPUIdx, uiCostCand, m_pcEncCfg->getUseHADME() );
    uiBitsCand = uiMergeCand + 1;
#if QC_SUB_PU_TMVP
#if MERGE_CAND_NUM_PATCH
    UInt uiMaxCand = m_pcEncCfg->getMaxNumMergeCand();
#else
    UInt uiMaxCand = m_pcEncCfg->getMaxNumMergeCand() + (m_pcEncCfg->getAtmvp()? 1:0);
#endif
    if (uiMergeCand == uiMaxCand -1)
#else
    if (uiMergeCand == m_pcEncCfg->getMaxNumMergeCand() -1)
#endif
    {
      uiBitsCand--;
    }
    uiCostCand = uiCostCand + m_pcRdCost->getCost( uiBitsCand );
    if ( uiCostCand < ruiCost )
    {
      ruiCost = uiCostCand;
      pacMvField[0] = cMvFieldNeighbours[0 + 2*uiMergeCand];
      pacMvField[1] = cMvFieldNeighbours[1 + 2*uiMergeCand];
      uiInterDir = uhInterDirNeighbours[uiMergeCand];
      uiMergeIndex = uiMergeCand;
    }
  }
}

#if QC_FRUC_MERGE
Void TEncSearch::xFRUCMgrEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPUIdx, TComMvField* pacMvField, 
  UChar * phInterDir , UChar ** phFRUCRefineDist , UChar ** phFRUCSBlkRefineDist ,
  UInt& ruiMinCost , UChar & ruhFRUCMode )
{
  UInt uiAbsPartIdx = 0;
  Int iWidth = 0;
  Int iHeight = 0; 
  pcCU->getPartIndexAndSize( iPUIdx, uiAbsPartIdx, iWidth, iHeight );
  UInt uiDepth = pcCU->getDepth( uiAbsPartIdx );
  ruiMinCost = MAX_UINT;

  const UChar uhFRUCME[2] = { QC_FRUC_MERGE_BILATERALMV , QC_FRUC_MERGE_TEMPLATE };
  pcCU->setMergeFlagSubParts( true, uiAbsPartIdx, iPUIdx, uiDepth );
  for( Int nME = 0 ; nME < 2 ; nME++ )
  {
    pcCU->setFRUCMgrModeSubParts( uhFRUCME[nME] , uiAbsPartIdx , iPUIdx , uiDepth );
    Bool bAvailable = deriveFRUCMV( pcCU , uiDepth , uiAbsPartIdx , iPUIdx );
    if( bAvailable )
    {
      UInt uiCostCand = 0;
      xGetInterPredictionError( pcCU, pcYuvOrg, iPUIdx, uiCostCand, m_pcEncCfg->getUseHADME() );
      UInt uiBitsCand = 1;
      UInt uiCost = uiCostCand + m_pcRdCost->getCost( uiBitsCand );
      if( uiCost < ruiMinCost )
      {
        ruiMinCost = uiCost;
        ruhFRUCMode = uhFRUCME[nME];
        for( Int y = 0 , yRasterOffset = 0 ; y < iHeight ; y += 4 , yRasterOffset += pcCU->getPic()->getNumPartInWidth() )
        {
          for( Int x = 0 , xRasterOffset = 0 ; x < iWidth ; x += 4 , xRasterOffset++ )
          {
            UInt idx = g_auiRasterToZscan[g_auiZscanToRaster[pcCU->getZorderIdxInCU() + uiAbsPartIdx] + yRasterOffset + xRasterOffset] - pcCU->getZorderIdxInCU();
            pacMvField[idx<<1].setMvField( pcCU->getCUMvField( REF_PIC_LIST_0 )->getMv( idx ) , pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( idx ) );
            pacMvField[(idx<<1)+1].setMvField( pcCU->getCUMvField( REF_PIC_LIST_1 )->getMv( idx ) , pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( idx ) );
            phInterDir[idx] = pcCU->getInterDir( idx );
          }
        }
      }
    }
  }
}
#endif

/** convert bi-pred merge candidates to uni-pred
 * \param pcCU
 * \param puIdx
 * \param mvFieldNeighbours
 * \param interDirNeighbours
 * \param numValidMergeCand
 * \returns Void
 */
Void TEncSearch::xRestrictBipredMergeCand( TComDataCU* pcCU, UInt puIdx, TComMvField* mvFieldNeighbours, UChar* interDirNeighbours, Int numValidMergeCand )
{
  if ( pcCU->isBipredRestriction(puIdx) )
  {
    for( UInt mergeCand = 0; mergeCand < numValidMergeCand; ++mergeCand )
    {
      if ( interDirNeighbours[mergeCand] == 3 )
      {
        interDirNeighbours[mergeCand] = 1;
        mvFieldNeighbours[(mergeCand << 1) + 1].setMvField(TComMv(0,0), -1);
      }
    }
  }
}

/** search of the best candidate for inter prediction
 * \param pcCU
 * \param pcOrgYuv
 * \param rpcPredYuv
 * \param rpcResiYuv
 * \param rpcRecoYuv
 * \param bUseRes
 * \returns Void
 */
#if AMP_MRG
#if QC_OBMC
Void TEncSearch::predInterSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv, TComYuv *pcPredYuvWoOBMC, TComYuv *pcTmpYuv1, TComYuv *pcTmpYuv2, Bool bUseRes, Bool bUseMRG )
#else
Void TEncSearch::predInterSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv, Bool bUseRes, Bool bUseMRG )
#endif
#else
Void TEncSearch::predInterSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv, Bool bUseRes )
#endif
{
#if !QC_LARGE_CTU
  m_acYuvPred[0].clear();
  m_acYuvPred[1].clear();
  m_cYuvPredTemp.clear();
  rpcPredYuv->clear();
#endif

  if ( !bUseRes )
  {
    rpcResiYuv->clear();
  }
  
#if !QC_LARGE_CTU
  rpcRecoYuv->clear();
#endif
  
  TComMv        cMvSrchRngLT;
  TComMv        cMvSrchRngRB;
  
  TComMv        cMvZero;
  TComMv        TempMv; //kolya
  
  TComMv        cMv[2];
  TComMv        cMvBi[2];
  TComMv        cMvTemp[2][33];
  
  Int           iNumPart    = pcCU->getNumPartitions();
  Int           iNumPredDir = pcCU->getSlice()->isInterP() ? 1 : 2;
  
  TComMv        cMvPred[2][33];
  
  TComMv        cMvPredBi[2][33];
  Int           aaiMvpIdxBi[2][33];
  
  Int           aaiMvpIdx[2][33];
  Int           aaiMvpNum[2][33];
  
  AMVPInfo aacAMVPInfo[2][33];
  
  Int           iRefIdx[2]={0,0}; //If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  Int           iRefIdxBi[2];
  
  UInt          uiPartAddr;
  Int           iRoiWidth, iRoiHeight;
  
  UInt          uiMbBits[3] = {1, 1, 0};
  
  UInt          uiLastMode = 0;
  Int           iRefStart, iRefEnd;
  
  PartSize      ePartSize = pcCU->getPartitionSize( 0 );

  Int           bestBiPRefIdxL1 = 0;
  Int           bestBiPMvpL1 = 0;
  UInt          biPDistTemp = MAX_INT;

#if ZERO_MVD_EST
  Int           aiZeroMvdMvpIdx[2] = {-1, -1};
  Int           aiZeroMvdRefIdx[2] = {0, 0};
  Int           iZeroMvdDir = -1;
#endif

  TComMvField cMvFieldNeighbours[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
#if QC_SUB_PU_TMVP
  UChar    eMergeCandTypeNieghors[MRG_MAX_NUM_CANDS];
  for ( Int i=0; i< MRG_MAX_NUM_CANDS ; i++)
    eMergeCandTypeNieghors[i] = MGR_TYPE_DEFAULT_N;
#endif
  Int numValidMergeCand = 0 ;

  for ( Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx++ )
  {
#if QC_OBMC
    //consider OBMC in motion estimation
    pcOrgYuv->copyFromPicYuv( pcCU->getPic()->getPicYuvOrg(), pcCU->getAddr(), pcCU->getZorderIdxInCU() );
    subBlockOBMC( pcCU, 0, pcOrgYuv, pcTmpYuv1, pcTmpYuv2, true );
#endif
    UInt          uiCost[2] = { MAX_UINT, MAX_UINT };
    UInt          uiCostBi  =   MAX_UINT;
    UInt          uiCostTemp;
    
    UInt          uiBits[3];
    UInt          uiBitsTemp;
#if ZERO_MVD_EST
    UInt          uiZeroMvdCost = MAX_UINT;
    UInt          uiZeroMvdCostTemp;
    UInt          uiZeroMvdBitsTemp;
    UInt          uiZeroMvdDistTemp = MAX_UINT;
    UInt          auiZeroMvdBits[3];
#endif
    UInt          bestBiPDist = MAX_INT;

    UInt          uiCostTempL0[MAX_NUM_REF];
    for (Int iNumRef=0; iNumRef < MAX_NUM_REF; iNumRef++)
    {
      uiCostTempL0[iNumRef] = MAX_UINT;
    }
    UInt          uiBitsTempL0[MAX_NUM_REF];

    TComMv        mvValidList1;
    Int           refIdxValidList1 = 0;
    UInt          bitsValidList1 = MAX_UINT;
    UInt          costValidList1 = MAX_UINT;

    xGetBlkBits( ePartSize, pcCU->getSlice()->isInterP(), iPartIdx, uiLastMode, uiMbBits);
    
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );
    
#if AMP_MRG
    Bool bTestNormalMC = true;
    
    if ( bUseMRG && pcCU->getWidth( 0 ) > 8 && iNumPart == 2 )
    {
      bTestNormalMC = false;
    }
    
    if (bTestNormalMC)
    {
#endif

    //  Uni-directional prediction
    for ( Int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
    {
      RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
      
      for ( Int iRefIdxTemp = 0; iRefIdxTemp < pcCU->getSlice()->getNumRefIdx(eRefPicList); iRefIdxTemp++ )
      {
        uiBitsTemp = uiMbBits[iRefList];
        if ( pcCU->getSlice()->getNumRefIdx(eRefPicList) > 1 )
        {
          uiBitsTemp += iRefIdxTemp+1;
          if ( iRefIdxTemp == pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 ) uiBitsTemp--;
        }
#if ZERO_MVD_EST
        xEstimateMvPredAMVP( pcCU, pcOrgYuv, iPartIdx, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], false, &biPDistTemp, &uiZeroMvdDistTemp);
#else
        xEstimateMvPredAMVP( pcCU, pcOrgYuv, iPartIdx, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], false, &biPDistTemp);
#endif
        aaiMvpIdx[iRefList][iRefIdxTemp] = pcCU->getMVPIdx(eRefPicList, uiPartAddr);
        aaiMvpNum[iRefList][iRefIdxTemp] = pcCU->getMVPNum(eRefPicList, uiPartAddr);
        
        if(pcCU->getSlice()->getMvdL1ZeroFlag() && iRefList==1 && biPDistTemp < bestBiPDist)
        {
          bestBiPDist = biPDistTemp;
          bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
          bestBiPRefIdxL1 = iRefIdxTemp;
        }

        uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];
#if ZERO_MVD_EST
        if ( iRefList == 0 || pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp ) < 0 )
        {
          uiZeroMvdBitsTemp = uiBitsTemp;
          uiZeroMvdBitsTemp += 2; //zero mvd bits

          m_pcRdCost->getMotionCost( 1, 0 );
          uiZeroMvdCostTemp = uiZeroMvdDistTemp + m_pcRdCost->getCost(uiZeroMvdBitsTemp);

          if (uiZeroMvdCostTemp < uiZeroMvdCost)
          {
            uiZeroMvdCost = uiZeroMvdCostTemp;
            iZeroMvdDir = iRefList + 1;
            aiZeroMvdRefIdx[iRefList] = iRefIdxTemp;
            aiZeroMvdMvpIdx[iRefList] = aaiMvpIdx[iRefList][iRefIdxTemp];
            auiZeroMvdBits[iRefList] = uiZeroMvdBitsTemp;
          }          
        }
#endif
        
#if GPB_SIMPLE_UNI
        if ( iRefList == 1 )    // list 1
        {
          if ( pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp ) >= 0 )
          {
            cMvTemp[1][iRefIdxTemp] = cMvTemp[0][pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp )];
            uiCostTemp = uiCostTempL0[pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp )];
            /*first subtract the bit-rate part of the cost of the other list*/
            uiCostTemp -= m_pcRdCost->getCost( uiBitsTempL0[pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp )] );
            /*correct the bit-rate part of the current ref*/
            m_pcRdCost->setPredictor  ( cMvPred[iRefList][iRefIdxTemp] );
            uiBitsTemp += m_pcRdCost->getBits( 
#if QC_MV_STORE_PRECISION_BIT
              cMvTemp[1][iRefIdxTemp].getHor() >> ( QC_MV_STORE_PRECISION_BIT - QC_MV_SIGNAL_PRECISION_BIT ) , cMvTemp[1][iRefIdxTemp].getVer() >> ( QC_MV_STORE_PRECISION_BIT - QC_MV_SIGNAL_PRECISION_BIT )
#else
              cMvTemp[1][iRefIdxTemp].getHor(), cMvTemp[1][iRefIdxTemp].getVer() 
#endif
#if QC_IMV
              , pcCU->getiMVFlag( uiPartAddr )
#endif
              );
            /*calculate the correct cost*/
            uiCostTemp += m_pcRdCost->getCost( uiBitsTemp );
          }
          else
          {
            xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
          }
        }
        else
        {
          xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
        }
#else
        xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
#endif
        xCopyAMVPInfo(pcCU->getCUMvField(eRefPicList)->getAMVPInfo(), &aacAMVPInfo[iRefList][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
        xCheckBestMVP(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp
#if QC_IMV
          , uiPartAddr
#endif
          );

        if ( iRefList == 0 )
        {
          uiCostTempL0[iRefIdxTemp] = uiCostTemp;
          uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
        }
        if ( uiCostTemp < uiCost[iRefList] )
        {
          uiCost[iRefList] = uiCostTemp;
          uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

          // set motion
          cMv[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
          iRefIdx[iRefList] = iRefIdxTemp;
        }

        if ( iRefList == 1 && uiCostTemp < costValidList1 && pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp ) < 0 )
        {
          costValidList1 = uiCostTemp;
          bitsValidList1 = uiBitsTemp;

          // set motion
          mvValidList1     = cMvTemp[iRefList][iRefIdxTemp];
          refIdxValidList1 = iRefIdxTemp;
        }
      }
    }
    //  Bi-directional prediction
#if QC_LARGE_CTU_FAST
    UChar uiMaxCUDepth = 0 , uiMinCUDepth = 0;
    pcCU->getMaxMinCUDepth( uiMinCUDepth , uiMaxCUDepth , pcCU->getZorderIdxInCU() );
    Bool bCheckBi = true;
    if( m_pcEncCfg->getLCTUFast() )
    {
      bCheckBi = pcCU->getDepth( 0 ) < uiMaxCUDepth || iRoiWidth * iRoiHeight >= 64;
    }
    if( bCheckBi )
    {
#endif
    if ( (pcCU->getSlice()->isInterB()) && (pcCU->isBipredRestriction(iPartIdx) == false) )
    {
      
      cMvBi[0] = cMv[0];            cMvBi[1] = cMv[1];
      iRefIdxBi[0] = iRefIdx[0];    iRefIdxBi[1] = iRefIdx[1];
      
      ::memcpy(cMvPredBi, cMvPred, sizeof(cMvPred));
      ::memcpy(aaiMvpIdxBi, aaiMvpIdx, sizeof(aaiMvpIdx));
      
      UInt uiMotBits[2];

      if(pcCU->getSlice()->getMvdL1ZeroFlag())
      {
        xCopyAMVPInfo(&aacAMVPInfo[1][bestBiPRefIdxL1], pcCU->getCUMvField(REF_PIC_LIST_1)->getAMVPInfo());
        pcCU->setMVPIdxSubParts( bestBiPMvpL1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;
        cMvPredBi[1][bestBiPRefIdxL1]   = pcCU->getCUMvField(REF_PIC_LIST_1)->getAMVPInfo()->m_acMvCand[bestBiPMvpL1];

        cMvBi[1] = cMvPredBi[1][bestBiPRefIdxL1];
        iRefIdxBi[1] = bestBiPRefIdxL1;
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( cMvBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( iRefIdxBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
        TComYuv* pcYuvPred = &m_acYuvPred[1];
        motionCompensation( pcCU, pcYuvPred, REF_PIC_LIST_1, iPartIdx );

        uiMotBits[0] = uiBits[0] - uiMbBits[0];
        uiMotBits[1] = uiMbBits[1];

        if ( pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1) > 1 )
        {
          uiMotBits[1] += bestBiPRefIdxL1+1;
          if ( bestBiPRefIdxL1 == pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1)-1 ) uiMotBits[1]--;
        }

        uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];

        uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];

        cMvTemp[1][bestBiPRefIdxL1] = cMvBi[1];
      }
      else
      {
        uiMotBits[0] = uiBits[0] - uiMbBits[0];
        uiMotBits[1] = uiBits[1] - uiMbBits[1];
        uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
      }

      // 4-times iteration (default)
      Int iNumIter = 4;
      
      // fast encoder setting: only one iteration
      if ( m_pcEncCfg->getUseFastEnc() || pcCU->getSlice()->getMvdL1ZeroFlag())
      {
        iNumIter = 1;
      }
      
      for ( Int iIter = 0; iIter < iNumIter; iIter++ )
      {
        
        Int         iRefList    = iIter % 2;
        if ( m_pcEncCfg->getUseFastEnc() )
        {
          if( uiCost[0] <= uiCost[1] )
          {
            iRefList = 1;
          }
          else
          {
            iRefList = 0;
          }
        }
        else if ( iIter == 0 )
        {
          iRefList = 0;
        }
        if ( iIter == 0 && !pcCU->getSlice()->getMvdL1ZeroFlag())
        {
          pcCU->getCUMvField(RefPicList(1-iRefList))->setAllMv( cMv[1-iRefList], ePartSize, uiPartAddr, 0, iPartIdx );
          pcCU->getCUMvField(RefPicList(1-iRefList))->setAllRefIdx( iRefIdx[1-iRefList], ePartSize, uiPartAddr, 0, iPartIdx );
          TComYuv*  pcYuvPred = &m_acYuvPred[1-iRefList];
          motionCompensation ( pcCU, pcYuvPred, RefPicList(1-iRefList), iPartIdx );
        }
        RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

        if(pcCU->getSlice()->getMvdL1ZeroFlag())
        {
          iRefList = 0;
          eRefPicList = REF_PIC_LIST_0;
        }

        Bool bChanged = false;
        
        iRefStart = 0;
        iRefEnd   = pcCU->getSlice()->getNumRefIdx(eRefPicList)-1;
        
        for ( Int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++ )
        {
          uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
          if ( pcCU->getSlice()->getNumRefIdx(eRefPicList) > 1 )
          {
            uiBitsTemp += iRefIdxTemp+1;
            if ( iRefIdxTemp == pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 ) uiBitsTemp--;
          }

          uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];
          // call ME
          xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, true );
          xCopyAMVPInfo(&aacAMVPInfo[iRefList][iRefIdxTemp], pcCU->getCUMvField(eRefPicList)->getAMVPInfo());
          xCheckBestMVP(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp
#if QC_IMV
            , uiPartAddr
#endif
            );
          if ( uiCostTemp < uiCostBi )
          {
            bChanged = true;
            
            cMvBi[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
            iRefIdxBi[iRefList] = iRefIdxTemp;
            
            uiCostBi            = uiCostTemp;
            uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1-iRefList];
            uiBits[2]           = uiBitsTemp;
            
            if(iNumIter!=1)
            {
              //  Set motion
              pcCU->getCUMvField( eRefPicList )->setAllMv( cMvBi[iRefList], ePartSize, uiPartAddr, 0, iPartIdx );
              pcCU->getCUMvField( eRefPicList )->setAllRefIdx( iRefIdxBi[iRefList], ePartSize, uiPartAddr, 0, iPartIdx );

              TComYuv* pcYuvPred = &m_acYuvPred[iRefList];
              motionCompensation( pcCU, pcYuvPred, eRefPicList, iPartIdx );
            }
          }
        } // for loop-iRefIdxTemp
        
        if ( !bChanged )
        {
          if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] )
          {
            xCopyAMVPInfo(&aacAMVPInfo[0][iRefIdxBi[0]], pcCU->getCUMvField(REF_PIC_LIST_0)->getAMVPInfo());
            xCheckBestMVP(pcCU, REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], uiBits[2], uiCostBi
#if QC_IMV
              , uiPartAddr
#endif
              );
            if(!pcCU->getSlice()->getMvdL1ZeroFlag())
            {
              xCopyAMVPInfo(&aacAMVPInfo[1][iRefIdxBi[1]], pcCU->getCUMvField(REF_PIC_LIST_1)->getAMVPInfo());
              xCheckBestMVP(pcCU, REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], uiBits[2], uiCostBi
#if QC_IMV
                , uiPartAddr
#endif
                );
            }
          }
          break;
        }
      } // for loop-iter
    } // if (B_SLICE)
#if QC_LARGE_CTU_FAST
    }
#endif
#if ZERO_MVD_EST
    if ( (pcCU->getSlice()->isInterB()) && (pcCU->isBipredRestriction(iPartIdx) == false) )
    {
      m_pcRdCost->getMotionCost( 1, 0 );

      for ( Int iL0RefIdxTemp = 0; iL0RefIdxTemp <= pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_0)-1; iL0RefIdxTemp++ )
      for ( Int iL1RefIdxTemp = 0; iL1RefIdxTemp <= pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1)-1; iL1RefIdxTemp++ )
      {
        UInt uiRefIdxBitsTemp = 0;
        if ( pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_0) > 1 )
        {
          uiRefIdxBitsTemp += iL0RefIdxTemp+1;
          if ( iL0RefIdxTemp == pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_0)-1 ) uiRefIdxBitsTemp--;
        }
        if ( pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1) > 1 )
        {
          uiRefIdxBitsTemp += iL1RefIdxTemp+1;
          if ( iL1RefIdxTemp == pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1)-1 ) uiRefIdxBitsTemp--;
        }

        Int iL0MVPIdx = 0;
        Int iL1MVPIdx = 0;

        for (iL0MVPIdx = 0; iL0MVPIdx < aaiMvpNum[0][iL0RefIdxTemp]; iL0MVPIdx++)
        {
          for (iL1MVPIdx = 0; iL1MVPIdx < aaiMvpNum[1][iL1RefIdxTemp]; iL1MVPIdx++)
          {
            uiZeroMvdBitsTemp = uiRefIdxBitsTemp;
            uiZeroMvdBitsTemp += uiMbBits[2];
            uiZeroMvdBitsTemp += m_auiMVPIdxCost[iL0MVPIdx][aaiMvpNum[0][iL0RefIdxTemp]] + m_auiMVPIdxCost[iL1MVPIdx][aaiMvpNum[1][iL1RefIdxTemp]];
            uiZeroMvdBitsTemp += 4; //zero mvd for both directions
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( aacAMVPInfo[0][iL0RefIdxTemp].m_acMvCand[iL0MVPIdx], iL0RefIdxTemp, ePartSize, uiPartAddr, iPartIdx, 0 );
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( aacAMVPInfo[1][iL1RefIdxTemp].m_acMvCand[iL1MVPIdx], iL1RefIdxTemp, ePartSize, uiPartAddr, iPartIdx, 0 );
  
            xGetInterPredictionError( pcCU, pcOrgYuv, iPartIdx, uiZeroMvdDistTemp, m_pcEncCfg->getUseHADME() );
            uiZeroMvdCostTemp = uiZeroMvdDistTemp + m_pcRdCost->getCost( uiZeroMvdBitsTemp );
            if (uiZeroMvdCostTemp < uiZeroMvdCost)
            {
              uiZeroMvdCost = uiZeroMvdCostTemp;
              iZeroMvdDir = 3;
              aiZeroMvdMvpIdx[0] = iL0MVPIdx;
              aiZeroMvdMvpIdx[1] = iL1MVPIdx;
              aiZeroMvdRefIdx[0] = iL0RefIdxTemp;
              aiZeroMvdRefIdx[1] = iL1RefIdxTemp;
              auiZeroMvdBits[2] = uiZeroMvdBitsTemp;
            }
          }
        }
      }
    }
#endif

#if AMP_MRG
    } //end if bTestNormalMC
#endif
    //  Clear Motion Field
    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( TComMvField(), ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( TComMvField(), ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( cMvZero,       ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( cMvZero,       ePartSize, uiPartAddr, 0, iPartIdx );

    pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    
    UInt uiMEBits = 0;
    // Set Motion Field_
    cMv[1] = mvValidList1;
    iRefIdx[1] = refIdxValidList1;
    uiBits[1] = bitsValidList1;
    uiCost[1] = costValidList1;
#if AMP_MRG
    if (bTestNormalMC)
    {
#endif
#if ZERO_MVD_EST
    if (uiZeroMvdCost <= uiCostBi && uiZeroMvdCost <= uiCost[0] && uiZeroMvdCost <= uiCost[1])
    {
      if (iZeroMvdDir == 3)
      {
        uiLastMode = 2;

        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( aacAMVPInfo[0][aiZeroMvdRefIdx[0]].m_acMvCand[aiZeroMvdMvpIdx[0]], aiZeroMvdRefIdx[0], ePartSize, uiPartAddr, iPartIdx, 0 );
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( aacAMVPInfo[1][aiZeroMvdRefIdx[1]].m_acMvCand[aiZeroMvdMvpIdx[1]], aiZeroMvdRefIdx[1], ePartSize, uiPartAddr, iPartIdx, 0 );
  
        pcCU->setInterDirSubParts( 3, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
        
        pcCU->setMVPIdxSubParts( aiZeroMvdMvpIdx[0], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( aaiMvpNum[0][aiZeroMvdRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPIdxSubParts( aiZeroMvdMvpIdx[1], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( aaiMvpNum[1][aiZeroMvdRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        uiMEBits = auiZeroMvdBits[2];
      }
      else if (iZeroMvdDir == 1)
      {        
        uiLastMode = 0;

        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( aacAMVPInfo[0][aiZeroMvdRefIdx[0]].m_acMvCand[aiZeroMvdMvpIdx[0]], aiZeroMvdRefIdx[0], ePartSize, uiPartAddr, iPartIdx, 0 );

        pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
        
        pcCU->setMVPIdxSubParts( aiZeroMvdMvpIdx[0], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( aaiMvpNum[0][aiZeroMvdRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        uiMEBits = auiZeroMvdBits[0];
      }
      else if (iZeroMvdDir == 2)
      {
        uiLastMode = 1;

        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( aacAMVPInfo[1][aiZeroMvdRefIdx[1]].m_acMvCand[aiZeroMvdMvpIdx[1]], aiZeroMvdRefIdx[1], ePartSize, uiPartAddr, iPartIdx, 0 );

        pcCU->setInterDirSubParts( 2, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
        
        pcCU->setMVPIdxSubParts( aiZeroMvdMvpIdx[1], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( aaiMvpNum[1][aiZeroMvdRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        uiMEBits = auiZeroMvdBits[1];
      }
      else
      {
        assert(0);
      }
    }
    else
#endif
    if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
    {
      uiLastMode = 2;
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMv( cMvBi[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllRefIdx( iRefIdxBi[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMv( cMvBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllRefIdx( iRefIdxBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
      
      TempMv = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );
      
      TempMv = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );
      
      pcCU->setInterDirSubParts( 3, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
      
      pcCU->setMVPIdxSubParts( aaiMvpIdxBi[0][iRefIdxBi[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[0][iRefIdxBi[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPIdxSubParts( aaiMvpIdxBi[1][iRefIdxBi[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[1][iRefIdxBi[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

      uiMEBits = uiBits[2];
    }
    else if ( uiCost[0] <= uiCost[1] )
    {
      uiLastMode = 0;
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMv( cMv[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllRefIdx( iRefIdx[0], ePartSize, uiPartAddr, 0, iPartIdx );
      TempMv = cMv[0] - cMvPred[0][iRefIdx[0]];
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
      
      pcCU->setMVPIdxSubParts( aaiMvpIdx[0][iRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[0][iRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

      uiMEBits = uiBits[0];
    }
    else
    {
      uiLastMode = 1;
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMv( cMv[1], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllRefIdx( iRefIdx[1], ePartSize, uiPartAddr, 0, iPartIdx );
      TempMv = cMv[1] - cMvPred[1][iRefIdx[1]];
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setInterDirSubParts( 2, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
      
      pcCU->setMVPIdxSubParts( aaiMvpIdx[1][iRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[1][iRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

      uiMEBits = uiBits[1];
    }
#if AMP_MRG
    } // end if bTestNormalMC
#endif

    if ( pcCU->getPartitionSize( uiPartAddr ) != SIZE_2Nx2N )
    {
      UInt uiMRGInterDir = 0;     
      TComMvField cMRGMvField[2];
      UInt uiMRGIndex = 0;

      UInt uiMEInterDir = 0;
      TComMvField cMEMvField[2];

      m_pcRdCost->getMotionCost( 1, 0 );
#if AMP_MRG
      // calculate ME cost
      UInt uiMEError = MAX_UINT;
      UInt uiMECost = MAX_UINT;

      if (bTestNormalMC)
      {
        xGetInterPredictionError( pcCU, pcOrgYuv, iPartIdx, uiMEError, m_pcEncCfg->getUseHADME() );
        uiMECost = uiMEError + m_pcRdCost->getCost( uiMEBits );
      }
#else
      // calculate ME cost
      UInt uiMEError = MAX_UINT;
      xGetInterPredictionError( pcCU, pcOrgYuv, iPartIdx, uiMEError, m_pcEncCfg->getUseHADME() );
      UInt uiMECost = uiMEError + m_pcRdCost->getCost( uiMEBits );
#endif 
      // save ME result.
      uiMEInterDir = pcCU->getInterDir( uiPartAddr );
      pcCU->getMvField( pcCU, uiPartAddr, REF_PIC_LIST_0, cMEMvField[0] );
      pcCU->getMvField( pcCU, uiPartAddr, REF_PIC_LIST_1, cMEMvField[1] );

      // find Merge result
      UInt uiMRGCost = MAX_UINT;
#if QC_IMV
      if( pcCU->getSlice()->getSPS()->getIMV() )
      {
        pcCU->setMergeFlagSubParts ( true,          uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
      }
#endif
#if QC_SUB_PU_TMVP
      memset(eMergeCandTypeNieghors,MGR_TYPE_DEFAULT_N, MRG_MAX_NUM_CANDS*sizeof(UChar));
#endif
      xMergeEstimation( pcCU, pcOrgYuv, iPartIdx, uiMRGInterDir, cMRGMvField, uiMRGIndex, uiMRGCost, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand
#if QC_SUB_PU_TMVP
      ,eMergeCandTypeNieghors, m_pMvFieldSP, m_phInterDirSP
#endif 
        );

#if QC_FRUC_MERGE
      UInt uiFRUCMgrCost = MAX_UINT;
      UChar uhFRUCMode = 0;
      if( pcCU->getSlice()->getSPS()->getUseFRUCMgrMode() )
      {
        xFRUCMgrEstimation( pcCU, pcOrgYuv, iPartIdx, 
          m_pMvFieldFRUC, m_phInterDirFRUC , m_phFRUCRefineDist , m_phFRUCSBlkRefineDist ,
          uiFRUCMgrCost , uhFRUCMode );
      }
#endif

      if ( uiMRGCost < uiMECost 
#if QC_FRUC_MERGE
        || uiFRUCMgrCost < uiMECost
#endif
        )
      {
        // set Merge result
#if QC_FRUC_MERGE
        if( uiFRUCMgrCost < uiMRGCost )
        {
          pcCU->setFRUCMgrModeSubParts( uhFRUCMode, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
          pcCU->setMergeFlagSubParts ( true, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
          for( Int y = 0 , yRasterOffset = 0 ; y < iRoiHeight ; y += 4 , yRasterOffset += pcCU->getPic()->getNumPartInWidth() )
          {
            for( Int x = 0 , xRasterOffset = 0 ; x < iRoiWidth ; x += 4 , xRasterOffset++  )
            {
              UInt idx = g_auiRasterToZscan[g_auiZscanToRaster[pcCU->getZorderIdxInCU() + uiPartAddr] + yRasterOffset + xRasterOffset] - pcCU->getZorderIdxInCU();
              pcCU->getCUMvField( REF_PIC_LIST_0 )->setMvFieldSP( pcCU , idx , m_pMvFieldFRUC[idx<<1] , 4 , 4 );
              pcCU->getCUMvField( REF_PIC_LIST_1 )->setMvFieldSP( pcCU , idx , m_pMvFieldFRUC[(idx<<1)+1] , 4 , 4 );
              pcCU->setInterDir( idx , m_phInterDirFRUC[idx] );
            }
          }
        }
        else
        {
          pcCU->setFRUCMgrModeSubParts( QC_FRUC_MERGE_OFF, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
#endif
        pcCU->setMergeFlagSubParts ( true,          uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->setMergeIndexSubParts( uiMRGIndex,    uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
#if QC_SUB_PU_TMVP
        pcCU->setMergeTypeSubParts( eMergeCandTypeNieghors[uiMRGIndex], uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) ); 
#if QC_SUB_PU_TMVP_EXT
        if (eMergeCandTypeNieghors[uiMRGIndex]==MGR_TYPE_SUBPU_TMVP || eMergeCandTypeNieghors[uiMRGIndex]==MGR_TYPE_SUBPU_TMVP_EXT)
#else
        if (eMergeCandTypeNieghors[uiMRGIndex]==MGR_TYPE_SUBPU_TMVP )
#endif
        {
          UInt uiSPAddr;
          Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;
          pcCU->getSPPara(iRoiWidth, iRoiHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);
#if QC_SUB_PU_TMVP_EXT
          UInt uiSPListIndex = eMergeCandTypeNieghors[uiMRGIndex]==MGR_TYPE_SUBPU_TMVP ? 0:1;
#endif
          for (Int iPartitionIdx = 0; iPartitionIdx < iNumSP; iPartitionIdx++)
          {
            pcCU->getSPAbsPartIdx(uiPartAddr, iSPWidth, iSPHeight, iPartitionIdx, iNumSPInOneLine, uiSPAddr);
#if QC_SUB_PU_TMVP_EXT
            pcCU->setInterDirSP(m_phInterDirSP[uiSPListIndex][iPartitionIdx], uiSPAddr, iSPWidth, iSPHeight);
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setMvFieldSP(pcCU, uiSPAddr, m_pMvFieldSP[uiSPListIndex][2*iPartitionIdx], iSPWidth, iSPHeight);
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setMvFieldSP(pcCU, uiSPAddr, m_pMvFieldSP[uiSPListIndex][2*iPartitionIdx + 1], iSPWidth, iSPHeight);
#else
            pcCU->setInterDirSP(m_phInterDirSP[iPartitionIdx], uiSPAddr, iSPWidth, iSPHeight);
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setMvFieldSP(pcCU, uiSPAddr, m_pMvFieldSP[2*iPartitionIdx], iSPWidth, iSPHeight);
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setMvFieldSP(pcCU, uiSPAddr, m_pMvFieldSP[2*iPartitionIdx + 1], iSPWidth, iSPHeight);
#endif
          }
          if ( pcCU->getInterDir(uiPartAddr) == 3 && pcCU->isBipredRestriction(iPartIdx) )
          {
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( TComMv(0,0), ePartSize, uiPartAddr, 0, iPartIdx);
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, ePartSize, uiPartAddr, 0, iPartIdx);
            pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ));
          }
        }
        else
        {
#endif
        pcCU->setInterDirSubParts  ( uiMRGInterDir, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMRGMvField[0], ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMRGMvField[1], ePartSize, uiPartAddr, 0, iPartIdx );
#if QC_SUB_PU_TMVP
        }
#endif
#if QC_FRUC_MERGE
        }
#endif

        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( cMvZero,            ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( cMvZero,            ePartSize, uiPartAddr, 0, iPartIdx );

        pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      }
      else
      {
#if QC_SUB_PU_TMVP
        pcCU->setMergeTypeSubParts( MGR_TYPE_DEFAULT_N , uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) ); 
#endif
#if QC_FRUC_MERGE
        pcCU->setFRUCMgrModeSubParts( QC_FRUC_MERGE_OFF, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
#endif
        // set ME result
        pcCU->setMergeFlagSubParts( false,        uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->setInterDirSubParts ( uiMEInterDir, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMEMvField[0], ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMEMvField[1], ePartSize, uiPartAddr, 0, iPartIdx );
      }
    }

    //  MC
    motionCompensation ( pcCU, rpcPredYuv, REF_PIC_LIST_X, iPartIdx );
    
  } //  end of for ( Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx++ )

#if QC_OBMC
  rpcPredYuv->copyToPartYuv( pcPredYuvWoOBMC, 0 );
  subBlockOBMC( pcCU, 0, rpcPredYuv, pcTmpYuv1, pcTmpYuv2 );
  //re-fetch original YUV
  pcOrgYuv->copyFromPicYuv( pcCU->getPic()->getPicYuvOrg(), pcCU->getAddr(), pcCU->getZorderIdxInCU() );
#endif
  // Before or After the following function call:setWpScalingDistParam
  setWpScalingDistParam( pcCU, -1, REF_PIC_LIST_X );

  return;
}

// AMVP
#if ZERO_MVD_EST
Void TEncSearch::xEstimateMvPredAMVP( TComDataCU* pcCU, TComYuv* pcOrgYuv, UInt uiPartIdx, RefPicList eRefPicList, Int iRefIdx, TComMv& rcMvPred, Bool bFilled, UInt* puiDistBiP, UInt* puiDist  )
#else
Void TEncSearch::xEstimateMvPredAMVP( TComDataCU* pcCU, TComYuv* pcOrgYuv, UInt uiPartIdx, RefPicList eRefPicList, Int iRefIdx, TComMv& rcMvPred, Bool bFilled, UInt* puiDistBiP )
#endif
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
  
  TComMv  cBestMv;
  Int     iBestIdx = 0;
  TComMv  cZeroMv;
  TComMv  cMvPred;
  UInt    uiBestCost = MAX_INT;
  UInt    uiPartAddr = 0;
  Int     iRoiWidth, iRoiHeight;
  Int     i;
  
  pcCU->getPartIndexAndSize( uiPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );
  // Fill the MV Candidates
  if (!bFilled)
  {
    pcCU->fillMvpCand( uiPartIdx, uiPartAddr, eRefPicList, iRefIdx, pcAMVPInfo 
#if QC_FRUC_MERGE
      , this
#endif
      );
  }
  
  // initialize Mvp index & Mvp
  iBestIdx = 0;
  cBestMv  = pcAMVPInfo->m_acMvCand[0];
#if !ZERO_MVD_EST
  if (pcAMVPInfo->iN <= 1)
  {
    rcMvPred = cBestMv;
    
    pcCU->setMVPIdxSubParts( iBestIdx, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));

    if(pcCU->getSlice()->getMvdL1ZeroFlag() && eRefPicList==REF_PIC_LIST_1)
    {
#if ZERO_MVD_EST
      (*puiDistBiP) = xGetTemplateCost( pcCU, uiPartIdx, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, rcMvPred, 0, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight, uiDist );
#else
      (*puiDistBiP) = xGetTemplateCost( pcCU, uiPartIdx, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, rcMvPred, 0, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight);
#endif
    }
    return;
  }
#endif  
  if (bFilled)
  {
    assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
    rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
    return;
  }
  
#if !QC_LARGE_CTU
  m_cYuvPredTemp.clear();
#endif
#if ZERO_MVD_EST
  UInt uiDist;
#endif
  //-- Check Minimum Cost.
  for ( i = 0 ; i < pcAMVPInfo->iN; i++)
  {
    UInt uiTmpCost;
#if ZERO_MVD_EST
    uiTmpCost = xGetTemplateCost( pcCU, uiPartIdx, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, pcAMVPInfo->m_acMvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight, uiDist );
#else
    uiTmpCost = xGetTemplateCost( pcCU, uiPartIdx, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, pcAMVPInfo->m_acMvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight);
#endif      
    if ( uiBestCost > uiTmpCost )
    {
      uiBestCost = uiTmpCost;
      cBestMv   = pcAMVPInfo->m_acMvCand[i];
      iBestIdx  = i;
      (*puiDistBiP) = uiTmpCost;
#if ZERO_MVD_EST
      (*puiDist) = uiDist;
#endif
    }
  }

#if !QC_LARGE_CTU
  m_cYuvPredTemp.clear();
#endif

  // Setting Best MVP
  rcMvPred = cBestMv;
  pcCU->setMVPIdxSubParts( iBestIdx, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
  pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
  return;
}

UInt TEncSearch::xGetMvpIdxBits(Int iIdx, Int iNum)
{
  assert(iIdx >= 0 && iNum >= 0 && iIdx < iNum);
  
  if (iNum == 1)
  {
    return 0;
  }
  
  UInt uiLength = 1;
  Int iTemp = iIdx;
  if ( iTemp == 0 )
  {
    return uiLength;
  }
  
  Bool bCodeLast = ( iNum-1 > iTemp );
  
  uiLength += (iTemp-1);
  
  if( bCodeLast )
  {
    uiLength++;
  }
  
  return uiLength;
}

Void TEncSearch::xGetBlkBits( PartSize eCUMode, Bool bPSlice, Int iPartIdx, UInt uiLastMode, UInt uiBlkBit[3])
{
  if ( eCUMode == SIZE_2Nx2N )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
  else if ( (eCUMode == SIZE_2NxN || eCUMode == SIZE_2NxnU) || eCUMode == SIZE_2NxnD )
  {
    UInt aauiMbBits[2][3][3] = { { {0,0,3}, {0,0,0}, {0,0,0} } , { {5,7,7}, {7,5,7}, {9-3,9-3,9-3} } };
    if ( bPSlice )
    {
      uiBlkBit[0] = 3;
      uiBlkBit[1] = 0;
      uiBlkBit[2] = 0;
    }
    else
    {
      ::memcpy( uiBlkBit, aauiMbBits[iPartIdx][uiLastMode], 3*sizeof(UInt) );
    }
  }
  else if ( (eCUMode == SIZE_Nx2N || eCUMode == SIZE_nLx2N) || eCUMode == SIZE_nRx2N )
  {
    UInt aauiMbBits[2][3][3] = { { {0,2,3}, {0,0,0}, {0,0,0} } , { {5,7,7}, {7-2,7-2,9-2}, {9-3,9-3,9-3} } };
    if ( bPSlice )
    {
      uiBlkBit[0] = 3;
      uiBlkBit[1] = 0;
      uiBlkBit[2] = 0;
    }
    else
    {
      ::memcpy( uiBlkBit, aauiMbBits[iPartIdx][uiLastMode], 3*sizeof(UInt) );
    }
  }
  else if ( eCUMode == SIZE_NxN )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
  else
  {
    printf("Wrong!\n");
    assert( 0 );
  }
}

Void TEncSearch::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->iN = pSrc->iN;
  for (Int i = 0; i < pSrc->iN; i++)
  {
    pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];
  }
}

Void TEncSearch::xCheckBestMVP ( TComDataCU* pcCU, RefPicList eRefPicList, TComMv cMv, TComMv& rcMvPred, Int& riMVPIdx, UInt& ruiBits, UInt& ruiCost 
#if QC_IMV
  , UInt uiPartAddr
#endif
  )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
  
  assert(pcAMVPInfo->m_acMvCand[riMVPIdx] == rcMvPred);
  
  if (pcAMVPInfo->iN < 2) return;
  
  m_pcRdCost->getMotionCost( 1, 0 );
  m_pcRdCost->setCostScale ( 0    );
  
  Int iBestMVPIdx = riMVPIdx;
  
  m_pcRdCost->setPredictor( rcMvPred );
  Int iOrgMvBits  = m_pcRdCost->getBits(
#if QC_MV_STORE_PRECISION_BIT
    cMv.getHor() >> (QC_MV_STORE_PRECISION_BIT-QC_MV_SIGNAL_PRECISION_BIT), cMv.getVer() >> (QC_MV_STORE_PRECISION_BIT-QC_MV_SIGNAL_PRECISION_BIT)
#else
    cMv.getHor(), cMv.getVer()
#endif
#if QC_IMV
    , pcCU->getiMVFlag( uiPartAddr )
#endif
    );

  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];
  Int iBestMvBits = iOrgMvBits;
  
  for (Int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->iN; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx) continue;
    
    m_pcRdCost->setPredictor( pcAMVPInfo->m_acMvCand[iMVPIdx] );
    Int iMvBits = m_pcRdCost->getBits(
#if QC_MV_STORE_PRECISION_BIT
      cMv.getHor() >> (QC_MV_STORE_PRECISION_BIT-QC_MV_SIGNAL_PRECISION_BIT), cMv.getVer() >> (QC_MV_STORE_PRECISION_BIT-QC_MV_SIGNAL_PRECISION_BIT)
#else
      cMv.getHor(), cMv.getVer()
#endif
#if QC_IMV
      , pcCU->getiMVFlag( uiPartAddr )
#endif
      );

    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];
    
    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }
  
  if (iBestMVPIdx != riMVPIdx)  //if changed
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[iBestMVPIdx];
    
    riMVPIdx = iBestMVPIdx;
    UInt uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits ))  + m_pcRdCost->getCost( ruiBits );
  }
}

UInt TEncSearch::xGetTemplateCost( TComDataCU* pcCU,
                                  UInt        uiPartIdx,
                                  UInt      uiPartAddr,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*    pcTemplateCand,
                                  TComMv      cMvCand,
                                  Int         iMVPIdx,
                                  Int     iMVPNum,
                                  RefPicList  eRefPicList,
                                  Int         iRefIdx,
                                  Int         iSizeX,
                                  Int         iSizeY
                               #if ZERO_MVD_EST
                                , UInt&       ruiDist
                               #endif
                                  )
{
  UInt uiCost  = MAX_INT;
  
  TComPicYuv* pcPicYuvRef = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec();
  
  pcCU->clipMv( cMvCand );
#if QC_IC
  Bool bICFlag = pcCU->getICFlag( uiPartAddr );
#endif
  // prediction pattern
  if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType()==P_SLICE 
#if QC_IC
      && !bICFlag
#endif
    )
  {
    xPredInterLumaBlk( pcCU, pcPicYuvRef, uiPartAddr, &cMvCand, iSizeX, iSizeY, pcTemplateCand, true 
#if BIO
,false
#endif
    );
  }
  else
  {
    xPredInterLumaBlk( pcCU, pcPicYuvRef, uiPartAddr, &cMvCand, iSizeX, iSizeY, pcTemplateCand, false
#if BIO
,false
#endif
#if QC_IC
#if QC_FRUC_MERGE
      , false
#endif
      , bICFlag
#endif
      );
  }

  if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType()==P_SLICE 
#if QC_IC
    && !bICFlag
#endif
    )
  {
    xWeightedPredictionUni( pcCU, pcTemplateCand, uiPartAddr, iSizeX, iSizeY, eRefPicList, pcTemplateCand, iRefIdx );
  }

  // calc distortion
#if ZERO_MVD_EST
  m_pcRdCost->getMotionCost( 1, 0 );
  DistParam cDistParam;
  m_pcRdCost->setDistParam( cDistParam, g_bitDepthY,
                            pcOrgYuv->getLumaAddr(uiPartAddr), pcOrgYuv->getStride(), 
                            pcTemplateCand->getLumaAddr(uiPartAddr), pcTemplateCand->getStride(), 
                            iSizeX, iSizeY, m_pcEncCfg->getUseHADME() );
  ruiDist = cDistParam.DistFunc( &cDistParam );
  uiCost = ruiDist + m_pcRdCost->getCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum] );
#else
  uiCost = m_pcRdCost->getDistPart(g_bitDepthY, pcTemplateCand->getLumaAddr(uiPartAddr), pcTemplateCand->getStride(), pcOrgYuv->getLumaAddr(uiPartAddr), pcOrgYuv->getStride(), iSizeX, iSizeY, TEXT_LUMA, DF_SAD );
  uiCost = (UInt) m_pcRdCost->calcRdCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum], uiCost, false, DF_SAD );
#endif
  return uiCost;
}

Void TEncSearch::xMotionEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, RefPicList eRefPicList, TComMv* pcMvPred, Int iRefIdxPred, TComMv& rcMv, UInt& ruiBits, UInt& ruiCost, Bool bBi )
{
  UInt          uiPartAddr;
  Int           iRoiWidth;
  Int           iRoiHeight;
  
  TComMv        cMvHalf, cMvQter;
  TComMv        cMvSrchRngLT;
  TComMv        cMvSrchRngRB;
  
  TComYuv*      pcYuv = pcYuvOrg;
  m_iSearchRange = m_aaiAdaptSR[eRefPicList][iRefIdxPred];
  
  Int           iSrchRng      = ( bBi ? m_bipredSearchRange : m_iSearchRange );
  TComPattern*  pcPatternKey  = pcCU->getPattern        ();
  
  Double        fWeight       = 1.0;
  
  pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );
#if QC_IMV
  pcPatternKey->setImvFlag( pcCU->getiMVFlag( uiPartAddr ) );
#endif
#if QC_IC
  Bool bICFlag = pcCU->getICFlag( uiPartAddr );
  pcPatternKey->setMRFlag( bICFlag );
#endif
  if ( bBi )
  {
    TComYuv*  pcYuvOther = &m_acYuvPred[1-(Int)eRefPicList];
    pcYuv                = &m_cYuvPredTemp;
    
    pcYuvOrg->copyPartToPartYuv( pcYuv, uiPartAddr, iRoiWidth, iRoiHeight );
    
    pcYuv->removeHighFreq( pcYuvOther, uiPartAddr, iRoiWidth, iRoiHeight );
    
    fWeight = 0.5;
  }
  
  //  Search key pattern initialization
  pcPatternKey->initPattern( pcYuv->getLumaAddr( uiPartAddr ),
                            pcYuv->getCbAddr  ( uiPartAddr ),
                            pcYuv->getCrAddr  ( uiPartAddr ),
                            iRoiWidth,
                            iRoiHeight,
                            pcYuv->getStride(),
                            0, 0 );
  
  Pel*        piRefY      = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRec()->getLumaAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr );
  Int         iRefStride  = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRec()->getStride();
  
  TComMv      cMvPred = *pcMvPred;
  
  if ( bBi )  xSetSearchRange   ( pcCU, rcMv   , iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
  else        xSetSearchRange   ( pcCU, cMvPred, iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
  
  m_pcRdCost->getMotionCost ( 1, 0 );
  
  m_pcRdCost->setPredictor  ( *pcMvPred );
  m_pcRdCost->setCostScale  ( 2 );
#if QC_IC
  if( bICFlag )
  {
    m_cDistParam.bApplyWeight = false;
  }
  else
#endif
  setWpScalingDistParam( pcCU, iRefIdxPred, eRefPicList );
  //  Do integer search
  if ( !m_iFastSearch || bBi )
  {
    xPatternSearch      ( pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost );
  }
  else
  {
    rcMv = *pcMvPred;
    xPatternSearchFast  ( pcCU, pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost );
  }
  
  m_pcRdCost->getMotionCost( 1, 0 );
  m_pcRdCost->setCostScale ( 1 );
  
  xPatternSearchFracDIF( pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost,bBi );
  
  m_pcRdCost->setCostScale( 0 );
#if QC_MV_STORE_PRECISION_BIT
  rcMv <<= QC_MV_STORE_PRECISION_BIT;
  rcMv += (cMvHalf <<= (QC_MV_STORE_PRECISION_BIT-1));
  rcMv += (cMvQter <<= (QC_MV_STORE_PRECISION_BIT-2));
  UInt uiMvBits = m_pcRdCost->getBits( rcMv.getHor() >> ( QC_MV_STORE_PRECISION_BIT-QC_MV_SIGNAL_PRECISION_BIT), rcMv.getVer() >> (QC_MV_STORE_PRECISION_BIT-QC_MV_SIGNAL_PRECISION_BIT)
#if QC_IMV
    , pcCU->getiMVFlag( uiPartAddr )
#endif
    );
#else
  rcMv <<= 2;
  rcMv += (cMvHalf <<= 1);
  rcMv +=  cMvQter;
  UInt uiMvBits = m_pcRdCost->getBits( rcMv.getHor(), rcMv.getVer() 
#if QC_IMV
    , pcCU->getiMVFlag( uiPartAddr )
#endif
    );
#endif
  
  ruiBits      += uiMvBits;
  ruiCost       = (UInt)( floor( fWeight * ( (Double)ruiCost - (Double)m_pcRdCost->getCost( uiMvBits ) ) ) + (Double)m_pcRdCost->getCost( ruiBits ) );
}


Void TEncSearch::xSetSearchRange ( TComDataCU* pcCU, TComMv& cMvPred, Int iSrchRng, TComMv& rcMvSrchRngLT, TComMv& rcMvSrchRngRB )
{
#if QC_MV_STORE_PRECISION_BIT
  Int  iMvShift = QC_MV_STORE_PRECISION_BIT;
#else
  Int  iMvShift = 2;
#endif
  TComMv cTmpMvPred = cMvPred;
  pcCU->clipMv( cTmpMvPred );

  rcMvSrchRngLT.setHor( cTmpMvPred.getHor() - (iSrchRng << iMvShift) );
  rcMvSrchRngLT.setVer( cTmpMvPred.getVer() - (iSrchRng << iMvShift) );
  
  rcMvSrchRngRB.setHor( cTmpMvPred.getHor() + (iSrchRng << iMvShift) );
  rcMvSrchRngRB.setVer( cTmpMvPred.getVer() + (iSrchRng << iMvShift) );
  pcCU->clipMv        ( rcMvSrchRngLT );
  pcCU->clipMv        ( rcMvSrchRngRB );
  
  rcMvSrchRngLT >>= iMvShift;
  rcMvSrchRngRB >>= iMvShift;
}

Void TEncSearch::xPatternSearch( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, UInt& ruiSAD )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  UInt  uiSad;
  UInt  uiSadBest         = MAX_UINT;
  Int   iBestX = 0;
  Int   iBestY = 0;
  
  Pel*  piRefSrch;
  
  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefY, iRefStride,  m_cDistParam );
  
  // fast encoder decision: use subsampled SAD for integer ME
  if ( m_pcEncCfg->getUseFastEnc() )
  {
    if ( m_cDistParam.iRows > 8 )
    {
      m_cDistParam.iSubShift = 1;
    }
  }
  
  piRefY += (iSrchRngVerTop * iRefStride);
  for ( Int y = iSrchRngVerTop; y <= iSrchRngVerBottom; y++ )
  {
    for ( Int x = iSrchRngHorLeft; x <= iSrchRngHorRight; x++ )
    {
      //  find min. distortion position
      piRefSrch = piRefY + x;
      m_cDistParam.pCur = piRefSrch;

      setDistParamComp(0);
#if QC_IC
      m_cDistParam.bMRFlag = pcPatternKey->getMRFlag();
#endif
      m_cDistParam.bitDepth = g_bitDepthY;
      uiSad = m_cDistParam.DistFunc( &m_cDistParam );
      
      // motion cost
      uiSad += m_pcRdCost->getCost( x, y 
#if QC_IMV
        , pcPatternKey->getImvFlag()
#endif
        );
      if ( uiSad < uiSadBest )
      {
        uiSadBest = uiSad;
        iBestX    = x;
        iBestY    = y;
      }
    }
    piRefY += iRefStride;
  }
  
  rcMv.set( iBestX, iBestY );
  ruiSAD = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY 
#if QC_IMV
    , pcPatternKey->getImvFlag()
#endif
    );
  return;
}

Void TEncSearch::xPatternSearchFast( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, UInt& ruiSAD )
{
  pcCU->getMvPredLeft       ( m_acMvPredictors[0] );
  pcCU->getMvPredAbove      ( m_acMvPredictors[1] );
  pcCU->getMvPredAboveRight ( m_acMvPredictors[2] );
  
  switch ( m_iFastSearch )
  {
    case 1:
      xTZSearch( pcCU, pcPatternKey, piRefY, iRefStride, pcMvSrchRngLT, pcMvSrchRngRB, rcMv, ruiSAD );
      break;
      
    default:
      break;
  }
}

Void TEncSearch::xTZSearch( TComDataCU* pcCU, TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, UInt& ruiSAD )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();
  
  TZ_SEARCH_CONFIGURATION
  
  UInt uiSearchRange = m_iSearchRange;
  pcCU->clipMv( rcMv );
#if QC_MV_STORE_PRECISION_BIT
  rcMv >>= QC_MV_STORE_PRECISION_BIT;
#else
  rcMv >>= 2;
#endif
  // init TZSearchStruct
  IntTZSearchStruct cStruct;
  cStruct.iYStride    = iRefStride;
  cStruct.piRefY      = piRefY;
  cStruct.uiBestSad   = MAX_UINT;
  
  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( pcPatternKey, cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );
  
  // test whether one of PRED_A, PRED_B, PRED_C MV is better start point than Median predictor
  if ( bTestOtherPredictedMV )
  {
    for ( UInt index = 0; index < 3; index++ )
    {
      TComMv cMv = m_acMvPredictors[index];
      pcCU->clipMv( cMv );
#if QC_MV_STORE_PRECISION_BIT
      cMv >>= QC_MV_STORE_PRECISION_BIT;
#else
      cMv >>= 2;
#endif
      xTZSearchHelp( pcPatternKey, cStruct, cMv.getHor(), cMv.getVer(), 0, 0 );
    }
  }
  
  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    xTZSearchHelp( pcPatternKey, cStruct, 0, 0, 0, 0 );
  }
  
  // start search
  Int  iDist = 0;
  Int  iStartX = cStruct.iBestX;
  Int  iStartY = cStruct.iBestY;
  
  // first search
  for ( iDist = 1; iDist <= (Int)uiSearchRange; iDist*=2 )
  {
    if ( bFirstSearchDiamond == 1 )
    {
      xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
    }
    else
    {
      xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
    }
    
    if ( bFirstSearchStop && ( cStruct.uiBestRound >= uiFirstSearchRounds ) ) // stop criterion
    {
      break;
    }
  }
  
  // test whether zero Mv is a better start point than Median predictor
  if ( bTestZeroVectorStart && ((cStruct.iBestX != 0) || (cStruct.iBestY != 0)) )
  {
    xTZSearchHelp( pcPatternKey, cStruct, 0, 0, 0, 0 );
    if ( (cStruct.iBestX == 0) && (cStruct.iBestY == 0) )
    {
      // test its neighborhood
      for ( iDist = 1; iDist <= (Int)uiSearchRange; iDist*=2 )
      {
        xTZ8PointDiamondSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, 0, 0, iDist );
        if ( bTestZeroVectorStop && (cStruct.uiBestRound > 0) ) // stop criterion
        {
          break;
        }
      }
    }
  }
  
  // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
  if ( cStruct.uiBestDistance == 1 )
  {
    cStruct.uiBestDistance = 0;
    xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
  }
  
  // raster search if distance is too big
  if ( bEnableRasterSearch && ( ((Int)(cStruct.uiBestDistance) > iRaster) || bAlwaysRasterSearch ) )
  {
    cStruct.uiBestDistance = iRaster;
    for ( iStartY = iSrchRngVerTop; iStartY <= iSrchRngVerBottom; iStartY += iRaster )
    {
      for ( iStartX = iSrchRngHorLeft; iStartX <= iSrchRngHorRight; iStartX += iRaster )
      {
        xTZSearchHelp( pcPatternKey, cStruct, iStartX, iStartY, 0, iRaster );
      }
    }
  }
  
  // raster refinement
  if ( bRasterRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      if ( cStruct.uiBestDistance > 1 )
      {
        iDist = cStruct.uiBestDistance >>= 1;
        if ( bRasterRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        else
        {
          xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
      }
      
      // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
        }
      }
    }
  }
  
  // start refinement
  if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < (Int)uiSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        else
        {
          xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }
      
      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
        }
      }
    }
  }
  
  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCost( cStruct.iBestX, cStruct.iBestY 
#if QC_IMV
    , pcPatternKey->getImvFlag()
#endif
    );
}

Void TEncSearch::xPatternSearchFracDIF(TComDataCU* pcCU,
                                       TComPattern* pcPatternKey,
                                       Pel* piRefY,
                                       Int iRefStride,
                                       TComMv* pcMvInt,
                                       TComMv& rcMvHalf,
                                       TComMv& rcMvQter,
                                       UInt& ruiCost
                                       ,Bool biPred
                                       )
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern( piRefY +  iOffset,
                          NULL,
                          NULL,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          0, 0 );
  
  //  Half-pel refinement
  xExtDIFUpSamplingH ( &cPatternRoi, biPred );
  
  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  TComMv baseRefMv(0, 0);
  ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 2, rcMvHalf );
  
  m_pcRdCost->setCostScale( 0 );
#if QC_IMV
  if( pcPatternKey->getImvFlag() )
  {
    rcMvQter = TComMv( 0, 0 );
    return;
  }
#endif
  xExtDIFUpSamplingQ ( &cPatternRoi, rcMvHalf, biPred );
  baseRefMv = rcMvHalf;
  baseRefMv <<= 1;
  
  rcMvQter = *pcMvInt;   rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 1, rcMvQter );
}

/** encode residual and calculate rate-distortion for a CU block
 * \param pcCU
 * \param pcYuvOrg
 * \param pcYuvPred
 * \param rpcYuvResi
 * \param rpcYuvResiBest
 * \param rpcYuvRec
 * \param bSkipRes
 * \returns Void
 */
Void TEncSearch::encodeResAndCalcRdInterCU( TComDataCU* pcCU, TComYuv* pcYuvOrg, TComYuv* pcYuvPred, TComYuv*& rpcYuvResi, TComYuv*& rpcYuvResiBest, TComYuv*& rpcYuvRec, Bool bSkipRes 
#if QC_EMT_INTER_FAST
                                           , Double dBestCost
#endif
                                           )
{
  if ( pcCU->isIntra(0) )
  {
    return;
  }
  
  Bool      bHighPass    = pcCU->getSlice()->getDepth() ? true : false;
  UInt      uiBits       = 0, uiBitsBest = 0;
  UInt      uiDistortion = 0, uiDistortionBest = 0;
  
  UInt      uiWidth      = pcCU->getWidth ( 0 );
  UInt      uiHeight     = pcCU->getHeight( 0 );
  
  //  No residual coding : SKIP mode
  if ( bSkipRes )
  {
    pcCU->setSkipFlagSubParts( true, 0, pcCU->getDepth(0) );

    rpcYuvResi->clear();
    
    pcYuvPred->copyToPartYuv( rpcYuvRec, 0 );
    
    uiDistortion = m_pcRdCost->getDistPart(g_bitDepthY, rpcYuvRec->getLumaAddr(), rpcYuvRec->getStride(),  pcYuvOrg->getLumaAddr(), pcYuvOrg->getStride(),  uiWidth,      uiHeight      )
    + m_pcRdCost->getDistPart(g_bitDepthC, rpcYuvRec->getCbAddr(),   rpcYuvRec->getCStride(), pcYuvOrg->getCbAddr(),   pcYuvOrg->getCStride(), uiWidth >> 1, uiHeight >> 1, TEXT_CHROMA_U )
    + m_pcRdCost->getDistPart(g_bitDepthC, rpcYuvRec->getCrAddr(),   rpcYuvRec->getCStride(), pcYuvOrg->getCrAddr(),   pcYuvOrg->getCStride(), uiWidth >> 1, uiHeight >> 1, TEXT_CHROMA_V );

    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST]);
    
    m_pcEntropyCoder->resetBits();
    if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
    }
    m_pcEntropyCoder->encodeSkipFlag(pcCU, 0, true);
#if QC_FRUC_MERGE
    m_pcEntropyCoder->encodeFRUCMgrMode( pcCU, 0 , 0 );
    if( !pcCU->getFRUCMgrMode( 0 ) )
#endif
    m_pcEntropyCoder->encodeMergeIndex( pcCU, 0, true );
    
    uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    pcCU->getTotalBits()       = uiBits;
    pcCU->getTotalDistortion() = uiDistortion;
    pcCU->getTotalCost()       = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
    
    m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_TEMP_BEST]);
    
    pcCU->setCbfSubParts( 0, 0, 0, 0, pcCU->getDepth( 0 ) );
    pcCU->setTrIdxSubParts( 0, 0, pcCU->getDepth(0) );
    
    return;
  }
  
  //  Residual coding.
  Int    qp, qpBest = 0, qpMin, qpMax;
  Double  dCost, dCostBest = MAX_DOUBLE;
  
  UInt uiTrLevel = 0;
  if( (pcCU->getWidth(0) > pcCU->getSlice()->getSPS()->getMaxTrSize()) )
  {
    while( pcCU->getWidth(0) > (pcCU->getSlice()->getSPS()->getMaxTrSize()<<uiTrLevel) ) uiTrLevel++;
  }
  UInt uiMaxTrMode = 1 + uiTrLevel;
  
  while((uiWidth>>uiMaxTrMode) < (g_uiMaxCUWidth>>g_uiMaxCUDepth)) uiMaxTrMode--;
  
  qpMin =  bHighPass ? Clip3( -pcCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, pcCU->getQP(0) - m_iMaxDeltaQP ) : pcCU->getQP( 0 );
  qpMax =  bHighPass ? Clip3( -pcCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, pcCU->getQP(0) + m_iMaxDeltaQP ) : pcCU->getQP( 0 );

  rpcYuvResi->subtract( pcYuvOrg, pcYuvPred, 0, uiWidth );

#if QC_EMT_INTER
  UChar ucBestCuFlag = 0;
  UChar ucEmtUsage = ( ( uiWidth > QC_EMT_INTER_MAX_CU ) || ( pcCU->getSlice()->getSPS()->getUseInterEMT()==0 ) ) ? 1 : 2;

#if QC_OBMC && QC_EMT_INTER_FAST
  // Disable EMT when CU-level OBMC control flag is 0
  if( m_pcEncCfg->getUseFastInterEMT() && pcCU->getSlice()->getSPS()->getOBMC() && pcCU->getOBMCFlag( 0 ) == 0 )
  {
    ucEmtUsage = 1;
  }
#endif
#if QC_EMT_INTER_FAST
  Bool  bZeroCu = false;
#endif
#endif

  for ( qp = qpMin; qp <= qpMax; qp++ )
  {
#if QC_EMT_INTER
  for (UChar ucCuFlag = 0; ucCuFlag < ucEmtUsage; ucCuFlag++)
  {
#if QC_EMT_INTER_FAST
    if( ucCuFlag && m_pcEncCfg->getUseFastInterEMT() )
    {
      if ( dCost > QC_EMT_INTER_SKIP_CHECK_EMT_THR * dBestCost )
      {
        break;
      }
    }
#endif
#if QC_EMT_INTER_FAST
    if( ucCuFlag && bZeroCu && m_pcEncCfg->getUseFastInterEMT() )
    {
      break;
    }
#endif
    pcCU->setEmtCuFlagSubParts(ucCuFlag, 0, pcCU->getDepth(0));
    pcCU->setSkipFlagSubParts(false, 0, pcCU->getDepth(0));
#endif
    dCost = 0.;
    uiBits = 0;
    uiDistortion = 0;
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_CURR_BEST ] );
    
    UInt uiZeroDistortion = 0;
#if INTER_KLT
  xEstimateResidualQT( pcCU, 0, 0, 0, rpcYuvResi,  pcYuvPred, pcCU->getDepth(0), dCost, uiBits, uiDistortion, &uiZeroDistortion );
#else
    xEstimateResidualQT( pcCU, 0, 0, 0, rpcYuvResi,  pcCU->getDepth(0), dCost, uiBits, uiDistortion, &uiZeroDistortion );
#endif
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeQtRootCbfZero( pcCU );
    UInt zeroResiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    Double dZeroCost = m_pcRdCost->calcRdCost( zeroResiBits, uiZeroDistortion );
    if(pcCU->isLosslessCoded( 0 ))
    {  
      dZeroCost = dCost + 1;
    }
#if QC_SUB_PU_TMVP
    if ( dZeroCost < dCost || pcCU->getQtRootCbf(0)==0)
#else
    if ( dZeroCost < dCost )
#endif
    {
      dCost        = dZeroCost;
      uiBits       = 0;
      uiDistortion = uiZeroDistortion;
      
      const UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> (pcCU->getDepth(0) << 1);
      ::memset( pcCU->getTransformIdx()      , 0, uiQPartNum * sizeof(UChar) );
      ::memset( pcCU->getCbf( TEXT_LUMA )    , 0, uiQPartNum * sizeof(UChar) );
      ::memset( pcCU->getCbf( TEXT_CHROMA_U ), 0, uiQPartNum * sizeof(UChar) );
      ::memset( pcCU->getCbf( TEXT_CHROMA_V ), 0, uiQPartNum * sizeof(UChar) );
      ::memset( pcCU->getCoeffY()            , 0, uiWidth * uiHeight * sizeof( TCoeff )      );
      ::memset( pcCU->getCoeffCb()           , 0, uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
      ::memset( pcCU->getCoeffCr()           , 0, uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
      pcCU->setTransformSkipSubParts ( 0, 0, 0, 0, pcCU->getDepth(0) );
#if KLT_COMMON
    pcCU->setKLTFlagSubParts( 0, 0, 0, 0, pcCU->getDepth(0) );
#endif
    }
    else
    {
      xSetResidualQTData( pcCU, 0, 0, 0, NULL, pcCU->getDepth(0), false );
    }
    
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST] );
    
    uiBits = 0;
    {
      TComYuv *pDummy = NULL;
      xAddSymbolBitsInter( pcCU, 0, 0, uiBits, pDummy, NULL, pDummy );
    }
    
#if QC_EMT_INTER_FAST
    bZeroCu = pcCU->getCbf( 0, TEXT_LUMA ) ? false : true;
#endif
    
    Double dExactCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
    dCost = dExactCost;

    if ( dCost < dCostBest )
    {
      if ( !pcCU->getQtRootCbf( 0 ) )
      {
        rpcYuvResiBest->clear();
      }
      else
      {
        xSetResidualQTData( pcCU, 0, 0, 0, rpcYuvResiBest, pcCU->getDepth(0), true );
      }
      
#if QC_EMT_INTER
      ucBestCuFlag = ucCuFlag;
#endif

#if QC_EMT_INTER
      if( ( qpMin != qpMax && qp != qpMax ) || ucCuFlag < (ucEmtUsage-1) )
#else
      if( qpMin != qpMax && qp != qpMax )
#endif
      {
        const UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> (pcCU->getDepth(0) << 1);
        ::memcpy( m_puhQTTempTrIdx, pcCU->getTransformIdx(),        uiQPartNum * sizeof(UChar) );
        ::memcpy( m_puhQTTempCbf[0], pcCU->getCbf( TEXT_LUMA ),     uiQPartNum * sizeof(UChar) );
        ::memcpy( m_puhQTTempCbf[1], pcCU->getCbf( TEXT_CHROMA_U ), uiQPartNum * sizeof(UChar) );
        ::memcpy( m_puhQTTempCbf[2], pcCU->getCbf( TEXT_CHROMA_V ), uiQPartNum * sizeof(UChar) );
        ::memcpy( m_pcQTTempCoeffY,  pcCU->getCoeffY(),  uiWidth * uiHeight * sizeof( TCoeff )      );
        ::memcpy( m_pcQTTempCoeffCb, pcCU->getCoeffCb(), uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
        ::memcpy( m_pcQTTempCoeffCr, pcCU->getCoeffCr(), uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
#if ADAPTIVE_QP_SELECTION
        ::memcpy( m_pcQTTempArlCoeffY,  pcCU->getArlCoeffY(),  uiWidth * uiHeight * sizeof( Int )      );
        ::memcpy( m_pcQTTempArlCoeffCb, pcCU->getArlCoeffCb(), uiWidth * uiHeight * sizeof( Int ) >> 2 );
        ::memcpy( m_pcQTTempArlCoeffCr, pcCU->getArlCoeffCr(), uiWidth * uiHeight * sizeof( Int ) >> 2 );
#endif
        ::memcpy( m_puhQTTempTransformSkipFlag[0], pcCU->getTransformSkip(TEXT_LUMA),     uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempTransformSkipFlag[1], pcCU->getTransformSkip(TEXT_CHROMA_U), uiQPartNum * sizeof( UChar ) );
        ::memcpy( m_puhQTTempTransformSkipFlag[2], pcCU->getTransformSkip(TEXT_CHROMA_V), uiQPartNum * sizeof( UChar ) );
#if KLT_COMMON 
    ::memcpy( m_puhQTTempKLTFlag[0], pcCU->getKLTFlag(TEXT_LUMA), uiQPartNum * sizeof( UChar ) );
    ::memcpy( m_puhQTTempKLTFlag[1], pcCU->getKLTFlag(TEXT_CHROMA_U), uiQPartNum * sizeof( UChar ) );
    ::memcpy( m_puhQTTempKLTFlag[2], pcCU->getKLTFlag(TEXT_CHROMA_V), uiQPartNum * sizeof( UChar ) );
#endif
      }
      uiBitsBest       = uiBits;
      uiDistortionBest = uiDistortion;
      dCostBest        = dCost;
      qpBest           = qp;
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_TEMP_BEST ] );
    }
#if QC_EMT_INTER
  }
#endif
  }
  
#if QC_EMT_INTER
  pcCU->setEmtCuFlagSubParts( ucBestCuFlag, 0, pcCU->getDepth(0) );
  pcCU->setSkipFlagSubParts( false, 0, pcCU->getDepth(0) );
#endif

  assert ( dCostBest != MAX_DOUBLE );
  
#if QC_EMT_INTER
  if( ( qpMin != qpMax && qpBest != qpMax ) || ucBestCuFlag < (ucEmtUsage-1) )
#else
  if( qpMin != qpMax && qpBest != qpMax )
#endif
  {
#if !QC_EMT_INTER
    assert( 0 ); // check
#endif

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_TEMP_BEST ] );

    // copy best cbf and trIdx to pcCU
    const UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> (pcCU->getDepth(0) << 1);
    ::memcpy( pcCU->getTransformIdx(),       m_puhQTTempTrIdx,  uiQPartNum * sizeof(UChar) );
    ::memcpy( pcCU->getCbf( TEXT_LUMA ),     m_puhQTTempCbf[0], uiQPartNum * sizeof(UChar) );
    ::memcpy( pcCU->getCbf( TEXT_CHROMA_U ), m_puhQTTempCbf[1], uiQPartNum * sizeof(UChar) );
    ::memcpy( pcCU->getCbf( TEXT_CHROMA_V ), m_puhQTTempCbf[2], uiQPartNum * sizeof(UChar) );
    ::memcpy( pcCU->getCoeffY(),  m_pcQTTempCoeffY,  uiWidth * uiHeight * sizeof( TCoeff )      );
    ::memcpy( pcCU->getCoeffCb(), m_pcQTTempCoeffCb, uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
    ::memcpy( pcCU->getCoeffCr(), m_pcQTTempCoeffCr, uiWidth * uiHeight * sizeof( TCoeff ) >> 2 );
#if ADAPTIVE_QP_SELECTION
    ::memcpy( pcCU->getArlCoeffY(),  m_pcQTTempArlCoeffY,  uiWidth * uiHeight * sizeof( Int )      );
    ::memcpy( pcCU->getArlCoeffCb(), m_pcQTTempArlCoeffCb, uiWidth * uiHeight * sizeof( Int ) >> 2 );
    ::memcpy( pcCU->getArlCoeffCr(), m_pcQTTempArlCoeffCr, uiWidth * uiHeight * sizeof( Int ) >> 2 );
#endif
    ::memcpy( pcCU->getTransformSkip(TEXT_LUMA),     m_puhQTTempTransformSkipFlag[0], uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getTransformSkip(TEXT_CHROMA_U), m_puhQTTempTransformSkipFlag[1], uiQPartNum * sizeof( UChar ) );
    ::memcpy( pcCU->getTransformSkip(TEXT_CHROMA_V), m_puhQTTempTransformSkipFlag[2], uiQPartNum * sizeof( UChar ) );
#if KLT_COMMON
  ::memcpy( pcCU->getKLTFlag(TEXT_LUMA), m_puhQTTempKLTFlag[0], uiQPartNum * sizeof( UChar ) );
  ::memcpy( pcCU->getKLTFlag(TEXT_CHROMA_U), m_puhQTTempKLTFlag[1], uiQPartNum * sizeof( UChar ) );
  ::memcpy( pcCU->getKLTFlag(TEXT_CHROMA_V), m_puhQTTempKLTFlag[2], uiQPartNum * sizeof( UChar ) );
#endif
  }
  rpcYuvRec->addClip ( pcYuvPred, rpcYuvResiBest, 0, uiWidth );
  
  // update with clipped distortion and cost (qp estimation loop uses unclipped values)
    uiDistortionBest = m_pcRdCost->getDistPart(g_bitDepthY, rpcYuvRec->getLumaAddr(), rpcYuvRec->getStride(),  pcYuvOrg->getLumaAddr(), pcYuvOrg->getStride(),  uiWidth,      uiHeight      )
    + m_pcRdCost->getDistPart(g_bitDepthC, rpcYuvRec->getCbAddr(),   rpcYuvRec->getCStride(), pcYuvOrg->getCbAddr(),   pcYuvOrg->getCStride(), uiWidth >> 1, uiHeight >> 1, TEXT_CHROMA_U )
    + m_pcRdCost->getDistPart(g_bitDepthC, rpcYuvRec->getCrAddr(),   rpcYuvRec->getCStride(), pcYuvOrg->getCrAddr(),   pcYuvOrg->getCStride(), uiWidth >> 1, uiHeight >> 1, TEXT_CHROMA_V );
  dCostBest = m_pcRdCost->calcRdCost( uiBitsBest, uiDistortionBest );
  
  pcCU->getTotalBits()       = uiBitsBest;
  pcCU->getTotalDistortion() = uiDistortionBest;
  pcCU->getTotalCost()       = dCostBest;
  
  if ( pcCU->isSkipped(0) )
  {
    pcCU->setCbfSubParts( 0, 0, 0, 0, pcCU->getDepth( 0 ) );
  }
  
  pcCU->setQPSubParts( qpBest, 0, pcCU->getDepth(0) );
}

#if INTER_KLT
Void TEncSearch::xEstimateResidualQT( TComDataCU* pcCU, UInt uiQuadrant, UInt uiAbsPartIdx, UInt absTUPartIdx, TComYuv* pcResi, TComYuv* pcPred, const UInt uiDepth, Double &rdCost, UInt &ruiBits, UInt &ruiDist, UInt *puiZeroDist )
#else
Void TEncSearch::xEstimateResidualQT( TComDataCU* pcCU, UInt uiQuadrant, UInt uiAbsPartIdx, UInt absTUPartIdx, TComYuv* pcResi, const UInt uiDepth, Double &rdCost, UInt &ruiBits, UInt &ruiDist, UInt *puiZeroDist )
#endif
{
  const UInt uiTrMode = uiDepth - pcCU->getDepth( 0 );
#if QC_EMT_INTER
  UChar ucBestEmtTrIdx=0;
#endif

  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiLog2TrSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth]+2;
  
  UInt SplitFlag = ((pcCU->getSlice()->getSPS()->getQuadtreeTUMaxDepthInter() == 1) && pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTER && ( pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N ));
  Bool bCheckFull;
  if ( SplitFlag && uiDepth == pcCU->getDepth(uiAbsPartIdx) && ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) ) )
  {
     bCheckFull = false;
  }
  else
  {
     bCheckFull =  ( uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
  }
  
  Bool bCheckSplit  = ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
  
  assert( bCheckFull || bCheckSplit );
  
  Bool  bCodeChroma   = true;
  UInt  uiTrModeC     = uiTrMode;
  UInt  uiLog2TrSizeC = uiLog2TrSize-1;
  if( uiLog2TrSize == 2 )
  {
    uiLog2TrSizeC++;
    uiTrModeC    --;
    UInt  uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrModeC ) << 1 );
    bCodeChroma   = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
  }
  
  const UInt uiSetCbf = 1 << uiTrMode;
  // code full block
  Double dSingleCost = MAX_DOUBLE;
  UInt uiSingleBits = 0;
  UInt uiSingleDist = 0;
  UInt uiAbsSumY = 0, uiAbsSumU = 0, uiAbsSumV = 0;
  UInt uiBestTransformMode[3] = {0};
#if INTER_KLT
  UInt uiBestUseKLTModeOrNot[3] = { 0 };
#endif

  m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
  
  if( bCheckFull )
  {
    const UInt uiNumCoeffPerAbsPartIdxIncrement = pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
    const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    TCoeff *pcCoeffCurrY = m_ppcQTTempCoeffY [uiQTTempAccessLayer] +  uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
    TCoeff *pcCoeffCurrU = m_ppcQTTempCoeffCb[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
    TCoeff *pcCoeffCurrV = m_ppcQTTempCoeffCr[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
#if ADAPTIVE_QP_SELECTION    
    Int *pcArlCoeffCurrY = m_ppcQTTempArlCoeffY [uiQTTempAccessLayer] +  uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
    Int *pcArlCoeffCurrU = m_ppcQTTempArlCoeffCb[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
    Int *pcArlCoeffCurrV = m_ppcQTTempArlCoeffCr[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);   
#endif
    
    Int trWidth = 0, trHeight = 0, trWidthC = 0, trHeightC = 0;
    UInt absTUPartIdxC = uiAbsPartIdx;

    trWidth  = trHeight  = 1 << uiLog2TrSize;
    trWidthC = trHeightC = 1 <<uiLog2TrSizeC;
    pcCU->setTrIdxSubParts( uiDepth - pcCU->getDepth( 0 ), uiAbsPartIdx, uiDepth );
    Double minCostY = MAX_DOUBLE;
    Double minCostU = MAX_DOUBLE;
    Double minCostV = MAX_DOUBLE;
#if INTER_KLT
  Double minCostTransY[2] = { MAX_DOUBLE, MAX_DOUBLE};
#define OriTrans 0
#define KLTrans  1
  UInt uiMaxTrWidth = g_uiDepth2Width[USE_MORE_BLOCKSIZE_DEPTH_MAX - 1];
  UInt uiMinTrWidth = g_uiDepth2Width[USE_MORE_BLOCKSIZE_DEPTH_MIN - 1];
  Bool checkKLTY = ((trWidth == trHeight) && (trWidth <= uiMaxTrWidth) && (trWidth >= uiMinTrWidth));
  checkKLTY &= g_bEnableCheck;
#endif
    Bool checkTransformSkipY  = pcCU->getSlice()->getPPS()->getUseTransformSkip() && trWidth == 4 && trHeight == 4;
    Bool checkTransformSkipUV = pcCU->getSlice()->getPPS()->getUseTransformSkip() && trWidthC == 4 && trHeightC == 4;

    const UInt uiNumSamplesLuma = 1 << (uiLog2TrSize<<1);
    const UInt uiNumSamplesChro = 1 << (uiLog2TrSizeC<<1);

    checkTransformSkipY         &= (!pcCU->isLosslessCoded(0));
    checkTransformSkipUV        &= (!pcCU->isLosslessCoded(0));

    pcCU->setTransformSkipSubParts ( 0, TEXT_LUMA, uiAbsPartIdx, uiDepth ); 
    if( bCodeChroma )
    {
      pcCU->setTransformSkipSubParts ( 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC ); 
      pcCU->setTransformSkipSubParts ( 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC ); 
    }
#if INTER_KLT
  pcCU->setKLTFlagSubParts(0, TEXT_LUMA, uiAbsPartIdx, uiDepth);
  if (bCodeChroma)
  {
    pcCU->setKLTFlagSubParts(0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0) + uiTrModeC);
    pcCU->setKLTFlagSubParts(0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0) + uiTrModeC);
  }
#endif
    if (m_pcEncCfg->getUseRDOQ())
    {
      m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, trWidth, trHeight, TEXT_LUMA );        
    }

    m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_LUMA, pcCU->getSlice()->getSPS()->getQpBDOffsetY(), 0 );

#if RDOQ_CHROMA_LAMBDA 
    m_pcTrQuant->selectLambda(TEXT_LUMA);  
#endif
#if INTER_KLT
  UInt uiZeroDistY = MAX_INT;
  UInt uiZeroBitY = 0;
#endif
#if QC_EMT_INTER
    UInt uiBestAbsSumY=0, uiBestDistY=0, uiZeroDist=0, uiBestZeroDistY=0, uiBestSingleBitsY=0;
    UInt uiDistY=0, uiSingleBitsY=0;
    Double dBestMinCostY = MAX_DOUBLE;
    Pel bestResiY[QC_EMT_INTER_MAX_CU*QC_EMT_INTER_MAX_CU];
    TCoeff bestCoeffY[QC_EMT_INTER_MAX_CU*QC_EMT_INTER_MAX_CU];
#if ADAPTIVE_QP_SELECTION
    TCoeff bestArlCoeffY[QC_EMT_INTER_MAX_CU*QC_EMT_INTER_MAX_CU];
#endif

    ::memset( m_pTempPel, 0, sizeof( Pel ) * uiNumSamplesLuma );
    uiZeroDist = m_pcRdCost->getDistPart(g_bitDepthY, m_pTempPel, trWidth, pcResi->getLumaAddr( absTUPartIdx ), pcResi->getStride(), trWidth, trHeight );
#if INTER_KLT
  uiZeroDistY = uiZeroDist;
#endif
    UChar ucNumEmtTrCands = ( pcCU->getWidth(uiAbsPartIdx) > QC_EMT_INTER_MAX_CU || pcCU->getEmtCuFlag(uiAbsPartIdx)==0 ) ? 1 : 4;

    for( UChar ucTrIdx=0; ucTrIdx < ucNumEmtTrCands; ucTrIdx++ )
    {
      pcCU->setEmtTuIdxSubParts( ucTrIdx, uiAbsPartIdx, uiDepth );
      if( ucTrIdx )
      {
        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
      }
#endif

    m_pcTrQuant->transformNxN( pcCU, pcResi->getLumaAddr( absTUPartIdx ), pcResi->getStride (), pcCoeffCurrY, 
#if ADAPTIVE_QP_SELECTION
                                 pcArlCoeffCurrY, 
#endif      
                                 trWidth,   trHeight,    uiAbsSumY, TEXT_LUMA,     uiAbsPartIdx
#if QC_EMT_INTER
                                 , false
                                 , pcCU->getEmtCuFlag(uiAbsPartIdx) ? pcCU->getEmtTuIdx(uiAbsPartIdx) : ( pcCU->getSlice()->getSPS()->getUseInterEMT() ? DCT2_EMT : DCT2_HEVC )
#endif
                                 );
    
    pcCU->setCbfSubParts( uiAbsSumY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth );

#if !QC_EMT_INTER
    if( bCodeChroma )
    {
      if (m_pcEncCfg->getUseRDOQ())
      {
        m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, trWidthC, trHeightC, TEXT_CHROMA );          
      }

      Int curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
      m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

#if RDOQ_CHROMA_LAMBDA 
      m_pcTrQuant->selectLambda(TEXT_CHROMA_U);
#endif

      m_pcTrQuant->transformNxN( pcCU, pcResi->getCbAddr(absTUPartIdxC), pcResi->getCStride(), pcCoeffCurrU, 
#if ADAPTIVE_QP_SELECTION
                                 pcArlCoeffCurrU, 
#endif        
                                 trWidthC, trHeightC, uiAbsSumU, TEXT_CHROMA_U, uiAbsPartIdx );

      curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
      m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );
      
#if RDOQ_CHROMA_LAMBDA
      m_pcTrQuant->selectLambda(TEXT_CHROMA_V);
#endif

      m_pcTrQuant->transformNxN( pcCU, pcResi->getCrAddr(absTUPartIdxC), pcResi->getCStride(), pcCoeffCurrV,
#if ADAPTIVE_QP_SELECTION
                                 pcArlCoeffCurrV, 
#endif        
                                 trWidthC, trHeightC, uiAbsSumV, TEXT_CHROMA_V, uiAbsPartIdx );

      pcCU->setCbfSubParts( uiAbsSumU ? uiSetCbf : 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
      pcCU->setCbfSubParts( uiAbsSumV ? uiSetCbf : 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
    }
#endif

    m_pcEntropyCoder->resetBits();
    
    m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_LUMA,     uiTrMode );
    
    m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrY, uiAbsPartIdx,  trWidth,  trHeight,    uiDepth, TEXT_LUMA );
#if QC_EMT_INTER
    uiSingleBitsY = m_pcEntropyCoder->getNumberOfWrittenBits();
#else
    const UInt uiSingleBitsY = m_pcEntropyCoder->getNumberOfWrittenBits();
#endif
    
#if !QC_EMT_INTER
    UInt uiSingleBitsU = 0;
    UInt uiSingleBitsV = 0;
    if( bCodeChroma )
    {
      m_pcEntropyCoder->encodeQtCbf   ( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrMode );
      m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrU, uiAbsPartIdx, trWidthC, trHeightC, uiDepth, TEXT_CHROMA_U );
      uiSingleBitsU = m_pcEntropyCoder->getNumberOfWrittenBits() - uiSingleBitsY;
      
      m_pcEntropyCoder->encodeQtCbf   ( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrMode );
      m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrV, uiAbsPartIdx, trWidthC, trHeightC, uiDepth, TEXT_CHROMA_V );
      uiSingleBitsV = m_pcEntropyCoder->getNumberOfWrittenBits() - ( uiSingleBitsY + uiSingleBitsU );
    }
#endif
    
#if !QC_EMT_INTER
    ::memset( m_pTempPel, 0, sizeof( Pel ) * uiNumSamplesLuma ); // not necessary needed for inside of recursion (only at the beginning)
#endif

#if QC_EMT_INTER
    uiDistY = uiZeroDist;
#else
    UInt uiDistY = m_pcRdCost->getDistPart(g_bitDepthY, m_pTempPel, trWidth, pcResi->getLumaAddr( absTUPartIdx ), pcResi->getStride(), trWidth, trHeight ); // initialized with zero residual destortion
#if INTER_KLT
  uiZeroDistY = uiDistY;
#endif
#endif

#if !QC_EMT_INTER
    if ( puiZeroDist )
    {
      *puiZeroDist += uiDistY;
    }
#endif



    if( uiAbsSumY )
    {
      Pel *pcResiCurrY = m_pcQTTempTComYuv[ uiQTTempAccessLayer ].getLumaAddr( absTUPartIdx );

      m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_LUMA, pcCU->getSlice()->getSPS()->getQpBDOffsetY(), 0 );

      Int scalingListType = 3 + g_eTTable[(Int)TEXT_LUMA];
      assert(scalingListType < SCALING_LIST_NUM);
      m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_LUMA
#if QC_EMT_INTER
        , pcCU->getSlice()->getSPS()->getUseInterEMT() ? INTER_MODE : REG_DCT
#else
        , REG_DCT
#endif
        , pcResiCurrY, m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(),  pcCoeffCurrY, trWidth, trHeight, scalingListType
#if QC_EMT_INTER
        , false
        , pcCU->getEmtCuFlag(uiAbsPartIdx) ? pcCU->getEmtTuIdx(uiAbsPartIdx) : ( pcCU->getSlice()->getSPS()->getUseInterEMT() ? DCT2_EMT : DCT2_HEVC )
#endif
#if ROT_TR 
   ,  0
#endif
        );//this is for inter mode only
      
      const UInt uiNonzeroDistY = m_pcRdCost->getDistPart(g_bitDepthY, m_pcQTTempTComYuv[uiQTTempAccessLayer].getLumaAddr( absTUPartIdx ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(),
      pcResi->getLumaAddr( absTUPartIdx ), pcResi->getStride(), trWidth,trHeight );
      if (pcCU->isLosslessCoded(0)) 
      {
        uiDistY = uiNonzeroDistY;
      }
      else
      {
        const Double singleCostY = m_pcRdCost->calcRdCost( uiSingleBitsY, uiNonzeroDistY );
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeQtCbfZero( pcCU, TEXT_LUMA,     uiTrMode );
        const UInt uiNullBitsY   = m_pcEntropyCoder->getNumberOfWrittenBits();
        const Double nullCostY   = m_pcRdCost->calcRdCost( uiNullBitsY, uiDistY );
        if( nullCostY < singleCostY )  
        {    
          uiAbsSumY = 0;
          ::memset( pcCoeffCurrY, 0, sizeof( TCoeff ) * uiNumSamplesLuma );
#if INTER_KLT
#if QC_EMT_INTER
      uiSingleBitsY = uiNullBitsY;
      if( checkTransformSkipY || ucNumEmtTrCands>1 || checkKLTY)
#else
      if( checkTransformSkipY || checkKLTY)
#endif
#else
#if QC_EMT_INTER
          uiSingleBitsY = uiNullBitsY;
          if( checkTransformSkipY || ucNumEmtTrCands>1 )
#else
          if( checkTransformSkipY )
#endif
#endif
          {
            minCostY = nullCostY;
          }
        }
        else
        {
          uiDistY = uiNonzeroDistY;
#if INTER_KLT
#if QC_EMT_INTER
      if( checkTransformSkipY || ucNumEmtTrCands>1 || checkKLTY)
#else
      if( checkTransformSkipY || checkKLTY)
#endif
#else
#if QC_EMT_INTER
          if( checkTransformSkipY || ucNumEmtTrCands>1 )
#else
          if( checkTransformSkipY )
#endif
#endif
          {
            minCostY = singleCostY;
          }
        }      
    }
    }

#if INTER_KLT
#if QC_EMT_INTER
  else if (checkTransformSkipY || ucNumEmtTrCands>1 || checkKLTY)
#else
  else if( checkTransformSkipY || checkKLTY || checkKLTY)
#endif
#else
#if QC_EMT_INTER
    else if( checkTransformSkipY || ucNumEmtTrCands>1 )
#else
    else if( checkTransformSkipY )
#endif
#endif
    {
      m_pcEntropyCoder->resetBits();
      m_pcEntropyCoder->encodeQtCbfZero( pcCU, TEXT_LUMA, uiTrMode );
      const UInt uiNullBitsY = m_pcEntropyCoder->getNumberOfWrittenBits();
      minCostY = m_pcRdCost->calcRdCost( uiNullBitsY, uiDistY );
#if INTER_KLT
    uiZeroBitY = uiNullBitsY;
#endif
#if QC_EMT_INTER
      uiSingleBitsY = uiNullBitsY;
#endif
    }

#if INTER_KLT && !QC_EMT_INTER
  minCostTransY[OriTrans] = minCostY;
#endif

#if QC_EMT_INTER
    if( minCostY < dBestMinCostY || ucTrIdx==0 )
    {
      if( ucTrIdx != (ucNumEmtTrCands-1) )
      {
        Pel *pcResiCurrY = m_pcQTTempTComYuv[ uiQTTempAccessLayer ].getLumaAddr( absTUPartIdx );
        UInt resiYStride = m_pcQTTempTComYuv[ uiQTTempAccessLayer ].getStride();

        memcpy( bestCoeffY, pcCoeffCurrY, sizeof(TCoeff) * uiNumSamplesLuma );
#if ADAPTIVE_QP_SELECTION
        if( m_pcEncCfg->getUseAdaptQpSelect() )
          memcpy( bestArlCoeffY, pcArlCoeffCurrY, sizeof(TCoeff) * uiNumSamplesLuma );
#endif
        for ( Int i = 0; i < trHeight; ++i )
        {
          memcpy( &bestResiY[i*trWidth], pcResiCurrY+i*resiYStride, sizeof(Pel) * trWidth );
        }

        m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_TEMP_BEST ] );
      }

      dBestMinCostY = minCostY;
      uiBestAbsSumY = uiAbsSumY;
      uiBestDistY = uiDistY;
      uiBestZeroDistY = uiZeroDist;
      uiBestSingleBitsY= uiSingleBitsY;
      ucBestEmtTrIdx = ucTrIdx;
    }

#if INTER_KLT
  minCostTransY[OriTrans] = dBestMinCostY;
#endif
    }
#endif

#if QC_EMT_INTER
    pcCU->setEmtTuIdxSubParts( ucBestEmtTrIdx, uiAbsPartIdx, uiDepth );
    m_pcEntropyCoder->resetBits();
    minCostY = dBestMinCostY;
    uiAbsSumY = uiBestAbsSumY;
    uiDistY = uiBestDistY;
    uiSingleBitsY = uiBestSingleBitsY;
    if ( puiZeroDist )
    {
      *puiZeroDist += uiBestZeroDistY;
    }
#endif

    if( !uiAbsSumY )
    {
      Pel *pcPtr =  m_pcQTTempTComYuv[uiQTTempAccessLayer].getLumaAddr( absTUPartIdx );
      const UInt uiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride();
      for( UInt uiY = 0; uiY < trHeight; ++uiY )
      {
        ::memset( pcPtr, 0, sizeof( Pel ) * trWidth );
        pcPtr += uiStride;
      } 
    }
    
#if QC_EMT_INTER
    if( ucBestEmtTrIdx != (ucNumEmtTrCands-1) )
    {
      Pel *pcResiCurrY = m_pcQTTempTComYuv[ uiQTTempAccessLayer ].getLumaAddr( absTUPartIdx );
      UInt resiYStride = m_pcQTTempTComYuv[ uiQTTempAccessLayer ].getStride();

      memcpy( pcCoeffCurrY, bestCoeffY, sizeof(TCoeff) * uiNumSamplesLuma );
#if ADAPTIVE_QP_SELECTION
      if( m_pcEncCfg->getUseAdaptQpSelect() )
        memcpy( pcArlCoeffCurrY, bestArlCoeffY, sizeof(TCoeff) * uiNumSamplesLuma );
#endif
      if( uiAbsSumY )
      {
        for ( Int i = 0; i < trHeight; ++i )
        {
          memcpy( pcResiCurrY+i*resiYStride, &bestResiY[i*trWidth], sizeof(Pel) * trWidth );
        }
      }
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_TEMP_BEST ] );
    }
#endif
#if QC_EMT_INTER
    pcCU->setCbfSubParts( uiAbsSumY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth );
#endif

    UInt uiDistU = 0;
    UInt uiDistV = 0;
#if QC_EMT_INTER
    UInt uiSingleBitsU = 0;
    UInt uiSingleBitsV = 0;
#endif
    if( bCodeChroma )
    {

#if QC_EMT_INTER
      if (m_pcEncCfg->getUseRDOQ())
      {
        m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, trWidthC, trHeightC, TEXT_CHROMA );          
      }

      Int curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
      m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

#if RDOQ_CHROMA_LAMBDA 
      m_pcTrQuant->selectLambda(TEXT_CHROMA_U);
#endif

      m_pcTrQuant->transformNxN( pcCU, pcResi->getCbAddr(absTUPartIdxC), pcResi->getCStride(), pcCoeffCurrU, 
#if ADAPTIVE_QP_SELECTION
                                 pcArlCoeffCurrU, 
#endif        
                                 trWidthC, trHeightC, uiAbsSumU, TEXT_CHROMA_U, uiAbsPartIdx
                                 , false
                                 , pcCU->getSlice()->getSPS()->getUseInterEMT() ? DCT2_EMT : DCT2_HEVC
                                 );

      curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
      m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );
      
#if RDOQ_CHROMA_LAMBDA
      m_pcTrQuant->selectLambda(TEXT_CHROMA_V);
#endif

      m_pcTrQuant->transformNxN( pcCU, pcResi->getCrAddr(absTUPartIdxC), pcResi->getCStride(), pcCoeffCurrV,
#if ADAPTIVE_QP_SELECTION
                                 pcArlCoeffCurrV, 
#endif        
                                 trWidthC, trHeightC, uiAbsSumV, TEXT_CHROMA_V, uiAbsPartIdx
                                 , false
                                 , pcCU->getSlice()->getSPS()->getUseInterEMT() ? DCT2_EMT : DCT2_HEVC
                                 );

      pcCU->setCbfSubParts( uiAbsSumU ? uiSetCbf : 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
      pcCU->setCbfSubParts( uiAbsSumV ? uiSetCbf : 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );

      // Encoding
      m_pcEntropyCoder->resetBits();
      m_pcEntropyCoder->encodeQtCbf   ( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrMode );
      m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrU, uiAbsPartIdx, trWidthC, trHeightC, uiDepth, TEXT_CHROMA_U );
      uiSingleBitsU = m_pcEntropyCoder->getNumberOfWrittenBits();
      
      m_pcEntropyCoder->encodeQtCbf   ( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrMode );
      m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrV, uiAbsPartIdx, trWidthC, trHeightC, uiDepth, TEXT_CHROMA_V );
      uiSingleBitsV = m_pcEntropyCoder->getNumberOfWrittenBits() - uiSingleBitsU;
#endif

      uiDistU = m_pcRdCost->getDistPart(g_bitDepthC, m_pTempPel, trWidthC, pcResi->getCbAddr( absTUPartIdxC ), pcResi->getCStride(), trWidthC, trHeightC
                                        , TEXT_CHROMA_U
                                        ); // initialized with zero residual destortion
      if ( puiZeroDist )
      {
        *puiZeroDist += uiDistU;
      }
      if( uiAbsSumU )
      {
        Pel *pcResiCurrU = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCbAddr( absTUPartIdxC );

#if QC_EMT_INTER
        curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
#else
        Int curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
#endif
        m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

        Int scalingListType = 3 + g_eTTable[(Int)TEXT_CHROMA_U];
        assert(scalingListType < SCALING_LIST_NUM);
        m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_CHROMA, REG_DCT, pcResiCurrU, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(), pcCoeffCurrU, trWidthC, trHeightC, scalingListType  
#if QC_EMT_INTER
          , false
          , pcCU->getSlice()->getSPS()->getUseInterEMT() ? DCT2_EMT : DCT2_HEVC
#endif
#if ROT_TR 
   ,  0
#endif
          );
        
        const UInt uiNonzeroDistU = m_pcRdCost->getDistPart(g_bitDepthC, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCbAddr( absTUPartIdxC), m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(),
                                                            pcResi->getCbAddr( absTUPartIdxC), pcResi->getCStride(), trWidthC, trHeightC
                                                            , TEXT_CHROMA_U
                                                            );

        if(pcCU->isLosslessCoded(0))  
        {
          uiDistU = uiNonzeroDistU;
        }
        else
        {
          const Double dSingleCostU = m_pcRdCost->calcRdCost( uiSingleBitsU, uiNonzeroDistU );
          m_pcEntropyCoder->resetBits();
          m_pcEntropyCoder->encodeQtCbfZero( pcCU, TEXT_CHROMA_U,     uiTrMode );
          const UInt uiNullBitsU    = m_pcEntropyCoder->getNumberOfWrittenBits();
          const Double dNullCostU   = m_pcRdCost->calcRdCost( uiNullBitsU, uiDistU );
          if( dNullCostU < dSingleCostU )
          {
            uiAbsSumU = 0;
            ::memset( pcCoeffCurrU, 0, sizeof( TCoeff ) * uiNumSamplesChro );
            if( checkTransformSkipUV )
            {
              minCostU = dNullCostU;
            }
          }
          else
          {
            uiDistU = uiNonzeroDistU;
            if( checkTransformSkipUV )
            {
              minCostU = dSingleCostU;
            }
          }
        }
      }
      else if( checkTransformSkipUV )
      {
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeQtCbfZero( pcCU, TEXT_CHROMA_U, uiTrMode );
        const UInt uiNullBitsU = m_pcEntropyCoder->getNumberOfWrittenBits();
        minCostU = m_pcRdCost->calcRdCost( uiNullBitsU, uiDistU );
      }
      if( !uiAbsSumU )
      {
        Pel *pcPtr =  m_pcQTTempTComYuv[uiQTTempAccessLayer].getCbAddr( absTUPartIdxC );
          const UInt uiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride();
        for( UInt uiY = 0; uiY < trHeightC; ++uiY )
        {
          ::memset( pcPtr, 0, sizeof(Pel) * trWidthC );
          pcPtr += uiStride;
        }
      }
      
      uiDistV = m_pcRdCost->getDistPart(g_bitDepthC, m_pTempPel, trWidthC, pcResi->getCrAddr( absTUPartIdxC), pcResi->getCStride(), trWidthC, trHeightC
                                        , TEXT_CHROMA_V
                                        ); // initialized with zero residual destortion
      if ( puiZeroDist )
      {
        *puiZeroDist += uiDistV;
      }
      if( uiAbsSumV )
      {
        Pel *pcResiCurrV = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCrAddr( absTUPartIdxC );
#if QC_EMT_INTER
        curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
#else
        Int curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
#endif
        m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

        Int scalingListType = 3 + g_eTTable[(Int)TEXT_CHROMA_V];
        assert(scalingListType < SCALING_LIST_NUM);
        m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_CHROMA,REG_DCT, pcResiCurrV, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(), pcCoeffCurrV, trWidthC, trHeightC, scalingListType 
#if QC_EMT_INTER
          , false
          , pcCU->getSlice()->getSPS()->getUseInterEMT() ? DCT2_EMT : DCT2_HEVC
#endif
#if ROT_TR 
   ,  0
#endif
          );
        
        const UInt uiNonzeroDistV = m_pcRdCost->getDistPart(g_bitDepthC, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCrAddr( absTUPartIdxC ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(),
                                                            pcResi->getCrAddr( absTUPartIdxC ), pcResi->getCStride(), trWidthC, trHeightC
                                                            , TEXT_CHROMA_V
                                                            );
        if (pcCU->isLosslessCoded(0)) 
        {
          uiDistV = uiNonzeroDistV;
        }
        else
        {
          const Double dSingleCostV = m_pcRdCost->calcRdCost( uiSingleBitsV, uiNonzeroDistV );
          m_pcEntropyCoder->resetBits();
          m_pcEntropyCoder->encodeQtCbfZero( pcCU, TEXT_CHROMA_V,     uiTrMode );
          const UInt uiNullBitsV    = m_pcEntropyCoder->getNumberOfWrittenBits();
          const Double dNullCostV   = m_pcRdCost->calcRdCost( uiNullBitsV, uiDistV );
          if( dNullCostV < dSingleCostV )
          {
            uiAbsSumV = 0;
            ::memset( pcCoeffCurrV, 0, sizeof( TCoeff ) * uiNumSamplesChro );
            if( checkTransformSkipUV )
            {
              minCostV = dNullCostV;
            }
          }
          else
          {
            uiDistV = uiNonzeroDistV;
            if( checkTransformSkipUV )
            {
              minCostV = dSingleCostV;
            }
          }
        }
      }
      else if( checkTransformSkipUV )
      {
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeQtCbfZero( pcCU, TEXT_CHROMA_V, uiTrMode );
        const UInt uiNullBitsV = m_pcEntropyCoder->getNumberOfWrittenBits();
        minCostV = m_pcRdCost->calcRdCost( uiNullBitsV, uiDistV );
      }
      if( !uiAbsSumV )
      {
        Pel *pcPtr =  m_pcQTTempTComYuv[uiQTTempAccessLayer].getCrAddr( absTUPartIdxC );
        const UInt uiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride();
        for( UInt uiY = 0; uiY < trHeightC; ++uiY )
        {   
          ::memset( pcPtr, 0, sizeof(Pel) * trWidthC );
          pcPtr += uiStride;
        }
      }
#if QC_EMT_INTER
      pcCU->setCbfSubParts( uiAbsSumU ? uiSetCbf : 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
      pcCU->setCbfSubParts( uiAbsSumV ? uiSetCbf : 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
#endif
    }
#if !QC_EMT_INTER
    pcCU->setCbfSubParts( uiAbsSumY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth );
    if( bCodeChroma )
    {
      pcCU->setCbfSubParts( uiAbsSumU ? uiSetCbf : 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
      pcCU->setCbfSubParts( uiAbsSumV ? uiSetCbf : 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
    }
#endif

#if INTER_KLT  //check SDT mode
  if (checkKLTY && uiAbsSumY)
  {
    UInt uiTarDepth = g_aucConvertToBit[trWidth];
    UInt uiTempSize = g_uiDepth2TempSize[uiTarDepth];
    m_pcTrQuant->getTargetPatch(pcCU, uiAbsPartIdx, uiAbsPartIdx, pcPred, trWidth, uiTempSize);
    m_pcTrQuant->candidateSearch(pcCU, uiAbsPartIdx, trWidth, uiTempSize);
    Bool useKLT = true;
    Bool bKLTAvailable = m_pcTrQuant->candidateTrain(pcCU, uiAbsPartIdx, trWidth, uiTempSize);
    if (bKLTAvailable == true)
    {
      UInt uiNonzeroDistY = 0, uiAbsSumKLTY;
      Double dSingleCostY;

      Pel *pcResiCurrY = m_pcQTTempTComYuv[uiQTTempAccessLayer].getLumaAddr(absTUPartIdx);
      UInt resiYStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride();
#if !QC_EMT_INTER
      TCoeff bestCoeffY[32 * 32];
#endif
      memcpy(bestCoeffY, pcCoeffCurrY, sizeof(TCoeff)* uiNumSamplesLuma);

#if ADAPTIVE_QP_SELECTION
#if !QC_EMT_INTER
      TCoeff bestArlCoeffY[32 * 32];
#endif
      memcpy(bestArlCoeffY, pcArlCoeffCurrY, sizeof(TCoeff)* uiNumSamplesLuma);
#endif

#if !QC_EMT_INTER
      Pel bestResiY[32 * 32];
#endif
      for (Int i = 0; i < trHeight; ++i)
      {
        memcpy(&bestResiY[i*trWidth], pcResiCurrY + i*resiYStride, sizeof(Pel)* trWidth);
      }
      m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_QT_TRAFO_ROOT]);
      if (m_pcEncCfg->getUseRDOQTS())
      {
        m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, trWidth, trHeight, TEXT_LUMA);
      }
      m_pcTrQuant->setQPforQuant(pcCU->getQP(0), TEXT_LUMA, pcCU->getSlice()->getSPS()->getQpBDOffsetY(), 0);
#if QC_EMT_INTER
      pcCU->setEmtTuIdxSubParts(0, uiAbsPartIdx, uiDepth);
#endif
      pcCU->setTransformSkipSubParts(0, TEXT_LUMA, uiAbsPartIdx, uiDepth);
      pcCU->setKLTFlagSubParts(1, TEXT_LUMA, uiAbsPartIdx, uiDepth);
      
#if RDOQ_CHROMA_LAMBDA 
      m_pcTrQuant->selectLambda(TEXT_LUMA);
#endif
#if QC_EMT
      UChar ucTrIdx = DCT2_HEVC;
#endif
#if ROT_TR 
      UChar ucROTIdx = 0;
#endif
      m_pcTrQuant->transformNxN(pcCU, pcResi->getLumaAddr(absTUPartIdx), pcResi->getStride(), pcCoeffCurrY,
#if ADAPTIVE_QP_SELECTION
        pcArlCoeffCurrY,
#endif      
        trWidth, trHeight, uiAbsSumKLTY, TEXT_LUMA, uiAbsPartIdx, false, 
#if QC_EMT
        ucTrIdx,
#endif
#if ROT_TR 
        ucROTIdx,
#endif
        useKLT);
      pcCU->setCbfSubParts(uiAbsSumKLTY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth);

      if (uiAbsSumKLTY != 0)
      {
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeQtCbf(pcCU, uiAbsPartIdx, TEXT_LUMA, uiTrMode);
        m_pcEntropyCoder->encodeCoeffNxN(pcCU, pcCoeffCurrY, uiAbsPartIdx, trWidth, trHeight, uiDepth, TEXT_LUMA);
        const UInt uiTsSingleBitsY = m_pcEntropyCoder->getNumberOfWrittenBits();

        m_pcTrQuant->setQPforQuant(pcCU->getQP(0), TEXT_LUMA, pcCU->getSlice()->getSPS()->getQpBDOffsetY(), 0);

        Int scalingListType = 3 + g_eTTable[(Int)TEXT_LUMA];
        assert(scalingListType < 6);

        const UInt  uiLog2BlockSize = g_aucConvertToBit[trWidth] + 2;
        UInt scanIdx = pcCU->getCoefScanIdx(uiAbsPartIdx, trWidth, TEXT_LUMA == TEXT_LUMA, pcCU->isIntra(uiAbsPartIdx));
        const UInt *scan = g_auiSigLastScan[scanIdx][uiLog2BlockSize - 1];
        //reorder the coefficients to up-right diagonal order
        recoverOrderCoeff(pcCoeffCurrY, scan, trWidth, trHeight);
#if ADAPTIVE_QP_SELECTION
        recoverOrderCoeff(pcArlCoeffCurrY, scan, trWidth, trHeight);
#endif 
        m_pcTrQuant->invtransformNxN(pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_LUMA, REG_DCT, pcResiCurrY, m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(), pcCoeffCurrY, trWidth, trHeight, scalingListType, false, 
#if QC_EMT
          ucTrIdx,
#endif
#if ROT_TR 
          ucROTIdx,
#endif
#if QC_USE_65ANG_MODES
          false,
#endif
          useKLT);

        reOrderCoeff(pcCoeffCurrY, scan, trWidth, trHeight);
#if ADAPTIVE_QP_SELECTION
        reOrderCoeff(pcArlCoeffCurrY, scan, trWidth, trHeight);
#endif 
        uiNonzeroDistY = m_pcRdCost->getDistPart(g_bitDepthY, m_pcQTTempTComYuv[uiQTTempAccessLayer].getLumaAddr(absTUPartIdx), m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(),
          pcResi->getLumaAddr(absTUPartIdx), pcResi->getStride(), trWidth, trHeight);
        dSingleCostY = m_pcRdCost->calcRdCost(uiTsSingleBitsY, uiNonzeroDistY);
      }
      else
      {
        dSingleCostY = m_pcRdCost->calcRdCost(uiZeroBitY, uiZeroDistY);
      }
      minCostTransY[KLTrans] = dSingleCostY;

      if (!uiAbsSumKLTY || minCostTransY[OriTrans] <= dSingleCostY)
      {
        pcCU->setTransformSkipSubParts(0, TEXT_LUMA, uiAbsPartIdx, uiDepth);
        pcCU->setKLTFlagSubParts(0, TEXT_LUMA, uiAbsPartIdx, uiDepth);
#if QC_EMT_INTER
        pcCU->setEmtTuIdxSubParts(ucBestEmtTrIdx, uiAbsPartIdx, uiDepth);
#endif
        memcpy(pcCoeffCurrY, bestCoeffY, sizeof(TCoeff)* uiNumSamplesLuma);
#if ADAPTIVE_QP_SELECTION
        memcpy(pcArlCoeffCurrY, bestArlCoeffY, sizeof(TCoeff)* uiNumSamplesLuma);
#endif
        for (Int i = 0; i < trHeight; ++i)
        {
          memcpy(pcResiCurrY + i*resiYStride, &bestResiY[i*trWidth], sizeof(Pel)* trWidth);
        }
        minCostY = minCostTransY[OriTrans];
      }
      else
      {
        uiDistY = uiNonzeroDistY;
        uiAbsSumY = uiAbsSumKLTY;
        uiBestTransformMode[0] = 0;
        uiBestUseKLTModeOrNot[0] = 1;
#if QC_EMT_INTER
        ucBestEmtTrIdx = 0; //new added
#endif
        if (checkTransformSkipY)
        {
          minCostY = dSingleCostY;
        }
      }
      pcCU->setCbfSubParts(uiAbsSumY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth);
    }
    else
    {
      uiBestUseKLTModeOrNot[0] = 0;
    }
  }
#endif

    if( checkTransformSkipY )
    {
      UInt uiNonzeroDistY, uiAbsSumTransformSkipY;
      Double dSingleCostY;

      Pel *pcResiCurrY = m_pcQTTempTComYuv[ uiQTTempAccessLayer ].getLumaAddr( absTUPartIdx );
      UInt resiYStride = m_pcQTTempTComYuv[ uiQTTempAccessLayer ].getStride();

#if !QC_EMT_INTER
      TCoeff bestCoeffY[32*32];
#endif
      memcpy( bestCoeffY, pcCoeffCurrY, sizeof(TCoeff) * uiNumSamplesLuma );
      
#if ADAPTIVE_QP_SELECTION
#if !QC_EMT_INTER
      TCoeff bestArlCoeffY[32*32];
#endif
      memcpy( bestArlCoeffY, pcArlCoeffCurrY, sizeof(TCoeff) * uiNumSamplesLuma );
#endif

#if !QC_EMT_INTER
      Pel bestResiY[32*32];
#endif
      for ( Int i = 0; i < trHeight; ++i )
      {
        memcpy( &bestResiY[i*trWidth], pcResiCurrY+i*resiYStride, sizeof(Pel) * trWidth );
      }

      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );

      pcCU->setTransformSkipSubParts ( 1, TEXT_LUMA, uiAbsPartIdx, uiDepth );
#if INTER_KLT
    pcCU->setKLTFlagSubParts(0, TEXT_LUMA, uiAbsPartIdx, uiDepth);
#endif
      if (m_pcEncCfg->getUseRDOQTS())
      {
        m_pcEntropyCoder->estimateBit( m_pcTrQuant->m_pcEstBitsSbac, trWidth, trHeight, TEXT_LUMA );        
      }

      m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_LUMA, pcCU->getSlice()->getSPS()->getQpBDOffsetY(), 0 );

#if RDOQ_CHROMA_LAMBDA 
      m_pcTrQuant->selectLambda(TEXT_LUMA);
#endif
      m_pcTrQuant->transformNxN( pcCU, pcResi->getLumaAddr( absTUPartIdx ), pcResi->getStride (), pcCoeffCurrY, 
#if ADAPTIVE_QP_SELECTION
        pcArlCoeffCurrY, 
#endif      
        trWidth,   trHeight,    uiAbsSumTransformSkipY, TEXT_LUMA, uiAbsPartIdx, true );
      pcCU->setCbfSubParts( uiAbsSumTransformSkipY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth );

      if( uiAbsSumTransformSkipY != 0 )
      {
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_LUMA, uiTrMode );
        m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrY, uiAbsPartIdx, trWidth, trHeight, uiDepth, TEXT_LUMA );
        const UInt uiTsSingleBitsY = m_pcEntropyCoder->getNumberOfWrittenBits();

        m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_LUMA, pcCU->getSlice()->getSPS()->getQpBDOffsetY(), 0 );

        Int scalingListType = 3 + g_eTTable[(Int)TEXT_LUMA];
        assert(scalingListType < SCALING_LIST_NUM);

        m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_LUMA,REG_DCT, pcResiCurrY, m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(),  pcCoeffCurrY, trWidth, trHeight, scalingListType, true );

        uiNonzeroDistY = m_pcRdCost->getDistPart(g_bitDepthY, m_pcQTTempTComYuv[uiQTTempAccessLayer].getLumaAddr( absTUPartIdx ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(),
          pcResi->getLumaAddr( absTUPartIdx ), pcResi->getStride(), trWidth, trHeight );

        dSingleCostY = m_pcRdCost->calcRdCost( uiTsSingleBitsY, uiNonzeroDistY );
      }

      if( !uiAbsSumTransformSkipY || minCostY < dSingleCostY )
      {
        pcCU->setTransformSkipSubParts ( 0, TEXT_LUMA, uiAbsPartIdx, uiDepth );
#if INTER_KLT
    pcCU->setKLTFlagSubParts(uiBestUseKLTModeOrNot[0], TEXT_LUMA, uiAbsPartIdx, uiDepth);
#endif
        memcpy( pcCoeffCurrY, bestCoeffY, sizeof(TCoeff) * uiNumSamplesLuma );
#if ADAPTIVE_QP_SELECTION
        memcpy( pcArlCoeffCurrY, bestArlCoeffY, sizeof(TCoeff) * uiNumSamplesLuma );
#endif
        for( Int i = 0; i < trHeight; ++i )
        {
          memcpy( pcResiCurrY+i*resiYStride, &bestResiY[i*trWidth], sizeof(Pel) * trWidth );
        }
      }
      else
      {
        uiDistY = uiNonzeroDistY;
        uiAbsSumY = uiAbsSumTransformSkipY;
        uiBestTransformMode[0] = 1;
#if INTER_KLT
    uiBestUseKLTModeOrNot[0] = 0;
#endif
      }

      pcCU->setCbfSubParts( uiAbsSumY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth );
    }

    if( bCodeChroma && checkTransformSkipUV  )
    {
      UInt uiNonzeroDistU, uiNonzeroDistV, uiAbsSumTransformSkipU, uiAbsSumTransformSkipV;
      Double dSingleCostU, dSingleCostV;

      Pel *pcResiCurrU = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCbAddr( absTUPartIdxC );
      Pel *pcResiCurrV = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCrAddr( absTUPartIdxC );
      UInt resiCStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride();

      TCoeff bestCoeffU[32*32], bestCoeffV[32*32];
      memcpy( bestCoeffU, pcCoeffCurrU, sizeof(TCoeff) * uiNumSamplesChro );
      memcpy( bestCoeffV, pcCoeffCurrV, sizeof(TCoeff) * uiNumSamplesChro );

#if ADAPTIVE_QP_SELECTION
      TCoeff bestArlCoeffU[32*32], bestArlCoeffV[32*32];
      memcpy( bestArlCoeffU, pcArlCoeffCurrU, sizeof(TCoeff) * uiNumSamplesChro );
      memcpy( bestArlCoeffV, pcArlCoeffCurrV, sizeof(TCoeff) * uiNumSamplesChro );
#endif

      Pel bestResiU[32*32], bestResiV[32*32];
      for (Int i = 0; i < trHeightC; ++i )
      {
        memcpy( &bestResiU[i*trWidthC], pcResiCurrU+i*resiCStride, sizeof(Pel) * trWidthC );
        memcpy( &bestResiV[i*trWidthC], pcResiCurrV+i*resiCStride, sizeof(Pel) * trWidthC );
      }

      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );

      pcCU->setTransformSkipSubParts ( 1, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC ); 
      pcCU->setTransformSkipSubParts ( 1, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );

      if (m_pcEncCfg->getUseRDOQTS())
      {
        m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, trWidthC, trHeightC, TEXT_CHROMA );          
      }

      Int curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
      m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

#if RDOQ_CHROMA_LAMBDA 
      m_pcTrQuant->selectLambda(TEXT_CHROMA_U);
#endif

      m_pcTrQuant->transformNxN( pcCU, pcResi->getCbAddr(absTUPartIdxC), pcResi->getCStride(), pcCoeffCurrU, 
#if ADAPTIVE_QP_SELECTION
        pcArlCoeffCurrU, 
#endif        
        trWidthC, trHeightC, uiAbsSumTransformSkipU, TEXT_CHROMA_U, uiAbsPartIdx, true );
      curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
      m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );
#if RDOQ_CHROMA_LAMBDA
      m_pcTrQuant->selectLambda(TEXT_CHROMA_V);
#endif
      m_pcTrQuant->transformNxN( pcCU, pcResi->getCrAddr(absTUPartIdxC), pcResi->getCStride(), pcCoeffCurrV,
#if ADAPTIVE_QP_SELECTION
        pcArlCoeffCurrV, 
#endif        
        trWidthC, trHeightC, uiAbsSumTransformSkipV, TEXT_CHROMA_V, uiAbsPartIdx, true );

      pcCU->setCbfSubParts( uiAbsSumTransformSkipU ? uiSetCbf : 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
      pcCU->setCbfSubParts( uiAbsSumTransformSkipV ? uiSetCbf : 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );

      m_pcEntropyCoder->resetBits();
      uiSingleBitsU = 0;
      uiSingleBitsV = 0;

      if( uiAbsSumTransformSkipU )
      {
        m_pcEntropyCoder->encodeQtCbf   ( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrMode );
        m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrU, uiAbsPartIdx, trWidthC, trHeightC, uiDepth, TEXT_CHROMA_U );
        uiSingleBitsU = m_pcEntropyCoder->getNumberOfWrittenBits();    

        curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCbQpOffset() + pcCU->getSlice()->getSliceQpDeltaCb();
        m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

        Int scalingListType = 3 + g_eTTable[(Int)TEXT_CHROMA_U];
        assert(scalingListType < SCALING_LIST_NUM);

        m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_CHROMA,REG_DCT, pcResiCurrU, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(), pcCoeffCurrU, trWidthC, trHeightC, scalingListType, true  );

        uiNonzeroDistU = m_pcRdCost->getDistPart(g_bitDepthC, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCbAddr( absTUPartIdxC), m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(),
                                                 pcResi->getCbAddr( absTUPartIdxC), pcResi->getCStride(), trWidthC, trHeightC
                                                 , TEXT_CHROMA_U
                                                 );

        dSingleCostU = m_pcRdCost->calcRdCost( uiSingleBitsU, uiNonzeroDistU );
      }

      if( !uiAbsSumTransformSkipU || minCostU < dSingleCostU )
      {
        pcCU->setTransformSkipSubParts ( 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC ); 

        memcpy( pcCoeffCurrU, bestCoeffU, sizeof (TCoeff) * uiNumSamplesChro );
#if ADAPTIVE_QP_SELECTION
        memcpy( pcArlCoeffCurrU, bestArlCoeffU, sizeof (TCoeff) * uiNumSamplesChro );
#endif
        for( Int i = 0; i < trHeightC; ++i )
        {
          memcpy( pcResiCurrU+i*resiCStride, &bestResiU[i*trWidthC], sizeof(Pel) * trWidthC );
        }
      }
      else
      {
        uiDistU = uiNonzeroDistU;
        uiAbsSumU = uiAbsSumTransformSkipU;
        uiBestTransformMode[1] = 1;
      }

      if( uiAbsSumTransformSkipV )
      {
        m_pcEntropyCoder->encodeQtCbf   ( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrMode );
        m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrV, uiAbsPartIdx, trWidthC, trHeightC, uiDepth, TEXT_CHROMA_V );
        uiSingleBitsV = m_pcEntropyCoder->getNumberOfWrittenBits() - uiSingleBitsU;

        curChromaQpOffset = pcCU->getSlice()->getPPS()->getChromaCrQpOffset() + pcCU->getSlice()->getSliceQpDeltaCr();
        m_pcTrQuant->setQPforQuant( pcCU->getQP( 0 ), TEXT_CHROMA, pcCU->getSlice()->getSPS()->getQpBDOffsetC(), curChromaQpOffset );

        Int scalingListType = 3 + g_eTTable[(Int)TEXT_CHROMA_V];
        assert(scalingListType < SCALING_LIST_NUM);

        m_pcTrQuant->invtransformNxN( pcCU->getCUTransquantBypass(uiAbsPartIdx), TEXT_CHROMA,REG_DCT, pcResiCurrV, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(), pcCoeffCurrV, trWidthC, trHeightC, scalingListType, true );

        uiNonzeroDistV = m_pcRdCost->getDistPart(g_bitDepthC, m_pcQTTempTComYuv[uiQTTempAccessLayer].getCrAddr( absTUPartIdxC ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride(),
                                                 pcResi->getCrAddr( absTUPartIdxC ), pcResi->getCStride(), trWidthC, trHeightC
                                                 , TEXT_CHROMA_V
                                                 );

        dSingleCostV = m_pcRdCost->calcRdCost( uiSingleBitsV, uiNonzeroDistV );
      }

      if( !uiAbsSumTransformSkipV || minCostV < dSingleCostV )
      {
        pcCU->setTransformSkipSubParts ( 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC ); 

        memcpy( pcCoeffCurrV, bestCoeffV, sizeof(TCoeff) * uiNumSamplesChro );
#if ADAPTIVE_QP_SELECTION
        memcpy( pcArlCoeffCurrV, bestArlCoeffV, sizeof(TCoeff) * uiNumSamplesChro );
#endif
        for( Int i = 0; i < trHeightC; ++i )
        {
          memcpy( pcResiCurrV+i*resiCStride, &bestResiV[i*trWidthC], sizeof(Pel) * trWidthC );
        }
      }
      else
      {
        uiDistV = uiNonzeroDistV;
        uiAbsSumV = uiAbsSumTransformSkipV;
        uiBestTransformMode[2] = 1;
      }

      pcCU->setCbfSubParts( uiAbsSumU ? uiSetCbf : 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
      pcCU->setCbfSubParts( uiAbsSumV ? uiSetCbf : 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
    }

#if INTER_KLT 
  // The reconstruction will be updated for each TU in inter mode coding
  // Save reconstruction to (1) facilitate the following utilization in next blocks; (2) backup the possible best reconstruction.
  Pel *pcPtrPred = pcPred->getLumaAddr(absTUPartIdx);
  UInt uiStridePred = pcPred->getStride();
  Pel *pcPtrRes = m_pcQTTempTComYuv[uiQTTempAccessLayer].getLumaAddr(absTUPartIdx);
  UInt uiStrideRes = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride();
  Pel *pcPtrRec = m_pcQTTempTComYuvRec[uiQTTempAccessLayer].getLumaAddr(absTUPartIdx);
  UInt uiStrideRec = m_pcQTTempTComYuvRec[uiQTTempAccessLayer].getStride();

  UInt    uiZOrder = pcCU->getZorderIdxInCU() + absTUPartIdx;
  Pel*    piRecIPred = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), uiZOrder);
  UInt    uiRecIPredStride = pcCU->getPic()->getPicYuvRec()->getStride();

  for (UInt uiY = 0; uiY < trHeight; ++uiY)
  {
    for (UInt uiX = 0; uiX < trWidth; ++uiX)
    {
      //save the temporary reconstruction to temporary buffer.
      pcPtrRec[uiX] = ClipY(pcPtrPred[uiX] + pcPtrRes[uiX]);
      //save the temporary reconstruction to reconstruction picture.
      piRecIPred[uiX] = pcPtrRec[uiX];
    }
    pcPtrPred += uiStridePred;
    pcPtrRes += uiStrideRes;
    pcPtrRec += uiStrideRec;
    piRecIPred += uiRecIPredStride;
  }
  if (bCodeChroma)
  {
    uiStridePred = pcPred->getCStride();
    uiStrideRes = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCStride();
    uiStrideRec = m_pcQTTempTComYuvRec[uiQTTempAccessLayer].getCStride();
    uiRecIPredStride = pcCU->getPic()->getPicYuvRec()->getCStride();
    for (UInt uiCbCr = 0; uiCbCr < 2; uiCbCr++)
    {
      if (uiCbCr == 0)
      {
        pcPtrPred = pcPred->getCbAddr(absTUPartIdxC);
        pcPtrRes = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCbAddr(absTUPartIdxC);
        pcPtrRec = m_pcQTTempTComYuvRec[uiQTTempAccessLayer].getCbAddr(absTUPartIdxC);
        piRecIPred = pcCU->getPic()->getPicYuvRec()->getCbAddr(pcCU->getAddr(), uiZOrder);
      }
      else
      {
        pcPtrPred = pcPred->getCrAddr(absTUPartIdxC);
        pcPtrRes = m_pcQTTempTComYuv[uiQTTempAccessLayer].getCrAddr(absTUPartIdxC);
        pcPtrRec = m_pcQTTempTComYuvRec[uiQTTempAccessLayer].getCrAddr(absTUPartIdxC);
        piRecIPred = pcCU->getPic()->getPicYuvRec()->getCrAddr(pcCU->getAddr(), uiZOrder);
      }

      for (UInt uiY = 0; uiY < trHeightC; ++uiY)
      {
        for (UInt uiX = 0; uiX < trWidthC; ++uiX)
        {
          pcPtrRec[uiX] = ClipY(pcPtrPred[uiX] + pcPtrRes[uiX]);
          piRecIPred[uiX] = pcPtrRec[uiX];
        }
        pcPtrPred += uiStridePred;
        pcPtrRes += uiStrideRes;
        pcPtrRec += uiStrideRec;
        piRecIPred += uiRecIPredStride;
      }
    }
  }
#endif

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    m_pcEntropyCoder->resetBits();

    if( uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
    {
#if QC_T64
      m_pcEntropyCoder->encodeTransformSubdivFlag( 0, pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize );
#else
      m_pcEntropyCoder->encodeTransformSubdivFlag( 0, 5 - uiLog2TrSize );
#endif
    }

    if( bCodeChroma )
    {
      m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrMode );
      m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrMode );
    }

    m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_LUMA,     uiTrMode );

    m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrY, uiAbsPartIdx, trWidth, trHeight,    uiDepth, TEXT_LUMA );

    if( bCodeChroma )
    {
      m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrU, uiAbsPartIdx, trWidthC, trHeightC, uiDepth, TEXT_CHROMA_U );
      m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrV, uiAbsPartIdx, trWidthC, trHeightC, uiDepth, TEXT_CHROMA_V );
    }

    uiSingleBits = m_pcEntropyCoder->getNumberOfWrittenBits();

    uiSingleDist = uiDistY + uiDistU + uiDistV;
    dSingleCost = m_pcRdCost->calcRdCost( uiSingleBits, uiSingleDist );
  }  
  
  // code sub-blocks
  if( bCheckSplit )
  {
    if( bCheckFull )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_TEST ] );
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    UInt uiSubdivDist = 0;
    UInt uiSubdivBits = 0;
    Double dSubdivCost = 0.0;
    
    const UInt uiQPartNumSubdiv = pcCU->getPic()->getNumPartInCU() >> ((uiDepth + 1 ) << 1);
    for( UInt ui = 0; ui < 4; ++ui )
    {
      UInt nsAddr = uiAbsPartIdx + ui * uiQPartNumSubdiv;
#if INTER_KLT
    xEstimateResidualQT(pcCU, ui, uiAbsPartIdx + ui * uiQPartNumSubdiv, nsAddr, pcResi, pcPred, uiDepth + 1, dSubdivCost, uiSubdivBits, uiSubdivDist, bCheckFull ? NULL : puiZeroDist);
#else
      xEstimateResidualQT( pcCU, ui, uiAbsPartIdx + ui * uiQPartNumSubdiv, nsAddr, pcResi, uiDepth + 1, dSubdivCost, uiSubdivBits, uiSubdivDist, bCheckFull ? NULL : puiZeroDist );
#endif
#if QC_EMT
      if( dSubdivCost >= dSingleCost )
      {
        break;
      }
#endif
    }
    
#if QC_EMT
    if( dSubdivCost < dSingleCost )
    {
#endif
    UInt uiYCbf = 0;
    UInt uiUCbf = 0;
    UInt uiVCbf = 0;
    for( UInt ui = 0; ui < 4; ++ui )
    {
      uiYCbf |= pcCU->getCbf( uiAbsPartIdx + ui * uiQPartNumSubdiv, TEXT_LUMA,     uiTrMode + 1 );
      uiUCbf |= pcCU->getCbf( uiAbsPartIdx + ui * uiQPartNumSubdiv, TEXT_CHROMA_U, uiTrMode + 1 );
      uiVCbf |= pcCU->getCbf( uiAbsPartIdx + ui * uiQPartNumSubdiv, TEXT_CHROMA_V, uiTrMode + 1 );
    }
    for( UInt ui = 0; ui < 4 * uiQPartNumSubdiv; ++ui )
    {
      pcCU->getCbf( TEXT_LUMA     )[uiAbsPartIdx + ui] |= uiYCbf << uiTrMode;
      pcCU->getCbf( TEXT_CHROMA_U )[uiAbsPartIdx + ui] |= uiUCbf << uiTrMode;
      pcCU->getCbf( TEXT_CHROMA_V )[uiAbsPartIdx + ui] |= uiVCbf << uiTrMode;
    }
    
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    m_pcEntropyCoder->resetBits();
    
    xEncodeResidualQT( pcCU, uiAbsPartIdx, uiDepth, true,  TEXT_LUMA );
    xEncodeResidualQT( pcCU, uiAbsPartIdx, uiDepth, false, TEXT_LUMA );
    xEncodeResidualQT( pcCU, uiAbsPartIdx, uiDepth, false, TEXT_CHROMA_U );
    xEncodeResidualQT( pcCU, uiAbsPartIdx, uiDepth, false, TEXT_CHROMA_V );
    
    uiSubdivBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    dSubdivCost  = m_pcRdCost->calcRdCost( uiSubdivBits, uiSubdivDist );
    
    if( uiYCbf || uiUCbf || uiVCbf || !bCheckFull )
    {
      if( dSubdivCost < dSingleCost )
      {
        rdCost += dSubdivCost;
        ruiBits += uiSubdivBits;
        ruiDist += uiSubdivDist;
        return;
      }
    }
#if QC_EMT
    }
#endif
    pcCU->setTransformSkipSubParts ( uiBestTransformMode[0], TEXT_LUMA, uiAbsPartIdx, uiDepth ); 
    if(bCodeChroma)
    {
      pcCU->setTransformSkipSubParts ( uiBestTransformMode[1], TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC ); 
      pcCU->setTransformSkipSubParts ( uiBestTransformMode[2], TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC ); 
    }
#if INTER_KLT
  pcCU->setKLTFlagSubParts(uiBestUseKLTModeOrNot[0], TEXT_LUMA, uiAbsPartIdx, uiDepth);
  if (bCodeChroma)
  {
    pcCU->setKLTFlagSubParts(uiBestUseKLTModeOrNot[1], TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0) + uiTrModeC);
    pcCU->setKLTFlagSubParts(uiBestUseKLTModeOrNot[2], TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0) + uiTrModeC);
  }
#endif
    assert( bCheckFull );

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_TEST ] );

#if INTER_KLT
  //Set reconstruction for the possible utility of template of next blocks
  UInt trWidth = 1 << uiLog2TrSize;
  UInt trHeight = 1 << uiLog2TrSize;
  const UInt uiQTLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
  const UInt uiZOrder = pcCU->getZorderIdxInCU() + absTUPartIdx;

  Pel*  piSrc = m_pcQTTempTComYuvRec[uiQTLayer].getLumaAddr(absTUPartIdx);
  UInt  uiSrcStride = m_pcQTTempTComYuvRec[uiQTLayer].getStride();
  Pel*  piDes = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), uiZOrder);
  UInt  uiDesStride = pcCU->getPic()->getPicYuvRec()->getStride();
  for (UInt uiY = 0; uiY < trHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride)
  {
    for (UInt uiX = 0; uiX < trWidth; uiX++)
    {
      piDes[uiX] = piSrc[uiX];
    }
  }
  if (bCodeChroma)
  {
    UInt trWidthC = 1 << uiLog2TrSizeC;
    UInt trHeightC = 1 << uiLog2TrSizeC;
    piSrc = m_pcQTTempTComYuvRec[uiQTLayer].getCbAddr(absTUPartIdx);
    uiSrcStride = m_pcQTTempTComYuvRec[uiQTLayer].getCStride();
    piDes = pcCU->getPic()->getPicYuvRec()->getCbAddr(pcCU->getAddr(), uiZOrder);
    uiDesStride = pcCU->getPic()->getPicYuvRec()->getCStride();
    for (UInt uiY = 0; uiY < trHeightC; uiY++, piSrc += uiSrcStride, piDes += uiDesStride)
    {
      for (UInt uiX = 0; uiX < trWidthC; uiX++)
      {
        piDes[uiX] = piSrc[uiX];
      }
    }
    piSrc = m_pcQTTempTComYuvRec[uiQTLayer].getCrAddr(absTUPartIdx);
    piDes = pcCU->getPic()->getPicYuvRec()->getCrAddr(pcCU->getAddr(), uiZOrder);
    for (UInt uiY = 0; uiY < trHeightC; uiY++, piSrc += uiSrcStride, piDes += uiDesStride)
    {
      for (UInt uiX = 0; uiX < trWidthC; uiX++)
      {
        piDes[uiX] = piSrc[uiX];
      }
    }
  }
#endif
  }
  rdCost += dSingleCost;
  ruiBits += uiSingleBits;
  ruiDist += uiSingleDist;
  
  pcCU->setTrIdxSubParts( uiTrMode, uiAbsPartIdx, uiDepth );
  
#if QC_EMT_INTER
  pcCU->setEmtTuIdxSubParts( ucBestEmtTrIdx, uiAbsPartIdx, uiDepth );
#endif

  pcCU->setCbfSubParts( uiAbsSumY ? uiSetCbf : 0, TEXT_LUMA, uiAbsPartIdx, uiDepth );
  if( bCodeChroma )
  {
    pcCU->setCbfSubParts( uiAbsSumU ? uiSetCbf : 0, TEXT_CHROMA_U, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
    pcCU->setCbfSubParts( uiAbsSumV ? uiSetCbf : 0, TEXT_CHROMA_V, uiAbsPartIdx, pcCU->getDepth(0)+uiTrModeC );
  }
}

Void TEncSearch::xEncodeResidualQT( TComDataCU* pcCU, UInt uiAbsPartIdx, const UInt uiDepth, Bool bSubdivAndCbf, TextType eType )
{
  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiCurrTrMode = uiDepth - pcCU->getDepth( 0 );
  const UInt uiTrMode = pcCU->getTransformIdx( uiAbsPartIdx );
  
  const Bool bSubdiv = uiCurrTrMode != uiTrMode;
  
  const UInt uiLog2TrSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth]+2;

  if( bSubdivAndCbf && uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() && uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
  {
#if QC_T64
    m_pcEntropyCoder->encodeTransformSubdivFlag( bSubdiv, pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize );
#else
    m_pcEntropyCoder->encodeTransformSubdivFlag( bSubdiv, 5 - uiLog2TrSize );
#endif
  }

  assert( pcCU->getPredictionMode(uiAbsPartIdx) != MODE_INTRA );
  if( bSubdivAndCbf )
  {
    const Bool bFirstCbfOfCU = uiCurrTrMode == 0;
    if( bFirstCbfOfCU || uiLog2TrSize > 2 )
    {
      if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiCurrTrMode - 1 ) )
      {
        m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiCurrTrMode );
      }
      if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiCurrTrMode - 1 ) )
      {
        m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiCurrTrMode );
      }
    }
    else if( uiLog2TrSize == 2 )
    {
      assert( pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiCurrTrMode ) == pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiCurrTrMode - 1 ) );
      assert( pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiCurrTrMode ) == pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiCurrTrMode - 1 ) );
    }
  }
  
  if( !bSubdiv )
  {
    const UInt uiNumCoeffPerAbsPartIdxIncrement = pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
    //assert( 16 == uiNumCoeffPerAbsPartIdxIncrement ); // check
    const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    TCoeff *pcCoeffCurrY = m_ppcQTTempCoeffY [uiQTTempAccessLayer] +  uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
    TCoeff *pcCoeffCurrU = m_ppcQTTempCoeffCb[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
    TCoeff *pcCoeffCurrV = m_ppcQTTempCoeffCr[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
    
    Bool  bCodeChroma   = true;
    UInt  uiTrModeC     = uiTrMode;
    UInt  uiLog2TrSizeC = uiLog2TrSize-1;
    if( uiLog2TrSize == 2 )
    {
      uiLog2TrSizeC++;
      uiTrModeC    --;
      UInt  uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrModeC ) << 1 );
      bCodeChroma   = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
    }
    
    if( bSubdivAndCbf )
    {
      m_pcEntropyCoder->encodeQtCbf( pcCU, uiAbsPartIdx, TEXT_LUMA,     uiTrMode );
    }
    else
    {
      if( eType == TEXT_LUMA     && pcCU->getCbf( uiAbsPartIdx, TEXT_LUMA,     uiTrMode ) )
      {
        Int trWidth  = 1 << uiLog2TrSize;
        Int trHeight = 1 << uiLog2TrSize;
        m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrY, uiAbsPartIdx, trWidth, trHeight,    uiDepth, TEXT_LUMA );
      }
      if( bCodeChroma )
      {
        Int trWidth  = 1 << uiLog2TrSizeC;
        Int trHeight = 1 << uiLog2TrSizeC;
        if( eType == TEXT_CHROMA_U && pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiTrMode ) )
        {
          m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrU, uiAbsPartIdx, trWidth, trHeight, uiDepth, TEXT_CHROMA_U );
        }
        if( eType == TEXT_CHROMA_V && pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiTrMode ) )
        {
          m_pcEntropyCoder->encodeCoeffNxN( pcCU, pcCoeffCurrV, uiAbsPartIdx, trWidth, trHeight, uiDepth, TEXT_CHROMA_V );
        }
      }
    }
  }
  else
  {
    if( bSubdivAndCbf || pcCU->getCbf( uiAbsPartIdx, eType, uiCurrTrMode ) )
    {
      const UInt uiQPartNumSubdiv = pcCU->getPic()->getNumPartInCU() >> ((uiDepth + 1 ) << 1);
      for( UInt ui = 0; ui < 4; ++ui )
      {
        xEncodeResidualQT( pcCU, uiAbsPartIdx + ui * uiQPartNumSubdiv, uiDepth + 1, bSubdivAndCbf, eType );
      }
    }
  }
}

Void TEncSearch::xSetResidualQTData( TComDataCU* pcCU, UInt uiQuadrant, UInt uiAbsPartIdx, UInt absTUPartIdx, TComYuv* pcResi, UInt uiDepth, Bool bSpatial )
{
  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiCurrTrMode = uiDepth - pcCU->getDepth( 0 );
  const UInt uiTrMode = pcCU->getTransformIdx( uiAbsPartIdx );

  if( uiCurrTrMode == uiTrMode )
  {
    const UInt uiLog2TrSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth]+2;
    const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    Bool  bCodeChroma   = true;
    UInt  uiTrModeC     = uiTrMode;
    UInt  uiLog2TrSizeC = uiLog2TrSize-1;
    if( uiLog2TrSize == 2 )
    {
      uiLog2TrSizeC++;
      uiTrModeC    --;
      UInt  uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( pcCU->getDepth( 0 ) + uiTrModeC ) << 1 );
      bCodeChroma   = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
    }

    if( bSpatial )
    {      
      Int trWidth  = 1 << uiLog2TrSize;
      Int trHeight = 1 << uiLog2TrSize;
      m_pcQTTempTComYuv[uiQTTempAccessLayer].copyPartToPartLuma    ( pcResi, absTUPartIdx, trWidth , trHeight );

      if( bCodeChroma )
      {
        m_pcQTTempTComYuv[uiQTTempAccessLayer].copyPartToPartChroma( pcResi, uiAbsPartIdx, 1 << uiLog2TrSizeC, 1 << uiLog2TrSizeC );
      }
    }
    else
    {
      UInt    uiNumCoeffPerAbsPartIdxIncrement = pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() >> ( pcCU->getSlice()->getSPS()->getMaxCUDepth() << 1 );
      UInt    uiNumCoeffY = ( 1 << ( uiLog2TrSize << 1 ) );
      TCoeff* pcCoeffSrcY = m_ppcQTTempCoeffY [uiQTTempAccessLayer] +  uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
      TCoeff* pcCoeffDstY = pcCU->getCoeffY() + uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
      ::memcpy( pcCoeffDstY, pcCoeffSrcY, sizeof( TCoeff ) * uiNumCoeffY );
#if ADAPTIVE_QP_SELECTION
      Int* pcArlCoeffSrcY = m_ppcQTTempArlCoeffY [uiQTTempAccessLayer] +  uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
      Int* pcArlCoeffDstY = pcCU->getArlCoeffY() + uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx;
      ::memcpy( pcArlCoeffDstY, pcArlCoeffSrcY, sizeof( Int ) * uiNumCoeffY );
#endif
      if( bCodeChroma )
      {
        UInt    uiNumCoeffC = ( 1 << ( uiLog2TrSizeC << 1 ) );
        TCoeff* pcCoeffSrcU = m_ppcQTTempCoeffCb[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        TCoeff* pcCoeffSrcV = m_ppcQTTempCoeffCr[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        TCoeff* pcCoeffDstU = pcCU->getCoeffCb() + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        TCoeff* pcCoeffDstV = pcCU->getCoeffCr() + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        ::memcpy( pcCoeffDstU, pcCoeffSrcU, sizeof( TCoeff ) * uiNumCoeffC );
        ::memcpy( pcCoeffDstV, pcCoeffSrcV, sizeof( TCoeff ) * uiNumCoeffC );
#if ADAPTIVE_QP_SELECTION
        Int* pcArlCoeffSrcU = m_ppcQTTempArlCoeffCb[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        Int* pcArlCoeffSrcV = m_ppcQTTempArlCoeffCr[uiQTTempAccessLayer] + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        Int* pcArlCoeffDstU = pcCU->getArlCoeffCb() + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        Int* pcArlCoeffDstV = pcCU->getArlCoeffCr() + (uiNumCoeffPerAbsPartIdxIncrement * uiAbsPartIdx>>2);
        ::memcpy( pcArlCoeffDstU, pcArlCoeffSrcU, sizeof( Int ) * uiNumCoeffC );
        ::memcpy( pcArlCoeffDstV, pcArlCoeffSrcV, sizeof( Int ) * uiNumCoeffC );
#endif
      }
    }
  }
  else
  {
    const UInt uiQPartNumSubdiv = pcCU->getPic()->getNumPartInCU() >> ((uiDepth + 1 ) << 1);
    for( UInt ui = 0; ui < 4; ++ui )
    {
      UInt nsAddr = uiAbsPartIdx + ui * uiQPartNumSubdiv;
      xSetResidualQTData( pcCU, ui, uiAbsPartIdx + ui * uiQPartNumSubdiv, nsAddr, pcResi, uiDepth + 1, bSpatial );
    }
  }
}

UInt TEncSearch::xModeBitsIntra( TComDataCU* pcCU, UInt uiMode, UInt uiPU, UInt uiPartOffset, UInt uiDepth, UInt uiInitTrDepth 
#if QC_USE_65ANG_MODES
                                , Int* piModes, Int  iCase
#endif
                                )
{
  // Reload only contexts required for coding intra mode information
  m_pcRDGoOnSbacCoder->loadIntraDirModeLuma( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );
  
  pcCU->setLumaIntraDirSubParts ( uiMode, uiPartOffset, uiDepth + uiInitTrDepth );
  
  m_pcEntropyCoder->resetBits();
  m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiPartOffset
#if QC_USE_65ANG_MODES
    , false, piModes, iCase
#endif
    );
  
  return m_pcEntropyCoder->getNumberOfWrittenBits();
}

UInt TEncSearch::xUpdateCandList( UInt uiMode, Double uiCost, UInt uiFastCandNum, UInt * CandModeList, Double * CandCostList )
{
  UInt i;
  UInt shift=0;
  
  while ( shift<uiFastCandNum && uiCost<CandCostList[ uiFastCandNum-1-shift ] ) shift++;
  
  if( shift!=0 )
  {
    for(i=1; i<shift; i++)
    {
      CandModeList[ uiFastCandNum-i ] = CandModeList[ uiFastCandNum-1-i ];
      CandCostList[ uiFastCandNum-i ] = CandCostList[ uiFastCandNum-1-i ];
    }
    CandModeList[ uiFastCandNum-shift ] = uiMode;
    CandCostList[ uiFastCandNum-shift ] = uiCost;
    return 1;
  }
  
  return 0;
}

/** add inter-prediction syntax elements for a CU block
 * \param pcCU
 * \param uiQp
 * \param uiTrMode
 * \param ruiBits
 * \param rpcYuvRec
 * \param pcYuvPred
 * \param rpcYuvResi
 * \returns Void
 */
Void  TEncSearch::xAddSymbolBitsInter( TComDataCU* pcCU, UInt uiQp, UInt uiTrMode, UInt& ruiBits, TComYuv*& rpcYuvRec, TComYuv*pcYuvPred, TComYuv*& rpcYuvResi )
{
  if(pcCU->getMergeFlag( 0 ) && pcCU->getPartitionSize( 0 ) == SIZE_2Nx2N && !pcCU->getQtRootCbf( 0 ))
  {
    pcCU->setSkipFlagSubParts( true, 0, pcCU->getDepth(0) );

    m_pcEntropyCoder->resetBits();
    if(pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
    }
    m_pcEntropyCoder->encodeSkipFlag(pcCU, 0, true);
#if QC_FRUC_MERGE
    m_pcEntropyCoder->encodeFRUCMgrMode( pcCU, 0 , 0 );
    if( !pcCU->getFRUCMgrMode( 0 ) )
#endif
    m_pcEntropyCoder->encodeMergeIndex(pcCU, 0, true);
#if QC_IC
    m_pcEntropyCoder->encodeICFlag( pcCU, 0 );
#endif
    ruiBits += m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  else
  {
    m_pcEntropyCoder->resetBits();
    if(pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
    }
    m_pcEntropyCoder->encodeSkipFlag ( pcCU, 0, true );
    m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
    m_pcEntropyCoder->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );
    m_pcEntropyCoder->encodePredInfo( pcCU, 0, true );
#if QC_OBMC
    m_pcEntropyCoder->encodeOBMCFlag( pcCU, 0, true );
#endif
#if QC_IC
    m_pcEntropyCoder->encodeICFlag( pcCU, 0 );
#endif
    Bool bDummy = false;
#if ROT_TR  || CU_LEVEL_MPI
  Int bNonZeroCoeff = false;
#endif
    m_pcEntropyCoder->encodeCoeff   ( pcCU, 0, pcCU->getDepth(0), pcCU->getWidth(0), pcCU->getHeight(0), bDummy 
#if ROT_TR  || CU_LEVEL_MPI
  , bNonZeroCoeff 
#endif
  );
    
    ruiBits += m_pcEntropyCoder->getNumberOfWrittenBits();
  }
}

/**
 * \brief Generate half-sample interpolated block
 *
 * \param pattern Reference picture ROI
 * \param biPred    Flag indicating whether block is for biprediction
 */
Void TEncSearch::xExtDIFUpSamplingH( TComPattern* pattern, Bool biPred )
{
  Int width      = pattern->getROIYWidth();
  Int height     = pattern->getROIYHeight();
  Int srcStride  = pattern->getPatternLStride();
  
  Int intStride = m_filteredBlockTmp[0].getStride();
  Int dstStride = m_filteredBlock[0][0].getStride();
  Short *intPtr;
  Short *dstPtr;
  Int filterSize = NTAPS_LUMA;
  Int halfFilterSize = (filterSize>>1);
  Pel *srcPtr = pattern->getROIY() - halfFilterSize*srcStride - 1;
  
  m_if.filterHorLuma(srcPtr, srcStride, m_filteredBlockTmp[0].getLumaAddr(), intStride, width+1, height+filterSize, 0, false);
#if QC_MV_STORE_PRECISION_BIT
  m_if.filterHorLuma(srcPtr, srcStride, m_filteredBlockTmp[2].getLumaAddr(), intStride, width+1, height+filterSize, 2<<(QC_MV_STORE_PRECISION_BIT-2), false);
#else
  m_if.filterHorLuma(srcPtr, srcStride, m_filteredBlockTmp[2].getLumaAddr(), intStride, width+1, height+filterSize, 2, false);
#endif
  
  intPtr = m_filteredBlockTmp[0].getLumaAddr() + halfFilterSize * intStride + 1;  
  dstPtr = m_filteredBlock[0][0].getLumaAddr();
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width+0, height+0, 0, false, true);
  
  intPtr = m_filteredBlockTmp[0].getLumaAddr() + (halfFilterSize-1) * intStride + 1;  
  dstPtr = m_filteredBlock[2][0].getLumaAddr();
#if QC_MV_STORE_PRECISION_BIT
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width+0, height+1, 2<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width+0, height+1, 2, false, true);
#endif
  
  intPtr = m_filteredBlockTmp[2].getLumaAddr() + halfFilterSize * intStride;
  dstPtr = m_filteredBlock[0][2].getLumaAddr();
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width+1, height+0, 0, false, true);
  
  intPtr = m_filteredBlockTmp[2].getLumaAddr() + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[2][2].getLumaAddr();
#if QC_MV_STORE_PRECISION_BIT
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width+1, height+1, 2<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width+1, height+1, 2, false, true);
#endif
}

/**
 * \brief Generate quarter-sample interpolated blocks
 *
 * \param pattern    Reference picture ROI
 * \param halfPelRef Half-pel mv
 * \param biPred     Flag indicating whether block is for biprediction
 */
Void TEncSearch::xExtDIFUpSamplingQ( TComPattern* pattern, TComMv halfPelRef, Bool biPred )
{
  Int width      = pattern->getROIYWidth();
  Int height     = pattern->getROIYHeight();
  Int srcStride  = pattern->getPatternLStride();
  
  Pel *srcPtr;
  Int intStride = m_filteredBlockTmp[0].getStride();
  Int dstStride = m_filteredBlock[0][0].getStride();
  Short *intPtr;
  Short *dstPtr;
  Int filterSize = NTAPS_LUMA;
  
  Int halfFilterSize = (filterSize>>1);

  Int extHeight = (halfPelRef.getVer() == 0) ? height + filterSize : height + filterSize-1;
  
  // Horizontal filter 1/4
  srcPtr = pattern->getROIY() - halfFilterSize * srcStride - 1;
  intPtr = m_filteredBlockTmp[1].getLumaAddr();
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() >= 0)
  {
    srcPtr += 1;
  }
#if QC_MV_STORE_PRECISION_BIT
  m_if.filterHorLuma(srcPtr, srcStride, intPtr, intStride, width, extHeight, 1<<(QC_MV_STORE_PRECISION_BIT-2), false);
#else
  m_if.filterHorLuma(srcPtr, srcStride, intPtr, intStride, width, extHeight, 1, false);
#endif
  
  // Horizontal filter 3/4
  srcPtr = pattern->getROIY() - halfFilterSize*srcStride - 1;
  intPtr = m_filteredBlockTmp[3].getLumaAddr();
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() > 0)
  {
    srcPtr += 1;
  }
#if QC_MV_STORE_PRECISION_BIT
  m_if.filterHorLuma(srcPtr, srcStride, intPtr, intStride, width, extHeight, 3<<(QC_MV_STORE_PRECISION_BIT-2), false);        
#else
  m_if.filterHorLuma(srcPtr, srcStride, intPtr, intStride, width, extHeight, 3, false);        
#endif
  
  // Generate @ 1,1
  intPtr = m_filteredBlockTmp[1].getLumaAddr() + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][1].getLumaAddr();
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
#if QC_MV_STORE_PRECISION_BIT
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true);
#endif
  
  // Generate @ 3,1
  intPtr = m_filteredBlockTmp[1].getLumaAddr() + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][1].getLumaAddr();
#if QC_MV_STORE_PRECISION_BIT
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true);
#endif
  
  if (halfPelRef.getVer() != 0)
  {
    // Generate @ 2,1
    intPtr = m_filteredBlockTmp[1].getLumaAddr() + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[2][1].getLumaAddr();
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
#if QC_MV_STORE_PRECISION_BIT
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 2<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true);
#endif
    
    // Generate @ 2,3
    intPtr = m_filteredBlockTmp[3].getLumaAddr() + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[2][3].getLumaAddr();
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
#if QC_MV_STORE_PRECISION_BIT
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 2<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true);
#endif
  }
  else
  {
    // Generate @ 0,1
    intPtr = m_filteredBlockTmp[1].getLumaAddr() + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][1].getLumaAddr();
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true);
    
    // Generate @ 0,3
    intPtr = m_filteredBlockTmp[3].getLumaAddr() + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][3].getLumaAddr();
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true);
  }
  
  if (halfPelRef.getHor() != 0)
  {
    // Generate @ 1,2
    intPtr = m_filteredBlockTmp[2].getLumaAddr() + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[1][2].getLumaAddr();
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
#if QC_MV_STORE_PRECISION_BIT
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true);
#endif
    
    // Generate @ 3,2
    intPtr = m_filteredBlockTmp[2].getLumaAddr() + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[3][2].getLumaAddr();
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
#if QC_MV_STORE_PRECISION_BIT
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3<<(QC_MV_STORE_PRECISION_BIT-2), false, true);  
#else
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true);  
#endif
  }
  else
  {
    // Generate @ 1,0
    intPtr = m_filteredBlockTmp[0].getLumaAddr() + (halfFilterSize-1) * intStride + 1;
    dstPtr = m_filteredBlock[1][0].getLumaAddr();
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
#if QC_MV_STORE_PRECISION_BIT
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true);
#endif
    
    // Generate @ 3,0
    intPtr = m_filteredBlockTmp[0].getLumaAddr() + (halfFilterSize-1) * intStride + 1;
    dstPtr = m_filteredBlock[3][0].getLumaAddr();
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
#if QC_MV_STORE_PRECISION_BIT
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
    m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true);
#endif
  }
  
  // Generate @ 1,3
  intPtr = m_filteredBlockTmp[3].getLumaAddr() + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][3].getLumaAddr();
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
#if QC_MV_STORE_PRECISION_BIT
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true);
#endif
  
  // Generate @ 3,3
  intPtr = m_filteredBlockTmp[3].getLumaAddr() + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][3].getLumaAddr();
#if QC_MV_STORE_PRECISION_BIT
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3<<(QC_MV_STORE_PRECISION_BIT-2), false, true);
#else
  m_if.filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true);
#endif
}

/** set wp tables
 * \param TComDataCU* pcCU
 * \param iRefIdx
 * \param eRefPicListCur
 * \returns Void
 */
Void  TEncSearch::setWpScalingDistParam( TComDataCU* pcCU, Int iRefIdx, RefPicList eRefPicListCur )
{
  if ( iRefIdx<0 )
  {
    m_cDistParam.bApplyWeight = false;
    return;
  }

  TComSlice       *pcSlice  = pcCU->getSlice();
  TComPPS         *pps      = pcCU->getSlice()->getPPS();
  wpScalingParam  *wp0 , *wp1;
  m_cDistParam.bApplyWeight = ( pcSlice->getSliceType()==P_SLICE && pps->getUseWP() ) || ( pcSlice->getSliceType()==B_SLICE && pps->getWPBiPred() ) ;
  if ( !m_cDistParam.bApplyWeight ) return;

  Int iRefIdx0 = ( eRefPicListCur == REF_PIC_LIST_0 ) ? iRefIdx : (-1);
  Int iRefIdx1 = ( eRefPicListCur == REF_PIC_LIST_1 ) ? iRefIdx : (-1);

  getWpScaling( pcCU, iRefIdx0, iRefIdx1, wp0 , wp1 );

  if ( iRefIdx0 < 0 ) wp0 = NULL;
  if ( iRefIdx1 < 0 ) wp1 = NULL;

  m_cDistParam.wpCur  = NULL;

  if ( eRefPicListCur == REF_PIC_LIST_0 )
  {
    m_cDistParam.wpCur = wp0;
  }
  else
  {
    m_cDistParam.wpCur = wp1;
  }
}

//! \}