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

/** \file     TComTrQuant.h
    \brief    transform and quantization class (header)
*/

#ifndef __TCOMTRQUANT__
#define __TCOMTRQUANT__

#include "CommonDef.h"
#include "TComYuv.h"
#include "TComDataCU.h"
#include "ContextTables.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define QP_BITS                 15

#if KLT_COMMON
#define MAX_1DTRANS_LEN              (1 << (((USE_MORE_BLOCKSIZE_DEPTH_MAX) + 1) << 1)) ///< 4x4 = 16, 8x8 = 64, 16x16=256, 32x32 = 1024
extern UInt g_uiDepth2Width[5];
#if INTER_KLT
extern UInt g_uiDepth2TempSize[5];
#endif
#if INTRA_KLT
extern UInt g_uiDepth2IntraTempSize[5];
#endif
extern UInt g_uiDepth2MaxCandiNum[5];
extern UInt g_uiDepth2MinCandiNum[5];
#endif
// ====================================================================================================================
// Type definition
// ====================================================================================================================
#if ROT_TR
 __inline static Short  xMult ( Int i, UInt uiShift ) { return ((i)>>uiShift); }
#endif
void fastForwardDCT2_B4 (Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT2_B4 (Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDCT2_B8 (Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT2_B8 (Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDCT2_B16(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT2_B16(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDCT2_B32(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT2_B32(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDCT2_B64(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT2_B64(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);

void fastForwardDST7_B4 (Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDST7_B4 (Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
#if QC_EMT
void fastForwardDST7_B8 (Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDST7_B8 (Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDST7_B16(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDST7_B16(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDST7_B32(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDST7_B32(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);

void fastForwardDCT5_B4 (Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT5_B4 (Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDCT5_B8 (Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT5_B8 (Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDCT5_B16(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT5_B16(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDCT5_B32(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT5_B32(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);

void fastForwardDCT8_B4 (Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT8_B4 (Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDCT8_B8 (Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT8_B8 (Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDCT8_B16(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT8_B16(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDCT8_B32(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDCT8_B32(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);

void fastForwardDST1_B4 (Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDST1_B4 (Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDST1_B8 (Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDST1_B8 (Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDST1_B16(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDST1_B16(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);
void fastForwardDST1_B32(Short *block,Short *coeff,Int shift, Int line, Int zo, Int use);
void fastInverseDST1_B32(Short *coeff,Short *block,Int shift, Int line, Int zo, Int use);

typedef void Trans (Short *, Short *, Int, Int, Int, Int);
#endif

typedef struct
{
  Int significantCoeffGroupBits[NUM_SIG_CG_FLAG_CTX][2];
  Int significantBits[NUM_SIG_FLAG_CTX][2];
  Int lastXBits[32];
  Int lastYBits[32];
  Int m_greaterOneBits[NUM_ONE_FLAG_CTX][2];
#if !QC_CTX_RESIDUALCODING
  Int m_levelAbsBits[NUM_ABS_FLAG_CTX][2];
#endif

  Int blockCbpBits[3*NUM_QT_CBF_CTX][2];
  Int blockRootCbpBits[4][2];
} estBitsSbacStruct;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// QP class
class QpParam
{
public:
  QpParam();
  
  Int m_iQP;
  Int m_iPer;
  Int m_iRem;
  
public:
  Int m_iBits;
    
  Void setQpParam( Int qpScaled )
  {
    m_iQP   = qpScaled;
    m_iPer  = qpScaled / 6;
    m_iRem  = qpScaled % 6;
    m_iBits = QP_BITS + m_iPer;
  }
  
  Void clear()
  {
    m_iQP   = 0;
    m_iPer  = 0;
    m_iRem  = 0;
    m_iBits = 0;
  }
  
  
  Int per()   const { return m_iPer; }
  Int rem()   const { return m_iRem; }
  Int bits()  const { return m_iBits; }
  
  Int qp() {return m_iQP;}
}; // END CLASS DEFINITION QpParam

#if KLT_COMMON
class TempLibFast
{
public:
  Int *m_pX;    //offset X
  Int *m_pY;    //offset Y
  Int *m_pXInteger;    //offset X for integer pixel search
  Int *m_pYInteger;    //offset Y for integer pixel search
  DistType *m_pDiffInteger;
  Int* getXInteger() { return m_pXInteger; }
  Int* getYInteger() { return m_pYInteger; }
  DistType* getDiffInteger() { return m_pDiffInteger; }
  Short *m_pIdInteger; //frame id
  Short* getIdInteger() { return m_pIdInteger; }
  DistType *m_pDiff; //mse
  Short *m_pId; //frame id
  Int m_iSize;

  TempLibFast();
  ~TempLibFast();
  Void init(UInt iSize);
  Int* getX() { return m_pX; }
  Int* getY() { return m_pY; }
  DistType* getDiff() { return m_pDiff; }
  Short* getId() { return m_pId; }
  Void initDiff(UInt uiPatchSize, Int bitDepth);
  Void initDiff(UInt uiPatchSize, Int bitDepth, Int iCandiNumber);
#if INTRA_KLT
  Void initTemplateDiff(UInt uiPatchSize, UInt uiBlkSize, Int bitDepth, Int iCandiNumber);
#endif
  Int m_diffMax;
  Int getDiffMax() { return m_diffMax; }
};

typedef Short TrainDataType; //typedef Int TrainDataType; can reduce from 18.46second to 17.4second if use short 1.22.2014
typedef Double TrainDataTypeD;
#endif

/// transform and quantization class
class TComTrQuant
{
public:
  TComTrQuant();
  ~TComTrQuant();
  
  // initialize class
  Void init                 ( UInt uiMaxTrSize, Bool useRDOQ = false,  
    Bool useRDOQTS = false,
    Bool bEnc = false, Bool useTransformSkipFast = false
#if ADAPTIVE_QP_SELECTION
    , Bool bUseAdaptQpSelect = false
#endif 
    );
#if ROT_TR
Void InvRotTransform4I(  Int* matrix, UChar index );
Void RotTransform4I( Int* matrix, UChar index );
#endif 
  // transform & inverse transform functions
  Void transformNxN( TComDataCU* pcCU, 
                     Pel*        pcResidual, 
                     UInt        uiStride, 
                     TCoeff*     rpcCoeff, 
#if ADAPTIVE_QP_SELECTION
                     Int*&       rpcArlCoeff, 
#endif
                     UInt        uiWidth, 
                     UInt        uiHeight, 
                     UInt&       uiAbsSum, 
                     TextType    eTType, 
                     UInt        uiAbsPartIdx,
                     Bool        useTransformSkip = false 
#if QC_EMT
                     , UChar     ucTrIdx = DCT2_HEVC
#endif
#if ROT_TR 
   , UChar ucROTIdx = 0
#endif
#if KLT_COMMON
   , Bool useKLT = false
#endif
                     );

  Void invtransformNxN( Bool transQuantBypass, TextType eText, UInt uiMode,Pel* rpcResidual, UInt uiStride, TCoeff*   pcCoeff, UInt uiWidth, UInt uiHeight,  Int scalingListType, Bool useTransformSkip = false 
#if QC_EMT
    , UChar ucTrIdx = DCT2_HEVC
#endif
#if ROT_TR 
   , UChar ucROTIdx = 0
#endif
#if QC_USE_65ANG_MODES
    , Bool bUseExtIntraAngModes = false
#endif
#if KLT_COMMON
  , Bool useKLT = false
#endif
    );

#if INTER_KLT
  Void invRecurTransformNxN( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eTxt, Pel* rpcResidual, UInt uiAddr, UInt uiStride, UInt uiWidth, UInt uiHeight,
    UInt uiMaxTrMode, UInt uiTrMode, TCoeff* rpcCoeff, TComYuv *pcPred);
#else
  Void invRecurTransformNxN ( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eTxt, Pel* rpcResidual, UInt uiAddr,   UInt uiStride, UInt uiWidth, UInt uiHeight,
                             UInt uiMaxTrMode,  UInt uiTrMode, TCoeff* rpcCoeff );
#endif
  
  // Misc functions
  Void setQPforQuant( Int qpy, TextType eTxtType, Int qpBdOffset, Int chromaQPOffset);

#if RDOQ_CHROMA_LAMBDA
  Void setLambdas ( const Double lambdas[3] ) { for (Int component = 0; component < 3; component++) m_lambdas[component] = lambdas[component]; }
  Void selectLambda(TextType eTType) { m_dLambda = (eTType == TEXT_LUMA) ? m_lambdas[0] : ((eTType == TEXT_CHROMA_U) ? m_lambdas[1] : m_lambdas[2]); }
#else
  Void setLambda(Double dLambda) { m_dLambda = dLambda;}
#endif

#if CR_FROM_CB_LAMBDA_ADJUSTMENT
  Void setLambda( Double dLambda) { m_dLambda = dLambda; }
  Double getlambda () { return m_dLambda; }
#endif

  Void setRDOQOffset( UInt uiRDOQOffset ) { m_uiRDOQOffset = uiRDOQOffset; }
  
  estBitsSbacStruct* m_pcEstBitsSbac;
#if QC_CTX_RESIDUALCODING
  static Int getGrtZeroCtxInc    ( TCoeff*                         pcCoeff,
                                   Int                             posX,
                                   Int                             posY,
                                   Int                             width
                                  ,Int                             height
                                  ,TextType                        textureType 
                                  );
  static Int getGrtOneCtxInc    ( TCoeff*                         pcCoeff,
                                   Int                             posX,
                                   Int                             posY,
                                   Int                             width
                                  ,Int                             height
                                  ,TextType                        textureType 
                                  );
  static Int getGrtTwoCtxInc    ( TCoeff*                         pcCoeff,
                                   Int                             posX,
                                   Int                             posY,
                                   Int                             width
                                  ,Int                             height
                                  ,TextType                        textureType 
                                  );
  static Int getRemainCoeffCtxInc( TCoeff*                         pcCoeff,
                                   Int                             posX,
                                   Int                             posY,
                                   Int                             width
                                  ,Int                             height
                                  );
  static Int      getSigCtxInc     ( TCoeff*                         pcCoeff,
                                     Int                             posX,
                                     Int                             posY,
                                     Int                             width
                                    ,Int                             height
                                    ,TextType                        textureType
                                    ,UInt&                           sumOne
                                    ,UInt&                           sumTwo
                                    ,UInt&                           sumAbs
                                    );
#else  
  static Int      calcPatternSigCtx( const UInt* sigCoeffGroupFlag, UInt posXCG, UInt posYCG, Int width, Int height );

  static Int      getSigCtxInc     (
                                     Int                             patternSigCtx,
                                     UInt                            scanIdx,
                                     Int                             posX,
                                     Int                             posY,
                                     Int                             log2BlkSize,
                                     TextType                        textureType
                                    );
#endif
  static UInt getSigCoeffGroupCtxInc  ( const UInt*                   uiSigCoeffGroupFlag,
                                       const UInt                       uiCGPosX,
                                       const UInt                       uiCGPosY,
#if QC_CTX_RESIDUALCODING
                                       const UInt                      scanIdx,
#endif
                                       Int width, Int height);
  Void initScalingList                      ();
  Void destroyScalingList                   ();
  Void setErrScaleCoeff    ( UInt list, UInt size, UInt qp);
  Double* getErrScaleCoeff ( UInt list, UInt size, UInt qp) {return m_errScale[size][list][qp];};    //!< get Error Scale Coefficent
  Int* getQuantCoeff       ( UInt list, UInt qp, UInt size) {return m_quantCoef[size][list][qp];};   //!< get Quant Coefficent
  Int* getDequantCoeff     ( UInt list, UInt qp, UInt size) {return m_dequantCoef[size][list][qp];}; //!< get DeQuant Coefficent
  Void setUseScalingList   ( Bool bUseScalingList){ m_scalingListEnabledFlag = bUseScalingList; };
  Bool getUseScalingList   (){ return m_scalingListEnabledFlag; };
#if HM14_CLEAN_UP
  Void setFlatScalingList  (Bool bEnc = true);
#else
  Void setFlatScalingList  ();
#endif
  Void xsetFlatScalingList ( UInt list, UInt size, UInt qp);
  Void xSetScalingListEnc  ( TComScalingList *scalingList, UInt list, UInt size, UInt qp);
  Void xSetScalingListDec  ( TComScalingList *scalingList, UInt list, UInt size, UInt qp);
  Void setScalingList      ( TComScalingList *scalingList);
  Void setScalingListDec   ( TComScalingList *scalingList);
  Void processScalingListEnc( Int *coeff, Int *quantcoeff, Int quantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc);
  Void processScalingListDec( Int *coeff, Int *dequantcoeff, Int invQuantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc);
#if ADAPTIVE_QP_SELECTION
  Void    initSliceQpDelta() ;
  Void    storeSliceQpNext(TComSlice* pcSlice);
  Void    clearSliceARLCnt();
  Int     getQpDelta(Int qp) { return m_qpDelta[qp]; } 
  Int*    getSliceNSamples(){ return m_sliceNsamples ;} 
  Double* getSliceSumC()    { return m_sliceSumC; }
#endif

#if KLT_COMMON
  Void calcCovMatrix(TrainDataType **pData, UInt uiSampleNum, covMatrixType *pCovMatrix, UInt uiDim, DistType *pDiff);
  Void calcCovMatrixXXt(TrainDataType **pData, UInt uiSampleNum, covMatrixType *pCovMatrix, UInt uiDim);
  DistType calcTemplateDiff(Pel *ref, UInt uiStride, Pel **tarPatch, UInt uiPatchSize, UInt uiTempSize, DistType iMax);
#if ENABLE_SEP_KLT
  Void calcMatrixCovMatrix(TrainDataType **pData, UInt uiSampleNum, covMatrixType *pCovMatrix, UInt uiRows, UInt uiCols, Bool bCorrAmongColums);
#endif
  Void calcCovMatrix(TrainDataType **pData, UInt uiSampleNum, covMatrixType *pCovMatrix, UInt uiDim);
  Bool deriveKLT(UInt uiBlkSize, UInt uiUseCandiNumber);
  Bool derive1DimKLT_Fast(UInt uiBlkSize, DistType *pDiff, UInt uiUseCandiNumber);
  Bool derive1DimKLT(UInt uiBlkSize, DistType *pDiff, UInt uiUseCandiNumber);
  Bool derive2DimKLT(UInt uiBlkSize, DistType *pDiff);
  Pel  **getTargetPatch(UInt uiDepth) { return m_pppTarPatch[uiDepth]; }
  Pel* getRefPicUsed(UInt uiId) { return m_refPicUsed[uiId]; }
  Void setRefPicUsed(UInt uiId, Pel *ref) { m_refPicUsed[uiId] = ref; }
  UInt getStride() { return m_uiPicStride; }
  Void setStride(UInt uiPicStride) { m_uiPicStride = uiPicStride; }

#endif
#if INTRA_KLT
  Void searchCandidateFromOnePicIntra(TComDataCU *pcCU, UInt uiPartAddr, TComPic* refPicSrc, TComPicYuv *refPic, TComMv  cMv, Pel **tarPatch, UInt uiPatchSize, UInt uiTempSize, UInt setId);
  Void candidateSearchIntra(TComDataCU *pcCU, UInt uiPartAddr, UInt uiBlkSize, UInt uiTempSize);
  Bool generateTMPrediction(Pel *piPred, UInt uiStride, UInt uiBlkSize, UInt uiTempSize, Int genPred0genPredAndtrainKLT1, Int &foundCandiNum);
  Void getTargetTemplate(TComDataCU *pcCU, UInt uiAbsPartIdx, UInt absTUPartIdx, TComYuv* pcPred, UInt uiBlkSize, UInt uiTempSize);
  Bool calcKLTIntra(Pel *piPred, UInt uiStride, UInt uiBlkSize, UInt uiTempSize);
  Bool prepareKLTSamplesIntra(Pel *piPred, UInt uiStride, UInt uiBlkSize, UInt uiTempSize);
#endif
#if INTER_KLT
  Void getTargetPatch(TComDataCU *pcCU, UInt uiAbsPartIdx, UInt absTUPartIdx, TComYuv* pcPred, UInt uiBlkSize, UInt uiTempSize);
  Void candidateSearch(TComDataCU *pcCU, UInt uiPartAddr, UInt uiBlkSize, UInt uiTempSize);
  Void searchCandidateFromOnePicInteger(TComDataCU *pcCU, UInt uiPartAddr, TComPic* refPicSrc, TComPicYuv *refPic, TComMv  cMv, Pel **tarPatch, UInt uiPatchSize, UInt uiTempSize, UInt setId, Bool bInteger);
  Void searchCandidateFraBasedOnInteger(TComDataCU *pcCU, Pel **tarPatch, UInt uiPatchSize, UInt uiTempSize, UInt uiPartAddr, Short setIdFraStart);
  Void RecordPosition(UInt uiTargetCandiNum);
  Bool candidateTrain(TComDataCU *pcCU, UInt uiAbsPartIdx, UInt uiBlkSize, UInt uiTempSize);
  Bool prepareKLTSamplesInter(TComDataCU *pcCU, UInt uiAbsPartIdx, UInt uiBlkSize, UInt uiTempSize);
  Void setRefPicBuf(UInt uiId, TComPic *refPic) { m_refPicBuf[uiId] = refPic; }
  TComPic* getRefPicBuf(UInt uiId) { return m_refPicBuf[uiId]; }
  DistType calcPatchDiff(Pel *ref, UInt uiStride, Pel **tarPatch, UInt uiPatchSize, UInt uiTempSize, DistType iMax);
  Void xSetSearchRange(TComDataCU* pcCU, TComMv& cMvPred, Int iSrchRng, TComMv& rcMvSrchRngLT, TComMv& rcMvSrchRngRB);
#endif

protected:
#if ADAPTIVE_QP_SELECTION
  Int     m_qpDelta[MAX_QP+1]; 
  Int     m_sliceNsamples[LEVEL_RANGE+1];  
  Double  m_sliceSumC[LEVEL_RANGE+1] ;  
#endif
  Int*    m_plTempCoeff;
#if ROT_TR
  //Int* ROT_MATRIX;  
#endif  
  QpParam  m_cQP;
#if RDOQ_CHROMA_LAMBDA
  Double   m_lambdas[3];
#endif
  Double   m_dLambda;
  UInt     m_uiRDOQOffset;
  UInt     m_uiMaxTrSize;
  Bool     m_bEnc;
  Bool     m_useRDOQ;
  Bool     m_useRDOQTS;
#if ADAPTIVE_QP_SELECTION
  Bool     m_bUseAdaptQpSelect;
#endif
  Bool     m_useTransformSkipFast;
  Bool     m_scalingListEnabledFlag;
  Int      *m_quantCoef      [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
  Int      *m_dequantCoef    [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of dequantization matrix coefficient 4x4
  Double   *m_errScale       [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
#if KLT_COMMON
  Int m_uiPartLibSize;
  TempLibFast m_tempLibFast;
  Pel *m_refPicUsed[MAX_NUM_REF_IDS];
  TComPic *m_refPicBuf[MAX_NUM_REF_IDS];
  UInt m_uiPicStride;
  TrainDataType *m_pData[MAX_CANDI_NUM];
#if USE_TRANSPOSE_CANDDIATEARRAY
  TrainDataType *m_pDataT[MAX_1DTRANS_LEN];
#endif
  UInt m_uiVaildCandiNum;
  Double m_pEigenValues[MAX_1DTRANS_LEN];
  Int m_pIDTmp[MAX_1DTRANS_LEN];
  EigenType ***m_pppdEigenVector;
  Short ***m_pppsEigenVector;
  covMatrixType **m_pCovMatrix;
  Pel ***m_pppTarPatch;
#if FAST_DERIVE_KLT
  EigenType **m_pppdTmpEigenVector;
#endif

#if ENABLE_SEP_KLT
  EigenType ****m_ppppd2DimEigenVector; //[depth][R or C][row][col]
  Short ****m_pppps2DimEigenVector;
#endif
#endif

private:
  // forward Transform
  Void xT   (Int bitDepth, UInt uiMode,Pel* pResidual, UInt uiStride, Int* plCoeff, Int iWidth, Int iHeight 
#if QC_EMT
    , UChar ucTrIdx
#endif
#if QC_USE_65ANG_MODES
    , Bool bUseExtIntraAngModes = false
#endif
    );
  
  // skipping Transform
  Void xTransformSkip (Int bitDepth, Pel* piBlkResi, UInt uiStride, Int* psCoeff, Int width, Int height );

  Void signBitHidingHDQ( TCoeff* pQCoef, TCoeff* pCoef, UInt const *scan, Int* deltaU, Int width, Int height );

  // quantization
  Void xQuant( TComDataCU* pcCU, 
               Int*        pSrc, 
               TCoeff*     pDes, 
#if ADAPTIVE_QP_SELECTION
               Int*&       pArlDes,
#endif
               Int         iWidth, 
               Int         iHeight, 
               UInt&       uiAcSum, 
               TextType    eTType, 
               UInt        uiAbsPartIdx );

  // RDOQ functions
  
  Void           xRateDistOptQuant ( TComDataCU*                     pcCU,
                                     Int*                            plSrcCoeff,
                                     TCoeff*                         piDstCoeff,
#if ADAPTIVE_QP_SELECTION
                                     Int*&                           piArlDstCoeff,
#endif
                                     UInt                            uiWidth,
                                     UInt                            uiHeight,
                                     UInt&                           uiAbsSum,
                                     TextType                        eTType,
                                     UInt                            uiAbsPartIdx );
__inline UInt              xGetCodedLevel  ( Double&                         rd64CodedCost,
                                             Double&                         rd64CodedCost0,
                                             Double&                         rd64CodedCostSig,
                                             Int                             lLevelDouble,
                                             UInt                            uiMaxAbsLevel,
                                             UShort                          ui16CtxNumSig,
                                             UShort                          ui16CtxNumOne,
                                             UShort                          ui16CtxNumAbs,
                                             UShort                          ui16AbsGoRice,
                                             UInt                            c1Idx,  
                                             UInt                            c2Idx,  
                                             Int                             iQBits,
                                             Double                          dTemp,
                                             Bool                            bLast        ) const;
__inline Int xGetICRate  ( UInt                            uiAbsLevel,
                           UShort                          ui16CtxNumOne,
                           UShort                          ui16CtxNumAbs,
                           UShort                          ui16AbsGoRice
                         , UInt                            c1Idx,
                           UInt                            c2Idx
                         ) const;
  __inline Double xGetRateLast     ( const UInt                      uiPosX,
                                     const UInt                      uiPosY ) const;
  __inline Double xGetRateSigCoeffGroup (  UShort                    uiSignificanceCoeffGroup,
                                     UShort                          ui16CtxNumSig ) const;
  __inline Double xGetRateSigCoef (  UShort                          uiSignificance,
                                     UShort                          ui16CtxNumSig ) const;
  __inline Double xGetICost        ( Double                          dRate         ) const; 
  __inline Double xGetIEPRate      (                                               ) const;
  
  
  // dequantization
  Void xDeQuant(Int bitDepth, const TCoeff* pSrc, Int* pDes, Int iWidth, Int iHeight, Int scalingListType );
  
  // inverse transform
  Void xIT    (Int bitDepth, UInt uiMode, Int* plCoef, Pel* pResidual, UInt uiStride, Int iWidth, Int iHeight 
#if QC_EMT
    , UChar ucTrIdx
#endif
#if QC_USE_65ANG_MODES
    , Bool bUseExtIntraAngModes = false
#endif
    );
  
  // inverse skipping transform
  Void xITransformSkip (Int bitDepth, Int* plCoef, Pel* pResidual, UInt uiStride, Int width, Int height );
};// END CLASS DEFINITION TComTrQuant

//! \}

#endif // __TCOMTRQUANT__


#ifndef __SSE_H__
#define __SSE_H__

typedef unsigned short U16;
typedef unsigned int UInt;
typedef short I16;
typedef int   Int;

float InnerProduct_SSE_FLOATXSHORT(float *pa, short *pb, int m);
int InnerProduct_SSE_SHORT(short *pa, short *pb, int m);
void scaleMatrix(float **ppx, short **ppout, float scale, int rows, int cols);
void scaleMatrix(float **ppx, short **ppout, float scale, int rows, int cols);
int AbsSumOfVector(short *pa, short *pb, int m);
int AbsSumOfVectorLesseqthan8(short *pa, short *pb, int m);

UInt GetSAD4x4_SSE_U16(I16 **pSrc, I16 *pRef, Int iRefStride, Int iYOffset, Int iXOffset, UInt uiBestSAD);
UInt GetSAD8x8_SSE_U16(I16 **pSrc, I16 *pRef, Int iRefStride, Int iYOffset, Int iXOffset, UInt uiBestSAD);
UInt GetSAD16x16_SSE_U16(I16 **pSrc, I16 *pRef, Int iRefStride, Int iYOffset, Int iXOffset, UInt uiBestSAD);
UInt GetSAD32x32_SSE_U16(I16 **pSrc, I16 *pRef, Int iRefStride, Int iYOffset, Int iXOffset, UInt uiBestSAD);

#endif
