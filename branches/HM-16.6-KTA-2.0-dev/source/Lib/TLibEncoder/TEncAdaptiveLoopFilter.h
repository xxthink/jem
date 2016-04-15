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

/** \file     TEncAdaptiveLoopFilter.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#ifndef __TENCADAPTIVELOOPFILTER__
#define __TENCADAPTIVELOOPFILTER__

#include "../TLibCommon/TComAdaptiveLoopFilter.h"
#include "../TLibCommon/TComPic.h"

#include "TEncEntropy.h"
#include "TEncSbac.h"
#include "../TLibCommon/TComBitCounter.h"

// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if ALF_HM3_REFACTOR
/// estimation part of adaptive loop filter class
class TEncAdaptiveLoopFilter : public TComAdaptiveLoopFilter
{
private:
  static const Int m_aiSymmetricArray9x9[81];     ///< scan index for 9x9 filter
  static const Int m_aiSymmetricArray7x7[49];     ///< scan index for 7x7 filter
  static const Int m_aiSymmetricArray5x5[25];     ///< scan index for 5x5 filter
  static const Int m_aiSymmetricArray9x7[63];     ///< scan index for 9x7 filter
  
  Double** m_ppdAlfCorr;
  Double* m_pdDoubleAlfCoeff;
  
  SliceType m_eSliceType;
  Int m_iPicNalReferenceIdc;
  
  Double m_dLambdaLuma;
  Double m_dLambdaChroma;
  
  TEncEntropy* m_pcEntropyCoder;
  
  TComPic* m_pcPic;
  ALFParam* m_pcBestAlfParam;
  ALFParam* m_pcTempAlfParam;
  
  TComPicYuv* m_pcPicYuvBest;
  TComPicYuv* m_pcPicYuvTmp;
  
  UInt m_uiNumSCUInCU;
  
  Int m_varIndTab[m_NO_VAR_BINS];
  double ***m_yGlobalSym;
  double ****m_EGlobalSym;
  double *m_pixAcc;
  Int **m_filterCoeffSymQuant;
  imgpel **m_varImg;
  imgpel **m_maskImg;
  Int m_im_width;
  Int m_im_height;
  ALFParam *m_tempALFp;
  TEncEntropy* m_pcDummyEntropyCoder;
  
  double **m_y_merged;
  double ***m_E_merged;
  double *m_pixAcc_merged;
  double *m_y_temp;
  double **m_E_temp;
  
  Int *m_filterCoeffQuantMod;
  double *m_filterCoeff;
  Int *m_filterCoeffQuant;
  Int **m_diffFilterCoeffQuant;
  Int **m_FilterCoeffQuantTemp;
  
  static Int  m_aiTapPos9x9_In9x9Sym[21];
  static Int  m_aiTapPos7x7_In9x9Sym[14];
  static Int  m_aiTapPos5x5_In9x9Sym[8];
  static Int* m_iTapPosTabIn9x9Sym[m_NO_TEST_FILT];

private:
  // init / uninit internal variables
  Void xInitParam      ();
  Void xUninitParam    ();
  
  // create/destroy/copy/set functions of ALF control flags
  Void xCreateTmpAlfCtrlFlags   ();
  Void xDestroyTmpAlfCtrlFlags  ();
  Void xCopyTmpAlfCtrlFlagsTo   ();
  Void xCopyTmpAlfCtrlFlagsFrom ();
  
  // encoder ALF control flags
  Void xEncodeCUAlfCtrlFlags  ();
  Void xEncodeCUAlfCtrlFlag   ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth);
  
  // functions related to correlation computation
  Void xCalcCorrelationFunc   ( Pel* pOrg, Pel* pCmp, Int iTap, Int iWidth, Int iHeight, Int iOrgStride, Int iCmpStride);

  // functions related to filtering
  Void xFilterCoefQuickSort   ( Double *coef_data, Int *coef_num, Int upper, Int lower );
  Void xQuantFilterCoef       ( Double* h, Int* qh, Int tap, int bit_depth );
  Void xClearFilterCoefInt    ( Int* qh, Int N );
  Void xCopyDecToRestCUs      ( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );
  Void xCopyDecToRestCU       ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );
  Void xFilteringFrameChroma  ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );
  
  // distortion / misc functions
  UInt64 xCalcSSD             ( Pel* pOrg, Pel* pCmp, Int iWidth, Int iHeight, Int iStride );
  Void   xCalcRDCost          ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicCmp, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost, const TComSlice * pSlice);
  Void   xCalcRDCostChroma    ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicCmp, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost, const TComSlice * pSlice);
  Void   xCalcRDCost          ( ALFParam* pAlfParam, UInt64& ruiRate, UInt64 uiDist, Double& rdCost, const TComSlice * pSlice);
  Int    xGauss               ( Double **a, Int N );
  
  Void  xretriveBlockMatrix    (Int iNumTaps, Int* piTapPosInMaxFilter, Double*** pppdEBase, Double*** pppdETarget, Double** ppdyBase, Double** ppdyTarget );
  Void  setInitialMask(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec);

protected:
  /// do ALF for chroma
  Void xEncALFChroma          ( UInt64 uiLumaRate, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist, UInt64& ruiBits , const TComSlice * pSlice );
public:
  TEncAdaptiveLoopFilter          ();
  virtual ~TEncAdaptiveLoopFilter () {}
  
  /// allocate temporal memory
  Void startALFEnc(TComPic* pcPic, TEncEntropy* pcEntropyCoder);
  
  /// destroy temporal memory
  Void endALFEnc();
  
  /// estimate ALF parameters
  Void ALFProcess(ALFParam* pcAlfParam, Double dLambdaLuma, Double dLambdaChroma, UInt64& ruiDist, UInt64& ruiBits, UInt& ruiMaxAlfCtrlDepth
#if COM16_C806_ALF_TEMPPRED_NUM
    , ALFParam *pcStoredAlfPara, Int iStoredAlfParaNum
#endif
    );
#if COM16_C806_ALF_TEMPPRED_NUM
  Bool xFilteringLumaChroma( ALFParam *pAlfParam, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost, Int uiIndex, const TComSlice * pSlice );
  Void xcopyFilterCoeff(int filtNo, int **filterCoeff);
#endif
  /// test ALF for luma
  Void xEncALFLuma_qc                  ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, 
                                         UInt64& ruiMinDist, Double& rdMinCost, const TComSlice * pSlice);
  Void xCUAdaptiveControl_qc           ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, 
                                         UInt64& ruiMinDist, Double& rdMinCost, const TComSlice * pSlice );
  Void xSetCUAlfCtrlFlags_qc            (UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, 
                                         UInt64& ruiDist, ALFParam *pAlfParam);
  Void xSetCUAlfCtrlFlag_qc             (TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg,
                                         TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist, ALFParam *pAlfParam);
  Void xReDesignFilterCoeff_qc          (TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec,  TComPicYuv* pcPicRest, Bool bReadCorr, const TComSlice * pSlice);
  Void xFilterTapDecision_qc            (TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, 
                                         UInt64& ruiMinDist, Double& rdMinCost, const TComSlice * pSlice);
  Void xFirstFilteringFrameLuma         (imgpel* ImgOrg, imgpel* ImgDec, imgpel* ImgRest, ALFParam* ALFp, Int tap,  Int Stride, const TComSlice * pSlice);
  Void xstoreInBlockMatrix(imgpel* ImgOrg, imgpel* ImgDec, Int tap, Int Stride);
  Void xFilteringFrameLuma_qc(imgpel* ImgOrg, imgpel* imgY_pad, imgpel* ImgFilt, ALFParam* ALFp, Int tap, Int Stride, const TComSlice * pSlice);
  Void xfilterFrame_en(imgpel* ImgDec, imgpel* ImgRest,int filtNo, int Stride);
  Void xcalcPredFilterCoeff(Int filtNo);
  UInt xcodeFiltCoeff(Int **filterCoeffSymQuant, Int filtNo, Int varIndTab[], Int filters_per_fr_best, Int frNo, ALFParam* ALFp, const TComSlice * pSlice);
  Void xfindBestFilterVarPred(double **ySym, double ***ESym, double *pixAcc, Int **filterCoeffSym, Int **filterCoeffSymQuant,
                              Int filtNo, Int *filters_per_fr_best, Int varIndTab[], imgpel **imgY_rec, imgpel **varImg, 
                              imgpel **maskImg, imgpel **imgY_pad, double lambda_val, const TComSlice * pSlice);
  double xfindBestCoeffCodMethod(int codedVarBins[m_NO_VAR_BINS], int *forceCoeff0, 
                                 int **filterCoeffSymQuant, int fl, int sqrFiltLength, 
                                 int filters_per_fr, double errorForce0CoeffTab[m_NO_VAR_BINS][2], 
                                 double *errorQuant, double lambda, const TComSlice * pSlice);
  Void xcollectStatCodeFilterCoeffForce0(int **pDiffQFilterCoeffIntPP, int fl, int sqrFiltLength, int filters_per_group, 
                                         int bitsVarBin[]);
  Void xdecideCoeffForce0(int codedVarBins[m_NO_VAR_BINS], double errorForce0Coeff[], double errorForce0CoeffTab[m_NO_VAR_BINS][2], 
                          int bitsVarBin[m_NO_VAR_BINS], double lambda, int filters_per_fr);
  Int xsendAllFiltersPPPredForce0(int **FilterCoeffQuant, int fl, int sqrFiltLength, int filters_per_group, 
                                  int codedVarBins[m_NO_VAR_BINS], int createBistream, ALFParam* ALFp, const TComSlice * pSlice);
  Int xsendAllFiltersPPPred(int **FilterCoeffQuant, int fl, int sqrFiltLength, 
                            int filters_per_group, int createBistream, ALFParam* ALFp, const TComSlice * pSlice);
  Int xcodeAuxInfo(int filtNo, int noFilters, int varIndTab[m_NO_VAR_BINS], int frNo, int createBitstream,int realfiltNo, ALFParam* ALFp, const TComSlice * pSlice);
  Int xcodeFilterCoeff(int **pDiffQFilterCoeffIntPP, int fl, int sqrFiltLength, int filters_per_group, int createBitstream);
  Int lengthGolomb(int coeffVal, int k);
  Int lengthPredFlags(int force0, int predMethod, int codedVarBins[m_NO_VAR_BINS], 
                      int filters_per_group, int createBitstream);
  Int lengthFilterCoeffs(int sqrFiltLength, int filters_per_group, const int pDepthInt[], 
                         int **FilterCoeff, int kMinTab[], int createBitstream);
  //cholesky related
  Double findFilterCoeff(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, int **filterCoeffSeq,
                         int **filterCoeffQuantSeq, int intervalBest[m_NO_VAR_BINS][2], int varIndTab[m_NO_VAR_BINS], int sqrFiltLength, 
                         int filters_per_fr, int *weights, int bit_depth, double errorTabForce0Coeff[m_NO_VAR_BINS][2]);
  Double QuantizeIntegerFilterPP(double *filterCoeff, int *filterCoeffQuant, double **E, double *y, 
                                 int sqrFiltLength, int *weights, int bit_depth);
  Void roundFiltCoeff(int *FilterCoeffQuan, double *FilterCoeff, int sqrFiltLength, int factor);
  double findFilterGroupingError(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, 
                                 int intervalBest[m_NO_VAR_BINS][2], int sqrFiltLength, int filters_per_fr);
  double mergeFiltersGreedy(double **yGlobalSeq, double ***EGlobalSeq, double *pixAccGlobalSeq, 
                            int intervalBest[m_NO_VAR_BINS][2], int sqrFiltLength, int noIntervals);
  double calculateErrorAbs(double **A, double *b, double y, int size);
  double calculateErrorCoeffProvided(double **A, double *b, double *c, int size);
  Void add_b(double *bmerged, double **b, int start, int stop, int size);
  Void add_A(double **Amerged, double ***A, int start, int stop, int size);
  Int gnsSolveByChol(double **LHS, double *rhs, double *x, int noEq);
  Void  gnsBacksubstitution(double R[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], double z[m_MAX_SQR_FILT_LENGTH], 
                            int R_size, double A[m_MAX_SQR_FILT_LENGTH]);
  Void gnsTransposeBacksubstitution(double U[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], double rhs[], double x[],
                                    int order);
  Int gnsCholeskyDec(double **inpMatr, double outMatr[m_MAX_SQR_FILT_LENGTH][m_MAX_SQR_FILT_LENGTH], int noEq);
};
#endif

#endif