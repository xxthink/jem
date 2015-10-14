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

/** \file     TComPrediction.h
    \brief    prediction class (header)
*/

#ifndef __TCOMPREDICTION__
#define __TCOMPREDICTION__


// Include files
#include "TComPic.h"
#include "TComMotionInfo.h"
#include "TComPattern.h"
#include "TComTrQuant.h"
#include "TComInterpolationFilter.h"
#include "TComWeightPrediction.h"
#if QC_FRUC_MERGE
#include "TComRdCost.h"
#include <list>
#endif

//! \ingroup TLibCommon
//! \{

#if QC_FRUC_MERGE
#define QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS    0
#define QC_FRUC_MERGE_MV_SEARCHPATTERN_SQUARE   1
#define QC_FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND  2
#define QC_FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON  3
#endif
// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
class TComPrediction : public TComWeightPrediction
{
protected:
#if BIO  
  Pel* m_pGradX0;
  Pel* m_pGradY0;
  Pel* m_pGradX1;
  Pel* m_pGradY1;
  Pel*  m_pPred0 ;
  Pel*  m_pPred1 ;

  Int iRefListIdx;
#endif
  Int*      m_piYuvExt;
  Int       m_iYuvExtStride;
  Int       m_iYuvExtHeight;
  
  TComYuv   m_acYuvPred[2];
  TComYuv   m_cYuvPredTemp;
  TComYuv m_filteredBlock[4][4];
  TComYuv m_filteredBlockTmp[4];
  
  TComInterpolationFilter m_if;

#if QC_LMCHROMA
  UInt m_uiaLMShift[ 32 ];       // Table for multiplication to substitue of division operation
#endif
 
  Int*   m_pLumaRecBuffer;       ///< array for downsampled reconstructed luma sample 
  Int    m_iLumaRecStride;       ///< stride of #m_pLumaRecBuffer array
#if QC_IC
  UInt   m_uiaICShift[ 64 ];     // Table for multiplication to substitue of division operation
#endif
#if QC_FRUC_MERGE
  TComRdCost m_cFRUCRDCost;
  std::list <TComMvField> m_listMVFieldCand[2];
  TComYuv   m_cYuvPredFrucTemplate[2];      // 0: top, 1: left
  Bool      m_bFrucTemplateAvailabe[2];
#endif
#if QC_SUB_PU_TMVP
  UChar    m_eMergeCandTypeNieghors[MRG_MAX_NUM_CANDS];
#if QC_SUB_PU_TMVP_EXT
  TComMvField  *m_cMvFieldSP[2];
  UChar *m_uhInterDirSP[2];
#else
  TComMvField  m_cMvFieldSP[MAX_NUM_SPU_W*MAX_NUM_SPU_W*2];
  UChar m_uhInterDirSP[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
#endif
#endif

  Void xPredIntraAng            (Int bitDepth, Int* pSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter
#if QC_INTRA_4TAP_FILTER
    , Bool bLuma = false
    , Bool bUse4TapFilter = false
#endif
    );
  Void xPredIntraPlanar         ( Int* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height );
  
  // motion compensation functions
#if BIO  //4
#define BIO_FILTER_LENGTH   6
#define BIO_FILTER_LENGTH_MINUS_1         (BIO_FILTER_LENGTH-1)
#define BIO_FILTER_HALF_LENGTH_MINUS_1   ((BIO_FILTER_LENGTH>>1)-1)
  Void xPredInterFrac(Pel* ref,Pel* dst,Int dstStride,Int refStride,Int xFrac,Int yFrac,Int width, Int height,Bool bi);
  Void  xGradFilterX(Pel*  piRefY, Int iRefStride,Pel*  piDstY,Int iDstStride, Int iWidth, Int iHeight,Int iMVyFrac,Int iMVxFrac);
  Void  xGradFilterY(Pel*  piRefY, Int iRefStride,Pel*  piDstY,Int iDstStride, Int iWidth, Int iHeight,Int iMVyFrac,Int iMVxFrac);
  __inline Void xCTI_Filter2DVerG (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMv);
  __inline Void xCTI_Filter2DHorG (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMV);
  __inline Void xCTI_Filter2DHorGG(Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMV);
  __inline Void xCTI_Filter2DVerGG(Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMv);
  __inline Void xCTI_Filter1DHorG (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMV);
  __inline Void xCTI_Filter1DVerG (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMV);
#endif
  Void xPredInterUni            ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv*& rpcYuvPred
#if BIO
    , Bool bBIOApplied =false
#endif
    , Bool bi=false         
#if QC_FRUC_MERGE
    , Bool bOBMC = false
#endif
    );
  Void xPredInterBi             ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight,                         TComYuv*& rpcYuvPred 
#if QC_FRUC_MERGE
    , Bool bOBMC = false
#endif
    );
  Void xPredInterLumaBlk  ( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi 
#if BIO                  
  ,Bool bBIOapplied =false
#endif
#if QC_FRUC_MERGE
    , Int nFRUCMode = QC_FRUC_MERGE_OFF
#endif
#if QC_IC
    , Bool bICFlag      = false
#endif
    );
  Void xPredInterChromaBlk( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi 
#if QC_IC
    , Bool bICFlag    = false
#endif
    );
  Void xWeightedAverage         ( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv*& rpcYuvDst 
#if BIO
, Bool bBIOApplied
#endif
    );
  
  Void xDCPredFiltering( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight );
#if INTRA_BOUNDARY_FILTER
  Void xIntraPredFilteringModeDGL( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight, UInt uiMode );
  Void xIntraPredFilteringMode34 ( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight);
  Void xIntraPredFilteringMode02 ( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight);
#endif
  Bool xCheckIdenticalMotion    ( TComDataCU* pcCU, UInt PartAddr);
#if CU_LEVEL_MPI
  Void xMPIredFiltering( Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight , Int idxMPI);
#endif
#if QC_SUB_PU_TMVP
  Bool xCheckTwoSPMotion ( TComDataCU* pcCU, UInt PartAddr0, UInt PartAddr1 );
  Void xGetSubPUAddrAndMerge(TComDataCU* pcCU, UInt uiPartAddr, Int iSPWidth, Int iSPHeight, Int iNumSPInOneLine, Int iNumSP, UInt* uiMergedSPW, UInt* uiMergedSPH, UInt* uiSPAddr );
#endif

#if QC_OBMC
  Void xSubblockOBMC ( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp );
  Void xSubtractOBMC ( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp );
  Void xSubBlockMotionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, Int uiPartAddr, Int iWidth, Int iHeight  );
#endif
#if QC_FRUC_MERGE
  Bool xFrucFindBlkMv( TComDataCU * pCU , UInt uiPUIdx );
  Bool xFrucFindBlkMv4Pred( TComDataCU * pCU , UInt uiPUIdx , RefPicList eTargetRefPicList , Int nTargetRefIdx );
  Bool xFrucRefineSubBlkMv( TComDataCU * pCU , UInt uiDepth , UInt uiPUIdx , Bool bTM );

  Void xFrucCollectBlkStartMv( TComDataCU * pCU , UInt uiPUIdx , RefPicList eTargetRefList = REF_PIC_LIST_0 , Int nTargetRefIdx = -1 );
  Void xFrucCollectSubBlkStartMv( TComDataCU * pCU , UInt uiAbsPartIdx , RefPicList eRefPicList , const TComMvField & rMvStart , Int nSubBlkWidth , Int nSubBlkHeight 
#if QC_SUB_PU_TMVP_EXT
    , UInt uiSubBlkRasterIdx , UInt uiSubBlkRasterStep
#endif
    );

  UInt xFrucFindBestMvFromList( TComMvField * pBestMvField , RefPicList & rBestRefPicList , TComDataCU * pCU , UInt uiAbsPartIdx , const TComMvField & rMvStart , Int nBlkWidth , Int nBlkHeight , Bool bTM , Bool bMvCost );

  UInt xFrucRefineMv( TComMvField * pBestMvField , RefPicList eCurRefPicList , UInt uiMinCost , Int nSearchMethod , TComDataCU * pCU , UInt uiAbsPartIdx , const TComMvField & rMvStart , Int nBlkWidth , Int nBlkHeight , Bool bTM );
  template<Int SearchPattern>
  UInt xFrucRefineMvSearch( TComMvField * pBestMvField , RefPicList eCurRefPicList , TComDataCU * pCU , UInt uiAbsPartIdx , TComMvField const & rMvStart , Int nBlkWidth , Int nBlkHeight , UInt uiMinDist , Bool bTM , Int nSearchStepShift , UInt uiMaxSearchRounds = MAX_UINT );

  UInt xFrucGetMvCost( const TComMv & rMvStart , const TComMv & rMvCur , Int nSearchRange , Int nWeighting );
  UInt xFrucGetBilaMatchCost( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nWidth , Int nHeight , RefPicList eCurRefPicList , const TComMvField & rCurMvField , TComMvField & rPairMVField , UInt uiMVCost );
  UInt xFrucGetTempMatchCost( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nWidth , Int nHeight , RefPicList eCurRefPicList , const TComMvField & rCurMvField , UInt uiMVCost );

  Void xFrucInsertMv2StartList( const TComMvField & rMvField , std::list<TComMvField> & rList );
  Bool xFrucIsInList( const TComMvField & rMvField , std::list<TComMvField> & rList );

  Bool xFrucGetCurBlkTemplate( TComDataCU * pCU , UInt uiAbsPartIdx , Int nCurBlkWidth , Int nCurBlkHeight );
  Bool xFrucIsTopTempAvailable( TComDataCU * pCU , UInt uiAbsPartIdx );
  Bool xFrucIsLeftTempAvailable( TComDataCU * pCU , UInt uiAbsPartIdx );
  Int  xFrucGetSubBlkSize( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nBlkWidth , Int nBlkHeight );
#endif
#if QC_IC
  Void xGetLLSICPrediction( TComDataCU* pcCU, TComMv *pMv, TComPicYuv *pRefPic, Int &a, Int &b, TextType eType );
#endif
public:
  TComPrediction();
  virtual ~TComPrediction();
  
  Void    initTempBuff();
  
  // inter
#if QC_OBMC
  Void subBlockOBMC               ( TComDataCU*  pcCU, UInt uiAbsPartIdx, TComYuv *pcYuvPred, TComYuv *pcYuvTmpPred1, TComYuv *pcYuvTmpPred2, Bool bOBMC4ME = false );
#endif
  Void motionCompensation         ( TComDataCU*  pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList = REF_PIC_LIST_X, Int iPartIdx = -1 );
#if QC_FRUC_MERGE
  Bool deriveFRUCMV( TComDataCU * pCU , UInt uiDepth , UInt uiAbsPartIdx , UInt uiPUIdx , Int nTargetRefIdx = -1 , RefPicList eTargetRefList = REF_PIC_LIST_0 );
#endif

  // motion vector prediction
  Void getMvPredAMVP              ( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred );
  
  // Angular Intra
  Void predIntraLumaAng           ( TComPattern* pcTComPattern, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft 
#if QC_INTRA_4TAP_FILTER
    , Bool bUse4TapFilter
#endif
#if INTRA_BOUNDARY_FILTER
    , Bool bUseBoundaryFilter
#endif
#if QC_USE_65ANG_MODES
    , Bool bUseExtIntraAngModes
#endif
#if CU_LEVEL_MPI
, TComDataCU* pcCU, UInt uiAbsPartIdx
#endif
    );
  Void predIntraChromaAng         ( Int* piSrc, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft 
#if QC_INTRA_4TAP_FILTER
    , Bool bUse4TapFilter
#endif
#if CU_LEVEL_MPI
, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiChromaIdx
#endif
    );
  
  Pel  predIntraGetPredValDC      ( Int* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft );
  
  Int* getPredicBuf()             { return m_piYuvExt;      }
  Int  getPredicBufWidth()        { return m_iYuvExtStride; }
  Int  getPredicBufHeight()       { return m_iYuvExtHeight; }

#if QC_LMCHROMA
  Void predLMIntraChroma ( TComPattern* pcPattern, UInt uiChromaId, Pel* pPred, UInt uiPredStride, UInt uiCWidth, UInt uiCHeight );
  Void getLumaRecPixels  ( TComPattern* pcPattern, UInt uiCWidth, UInt uiCHeight, Bool bLeftPicBoundary );
  Void addCrossColorResi ( TComPattern* pcPattern, Pel* piPred, UInt uiPredStride, UInt uiWidth, UInt uiHeight, Pel* piResi, UInt uiResiStride );
  Void xGetLMParameters  ( TComPattern* pcPattern, UInt uiWidth, UInt uiHeight, Int iPredType, UInt uiChromaId, Int &a, Int &b, Int &iShift );
  Void xCalcLMParameters ( Int x, Int y, Int xx, Int xy, Int iCountShift, Int iPredType, Int &a, Int &b, Int &iShift );
#endif
#if INTER_KLT
  Void interpolatePic ( TComPic* pcPic );
#endif
};

//! \}

#endif // __TCOMPREDICTION__
