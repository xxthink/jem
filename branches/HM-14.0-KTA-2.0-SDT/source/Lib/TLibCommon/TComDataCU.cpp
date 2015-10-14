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

/** \file     TComDataCU.cpp
    \brief    CU data structure
    \todo     not all entities are documented
*/

#include "TComDataCU.h"
#include "TComPic.h"
#if QC_FRUC_MERGE
#include "TComPrediction.h"
#endif


//! \ingroup TLibCommon
//! \{
#if KLT_COMMON
extern UInt g_uiDepth2Width[5];
#endif

#if ADAPTIVE_QP_SELECTION
Int * TComDataCU::m_pcGlbArlCoeffY  = NULL;
Int * TComDataCU::m_pcGlbArlCoeffCb = NULL;
Int * TComDataCU::m_pcGlbArlCoeffCr = NULL;
#endif

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComDataCU::TComDataCU()
{
  m_pcPic              = NULL;
  m_pcSlice            = NULL;
  m_puhDepth           = NULL;
  
  m_skipFlag           = NULL;
#if ROT_TR
  m_ROTIdx           = NULL;
#endif
#if CU_LEVEL_MPI
  m_MPIIdx           = NULL;
#endif
#if QC_IMV
  m_iMVFlag            = NULL;
  m_piMVCandNum        = NULL;
#endif
#if QC_OBMC
  m_OBMCFlag           = NULL;
#endif
#if QC_IC
  m_pbICFlag           = NULL;
#endif
  m_pePartSize         = NULL;
  m_pePredMode         = NULL;
  m_CUTransquantBypass = NULL;
  m_puhWidth           = NULL;
  m_puhHeight          = NULL;
  m_phQP               = NULL;
  m_pbMergeFlag        = NULL;
  m_puhMergeIndex      = NULL;
#if QC_FRUC_MERGE
  m_puhFRUCMgrMode     = NULL;
#endif
  m_puhLumaIntraDir    = NULL;
  m_puhChromaIntraDir  = NULL;
  m_puhInterDir        = NULL;
#if QC_EMT
  m_puhEmtTuIdx        = NULL;
  m_puhEmtCuFlag       = NULL;
#endif
  m_puhTrIdx           = NULL;
  m_puhTransformSkip[0] = NULL;
  m_puhTransformSkip[1] = NULL;
  m_puhTransformSkip[2] = NULL;
#if KLT_COMMON
  m_puhKLTFlag[0]    = NULL;
  m_puhKLTFlag[1]    = NULL;
  m_puhKLTFlag[2]    = NULL;
#endif
  m_puhCbf[0]          = NULL;
  m_puhCbf[1]          = NULL;
  m_puhCbf[2]          = NULL;
  m_pcTrCoeffY         = NULL;
  m_pcTrCoeffCb        = NULL;
  m_pcTrCoeffCr        = NULL;
#if ADAPTIVE_QP_SELECTION  
  m_ArlCoeffIsAliasedAllocation = false;
  m_pcArlCoeffY        = NULL;
  m_pcArlCoeffCb       = NULL;
  m_pcArlCoeffCr       = NULL;
#endif
  
  m_pbIPCMFlag         = NULL;
  m_pcIPCMSampleY      = NULL;
  m_pcIPCMSampleCb     = NULL;
  m_pcIPCMSampleCr     = NULL;

  m_pcPattern          = NULL;
  
  m_pcCUAboveLeft      = NULL;
  m_pcCUAboveRight     = NULL;
  m_pcCUAbove          = NULL;
  m_pcCULeft           = NULL;
  
  m_apcCUColocated[0]  = NULL;
  m_apcCUColocated[1]  = NULL;
  
  m_apiMVPIdx[0]       = NULL;
  m_apiMVPIdx[1]       = NULL;
  m_apiMVPNum[0]       = NULL;
  m_apiMVPNum[1]       = NULL;

  m_bDecSubCu          = false;
  m_sliceStartCU        = 0;
  m_sliceSegmentStartCU = 0;
#if QC_SUB_PU_TMVP
  m_peMergeType        = NULL;
#endif
#if ALF_HM3_QC_REFACTOR
  m_puiAlfCtrlFlag     = NULL;
  m_puiTmpAlfCtrlFlag  = NULL;
#endif
}

TComDataCU::~TComDataCU()
{
}

Void TComDataCU::create(UInt uiNumPartition, UInt uiWidth, UInt uiHeight, Bool bDecSubCu, Int unitSize
#if ADAPTIVE_QP_SELECTION
                        , Bool bGlobalRMARLBuffer
#endif                                              
                        )
{
  m_bDecSubCu = bDecSubCu;
  
  m_pcPic              = NULL;
  m_pcSlice            = NULL;
  m_uiNumPartition     = uiNumPartition;
  m_unitSize = unitSize;
  
  if ( !bDecSubCu )
  {
    m_phQP               = (Char*     )xMalloc(Char,     uiNumPartition);
    m_puhDepth           = (UChar*    )xMalloc(UChar,    uiNumPartition);
#if QC_LARGE_CTU
    m_puhWidth           = (UShort*   )xMalloc(UShort,    uiNumPartition);
    m_puhHeight          = (UShort*   )xMalloc(UShort,    uiNumPartition);
#else
    m_puhWidth           = (UChar*    )xMalloc(UChar,    uiNumPartition);
    m_puhHeight          = (UChar*    )xMalloc(UChar,    uiNumPartition);
#endif

    m_skipFlag           = new Bool[ uiNumPartition ];
#if ROT_TR
    m_ROTIdx           = new Char[ uiNumPartition ];
#endif
#if CU_LEVEL_MPI
    m_MPIIdx           = new Char[ uiNumPartition ];
#endif
#if QC_IMV
    m_iMVFlag            = new Bool[ uiNumPartition ];
    m_piMVCandNum        = new Char[ uiNumPartition ];
#endif
#if QC_OBMC
    m_OBMCFlag           = new Bool[ uiNumPartition ];
#endif
#if QC_IC
    m_pbICFlag           = new Bool[ uiNumPartition ];
#endif
    m_pePartSize         = new Char[ uiNumPartition ];
#if !HM14_CLEAN_UP
    memset( m_pePartSize, SIZE_NONE,uiNumPartition * sizeof( *m_pePartSize ) );
#endif
    m_pePredMode         = new Char[ uiNumPartition ];
#if QC_SUB_PU_TMVP
    m_peMergeType        = (UChar*  )xMalloc(MergeType,   uiNumPartition);
#endif
#if ALF_HM3_QC_REFACTOR
    m_puiAlfCtrlFlag     = (UInt*  )xMalloc(UInt,   uiNumPartition);
#endif
    m_CUTransquantBypass = new Bool[ uiNumPartition ];
    m_pbMergeFlag        = (Bool*  )xMalloc(Bool,   uiNumPartition);
    m_puhMergeIndex      = (UChar* )xMalloc(UChar,  uiNumPartition);
#if QC_FRUC_MERGE
    m_puhFRUCMgrMode      = (UChar*  )xMalloc(UChar,   uiNumPartition);
#endif
    m_puhLumaIntraDir    = (UChar* )xMalloc(UChar,  uiNumPartition);
    m_puhChromaIntraDir  = (UChar* )xMalloc(UChar,  uiNumPartition);
    m_puhInterDir        = (UChar* )xMalloc(UChar,  uiNumPartition);
    
#if QC_EMT
    m_puhEmtTuIdx        = (UChar*)xMalloc(UChar, uiNumPartition);
    m_puhEmtCuFlag       = (UChar*)xMalloc(UChar, uiNumPartition);
#endif

    m_puhTrIdx           = (UChar* )xMalloc(UChar,  uiNumPartition);
    m_puhTransformSkip[0] = (UChar* )xMalloc(UChar,  uiNumPartition);
    m_puhTransformSkip[1] = (UChar* )xMalloc(UChar,  uiNumPartition);
    m_puhTransformSkip[2] = (UChar* )xMalloc(UChar,  uiNumPartition);
#if KLT_COMMON
  m_puhKLTFlag[0]      = (UChar*)xMalloc(UChar, uiNumPartition);
  m_puhKLTFlag[1]      = (UChar*)xMalloc(UChar, uiNumPartition);
  m_puhKLTFlag[2]      = (UChar*)xMalloc(UChar, uiNumPartition);
#endif
    m_puhCbf[0]          = (UChar* )xMalloc(UChar,  uiNumPartition);
    m_puhCbf[1]          = (UChar* )xMalloc(UChar,  uiNumPartition);
    m_puhCbf[2]          = (UChar* )xMalloc(UChar,  uiNumPartition);
    
    m_apiMVPIdx[0]       = new Char[ uiNumPartition ];
    m_apiMVPIdx[1]       = new Char[ uiNumPartition ];
    m_apiMVPNum[0]       = new Char[ uiNumPartition ];
    m_apiMVPNum[1]       = new Char[ uiNumPartition ];
#if !HM14_CLEAN_UP
    memset( m_apiMVPIdx[0], -1,uiNumPartition * sizeof( Char ) );
    memset( m_apiMVPIdx[1], -1,uiNumPartition * sizeof( Char ) );
#endif
    
    m_pcTrCoeffY         = (TCoeff*)xMalloc(TCoeff, uiWidth*uiHeight);
    m_pcTrCoeffCb        = (TCoeff*)xMalloc(TCoeff, uiWidth*uiHeight/4);
    m_pcTrCoeffCr        = (TCoeff*)xMalloc(TCoeff, uiWidth*uiHeight/4);
#if !HM14_CLEAN_UP
    memset( m_pcTrCoeffY, 0,uiWidth*uiHeight * sizeof( TCoeff ) );
    memset( m_pcTrCoeffCb, 0,uiWidth*uiHeight/4 * sizeof( TCoeff ) );
    memset( m_pcTrCoeffCr, 0,uiWidth*uiHeight/4 * sizeof( TCoeff ) );
#endif
#if ADAPTIVE_QP_SELECTION    
    if( bGlobalRMARLBuffer )
    {
      if( m_pcGlbArlCoeffY == NULL )
      {
        m_pcGlbArlCoeffY   = (Int*)xMalloc(Int, uiWidth*uiHeight);
        m_pcGlbArlCoeffCb  = (Int*)xMalloc(Int, uiWidth*uiHeight/4);
        m_pcGlbArlCoeffCr  = (Int*)xMalloc(Int, uiWidth*uiHeight/4);
      }
      m_pcArlCoeffY        = m_pcGlbArlCoeffY;
      m_pcArlCoeffCb       = m_pcGlbArlCoeffCb;
      m_pcArlCoeffCr       = m_pcGlbArlCoeffCr;
      m_ArlCoeffIsAliasedAllocation = true;
    }
    else
    {
      m_pcArlCoeffY        = (Int*)xMalloc(Int, uiWidth*uiHeight);
      m_pcArlCoeffCb       = (Int*)xMalloc(Int, uiWidth*uiHeight/4);
      m_pcArlCoeffCr       = (Int*)xMalloc(Int, uiWidth*uiHeight/4);
    }
#endif
    
    m_pbIPCMFlag         = (Bool*  )xMalloc(Bool, uiNumPartition);
    m_pcIPCMSampleY      = (Pel*   )xMalloc(Pel , uiWidth*uiHeight);
    m_pcIPCMSampleCb     = (Pel*   )xMalloc(Pel , uiWidth*uiHeight/4);
    m_pcIPCMSampleCr     = (Pel*   )xMalloc(Pel , uiWidth*uiHeight/4);

    m_acCUMvField[0].create( uiNumPartition );
    m_acCUMvField[1].create( uiNumPartition );
    
  }
  else
  {
    m_acCUMvField[0].setNumPartition(uiNumPartition );
    m_acCUMvField[1].setNumPartition(uiNumPartition );
  }
  
#if QC_FRUC_MERGE
  m_acFRUCUniLateralMVField[0].create( uiNumPartition );
  m_acFRUCUniLateralMVField[1].create( uiNumPartition );
#endif

  m_sliceStartCU        = (UInt*  )xMalloc(UInt, uiNumPartition);
  m_sliceSegmentStartCU = (UInt*  )xMalloc(UInt, uiNumPartition);
  
  // create pattern memory
  m_pcPattern            = (TComPattern*)xMalloc(TComPattern, 1);
  
  // create motion vector fields
  
  m_pcCUAboveLeft      = NULL;
  m_pcCUAboveRight     = NULL;
  m_pcCUAbove          = NULL;
  m_pcCULeft           = NULL;
  
  m_apcCUColocated[0]  = NULL;
  m_apcCUColocated[1]  = NULL;
}

Void TComDataCU::destroy()
{
  m_pcPic              = NULL;
  m_pcSlice            = NULL;
  
  if ( m_pcPattern )
  { 
    xFree(m_pcPattern);
    m_pcPattern = NULL;
  }
  
  // encoder-side buffer free
  if ( !m_bDecSubCu )
  {
    if ( m_phQP               ) { xFree(m_phQP);                m_phQP              = NULL; }
    if ( m_puhDepth           ) { xFree(m_puhDepth);            m_puhDepth          = NULL; }
    if ( m_puhWidth           ) { xFree(m_puhWidth);            m_puhWidth          = NULL; }
    if ( m_puhHeight          ) { xFree(m_puhHeight);           m_puhHeight         = NULL; }

    if ( m_skipFlag           ) { delete[] m_skipFlag;          m_skipFlag          = NULL; }
#if ROT_TR
    if ( m_ROTIdx           ) { delete[] m_ROTIdx;          m_ROTIdx          = NULL; }
#endif
#if CU_LEVEL_MPI
    if ( m_MPIIdx           ) { delete[] m_MPIIdx;          m_MPIIdx          = NULL; }
#endif
#if QC_IMV
    if ( m_iMVFlag            ) { delete[] m_iMVFlag;           m_iMVFlag           = NULL; }
    if ( m_piMVCandNum        ) { delete[] m_piMVCandNum;       m_piMVCandNum       = NULL; }
#endif
#if QC_OBMC
    if ( m_OBMCFlag           ) { delete[] m_OBMCFlag;          m_OBMCFlag          = NULL; }
#endif
#if QC_IC
    if ( m_pbICFlag           ) { delete [] m_pbICFlag;         m_pbICFlag          = NULL; }
#endif
    if ( m_pePartSize         ) { delete[] m_pePartSize;        m_pePartSize        = NULL; }
    if ( m_pePredMode         ) { delete[] m_pePredMode;        m_pePredMode        = NULL; }
    if ( m_CUTransquantBypass ) { delete[] m_CUTransquantBypass;m_CUTransquantBypass = NULL; }
    if ( m_puhCbf[0]          ) { xFree(m_puhCbf[0]);           m_puhCbf[0]         = NULL; }
    if ( m_puhCbf[1]          ) { xFree(m_puhCbf[1]);           m_puhCbf[1]         = NULL; }
    if ( m_puhCbf[2]          ) { xFree(m_puhCbf[2]);           m_puhCbf[2]         = NULL; }
#if QC_SUB_PU_TMVP
    if ( m_peMergeType        ) { xFree(m_peMergeType );        m_peMergeType      = NULL;  }
#endif
#if ALF_HM3_QC_REFACTOR
    if ( m_puiAlfCtrlFlag     ) { xFree(m_puiAlfCtrlFlag);      m_puiAlfCtrlFlag    = NULL; }
#endif
    if ( m_puhInterDir        ) { xFree(m_puhInterDir);         m_puhInterDir       = NULL; }
    if ( m_pbMergeFlag        ) { xFree(m_pbMergeFlag);         m_pbMergeFlag       = NULL; }
    if ( m_puhMergeIndex      ) { xFree(m_puhMergeIndex);       m_puhMergeIndex     = NULL; }
#if QC_FRUC_MERGE
    if ( m_puhFRUCMgrMode     ) { xFree(m_puhFRUCMgrMode);      m_puhFRUCMgrMode    = NULL; }
#endif
    if ( m_puhLumaIntraDir    ) { xFree(m_puhLumaIntraDir);     m_puhLumaIntraDir   = NULL; }
    if ( m_puhChromaIntraDir  ) { xFree(m_puhChromaIntraDir);   m_puhChromaIntraDir = NULL; }
#if QC_EMT
    if ( m_puhEmtTuIdx        ) { xFree(m_puhEmtTuIdx);         m_puhEmtTuIdx       = NULL; }
    if ( m_puhEmtCuFlag       ) { xFree(m_puhEmtCuFlag);        m_puhEmtCuFlag      = NULL; }
#endif
    if ( m_puhTrIdx           ) { xFree(m_puhTrIdx);            m_puhTrIdx          = NULL; }
    if ( m_puhTransformSkip[0]) { xFree(m_puhTransformSkip[0]); m_puhTransformSkip[0] = NULL; }
    if ( m_puhTransformSkip[1]) { xFree(m_puhTransformSkip[1]); m_puhTransformSkip[1] = NULL; }
    if ( m_puhTransformSkip[2]) { xFree(m_puhTransformSkip[2]); m_puhTransformSkip[2] = NULL; }
#if KLT_COMMON
  if ( m_puhKLTFlag[0]    ) { xFree(m_puhKLTFlag[0]);    m_puhKLTFlag[0]    = NULL; }
  if ( m_puhKLTFlag[1]    ) { xFree(m_puhKLTFlag[1]);    m_puhKLTFlag[1]    = NULL; }
  if ( m_puhKLTFlag[2]    ) { xFree(m_puhKLTFlag[2]);    m_puhKLTFlag[2]    = NULL; }
#endif
    if ( m_pcTrCoeffY         ) { xFree(m_pcTrCoeffY);          m_pcTrCoeffY        = NULL; }
    if ( m_pcTrCoeffCb        ) { xFree(m_pcTrCoeffCb);         m_pcTrCoeffCb       = NULL; }
    if ( m_pcTrCoeffCr        ) { xFree(m_pcTrCoeffCr);         m_pcTrCoeffCr       = NULL; }
#if ADAPTIVE_QP_SELECTION
    if (!m_ArlCoeffIsAliasedAllocation)
    {
      xFree(m_pcArlCoeffY); m_pcArlCoeffY = 0;
      xFree(m_pcArlCoeffCb); m_pcArlCoeffCb = 0;
      xFree(m_pcArlCoeffCr); m_pcArlCoeffCr = 0;
    }
    if ( m_pcGlbArlCoeffY     ) { xFree(m_pcGlbArlCoeffY);      m_pcGlbArlCoeffY    = NULL; }
    if ( m_pcGlbArlCoeffCb    ) { xFree(m_pcGlbArlCoeffCb);     m_pcGlbArlCoeffCb   = NULL; }
    if ( m_pcGlbArlCoeffCr    ) { xFree(m_pcGlbArlCoeffCr);     m_pcGlbArlCoeffCr   = NULL; }
#endif
    if ( m_pbIPCMFlag         ) { xFree(m_pbIPCMFlag   );       m_pbIPCMFlag        = NULL; }
    if ( m_pcIPCMSampleY      ) { xFree(m_pcIPCMSampleY);       m_pcIPCMSampleY     = NULL; }
    if ( m_pcIPCMSampleCb     ) { xFree(m_pcIPCMSampleCb);      m_pcIPCMSampleCb    = NULL; }
    if ( m_pcIPCMSampleCr     ) { xFree(m_pcIPCMSampleCr);      m_pcIPCMSampleCr    = NULL; }
    if ( m_apiMVPIdx[0]       ) { delete[] m_apiMVPIdx[0];      m_apiMVPIdx[0]      = NULL; }
    if ( m_apiMVPIdx[1]       ) { delete[] m_apiMVPIdx[1];      m_apiMVPIdx[1]      = NULL; }
    if ( m_apiMVPNum[0]       ) { delete[] m_apiMVPNum[0];      m_apiMVPNum[0]      = NULL; }
    if ( m_apiMVPNum[1]       ) { delete[] m_apiMVPNum[1];      m_apiMVPNum[1]      = NULL; }
    
    m_acCUMvField[0].destroy();
    m_acCUMvField[1].destroy();
  }
#if QC_FRUC_MERGE
  m_acFRUCUniLateralMVField[0].destroy();
  m_acFRUCUniLateralMVField[1].destroy();
#endif
  
  m_pcCUAboveLeft       = NULL;
  m_pcCUAboveRight      = NULL;
  m_pcCUAbove           = NULL;
  m_pcCULeft            = NULL;
  
  m_apcCUColocated[0]   = NULL;
  m_apcCUColocated[1]   = NULL;

  if( m_sliceStartCU )
  {
    xFree(m_sliceStartCU);
    m_sliceStartCU=NULL;
  }
  if(m_sliceSegmentStartCU )
  {
    xFree(m_sliceSegmentStartCU);
    m_sliceSegmentStartCU=NULL;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// Initialization
// --------------------------------------------------------------------------------------------------------------------

/**
 - initialize top-level CU
 - internal buffers are already created
 - set values before encoding a CU
 .
 \param  pcPic     picture (TComPic) class pointer
 \param  iCUAddr   CU address
 */
Void TComDataCU::initCU( TComPic* pcPic, UInt iCUAddr 
#if HM14_CLEAN_UP
  , Bool bDec
#endif
  )
{

  m_pcPic              = pcPic;
  m_pcSlice            = pcPic->getSlice(pcPic->getCurrSliceIdx());
  m_uiCUAddr           = iCUAddr;
  m_uiCUPelX           = ( iCUAddr % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth;
  m_uiCUPelY           = ( iCUAddr / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight;
  m_uiAbsIdxInLCU      = 0;
  m_dTotalCost         = MAX_DOUBLE;
  m_uiTotalDistortion  = 0;
  m_uiTotalBits        = 0;
  m_uiTotalBins        = 0;
  m_uiNumPartition     = pcPic->getNumPartInCU();
  
  for(Int i=0; i<pcPic->getNumPartInCU(); i++)
  {
    if(pcPic->getPicSym()->getInverseCUOrderMap(iCUAddr)*pcPic->getNumPartInCU()+i>=getSlice()->getSliceCurStartCUAddr())
    {
      m_sliceStartCU[i]=getSlice()->getSliceCurStartCUAddr();
    }
    else
    {
      m_sliceStartCU[i]=pcPic->getCU(getAddr())->m_sliceStartCU[i];
    }
  }
  for(Int i=0; i<pcPic->getNumPartInCU(); i++)
  {
    if(pcPic->getPicSym()->getInverseCUOrderMap(iCUAddr)*pcPic->getNumPartInCU()+i>=getSlice()->getSliceSegmentCurStartCUAddr())
    {
      m_sliceSegmentStartCU[i]=getSlice()->getSliceSegmentCurStartCUAddr();
    }
    else
    {
      m_sliceSegmentStartCU[i]=pcPic->getCU(getAddr())->m_sliceSegmentStartCU[i];
    }
  }

  Int partStartIdx = getSlice()->getSliceSegmentCurStartCUAddr() - pcPic->getPicSym()->getInverseCUOrderMap(iCUAddr) * pcPic->getNumPartInCU();

  Int numElements = min<Int>( partStartIdx, m_uiNumPartition );
  for ( Int ui = 0; ui < numElements; ui++ )
  {
    TComDataCU * pcFrom = pcPic->getCU(getAddr());
    m_skipFlag[ui]   = pcFrom->getSkipFlag(ui);
#if ROT_TR
    m_ROTIdx[ui]   = pcFrom->getROTIdx(ui);
#endif
#if CU_LEVEL_MPI
    m_MPIIdx[ui]   = pcFrom->getMPIIdx(ui);
#endif
#if QC_IMV
    m_iMVFlag[ui]    = pcFrom->getiMVFlag(ui);
    if( !m_bDecSubCu )
    {
      m_piMVCandNum[ui]= pcFrom->getiMVCandNum(ui);
    }
#endif
#if QC_OBMC
    m_OBMCFlag[ui]   = pcFrom->getOBMCFlag(ui);
#endif
#if QC_IC
    m_pbICFlag[ui]   =  pcFrom->m_pbICFlag[ui];
#endif
    m_pePartSize[ui] = pcFrom->getPartitionSize(ui);
    m_pePredMode[ui] = pcFrom->getPredictionMode(ui);
    m_CUTransquantBypass[ui] = pcFrom->getCUTransquantBypass(ui);
    m_puhDepth[ui] = pcFrom->getDepth(ui);
    m_puhWidth  [ui] = pcFrom->getWidth(ui);
    m_puhHeight [ui] = pcFrom->getHeight(ui);
#if QC_EMT
    m_puhEmtTuIdx [ui] = pcFrom->getEmtTuIdx(ui);
    m_puhEmtCuFlag [ui] = pcFrom->getEmtCuFlag(ui);
#endif
    m_puhTrIdx  [ui] = pcFrom->getTransformIdx(ui);
    m_puhTransformSkip[0][ui] = pcFrom->getTransformSkip(ui,TEXT_LUMA);
    m_puhTransformSkip[1][ui] = pcFrom->getTransformSkip(ui,TEXT_CHROMA_U);
    m_puhTransformSkip[2][ui] = pcFrom->getTransformSkip(ui,TEXT_CHROMA_V);
#if KLT_COMMON
  m_puhKLTFlag[0][ui] = pcFrom->getKLTFlag(ui, TEXT_LUMA);
  m_puhKLTFlag[1][ui] = pcFrom->getKLTFlag(ui, TEXT_CHROMA_U);
  m_puhKLTFlag[2][ui] = pcFrom->getKLTFlag(ui, TEXT_CHROMA_V);
#endif
    m_apiMVPIdx[0][ui] = pcFrom->m_apiMVPIdx[0][ui];;
    m_apiMVPIdx[1][ui] = pcFrom->m_apiMVPIdx[1][ui];
    m_apiMVPNum[0][ui] = pcFrom->m_apiMVPNum[0][ui];
    m_apiMVPNum[1][ui] = pcFrom->m_apiMVPNum[1][ui];
    m_phQP[ui]=pcFrom->m_phQP[ui];
    m_pbMergeFlag[ui]=pcFrom->m_pbMergeFlag[ui];
    m_puhMergeIndex[ui]=pcFrom->m_puhMergeIndex[ui];
#if QC_FRUC_MERGE
    m_puhFRUCMgrMode[ui]=pcFrom->m_puhFRUCMgrMode[ui];
#endif
#if QC_SUB_PU_TMVP
    m_peMergeType[ui] =  pcFrom->m_peMergeType[ui];
#endif
    m_puhLumaIntraDir[ui]=pcFrom->m_puhLumaIntraDir[ui];
    m_puhChromaIntraDir[ui]=pcFrom->m_puhChromaIntraDir[ui];
    m_puhInterDir[ui]=pcFrom->m_puhInterDir[ui];
    m_puhCbf[0][ui]=pcFrom->m_puhCbf[0][ui];
    m_puhCbf[1][ui]=pcFrom->m_puhCbf[1][ui];
    m_puhCbf[2][ui]=pcFrom->m_puhCbf[2][ui];
    m_pbIPCMFlag[ui] = pcFrom->m_pbIPCMFlag[ui];
  }
  
  Int firstElement = max<Int>( partStartIdx, 0 );
  numElements = m_uiNumPartition - firstElement;
  
  if ( numElements > 0 )
  {
    memset( m_skipFlag          + firstElement, false,                    numElements * sizeof( *m_skipFlag ) );
#if ROT_TR
    memset( m_ROTIdx          + firstElement, false,                    numElements * sizeof( *m_ROTIdx ) );
#endif
#if CU_LEVEL_MPI
    memset( m_MPIIdx          + firstElement, true,                    numElements * sizeof( *m_MPIIdx ) );
#endif
#if QC_IMV
    memset( m_iMVFlag           + firstElement, false,                      numElements * sizeof( *m_iMVFlag ) );
    if( !m_bDecSubCu )
    {
      memset( m_piMVCandNum       + firstElement, 0,                      numElements * sizeof( *m_piMVCandNum ) );
    }
#endif
#if QC_OBMC
    memset( m_OBMCFlag          + firstElement, true,                    numElements * sizeof( *m_OBMCFlag ) );
#endif
#if QC_IC
    memset( m_pbICFlag          + firstElement, false,                    numElements * sizeof( *m_pbICFlag )   );
#endif
    memset( m_pePartSize        + firstElement, SIZE_NONE,                numElements * sizeof( *m_pePartSize ) );
    memset( m_pePredMode        + firstElement, MODE_NONE,                numElements * sizeof( *m_pePredMode ) );
    memset( m_CUTransquantBypass+ firstElement, false,                    numElements * sizeof( *m_CUTransquantBypass) );
    memset( m_puhDepth          + firstElement, 0,                        numElements * sizeof( *m_puhDepth ) );
#if QC_EMT
    memset( m_puhEmtTuIdx       + firstElement, 0,                        numElements * sizeof( *m_puhEmtTuIdx) );
    memset( m_puhEmtCuFlag      + firstElement, 0,                        numElements * sizeof( *m_puhEmtCuFlag) );
#endif
    memset( m_puhTrIdx          + firstElement, 0,                        numElements * sizeof( *m_puhTrIdx ) );
    memset( m_puhTransformSkip[0] + firstElement, 0,                      numElements * sizeof( *m_puhTransformSkip[0]) );
    memset( m_puhTransformSkip[1] + firstElement, 0,                      numElements * sizeof( *m_puhTransformSkip[1]) );
    memset( m_puhTransformSkip[2] + firstElement, 0,                      numElements * sizeof( *m_puhTransformSkip[2]) );
#if KLT_COMMON
  memset( m_puhKLTFlag[0]     + firstElement, 0,              numElements * sizeof( *m_puhKLTFlag[0] ) );
  memset( m_puhKLTFlag[1]    + firstElement, 0,              numElements * sizeof( *m_puhKLTFlag[1] ) );
  memset( m_puhKLTFlag[2]    + firstElement, 0,              numElements * sizeof( *m_puhKLTFlag[2] ) );
#endif
#if QC_LARGE_CTU
    for( Int n = 0 ; n < numElements ; n++ )
    {
      m_puhWidth[firstElement+n] = g_uiMaxCUWidth;
      m_puhHeight[firstElement+n] = g_uiMaxCUHeight;
    }
#else
    memset( m_puhWidth          + firstElement, g_uiMaxCUWidth,           numElements * sizeof( *m_puhWidth ) );
    memset( m_puhHeight         + firstElement, g_uiMaxCUHeight,          numElements * sizeof( *m_puhHeight ) );
#endif
    memset( m_apiMVPIdx[0]      + firstElement, -1,                       numElements * sizeof( *m_apiMVPIdx[0] ) );
    memset( m_apiMVPIdx[1]      + firstElement, -1,                       numElements * sizeof( *m_apiMVPIdx[1] ) );
    memset( m_apiMVPNum[0]      + firstElement, -1,                       numElements * sizeof( *m_apiMVPNum[0] ) );
    memset( m_apiMVPNum[1]      + firstElement, -1,                       numElements * sizeof( *m_apiMVPNum[1] ) );
    memset( m_phQP              + firstElement, getSlice()->getSliceQp(), numElements * sizeof( *m_phQP ) );
#if QC_SUB_PU_TMVP
    memset( m_peMergeType       + firstElement, 0,                        numElements * sizeof( *m_peMergeType  ) );
#endif
#if ALF_HM3_QC_REFACTOR
    memset( m_puiAlfCtrlFlag    + firstElement, 0,                        numElements * sizeof( *m_puiAlfCtrlFlag ) );
#endif
    memset( m_pbMergeFlag       + firstElement, false,                    numElements * sizeof( *m_pbMergeFlag ) );
    memset( m_puhMergeIndex     + firstElement, 0,                        numElements * sizeof( *m_puhMergeIndex ) );
#if QC_FRUC_MERGE
    memset( m_puhFRUCMgrMode    + firstElement, 0,                    numElements * sizeof( *m_puhFRUCMgrMode ) );
#endif
    memset( m_puhLumaIntraDir   + firstElement, DC_IDX,                   numElements * sizeof( *m_puhLumaIntraDir ) );
    memset( m_puhChromaIntraDir + firstElement, 0,                        numElements * sizeof( *m_puhChromaIntraDir ) );
    memset( m_puhInterDir       + firstElement, 0,                        numElements * sizeof( *m_puhInterDir ) );
    memset( m_puhCbf[0]         + firstElement, 0,                        numElements * sizeof( *m_puhCbf[0] ) );
    memset( m_puhCbf[1]         + firstElement, 0,                        numElements * sizeof( *m_puhCbf[1] ) );
    memset( m_puhCbf[2]         + firstElement, 0,                        numElements * sizeof( *m_puhCbf[2] ) );
    memset( m_pbIPCMFlag        + firstElement, false,                    numElements * sizeof( *m_pbIPCMFlag ) );
  }
  
  UInt uiTmp = g_uiMaxCUWidth*g_uiMaxCUHeight;
  if ( 0 >= partStartIdx ) 
  {
    m_acCUMvField[0].clearMvField();
    m_acCUMvField[1].clearMvField();
#if HM14_CLEAN_UP
    if( bDec == false )
    {
#endif
    memset( m_pcTrCoeffY , 0, sizeof( TCoeff ) * uiTmp );
#if ADAPTIVE_QP_SELECTION
    memset( m_pcArlCoeffY , 0, sizeof( Int ) * uiTmp );  
#endif
    memset( m_pcIPCMSampleY , 0, sizeof( Pel ) * uiTmp );
    uiTmp  >>= 2;
    memset( m_pcTrCoeffCb, 0, sizeof( TCoeff ) * uiTmp );
    memset( m_pcTrCoeffCr, 0, sizeof( TCoeff ) * uiTmp );
#if ADAPTIVE_QP_SELECTION  
    memset( m_pcArlCoeffCb, 0, sizeof( Int ) * uiTmp );
    memset( m_pcArlCoeffCr, 0, sizeof( Int ) * uiTmp );
#endif
    memset( m_pcIPCMSampleCb , 0, sizeof( Pel ) * uiTmp );
    memset( m_pcIPCMSampleCr , 0, sizeof( Pel ) * uiTmp );
#if HM14_CLEAN_UP
    }
#endif
  }
  else 
  {
    TComDataCU * pcFrom = pcPic->getCU(getAddr());
    m_acCUMvField[0].copyFrom(&pcFrom->m_acCUMvField[0],m_uiNumPartition,0);
    m_acCUMvField[1].copyFrom(&pcFrom->m_acCUMvField[1],m_uiNumPartition,0);
    for(Int i=0; i<uiTmp; i++)
    {
      m_pcTrCoeffY[i]=pcFrom->m_pcTrCoeffY[i];
#if ADAPTIVE_QP_SELECTION
      m_pcArlCoeffY[i]=pcFrom->m_pcArlCoeffY[i];
#endif
      m_pcIPCMSampleY[i]=pcFrom->m_pcIPCMSampleY[i];
    }
    for(Int i=0; i<(uiTmp>>2); i++)
    {
      m_pcTrCoeffCb[i]=pcFrom->m_pcTrCoeffCb[i];
      m_pcTrCoeffCr[i]=pcFrom->m_pcTrCoeffCr[i];
#if ADAPTIVE_QP_SELECTION
      m_pcArlCoeffCb[i]=pcFrom->m_pcArlCoeffCb[i];
      m_pcArlCoeffCr[i]=pcFrom->m_pcArlCoeffCr[i];
#endif
      m_pcIPCMSampleCb[i]=pcFrom->m_pcIPCMSampleCb[i];
      m_pcIPCMSampleCr[i]=pcFrom->m_pcIPCMSampleCr[i];
    }
  }

  // Setting neighbor CU
  m_pcCULeft        = NULL;
  m_pcCUAbove       = NULL;
  m_pcCUAboveLeft   = NULL;
  m_pcCUAboveRight  = NULL;

  m_apcCUColocated[0] = NULL;
  m_apcCUColocated[1] = NULL;

  UInt uiWidthInCU = pcPic->getFrameWidthInCU();
  if ( m_uiCUAddr % uiWidthInCU )
  {
    m_pcCULeft = pcPic->getCU( m_uiCUAddr - 1 );
  }

  if ( m_uiCUAddr / uiWidthInCU )
  {
    m_pcCUAbove = pcPic->getCU( m_uiCUAddr - uiWidthInCU );
  }

  if ( m_pcCULeft && m_pcCUAbove )
  {
    m_pcCUAboveLeft = pcPic->getCU( m_uiCUAddr - uiWidthInCU - 1 );
  }

  if ( m_pcCUAbove && ( (m_uiCUAddr%uiWidthInCU) < (uiWidthInCU-1) )  )
  {
    m_pcCUAboveRight = pcPic->getCU( m_uiCUAddr - uiWidthInCU + 1 );
  }

  if ( getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 )
  {
    m_apcCUColocated[0] = getSlice()->getRefPic( REF_PIC_LIST_0, 0)->getCU( m_uiCUAddr );
  }

  if ( getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 0 )
  {
    m_apcCUColocated[1] = getSlice()->getRefPic( REF_PIC_LIST_1, 0)->getCU( m_uiCUAddr );
  }
}

/** initialize prediction data with enabling sub-LCU-level delta QP
*\param  uiDepth  depth of the current CU
*\param  qp     qp for the current CU
*- set CU width and CU height according to depth
*- set qp value according to input qp 
*- set last-coded qp value according to input last-coded qp 
*/
Void TComDataCU::initEstData( UInt uiDepth, Int qp, Bool bTransquantBypass )
{
  m_dTotalCost         = MAX_DOUBLE;
  m_uiTotalDistortion  = 0;
  m_uiTotalBits        = 0;
  m_uiTotalBins        = 0;

#if QC_LARGE_CTU
  UShort uhWidth  = g_uiMaxCUWidth  >> uiDepth;
  UShort uhHeight = g_uiMaxCUHeight >> uiDepth;
#else
  UChar uhWidth  = g_uiMaxCUWidth  >> uiDepth;
  UChar uhHeight = g_uiMaxCUHeight >> uiDepth;
#endif

  for (UInt ui = 0; ui < m_uiNumPartition; ui++)
  {
    if(getPic()->getPicSym()->getInverseCUOrderMap(getAddr())*m_pcPic->getNumPartInCU()+m_uiAbsIdxInLCU+ui >= getSlice()->getSliceSegmentCurStartCUAddr())
    {
      m_apiMVPIdx[0][ui] = -1;
      m_apiMVPIdx[1][ui] = -1;
      m_apiMVPNum[0][ui] = -1;
      m_apiMVPNum[1][ui] = -1;
      m_puhDepth  [ui] = uiDepth;
      m_puhWidth  [ui] = uhWidth;
      m_puhHeight [ui] = uhHeight;
#if QC_EMT
      m_puhEmtTuIdx [ui] = 0;
      m_puhEmtCuFlag[ui] = 0;
#endif
      m_puhTrIdx  [ui] = 0;
      m_puhTransformSkip[0][ui] = 0;
      m_puhTransformSkip[1][ui] = 0;
      m_puhTransformSkip[2][ui] = 0;
#if KLT_COMMON
    m_puhKLTFlag[0][ui] = 0;
    m_puhKLTFlag[1][ui] = 0;
    m_puhKLTFlag[2][ui] = 0;
#endif
      m_skipFlag[ui]   = false;
#if ROT_TR
      m_ROTIdx[ui]   = false;
#endif
#if CU_LEVEL_MPI
    m_MPIIdx[ui]   = 0;
#endif
#if QC_IMV
      m_iMVFlag[ui]       = false;
      if( !m_bDecSubCu )
      {
        m_piMVCandNum[ui]   = 0;
      }
#endif
#if QC_OBMC
      m_OBMCFlag[ui]   = true;
#endif
#if QC_IC
      m_pbICFlag[ui]  = false;
#endif
      m_pePartSize[ui] = SIZE_NONE;
      m_pePredMode[ui] = MODE_NONE;
      m_CUTransquantBypass[ui] = bTransquantBypass;
      m_pbIPCMFlag[ui] = 0;
      m_phQP[ui] = qp;
#if ALF_HM3_QC_REFACTOR
      m_puiAlfCtrlFlag[ui] = 0;
#endif
      m_pbMergeFlag[ui] = 0;
      m_puhMergeIndex[ui] = 0;
#if QC_FRUC_MERGE
      m_puhFRUCMgrMode[ui] = 0;
#endif
#if QC_SUB_PU_TMVP
      m_peMergeType[ui]   = MGR_TYPE_DEFAULT_N;
#endif
      m_puhLumaIntraDir[ui] = DC_IDX;
      m_puhChromaIntraDir[ui] = 0;
      m_puhInterDir[ui] = 0;
      m_puhCbf[0][ui] = 0;
      m_puhCbf[1][ui] = 0;
      m_puhCbf[2][ui] = 0;
    }
  }

  if(getPic()->getPicSym()->getInverseCUOrderMap(getAddr())*m_pcPic->getNumPartInCU()+m_uiAbsIdxInLCU >= getSlice()->getSliceSegmentCurStartCUAddr())
  {
    m_acCUMvField[0].clearMvField();
    m_acCUMvField[1].clearMvField();
    UInt uiTmp = uhWidth*uhHeight;
    
    memset( m_pcTrCoeffY,    0, uiTmp * sizeof( *m_pcTrCoeffY    ) );
#if ADAPTIVE_QP_SELECTION
    memset( m_pcArlCoeffY ,  0, uiTmp * sizeof( *m_pcArlCoeffY   ) );
#endif
    memset( m_pcIPCMSampleY, 0, uiTmp * sizeof( *m_pcIPCMSampleY ) );

    uiTmp>>=2;
    memset( m_pcTrCoeffCb,    0, uiTmp * sizeof( *m_pcTrCoeffCb    ) );
    memset( m_pcTrCoeffCr,    0, uiTmp * sizeof( *m_pcTrCoeffCr    ) );
#if ADAPTIVE_QP_SELECTION  
    memset( m_pcArlCoeffCb,   0, uiTmp * sizeof( *m_pcArlCoeffCb   ) );
    memset( m_pcArlCoeffCr,   0, uiTmp * sizeof( *m_pcArlCoeffCr   ) );
#endif
    memset( m_pcIPCMSampleCb, 0, uiTmp * sizeof( *m_pcIPCMSampleCb ) );
    memset( m_pcIPCMSampleCr, 0, uiTmp * sizeof( *m_pcIPCMSampleCr ) );
  }
}


// initialize Sub partition
Void TComDataCU::initSubCU( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth, Int qp )
{
  assert( uiPartUnitIdx<4 );

  UInt uiPartOffset = ( pcCU->getTotalNumPart()>>2 )*uiPartUnitIdx;

  m_pcPic              = pcCU->getPic();
  m_pcSlice            = m_pcPic->getSlice(m_pcPic->getCurrSliceIdx());
  m_uiCUAddr           = pcCU->getAddr();
  m_uiAbsIdxInLCU      = pcCU->getZorderIdxInCU() + uiPartOffset;

  m_uiCUPelX           = pcCU->getCUPelX() + ( g_uiMaxCUWidth>>uiDepth  )*( uiPartUnitIdx &  1 );
  m_uiCUPelY           = pcCU->getCUPelY() + ( g_uiMaxCUHeight>>uiDepth  )*( uiPartUnitIdx >> 1 );

  m_dTotalCost         = MAX_DOUBLE;
  m_uiTotalDistortion  = 0;
  m_uiTotalBits        = 0;
  m_uiTotalBins        = 0;
  m_uiNumPartition     = pcCU->getTotalNumPart() >> 2;

  Int iSizeInUchar = sizeof( UChar  ) * m_uiNumPartition;
  Int iSizeInBool  = sizeof( Bool   ) * m_uiNumPartition;

  Int sizeInChar = sizeof( Char  ) * m_uiNumPartition;
  memset( m_phQP,              qp,  sizeInChar );

#if ALF_HM3_QC_REFACTOR
  memset( m_puiAlfCtrlFlag,     0, sizeof( UInt   ) * m_uiNumPartition );
#endif
  memset( m_pbMergeFlag,        0, iSizeInBool  );
  memset( m_puhMergeIndex,      0, iSizeInUchar );
#if QC_FRUC_MERGE
  memset( m_puhFRUCMgrMode,     0, iSizeInUchar );
#endif
#if QC_SUB_PU_TMVP
  memset( m_peMergeType,        0, sizeof( UChar  ) * m_uiNumPartition );  
#endif
  memset( m_puhLumaIntraDir,    DC_IDX, iSizeInUchar );
  memset( m_puhChromaIntraDir,  0, iSizeInUchar );
  memset( m_puhInterDir,        0, iSizeInUchar );
#if QC_EMT
  memset( m_puhEmtTuIdx,        0, iSizeInUchar );
  memset( m_puhEmtCuFlag,       0, iSizeInUchar );
#endif
  memset( m_puhTrIdx,           0, iSizeInUchar );
  memset( m_puhTransformSkip[0], 0, iSizeInUchar );
  memset( m_puhTransformSkip[1], 0, iSizeInUchar );
  memset( m_puhTransformSkip[2], 0, iSizeInUchar );
#if KLT_COMMON
  memset( m_puhKLTFlag[0],    0, iSizeInUchar);
  memset( m_puhKLTFlag[1],    0, iSizeInUchar);
  memset( m_puhKLTFlag[2],    0, iSizeInUchar);
#endif
  memset( m_puhCbf[0],          0, iSizeInUchar );
  memset( m_puhCbf[1],          0, iSizeInUchar );
  memset( m_puhCbf[2],          0, iSizeInUchar );
  memset( m_puhDepth,     uiDepth, iSizeInUchar );

#if QC_LARGE_CTU
  UShort uhWidth  = g_uiMaxCUWidth  >> uiDepth;
  UShort uhHeight = g_uiMaxCUHeight >> uiDepth;
  for( Int n = 0 ; n < iSizeInUchar ; n++ )
  {
    m_puhWidth[n] = uhWidth;
    m_puhHeight[n] = uhHeight;
  }
#else
  UChar uhWidth  = g_uiMaxCUWidth  >> uiDepth;
  UChar uhHeight = g_uiMaxCUHeight >> uiDepth;
  memset( m_puhWidth,          uhWidth,  iSizeInUchar );
  memset( m_puhHeight,         uhHeight, iSizeInUchar );
#endif
  memset( m_pbIPCMFlag,        0, iSizeInBool  );
  for (UInt ui = 0; ui < m_uiNumPartition; ui++)
  {
    m_skipFlag[ui]   = false;
#if ROT_TR
    m_ROTIdx[ui]   = false;
#endif
#if CU_LEVEL_MPI
    m_MPIIdx[ui]   = false;
#endif
#if QC_IMV
    m_iMVFlag[ui]    = false;
    if( !m_bDecSubCu )
    {
      m_piMVCandNum[ui]= 0;
    }
#endif
#if QC_OBMC
    m_OBMCFlag[ui]   = true;
#endif
#if QC_IC
    m_pbICFlag[ui]   = false;
#endif
    m_pePartSize[ui] = SIZE_NONE;
    m_pePredMode[ui] = MODE_NONE;
    m_CUTransquantBypass[ui] = false;
    m_apiMVPIdx[0][ui] = -1;
    m_apiMVPIdx[1][ui] = -1;
    m_apiMVPNum[0][ui] = -1;
    m_apiMVPNum[1][ui] = -1;
    if(m_pcPic->getPicSym()->getInverseCUOrderMap(getAddr())*m_pcPic->getNumPartInCU()+m_uiAbsIdxInLCU+ui<getSlice()->getSliceSegmentCurStartCUAddr())
    {
      m_apiMVPIdx[0][ui] = pcCU->m_apiMVPIdx[0][uiPartOffset+ui];
      m_apiMVPIdx[1][ui] = pcCU->m_apiMVPIdx[1][uiPartOffset+ui];;
      m_apiMVPNum[0][ui] = pcCU->m_apiMVPNum[0][uiPartOffset+ui];;
      m_apiMVPNum[1][ui] = pcCU->m_apiMVPNum[1][uiPartOffset+ui];;
      m_puhDepth  [ui] = pcCU->getDepth(uiPartOffset+ui);
      m_puhWidth  [ui] = pcCU->getWidth(uiPartOffset+ui);
      m_puhHeight  [ui] = pcCU->getHeight(uiPartOffset+ui);
#if QC_EMT
      m_puhEmtTuIdx [ui] = pcCU->getEmtTuIdx(uiPartOffset+ui);
      m_puhEmtCuFlag[ui] = pcCU->getEmtCuFlag(uiPartOffset+ui);
#endif
      m_puhTrIdx  [ui] = pcCU->getTransformIdx(uiPartOffset+ui);
      m_puhTransformSkip[0][ui] = pcCU->getTransformSkip(uiPartOffset+ui,TEXT_LUMA);
      m_puhTransformSkip[1][ui] = pcCU->getTransformSkip(uiPartOffset+ui,TEXT_CHROMA_U);
      m_puhTransformSkip[2][ui] = pcCU->getTransformSkip(uiPartOffset+ui,TEXT_CHROMA_V);
#if KLT_COMMON
    m_puhKLTFlag[0][ui] = pcCU->getKLTFlag(uiPartOffset + ui, TEXT_LUMA);
    m_puhKLTFlag[1][ui] = pcCU->getKLTFlag(uiPartOffset + ui, TEXT_CHROMA_U);
    m_puhKLTFlag[2][ui] = pcCU->getKLTFlag(uiPartOffset + ui, TEXT_CHROMA_V);
#endif
      m_skipFlag[ui]   = pcCU->getSkipFlag(uiPartOffset+ui);
#if ROT_TR
      m_ROTIdx[ui]   = pcCU->getROTIdx(uiPartOffset+ui);
#endif
#if CU_LEVEL_MPI
      m_MPIIdx[ui]   = pcCU->getMPIIdx(uiPartOffset+ui);
#endif
#if QC_IMV
      m_iMVFlag[ui]    = pcCU->getiMVFlag(uiPartOffset+ui);
      if( !m_bDecSubCu )
      {
        m_piMVCandNum[ui] = pcCU->getiMVCandNum( uiPartOffset + ui );
      }
#endif
#if QC_OBMC
      m_OBMCFlag[ui]   = pcCU->getOBMCFlag(uiPartOffset+ui);
#endif
#if QC_IC
      m_pbICFlag[ui]   = pcCU->m_pbICFlag[uiPartOffset+ui];
#endif
      m_pePartSize[ui] = pcCU->getPartitionSize(uiPartOffset+ui);
      m_pePredMode[ui] = pcCU->getPredictionMode(uiPartOffset+ui);
      m_CUTransquantBypass[ui] = pcCU->getCUTransquantBypass(uiPartOffset+ui);
      m_pbIPCMFlag[ui]=pcCU->m_pbIPCMFlag[uiPartOffset+ui];
      m_phQP[ui] = pcCU->m_phQP[uiPartOffset+ui];
      m_pbMergeFlag[ui]=pcCU->m_pbMergeFlag[uiPartOffset+ui];
      m_puhMergeIndex[ui]=pcCU->m_puhMergeIndex[uiPartOffset+ui];
#if QC_FRUC_MERGE
      m_puhFRUCMgrMode[ui]=pcCU->m_puhFRUCMgrMode[uiPartOffset+ui];
#endif
#if QC_SUB_PU_TMVP
      m_peMergeType[ui]=pcCU->m_peMergeType[uiPartOffset+ui];
#endif
      m_puhLumaIntraDir[ui]=pcCU->m_puhLumaIntraDir[uiPartOffset+ui];
      m_puhChromaIntraDir[ui]=pcCU->m_puhChromaIntraDir[uiPartOffset+ui];
      m_puhInterDir[ui]=pcCU->m_puhInterDir[uiPartOffset+ui];
      m_puhCbf[0][ui]=pcCU->m_puhCbf[0][uiPartOffset+ui];
      m_puhCbf[1][ui]=pcCU->m_puhCbf[1][uiPartOffset+ui];
      m_puhCbf[2][ui]=pcCU->m_puhCbf[2][uiPartOffset+ui];

    }
  }
  UInt uiTmp = uhWidth*uhHeight;
  memset( m_pcTrCoeffY , 0, sizeof(TCoeff)*uiTmp );
#if ADAPTIVE_QP_SELECTION  
  memset( m_pcArlCoeffY , 0, sizeof(Int)*uiTmp );
#endif
  memset( m_pcIPCMSampleY , 0, sizeof( Pel ) * uiTmp );
  uiTmp >>= 2;
  memset( m_pcTrCoeffCb, 0, sizeof(TCoeff)*uiTmp );
  memset( m_pcTrCoeffCr, 0, sizeof(TCoeff)*uiTmp );
#if ADAPTIVE_QP_SELECTION
  memset( m_pcArlCoeffCb, 0, sizeof(Int)*uiTmp );
  memset( m_pcArlCoeffCr, 0, sizeof(Int)*uiTmp );
#endif
  memset( m_pcIPCMSampleCb , 0, sizeof( Pel ) * uiTmp );
  memset( m_pcIPCMSampleCr , 0, sizeof( Pel ) * uiTmp );
  m_acCUMvField[0].clearMvField();
  m_acCUMvField[1].clearMvField();

  if(m_pcPic->getPicSym()->getInverseCUOrderMap(getAddr())*m_pcPic->getNumPartInCU()+m_uiAbsIdxInLCU<getSlice()->getSliceSegmentCurStartCUAddr())
  {
    // Part of this CU contains data from an older slice. Now copy in that data.
    UInt uiMaxCuWidth=pcCU->getSlice()->getSPS()->getMaxCUWidth();
    UInt uiMaxCuHeight=pcCU->getSlice()->getSPS()->getMaxCUHeight();
    TComDataCU * bigCU = getPic()->getCU(getAddr());
    Int minui = uiPartOffset;
    minui = -minui;
    pcCU->m_acCUMvField[0].copyTo(&m_acCUMvField[0],minui,uiPartOffset,m_uiNumPartition);
    pcCU->m_acCUMvField[1].copyTo(&m_acCUMvField[1],minui,uiPartOffset,m_uiNumPartition);
    UInt uiCoffOffset = uiMaxCuWidth*uiMaxCuHeight*m_uiAbsIdxInLCU/pcCU->getPic()->getNumPartInCU();
    uiTmp = uhWidth*uhHeight;
    for(Int i=0; i<uiTmp; i++)
    {
      m_pcTrCoeffY[i]=bigCU->m_pcTrCoeffY[uiCoffOffset+i];
#if ADAPTIVE_QP_SELECTION
      m_pcArlCoeffY[i]=bigCU->m_pcArlCoeffY[uiCoffOffset+i];
#endif
      m_pcIPCMSampleY[i]=bigCU->m_pcIPCMSampleY[uiCoffOffset+i];
    }
    uiTmp>>=2;
    uiCoffOffset>>=2;
    for(Int i=0; i<uiTmp; i++)
    {
      m_pcTrCoeffCr[i]=bigCU->m_pcTrCoeffCr[uiCoffOffset+i];
      m_pcTrCoeffCb[i]=bigCU->m_pcTrCoeffCb[uiCoffOffset+i];
#if ADAPTIVE_QP_SELECTION
      m_pcArlCoeffCr[i]=bigCU->m_pcArlCoeffCr[uiCoffOffset+i];
      m_pcArlCoeffCb[i]=bigCU->m_pcArlCoeffCb[uiCoffOffset+i];
#endif
      m_pcIPCMSampleCb[i]=bigCU->m_pcIPCMSampleCb[uiCoffOffset+i];
      m_pcIPCMSampleCr[i]=bigCU->m_pcIPCMSampleCr[uiCoffOffset+i];
    }
  }

  m_pcCULeft        = pcCU->getCULeft();
  m_pcCUAbove       = pcCU->getCUAbove();
  m_pcCUAboveLeft   = pcCU->getCUAboveLeft();
  m_pcCUAboveRight  = pcCU->getCUAboveRight();

  m_apcCUColocated[0] = pcCU->getCUColocated(REF_PIC_LIST_0);
  m_apcCUColocated[1] = pcCU->getCUColocated(REF_PIC_LIST_1);
  memcpy(m_sliceStartCU,pcCU->m_sliceStartCU+uiPartOffset,sizeof(UInt)*m_uiNumPartition);
  memcpy(m_sliceSegmentStartCU,pcCU->m_sliceSegmentStartCU+uiPartOffset,sizeof(UInt)*m_uiNumPartition);
}

Void TComDataCU::setOutsideCUPart( UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiNumPartition = m_uiNumPartition >> (uiDepth << 1);
  UInt uiSizeInUchar = sizeof( UChar  ) * uiNumPartition;
#if QC_LARGE_CTU
  UShort uhWidth  = g_uiMaxCUWidth  >> uiDepth;
  UShort uhHeight = g_uiMaxCUHeight >> uiDepth;
  memset( m_puhDepth    + uiAbsPartIdx,     uiDepth,  uiSizeInUchar );
  for( UInt n = 0 ; n < uiSizeInUchar ; n++ )
  {
    m_puhWidth[uiAbsPartIdx+n] = uhWidth;
    m_puhHeight[uiAbsPartIdx+n] = uhHeight;
  }
#else
  UChar uhWidth  = g_uiMaxCUWidth  >> uiDepth;
  UChar uhHeight = g_uiMaxCUHeight >> uiDepth;
  memset( m_puhDepth    + uiAbsPartIdx,     uiDepth,  uiSizeInUchar );
  memset( m_puhWidth    + uiAbsPartIdx,     uhWidth,  uiSizeInUchar );
  memset( m_puhHeight   + uiAbsPartIdx,     uhHeight, uiSizeInUchar );
#endif
}

// --------------------------------------------------------------------------------------------------------------------
// Copy
// --------------------------------------------------------------------------------------------------------------------

Void TComDataCU::copySubCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiPart = uiAbsPartIdx;
  
  m_pcPic              = pcCU->getPic();
  m_pcSlice            = pcCU->getSlice();
  m_uiCUAddr           = pcCU->getAddr();
  m_uiAbsIdxInLCU      = uiAbsPartIdx;
  
  m_uiCUPelX           = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  m_uiCUPelY           = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  
  m_skipFlag=pcCU->getSkipFlag()          + uiPart;
#if ROT_TR
  m_ROTIdx=pcCU->getROTIdx()          + uiPart;
#endif
#if CU_LEVEL_MPI
  m_MPIIdx=pcCU->getMPIIdx()          + uiPart;
#endif
#if QC_IMV
  m_iMVFlag=pcCU->getiMVFlag()            + uiPart;
  if( !m_bDecSubCu )
  {
    m_piMVCandNum = pcCU->getiMVCandNum()   + uiPart;
  }
#endif
#if QC_OBMC
  m_OBMCFlag=pcCU->getOBMCFlag()          + uiPart;
#endif
#if QC_IC
  m_pbICFlag            = pcCU->getICFlag() + uiPart;
#endif
  m_phQP=pcCU->getQP()                    + uiPart;
  m_pePartSize = pcCU->getPartitionSize() + uiPart;
  m_pePredMode=pcCU->getPredictionMode()  + uiPart;
  m_CUTransquantBypass  = pcCU->getCUTransquantBypass()+uiPart;
  
  m_pbMergeFlag         = pcCU->getMergeFlag()        + uiPart;
  m_puhMergeIndex       = pcCU->getMergeIndex()       + uiPart;
#if QC_FRUC_MERGE
  m_puhFRUCMgrMode      = pcCU->getFRUCMgrMode()      + uiPart;
#endif

#if QC_SUB_PU_TMVP
  m_peMergeType         = pcCU->getMergeType()        + uiPart;
#endif

  m_puhLumaIntraDir     = pcCU->getLumaIntraDir()     + uiPart;
  m_puhChromaIntraDir   = pcCU->getChromaIntraDir()   + uiPart;
  m_puhInterDir         = pcCU->getInterDir()         + uiPart;
#if QC_EMT
  m_puhEmtTuIdx         = pcCU->getEmtTuIdx()         + uiPart;
  m_puhEmtCuFlag        = pcCU->getEmtCuFlag()        + uiPart;
#endif
  m_puhTrIdx            = pcCU->getTransformIdx()     + uiPart;
  m_puhTransformSkip[0] = pcCU->getTransformSkip(TEXT_LUMA)     + uiPart;
  m_puhTransformSkip[1] = pcCU->getTransformSkip(TEXT_CHROMA_U) + uiPart;
  m_puhTransformSkip[2] = pcCU->getTransformSkip(TEXT_CHROMA_V) + uiPart;
#if KLT_COMMON
  m_puhKLTFlag[0]    = pcCU->getKLTFlag(TEXT_LUMA) + uiPart;
  m_puhKLTFlag[1]    = pcCU->getKLTFlag(TEXT_CHROMA_U) + uiPart;
  m_puhKLTFlag[2]    = pcCU->getKLTFlag(TEXT_CHROMA_V) + uiPart;
#endif
  m_puhCbf[0]= pcCU->getCbf(TEXT_LUMA)            + uiPart;
  m_puhCbf[1]= pcCU->getCbf(TEXT_CHROMA_U)        + uiPart;
  m_puhCbf[2]= pcCU->getCbf(TEXT_CHROMA_V)        + uiPart;
  
  m_puhDepth=pcCU->getDepth()                     + uiPart;
  m_puhWidth=pcCU->getWidth()                     + uiPart;
  m_puhHeight=pcCU->getHeight()                   + uiPart;
  
  m_apiMVPIdx[0]=pcCU->getMVPIdx(REF_PIC_LIST_0)  + uiPart;
  m_apiMVPIdx[1]=pcCU->getMVPIdx(REF_PIC_LIST_1)  + uiPart;
  m_apiMVPNum[0]=pcCU->getMVPNum(REF_PIC_LIST_0)  + uiPart;
  m_apiMVPNum[1]=pcCU->getMVPNum(REF_PIC_LIST_1)  + uiPart;
  
  m_pbIPCMFlag         = pcCU->getIPCMFlag()        + uiPart;

  m_pcCUAboveLeft      = pcCU->getCUAboveLeft();
  m_pcCUAboveRight     = pcCU->getCUAboveRight();
  m_pcCUAbove          = pcCU->getCUAbove();
  m_pcCULeft           = pcCU->getCULeft();
  
  m_apcCUColocated[0] = pcCU->getCUColocated(REF_PIC_LIST_0);
  m_apcCUColocated[1] = pcCU->getCUColocated(REF_PIC_LIST_1);
  
  UInt uiMaxCuWidth=pcCU->getSlice()->getSPS()->getMaxCUWidth();
  UInt uiMaxCuHeight=pcCU->getSlice()->getSPS()->getMaxCUHeight();
  
  UInt uiCoffOffset = uiMaxCuWidth*uiMaxCuHeight*uiAbsPartIdx/pcCU->getPic()->getNumPartInCU();
  
  m_pcTrCoeffY = pcCU->getCoeffY() + uiCoffOffset;
#if ADAPTIVE_QP_SELECTION
  m_pcArlCoeffY= pcCU->getArlCoeffY() + uiCoffOffset;  
#endif
  m_pcIPCMSampleY = pcCU->getPCMSampleY() + uiCoffOffset;

  uiCoffOffset >>=2;
  m_pcTrCoeffCb=pcCU->getCoeffCb() + uiCoffOffset;
  m_pcTrCoeffCr=pcCU->getCoeffCr() + uiCoffOffset;
#if ADAPTIVE_QP_SELECTION  
  m_pcArlCoeffCb=pcCU->getArlCoeffCb() + uiCoffOffset;
  m_pcArlCoeffCr=pcCU->getArlCoeffCr() + uiCoffOffset;
#endif
  m_pcIPCMSampleCb = pcCU->getPCMSampleCb() + uiCoffOffset;
  m_pcIPCMSampleCr = pcCU->getPCMSampleCr() + uiCoffOffset;

  m_acCUMvField[0].linkToWithOffset( pcCU->getCUMvField(REF_PIC_LIST_0), uiPart );
  m_acCUMvField[1].linkToWithOffset( pcCU->getCUMvField(REF_PIC_LIST_1), uiPart );
  memcpy(m_sliceStartCU,pcCU->m_sliceStartCU+uiPart,sizeof(UInt)*m_uiNumPartition);
  memcpy(m_sliceSegmentStartCU,pcCU->m_sliceSegmentStartCU+uiPart,sizeof(UInt)*m_uiNumPartition);
}

// Copy inter prediction info from the biggest CU
Void TComDataCU::copyInterPredInfoFrom    ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList )
{
  m_pcPic              = pcCU->getPic();
  m_pcSlice            = pcCU->getSlice();
  m_uiCUAddr           = pcCU->getAddr();
  m_uiAbsIdxInLCU      = uiAbsPartIdx;
  
  Int iRastPartIdx     = g_auiZscanToRaster[uiAbsPartIdx];
  m_uiCUPelX           = pcCU->getCUPelX() + m_pcPic->getMinCUWidth ()*( iRastPartIdx % m_pcPic->getNumPartInWidth() );
  m_uiCUPelY           = pcCU->getCUPelY() + m_pcPic->getMinCUHeight()*( iRastPartIdx / m_pcPic->getNumPartInWidth() );
  
  m_pcCUAboveLeft      = pcCU->getCUAboveLeft();
  m_pcCUAboveRight     = pcCU->getCUAboveRight();
  m_pcCUAbove          = pcCU->getCUAbove();
  m_pcCULeft           = pcCU->getCULeft();
  
  m_apcCUColocated[0]  = pcCU->getCUColocated(REF_PIC_LIST_0);
  m_apcCUColocated[1]  = pcCU->getCUColocated(REF_PIC_LIST_1);
  
  m_skipFlag           = pcCU->getSkipFlag ()             + uiAbsPartIdx;
#if ROT_TR
  m_ROTIdx           = pcCU->getROTIdx ()             + uiAbsPartIdx;
#endif
#if CU_LEVEL_MPI
  m_MPIIdx           = pcCU->getMPIIdx ()             + uiAbsPartIdx;
#endif
#if QC_IMV
  m_iMVFlag            = pcCU->getiMVFlag()               + uiAbsPartIdx;
  if( !m_bDecSubCu )
  {
    m_piMVCandNum        = pcCU->getiMVCandNum()            + uiAbsPartIdx;
  }
#endif
#if QC_OBMC
  m_OBMCFlag           = pcCU->getOBMCFlag ()             + uiAbsPartIdx;
#endif
#if QC_IC
  m_pbICFlag           = pcCU->getICFlag()                + uiAbsPartIdx;
#endif
  m_pePartSize         = pcCU->getPartitionSize ()        + uiAbsPartIdx;
  m_pePredMode         = pcCU->getPredictionMode()        + uiAbsPartIdx;
  m_CUTransquantBypass = pcCU->getCUTransquantBypass()    + uiAbsPartIdx;
  m_puhInterDir        = pcCU->getInterDir      ()        + uiAbsPartIdx;
  
  m_puhDepth           = pcCU->getDepth ()                + uiAbsPartIdx;
  m_puhWidth           = pcCU->getWidth ()                + uiAbsPartIdx;
  m_puhHeight          = pcCU->getHeight()                + uiAbsPartIdx;
  
  m_pbMergeFlag        = pcCU->getMergeFlag()             + uiAbsPartIdx;
  m_puhMergeIndex      = pcCU->getMergeIndex()            + uiAbsPartIdx;
#if QC_FRUC_MERGE
  m_puhFRUCMgrMode     = pcCU->getFRUCMgrMode()          + uiAbsPartIdx;
#endif

#if QC_SUB_PU_TMVP
  m_peMergeType        = pcCU->getMergeType()             + uiAbsPartIdx;
#endif

  m_apiMVPIdx[eRefPicList] = pcCU->getMVPIdx(eRefPicList) + uiAbsPartIdx;
  m_apiMVPNum[eRefPicList] = pcCU->getMVPNum(eRefPicList) + uiAbsPartIdx;
  
  m_acCUMvField[ eRefPicList ].linkToWithOffset( pcCU->getCUMvField(eRefPicList), uiAbsPartIdx );

  memcpy(m_sliceStartCU,pcCU->m_sliceStartCU+uiAbsPartIdx,sizeof(UInt)*m_uiNumPartition);
  memcpy(m_sliceSegmentStartCU,pcCU->m_sliceSegmentStartCU+uiAbsPartIdx,sizeof(UInt)*m_uiNumPartition);
}

#if INTER_KLT
// Copy  CU data to this CU.
// One of quarter parts overwritten by predicted sub part.
Void TComDataCU::copySameSizeCUFrom(TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth)
{
  assert(uiPartUnitIdx < 4);

  m_dTotalCost = 0;
  m_uiTotalDistortion = 0;
  m_uiTotalBits = 0;
  m_uiTotalBins = 0; 
  UInt uiOffset = 0;

  UInt uiNumPartition = pcCU->getTotalNumPart();
  Int iSizeInUchar = sizeof(UChar)* uiNumPartition;
  Int iSizeInBool = sizeof(Bool)* uiNumPartition;

  Int sizeInChar = sizeof(Char)* uiNumPartition;
  memcpy(m_skipFlag + uiOffset, pcCU->getSkipFlag(), sizeof(*m_skipFlag)   * uiNumPartition);
#if ROT_TR
  memcpy(m_ROTIdx + uiOffset, pcCU->getROTIdx(), sizeof(*m_ROTIdx)   * uiNumPartition);
#endif
#if CU_LEVEL_MPI
  memcpy(m_MPIIdx + uiOffset, pcCU->getMPIIdx(), sizeof(*m_MPIIdx)   * uiNumPartition);
#endif
#if QC_IMV
  memcpy(m_iMVFlag + uiOffset, pcCU->getiMVFlag(), sizeof(*m_iMVFlag)    * uiNumPartition);
  if (!m_bDecSubCu)
  {
    memcpy(m_piMVCandNum + uiOffset, pcCU->getiMVCandNum(), sizeof(*m_piMVCandNum)    * uiNumPartition);
  }
#endif
#if QC_OBMC
  memcpy(m_OBMCFlag + uiOffset, pcCU->getOBMCFlag(), sizeof(*m_OBMCFlag)   * uiNumPartition);
#endif
#if QC_IC
  memcpy(m_pbICFlag + uiOffset, pcCU->getICFlag(), iSizeInBool);
#endif
  memcpy(m_phQP + uiOffset, pcCU->getQP(), sizeInChar);
  memcpy(m_pePartSize + uiOffset, pcCU->getPartitionSize(), sizeof(*m_pePartSize) * uiNumPartition);
  memcpy(m_pePredMode + uiOffset, pcCU->getPredictionMode(), sizeof(*m_pePredMode) * uiNumPartition);
  memcpy(m_CUTransquantBypass + uiOffset, pcCU->getCUTransquantBypass(), sizeof(*m_CUTransquantBypass) * uiNumPartition);
#if ALF_HM3_QC_REFACTOR
  memcpy(m_puiAlfCtrlFlag + uiOffset, pcCU->getAlfCtrlFlag(), sizeof(*m_puiAlfCtrlFlag) * uiNumPartition);
#endif
  memcpy(m_pbMergeFlag + uiOffset, pcCU->getMergeFlag(), iSizeInBool);
  memcpy(m_puhMergeIndex + uiOffset, pcCU->getMergeIndex(), iSizeInUchar);
#if QC_FRUC_MERGE
  memcpy(m_puhFRUCMgrMode + uiOffset, pcCU->getFRUCMgrMode(), iSizeInUchar);
#endif
#if QC_SUB_PU_TMVP
  memcpy(m_peMergeType + uiOffset, pcCU->getMergeType(), sizeof(UChar)* uiNumPartition);
#endif
  memcpy(m_puhLumaIntraDir + uiOffset, pcCU->getLumaIntraDir(), iSizeInUchar);
  memcpy(m_puhChromaIntraDir + uiOffset, pcCU->getChromaIntraDir(), iSizeInUchar);
  memcpy(m_puhInterDir + uiOffset, pcCU->getInterDir(), iSizeInUchar);
#if QC_EMT
  memcpy(m_puhEmtTuIdx + uiOffset, pcCU->getEmtTuIdx(), iSizeInUchar);
  memcpy(m_puhEmtCuFlag + uiOffset, pcCU->getEmtCuFlag(), iSizeInUchar);
#endif
  memcpy(m_puhTrIdx + uiOffset, pcCU->getTransformIdx(), iSizeInUchar);
  memcpy(m_puhTransformSkip[0] + uiOffset, pcCU->getTransformSkip(TEXT_LUMA), iSizeInUchar);
  memcpy(m_puhTransformSkip[1] + uiOffset, pcCU->getTransformSkip(TEXT_CHROMA_U), iSizeInUchar);
  memcpy(m_puhTransformSkip[2] + uiOffset, pcCU->getTransformSkip(TEXT_CHROMA_V), iSizeInUchar);
#if KLT_COMMON
  memcpy(m_puhKLTFlag[0] + uiOffset, pcCU->getKLTFlag(TEXT_LUMA), iSizeInUchar);
  memcpy(m_puhKLTFlag[1] + uiOffset, pcCU->getKLTFlag(TEXT_CHROMA_U), iSizeInUchar);
  memcpy(m_puhKLTFlag[2] + uiOffset, pcCU->getKLTFlag(TEXT_CHROMA_V), iSizeInUchar);
#endif
  memcpy(m_puhCbf[0] + uiOffset, pcCU->getCbf(TEXT_LUMA), iSizeInUchar);
  memcpy(m_puhCbf[1] + uiOffset, pcCU->getCbf(TEXT_CHROMA_U), iSizeInUchar);
  memcpy(m_puhCbf[2] + uiOffset, pcCU->getCbf(TEXT_CHROMA_V), iSizeInUchar);

  memcpy(m_puhDepth + uiOffset, pcCU->getDepth(), iSizeInUchar);
#if QC_LARGE_CTU
  memcpy(m_puhWidth + uiOffset, pcCU->getWidth(), iSizeInUchar << 1);
  memcpy(m_puhHeight + uiOffset, pcCU->getHeight(), iSizeInUchar << 1);
#else
  memcpy(m_puhWidth + uiOffset, pcCU->getWidth(), iSizeInUchar);
  memcpy(m_puhHeight + uiOffset, pcCU->getHeight(), iSizeInUchar);
#endif

  memcpy(m_apiMVPIdx[0] + uiOffset, pcCU->getMVPIdx(REF_PIC_LIST_0), iSizeInUchar);
  memcpy(m_apiMVPIdx[1] + uiOffset, pcCU->getMVPIdx(REF_PIC_LIST_1), iSizeInUchar);
  memcpy(m_apiMVPNum[0] + uiOffset, pcCU->getMVPNum(REF_PIC_LIST_0), iSizeInUchar);
  memcpy(m_apiMVPNum[1] + uiOffset, pcCU->getMVPNum(REF_PIC_LIST_1), iSizeInUchar);

  memcpy(m_pbIPCMFlag + uiOffset, pcCU->getIPCMFlag(), iSizeInBool);

  m_pcCUAboveLeft = pcCU->getCUAboveLeft();
  m_pcCUAboveRight = pcCU->getCUAboveRight();
  m_pcCUAbove = pcCU->getCUAbove();
  m_pcCULeft = pcCU->getCULeft();

  m_apcCUColocated[0] = pcCU->getCUColocated(REF_PIC_LIST_0);
  m_apcCUColocated[1] = pcCU->getCUColocated(REF_PIC_LIST_1);

  m_acCUMvField[0].copyFrom(pcCU->getCUMvField(REF_PIC_LIST_0), pcCU->getTotalNumPart(), uiOffset);
  m_acCUMvField[1].copyFrom(pcCU->getCUMvField(REF_PIC_LIST_1), pcCU->getTotalNumPart(), uiOffset);

  UInt uiTmp = g_uiMaxCUWidth*g_uiMaxCUHeight >> (uiDepth << 1);
  UInt uiTmp2 = uiPartUnitIdx*uiTmp;
  memcpy(m_pcTrCoeffY + uiTmp2, pcCU->getCoeffY(), sizeof(TCoeff)*uiTmp);
#if ADAPTIVE_QP_SELECTION
  memcpy(m_pcArlCoeffY + uiTmp2, pcCU->getArlCoeffY(), sizeof(Int)*uiTmp);
#endif
  memcpy(m_pcIPCMSampleY + uiTmp2, pcCU->getPCMSampleY(), sizeof(Pel)* uiTmp);

  uiTmp >>= 2; uiTmp2 >>= 2;
  memcpy(m_pcTrCoeffCb + uiTmp2, pcCU->getCoeffCb(), sizeof(TCoeff)*uiTmp);
  memcpy(m_pcTrCoeffCr + uiTmp2, pcCU->getCoeffCr(), sizeof(TCoeff)*uiTmp);
#if ADAPTIVE_QP_SELECTION
  memcpy(m_pcArlCoeffCb + uiTmp2, pcCU->getArlCoeffCb(), sizeof(Int)*uiTmp);
  memcpy(m_pcArlCoeffCr + uiTmp2, pcCU->getArlCoeffCr(), sizeof(Int)*uiTmp);
#endif
  memcpy(m_pcIPCMSampleCb + uiTmp2, pcCU->getPCMSampleCb(), sizeof(Pel)* uiTmp);
  memcpy(m_pcIPCMSampleCr + uiTmp2, pcCU->getPCMSampleCr(), sizeof(Pel)* uiTmp);
  m_uiTotalBins += pcCU->getTotalBins();
  memcpy(m_sliceStartCU + uiOffset, pcCU->m_sliceStartCU, sizeof(UInt)* uiNumPartition);
  memcpy(m_sliceSegmentStartCU + uiOffset, pcCU->m_sliceSegmentStartCU, sizeof(UInt)* uiNumPartition);
}
#endif
// Copy small CU to bigger CU.
// One of quarter parts overwritten by predicted sub part.
Void TComDataCU::copyPartFrom( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth )
{
  assert( uiPartUnitIdx<4 );
  
  m_dTotalCost         += pcCU->getTotalCost();
  m_uiTotalDistortion  += pcCU->getTotalDistortion();
  m_uiTotalBits        += pcCU->getTotalBits();
  
  UInt uiOffset         = pcCU->getTotalNumPart()*uiPartUnitIdx;
  
  UInt uiNumPartition = pcCU->getTotalNumPart();
  Int iSizeInUchar  = sizeof( UChar ) * uiNumPartition;
  Int iSizeInBool   = sizeof( Bool  ) * uiNumPartition;
  
  Int sizeInChar  = sizeof( Char ) * uiNumPartition;
  memcpy( m_skipFlag   + uiOffset, pcCU->getSkipFlag(),       sizeof( *m_skipFlag )   * uiNumPartition );
#if ROT_TR
  memcpy( m_ROTIdx   + uiOffset, pcCU->getROTIdx(),       sizeof( *m_ROTIdx )   * uiNumPartition );
#endif
#if CU_LEVEL_MPI
  memcpy( m_MPIIdx   + uiOffset, pcCU->getMPIIdx(),       sizeof( *m_MPIIdx )   * uiNumPartition );
#endif
#if QC_IMV
  memcpy( m_iMVFlag    + uiOffset, pcCU->getiMVFlag(),        sizeof( *m_iMVFlag )    * uiNumPartition );
  if( !m_bDecSubCu )
  {
    memcpy( m_piMVCandNum + uiOffset, pcCU->getiMVCandNum(),        sizeof( *m_piMVCandNum )    * uiNumPartition );
  }
#endif
#if QC_OBMC
  memcpy( m_OBMCFlag   + uiOffset, pcCU->getOBMCFlag(),       sizeof( *m_OBMCFlag )   * uiNumPartition );
#endif
#if QC_IC
  memcpy( m_pbICFlag   + uiOffset, pcCU->getICFlag(),         iSizeInBool );
#endif
  memcpy( m_phQP       + uiOffset, pcCU->getQP(),             sizeInChar                        );
  memcpy( m_pePartSize + uiOffset, pcCU->getPartitionSize(),  sizeof( *m_pePartSize ) * uiNumPartition );
  memcpy( m_pePredMode + uiOffset, pcCU->getPredictionMode(), sizeof( *m_pePredMode ) * uiNumPartition );
  memcpy( m_CUTransquantBypass + uiOffset, pcCU->getCUTransquantBypass(), sizeof( *m_CUTransquantBypass ) * uiNumPartition );
#if ALF_HM3_QC_REFACTOR
  memcpy( m_puiAlfCtrlFlag      + uiOffset, pcCU->getAlfCtrlFlag(),       sizeof(*m_puiAlfCtrlFlag) * uiNumPartition  );
#endif
  memcpy( m_pbMergeFlag         + uiOffset, pcCU->getMergeFlag(),         iSizeInBool  );
  memcpy( m_puhMergeIndex       + uiOffset, pcCU->getMergeIndex(),        iSizeInUchar );
#if QC_FRUC_MERGE
  memcpy( m_puhFRUCMgrMode      + uiOffset, pcCU->getFRUCMgrMode(),      iSizeInUchar );
#endif
#if QC_SUB_PU_TMVP
  memcpy( m_peMergeType         + uiOffset, pcCU->getMergeType(),        sizeof( UChar ) * uiNumPartition );
#endif
  memcpy( m_puhLumaIntraDir     + uiOffset, pcCU->getLumaIntraDir(),      iSizeInUchar );
  memcpy( m_puhChromaIntraDir   + uiOffset, pcCU->getChromaIntraDir(),    iSizeInUchar );
  memcpy( m_puhInterDir         + uiOffset, pcCU->getInterDir(),          iSizeInUchar );
#if QC_EMT
  memcpy( m_puhEmtTuIdx         + uiOffset, pcCU->getEmtTuIdx(),          iSizeInUchar );
  memcpy( m_puhEmtCuFlag        + uiOffset, pcCU->getEmtCuFlag(),         iSizeInUchar );
#endif
  memcpy( m_puhTrIdx            + uiOffset, pcCU->getTransformIdx(),      iSizeInUchar );
  memcpy( m_puhTransformSkip[0] + uiOffset, pcCU->getTransformSkip(TEXT_LUMA),     iSizeInUchar );
  memcpy( m_puhTransformSkip[1] + uiOffset, pcCU->getTransformSkip(TEXT_CHROMA_U), iSizeInUchar );
  memcpy( m_puhTransformSkip[2] + uiOffset, pcCU->getTransformSkip(TEXT_CHROMA_V), iSizeInUchar );
#if KLT_COMMON
  memcpy( m_puhKLTFlag[0]    + uiOffset, pcCU->getKLTFlag(TEXT_LUMA),  iSizeInUchar);
  memcpy( m_puhKLTFlag[1]    + uiOffset, pcCU->getKLTFlag(TEXT_CHROMA_U), iSizeInUchar);
  memcpy( m_puhKLTFlag[2]    + uiOffset, pcCU->getKLTFlag(TEXT_CHROMA_V), iSizeInUchar);
#endif
  memcpy( m_puhCbf[0] + uiOffset, pcCU->getCbf(TEXT_LUMA)    , iSizeInUchar );
  memcpy( m_puhCbf[1] + uiOffset, pcCU->getCbf(TEXT_CHROMA_U), iSizeInUchar );
  memcpy( m_puhCbf[2] + uiOffset, pcCU->getCbf(TEXT_CHROMA_V), iSizeInUchar );
  
  memcpy( m_puhDepth  + uiOffset, pcCU->getDepth(),  iSizeInUchar );
#if QC_LARGE_CTU
  memcpy( m_puhWidth  + uiOffset, pcCU->getWidth(),  iSizeInUchar << 1 );
  memcpy( m_puhHeight + uiOffset, pcCU->getHeight(), iSizeInUchar << 1 );
#else
  memcpy( m_puhWidth  + uiOffset, pcCU->getWidth(),  iSizeInUchar );
  memcpy( m_puhHeight + uiOffset, pcCU->getHeight(), iSizeInUchar );
#endif
  
  memcpy( m_apiMVPIdx[0] + uiOffset, pcCU->getMVPIdx(REF_PIC_LIST_0), iSizeInUchar );
  memcpy( m_apiMVPIdx[1] + uiOffset, pcCU->getMVPIdx(REF_PIC_LIST_1), iSizeInUchar );
  memcpy( m_apiMVPNum[0] + uiOffset, pcCU->getMVPNum(REF_PIC_LIST_0), iSizeInUchar );
  memcpy( m_apiMVPNum[1] + uiOffset, pcCU->getMVPNum(REF_PIC_LIST_1), iSizeInUchar );
  
  memcpy( m_pbIPCMFlag + uiOffset, pcCU->getIPCMFlag(), iSizeInBool );

  m_pcCUAboveLeft      = pcCU->getCUAboveLeft();
  m_pcCUAboveRight     = pcCU->getCUAboveRight();
  m_pcCUAbove          = pcCU->getCUAbove();
  m_pcCULeft           = pcCU->getCULeft();
  
  m_apcCUColocated[0] = pcCU->getCUColocated(REF_PIC_LIST_0);
  m_apcCUColocated[1] = pcCU->getCUColocated(REF_PIC_LIST_1);
  
  m_acCUMvField[0].copyFrom( pcCU->getCUMvField( REF_PIC_LIST_0 ), pcCU->getTotalNumPart(), uiOffset );
  m_acCUMvField[1].copyFrom( pcCU->getCUMvField( REF_PIC_LIST_1 ), pcCU->getTotalNumPart(), uiOffset );
  
  UInt uiTmp  = g_uiMaxCUWidth*g_uiMaxCUHeight >> (uiDepth<<1);
  UInt uiTmp2 = uiPartUnitIdx*uiTmp;
  memcpy( m_pcTrCoeffY  + uiTmp2, pcCU->getCoeffY(),  sizeof(TCoeff)*uiTmp );
#if ADAPTIVE_QP_SELECTION
  memcpy( m_pcArlCoeffY  + uiTmp2, pcCU->getArlCoeffY(),  sizeof(Int)*uiTmp );
#endif
  memcpy( m_pcIPCMSampleY + uiTmp2 , pcCU->getPCMSampleY(), sizeof(Pel) * uiTmp );

  uiTmp >>= 2; uiTmp2>>= 2;
  memcpy( m_pcTrCoeffCb + uiTmp2, pcCU->getCoeffCb(), sizeof(TCoeff)*uiTmp );
  memcpy( m_pcTrCoeffCr + uiTmp2, pcCU->getCoeffCr(), sizeof(TCoeff)*uiTmp );
#if ADAPTIVE_QP_SELECTION
  memcpy( m_pcArlCoeffCb + uiTmp2, pcCU->getArlCoeffCb(), sizeof(Int)*uiTmp );
  memcpy( m_pcArlCoeffCr + uiTmp2, pcCU->getArlCoeffCr(), sizeof(Int)*uiTmp );
#endif
  memcpy( m_pcIPCMSampleCb + uiTmp2 , pcCU->getPCMSampleCb(), sizeof(Pel) * uiTmp );
  memcpy( m_pcIPCMSampleCr + uiTmp2 , pcCU->getPCMSampleCr(), sizeof(Pel) * uiTmp );
  m_uiTotalBins += pcCU->getTotalBins();
  memcpy( m_sliceStartCU        + uiOffset, pcCU->m_sliceStartCU,        sizeof( UInt ) * uiNumPartition  );
  memcpy( m_sliceSegmentStartCU + uiOffset, pcCU->m_sliceSegmentStartCU, sizeof( UInt ) * uiNumPartition  );
}

// Copy current predicted part to a CU in picture.
// It is used to predict for next part
Void TComDataCU::copyToPic( UChar uhDepth )
{
  TComDataCU*& rpcCU = m_pcPic->getCU( m_uiCUAddr );
  
  rpcCU->getTotalCost()       = m_dTotalCost;
  rpcCU->getTotalDistortion() = m_uiTotalDistortion;
  rpcCU->getTotalBits()       = m_uiTotalBits;
  
  Int iSizeInUchar  = sizeof( UChar ) * m_uiNumPartition;
  Int iSizeInBool   = sizeof( Bool  ) * m_uiNumPartition;
  
  Int sizeInChar  = sizeof( Char ) * m_uiNumPartition;

  memcpy( rpcCU->getSkipFlag() + m_uiAbsIdxInLCU, m_skipFlag, sizeof( *m_skipFlag ) * m_uiNumPartition );
#if ROT_TR
  memcpy( rpcCU->getROTIdx() + m_uiAbsIdxInLCU, m_ROTIdx, sizeof( *m_ROTIdx ) * m_uiNumPartition );
#endif
#if CU_LEVEL_MPI
  memcpy( rpcCU->getMPIIdx() + m_uiAbsIdxInLCU, m_MPIIdx, sizeof( *m_MPIIdx ) * m_uiNumPartition );
#endif
#if QC_IMV
  memcpy( rpcCU->getiMVFlag() + m_uiAbsIdxInLCU, m_iMVFlag, sizeof( *m_iMVFlag ) * m_uiNumPartition );
  if( !m_bDecSubCu )
  {
    memcpy( rpcCU->getiMVCandNum() + m_uiAbsIdxInLCU, m_piMVCandNum, sizeof( *m_piMVCandNum ) * m_uiNumPartition );
  }
#endif
#if QC_OBMC
  memcpy( rpcCU->getOBMCFlag() + m_uiAbsIdxInLCU, m_OBMCFlag, sizeof( *m_OBMCFlag ) * m_uiNumPartition );
#endif
#if QC_IC
  memcpy( rpcCU->getICFlag()         + m_uiAbsIdxInLCU, m_pbICFlag,            iSizeInBool );
#endif
  memcpy( rpcCU->getQP() + m_uiAbsIdxInLCU, m_phQP, sizeInChar  );

  memcpy( rpcCU->getPartitionSize()  + m_uiAbsIdxInLCU, m_pePartSize, sizeof( *m_pePartSize ) * m_uiNumPartition );
  memcpy( rpcCU->getPredictionMode() + m_uiAbsIdxInLCU, m_pePredMode, sizeof( *m_pePredMode ) * m_uiNumPartition );
  memcpy( rpcCU->getCUTransquantBypass()+ m_uiAbsIdxInLCU, m_CUTransquantBypass, sizeof( *m_CUTransquantBypass ) * m_uiNumPartition );
#if ALF_HM3_QC_REFACTOR
  memcpy( rpcCU->getAlfCtrlFlag()    + m_uiAbsIdxInLCU, m_puiAlfCtrlFlag,    sizeof( *m_puiAlfCtrlFlag ) * m_uiNumPartition );
#endif
  memcpy( rpcCU->getMergeFlag()         + m_uiAbsIdxInLCU, m_pbMergeFlag,         iSizeInBool  );
  memcpy( rpcCU->getMergeIndex()        + m_uiAbsIdxInLCU, m_puhMergeIndex,       iSizeInUchar );
#if QC_FRUC_MERGE
  memcpy( rpcCU->getFRUCMgrMode()      + m_uiAbsIdxInLCU, m_puhFRUCMgrMode,      iSizeInUchar  );
#endif
#if QC_SUB_PU_TMVP
  memcpy( rpcCU->getMergeType()         + m_uiAbsIdxInLCU, m_peMergeType,         sizeof( UChar ) * m_uiNumPartition );
#endif
  memcpy( rpcCU->getLumaIntraDir()      + m_uiAbsIdxInLCU, m_puhLumaIntraDir,     iSizeInUchar );
  memcpy( rpcCU->getChromaIntraDir()    + m_uiAbsIdxInLCU, m_puhChromaIntraDir,   iSizeInUchar );
  memcpy( rpcCU->getInterDir()          + m_uiAbsIdxInLCU, m_puhInterDir,         iSizeInUchar );
#if QC_EMT
  memcpy( rpcCU->getEmtTuIdx()          + m_uiAbsIdxInLCU, m_puhEmtTuIdx,         iSizeInUchar );
  memcpy( rpcCU->getEmtCuFlag()         + m_uiAbsIdxInLCU, m_puhEmtCuFlag,        iSizeInUchar );
#endif
  memcpy( rpcCU->getTransformIdx()      + m_uiAbsIdxInLCU, m_puhTrIdx,            iSizeInUchar );
  memcpy( rpcCU->getTransformSkip(TEXT_LUMA)     + m_uiAbsIdxInLCU, m_puhTransformSkip[0], iSizeInUchar );
  memcpy( rpcCU->getTransformSkip(TEXT_CHROMA_U) + m_uiAbsIdxInLCU, m_puhTransformSkip[1], iSizeInUchar );
  memcpy( rpcCU->getTransformSkip(TEXT_CHROMA_V) + m_uiAbsIdxInLCU, m_puhTransformSkip[2], iSizeInUchar );
#if KLT_COMMON
  memcpy( rpcCU->getKLTFlag(TEXT_LUMA)  + m_uiAbsIdxInLCU, m_puhKLTFlag[0],       iSizeInUchar);
  memcpy( rpcCU->getKLTFlag(TEXT_CHROMA_U) + m_uiAbsIdxInLCU, m_puhKLTFlag[1],    iSizeInUchar);
  memcpy( rpcCU->getKLTFlag(TEXT_CHROMA_V) + m_uiAbsIdxInLCU, m_puhKLTFlag[2],    iSizeInUchar);
#endif
  memcpy( rpcCU->getCbf(TEXT_LUMA)     + m_uiAbsIdxInLCU, m_puhCbf[0], iSizeInUchar );
  memcpy( rpcCU->getCbf(TEXT_CHROMA_U) + m_uiAbsIdxInLCU, m_puhCbf[1], iSizeInUchar );
  memcpy( rpcCU->getCbf(TEXT_CHROMA_V) + m_uiAbsIdxInLCU, m_puhCbf[2], iSizeInUchar );
  
  memcpy( rpcCU->getDepth()  + m_uiAbsIdxInLCU, m_puhDepth,  iSizeInUchar );
#if QC_LARGE_CTU
  memcpy( rpcCU->getWidth()  + m_uiAbsIdxInLCU, m_puhWidth,  iSizeInUchar << 1 );
  memcpy( rpcCU->getHeight() + m_uiAbsIdxInLCU, m_puhHeight, iSizeInUchar << 1 );
#else
  memcpy( rpcCU->getWidth()  + m_uiAbsIdxInLCU, m_puhWidth,  iSizeInUchar );
  memcpy( rpcCU->getHeight() + m_uiAbsIdxInLCU, m_puhHeight, iSizeInUchar );
#endif
  
  memcpy( rpcCU->getMVPIdx(REF_PIC_LIST_0) + m_uiAbsIdxInLCU, m_apiMVPIdx[0], iSizeInUchar );
  memcpy( rpcCU->getMVPIdx(REF_PIC_LIST_1) + m_uiAbsIdxInLCU, m_apiMVPIdx[1], iSizeInUchar );
  memcpy( rpcCU->getMVPNum(REF_PIC_LIST_0) + m_uiAbsIdxInLCU, m_apiMVPNum[0], iSizeInUchar );
  memcpy( rpcCU->getMVPNum(REF_PIC_LIST_1) + m_uiAbsIdxInLCU, m_apiMVPNum[1], iSizeInUchar );
  
  m_acCUMvField[0].copyTo( rpcCU->getCUMvField( REF_PIC_LIST_0 ), m_uiAbsIdxInLCU );
  m_acCUMvField[1].copyTo( rpcCU->getCUMvField( REF_PIC_LIST_1 ), m_uiAbsIdxInLCU );
  
  memcpy( rpcCU->getIPCMFlag() + m_uiAbsIdxInLCU, m_pbIPCMFlag,         iSizeInBool  );

  UInt uiTmp  = (g_uiMaxCUWidth*g_uiMaxCUHeight)>>(uhDepth<<1);
  UInt uiTmp2 = m_uiAbsIdxInLCU*m_pcPic->getMinCUWidth()*m_pcPic->getMinCUHeight();
  memcpy( rpcCU->getCoeffY()  + uiTmp2, m_pcTrCoeffY,  sizeof(TCoeff)*uiTmp  );
#if ADAPTIVE_QP_SELECTION  
  memcpy( rpcCU->getArlCoeffY()  + uiTmp2, m_pcArlCoeffY,  sizeof(Int)*uiTmp  );
#endif
  memcpy( rpcCU->getPCMSampleY() + uiTmp2 , m_pcIPCMSampleY, sizeof(Pel)*uiTmp );

  uiTmp >>= 2; uiTmp2 >>= 2;
  memcpy( rpcCU->getCoeffCb() + uiTmp2, m_pcTrCoeffCb, sizeof(TCoeff)*uiTmp  );
  memcpy( rpcCU->getCoeffCr() + uiTmp2, m_pcTrCoeffCr, sizeof(TCoeff)*uiTmp  );
#if ADAPTIVE_QP_SELECTION
  memcpy( rpcCU->getArlCoeffCb() + uiTmp2, m_pcArlCoeffCb, sizeof(Int)*uiTmp  );
  memcpy( rpcCU->getArlCoeffCr() + uiTmp2, m_pcArlCoeffCr, sizeof(Int)*uiTmp  );
#endif
  memcpy( rpcCU->getPCMSampleCb() + uiTmp2 , m_pcIPCMSampleCb, sizeof( Pel ) * uiTmp );
  memcpy( rpcCU->getPCMSampleCr() + uiTmp2 , m_pcIPCMSampleCr, sizeof( Pel ) * uiTmp );
  rpcCU->getTotalBins() = m_uiTotalBins;
  memcpy( rpcCU->m_sliceStartCU        + m_uiAbsIdxInLCU, m_sliceStartCU,        sizeof( UInt ) * m_uiNumPartition  );
  memcpy( rpcCU->m_sliceSegmentStartCU + m_uiAbsIdxInLCU, m_sliceSegmentStartCU, sizeof( UInt ) * m_uiNumPartition  );
}

Void TComDataCU::copyToPic( UChar uhDepth, UInt uiPartIdx, UInt uiPartDepth )
{
  TComDataCU*&  rpcCU       = m_pcPic->getCU( m_uiCUAddr );
  UInt          uiQNumPart  = m_uiNumPartition>>(uiPartDepth<<1);
  
  UInt uiPartStart          = uiPartIdx*uiQNumPart;
  UInt uiPartOffset         = m_uiAbsIdxInLCU + uiPartStart;
  
  rpcCU->getTotalCost()       = m_dTotalCost;
  rpcCU->getTotalDistortion() = m_uiTotalDistortion;
  rpcCU->getTotalBits()       = m_uiTotalBits;
  
  Int iSizeInUchar  = sizeof( UChar  ) * uiQNumPart;
  Int iSizeInBool   = sizeof( Bool   ) * uiQNumPart;
  
  Int sizeInChar  = sizeof( Char ) * uiQNumPart;
  memcpy( rpcCU->getSkipFlag()       + uiPartOffset, m_skipFlag,   sizeof( *m_skipFlag )   * uiQNumPart );
#if ROT_TR
  memcpy( rpcCU->getROTIdx()       + uiPartOffset, m_ROTIdx,   sizeof( *m_ROTIdx )   * uiQNumPart );
#endif
#if CU_LEVEL_MPI
  memcpy( rpcCU->getMPIIdx()       + uiPartOffset, m_MPIIdx,   sizeof( *m_MPIIdx )   * uiQNumPart );
#endif
#if QC_IMV
  memcpy( rpcCU->getiMVFlag()        + uiPartOffset, m_iMVFlag,     sizeof( *m_iMVFlag )   * uiQNumPart );
  if( !m_bDecSubCu )
  {
    memcpy( rpcCU->getiMVCandNum()     + uiPartOffset, m_piMVCandNum, sizeof( *m_piMVCandNum )   * uiQNumPart );
  }
#endif
#if QC_OBMC
  memcpy( rpcCU->getOBMCFlag()       + uiPartOffset, m_OBMCFlag,   sizeof( *m_OBMCFlag )   * uiQNumPart );
#endif
#if QC_IC
  memcpy( rpcCU->getICFlag()         + uiPartOffset, m_pbICFlag,   iSizeInBool );
#endif
  memcpy( rpcCU->getQP() + uiPartOffset, m_phQP, sizeInChar );
  memcpy( rpcCU->getPartitionSize()  + uiPartOffset, m_pePartSize, sizeof( *m_pePartSize ) * uiQNumPart );
  memcpy( rpcCU->getPredictionMode() + uiPartOffset, m_pePredMode, sizeof( *m_pePredMode ) * uiQNumPart );
  memcpy( rpcCU->getCUTransquantBypass()+ uiPartOffset, m_CUTransquantBypass, sizeof( *m_CUTransquantBypass ) * uiQNumPart );
#if ALF_HM3_QC_REFACTOR
  memcpy( rpcCU->getAlfCtrlFlag()       + uiPartOffset, m_puiAlfCtrlFlag,      sizeof( *m_puiAlfCtrlFlag ) * uiQNumPart );
#endif
  memcpy( rpcCU->getMergeFlag()         + uiPartOffset, m_pbMergeFlag,         iSizeInBool  );
  memcpy( rpcCU->getMergeIndex()        + uiPartOffset, m_puhMergeIndex,       iSizeInUchar );
#if QC_FRUC_MERGE
  memcpy( rpcCU->getFRUCMgrMode()      + uiPartOffset, m_puhFRUCMgrMode,      iSizeInUchar  );
#endif
#if QC_SUB_PU_TMVP
  memcpy( rpcCU->getMergeType()         + uiPartOffset, m_peMergeType,         sizeof( UChar ) * uiQNumPart );
#endif
  memcpy( rpcCU->getLumaIntraDir()      + uiPartOffset, m_puhLumaIntraDir,     iSizeInUchar );
  memcpy( rpcCU->getChromaIntraDir()    + uiPartOffset, m_puhChromaIntraDir,   iSizeInUchar );
  memcpy( rpcCU->getInterDir()          + uiPartOffset, m_puhInterDir,         iSizeInUchar );
#if QC_EMT
  memcpy( rpcCU->getEmtTuIdx()          + uiPartOffset, m_puhEmtTuIdx,         iSizeInUchar );
  memcpy( rpcCU->getEmtCuFlag()         + uiPartOffset, m_puhEmtCuFlag,        iSizeInUchar );
#endif
  memcpy( rpcCU->getTransformIdx()      + uiPartOffset, m_puhTrIdx,            iSizeInUchar );
  memcpy( rpcCU->getTransformSkip(TEXT_LUMA)     + uiPartOffset, m_puhTransformSkip[0], iSizeInUchar );
  memcpy( rpcCU->getTransformSkip(TEXT_CHROMA_U) + uiPartOffset, m_puhTransformSkip[1], iSizeInUchar );
  memcpy( rpcCU->getTransformSkip(TEXT_CHROMA_V) + uiPartOffset, m_puhTransformSkip[2], iSizeInUchar );
#if KLT_COMMON
  memcpy( rpcCU->getKLTFlag(TEXT_LUMA)  + uiPartOffset, m_puhKLTFlag[0],       iSizeInUchar );
  memcpy( rpcCU->getKLTFlag(TEXT_CHROMA_U) + uiPartOffset, m_puhKLTFlag[1],    iSizeInUchar );
  memcpy( rpcCU->getKLTFlag(TEXT_CHROMA_V) + uiPartOffset, m_puhKLTFlag[2],    iSizeInUchar );
#endif
  memcpy( rpcCU->getCbf(TEXT_LUMA)     + uiPartOffset, m_puhCbf[0], iSizeInUchar );
  memcpy( rpcCU->getCbf(TEXT_CHROMA_U) + uiPartOffset, m_puhCbf[1], iSizeInUchar );
  memcpy( rpcCU->getCbf(TEXT_CHROMA_V) + uiPartOffset, m_puhCbf[2], iSizeInUchar );
  
  memcpy( rpcCU->getDepth()  + uiPartOffset, m_puhDepth,  iSizeInUchar );
#if QC_LARGE_CTU
  memcpy( rpcCU->getWidth()  + uiPartOffset, m_puhWidth,  iSizeInUchar << 1 );
  memcpy( rpcCU->getHeight() + uiPartOffset, m_puhHeight, iSizeInUchar << 1 );
#else
  memcpy( rpcCU->getWidth()  + uiPartOffset, m_puhWidth,  iSizeInUchar );
  memcpy( rpcCU->getHeight() + uiPartOffset, m_puhHeight, iSizeInUchar );
#endif
  
  memcpy( rpcCU->getMVPIdx(REF_PIC_LIST_0) + uiPartOffset, m_apiMVPIdx[0], iSizeInUchar );
  memcpy( rpcCU->getMVPIdx(REF_PIC_LIST_1) + uiPartOffset, m_apiMVPIdx[1], iSizeInUchar );
  memcpy( rpcCU->getMVPNum(REF_PIC_LIST_0) + uiPartOffset, m_apiMVPNum[0], iSizeInUchar );
  memcpy( rpcCU->getMVPNum(REF_PIC_LIST_1) + uiPartOffset, m_apiMVPNum[1], iSizeInUchar );
  m_acCUMvField[0].copyTo( rpcCU->getCUMvField( REF_PIC_LIST_0 ), m_uiAbsIdxInLCU, uiPartStart, uiQNumPart );
  m_acCUMvField[1].copyTo( rpcCU->getCUMvField( REF_PIC_LIST_1 ), m_uiAbsIdxInLCU, uiPartStart, uiQNumPart );
  
  memcpy( rpcCU->getIPCMFlag() + uiPartOffset, m_pbIPCMFlag,         iSizeInBool  );

  UInt uiTmp  = (g_uiMaxCUWidth*g_uiMaxCUHeight)>>((uhDepth+uiPartDepth)<<1);
  UInt uiTmp2 = uiPartOffset*m_pcPic->getMinCUWidth()*m_pcPic->getMinCUHeight();
  memcpy( rpcCU->getCoeffY()  + uiTmp2, m_pcTrCoeffY,  sizeof(TCoeff)*uiTmp  );
#if ADAPTIVE_QP_SELECTION
  memcpy( rpcCU->getArlCoeffY()  + uiTmp2, m_pcArlCoeffY,  sizeof(Int)*uiTmp  );
#endif
 
  memcpy( rpcCU->getPCMSampleY() + uiTmp2 , m_pcIPCMSampleY, sizeof( Pel ) * uiTmp );

  uiTmp >>= 2; uiTmp2 >>= 2;
  memcpy( rpcCU->getCoeffCb() + uiTmp2, m_pcTrCoeffCb, sizeof(TCoeff)*uiTmp  );
  memcpy( rpcCU->getCoeffCr() + uiTmp2, m_pcTrCoeffCr, sizeof(TCoeff)*uiTmp  );
#if ADAPTIVE_QP_SELECTION
  memcpy( rpcCU->getArlCoeffCb() + uiTmp2, m_pcArlCoeffCb, sizeof(Int)*uiTmp  );
  memcpy( rpcCU->getArlCoeffCr() + uiTmp2, m_pcArlCoeffCr, sizeof(Int)*uiTmp  );
#endif

  memcpy( rpcCU->getPCMSampleCb() + uiTmp2 , m_pcIPCMSampleCb, sizeof( Pel ) * uiTmp );
  memcpy( rpcCU->getPCMSampleCr() + uiTmp2 , m_pcIPCMSampleCr, sizeof( Pel ) * uiTmp );
  rpcCU->getTotalBins() = m_uiTotalBins;
  memcpy( rpcCU->m_sliceStartCU        + uiPartOffset, m_sliceStartCU,        sizeof( UInt ) * uiQNumPart  );
  memcpy( rpcCU->m_sliceSegmentStartCU + uiPartOffset, m_sliceSegmentStartCU, sizeof( UInt ) * uiQNumPart  );
}

// --------------------------------------------------------------------------------------------------------------------
// Other public functions
// --------------------------------------------------------------------------------------------------------------------

TComDataCU* TComDataCU::getPULeft( UInt& uiLPartUnitIdx, 
                                   UInt uiCurrPartUnitIdx, 
                                   Bool bEnforceSliceRestriction, 
                                   Bool bEnforceTileRestriction )
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_uiAbsIdxInLCU];
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  
  if ( !RasterAddress::isZeroCol( uiAbsPartIdx, uiNumPartInCUWidth ) )
  {
    uiLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - 1 ];
    if ( RasterAddress::isEqualCol( uiAbsPartIdx, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
    {
      return m_pcPic->getCU( getAddr() );
    }
    else
    {
      uiLPartUnitIdx -= m_uiAbsIdxInLCU;
      return this;
    }
  }
  
  uiLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + uiNumPartInCUWidth - 1 ];


  if ( (bEnforceSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL || m_pcCULeft->getSCUAddr()+uiLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)))
      ||
       (bEnforceTileRestriction && ( m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL || (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))  )  )
      )
  {
    return NULL;
  }
  return m_pcCULeft;
}

TComDataCU* TComDataCU::getPUAbove( UInt& uiAPartUnitIdx,
                                    UInt uiCurrPartUnitIdx, 
                                    Bool bEnforceSliceRestriction, 
                                    Bool planarAtLCUBoundary ,
                                    Bool bEnforceTileRestriction )
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_uiAbsIdxInLCU];
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  
  if ( !RasterAddress::isZeroRow( uiAbsPartIdx, uiNumPartInCUWidth ) )
  {
    uiAPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - uiNumPartInCUWidth ];
    if ( RasterAddress::isEqualRow( uiAbsPartIdx, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
    {
      return m_pcPic->getCU( getAddr() );
    }
    else
    {
      uiAPartUnitIdx -= m_uiAbsIdxInLCU;
      return this;
    }
  }

  if(planarAtLCUBoundary)
  {
    return NULL;
  }
  
  uiAPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + m_pcPic->getNumPartInCU() - uiNumPartInCUWidth ];

  if ( (bEnforceSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL || m_pcCUAbove->getSCUAddr()+uiAPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)))
      ||
       (bEnforceTileRestriction &&(m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL || (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))))
      )
  {
    return NULL;
  }
  return m_pcCUAbove;
}

TComDataCU* TComDataCU::getPUAboveLeft( UInt& uiALPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction )
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_uiAbsIdxInLCU];
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  
  if ( !RasterAddress::isZeroCol( uiAbsPartIdx, uiNumPartInCUWidth ) )
  {
    if ( !RasterAddress::isZeroRow( uiAbsPartIdx, uiNumPartInCUWidth ) )
    {
      uiALPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - uiNumPartInCUWidth - 1 ];
      if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdx, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
      {
        return m_pcPic->getCU( getAddr() );
      }
      else
      {
        uiALPartUnitIdx -= m_uiAbsIdxInLCU;
        return this;
      }
    }
    uiALPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + getPic()->getNumPartInCU() - uiNumPartInCUWidth - 1 ];
    if ( (bEnforceSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL ||
       m_pcCUAbove->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCUAbove;
  }
  
  if ( !RasterAddress::isZeroRow( uiAbsPartIdx, uiNumPartInCUWidth ) )
  {
    uiALPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - 1 ];
    if ( (bEnforceSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL || 
       m_pcCULeft->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCULeft;
  }
  
  uiALPartUnitIdx = g_auiRasterToZscan[ m_pcPic->getNumPartInCU() - 1 ];
  if ( (bEnforceSliceRestriction && (m_pcCUAboveLeft==NULL || m_pcCUAboveLeft->getSlice()==NULL ||
       m_pcCUAboveLeft->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAboveLeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
  {
    return NULL;
  }
  return m_pcCUAboveLeft;
}

TComDataCU* TComDataCU::getPUAboveRight( UInt& uiARPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction )
{
  UInt uiAbsPartIdxRT     = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + m_puhWidth[0] / m_pcPic->getMinCUWidth() - 1;
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  
  if( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdxRT] + m_pcPic->getMinCUWidth() ) >= m_pcSlice->getSPS()->getPicWidthInLumaSamples() )
  {
    uiARPartUnitIdx = MAX_UINT;
    return NULL;
  }
  
  if ( RasterAddress::lessThanCol( uiAbsPartIdxRT, uiNumPartInCUWidth - 1, uiNumPartInCUWidth ) )
  {
    if ( !RasterAddress::isZeroRow( uiAbsPartIdxRT, uiNumPartInCUWidth ) )
    {
      if ( uiCurrPartUnitIdx > g_auiRasterToZscan[ uiAbsPartIdxRT - uiNumPartInCUWidth + 1 ] )
      {
        uiARPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxRT - uiNumPartInCUWidth + 1 ];
        if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdxRT, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
        {
          return m_pcPic->getCU( getAddr() );
        }
        else
        {
          uiARPartUnitIdx -= m_uiAbsIdxInLCU;
          return this;
        }
      }
      uiARPartUnitIdx = MAX_UINT;
      return NULL;
    }
    uiARPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxRT + m_pcPic->getNumPartInCU() - uiNumPartInCUWidth + 1 ];
    if ( (bEnforceSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL ||
       m_pcCUAbove->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCUAbove;
  }
  
  if ( !RasterAddress::isZeroRow( uiAbsPartIdxRT, uiNumPartInCUWidth ) )
  {
    uiARPartUnitIdx = MAX_UINT;
    return NULL;
  }
  
  uiARPartUnitIdx = g_auiRasterToZscan[ m_pcPic->getNumPartInCU() - uiNumPartInCUWidth ];
  if ( (bEnforceSliceRestriction && (m_pcCUAboveRight==NULL || m_pcCUAboveRight->getSlice()==NULL ||
       m_pcPic->getPicSym()->getInverseCUOrderMap( m_pcCUAboveRight->getAddr()) > m_pcPic->getPicSym()->getInverseCUOrderMap( getAddr()) ||
       m_pcCUAboveRight->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAboveRight->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
  {
    return NULL;
  }
  return m_pcCUAboveRight;
}

TComDataCU* TComDataCU::getPUBelowLeft( UInt& uiBLPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction )
{
  UInt uiAbsPartIdxLB     = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdxLB = g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + (m_puhHeight[0] / m_pcPic->getMinCUHeight() - 1)*m_pcPic->getNumPartInWidth();
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  
  if( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdxLB] + m_pcPic->getMinCUHeight() ) >= m_pcSlice->getSPS()->getPicHeightInLumaSamples() )
  {
    uiBLPartUnitIdx = MAX_UINT;
    return NULL;
  }
  
  if ( RasterAddress::lessThanRow( uiAbsPartIdxLB, m_pcPic->getNumPartInHeight() - 1, uiNumPartInCUWidth ) )
  {
    if ( !RasterAddress::isZeroCol( uiAbsPartIdxLB, uiNumPartInCUWidth ) )
    {
      if ( uiCurrPartUnitIdx > g_auiRasterToZscan[ uiAbsPartIdxLB + uiNumPartInCUWidth - 1 ] )
      {
        uiBLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxLB + uiNumPartInCUWidth - 1 ];
        if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdxLB, uiAbsZorderCUIdxLB, uiNumPartInCUWidth ) )
        {
          return m_pcPic->getCU( getAddr() );
        }
        else
        {
          uiBLPartUnitIdx -= m_uiAbsIdxInLCU;
          return this;
        }
      }
      uiBLPartUnitIdx = MAX_UINT;
      return NULL;
    }
    uiBLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxLB + uiNumPartInCUWidth*2 - 1 ];
    if ( (bEnforceSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL || 
       m_pcCULeft->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCULeft;
  }
  
  uiBLPartUnitIdx = MAX_UINT;
  return NULL;
}

#if QC_FRUC_MERGE
Bool TComDataCU::getBlockBelowRight( UInt uiAbsPartIdx, Int nCurBlkWidth , Int nCurBlkHeight , UInt & rCUAddr , UInt & rBRAbsPartIdx )
{
  assert( MIN_PU_SIZE == 4 );
  Int x = getCUPelX() + g_auiRasterToPelX[g_auiZscanToRaster[uiAbsPartIdx]] + nCurBlkWidth;
  Int y = getCUPelY() + g_auiRasterToPelY[g_auiZscanToRaster[uiAbsPartIdx]] + nCurBlkHeight;
  if( x >= getPic()->getPicYuvRec()->getWidth() || y >= getPic()->getPicYuvRec()->getHeight() )
    return( false );

  rCUAddr = getPic()->getFrameWidthInCU() * ( y >> ( g_aucConvertToBit[g_uiMaxCUWidth] + 2 ) ) + ( x >> ( g_aucConvertToBit[g_uiMaxCUWidth] + 2 ) );
  Int nMask = ( 1 << ( g_aucConvertToBit[g_uiMaxCUWidth] + 2 ) ) - 1;
  UInt uiRasterIdx = ( ( y & nMask ) >> 2 ) * getPic()->getNumPartInWidth() + ( ( x & nMask ) >> 2 );
  rBRAbsPartIdx = g_auiRasterToZscan[uiRasterIdx];
  return( true );
}
#endif

TComDataCU* TComDataCU::getPUBelowLeftAdi(UInt& uiBLPartUnitIdx,  UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset, Bool bEnforceSliceRestriction )
{
  UInt uiAbsPartIdxLB     = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdxLB = g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + ((m_puhHeight[0] / m_pcPic->getMinCUHeight()) - 1)*m_pcPic->getNumPartInWidth();
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  
  if( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdxLB] + (m_pcPic->getPicSym()->getMinCUHeight() * uiPartUnitOffset)) >= m_pcSlice->getSPS()->getPicHeightInLumaSamples())
  {
    uiBLPartUnitIdx = MAX_UINT;
    return NULL;
  }
  
  if ( RasterAddress::lessThanRow( uiAbsPartIdxLB, m_pcPic->getNumPartInHeight() - uiPartUnitOffset, uiNumPartInCUWidth ) )
  {
    if ( !RasterAddress::isZeroCol( uiAbsPartIdxLB, uiNumPartInCUWidth ) )
    {
      if ( uiCurrPartUnitIdx > g_auiRasterToZscan[ uiAbsPartIdxLB + uiPartUnitOffset * uiNumPartInCUWidth - 1 ] )
      {
        uiBLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxLB + uiPartUnitOffset * uiNumPartInCUWidth - 1 ];
        if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdxLB, uiAbsZorderCUIdxLB, uiNumPartInCUWidth ) )
        {
          return m_pcPic->getCU( getAddr() );
        }
        else
        {
          uiBLPartUnitIdx -= m_uiAbsIdxInLCU;
          return this;
        }
      }
      uiBLPartUnitIdx = MAX_UINT;
      return NULL;
    }
    uiBLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxLB + (1+uiPartUnitOffset) * uiNumPartInCUWidth - 1 ];
    if ( (bEnforceSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL || 
       m_pcCULeft->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCULeft;
  }
  
  uiBLPartUnitIdx = MAX_UINT;
  return NULL;
}

TComDataCU* TComDataCU::getPUAboveRightAdi(UInt&  uiARPartUnitIdx, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset, Bool bEnforceSliceRestriction )
{
  UInt uiAbsPartIdxRT     = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + (m_puhWidth[0] / m_pcPic->getMinCUWidth()) - 1;
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  
  if( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdxRT] + (m_pcPic->getPicSym()->getMinCUHeight() * uiPartUnitOffset)) >= m_pcSlice->getSPS()->getPicWidthInLumaSamples() )
  {
    uiARPartUnitIdx = MAX_UINT;
    return NULL;
  }
  
  if ( RasterAddress::lessThanCol( uiAbsPartIdxRT, uiNumPartInCUWidth - uiPartUnitOffset, uiNumPartInCUWidth ) )
  {
    if ( !RasterAddress::isZeroRow( uiAbsPartIdxRT, uiNumPartInCUWidth ) )
    {
      if ( uiCurrPartUnitIdx > g_auiRasterToZscan[ uiAbsPartIdxRT - uiNumPartInCUWidth + uiPartUnitOffset ] )
      {
        uiARPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxRT - uiNumPartInCUWidth + uiPartUnitOffset ];
        if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdxRT, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
        {
          return m_pcPic->getCU( getAddr() );
        }
        else
        {
          uiARPartUnitIdx -= m_uiAbsIdxInLCU;
          return this;
        }
      }
      uiARPartUnitIdx = MAX_UINT;
      return NULL;
    }
    uiARPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxRT + m_pcPic->getNumPartInCU() - uiNumPartInCUWidth + uiPartUnitOffset ];
    if ( (bEnforceSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL || 
       m_pcCUAbove->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCUAbove;
  }
  
  if ( !RasterAddress::isZeroRow( uiAbsPartIdxRT, uiNumPartInCUWidth ) )
  {
    uiARPartUnitIdx = MAX_UINT;
    return NULL;
  }
  
  uiARPartUnitIdx = g_auiRasterToZscan[ m_pcPic->getNumPartInCU() - uiNumPartInCUWidth + uiPartUnitOffset-1 ];
  if ( (bEnforceSliceRestriction && (m_pcCUAboveRight==NULL || m_pcCUAboveRight->getSlice()==NULL ||
       m_pcPic->getPicSym()->getInverseCUOrderMap( m_pcCUAboveRight->getAddr()) > m_pcPic->getPicSym()->getInverseCUOrderMap( getAddr()) ||
       m_pcCUAboveRight->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAboveRight->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
  {
    return NULL;
  }
  return m_pcCUAboveRight;
}

/** Get left QpMinCu
*\param   uiLPartUnitIdx
*\param   uiCurrAbsIdxInLCU
*\returns TComDataCU*   point of TComDataCU of left QpMinCu
*/
TComDataCU* TComDataCU::getQpMinCuLeft( UInt& uiLPartUnitIdx, UInt uiCurrAbsIdxInLCU)
{
  UInt numPartInCUWidth = m_pcPic->getNumPartInWidth();
  UInt absZorderQpMinCUIdx = (uiCurrAbsIdxInLCU>>((g_uiMaxCUDepth - getSlice()->getPPS()->getMaxCuDQPDepth())<<1))<<((g_uiMaxCUDepth -getSlice()->getPPS()->getMaxCuDQPDepth())<<1);
  UInt absRorderQpMinCUIdx = g_auiZscanToRaster[absZorderQpMinCUIdx];

  // check for left LCU boundary
  if ( RasterAddress::isZeroCol(absRorderQpMinCUIdx, numPartInCUWidth) )
  {
    return NULL;
  }

  // get index of left-CU relative to top-left corner of current quantization group
  uiLPartUnitIdx = g_auiRasterToZscan[absRorderQpMinCUIdx - 1];

  // return pointer to current LCU
  return m_pcPic->getCU( getAddr() );
}

/** Get Above QpMinCu
*\param   aPartUnitIdx
*\param   currAbsIdxInLCU
*\returns TComDataCU*   point of TComDataCU of above QpMinCu
*/
TComDataCU* TComDataCU::getQpMinCuAbove( UInt& aPartUnitIdx, UInt currAbsIdxInLCU )
{
  UInt numPartInCUWidth = m_pcPic->getNumPartInWidth();
  UInt absZorderQpMinCUIdx = (currAbsIdxInLCU>>((g_uiMaxCUDepth - getSlice()->getPPS()->getMaxCuDQPDepth())<<1))<<((g_uiMaxCUDepth - getSlice()->getPPS()->getMaxCuDQPDepth())<<1);
  UInt absRorderQpMinCUIdx = g_auiZscanToRaster[absZorderQpMinCUIdx];

  // check for top LCU boundary
  if ( RasterAddress::isZeroRow( absRorderQpMinCUIdx, numPartInCUWidth) )
  {
    return NULL;
  }

  // get index of top-CU relative to top-left corner of current quantization group
  aPartUnitIdx = g_auiRasterToZscan[absRorderQpMinCUIdx - numPartInCUWidth];

  // return pointer to current LCU
  return m_pcPic->getCU( getAddr() );
}

/** Get reference QP from left QpMinCu or latest coded QP
*\param   uiCurrAbsIdxInLCU
*\returns Char   reference QP value
*/
Char TComDataCU::getRefQP( UInt uiCurrAbsIdxInLCU )
{
  UInt        lPartIdx = 0, aPartIdx = 0;
  TComDataCU* cULeft  = getQpMinCuLeft ( lPartIdx, m_uiAbsIdxInLCU + uiCurrAbsIdxInLCU );
  TComDataCU* cUAbove = getQpMinCuAbove( aPartIdx, m_uiAbsIdxInLCU + uiCurrAbsIdxInLCU );
  return (((cULeft? cULeft->getQP( lPartIdx ): getLastCodedQP( uiCurrAbsIdxInLCU )) + (cUAbove? cUAbove->getQP( aPartIdx ): getLastCodedQP( uiCurrAbsIdxInLCU )) + 1) >> 1);
}

Int TComDataCU::getLastValidPartIdx( Int iAbsPartIdx )
{
  Int iLastValidPartIdx = iAbsPartIdx-1;
  while ( iLastValidPartIdx >= 0
       && getPredictionMode( iLastValidPartIdx ) == MODE_NONE )
  {
    UInt uiDepth = getDepth( iLastValidPartIdx );
    iLastValidPartIdx -= m_uiNumPartition>>(uiDepth<<1);
  }
  return iLastValidPartIdx;
}

Char TComDataCU::getLastCodedQP( UInt uiAbsPartIdx )
{
  UInt uiQUPartIdxMask = ~((1<<((g_uiMaxCUDepth - getSlice()->getPPS()->getMaxCuDQPDepth())<<1))-1);
  Int iLastValidPartIdx = getLastValidPartIdx( uiAbsPartIdx&uiQUPartIdxMask );
  if ( uiAbsPartIdx < m_uiNumPartition
    && (getSCUAddr()+iLastValidPartIdx < getSliceStartCU(m_uiAbsIdxInLCU+uiAbsPartIdx)))
  {
    return getSlice()->getSliceQp();
  }
  else if ( iLastValidPartIdx >= 0 )
  {
    return getQP( iLastValidPartIdx );
  }
  else
  {
    if ( getZorderIdxInCU() > 0 )
    {
      return getPic()->getCU( getAddr() )->getLastCodedQP( getZorderIdxInCU() );
    }
    else if ( getPic()->getPicSym()->getInverseCUOrderMap(getAddr()) > 0
      && getPic()->getPicSym()->getTileIdxMap(getAddr()) == getPic()->getPicSym()->getTileIdxMap(getPic()->getPicSym()->getCUOrderMap(getPic()->getPicSym()->getInverseCUOrderMap(getAddr())-1))
      && !( getSlice()->getPPS()->getEntropyCodingSyncEnabledFlag() && getAddr() % getPic()->getFrameWidthInCU() == 0 ) )
    {
      return getPic()->getCU( getPic()->getPicSym()->getCUOrderMap(getPic()->getPicSym()->getInverseCUOrderMap(getAddr())-1) )->getLastCodedQP( getPic()->getNumPartInCU() );
    }
    else
    {
      return getSlice()->getSliceQp();
    }
  }
}
/** Check whether the CU is coded in lossless coding mode
 * \param   uiAbsPartIdx
 * \returns true if the CU is coded in lossless coding mode; false if otherwise 
 */
Bool TComDataCU::isLosslessCoded(UInt absPartIdx)
{
  return (getSlice()->getPPS()->getTransquantBypassEnableFlag() && getCUTransquantBypass (absPartIdx));
}

/** Get allowed chroma intra modes
*\param   uiAbsPartIdx
*\param   uiModeList  pointer to chroma intra modes array
*\returns 
*- fill uiModeList with chroma intra modes
*/
Void TComDataCU::getAllowedChromaDir( UInt uiAbsPartIdx, UInt* uiModeList )
{
  uiModeList[0] = PLANAR_IDX;
  uiModeList[1] = VER_IDX;
  uiModeList[2] = HOR_IDX;
  uiModeList[3] = DC_IDX;
  
#if QC_LMCHROMA
  uiModeList[4] = LM_CHROMA_IDX;
  uiModeList[5] = DM_CHROMA_IDX;
#else
  uiModeList[4] = DM_CHROMA_IDX;
#endif

  UInt uiLumaMode = getLumaIntraDir( uiAbsPartIdx );

#if QC_LMCHROMA
  for( Int i = 0; i < NUM_CHROMA_MODE - 2; i++ )
#else
  for( Int i = 0; i < NUM_CHROMA_MODE - 1; i++ )
#endif
  {
    if( uiLumaMode == uiModeList[i] )
    {
#if QC_USE_65ANG_MODES
      uiModeList[i] = VDIA_IDX; // VER+8 mode
#else
      uiModeList[i] = 34; // VER+8 mode
#endif
      break;
    }
  }
}

/** Get most probable intra modes
*\param   uiAbsPartIdx
*\param   uiIntraDirPred  pointer to the array for MPM storage
*\param   piMode          it is set with MPM mode in case both MPM are equal. It is used to restrict RD search at encode side.
*\returns Number of MPM
*/
#if QC_USE_65ANG_MODES
TComDataCU* TComDataCU::getPULeftOffset( UInt& uiPartUnitIdx, 
                                        UInt uiCurrPartUnitIdx, 
                                        UInt uiPartOffset,
                                        Bool bEnforceSliceRestriction, 
                                        Bool bEnforceTileRestriction )
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_uiAbsIdxInLCU];
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  
  uiAbsPartIdx += uiPartOffset * uiNumPartInCUWidth;

  if ( !RasterAddress::isZeroCol( uiAbsPartIdx, uiNumPartInCUWidth ) )
  {
    uiPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - 1 ];
    if ( RasterAddress::isEqualCol( uiAbsPartIdx, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
    {
      return m_pcPic->getCU( getAddr() );
    }
    else
    {
      uiPartUnitIdx -= m_uiAbsIdxInLCU;
      return this;
    }
  }
  
  uiPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + uiNumPartInCUWidth - 1 ];


  if ( (bEnforceSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL || m_pcCULeft->getSCUAddr()+uiPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)))
      ||
       (bEnforceTileRestriction && ( m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL || (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))  )  )
      )
  {
    return NULL;
  }
  return m_pcCULeft;
}

TComDataCU* TComDataCU::getPUAboveOffset( UInt& uiPartUnitIdx, 
                                        UInt uiCurrPartUnitIdx, 
                                        UInt uiPartOffset,
                                        Bool bEnforceSliceRestriction, 
                                        Bool planarAtLCUBoundary,
                                        Bool bEnforceTileRestriction )
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_uiAbsIdxInLCU];
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  
  uiAbsPartIdx += uiPartOffset;

  if ( !RasterAddress::isZeroRow( uiAbsPartIdx, uiNumPartInCUWidth ) )
  {
    uiPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - uiNumPartInCUWidth ];
    if ( RasterAddress::isEqualRow( uiAbsPartIdx, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
    {
      return m_pcPic->getCU( getAddr() );
    }
    else
    {
      uiPartUnitIdx -= m_uiAbsIdxInLCU;
      return this;
    }
  }

  if(planarAtLCUBoundary)
  {
    return NULL;
  }
  
  uiPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + m_pcPic->getNumPartInCU() - uiNumPartInCUWidth ];

  if ( (bEnforceSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL || m_pcCUAbove->getSCUAddr()+uiPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)))
      ||
       (bEnforceTileRestriction &&(m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL || (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))))
      )
  {
    return NULL;
  }
  return m_pcCUAbove;
}
#endif

Int TComDataCU::getIntraDirLumaPredictor( UInt uiAbsPartIdx, Int* uiIntraDirPred
#if QC_USE_65ANG_MODES
                                         , Int &iLeftAboveCase
#endif
                                         , Int* piMode )
{
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  Int         iLeftIntraDir, iAboveIntraDir;
  Int         uiPredNum = 0;

#if QC_USE_65ANG_MODES
  UInt        uiPUWidth   = ( getWidth(uiAbsPartIdx) >> ( getPartitionSize(uiAbsPartIdx)==SIZE_NxN ? 1 : 0 ) );
  UInt        uiPUHeight  = ( getHeight(uiAbsPartIdx) >> ( getPartitionSize(uiAbsPartIdx)==SIZE_NxN ? 1 : 0 ) );

  UInt uiCaseIdx = 0;

  if( getUseExtIntraAngModes(uiPUWidth) )
  {
    UInt        uiPartIdxLT = m_uiAbsIdxInLCU + uiAbsPartIdx;
    UInt        uiPartIdxRT = g_auiRasterToZscan [g_auiZscanToRaster[ uiPartIdxLT ] + uiPUWidth / m_pcPic->getMinCUWidth() - 1 ];
    UInt        uiPartIdxLB = g_auiRasterToZscan [g_auiZscanToRaster[ m_uiAbsIdxInLCU + uiAbsPartIdx ] + ((uiPUHeight / m_pcPic->getMinCUHeight()) - 1)*m_pcPic->getNumPartInWidth()];;
    UChar       ucTempIntraDir;
    const UInt  uiNumUnitsInPU = (g_auiZscanToRaster[uiPartIdxLB] - g_auiZscanToRaster[uiPartIdxLT]) / getPic()->getNumPartInWidth() + 1;

    // Initializaiton
    Int uiLeftIntraDirCnt[NUM_INTRA_MODE], uiAboveIntraDirCnt[NUM_INTRA_MODE];
    memset( uiLeftIntraDirCnt,  0, sizeof(Int)*NUM_INTRA_MODE );
    memset( uiAboveIntraDirCnt, 0, sizeof(Int)*NUM_INTRA_MODE );
    iLeftIntraDir  = DC_IDX;
    iAboveIntraDir = DC_IDX;

    deriveLeftRightTopIdxGeneral( uiAbsPartIdx, 0, uiPartIdxLT, uiPartIdxRT );

    // Get intra direction from above side
    UInt uiDirMaxCount = 0;
    for ( UInt uiOffset = 0; uiOffset < uiNumUnitsInPU; uiOffset++ )
    {
      pcTempCU = getPUAboveOffset( uiTempPartIdx, uiPartIdxLT, uiOffset );

      if( pcTempCU && pcTempCU->isIntra( uiTempPartIdx ) == MODE_INTRA )
      {
        ucTempIntraDir = pcTempCU->getLumaIntraDir( uiTempPartIdx );
        uiAboveIntraDirCnt[ucTempIntraDir] ++;
        if( uiDirMaxCount<uiAboveIntraDirCnt[ucTempIntraDir] )
        {
          uiDirMaxCount  = uiAboveIntraDirCnt[ucTempIntraDir];
          iAboveIntraDir = ucTempIntraDir;
        }
      }
    }

    // Get intra direction from left side
    uiDirMaxCount = 0;
    for ( UInt uiOffset = 0; uiOffset < uiNumUnitsInPU; uiOffset++ )
    {
      pcTempCU = getPULeftOffset( uiTempPartIdx, uiPartIdxLT, uiOffset );

      if( pcTempCU && pcTempCU->isIntra( uiTempPartIdx ) == MODE_INTRA )
      {
        ucTempIntraDir = pcTempCU->getLumaIntraDir( uiTempPartIdx );
        uiLeftIntraDirCnt[ucTempIntraDir] ++;
        if( uiDirMaxCount<uiLeftIntraDirCnt[ucTempIntraDir] )
        {
          uiDirMaxCount = uiLeftIntraDirCnt[ucTempIntraDir];
          iLeftIntraDir = ucTempIntraDir;
        }
      }
    }
    uiPredNum = 6;
  }
  else
  {
#endif
  // Get intra direction of left PU
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  iLeftIntraDir  = pcTempCU ? ( pcTempCU->isIntra( uiTempPartIdx ) ? pcTempCU->getLumaIntraDir( uiTempPartIdx ) : DC_IDX ) : DC_IDX;

  // Get intra direction of above PU
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx, true, true );
  iAboveIntraDir = pcTempCU ? ( pcTempCU->isIntra( uiTempPartIdx ) ? pcTempCU->getLumaIntraDir( uiTempPartIdx ) : DC_IDX ) : DC_IDX;

  uiPredNum = 3;
#if QC_USE_65ANG_MODES
  iLeftIntraDir  = ( iLeftIntraDir<2  ? iLeftIntraDir  : ((((iLeftIntraDir-2)>>1)<<1)+2) );
  iAboveIntraDir = ( iAboveIntraDir<2 ? iAboveIntraDir : ((((iAboveIntraDir-2)>>1)<<1)+2) );
  }
#endif

#if QC_USE_65ANG_MODES
  const UInt uiHor = HOR_IDX;
  const UInt uiVer = VER_IDX;
  const UInt uiDia = DIA_IDX;
  const Int  iOffset = 62;
  const Int  iMod = iOffset+3;
#endif

  if(iLeftIntraDir == iAboveIntraDir)
  {
    if( piMode )
    {
      *piMode = 1;
    }
    
    if (iLeftIntraDir > 1) // angular modes
    {
      uiIntraDirPred[0] = iLeftIntraDir;
#if QC_USE_65ANG_MODES
      if( getUseExtIntraAngModes(uiPUWidth) )
      {
        uiIntraDirPred[1] = PLANAR_IDX;
        uiIntraDirPred[2] = ((iLeftIntraDir - 1 ) % iMod) + 2;
        uiIntraDirPred[3] = ((iLeftIntraDir + iOffset) % iMod) + 2;
        uiIntraDirPred[4] = ((uiIntraDirPred[2] - 1 ) % iMod) + 2;
        uiIntraDirPred[5] = DC_IDX;
        uiCaseIdx         = 0;
      }
      else
      {
        iLeftIntraDir = (iLeftIntraDir>>1) + 1;
        uiIntraDirPred[1] = ((iLeftIntraDir + 29) % 32) * 2 + 2;
        uiIntraDirPred[2] = ((iLeftIntraDir - 1 ) % 32) * 2 + 2;
      }
#else
      uiIntraDirPred[1] = ((iLeftIntraDir + 29) % 32) + 2;
      uiIntraDirPred[2] = ((iLeftIntraDir - 1 ) % 32) + 2;
#endif
    }
    else //non-angular
    {
      uiIntraDirPred[0] = PLANAR_IDX;
      uiIntraDirPred[1] = DC_IDX;
#if QC_USE_65ANG_MODES
      uiIntraDirPred[2] = uiVer; 
      uiIntraDirPred[3] = uiHor; 
      uiIntraDirPred[4] = 2;
      uiIntraDirPred[5] = uiDia;
      uiCaseIdx         = 1;
#else
      uiIntraDirPred[2] = VER_IDX; 
#endif
    }
  }
  else
  {
    if( piMode )
    {
      *piMode = 2;
    }
    uiIntraDirPred[0] = iLeftIntraDir;
    uiIntraDirPred[1] = iAboveIntraDir;
    
    if (iLeftIntraDir && iAboveIntraDir ) //both modes are non-planar
    {
      uiIntraDirPred[2] = PLANAR_IDX;
#if QC_USE_65ANG_MODES
      Int iMaxDir = max( iAboveIntraDir, iLeftIntraDir );
      Int iMinDir = min( iAboveIntraDir, iLeftIntraDir );
      if( iLeftIntraDir==DC_IDX || iAboveIntraDir==DC_IDX )
      {
        Int iNonDcMode = iMaxDir;
        uiIntraDirPred[3] = (( iNonDcMode + iOffset ) % iMod) + 2;
        uiIntraDirPred[4] = (( iNonDcMode - 1 ) % iMod) + 2;
        uiIntraDirPred[5] = (( uiIntraDirPred[4] -1 ) % iMod) + 2;
      }
      else
      {
        uiIntraDirPred[3] = DC_IDX;
        uiIntraDirPred[4] = (( iMaxDir - 1 ) % iMod) + 2;
        if( uiIntraDirPred[4] == iMinDir )
        {
          uiIntraDirPred[4]++;
        }
        uiIntraDirPred[5] = (( iMinDir + iOffset ) % iMod) + 2;
        if( uiIntraDirPred[5] == iMaxDir )
        {
          uiIntraDirPred[5]--;
        }
        if( uiIntraDirPred[5] == uiIntraDirPred[4] )
        {
          uiIntraDirPred[5] = iMinDir + 1;
        }
      }
      uiCaseIdx = 2;
#endif
    }
    else
    {
#if QC_USE_65ANG_MODES
      Int iMaxDir = max( iAboveIntraDir, iLeftIntraDir );
      uiIntraDirPred[2] =  (iLeftIntraDir+iAboveIntraDir)<2? uiVer : DC_IDX;
      if( (iLeftIntraDir+iAboveIntraDir)<2 )
      {
        uiIntraDirPred[3] = uiHor;
        uiIntraDirPred[4] = 2;
        uiIntraDirPred[5] = uiDia;
      }
      else
      {
        uiIntraDirPred[3] = ((iMaxDir + iOffset ) % iMod) + 2;
        uiIntraDirPred[4] = ((iMaxDir - 1 ) % iMod) + 2;
        uiIntraDirPred[5] = ((uiIntraDirPred[4] - 1 ) % iMod) + 2;
      }
      uiCaseIdx = 3;
#else
      uiIntraDirPred[2] =  (iLeftIntraDir+iAboveIntraDir)<2? VER_IDX : DC_IDX;
#endif
    }
  }

#if QC_USE_65ANG_MODES
  if ( iLeftAboveCase!=-1 )
  {
    iLeftAboveCase = uiCaseIdx;
  }
#endif

  return uiPredNum;
}

UInt TComDataCU::getCtxSplitFlag( UInt uiAbsPartIdx, UInt uiDepth )
{
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  UInt        uiCtx;
  // Get left split flag
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx  = ( pcTempCU ) ? ( ( pcTempCU->getDepth( uiTempPartIdx ) > uiDepth ) ? 1 : 0 ) : 0;
  
  // Get above split flag
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx += ( pcTempCU ) ? ( ( pcTempCU->getDepth( uiTempPartIdx ) > uiDepth ) ? 1 : 0 ) : 0;
  
#if QC_LARGE_CTU
  UChar ucMinDepth = 0;
  UChar ucMaxDepth = ( UChar )( g_uiMaxCUDepth - g_uiAddCUDepth );
  getMaxMinCUDepth( ucMinDepth , ucMaxDepth , uiAbsPartIdx + getZorderIdxInCU() );
  if( uiDepth < ucMinDepth )
  {
    uiCtx = 3;
  }
  else if( uiDepth >= ucMaxDepth + 1 )
  {    
    uiCtx = 4;
  }
#endif

  return uiCtx;
}

UInt TComDataCU::getCtxQtCbf( TextType eType, UInt uiTrDepth )
{
  if( eType )
  {
    return uiTrDepth;
  }
  else
  {
    const UInt uiCtx = ( uiTrDepth == 0 ? 1 : 0 );
    return uiCtx;
  }
}

UInt TComDataCU::getQuadtreeTULog2MinSizeInCU( UInt absPartIdx )
{
  UInt log2CbSize = g_aucConvertToBit[getWidth( absPartIdx )] + 2;
  PartSize  partSize  = getPartitionSize( absPartIdx );
  UInt quadtreeTUMaxDepth = getPredictionMode( absPartIdx ) == MODE_INTRA ? m_pcSlice->getSPS()->getQuadtreeTUMaxDepthIntra() : m_pcSlice->getSPS()->getQuadtreeTUMaxDepthInter(); 
  Int intraSplitFlag = ( getPredictionMode( absPartIdx ) == MODE_INTRA && partSize == SIZE_NxN ) ? 1 : 0;
  Int interSplitFlag = ((quadtreeTUMaxDepth == 1) && (getPredictionMode( absPartIdx ) == MODE_INTER) && (partSize != SIZE_2Nx2N) );
  
  UInt log2MinTUSizeInCU = 0;

  if (log2CbSize < (m_pcSlice->getSPS()->getQuadtreeTULog2MinSize() + quadtreeTUMaxDepth - 1 + interSplitFlag + intraSplitFlag) ) 
  {
    // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is < QuadtreeTULog2MinSize
    log2MinTUSizeInCU = m_pcSlice->getSPS()->getQuadtreeTULog2MinSize();
  }
  else
  {
    // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is still >= QuadtreeTULog2MinSize
    log2MinTUSizeInCU = log2CbSize - ( quadtreeTUMaxDepth - 1 + interSplitFlag + intraSplitFlag); // stop when trafoDepth == hierarchy_depth = splitFlag
    if ( log2MinTUSizeInCU > m_pcSlice->getSPS()->getQuadtreeTULog2MaxSize())
    {
      // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is still > QuadtreeTULog2MaxSize
      log2MinTUSizeInCU = m_pcSlice->getSPS()->getQuadtreeTULog2MaxSize();
    }  
  }
  return log2MinTUSizeInCU;
}

UInt TComDataCU::getCtxSkipFlag( UInt uiAbsPartIdx )
{
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  UInt        uiCtx = 0;
  
  // Get BCBP of left PU
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx    = ( pcTempCU ) ? pcTempCU->isSkipped( uiTempPartIdx ) : 0;
  
  // Get BCBP of above PU
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx   += ( pcTempCU ) ? pcTempCU->isSkipped( uiTempPartIdx ) : 0;
  
  return uiCtx;
}

#if QC_IMV
UInt TComDataCU::getCtxiMVFlag( UInt uiAbsPartIdx )
{
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  UInt        uiCtx = 0;

  // Get BCBP of left PU
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx    = ( pcTempCU ) ? pcTempCU->getiMVFlag( uiTempPartIdx ) : 0;

  // Get BCBP of above PU
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx   += ( pcTempCU ) ? pcTempCU->getiMVFlag( uiTempPartIdx ) : 0;

  return uiCtx;
}

Bool TComDataCU::hasSubCUNonZeroMVd()
{
  Bool bNonZeroMvd = false;
  Int iPartNum = ( getPartitionSize( 0 ) == SIZE_NxN ? 4 : ( getPartitionSize( 0 ) == SIZE_2Nx2N ? 1 : 2 ) );
  for( Int iPart = 0; iPart < iPartNum; iPart++ )
  {
    UInt uiPartAddr = 0;
    Int iHeight, iWidth;
    getPartIndexAndSize( iPart, uiPartAddr, iWidth, iHeight );
    for( Int iRefPicList = 0; iRefPicList < 2; iRefPicList++ )
    {
      RefPicList eRefPicList = (RefPicList) iRefPicList;
      if( getCUMvField( eRefPicList )->getRefIdx( uiPartAddr ) >= 0 )
      {
        bNonZeroMvd |= ( getCUMvField( eRefPicList )->getMvd( uiPartAddr ).getHor() != 0 );
        bNonZeroMvd |= ( getCUMvField( eRefPicList )->getMvd( uiPartAddr ).getVer() != 0 );
      }
    }
  }
  return( bNonZeroMvd );
}

Char TComDataCU::getMaxNeighboriMVCandNum( UInt uiAbsPartIdx )
{
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  const Char  cDefault = 0;
  Char        cMaxiMVCandNum = 0;

  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  cMaxiMVCandNum    = max( cMaxiMVCandNum , ( pcTempCU ) ? pcTempCU->getiMVCandNum( uiTempPartIdx ) : cDefault );

  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  cMaxiMVCandNum    = max( cMaxiMVCandNum , ( pcTempCU ) ? pcTempCU->getiMVCandNum( uiTempPartIdx ) : cDefault );

  return cMaxiMVCandNum;
}

#if QC_FRUC_MERGE
Bool TComDataCU::resetMVDandMV2Int( UInt uiAbsPartIdx , UInt uiPartIdx , Bool bResetMV , TComPrediction * pPred )
#else
Void TComDataCU::resetMVDandMV2Int( UInt uiAbsPartIdx , UInt uiPartIdx , Bool bResetMV )
#endif
{
  assert( getiMVFlag( uiAbsPartIdx ) == true );

  if( !getMergeFlag( uiAbsPartIdx ) )
  {
    for( RefPicList eRefPicList = REF_PIC_LIST_0 ; eRefPicList <= REF_PIC_LIST_1 ; eRefPicList = ( RefPicList )( ( Int )eRefPicList + 1 ) )
    {
      TComCUMvField * pMVField = getCUMvField( eRefPicList );
      if( pMVField->getRefIdx( uiAbsPartIdx ) >= 0 )
      {
        TComMv mv = pMVField->getMv( uiAbsPartIdx );
        TComMv mvPred;

        AMVPInfo*  pcAMVPInfo = getCUMvField( eRefPicList )->getAMVPInfo();
        fillMvpCand( uiPartIdx , uiAbsPartIdx , eRefPicList , pMVField->getRefIdx( uiAbsPartIdx ) , pcAMVPInfo 
#if QC_FRUC_MERGE
          , pPred
#endif
          );

        mvPred = pcAMVPInfo->m_acMvCand[getMVPIdx(eRefPicList , uiAbsPartIdx)];

        if( bResetMV )
          xRoundMV( mv );

        TComMv mvDiff = mv - mvPred;
        if( getSlice()->getMvdL1ZeroFlag() && eRefPicList == REF_PIC_LIST_1 && getInterDir( uiAbsPartIdx ) == 3 )
        {
          mvDiff.setZero();
          assert( bResetMV == true || mv == mvPred );
          mv = mvPred;
        }
        if( bResetMV )
          pMVField->setAllMv( mv , getPartitionSize( uiAbsPartIdx ) , uiAbsPartIdx , 0 , uiPartIdx );
        pMVField->setAllMvd( mvDiff , getPartitionSize( uiAbsPartIdx ) , uiAbsPartIdx , 0 , uiPartIdx );
      }
    }
  }
#if QC_FRUC_MERGE
  else if( getFRUCMgrMode( uiAbsPartIdx ) )
  {
    Bool bAvailable = pPred->deriveFRUCMV( this , getDepth( uiAbsPartIdx ) , uiAbsPartIdx , uiPartIdx );
    if( bAvailable == false )
      return( false );
  }
#endif
#if QC_SUB_PU_TMVP
  else
  {
    // QC_CY_SUP_TMVP  {{
    UChar    eMergeCandTypeNieghors[MRG_MAX_NUM_CANDS];
    memset(eMergeCandTypeNieghors, MGR_TYPE_DEFAULT_N, sizeof(UChar)*MRG_MAX_NUM_CANDS);
#if QC_SUB_PU_TMVP_EXT
    TComMvField*  pcMvFieldSP[2] = {NULL,NULL};
    UChar* puhInterDirSP[2] = {NULL,NULL};
    pcMvFieldSP[0] = new TComMvField[getPic()->getPicSym()->getNumPartition()*2]; 
    pcMvFieldSP[1] = new TComMvField[getPic()->getPicSym()->getNumPartition()*2]; 
    puhInterDirSP[0] = new UChar[getPic()->getPicSym()->getNumPartition()]; 
    puhInterDirSP[1] = new UChar[getPic()->getPicSym()->getNumPartition()]; 
#else
    TComMvField*  pcMvFieldSP = NULL;
    UChar* puhInterDirSP      = NULL;
    pcMvFieldSP = new TComMvField[getPic()->getPicSym()->getNumPartition()*2]; 
    puhInterDirSP = new UChar[getPic()->getPicSym()->getNumPartition()]; 
#endif
    // }}
    UInt uiMergeIndex = getMergeIndex( uiAbsPartIdx );
    UInt uiDepth = getDepth( uiAbsPartIdx );
    PartSize ePartSize = getPartitionSize( uiAbsPartIdx );
    TComMvField cMvFieldNeighbours[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
    UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
    Int numValidMergeCand = 0;
#if QC_IC
    Bool abICFlag[MRG_MAX_NUM_CANDS];
#endif
    getInterMergeCandidates( uiAbsPartIdx, uiPartIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand 
#if QC_IC
      , abICFlag
#endif
#if QC_SUB_PU_TMVP
      , eMergeCandTypeNieghors
      , pcMvFieldSP
      , puhInterDirSP
#endif
      );


    setInterDirSubParts( uhInterDirNeighbours[uiMergeIndex], uiAbsPartIdx, uiPartIdx, uiDepth );
#if QC_IC
    if( getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )
    {
      setICFlagSubParts( getSlice()->getApplyIC() ? abICFlag[uiMergeIndex] : 0, uiAbsPartIdx, uiDepth );
    }
#endif
    TComMv cTmpMv( 0, 0 );
    for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
    {        
      if ( getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
      {
        setMVPIdxSubParts( 0, RefPicList( uiRefListIdx ), uiAbsPartIdx, uiPartIdx, uiDepth);
        setMVPNumSubParts( 0, RefPicList( uiRefListIdx ), uiAbsPartIdx, uiPartIdx, uiDepth);
        getCUMvField( RefPicList( uiRefListIdx ) )->setAllMvd( cTmpMv, ePartSize, uiAbsPartIdx, 0, uiPartIdx );
        getCUMvField( RefPicList( uiRefListIdx ) )->setAllMvField( cMvFieldNeighbours[ 2*uiMergeIndex + uiRefListIdx ], ePartSize, uiAbsPartIdx, 0, uiPartIdx );
      }
    }

    setMergeTypeSubParts( eMergeCandTypeNieghors[uiMergeIndex], uiAbsPartIdx, uiPartIdx, uiDepth ); 


#if QC_SUB_PU_TMVP_EXT
    if( eMergeCandTypeNieghors[uiMergeIndex] == MGR_TYPE_SUBPU_TMVP || eMergeCandTypeNieghors[uiMergeIndex] == MGR_TYPE_SUBPU_TMVP_EXT)
#else
    if( eMergeCandTypeNieghors[uiMergeIndex] == MGR_TYPE_SUBPU_TMVP )
#endif
    {
      Int iWidth, iHeight;
      UInt uiIdx;
      getPartIndexAndSize( uiPartIdx, uiIdx, iWidth, iHeight, uiAbsPartIdx, true );

      UInt uiSPAddr;

      Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;
#if QC_SUB_PU_TMVP_EXT
      UInt uiSPListIndex = eMergeCandTypeNieghors[uiMergeIndex] == MGR_TYPE_SUBPU_TMVP?0:1;
#endif

      getSPPara(iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);

      for (Int iPartitionIdx = 0; iPartitionIdx < iNumSP; iPartitionIdx++)
      {
        getSPAbsPartIdx( uiAbsPartIdx, iSPWidth, iSPHeight, iPartitionIdx, iNumSPInOneLine, uiSPAddr);
#if QC_SUB_PU_TMVP_EXT
        setInterDirSP(puhInterDirSP[uiSPListIndex][iPartitionIdx], uiSPAddr, iSPWidth, iSPHeight);
        getCUMvField( REF_PIC_LIST_0 )->setMvFieldSP( this, uiSPAddr, pcMvFieldSP[uiSPListIndex][2*iPartitionIdx], iSPWidth, iSPHeight);
        getCUMvField( REF_PIC_LIST_1 )->setMvFieldSP( this, uiSPAddr, pcMvFieldSP[uiSPListIndex][2*iPartitionIdx + 1], iSPWidth, iSPHeight);
#else
        setInterDirSP(puhInterDirSP[iPartitionIdx], uiSPAddr, iSPWidth, iSPHeight);
        getCUMvField( REF_PIC_LIST_0 )->setMvFieldSP( this, uiSPAddr, pcMvFieldSP[2*iPartitionIdx], iSPWidth, iSPHeight);
        getCUMvField( REF_PIC_LIST_1 )->setMvFieldSP( this, uiSPAddr, pcMvFieldSP[2*iPartitionIdx + 1], iSPWidth, iSPHeight);
#endif
      }
    }
    if ( ( getInterDir( uiAbsPartIdx ) == 3 ) && isBipredRestriction( uiPartIdx ) )
    {
      getCUMvField( REF_PIC_LIST_1 )->setAllMv( TComMv(0,0), ePartSize, uiAbsPartIdx, 0, uiPartIdx);
      getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, ePartSize, uiAbsPartIdx, 0, uiPartIdx);
      setInterDirSubParts( 1, uiAbsPartIdx, uiPartIdx, uiDepth);
    }
#if QC_SUB_PU_TMVP_EXT
    delete [] pcMvFieldSP[0];
    delete [] pcMvFieldSP[1];
    delete [] puhInterDirSP[0];
    delete [] puhInterDirSP[1];
#else
    delete [] pcMvFieldSP;
    delete [] puhInterDirSP;
#endif
  }
#endif
#if QC_FRUC_MERGE
  return( true );
#endif
}

#if QC_FRUC_MERGE
Bool TComDataCU::resetMVDandMV2Int( Bool bResetMV , TComPrediction * pPred )
#else
Void TComDataCU::resetMVDandMV2Int( Bool bResetMV )
#endif
{
  UInt uiPartOffset = ( getPic()->getNumPartInCU() >> ( getDepth(0) << 1 ) ) >> 2;
#if QC_FRUC_MERGE
  Bool bReset = true;
#endif

  PartSize ePart = getPartitionSize( 0 );
  UInt uiAbsPartIdx = 0;

  switch ( ePart )
  {
  case SIZE_2Nx2N:
    {
#if QC_FRUC_MERGE
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV , pPred );
#else
      resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV );
#endif
      break;
    }

  case SIZE_2NxN:
    {
#if QC_FRUC_MERGE
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV , pPred );
      uiAbsPartIdx += ( uiPartOffset << 1 );
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV , pPred );
#else
      resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV );
      uiAbsPartIdx += ( uiPartOffset << 1 );
      resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV );
#endif
      break;
    }

  case SIZE_Nx2N:
    {
#if QC_FRUC_MERGE
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV , pPred );
      uiAbsPartIdx += uiPartOffset;
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV , pPred );
#else
      resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV );
      uiAbsPartIdx += uiPartOffset;
      resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV );
#endif
      break;
    }

  case SIZE_NxN:
    {
      for ( Int iPartIdx = 0; iPartIdx < 4; iPartIdx++ )
      {
#if QC_FRUC_MERGE
        bReset &= resetMVDandMV2Int( uiAbsPartIdx , iPartIdx , bResetMV , pPred );
#else
        resetMVDandMV2Int( uiAbsPartIdx , iPartIdx , bResetMV );
#endif
        uiAbsPartIdx += uiPartOffset;
      }
      break;
    }

  case SIZE_2NxnU:
    {
#if QC_FRUC_MERGE
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV , pPred );
      uiAbsPartIdx += ( uiPartOffset >> 1 );
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV , pPred );
#else
      resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV );
      uiAbsPartIdx += ( uiPartOffset >> 1 );
      resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV );
#endif
      break;
    }

  case SIZE_2NxnD:
    {
#if QC_FRUC_MERGE
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV , pPred );
      uiAbsPartIdx += ( uiPartOffset >> 1 ) + ( uiPartOffset << 1 );
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV , pPred );
#else
      resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV );
      uiAbsPartIdx += ( uiPartOffset >> 1 ) + ( uiPartOffset << 1 );
      resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV );
#endif
      break;
    }


  case SIZE_nLx2N:
    {
#if QC_FRUC_MERGE
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV , pPred );
      uiAbsPartIdx += ( uiPartOffset >> 2 );
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV , pPred );
#else
      resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV );
      uiAbsPartIdx += ( uiPartOffset >> 2 );
      resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV );
#endif
      break;
    }

  case SIZE_nRx2N:
    {
#if QC_FRUC_MERGE
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV , pPred );
      uiAbsPartIdx += uiPartOffset + ( uiPartOffset >> 2 );
      bReset &= resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV , pPred );
#else
      resetMVDandMV2Int( uiAbsPartIdx , 0 , bResetMV );
      uiAbsPartIdx += uiPartOffset + ( uiPartOffset >> 2 );
      resetMVDandMV2Int( uiAbsPartIdx , 1 , bResetMV );
#endif
      break;
    }
  default:
    assert( 0 );
    break;
  }

#if QC_FRUC_MERGE
  return( bReset );
#endif
}
#endif

#if QC_FRUC_MERGE
UInt TComDataCU::getCtxFRUCMgrMode( UInt uiAbsPartIdx )
{
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  UInt        uiCtx = 0;

  // Get BCBP of left PU
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx    = ( pcTempCU ) ? pcTempCU->getFRUCMgrMode( uiTempPartIdx ) > 0 : 0;

  // Get BCBP of above PU
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx   += ( pcTempCU ) ? pcTempCU->getFRUCMgrMode( uiTempPartIdx ) > 0 : 0;

  return( uiCtx );
}

UInt TComDataCU::getCtxFRUCME( UInt uiAbsPartIdx )
{
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  UInt        uiCtx = 0;

  // Get BCBP of left PU
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx    = ( pcTempCU ) ? pcTempCU->getFRUCMgrMode( uiTempPartIdx ) == QC_FRUC_MERGE_BILATERALMV : 0;

  // Get BCBP of above PU
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx   += ( pcTempCU ) ? pcTempCU->getFRUCMgrMode( uiTempPartIdx ) == QC_FRUC_MERGE_BILATERALMV : 0;

  return( uiCtx );
}

Bool TComDataCU::getMvPair( RefPicList eCurRefPicList , const TComMvField & rCurMvField , TComMvField & rMvPair )
{
  Int nTargetRefIdx = getSlice()->getRefIdx4MVPair( eCurRefPicList , rCurMvField.getRefIdx() );
  if( nTargetRefIdx < 0 )
    return( false );

  RefPicList eTarRefPicList = ( RefPicList )( 1 - ( Int )eCurRefPicList );
  Int nCurPOC = getSlice()->getPOC();
  Int nRefPOC = getSlice()->getRefPOC( eCurRefPicList , rCurMvField.getRefIdx() );
  Int nTargetPOC = getSlice()->getRefPOC( eTarRefPicList , nTargetRefIdx );
  Int nScale = xGetDistScaleFactor( nCurPOC , nTargetPOC , nCurPOC , nRefPOC );
  rMvPair.getMv() = rCurMvField.getMv().scaleMv( nScale );
  rMvPair.setRefIdx( nTargetRefIdx );

  return( true );
}
#endif

UInt TComDataCU::getCtxInterDir( UInt uiAbsPartIdx )
{
#if QC_LARGE_CTU
  return( Clip3( 0 , 3 , 4 - g_aucConvertToBit[getWidth( uiAbsPartIdx )] ) );
#else
  return getDepth( uiAbsPartIdx );
#endif
}

Void TComDataCU::setCbfSubParts( UInt uiCbfY, UInt uiCbfU, UInt uiCbfV, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  memset( m_puhCbf[0] + uiAbsPartIdx, uiCbfY, sizeof( UChar ) * uiCurrPartNumb );
  memset( m_puhCbf[1] + uiAbsPartIdx, uiCbfU, sizeof( UChar ) * uiCurrPartNumb );
  memset( m_puhCbf[2] + uiAbsPartIdx, uiCbfV, sizeof( UChar ) * uiCurrPartNumb );
}

Void TComDataCU::setCbfSubParts( UInt uiCbf, TextType eTType, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  memset( m_puhCbf[g_aucConvertTxtTypeToIdx[eTType]] + uiAbsPartIdx, uiCbf, sizeof( UChar ) * uiCurrPartNumb );
}

/** Sets a coded block flag for all sub-partitions of a partition
 * \param uiCbf The value of the coded block flag to be set
 * \param eTType
 * \param uiAbsPartIdx
 * \param uiPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TComDataCU::setCbfSubParts ( UInt uiCbf, TextType eTType, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<UChar>( uiCbf, m_puhCbf[g_aucConvertTxtTypeToIdx[eTType]], uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setDepthSubParts( UInt uiDepth, UInt uiAbsPartIdx )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  memset( m_puhDepth + uiAbsPartIdx, uiDepth, sizeof(UChar)*uiCurrPartNumb );
}

Bool TComDataCU::isFirstAbsZorderIdxInDepth (UInt uiAbsPartIdx, UInt uiDepth)
{
  UInt uiPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  return (((m_uiAbsIdxInLCU + uiAbsPartIdx)% uiPartNumb) == 0);
}

Void TComDataCU::setPartSizeSubParts( PartSize eMode, UInt uiAbsPartIdx, UInt uiDepth )
{
  assert( sizeof( *m_pePartSize) == 1 );
  memset( m_pePartSize + uiAbsPartIdx, eMode, m_pcPic->getNumPartInCU() >> ( 2 * uiDepth ) );
}

Void TComDataCU::setCUTransquantBypassSubParts( Bool flag, UInt uiAbsPartIdx, UInt uiDepth )
{
  memset( m_CUTransquantBypass + uiAbsPartIdx, flag, m_pcPic->getNumPartInCU() >> ( 2 * uiDepth ) );
}

Void TComDataCU::setSkipFlagSubParts( Bool skip, UInt absPartIdx, UInt depth )
{
  assert( sizeof( *m_skipFlag) == 1 );
  memset( m_skipFlag + absPartIdx, skip, m_pcPic->getNumPartInCU() >> ( 2 * depth ) );
}
#if ROT_TR
Void TComDataCU::setROTIdxSubParts( Char ROTIdx, UInt absPartIdx, UInt depth  )
{
  assert( sizeof( *m_ROTIdx) == 1 );
  memset( m_ROTIdx + absPartIdx, ROTIdx, m_pcPic->getNumPartInCU() >> ( 2 * depth ) );
}
#endif
#if CU_LEVEL_MPI
Void TComDataCU::setMPIIdxSubParts( Char MPIIdx, UInt absPartIdx, UInt depth  )
{
  assert( sizeof( *m_MPIIdx) == 1 );
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (depth << 1);
  memset(  m_MPIIdx + absPartIdx, MPIIdx, sizeof(Char)*uiCurrPartNumb );
}
#endif
#if QC_IMV
Void TComDataCU::setiMVFlagSubParts( Bool iMV, UInt absPartIdx, UInt depth )
{
  assert( sizeof( *m_iMVFlag) == 1 );
  memset( m_iMVFlag + absPartIdx, iMV, m_pcPic->getNumPartInCU() >> ( 2 * depth ) );
}

Void TComDataCU::setiMVCandNumSubParts( Char ciMVCandNum, UInt absPartIdx, UInt depth )
{
  assert( sizeof( *m_piMVCandNum) == 1 );
  memset( m_piMVCandNum + absPartIdx, ciMVCandNum, m_pcPic->getNumPartInCU() >> ( 2 * depth ) );
}
#endif

#if QC_OBMC
Void TComDataCU::setOBMCFlagSubParts( Bool OBMC, UInt absPartIdx, UInt depth )
{
  assert( sizeof( *m_OBMCFlag) == 1 );
  memset( m_OBMCFlag + absPartIdx, OBMC, m_pcPic->getNumPartInCU() >> ( 2 * depth ) );
}

Bool TComDataCU::isOBMCFlagCoded( UInt uiAbsPartIdx )
{
  if ( isIntra( uiAbsPartIdx ) || ( getMergeFlag( uiAbsPartIdx ) && getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N ) || getWidth( uiAbsPartIdx ) > QC_AOBMC_MAXCUSIZE )
  {
    return false;
  }
  else
  {
    return true;
  }
}
#endif

Void TComDataCU::setPredModeSubParts( PredMode eMode, UInt uiAbsPartIdx, UInt uiDepth )
{
  assert( sizeof( *m_pePredMode) == 1 );
  memset( m_pePredMode + uiAbsPartIdx, eMode, m_pcPic->getNumPartInCU() >> ( 2 * uiDepth ) );
}

Void TComDataCU::setQPSubCUs( Int qp, TComDataCU* pcCU, UInt absPartIdx, UInt depth, Bool &foundNonZeroCbf )
{
  UInt currPartNumb = m_pcPic->getNumPartInCU() >> (depth << 1);
  UInt currPartNumQ = currPartNumb >> 2;

  if(!foundNonZeroCbf)
  {
    if(pcCU->getDepth(absPartIdx) > depth)
    {
      for ( UInt partUnitIdx = 0; partUnitIdx < 4; partUnitIdx++ )
      {
        pcCU->setQPSubCUs( qp, pcCU, absPartIdx+partUnitIdx*currPartNumQ, depth+1, foundNonZeroCbf );
      }
    }
    else
    {
      if(pcCU->getCbf( absPartIdx, TEXT_LUMA ) || pcCU->getCbf( absPartIdx, TEXT_CHROMA_U ) || pcCU->getCbf( absPartIdx, TEXT_CHROMA_V ) )
      {
        foundNonZeroCbf = true;
      }
      else
      {
        setQPSubParts(qp, absPartIdx, depth);
      }
    }
  }
}

Void TComDataCU::setQPSubParts( Int qp, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  TComSlice * pcSlice = getPic()->getSlice(getPic()->getCurrSliceIdx());

  for(UInt uiSCUIdx = uiAbsPartIdx; uiSCUIdx < uiAbsPartIdx+uiCurrPartNumb; uiSCUIdx++)
  {
    if( m_pcPic->getCU( getAddr() )->getSliceSegmentStartCU(uiSCUIdx+getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr() )
    {
      m_phQP[uiSCUIdx] = qp;
    }
  }
}

Void TComDataCU::setLumaIntraDirSubParts( UInt uiDir, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  
  memset( m_puhLumaIntraDir + uiAbsPartIdx, uiDir, sizeof(UChar)*uiCurrPartNumb );
}

template<typename T>
Void TComDataCU::setSubPart( T uiParameter, T* puhBaseLCU, UInt uiCUAddr, UInt uiCUDepth, UInt uiPUIdx )
{
  assert( sizeof(T) == 1 ); // Using memset() works only for types of size 1
#if QC_OBMC
  UInt uiCurrPartNum = (m_pcPic->getNumPartInCU() >> (2 * uiCUDepth));
  UInt uiCurrPartNumQ = uiCurrPartNum >> 2;
#else
  UInt uiCurrPartNumQ = (m_pcPic->getNumPartInCU() >> (2 * uiCUDepth)) >> 2;
#endif
  switch ( m_pePartSize[ uiCUAddr ] )
  {
    case SIZE_2Nx2N:
#if QC_OBMC
      memset( puhBaseLCU + uiCUAddr, uiParameter, uiCurrPartNum );
#else
      memset( puhBaseLCU + uiCUAddr, uiParameter, 4 * uiCurrPartNumQ );
#endif
      break;
    case SIZE_2NxN:
      memset( puhBaseLCU + uiCUAddr, uiParameter, 2 * uiCurrPartNumQ );
      break;
    case SIZE_Nx2N:
      memset( puhBaseLCU + uiCUAddr, uiParameter, uiCurrPartNumQ );
      memset( puhBaseLCU + uiCUAddr + 2 * uiCurrPartNumQ, uiParameter, uiCurrPartNumQ );
      break;
    case SIZE_NxN:
      memset( puhBaseLCU + uiCUAddr, uiParameter, uiCurrPartNumQ ); 
      break;
    case SIZE_2NxnU:
      if ( uiPUIdx == 0 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 1) );                      
        memset( puhBaseLCU + uiCUAddr + uiCurrPartNumQ, uiParameter, (uiCurrPartNumQ >> 1) );                      
      }
      else if ( uiPUIdx == 1 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 1) );                      
        memset( puhBaseLCU + uiCUAddr + uiCurrPartNumQ, uiParameter, ((uiCurrPartNumQ >> 1) + (uiCurrPartNumQ << 1)) );                      
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_2NxnD:
      if ( uiPUIdx == 0 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, ((uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1)) );                      
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1) + uiCurrPartNumQ, uiParameter, (uiCurrPartNumQ >> 1) );                      
      }
      else if ( uiPUIdx == 1 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 1) );                      
        memset( puhBaseLCU + uiCUAddr + uiCurrPartNumQ, uiParameter, (uiCurrPartNumQ >> 1) );                      
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nLx2N:
      if ( uiPUIdx == 0 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) ); 
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ >> 2) ); 
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) ); 
      }
      else if ( uiPUIdx == 1 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) ); 
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ >> 2) ); 
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) ); 
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nRx2N:
      if ( uiPUIdx == 0 )
      {      
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) );                           
        memset( puhBaseLCU + uiCUAddr + uiCurrPartNumQ + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );                           
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) );                           
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1) + uiCurrPartNumQ + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );                           
      }
      else if ( uiPUIdx == 1 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 2) );                           
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );                           
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ >> 2) );                           
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );                          
      }
      else
      {
        assert(0);
      }
      break;
    default:
      assert( 0 );
  }
}

Void TComDataCU::setMergeFlagSubParts ( Bool bMergeFlag, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart( bMergeFlag, m_pbMergeFlag, uiAbsPartIdx, uiDepth, uiPartIdx );
}

#if QC_FRUC_MERGE
Void TComDataCU::setFRUCMgrModeSubParts ( UChar uhFRUCMgrMode, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart( uhFRUCMgrMode, m_puhFRUCMgrMode, uiAbsPartIdx, uiDepth, uiPartIdx );
}
#endif

Void TComDataCU::setMergeIndexSubParts ( UInt uiMergeIndex, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<UChar>( uiMergeIndex, m_puhMergeIndex, uiAbsPartIdx, uiDepth, uiPartIdx );
}

#if QC_SUB_PU_TMVP
Void TComDataCU::setMergeTypeSubParts( UChar eMergeType, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<UChar>( eMergeType, m_peMergeType, uiAbsPartIdx, uiDepth, uiPartIdx );
}
#endif

Void TComDataCU::setChromIntraDirSubParts( UInt uiDir, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  
  memset( m_puhChromaIntraDir + uiAbsPartIdx, uiDir, sizeof(UChar)*uiCurrPartNumb );
}

Void TComDataCU::setInterDirSubParts( UInt uiDir, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<UChar>( uiDir, m_puhInterDir, uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setMVPIdxSubParts( Int iMVPIdx, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<Char>( iMVPIdx, m_apiMVPIdx[eRefPicList], uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setMVPNumSubParts( Int iMVPNum, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<Char>( iMVPNum, m_apiMVPNum[eRefPicList], uiAbsPartIdx, uiDepth, uiPartIdx );
}

#if QC_EMT
Void TComDataCU::setEmtTuIdxSubParts( UInt uiTrMode, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  
  memset( m_puhEmtTuIdx + uiAbsPartIdx, uiTrMode, sizeof(UChar)*uiCurrPartNumb );
}

Void TComDataCU::setEmtCuFlagSubParts( UInt uiTrMode, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  
  memset( m_puhEmtCuFlag + uiAbsPartIdx, uiTrMode, sizeof(UChar)*uiCurrPartNumb );
}
#endif

Void TComDataCU::setTrIdxSubParts( UInt uiTrIdx, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  
  memset( m_puhTrIdx + uiAbsPartIdx, uiTrIdx, sizeof(UChar)*uiCurrPartNumb );
}

Void TComDataCU::setTransformSkipSubParts( UInt useTransformSkipY, UInt useTransformSkipU, UInt useTransformSkipV, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  memset( m_puhTransformSkip[0] + uiAbsPartIdx, useTransformSkipY, sizeof( UChar ) * uiCurrPartNumb );
  memset( m_puhTransformSkip[1] + uiAbsPartIdx, useTransformSkipU, sizeof( UChar ) * uiCurrPartNumb );
  memset( m_puhTransformSkip[2] + uiAbsPartIdx, useTransformSkipV, sizeof( UChar ) * uiCurrPartNumb );
}

Void TComDataCU::setTransformSkipSubParts( UInt useTransformSkip, TextType eType, UInt uiAbsPartIdx, UInt uiDepth)
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  memset( m_puhTransformSkip[g_aucConvertTxtTypeToIdx[eType]] + uiAbsPartIdx, useTransformSkip, sizeof( UChar ) * uiCurrPartNumb );
}

#if KLT_COMMON
Void TComDataCU::setKLTFlagSubParts(UInt useKLTY, UInt useKLTU, UInt useKLTV, UInt uiAbsPartIdx, UInt uiDepth)
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  memset(m_puhKLTFlag[0] + uiAbsPartIdx, useKLTY, sizeof(UChar)* uiCurrPartNumb);
  memset(m_puhKLTFlag[1] + uiAbsPartIdx, useKLTU, sizeof(UChar)* uiCurrPartNumb);
  memset(m_puhKLTFlag[2] + uiAbsPartIdx, useKLTV, sizeof(UChar)* uiCurrPartNumb);
}

Void TComDataCU::setKLTFlagSubParts(UInt useKLTY, TextType eType, UInt uiAbsPartIdx, UInt uiDepth)
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  memset(m_puhKLTFlag[g_aucConvertTxtTypeToIdx[eType]] + uiAbsPartIdx, useKLTY, sizeof(UChar)* uiCurrPartNumb);
}
#endif

Void TComDataCU::setSizeSubParts( UInt uiWidth, UInt uiHeight, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  
#if QC_LARGE_CTU
  for( UInt n = 0 ; n < uiCurrPartNumb ; n++ )
  {
    m_puhWidth[uiAbsPartIdx+n] = uiWidth;
    m_puhHeight[uiAbsPartIdx+n] = uiHeight;
  }
#else
  memset( m_puhWidth  + uiAbsPartIdx, uiWidth,  sizeof(UChar)*uiCurrPartNumb );
  memset( m_puhHeight + uiAbsPartIdx, uiHeight, sizeof(UChar)*uiCurrPartNumb );
#endif
}

UChar TComDataCU::getNumPartitions()
{
  UChar iNumPart = 0;
  
  switch ( m_pePartSize[0] )
  {
    case SIZE_2Nx2N:    iNumPart = 1; break;
    case SIZE_2NxN:     iNumPart = 2; break;
    case SIZE_Nx2N:     iNumPart = 2; break;
    case SIZE_NxN:      iNumPart = 4; break;
    case SIZE_2NxnU:    iNumPart = 2; break;
    case SIZE_2NxnD:    iNumPart = 2; break;
    case SIZE_nLx2N:    iNumPart = 2; break;
    case SIZE_nRx2N:    iNumPart = 2; break;
    default:            assert (0);   break;
  }
  
  return  iNumPart;
}

Void TComDataCU::getPartIndexAndSize( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight )
{
  switch ( m_pePartSize[0] )
  {
    case SIZE_2NxN:
      riWidth = getWidth(0);      riHeight = getHeight(0) >> 1; ruiPartAddr = ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N:
      riWidth = getWidth(0) >> 1; riHeight = getHeight(0);      ruiPartAddr = ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 2;
      break;
    case SIZE_NxN:
      riWidth = getWidth(0) >> 1; riHeight = getHeight(0) >> 1; ruiPartAddr = ( m_uiNumPartition >> 2 ) * uiPartIdx;
      break;
    case SIZE_2NxnU:
      riWidth     = getWidth(0);
      riHeight    = ( uiPartIdx == 0 ) ?  getHeight(0) >> 2 : ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 );
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : m_uiNumPartition >> 3;
      break;
    case SIZE_2NxnD:
      riWidth     = getWidth(0);
      riHeight    = ( uiPartIdx == 0 ) ?  ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 ) : getHeight(0) >> 2;
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : (m_uiNumPartition >> 1) + (m_uiNumPartition >> 3);
      break;
    case SIZE_nLx2N:
      riWidth     = ( uiPartIdx == 0 ) ? getWidth(0) >> 2 : ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 );
      riHeight    = getHeight(0);
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : m_uiNumPartition >> 4;
      break;
    case SIZE_nRx2N:
      riWidth     = ( uiPartIdx == 0 ) ? ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 ) : getWidth(0) >> 2;
      riHeight    = getHeight(0);
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : (m_uiNumPartition >> 2) + (m_uiNumPartition >> 4);
      break;
    default:
      assert ( m_pePartSize[0] == SIZE_2Nx2N );
      riWidth = getWidth(0);      riHeight = getHeight(0);      ruiPartAddr = 0;
      break;
  }
}

#if QC_SUB_PU_TMVP
Void TComDataCU::getPartIndexAndSize( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight, UInt uiAbsPartIdx, Bool bLCU)
{
  UInt uiNumPartition  = bLCU ? (getWidth(uiAbsPartIdx)*getHeight(uiAbsPartIdx) >> 4) : m_uiNumPartition;
  UInt  uiTmpAbsPartIdx  = bLCU ? uiAbsPartIdx : 0;

  switch ( m_pePartSize[uiTmpAbsPartIdx] )
  {
  case SIZE_2NxN:
    riWidth = getWidth( uiTmpAbsPartIdx );      riHeight = getHeight( uiTmpAbsPartIdx ) >> 1; ruiPartAddr = ( uiPartIdx == 0 )? 0 : uiNumPartition >> 1;
    break;
  case SIZE_Nx2N:
    riWidth = getWidth( uiTmpAbsPartIdx ) >> 1; riHeight = getHeight( uiTmpAbsPartIdx );      ruiPartAddr = ( uiPartIdx == 0 )? 0 : uiNumPartition >> 2;
    break;
  case SIZE_NxN:
    riWidth = getWidth( uiTmpAbsPartIdx ) >> 1; riHeight = getHeight( uiTmpAbsPartIdx ) >> 1; ruiPartAddr = ( uiNumPartition >> 2 ) * uiPartIdx;
    break;
  case SIZE_2NxnU:
    riWidth     = getWidth( uiTmpAbsPartIdx );
    riHeight    = ( uiPartIdx == 0 ) ?  getHeight( uiTmpAbsPartIdx ) >> 2 : ( getHeight( uiTmpAbsPartIdx ) >> 2 ) + ( getHeight( uiTmpAbsPartIdx ) >> 1 );
    ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : uiNumPartition >> 3;
    break;
  case SIZE_2NxnD:
    riWidth     = getWidth( uiTmpAbsPartIdx );
    riHeight    = ( uiPartIdx == 0 ) ?  ( getHeight( uiTmpAbsPartIdx ) >> 2 ) + ( getHeight( uiTmpAbsPartIdx ) >> 1 ) : getHeight( uiTmpAbsPartIdx ) >> 2;
    ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : (uiNumPartition >> 1) + (uiNumPartition >> 3);
    break;
  case SIZE_nLx2N:
    riWidth     = ( uiPartIdx == 0 ) ? getWidth( uiTmpAbsPartIdx ) >> 2 : ( getWidth( uiTmpAbsPartIdx ) >> 2 ) + ( getWidth( uiTmpAbsPartIdx ) >> 1 );
    riHeight    = getHeight( uiTmpAbsPartIdx );
    ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : uiNumPartition >> 4;
    break;
  case SIZE_nRx2N:
    riWidth     = ( uiPartIdx == 0 ) ? ( getWidth( uiTmpAbsPartIdx ) >> 2 ) + ( getWidth( uiTmpAbsPartIdx ) >> 1 ) : getWidth( uiTmpAbsPartIdx ) >> 2;
    riHeight    = getHeight( uiTmpAbsPartIdx );
    ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : (uiNumPartition >> 2) + (uiNumPartition >> 4);
    break;
  default:
    assert ( m_pePartSize[uiTmpAbsPartIdx] == SIZE_2Nx2N ); 
    riWidth = getWidth( uiTmpAbsPartIdx );      riHeight = getHeight( uiTmpAbsPartIdx );      ruiPartAddr = 0;
    break;
  }
}

#endif

#if QC_IC
Void TComDataCU::setICFlagSubParts( Bool bICFlag, UInt uiAbsPartIdx, UInt uiDepth )
{
  memset( m_pbICFlag + uiAbsPartIdx, bICFlag, (m_pcPic->getNumPartInCU() >> ( 2 * uiDepth ))*sizeof(Bool) );
}

Bool TComDataCU::isICFlagCoded( UInt uiAbsPartIdx )
{
  if ( isIntra( uiAbsPartIdx ) || !getSlice()->getApplyIC() )
  {
    return false;
  }

  if( getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N && getMergeFlag( uiAbsPartIdx ) 
#if QC_FRUC_MERGE
    && getFRUCMgrMode( uiAbsPartIdx ) == QC_FRUC_MERGE_OFF 
#endif
    )
  {
    return false;
  }

#if QC_IC_SPDUP
  if( getPartitionSize( uiAbsPartIdx ) != SIZE_2Nx2N )
  {
    return false;
  }
#endif

  return true;
}
#endif

Void TComDataCU::getMvField ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, TComMvField& rcMvField )
{
  if ( pcCU == NULL )  // OUT OF BOUNDARY
  {
    TComMv  cZeroMv;
    rcMvField.setMvField( cZeroMv, NOT_VALID );
    return;
  }
  
  TComCUMvField*  pcCUMvField = pcCU->getCUMvField( eRefPicList );
  rcMvField.setMvField( pcCUMvField->getMv( uiAbsPartIdx ), pcCUMvField->getRefIdx( uiAbsPartIdx ) );
}

Void TComDataCU::deriveLeftRightTopIdxGeneral ( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT )
{
  ruiPartIdxLT = m_uiAbsIdxInLCU + uiAbsPartIdx;
  UInt uiPUWidth = 0;
  
  switch ( m_pePartSize[uiAbsPartIdx] )
  {
    case SIZE_2Nx2N: uiPUWidth = m_puhWidth[uiAbsPartIdx];  break;
    case SIZE_2NxN:  uiPUWidth = m_puhWidth[uiAbsPartIdx];   break;
    case SIZE_Nx2N:  uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 1;  break;
    case SIZE_NxN:   uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 1; break;
    case SIZE_2NxnU:   uiPUWidth = m_puhWidth[uiAbsPartIdx]; break;
    case SIZE_2NxnD:   uiPUWidth = m_puhWidth[uiAbsPartIdx]; break;
    case SIZE_nLx2N:   
      if ( uiPartIdx == 0 )
      {
        uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 2; 
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUWidth = (m_puhWidth[uiAbsPartIdx]  >> 1) + (m_puhWidth[uiAbsPartIdx]  >> 2); 
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nRx2N:   
      if ( uiPartIdx == 0 )
      {
        uiPUWidth = (m_puhWidth[uiAbsPartIdx]  >> 1) + (m_puhWidth[uiAbsPartIdx]  >> 2); 
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 2; 
      }
      else
      {
        assert(0);
      }
      break;
    default:
      assert (0);
      break;
  }
  
  ruiPartIdxRT = g_auiRasterToZscan [g_auiZscanToRaster[ ruiPartIdxLT ] + uiPUWidth / m_pcPic->getMinCUWidth() - 1 ];
}

Void TComDataCU::deriveLeftBottomIdxGeneral( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLB )
{
  UInt uiPUHeight = 0;
  switch ( m_pePartSize[uiAbsPartIdx] )
  {
    case SIZE_2Nx2N: uiPUHeight = m_puhHeight[uiAbsPartIdx];    break;
    case SIZE_2NxN:  uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 1;    break;
    case SIZE_Nx2N:  uiPUHeight = m_puhHeight[uiAbsPartIdx];  break;
    case SIZE_NxN:   uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 1;    break;
    case SIZE_2NxnU: 
      if ( uiPartIdx == 0 )
      {
        uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 2;    
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUHeight = (m_puhHeight[uiAbsPartIdx] >> 1) + (m_puhHeight[uiAbsPartIdx] >> 2);    
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_2NxnD: 
      if ( uiPartIdx == 0 )
      {
        uiPUHeight = (m_puhHeight[uiAbsPartIdx] >> 1) + (m_puhHeight[uiAbsPartIdx] >> 2);    
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 2;    
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nLx2N: uiPUHeight = m_puhHeight[uiAbsPartIdx];  break;
    case SIZE_nRx2N: uiPUHeight = m_puhHeight[uiAbsPartIdx];  break;
    default:
      assert (0);
      break;
  }
  
  ruiPartIdxLB      = g_auiRasterToZscan [g_auiZscanToRaster[ m_uiAbsIdxInLCU + uiAbsPartIdx ] + ((uiPUHeight / m_pcPic->getMinCUHeight()) - 1)*m_pcPic->getNumPartInWidth()];
}

Void TComDataCU::deriveLeftRightTopIdx ( UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT )
{
  ruiPartIdxLT = m_uiAbsIdxInLCU;
  ruiPartIdxRT = g_auiRasterToZscan [g_auiZscanToRaster[ ruiPartIdxLT ] + m_puhWidth[0] / m_pcPic->getMinCUWidth() - 1 ];
  
  switch ( m_pePartSize[0] )
  {
    case SIZE_2Nx2N:                                                                                                                                break;
    case SIZE_2NxN:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1; ruiPartIdxRT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 2; ruiPartIdxRT -= ( uiPartIdx == 1 )? 0 : m_uiNumPartition >> 2;
      break;
    case SIZE_NxN:
      ruiPartIdxLT += ( m_uiNumPartition >> 2 ) * uiPartIdx;         ruiPartIdxRT +=  ( m_uiNumPartition >> 2 ) * ( uiPartIdx - 1 );
      break;
    case SIZE_2NxnU:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 3;
      ruiPartIdxRT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 3;
      break;
    case SIZE_2NxnD:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : ( m_uiNumPartition >> 1 ) + ( m_uiNumPartition >> 3 );
      ruiPartIdxRT += ( uiPartIdx == 0 )? 0 : ( m_uiNumPartition >> 1 ) + ( m_uiNumPartition >> 3 );
      break;
    case SIZE_nLx2N:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 4;
      ruiPartIdxRT -= ( uiPartIdx == 1 )? 0 : ( m_uiNumPartition >> 2 ) + ( m_uiNumPartition >> 4 );
      break;
    case SIZE_nRx2N:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : ( m_uiNumPartition >> 2 ) + ( m_uiNumPartition >> 4 );
      ruiPartIdxRT -= ( uiPartIdx == 1 )? 0 : m_uiNumPartition >> 4;
      break;
    default:
      assert (0);
      break;
  }
  
}

Void TComDataCU::deriveLeftBottomIdx( UInt  uiPartIdx,      UInt&      ruiPartIdxLB )
{
  ruiPartIdxLB      = g_auiRasterToZscan [g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + ( ((m_puhHeight[0] / m_pcPic->getMinCUHeight())>>1) - 1)*m_pcPic->getNumPartInWidth()];
  
  switch ( m_pePartSize[0] )
  {
    case SIZE_2Nx2N:
      ruiPartIdxLB += m_uiNumPartition >> 1;
      break;
    case SIZE_2NxN:
      ruiPartIdxLB += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N:
      ruiPartIdxLB += ( uiPartIdx == 0 )? m_uiNumPartition >> 1 : (m_uiNumPartition >> 2)*3;
      break;
    case SIZE_NxN:
      ruiPartIdxLB += ( m_uiNumPartition >> 2 ) * uiPartIdx;
      break;
    case SIZE_2NxnU:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? -((Int)m_uiNumPartition >> 3) : m_uiNumPartition >> 1;
      break;
    case SIZE_2NxnD:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 2) + (m_uiNumPartition >> 3): m_uiNumPartition >> 1;
      break;
    case SIZE_nLx2N:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? m_uiNumPartition >> 1 : (m_uiNumPartition >> 1) + (m_uiNumPartition >> 4);
      break;
    case SIZE_nRx2N:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? m_uiNumPartition >> 1 : (m_uiNumPartition >> 1) + (m_uiNumPartition >> 2) + (m_uiNumPartition >> 4);
      break;
    default:
      assert (0);
      break;
  }
}

/** Derives the partition index of neighbouring bottom right block
 * \param [in]  eCUMode
 * \param [in]  uiPartIdx 
 * \param [out] ruiPartIdxRB 
 */
Void TComDataCU::deriveRightBottomIdx( UInt  uiPartIdx,      UInt&      ruiPartIdxRB )
{
  ruiPartIdxRB      = g_auiRasterToZscan [g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + ( ((m_puhHeight[0] / m_pcPic->getMinCUHeight())>>1) - 1)*m_pcPic->getNumPartInWidth() +  m_puhWidth[0] / m_pcPic->getMinCUWidth() - 1];

  switch ( m_pePartSize[0] )
  {
    case SIZE_2Nx2N:  
      ruiPartIdxRB += m_uiNumPartition >> 1;    
      break;
    case SIZE_2NxN:  
      ruiPartIdxRB += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;   
      break;
    case SIZE_Nx2N:  
      ruiPartIdxRB += ( uiPartIdx == 0 )? m_uiNumPartition >> 2 : (m_uiNumPartition >> 1);   
      break;
    case SIZE_NxN:   
      ruiPartIdxRB += ( m_uiNumPartition >> 2 ) * ( uiPartIdx - 1 );   
      break;
    case SIZE_2NxnU:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? -((Int)m_uiNumPartition >> 3) : m_uiNumPartition >> 1;
      break;
    case SIZE_2NxnD:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 2) + (m_uiNumPartition >> 3): m_uiNumPartition >> 1;
      break;
    case SIZE_nLx2N:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 3) + (m_uiNumPartition >> 4): m_uiNumPartition >> 1;
      break;
    case SIZE_nRx2N:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 2) + (m_uiNumPartition >> 3) + (m_uiNumPartition >> 4) : m_uiNumPartition >> 1;
      break;
    default:
      assert (0);
      break;
  }
}

Void TComDataCU::deriveLeftRightTopIdxAdi ( UInt& ruiPartIdxLT, UInt& ruiPartIdxRT, UInt uiPartOffset, UInt uiPartDepth )
{
  UInt uiNumPartInWidth = (m_puhWidth[0]/m_pcPic->getMinCUWidth())>>uiPartDepth;
  ruiPartIdxLT = m_uiAbsIdxInLCU + uiPartOffset;
  ruiPartIdxRT = g_auiRasterToZscan[ g_auiZscanToRaster[ ruiPartIdxLT ] + uiNumPartInWidth - 1 ];
}

Void TComDataCU::deriveLeftBottomIdxAdi( UInt& ruiPartIdxLB, UInt uiPartOffset, UInt uiPartDepth )
{
  UInt uiAbsIdx;
  UInt uiMinCuWidth, uiWidthInMinCus;
  
  uiMinCuWidth    = getPic()->getMinCUWidth();
  uiWidthInMinCus = (getWidth(0)/uiMinCuWidth)>>uiPartDepth;
  uiAbsIdx        = getZorderIdxInCU()+uiPartOffset+(m_uiNumPartition>>(uiPartDepth<<1))-1;
  uiAbsIdx        = g_auiZscanToRaster[uiAbsIdx]-(uiWidthInMinCus-1);
  ruiPartIdxLB    = g_auiRasterToZscan[uiAbsIdx];
}

Bool TComDataCU::hasEqualMotion( UInt uiAbsPartIdx, TComDataCU* pcCandCU, UInt uiCandAbsPartIdx )
{

  if ( getInterDir( uiAbsPartIdx ) != pcCandCU->getInterDir( uiCandAbsPartIdx ) )
  {
    return false;
  }

  for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
  {
    if ( getInterDir( uiAbsPartIdx ) & ( 1 << uiRefListIdx ) )
    {
      if ( getCUMvField( RefPicList( uiRefListIdx ) )->getMv( uiAbsPartIdx )     != pcCandCU->getCUMvField( RefPicList( uiRefListIdx ) )->getMv( uiCandAbsPartIdx ) || 
        getCUMvField( RefPicList( uiRefListIdx ) )->getRefIdx( uiAbsPartIdx ) != pcCandCU->getCUMvField( RefPicList( uiRefListIdx ) )->getRefIdx( uiCandAbsPartIdx ) )
      {
        return false;
      }
    }
  }

  return true;
}

#if QC_SUB_PU_TMVP
Void TComDataCU::get1stTvFromSpatialNeighbor ( UInt uiAbsPartIdx, UInt uiPUIdx, Bool &bTvAva, Int &iPOC, TComMv &rcMv)
{
  TComPic *pColPic = getSlice()->getRefPic( RefPicList(getSlice()->isInterB() ? 1-getSlice()->getColFromL0Flag() : 0), getSlice()->getColRefIdx());

  iPOC             = pColPic->getPOC();

  UInt uiAbsPartAddr = m_uiAbsIdxInLCU + uiAbsPartIdx;

  UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB;
  deriveLeftRightTopIdxGeneral( uiAbsPartIdx, uiPUIdx, uiPartIdxLT, uiPartIdxRT );
  deriveLeftBottomIdxGeneral  ( uiAbsPartIdx, uiPUIdx, uiPartIdxLB );

  TComMvField tempMvField;
  bTvAva            = false;

  //left
  UInt uiLeftPartIdx = 0;
  TComDataCU* pcCULeft = 0;
  pcCULeft = getPULeft( uiLeftPartIdx, uiPartIdxLB );

  // above
  UInt uiAbovePartIdx = 0;
  TComDataCU* pcCUAbove = 0;
  pcCUAbove = getPUAbove( uiAbovePartIdx, uiPartIdxRT );

   // above right
  UInt uiAboveRightPartIdx = 0;
  TComDataCU* pcCUAboveRight = 0;
  pcCUAboveRight = getPUAboveRight( uiAboveRightPartIdx, uiPartIdxRT );

      //left bottom
  UInt uiLeftBottomPartIdx = 0;
  TComDataCU* pcCULeftBottom = 0;
  pcCULeftBottom = this->getPUBelowLeft( uiLeftBottomPartIdx, uiPartIdxLB );

  // above left
  UInt uiAboveLeftPartIdx = 0;
  TComDataCU* pcCUAboveLeft = 0;
  pcCUAboveLeft = getPUAboveLeft( uiAboveLeftPartIdx, uiAbsPartAddr );

  TComDataCU* pcNeighorPU[5]   = {pcCULeft,      pcCUAbove,        pcCUAboveRight,       pcCULeftBottom,       pcCUAboveLeft,};
  UInt        uiNeighorPartIdx[5] = {uiLeftPartIdx, uiAbovePartIdx,   uiAboveRightPartIdx,  uiLeftBottomPartIdx,  uiAboveLeftPartIdx,};
  for (UInt uiN=0; uiN< 5; uiN++) 
    for( UInt uiCurrRefListId = 0; uiCurrRefListId < (getSlice()->getSliceType() == B_SLICE ?  2 : 1 ) ; uiCurrRefListId++ )
    {
      RefPicList  eCurrRefPicList = RefPicList( RefPicList( getSlice()->isInterB() ? (getSlice()->getColFromL0Flag()? uiCurrRefListId: 1- uiCurrRefListId) : uiCurrRefListId ));

      if ( pcNeighorPU[uiN] && !pcNeighorPU[uiN]->isIntra( uiNeighorPartIdx[uiN] ) )
      {
        if ( pcNeighorPU[uiN]->getInterDir( uiNeighorPartIdx[uiN]  ) & (1<<eCurrRefPicList) )
        {
          pcNeighorPU[uiN]->getMvField( pcNeighorPU[uiN], uiNeighorPartIdx[uiN], eCurrRefPicList, tempMvField);
          iPOC = getSlice()->getRefPic( eCurrRefPicList, tempMvField.getRefIdx())->getPOC() ;
          rcMv  = tempMvField.getMv();
          bTvAva = true;
          return;
        }
      }
    }
}
#endif

/** Constructs a list of merging candidates
 * \param uiAbsPartIdx
 * \param uiPUIdx 
 * \param uiDepth
 * \param pcMvFieldNeighbours
 * \param puhInterDirNeighbours
 * \param numValidMergeCand
 * \param peMergeTypeNeighbors
 * \param pcMvFieldSP
 * \param puhInterDirSP
 * \param uiDecCurrAbsPartIdx
 * \param pDecCurrCU
  */


#if QC_SUB_PU_TMVP_EXT
Void TComDataCU::getNeighboringMvField(TComDataCU *pcCU, UInt uiPartIdx, TComMvField *cMvField,UChar *pucInterDir)
{
   Int iRefPOCSrc,iRefPOCMirror;
   RefPicList eRefPicListSrc /*, eRefPicListMirror*/;
   UInt uiMvIdxSrc,uiMvIdxMirror;
   TComMvField cMvFieldTemp;
   if (pcCU->getInterDir(uiPartIdx)==3)
   {
     *pucInterDir=3;
     for (uiMvIdxSrc=0;uiMvIdxSrc<2;uiMvIdxSrc++)
     {
       eRefPicListSrc = (RefPicList)uiMvIdxSrc;
       pcCU->getMvField(pcCU,uiPartIdx,eRefPicListSrc, cMvFieldTemp);
       if (cMvFieldTemp.getRefIdx() ==0)
       {
         cMvField[uiMvIdxSrc] = cMvFieldTemp;
       }
       else
       {
         iRefPOCSrc    = m_pcSlice->getRefPOC( eRefPicListSrc, cMvFieldTemp.getRefIdx() );       
         iRefPOCMirror = m_pcSlice->getRefPOC( eRefPicListSrc, 0 );
         Int iScale = xGetDistScaleFactor( getSlice()->getPOC(), iRefPOCMirror, getSlice()->getPOC(), iRefPOCSrc );
         if ( iScale == 4096 )
         {
           cMvField[uiMvIdxSrc].setMvField(cMvFieldTemp.getMv(), 0);
         }
         else
         {
           cMvField[uiMvIdxSrc].setMvField(cMvFieldTemp.getMv().scaleMv( iScale ), 0);
         }
       }
     }
   }
   else 
   {
     if (pcCU->getInterDir(uiPartIdx)&1)
     {
       eRefPicListSrc    = REF_PIC_LIST_0;
       //eRefPicListMirror = REF_PIC_LIST_1;
       uiMvIdxSrc = 0;
     }
     else
     {
       eRefPicListSrc    = REF_PIC_LIST_1;
       //eRefPicListMirror = REF_PIC_LIST_0;
       uiMvIdxSrc = 1;
     }
     *pucInterDir=uiMvIdxSrc+1;
     uiMvIdxMirror = 1- uiMvIdxSrc;
     pcCU->getMvField(pcCU,uiPartIdx,eRefPicListSrc, cMvFieldTemp);
     iRefPOCSrc    = m_pcSlice->getRefPOC( eRefPicListSrc, cMvFieldTemp.getRefIdx() );       
     if (cMvFieldTemp.getRefIdx() ==0)
     {
       cMvField[uiMvIdxSrc] = cMvFieldTemp;
     }
     else
     {
       iRefPOCMirror = m_pcSlice->getRefPOC( eRefPicListSrc, 0 );
       Int iScale = xGetDistScaleFactor( getSlice()->getPOC(), iRefPOCMirror, getSlice()->getPOC(), iRefPOCSrc );
       if ( iScale == 4096 )
       {
         cMvField[uiMvIdxSrc].setMvField(cMvFieldTemp.getMv(), 0);
       }
       else
       {
         cMvField[uiMvIdxSrc].setMvField(cMvFieldTemp.getMv().scaleMv( iScale ), 0);
       }
     }
     TComMv cZeroMv;
     cZeroMv.setZero();
     cMvField[uiMvIdxMirror].setMvField(cZeroMv,-1);
  }
}

Void TComDataCU::generateMvField(TComMvField *cMvField,UChar* pucInterDir, UInt uiMvNum,TComMvField* cMvFieldMedian,UChar &ucInterDirMedian)
{
  UChar ucDisable = uiMvNum;
  TComMv cMv;
  ucInterDirMedian = 0;

  if (uiMvNum==0)
  {
    if (getSlice()->getSliceType() == P_SLICE)
    {
      ucInterDirMedian = 1;
      cMv.setZero();
      cMvFieldMedian[0].setMvField( cMv ,0);
      cMvFieldMedian[1].setMvField( cMv ,-1);
    }
    else
    {
      ucInterDirMedian = 3;
      cMv.setZero();
      cMvFieldMedian[0].setMvField( cMv ,0);
      cMvFieldMedian[1].setMvField( cMv ,0);
    }
    return;
  }
  for (UInt j=0;j<2;j++)
  {
    Int iExistMvNum =0;
    Int cMvX=0,cMvY=0;
    for (UInt i=0;i<uiMvNum;i++)
    {
      if (pucInterDir[i] & (j+1) && ucDisable!= i)
      {
        cMvX +=  cMvField[ (i<<1)+j ].getHor();
        cMvY +=  cMvField[ (i<<1)+j ].getVer();
        iExistMvNum++;
      }
    }
    if (iExistMvNum)
    {
      ucInterDirMedian |= (j+1);
      if (iExistMvNum==3)
      {
        cMv.set((Short)(cMvX*43/128), (Short)(cMvY *43/128));
      }
      else if (iExistMvNum==2)
      {
        cMv.set((Short)(cMvX/2), (Short)(cMvY/2));
      }
      else
      {
        cMv.set((Short)(cMvX), (Short)(cMvY));
      }
      cMvFieldMedian[j].setMvField(cMv,0);
    }
    else
    {
      cMv.setZero();
      cMvFieldMedian[j].setMvField(cMv,-1);
    }
  }
}




Bool TComDataCU::getInterMergeSubPURecursiveCandidate( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMvFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand
  , UChar*          peMergeTypeNeighbors  , TComMvField*    pcMvFieldSP[2] , UChar*          puhInterDirSP[2], Int iCount )
{
    Bool bAtmvpAva=false;
    Int iPartitionIdx     = 0;
    // compute the location of the current PU
    Int iCurrPosX, iCurrPosY;
    Int iWidth, iHeight;
    Int iPUWidth, iPUHeight, iNumPart, iNumPartLine;

    getPartPosition( uiPUIdx, iCurrPosX, iCurrPosY,      iWidth, iHeight);
    getSPPara      ( iWidth,  iHeight,  iNumPart, iNumPartLine, iPUWidth, iPUHeight);
    UInt uiSameCount=0;
    UInt uiSameCountATMVP=0;
    for (Int i=0; i <  iHeight; i += iPUHeight)
    {
      for (Int j = 0; j <  iWidth; j += iPUWidth)
      {
          TComMvField cMvField[6],cMvFieldMedian[2];
          UChar ucInterDir[3],ucInterDirMedian=0;
          UInt uiMVCount=0;
          UInt uiSPAddr=0;
          TComDataCU* pcCULeftMedian= 0,*pcCUAboveMedian= 0;
          UInt uiLeftPartIdx_median=0, uiAbovePartIdx_median=0;
          getSPAbsPartIdx(uiAbsPartIdx, iPUWidth, iPUHeight, iPartitionIdx, iNumPartLine, uiSPAddr);
          uiSPAddr += m_uiAbsIdxInLCU;
          //get left
          if (iPartitionIdx%iNumPartLine ==0)
          {
              for (UInt uiCurAddrY = i/iPUHeight; uiCurAddrY <iHeight/iPUHeight ; uiCurAddrY++)
              {
                UInt uiSPAddrCur=0;
                getSPAbsPartIdx(uiAbsPartIdx, iPUWidth, iPUHeight, uiCurAddrY* iNumPartLine, iNumPartLine, uiSPAddrCur);
                uiSPAddrCur += m_uiAbsIdxInLCU;

                pcCULeftMedian = getPULeft( uiLeftPartIdx_median, uiSPAddrCur );
                if ( pcCULeftMedian &&!pcCULeftMedian->isIntra( uiLeftPartIdx_median ))
                {
                  getNeighboringMvField(pcCULeftMedian,uiLeftPartIdx_median,cMvField,ucInterDir);
                  uiMVCount++;
                  break;
                }
              }
          }
          else
          {
            ucInterDir[0]= puhInterDirSP[1][iPartitionIdx-1];
            cMvField[0]=pcMvFieldSP[1][((iPartitionIdx-1)<<1)  ];
            cMvField[1]=pcMvFieldSP[1][((iPartitionIdx-1)<<1)+1];
            uiMVCount++;
          }
          //get above
          if (iPartitionIdx < iNumPartLine)
          {
              for (UInt uiCurAddrX = iPartitionIdx; uiCurAddrX <iNumPartLine ; uiCurAddrX++)
              {
                UInt uiSPAddrCur=0;
                getSPAbsPartIdx(uiAbsPartIdx, iPUWidth, iPUHeight, uiCurAddrX, iNumPartLine, uiSPAddrCur);
                uiSPAddrCur += m_uiAbsIdxInLCU;
                pcCUAboveMedian = getPUAbove( uiAbovePartIdx_median, uiSPAddrCur );
                if (pcCUAboveMedian &&!pcCUAboveMedian->isIntra( uiAbovePartIdx_median ))
                {
                  getNeighboringMvField(pcCUAboveMedian,uiAbovePartIdx_median,cMvField+(uiMVCount<<1),ucInterDir+uiMVCount);
                  uiMVCount++;
                  break;
                }
              }
          }
          else
          {
            ucInterDir[uiMVCount]= puhInterDirSP[1][iPartitionIdx-iNumPartLine];
            cMvField[(uiMVCount<<1)  ]=pcMvFieldSP[1][((iPartitionIdx-iNumPartLine)<<1)  ];
            cMvField[(uiMVCount<<1)+1]=pcMvFieldSP[1][((iPartitionIdx-iNumPartLine)<<1)+1];
            uiMVCount++;
          }

          {
            Bool bExistMV = false;
            ucInterDir[uiMVCount] = 0;
            if ( getSlice()->getEnableTMVPFlag())
            {
              Int iRefIdx = 0;
              TComMv cColMvTemp;
              UInt uiAbsPartIdxSP = g_auiZscanToRaster[uiSPAddr];
              UInt uiAbsPartSPAddrRB = uiSPAddr;
              UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
              Int uiLCUIdx = getAddr();
              
              UInt uiPartAddrCenter = uiSPAddr;
              uiPartAddrCenter = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartAddrCenter ]
                                        + ( iPUHeight/m_pcPic->getMinCUHeight()  )/2*m_pcPic->getNumPartInWidth()
                                        + ( iPUWidth/m_pcPic->getMinCUWidth()  )/2];

              if      ( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdxSP] + m_pcPic->getMinCUWidth() ) >= m_pcSlice->getSPS()->getPicWidthInLumaSamples() )  // image boundary check
              {
              }
              else if ( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdxSP] + m_pcPic->getMinCUHeight() ) >= m_pcSlice->getSPS()->getPicHeightInLumaSamples() )
              {
              }
              else
              {
                if ( ( uiAbsPartIdxSP % uiNumPartInCUWidth < uiNumPartInCUWidth - 1 ) &&           // is not at the last column of LCU 
                     ( uiAbsPartIdxSP / uiNumPartInCUWidth < m_pcPic->getNumPartInHeight() - 1 ) ) // is not at the last row    of LCU
                {
                  uiAbsPartSPAddrRB = g_auiRasterToZscan[ uiAbsPartIdxSP + uiNumPartInCUWidth + 1 ];
                  uiLCUIdx = getAddr();
                }
                else if ( uiAbsPartIdxSP % uiNumPartInCUWidth < uiNumPartInCUWidth - 1 )           // is not at the last column of LCU But is last row of LCU
                {
#if GEN_MRG_IMPROVEMENT
                  uiLCUIdx = getAddr() + m_pcPic->getFrameWidthInCU();
#endif
                  uiAbsPartSPAddrRB = g_auiRasterToZscan[ (uiAbsPartIdxSP + uiNumPartInCUWidth + 1) % m_pcPic->getNumPartInCU() ];
                }
                else if ( uiAbsPartIdxSP / uiNumPartInCUWidth < m_pcPic->getNumPartInHeight() - 1 ) // is not at the last row of LCU But is last column of LCU
                {
                  uiAbsPartSPAddrRB = g_auiRasterToZscan[ uiAbsPartIdxSP + 1 ];
                  uiLCUIdx = getAddr() + 1;
                }
                else //is the right bottom corner of LCU                       
                {
                  uiAbsPartSPAddrRB = 0;
#if GEN_MRG_IMPROVEMENT
                  uiLCUIdx = getAddr() + m_pcPic->getFrameWidthInCU() + 1;
#endif
                }
              }
              bExistMV = uiLCUIdx >= 0 && xGetColMVP( REF_PIC_LIST_0, uiLCUIdx, uiAbsPartSPAddrRB, cColMvTemp, iRefIdx );
              if( bExistMV == false )
              {
                bExistMV = xGetColMVP( REF_PIC_LIST_0, getAddr(), uiPartAddrCenter, cColMvTemp, iRefIdx );
              }
              if( bExistMV )
              {
                ucInterDir[uiMVCount] |= 1;
                cMvField[ 2 * uiMVCount ].setMvField( cColMvTemp, iRefIdx );
              }
    
              if ( getSlice()->isInterB() )
              {
                bExistMV = uiLCUIdx >= 0 && xGetColMVP( REF_PIC_LIST_1, uiLCUIdx, uiAbsPartSPAddrRB, cColMvTemp, iRefIdx);
                if( bExistMV == false )
                {
                  bExistMV = xGetColMVP( REF_PIC_LIST_1, getAddr(), uiPartAddrCenter, cColMvTemp, iRefIdx );
                }
                if( bExistMV )
                {
                  ucInterDir[uiMVCount] |= 2;
                  cMvField[ 2 * uiMVCount + 1 ].setMvField( cColMvTemp, iRefIdx );
                }
              }
            }
            if(ucInterDir[uiMVCount]>0)
            {  
              uiMVCount++;
            }
          }
          generateMvField(   cMvField, ucInterDir, uiMVCount,cMvFieldMedian,ucInterDirMedian);
          puhInterDirSP[1][iPartitionIdx] = ucInterDirMedian;
          pcMvFieldSP[1][(iPartitionIdx<<1)  ] = cMvFieldMedian[0];
          pcMvFieldSP[1][(iPartitionIdx<<1)+1] = cMvFieldMedian[1];
          if (iPartitionIdx == 0  || 
            (uiSameCount == iPartitionIdx &&
            puhInterDirSP[1][iPartitionIdx]      == puhInterDirSP[1][0] && 
            pcMvFieldSP[1][(iPartitionIdx<<1)  ] == pcMvFieldSP[1][0] &&
            pcMvFieldSP[1][(iPartitionIdx<<1)+1] == pcMvFieldSP[1][1] ))
          {
            uiSameCount++;
          }
          if ( uiSameCountATMVP == iPartitionIdx && 
            puhInterDirSP[1][iPartitionIdx]      == puhInterDirSP[0][iPartitionIdx] &&
            pcMvFieldSP[1][(iPartitionIdx<<1)  ] == pcMvFieldSP[0][((iPartitionIdx)<<1)  ] &&
            pcMvFieldSP[1][(iPartitionIdx<<1)+1] == pcMvFieldSP[0][((iPartitionIdx)<<1)+1] )              
          {
             uiSameCountATMVP++;
          }
          iPartitionIdx++;
        }
      }
      Bool bAtmvpExtAva =true;
      if (uiSameCount == iNumPart)
      {
        for (UInt uiIdx=0; uiIdx <iCount;uiIdx++)
        {
          if (peMergeTypeNeighbors[uiIdx]  !=  MGR_TYPE_SUBPU_TMVP) 
          {
            if (puhInterDirNeighbours[uiIdx] == puhInterDirSP[1][0] &&
                pcMvFieldNeighbours[uiIdx<<1]      == pcMvFieldSP[1][0] &&
                pcMvFieldNeighbours[(uiIdx<<1)+1]  == pcMvFieldSP[1][1])
            {
              bAtmvpExtAva = false;
              break;
            }
          }
        }
      }
      if(bAtmvpExtAva && bAtmvpAva)
      {
        if(uiSameCountATMVP == iNumPart)
        {
           bAtmvpExtAva = false;
        }
      }
      return bAtmvpExtAva;
}

#endif


#if QC_SUB_PU_TMVP_EXT
Void TComDataCU::getInterMergeCandidates( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMvFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand
#if QC_IC
  , Bool*           pbICFlag
#endif
#if QC_SUB_PU_TMVP
  , UChar*          peMergeTypeNeighbors
  , TComMvField*    pcMvFieldSP[2]
  , UChar*          puhInterDirSP[2]
  , UInt            uiDecCurrAbsPartIdx
  , TComDataCU*     pDecCurrCU
#endif
  , Int mrgCandIdx)
#else
Void TComDataCU::getInterMergeCandidates( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMvFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand
#if QC_IC
  , Bool*           pbICFlag
#endif
#if QC_SUB_PU_TMVP
  , UChar*          peMergeTypeNeighbors
  , TComMvField*    pcMvFieldSP
  , UChar*          puhInterDirSP
  , UInt            uiDecCurrAbsPartIdx
  , TComDataCU*     pDecCurrCU
#endif
  , Int mrgCandIdx)
#endif
{
  UInt uiAbsPartAddr = m_uiAbsIdxInLCU + uiAbsPartIdx;
  Bool abCandIsInter[ MRG_MAX_NUM_CANDS ];
  for( UInt ui = 0; ui < getSlice()->getMaxNumMergeCand(); ++ui )
  {
    abCandIsInter[ui] = false;
#if GEN_MRG_IMPROVEMENT
    pcMvFieldNeighbours[ ( ui << 1 )     ].setMvField(TComMv(0,0), NOT_VALID);
    pcMvFieldNeighbours[ ( ui << 1 ) + 1 ].setMvField(TComMv(0,0), NOT_VALID);
#else
    pcMvFieldNeighbours[ ( ui << 1 )     ].setRefIdx(NOT_VALID);
    pcMvFieldNeighbours[ ( ui << 1 ) + 1 ].setRefIdx(NOT_VALID);
#endif
  }
#if QC_IC
  memset( pbICFlag, false, sizeof( Bool )*MRG_MAX_NUM_CANDS );
#if QC_SUB_PU_TMVP
  Bool bICFlag = false;
#endif
#endif
  numValidMergeCand = getSlice()->getMaxNumMergeCand();
  // compute the location of the current PU
  Int xP, yP, nPSW, nPSH;
  this->getPartPosition(uiPUIdx, xP, yP, nPSW, nPSH);

  Int iCount = 0;

  UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB;
  PartSize cCurPS = getPartitionSize( uiAbsPartIdx );
  deriveLeftRightTopIdxGeneral( uiAbsPartIdx, uiPUIdx, uiPartIdxLT, uiPartIdxRT );
  deriveLeftBottomIdxGeneral  ( uiAbsPartIdx, uiPUIdx, uiPartIdxLB );
#if QC_SUB_PU_TMVP
  Bool bEnableATMVP = getSlice()->getSPS()->getAtmvpEnableFlag();
#if QC_LARGE_CTU
#if QC_SUB_PU_TMVP_EXT
  TComMv cZeroMv;
  cZeroMv.setZero();
  for (Int i3=0; i3< 2; i3++)
  {
    for (Int i=0 , i2 = 0 ; i< getTotalNumPart(); i++ , i2 += 2)
    {
      puhInterDirSP[i3][i] = 0;
      pcMvFieldSP[i3][i2  ].setMvField(cZeroMv,-1);
      pcMvFieldSP[i3][i2+1].setMvField(cZeroMv,-1);
    }
  }
#else
  for (Int i=0 , i2 = 0 ; i< getTotalNumPart(); i++ , i2 += 2)
  {
    puhInterDirSP[i] = 0;
    pcMvFieldSP[i2].setRefIdx(-1);
    pcMvFieldSP[i2+1].setRefIdx(-1);
  }
#endif
#else
#if QC_SUB_PU_TMVP_EXT
  for (Int i=0; i< getPic()->getPicSym()->getNumPartition(); i++)
  {
    puhInterDirSP[0][i] = 0;
    pcMvFieldSP[0][2*i].getMv().set(0, 0);
    pcMvFieldSP[0][2*i+1].getMv().set(0, 0);
    pcMvFieldSP[0][2*i].setRefIdx(-1);
    pcMvFieldSP[0][2*i+1].setRefIdx(-1);
    puhInterDirSP[1][i] = 0;
    pcMvFieldSP[1][2*i].getMv().set(0, 0);
    pcMvFieldSP[1][2*i+1].getMv().set(0, 0);
    pcMvFieldSP[1][2*i].setRefIdx(-1);
    pcMvFieldSP[1][2*i+1].setRefIdx(-1);
  }
#else
  for (Int i=0; i< getPic()->getPicSym()->getNumPartition(); i++)
  {
    puhInterDirSP[i] = 0;
    pcMvFieldSP[2*i].getMv().set(0, 0);
    pcMvFieldSP[2*i+1].getMv().set(0, 0);
    pcMvFieldSP[2*i].setRefIdx(-1);
    pcMvFieldSP[2*i+1].setRefIdx(-1);
  }
#endif
#endif
  memset(peMergeTypeNeighbors, MGR_TYPE_DEFAULT_N, sizeof(UChar)*MRG_MAX_NUM_CANDS);
  Bool bAtmvpAva = false;
  Int uiAtmvpPos = -1;
  TComMv cNBTV  ; 
  Int  iPOCAtmvp =  0;
  Bool iAvaNBTV  =  false;
#endif 


  //left
  UInt uiLeftPartIdx = 0;
  TComDataCU* pcCULeft = 0;
  pcCULeft = getPULeft( uiLeftPartIdx, uiPartIdxLB );
  Bool isAvailableA1 = pcCULeft &&
    pcCULeft->isDiffMER(xP -1, yP+nPSH-1, xP, yP) &&
    !( uiPUIdx == 1 && (cCurPS == SIZE_Nx2N || cCurPS == SIZE_nLx2N || cCurPS == SIZE_nRx2N) ) &&
    !pcCULeft->isIntra( uiLeftPartIdx ) ;
  if ( isAvailableA1 )
  {
    abCandIsInter[iCount] = true;
#if QC_IC
    pbICFlag[iCount] = pcCULeft->getICFlag( uiLeftPartIdx );
#endif
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCULeft->getInterDir( uiLeftPartIdx );
    // get Mv from Left
    pcCULeft->getMvField( pcCULeft, uiLeftPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCULeft->getMvField( pcCULeft, uiLeftPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }
  
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand()) 
  {
    return;
  }


  // above
  UInt uiAbovePartIdx = 0;
  TComDataCU* pcCUAbove = 0;
  pcCUAbove = getPUAbove( uiAbovePartIdx, uiPartIdxRT );
  Bool isAvailableB1 = pcCUAbove &&
  pcCUAbove->isDiffMER(xP+nPSW-1, yP-1, xP, yP) &&
  !( uiPUIdx == 1 && (cCurPS == SIZE_2NxN || cCurPS == SIZE_2NxnU || cCurPS == SIZE_2NxnD) ) &&
  !pcCUAbove->isIntra( uiAbovePartIdx );
#if QC_IC
  if ( isAvailableB1 && (!isAvailableA1 || !( pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCUAbove, uiAbovePartIdx ) && pcCULeft->getICFlag( uiLeftPartIdx ) == pcCUAbove->getICFlag( uiAbovePartIdx ) ) ) )
#else
  if ( isAvailableB1 && (!isAvailableA1 || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCUAbove, uiAbovePartIdx ) ) )
#endif
  {
    abCandIsInter[iCount] = true;
#if QC_IC
    pbICFlag[iCount] = pcCUAbove->getICFlag( uiAbovePartIdx );
#endif
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCUAbove->getInterDir( uiAbovePartIdx );
    // get Mv from Left
    pcCUAbove->getMvField( pcCUAbove, uiAbovePartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCUAbove->getMvField( pcCUAbove, uiAbovePartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand()) 
  {
    return;
  }


  // above right
  UInt uiAboveRightPartIdx = 0;
  TComDataCU* pcCUAboveRight = 0;
  pcCUAboveRight = getPUAboveRight( uiAboveRightPartIdx, uiPartIdxRT );
  Bool isAvailableB0 = pcCUAboveRight &&
  pcCUAboveRight->isDiffMER(xP+nPSW, yP-1, xP, yP) &&
  !pcCUAboveRight->isIntra( uiAboveRightPartIdx );
#if GEN_MRG_IMPROVEMENT
#if QC_IC
  if ( isAvailableB0 
    && ( !isAvailableB1 || !( pcCUAbove->hasEqualMotion( uiAbovePartIdx, pcCUAboveRight, uiAboveRightPartIdx ) && pcCUAbove->getICFlag( uiAbovePartIdx ) == pcCUAboveRight->getICFlag( uiAboveRightPartIdx ) ) ) 
    && ( !isAvailableA1 || !( pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCUAboveRight, uiAboveRightPartIdx ) && pcCULeft->getICFlag( uiLeftPartIdx ) == pcCUAboveRight->getICFlag( uiAboveRightPartIdx ) ) ) )
#else
  if ( isAvailableB0 && ( !isAvailableB1 || !pcCUAbove->hasEqualMotion( uiAbovePartIdx, pcCUAboveRight, uiAboveRightPartIdx ) ) && (!isAvailableA1 || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCUAboveRight, uiAboveRightPartIdx ) ))
#endif
#else
  if ( isAvailableB0 && ( !isAvailableB1 || !pcCUAbove->hasEqualMotion( uiAbovePartIdx, pcCUAboveRight, uiAboveRightPartIdx ) ) )
#endif
  {
    abCandIsInter[iCount] = true;
#if QC_IC
    pbICFlag[iCount] = pcCUAboveRight->getICFlag( uiAboveRightPartIdx );
#endif
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCUAboveRight->getInterDir( uiAboveRightPartIdx );
    // get Mv from Left
    pcCUAboveRight->getMvField( pcCUAboveRight, uiAboveRightPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCUAboveRight->getMvField( pcCUAboveRight, uiAboveRightPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand()) 
  {
    return;
  }


  //left bottom
  UInt uiLeftBottomPartIdx = 0;
  TComDataCU* pcCULeftBottom = 0;
  pcCULeftBottom = this->getPUBelowLeft( uiLeftBottomPartIdx, uiPartIdxLB );
  Bool isAvailableA0 = pcCULeftBottom &&
  pcCULeftBottom->isDiffMER(xP-1, yP+nPSH, xP, yP) &&
  !pcCULeftBottom->isIntra( uiLeftBottomPartIdx ) ;
#if GEN_MRG_IMPROVEMENT
#if QC_IC
  if ( isAvailableA0 
    && ( !isAvailableA1 || !( pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCULeftBottom, uiLeftBottomPartIdx ) && pcCULeft->getICFlag( uiLeftPartIdx ) == pcCULeftBottom->getICFlag( uiLeftBottomPartIdx ) ) ) 
    && ( !isAvailableB1 || !( pcCUAbove->hasEqualMotion( uiAbovePartIdx, pcCULeftBottom, uiLeftBottomPartIdx  ) && pcCUAbove->getICFlag( uiAbovePartIdx ) == pcCULeftBottom->getICFlag( uiLeftBottomPartIdx ) ) ) 
    && ( !isAvailableB0 || !( pcCUAboveRight->hasEqualMotion( uiAboveRightPartIdx, pcCULeftBottom, uiLeftBottomPartIdx ) && pcCUAboveRight->getICFlag( uiAboveRightPartIdx ) == pcCULeftBottom->getICFlag( uiLeftBottomPartIdx ) ) ))
#else
  if ( isAvailableA0 && ( !isAvailableA1 || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCULeftBottom, uiLeftBottomPartIdx ) ) && ( !isAvailableB1 || !pcCUAbove->hasEqualMotion( uiAbovePartIdx, pcCULeftBottom, uiLeftBottomPartIdx  ) ) && (!isAvailableB0 || !pcCUAboveRight->hasEqualMotion( uiAboveRightPartIdx, pcCULeftBottom, uiLeftBottomPartIdx ) ))
#endif
#else
  if ( isAvailableA0 && ( !isAvailableA1 || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCULeftBottom, uiLeftBottomPartIdx ) ) )
#endif
  {
    abCandIsInter[iCount] = true;
#if QC_IC
    pbICFlag[iCount] = pcCULeftBottom->getICFlag( uiLeftBottomPartIdx );
#endif
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCULeftBottom->getInterDir( uiLeftBottomPartIdx );
    // get Mv from Left
    pcCULeftBottom->getMvField( pcCULeftBottom, uiLeftBottomPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCULeftBottom->getMvField( pcCULeftBottom, uiLeftBottomPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand()) 
  {
    return;
  }
#if QC_SUB_PU_TMVP
  if(bEnableATMVP && getSlice()->getEnableTMVPFlag())
  {
    Bool bMrgIdxMatchATMVPCan = (mrgCandIdx == iCount);
    ///////////////////////////////////////////////////////////////////////
    ////////               GET Temporal Vector                     ////////
    ///////////////////////////////////////////////////////////////////////
    get1stTvFromSpatialNeighbor(uiAbsPartIdx,uiPUIdx,iAvaNBTV, iPOCAtmvp, cNBTV);
  
    ///////////////////////////////////////////////////////////////////////
    //////GET Motion for the ATMVP candidate based on Temporal Vector//////
    ///////////////////////////////////////////////////////////////////////
#if QC_SUB_PU_TMVP_EXT
    bAtmvpAva = getInterMergeSubPUTmvpCandidate( uiPUIdx, pcMvFieldSP[0], puhInterDirSP[0], &pcMvFieldNeighbours[iCount<<1], &puhInterDirNeighbours[iCount], cNBTV, bMrgIdxMatchATMVPCan, 
#if QC_IC
      bICFlag,
#endif
      iPOCAtmvp, pDecCurrCU, uiDecCurrAbsPartIdx);
#else
    bAtmvpAva = getInterMergeSubPUTmvpCandidate( uiPUIdx, pcMvFieldSP, puhInterDirSP, &pcMvFieldNeighbours[iCount<<1], &puhInterDirNeighbours[iCount], cNBTV, bMrgIdxMatchATMVPCan, 
#if QC_IC
      bICFlag,
#endif
      iPOCAtmvp, pDecCurrCU, uiDecCurrAbsPartIdx);
#endif
    if ( bAtmvpAva ) 
    {
      abCandIsInter[iCount] = true;
#if QC_IC
      pbICFlag[iCount] = bICFlag;
#endif
      peMergeTypeNeighbors[iCount] = MGR_TYPE_SUBPU_TMVP;
      if ( bMrgIdxMatchATMVPCan )
      {
        return;
      }
      uiAtmvpPos = iCount;
      iCount ++;
      if (iCount == getSlice()->getMaxNumMergeCand()) 
      {
        return;
      }      
    }
  }
#endif

#if QC_SUB_PU_TMVP_EXT
  if(bEnableATMVP && getSlice()->getEnableTMVPFlag())
  {
    Bool bAtmvpExtAva = getInterMergeSubPURecursiveCandidate(  uiAbsPartIdx,  uiPUIdx, pcMvFieldNeighbours, puhInterDirNeighbours, numValidMergeCand, peMergeTypeNeighbors  , pcMvFieldSP , puhInterDirSP,  iCount );
    if (bAtmvpExtAva)
    {
        //store the sub-PU motion information
        Int iCurrPosX, iCurrPosY;
        Int iWidth, iHeight;
        Int iPUWidth, iPUHeight, iNumPart, iNumPartLine,iPartitionIdx;
        getPartPosition( uiPUIdx, iCurrPosX, iCurrPosY,      iWidth, iHeight);
        getSPPara      ( iWidth,  iHeight,  iNumPart, iNumPartLine, iPUWidth, iPUHeight);
        peMergeTypeNeighbors[iCount]  =  MGR_TYPE_SUBPU_TMVP_EXT;
        puhInterDirNeighbours[iCount] = puhInterDirSP[1][iNumPart-1];
        pcMvFieldNeighbours[(iCount<<1)]    =pcMvFieldSP[1][ (iNumPart-1)<<1   ];
        pcMvFieldNeighbours[(iCount<<1)+1]  =pcMvFieldSP[1][((iNumPart-1)<<1)+1];
        abCandIsInter[iCount] = true;
#if QC_IC
        pbICFlag[iCount] = bAtmvpAva ? !bICFlag : false;
#endif
        if ( mrgCandIdx == iCount )
        {
          for (iPartitionIdx = 0; iPartitionIdx < iNumPart; iPartitionIdx++)
          {
            UInt uiSPAddr;
            pDecCurrCU->getSPAbsPartIdx(uiDecCurrAbsPartIdx, iPUWidth, iPUHeight, iPartitionIdx, iNumPartLine, uiSPAddr);
            pDecCurrCU->setInterDirSP(puhInterDirSP[1][iPartitionIdx], uiSPAddr, iPUWidth, iPUHeight);
            pDecCurrCU->getCUMvField( REF_PIC_LIST_0 )->setMvFieldSP(pDecCurrCU, uiSPAddr, pcMvFieldSP[1][2*iPartitionIdx], iPUWidth, iPUHeight);
            pDecCurrCU->getCUMvField( REF_PIC_LIST_1 )->setMvFieldSP(pDecCurrCU, uiSPAddr, pcMvFieldSP[1][2*iPartitionIdx + 1], iPUWidth, iPUHeight);
          }
          return;
        }
        iCount ++;
        if (iCount == getSlice()->getMaxNumMergeCand()) 
        {
          return;
        }
      }
  }
#endif


  // above left 
#if QC_SUB_PU_TMVP 
#if QC_SUB_PU_TMVP_EXT
  if( iCount < (bEnableATMVP? 6: 5) ) 
#else
  if( iCount < (bEnableATMVP? 5: 4) ) 
#endif
#else
  if( iCount < 4 )
#endif
  {
    UInt uiAboveLeftPartIdx = 0;
    TComDataCU* pcCUAboveLeft = 0;
    pcCUAboveLeft = getPUAboveLeft( uiAboveLeftPartIdx, uiAbsPartAddr );
    Bool isAvailableB2 = pcCUAboveLeft &&
    pcCUAboveLeft->isDiffMER(xP-1, yP-1, xP, yP) &&
    !pcCUAboveLeft->isIntra( uiAboveLeftPartIdx );
#if GEN_MRG_IMPROVEMENT
#if QC_IC
    if ( isAvailableB2 
      && ( !isAvailableA1 || !( pcCULeft       ->hasEqualMotion(      uiLeftPartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) && pcCULeft->getICFlag( uiLeftPartIdx )   == pcCUAboveLeft->getICFlag( uiAboveLeftPartIdx ) ) )
      && ( !isAvailableB1 || !( pcCUAbove      ->hasEqualMotion(     uiAbovePartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) && pcCUAbove->getICFlag( uiAbovePartIdx ) == pcCUAboveLeft->getICFlag( uiAboveLeftPartIdx ) ) )
      && ( !isAvailableA0 || !( pcCULeftBottom ->hasEqualMotion(uiLeftBottomPartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) && pcCULeftBottom->getICFlag( uiLeftBottomPartIdx ) == pcCUAboveLeft->getICFlag( uiAboveLeftPartIdx ) ) )
      && ( !isAvailableB0 || !( pcCUAboveRight ->hasEqualMotion(uiAboveRightPartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) && pcCUAboveRight->getICFlag( uiAboveRightPartIdx ) == pcCUAboveLeft->getICFlag( uiAboveLeftPartIdx ) ) )
      )
#else
    if ( isAvailableB2 
        && ( !isAvailableA1 || !pcCULeft       ->hasEqualMotion(      uiLeftPartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) )
        && ( !isAvailableB1 || !pcCUAbove      ->hasEqualMotion(     uiAbovePartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) )
        && ( !isAvailableA0 || !pcCULeftBottom ->hasEqualMotion(uiLeftBottomPartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) )
        && ( !isAvailableB0 || !pcCUAboveRight ->hasEqualMotion(uiAboveRightPartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) )
       )
#endif
#else
    if ( isAvailableB2 && ( !isAvailableA1 || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) )
        && ( !isAvailableB1 || !pcCUAbove->hasEqualMotion( uiAbovePartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) ) )
#endif
    {
      abCandIsInter[iCount] = true;
#if QC_IC
      pbICFlag[iCount] = pcCUAboveLeft->getICFlag( uiAboveLeftPartIdx );
#endif
      // get Inter Dir
      puhInterDirNeighbours[iCount] = pcCUAboveLeft->getInterDir( uiAboveLeftPartIdx );
      // get Mv from Left
      pcCUAboveLeft->getMvField( pcCUAboveLeft, uiAboveLeftPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
      if ( getSlice()->isInterB() )
      {
        pcCUAboveLeft->getMvField( pcCUAboveLeft, uiAboveLeftPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
      }
      if ( mrgCandIdx == iCount )
      {
        return;
      }
      iCount ++;
    }
  }
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand()) 
  {
    return;
  }

  if ( getSlice()->getEnableTMVPFlag())
  {
    //>> MTK colocated-RightBottom
    UInt uiPartIdxRB;

    deriveRightBottomIdx( uiPUIdx, uiPartIdxRB );  

    UInt uiAbsPartIdxTmp = g_auiZscanToRaster[uiPartIdxRB];
    UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();

    TComMv cColMv;
    Int iRefIdx;
    Int uiLCUIdx = -1;

    if      ( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdxTmp] + m_pcPic->getMinCUWidth() ) >= m_pcSlice->getSPS()->getPicWidthInLumaSamples() )  // image boundary check
    {
    }
    else if ( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdxTmp] + m_pcPic->getMinCUHeight() ) >= m_pcSlice->getSPS()->getPicHeightInLumaSamples() )
    {
    }
    else
    {
      if ( ( uiAbsPartIdxTmp % uiNumPartInCUWidth < uiNumPartInCUWidth - 1 ) &&           // is not at the last column of LCU 
        ( uiAbsPartIdxTmp / uiNumPartInCUWidth < m_pcPic->getNumPartInHeight() - 1 ) ) // is not at the last row    of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdxTmp + uiNumPartInCUWidth + 1 ];
        uiLCUIdx = getAddr();
      }
      else if ( uiAbsPartIdxTmp % uiNumPartInCUWidth < uiNumPartInCUWidth - 1 )           // is not at the last column of LCU But is last row of LCU
      {
#if GEN_MRG_IMPROVEMENT
        uiLCUIdx = getAddr() + m_pcPic->getFrameWidthInCU();
#endif
        uiAbsPartAddr = g_auiRasterToZscan[ (uiAbsPartIdxTmp + uiNumPartInCUWidth + 1) % m_pcPic->getNumPartInCU() ];
      }
      else if ( uiAbsPartIdxTmp / uiNumPartInCUWidth < m_pcPic->getNumPartInHeight() - 1 ) // is not at the last row of LCU But is last column of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdxTmp + 1 ];
        uiLCUIdx = getAddr() + 1;
      }
      else //is the right bottom corner of LCU                       
      {
        uiAbsPartAddr = 0;
#if GEN_MRG_IMPROVEMENT
        uiLCUIdx = getAddr() + m_pcPic->getFrameWidthInCU() + 1;
#endif
      }
    }
    
    
    iRefIdx = 0;
    Bool bExistMV = false;
    UInt uiPartIdxCenter;
    UInt uiCurLCUIdx = getAddr();
    Int dir = 0;
    UInt uiArrayAddr = iCount;
    xDeriveCenterIdx( uiPUIdx, uiPartIdxCenter );
#if QC_IC
    Bool abTMVPICFlag[2] = { false, false }, bTMVPICFlag = false;
    bExistMV = uiLCUIdx >= 0 && xGetColMVP( REF_PIC_LIST_0, uiLCUIdx, uiAbsPartAddr, cColMv, iRefIdx, &abTMVPICFlag[0] );
#else
    bExistMV = uiLCUIdx >= 0 && xGetColMVP( REF_PIC_LIST_0, uiLCUIdx, uiAbsPartAddr, cColMv, iRefIdx );
#endif
    if( bExistMV == false )
    {
#if QC_IC
      bExistMV = xGetColMVP( REF_PIC_LIST_0, uiCurLCUIdx, uiPartIdxCenter, cColMv, iRefIdx, &abTMVPICFlag[0] );
#else
      bExistMV = xGetColMVP( REF_PIC_LIST_0, uiCurLCUIdx, uiPartIdxCenter, cColMv, iRefIdx );
#endif
    }
    if( bExistMV )
    {
      dir |= 1;
#if QC_IC
      bTMVPICFlag |= abTMVPICFlag[0];
#endif
      pcMvFieldNeighbours[ 2 * uiArrayAddr ].setMvField( cColMv, iRefIdx );
    }
    
    if ( getSlice()->isInterB() )
    {
#if QC_IC
      bExistMV = uiLCUIdx >= 0 && xGetColMVP( REF_PIC_LIST_1, uiLCUIdx, uiAbsPartAddr, cColMv, iRefIdx, &abTMVPICFlag[1] );
#else
      bExistMV = uiLCUIdx >= 0 && xGetColMVP( REF_PIC_LIST_1, uiLCUIdx, uiAbsPartAddr, cColMv, iRefIdx);
#endif
      if( bExistMV == false )
      {
#if QC_IC
        bExistMV = xGetColMVP( REF_PIC_LIST_1, uiCurLCUIdx, uiPartIdxCenter, cColMv, iRefIdx, &abTMVPICFlag[1] );
#else
        bExistMV = xGetColMVP( REF_PIC_LIST_1, uiCurLCUIdx, uiPartIdxCenter, cColMv, iRefIdx );
#endif
      }
      if( bExistMV )
      {
        dir |= 2;
#if QC_IC
        bTMVPICFlag |= abTMVPICFlag[1];
#endif
        pcMvFieldNeighbours[ 2 * uiArrayAddr + 1 ].setMvField( cColMv, iRefIdx );
      }
    }
    
    if (dir != 0)
    {
#if QC_SUB_PU_TMVP 
      Bool bAddT  = true;
      if (bAtmvpAva)
      {
        bAddT = false;
#if QC_IC
        if ( dir!= puhInterDirNeighbours[uiAtmvpPos ] || bTMVPICFlag != pbICFlag[uiAtmvpPos] )
#else
        if ( dir!= puhInterDirNeighbours[uiAtmvpPos ] )
#endif
        {
          bAddT = true ;
        }
        else
        {
          for ( UInt uiR = 0; uiR< 2; uiR++ )
          {
            if ( dir & ( 1 << uiR ) )
            {
              if ( pcMvFieldNeighbours[ 2 * uiArrayAddr+uiR].getRefIdx()!= pcMvFieldNeighbours[2*uiAtmvpPos+uiR].getRefIdx())
              {
                bAddT = true;
                break;
              }
              else 
              {
                if (pcMvFieldNeighbours[ 2 * uiArrayAddr+uiR].getMv()  != pcMvFieldNeighbours[2*uiAtmvpPos+uiR].getMv())
                {
                  bAddT = true;
                  break;
                }
              }
            }
          }
        }
      }
#if GEN_MRG_IMPROVEMENT
      Int iSpatCan = bAtmvpAva? (iCount-1): iCount;
      for(Int iCanIdx = 0; iCanIdx <iSpatCan; iCanIdx++)
      {
        if(puhInterDirNeighbours[iCanIdx]== dir && pcMvFieldNeighbours[iCanIdx<<1]==pcMvFieldNeighbours[uiArrayAddr<<1] && pcMvFieldNeighbours[1+ (iCanIdx<<1)]==pcMvFieldNeighbours[1+(uiArrayAddr<<1)]
#if QC_IC
        && pbICFlag[iCanIdx] == bTMVPICFlag
#endif
        )
        {
          bAddT = false;
        }
      }
#endif
      if ( bAddT )
      {
#endif
      puhInterDirNeighbours[uiArrayAddr] = dir;
      abCandIsInter[uiArrayAddr] = true;
#if QC_IC
      pbICFlag[uiArrayAddr] = bTMVPICFlag;
#endif
      if ( mrgCandIdx == iCount )
      {
        return;
      }
      iCount++;
#if QC_SUB_PU_TMVP
    }
#endif
    }
  }
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand()) 
  {
    return;
  }
  UInt uiArrayAddr = iCount;
  UInt uiCutoff = uiArrayAddr;
#if QC_SUB_PU_TMVP
  uiCutoff        = ( uiCutoff >4 ? 4: uiCutoff );
#endif
  if ( getSlice()->isInterB())
  {
    UInt uiPriorityList0[12] = {0 , 1, 0, 2, 1, 2, 0, 3, 1, 3, 2, 3};
    UInt uiPriorityList1[12] = {1 , 0, 2, 0, 2, 1, 3, 0, 3, 1, 3, 2};
    for (Int idx=0; idx<uiCutoff*(uiCutoff-1) && uiArrayAddr!= getSlice()->getMaxNumMergeCand(); idx++)
    {
      Int i = uiPriorityList0[idx]; Int j = uiPriorityList1[idx];
      if (abCandIsInter[i] && abCandIsInter[j]&& (puhInterDirNeighbours[i]&0x1)&&(puhInterDirNeighbours[j]&0x2))
      {
        abCandIsInter[uiArrayAddr] = true;
        puhInterDirNeighbours[uiArrayAddr] = 3;
#if QC_IC
        pbICFlag[uiArrayAddr] = pbICFlag[i] || pbICFlag[j];
#endif
        // get Mv from cand[i] and cand[j]
        pcMvFieldNeighbours[uiArrayAddr << 1].setMvField(pcMvFieldNeighbours[i<<1].getMv(), pcMvFieldNeighbours[i<<1].getRefIdx());
        pcMvFieldNeighbours[( uiArrayAddr << 1 ) + 1].setMvField(pcMvFieldNeighbours[(j<<1)+1].getMv(), pcMvFieldNeighbours[(j<<1)+1].getRefIdx());

        Int iRefPOCL0 = m_pcSlice->getRefPOC( REF_PIC_LIST_0, pcMvFieldNeighbours[(uiArrayAddr<<1)].getRefIdx() );
        Int iRefPOCL1 = m_pcSlice->getRefPOC( REF_PIC_LIST_1, pcMvFieldNeighbours[(uiArrayAddr<<1)+1].getRefIdx() );
        if (iRefPOCL0 == iRefPOCL1 && pcMvFieldNeighbours[(uiArrayAddr<<1)].getMv() == pcMvFieldNeighbours[(uiArrayAddr<<1)+1].getMv())
        {
          abCandIsInter[uiArrayAddr] = false;
        }
        else
        {
#if QC_SUB_PU_TMVP
          if(uiArrayAddr == mrgCandIdx)
          {
            return;
          }
#endif
          uiArrayAddr++;
        }
      }
    }
  }
  // early termination
  if (uiArrayAddr == getSlice()->getMaxNumMergeCand()) 
  {
    return;
  }
  Int iNumRefIdx = (getSlice()->isInterB()) ? min(m_pcSlice->getNumRefIdx(REF_PIC_LIST_0), m_pcSlice->getNumRefIdx(REF_PIC_LIST_1)) : m_pcSlice->getNumRefIdx(REF_PIC_LIST_0);
  Int r = 0;
  Int refcnt = 0;
  while (uiArrayAddr < getSlice()->getMaxNumMergeCand())
  {
    abCandIsInter[uiArrayAddr] = true;
    puhInterDirNeighbours[uiArrayAddr] = 1;
    pcMvFieldNeighbours[uiArrayAddr << 1].setMvField( TComMv(0, 0), r);
#if QC_IC
    pbICFlag[uiArrayAddr] = false;
#endif
    if ( getSlice()->isInterB() )
    {
      puhInterDirNeighbours[uiArrayAddr] = 3;
      pcMvFieldNeighbours[(uiArrayAddr << 1) + 1].setMvField(TComMv(0, 0), r);
    }
    uiArrayAddr++;
    if( uiArrayAddr >= getSlice()->getMaxNumMergeCand() )
    {
      break;
    }

    if ( refcnt == iNumRefIdx - 1 )
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }

  numValidMergeCand = uiArrayAddr;
}

#if QC_SUB_PU_TMVP
/** Constructs the ATMVP candidate with a given temporal vector
 * \param uiPUIdx
 * \param pcMvFieldSP 
 * \param puhInterDirSP
 * \param pcMvFieldDefault
 * \param pcInterDirDefault
 * \param numValidMergeCand
 * \param cTMv
 * \param bMrgIdxMatchATMVPCan
 * \param iPocColPic
 * \param pDecCurrCU
 * \param uiDecCurrAbsPartIdx
  */
Bool TComDataCU::getInterMergeSubPUTmvpCandidate ( UInt uiPUIdx,  TComMvField* pcMvFieldSP, UChar* puhInterDirSP,
                                                   TComMvField* pcMvFieldDefault, UChar* pcInterDirDefault, TComMv cTMv, Bool bMrgIdxMatchATMVPCan, 
#if QC_IC
                                                   Bool& rbICFlag,
#endif
                                                   Int iPocColPic, TComDataCU* pDecCurrCU, UInt uiDecCurrAbsPartIdx )
{
  TComDataCU* pcTempCU;
  Int iTempCUAddr, iTempAbsPartIdx, iTempPosX, iTempPosY;
  Int iPartition     = 0;
  Int iInterDirSaved = 0;
#if QC_IC
  Bool bTmpICFlag;
#endif
  ///////////////////////////////////////////////////////////////////////
  ////////          GET Initial Temporal Vector                  ////////
  ///////////////////////////////////////////////////////////////////////
  
  TComMv cTempVector = cTMv;

  // compute the location of the current PU
  Int iCurrPosX, iCurrPosY, iWidth, iHeight;
  Int iPUWidth, iPUHeight, iNumPart, iNumPartLine;

  getPartPosition( uiPUIdx, iCurrPosX, iCurrPosY,      iWidth, iHeight);
  getSPPara      (  iWidth,  iHeight,  iNumPart, iNumPartLine, iPUWidth, iPUHeight);

  Int iOffsetX = iPUWidth/2;;
  Int iOffsetY = iPUHeight/2;
  
  TComMv cColMv;
  // use coldir.
  Bool bBSlice = getSlice()->isInterB();
  UInt bColL0  = getSlice()->getColFromL0Flag();
  TComPic *pColPic = getPicfromPOC(iPocColPic);


  Bool bATMVPAvailFlag = false;
  TComMvField cDefaultMvField[2];
  cDefaultMvField[0].getMv().set(0, 0);
  cDefaultMvField[1].getMv().set(0, 0);


  Int         iTempCenterCUAddr, iTempCenterAbsPartIdx;
  Int         iCenterPosX, iCenterPosY;
#if QC_MV_STORE_PRECISION_BIT
  Int nOffset = 1 << ( QC_MV_STORE_PRECISION_BIT - 1 );
#endif

  Bool bInit = false;
  for( UInt uiLX = 0; uiLX < (bBSlice ? 2:1) && !bATMVPAvailFlag; uiLX++)
  {
    RefPicList eListY = RefPicList( bBSlice ? (bColL0 ? uiLX: 1- uiLX) : uiLX );
    for (Int refIdxY = (bInit ? 0 : -1); refIdxY < getSlice()->getNumRefIdx(eListY) && !bATMVPAvailFlag; refIdxY++)
    {
      if (!bInit) bInit = true;
      else
        pColPic = getSlice()->getRefPic(eListY, refIdxY);
      Int iNewColPicPOC = pColPic->getPOC();
      if ( iNewColPicPOC!= iPocColPic)
      {
      //////////////// POC based scaling of the temporal vector /////////////
          Int iScale = xGetDistScaleFactor(getSlice()->getPOC(), iNewColPicPOC, getSlice()->getPOC(), iPocColPic);
        if ( iScale != 4096 )
          cTempVector=cTMv.scaleMv( iScale );
      }
      else
        cTempVector=cTMv;


#if QC_SUB_PU_TMVP_V08 && !QC_SUB_PU_TMVP_EXT
#if QC_MV_STORE_PRECISION_BIT 
      iCurrPosX   +=  ((cTempVector.getHor()+nOffset)>>QC_MV_STORE_PRECISION_BIT); // consider rounding later: considered now
      iCurrPosY   +=  ((cTempVector.getVer()+nOffset)>>QC_MV_STORE_PRECISION_BIT); // consider rounding later: considered now
#else
      iCurrPosX   +=  ((cTempVector.getHor()+2)>>2); // consider rounding later: considered now
      iCurrPosY   +=  ((cTempVector.getVer()+2)>>2); // consider rounding later: considered now
#endif

      iCenterPosX = iCurrPosX + ( ( iWidth /  iPUWidth ) >> 1 )  * iPUWidth + ( iPUWidth >> 1 ) ; 
      iCenterPosY = iCurrPosY + ( ( iHeight /  iPUHeight ) >> 1 )  * iPUHeight + (iPUHeight >> 1) ;

      if(iWidth == iPUWidth && iHeight == iPUHeight)
      {
        iCenterPosX = iCurrPosX + (iWidth >> 1);
        iCenterPosY = iCurrPosY + (iHeight >> 1);
      }
#else
#if QC_MV_STORE_PRECISION_BIT 
      iCenterPosX = iCurrPosX + ( ( iWidth /  iPUWidth ) >> 1 )  * iPUWidth + ( iPUWidth >> 1 ) +   ((cTempVector.getHor()+nOffset)>>QC_MV_STORE_PRECISION_BIT); 
      iCenterPosY = iCurrPosY + ( ( iHeight /  iPUHeight ) >> 1 )  * iPUHeight + (iPUHeight >> 1) + ((cTempVector.getVer()+nOffset)>>QC_MV_STORE_PRECISION_BIT) ;
#else
      iCenterPosX = iCurrPosX + ( ( iWidth /  iPUWidth ) >> 1 )  * iPUWidth + ( iPUWidth >> 1 ) +   ((cTempVector.getHor()+2)>>2); 
      iCenterPosY = iCurrPosY + ( ( iHeight /  iPUHeight ) >> 1 )  * iPUHeight + (iPUHeight >> 1) + ((cTempVector.getVer()+2)>>2) ;
#endif
      
      if(iWidth == iPUWidth && iHeight == iPUHeight)
      {
#if QC_MV_STORE_PRECISION_BIT 
        iCenterPosX = iCurrPosX + (iWidth >> 1)  + ((cTempVector.getHor()+nOffset)>>QC_MV_STORE_PRECISION_BIT);
        iCenterPosY = iCurrPosY + (iHeight >> 1) + ((cTempVector.getVer()+nOffset)>>QC_MV_STORE_PRECISION_BIT);
#else
        iCenterPosX = iCurrPosX + (iWidth >> 1)  + ((cTempVector.getHor()+2)>>2);
        iCenterPosY = iCurrPosY + (iHeight >> 1) + ((cTempVector.getVer()+2)>>2);
#endif
      }
#endif

      iCenterPosX = Clip3( 0, pColPic->getPicYuvRec()->getWidth() - 1,  iCenterPosX  );
      iCenterPosY = Clip3( 0, pColPic->getPicYuvRec()->getHeight()- 1,  iCenterPosY  );

      // derivation of center motion parameters from the collocated CU
      pColPic->getPicYuvRec()->getCUAddrAndPartIdx( iCenterPosX , iCenterPosY , iTempCenterCUAddr, iTempCenterAbsPartIdx );
      TComDataCU* pcDefaultCU    = pColPic->getCU( iTempCenterCUAddr );
      if( pcDefaultCU->getPredictionMode( iTempCenterAbsPartIdx ) != MODE_INTRA )
      {
        for( UInt uiCurrRefListId = 0; uiCurrRefListId < (bBSlice ? 2:1) ; uiCurrRefListId++ )
        {
          RefPicList  eCurrRefPicList = RefPicList( uiCurrRefListId );
#if QC_SUB_PU_TMVP_EXT && QC_SUB_PU_TMVP_V08==0
          if (deriveScaledMotionTemporalForOneDirection(pcDefaultCU,eCurrRefPicList,cColMv, iTempCenterAbsPartIdx,0 ,pColPic
#if QC_IC
            , bTmpICFlag
#endif
            ))
#else
          if (deriveScaledMotionTemporalForOneDirection(pcDefaultCU,eCurrRefPicList,cColMv, iTempCenterAbsPartIdx,0 
#if QC_IC
            , bTmpICFlag
#endif
            ))
#endif
          {
            cDefaultMvField[uiCurrRefListId].setMvField(cColMv,0);
            bATMVPAvailFlag = true; // keep this variable here for later algrithm tuning
#if QC_IC
            rbICFlag = bTmpICFlag;
#endif
          }
        }
      }
    }
  }
  // The advanced TMVP candidate is considered as available if and only if the center block contains motion 
  if ( bATMVPAvailFlag == true )
  {   
    // perform ATMVP based on center now
#if !QC_SUB_PU_TMVP_V08 || QC_SUB_PU_TMVP_EXT
#if QC_MV_STORE_PRECISION_BIT
    iCurrPosX += ((cTempVector.getHor()+nOffset)>>QC_MV_STORE_PRECISION_BIT);
    iCurrPosY += ((cTempVector.getVer()+nOffset)>>QC_MV_STORE_PRECISION_BIT);
#else
    iCurrPosX += ((cTempVector.getHor()+2)>>2);
    iCurrPosY += ((cTempVector.getVer()+2)>>2);
#endif
#endif
    iInterDirSaved = (cDefaultMvField[0].getRefIdx() !=-1 ? 1: 0) + (cDefaultMvField[1].getRefIdx() !=-1 ? 2: 0);

    Int iPicWidth  = pColPic->getPicYuvRec()->getWidth()  - 1;
    Int iPicHeight = pColPic->getPicYuvRec()->getHeight() - 1;

    for (Int i=iCurrPosY; i < iCurrPosY + iHeight; i += iPUHeight)
      for (Int j = iCurrPosX; j < iCurrPosX + iWidth; j += iPUWidth)
      {
        iTempPosX     = j + iOffsetX;
        iTempPosY     = i + iOffsetY; 

        iTempPosX = Clip3( 0, iPicWidth,   iTempPosX  );
        iTempPosY = Clip3( 0, iPicHeight,  iTempPosY  );

        pColPic->getPicYuvRec()->getCUAddrAndPartIdx( iTempPosX, iTempPosY, iTempCUAddr, iTempAbsPartIdx );
        pcTempCU  = pColPic->getCU( iTempCUAddr );

        if( pcTempCU && !pcTempCU->isIntra(iTempAbsPartIdx) )
        {
          for( UInt uiCurrRefListId = 0; uiCurrRefListId < (bBSlice ? 2:1); uiCurrRefListId++ )
          {
            RefPicList  eCurrRefPicList = RefPicList( uiCurrRefListId );
#if QC_SUB_PU_TMVP_EXT && QC_SUB_PU_TMVP_V08==0
            if (deriveScaledMotionTemporalForOneDirection(pcTempCU,eCurrRefPicList, cColMv, iTempAbsPartIdx, 0 ,pColPic
#if QC_IC
              , bTmpICFlag
#endif
              ))
#else
            if (deriveScaledMotionTemporalForOneDirection(pcTempCU,eCurrRefPicList, cColMv, iTempAbsPartIdx, 0
#if QC_IC
              , bTmpICFlag
#endif
              ))
#endif
            {
              pcMvFieldSP[2*iPartition + uiCurrRefListId].setMvField(cColMv,0);
            }                
          }
        }
        else // intra coded, in this case, no motion vector is available for list 0 or list 1
        {
          pcMvFieldSP[2*iPartition + 0].setMvField(cDefaultMvField[0].getMv(),cDefaultMvField[0].getRefIdx());
          pcMvFieldSP[2*iPartition + 1].setMvField(cDefaultMvField[1].getMv(),cDefaultMvField[1].getRefIdx());
        }
        puhInterDirSP[iPartition] = (pcMvFieldSP[2*iPartition].getRefIdx()!=-1 ? 1: 0) + (pcMvFieldSP[2*iPartition+1].getRefIdx()!=-1 ? 2: 0);
        iPartition++;
      }  
      pcMvFieldDefault[0].setMvField(cDefaultMvField[0].getMv(),cDefaultMvField[0].getRefIdx());
      pcMvFieldDefault[1].setMvField(cDefaultMvField[1].getMv(),cDefaultMvField[1].getRefIdx());
      pcInterDirDefault[0] = iInterDirSaved ;
       
      if( bMrgIdxMatchATMVPCan) // only invoked at the decoder since here the parsing of the motion vectors are skipped
      {
        //store the sub-PU motion information
        UInt uiSPAddr;
        Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;
        getSPPara( iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);
        for (Int iPartitionIdx = 0; iPartitionIdx < iNumSP; iPartitionIdx++)
        {
          pDecCurrCU->getSPAbsPartIdx(uiDecCurrAbsPartIdx, iSPWidth, iSPHeight, iPartitionIdx, iNumSPInOneLine, uiSPAddr);
          pDecCurrCU->setInterDirSP(puhInterDirSP[iPartitionIdx], uiSPAddr, iSPWidth, iSPHeight);
          pDecCurrCU->getCUMvField( REF_PIC_LIST_0 )->setMvFieldSP(pDecCurrCU, uiSPAddr, pcMvFieldSP[2*iPartitionIdx], iSPWidth, iSPHeight);
          pDecCurrCU->getCUMvField( REF_PIC_LIST_1 )->setMvFieldSP(pDecCurrCU, uiSPAddr, pcMvFieldSP[2*iPartitionIdx + 1], iSPWidth, iSPHeight);
        }
      }
      return true;
  } 
  return false;
}
Void TComDataCU::getSPPara(Int iPUWidth, Int iPUHeight, Int& iNumSP, Int& iNumSPInOneLine, Int& iSPWidth, Int& iSPHeight)
{
  Int iSubPUSize = 1<<getSlice()->getSPS()->getSubPUTLog2Size();

  iNumSPInOneLine = iPUWidth/iSubPUSize;
  iNumSPInOneLine = iNumSPInOneLine < 1 ? 1: iNumSPInOneLine;
  Int iNumSPInOneColumn = iPUHeight/iSubPUSize;
  iNumSPInOneColumn = iNumSPInOneColumn < 1 ? 1: iNumSPInOneColumn;
  iNumSP = iNumSPInOneLine * iNumSPInOneColumn;

  iSPWidth = iNumSPInOneLine == 1 ? iPUWidth: iSubPUSize; 
  iSPHeight = iNumSPInOneColumn == 1 ? iPUHeight: iSubPUSize; 
}

Void TComDataCU::getSPAbsPartIdx(UInt uiBaseAbsPartIdx, Int iWidth, Int iHeight, Int iPartIdx, Int iNumPartLine, UInt& ruiPartAddr )
{
  uiBaseAbsPartIdx += m_uiAbsIdxInLCU;
  Int iBasePelX = g_auiRasterToPelX[g_auiZscanToRaster[uiBaseAbsPartIdx]];
  Int iBasePelY = g_auiRasterToPelY[g_auiZscanToRaster[uiBaseAbsPartIdx]];
  Int iCurrPelX = iBasePelX + iPartIdx%iNumPartLine * iWidth;
  Int iCurrPelY = iBasePelY + iPartIdx/iNumPartLine * iHeight;
  Int iCurrRaster = iCurrPelY / getPic()->getMinCUHeight() * getPic()->getNumPartInWidth() + iCurrPelX/getPic()->getMinCUWidth();
  ruiPartAddr = g_auiRasterToZscan[iCurrRaster];
  ruiPartAddr -= m_uiAbsIdxInLCU;  
}

Void TComDataCU::setInterDirSP( UInt uiDir, UInt uiAbsPartIdx, Int iWidth, Int iHeight )
{
  uiAbsPartIdx += getZorderIdxInCU();
  Int iStartPelX = g_auiRasterToPelX[g_auiZscanToRaster[uiAbsPartIdx]];
  Int iStartPelY = g_auiRasterToPelY[g_auiZscanToRaster[uiAbsPartIdx]];
  Int iEndPelX = iStartPelX + iWidth;
  Int iEndPelY = iStartPelY + iHeight;

  Int iCurrRaster, uiPartAddr;

  for (Int i=iStartPelY; i<iEndPelY; i+=getPic()->getMinCUHeight())
  {
    for (Int j=iStartPelX; j < iEndPelX; j += getPic()->getMinCUWidth())
    {
      iCurrRaster = i / getPic()->getMinCUHeight() * getPic()->getNumPartInWidth() + j/getPic()->getMinCUWidth();
      uiPartAddr = g_auiRasterToZscan[iCurrRaster];
      uiPartAddr -= getZorderIdxInCU();  

      m_puhInterDir[uiPartAddr] = uiDir;
    }
  }
}

#if QC_SUB_PU_TMVP_EXT && QC_SUB_PU_TMVP_V08==0
Bool TComDataCU::deriveScaledMotionTemporalForOneDirection( TComDataCU* pcTempCU,RefPicList eCurrRefPicList, TComMv &cColMv, UInt uiAbsPartIdx, Int iTargetRefIdx, TComPic *pColPic
#if QC_IC
  , Bool& rbICFlag
#endif
  )
#else
Bool TComDataCU::deriveScaledMotionTemporalForOneDirection( TComDataCU* pcTempCU,RefPicList eCurrRefPicList, TComMv &cColMv, UInt uiAbsPartIdx, Int iTargetRefIdx
#if QC_IC
  , Bool& rbICFlag
#endif
  )
#endif
{
  Int iColPOC, iColRefPOC, iCurrPOC, iCurrRefPOC, iScale;


#if QC_SUB_PU_TMVP_EXT && QC_SUB_PU_TMVP_V08==0
  Bool bAllowMirrorMV= false; 
  if ( pColPic == m_pcSlice->getRefPic(RefPicList(getSlice()->getColFromL0Flag()),getSlice()->getColRefIdx()))
  {
    bAllowMirrorMV =true;
  }
  else
  {
    if (getSlice()->getCheckLDC())
    {
      if (getWidth(0)  <= 32 && getHeight(0)  <= 32)
      {
        bAllowMirrorMV = true;
      }
    }
    else
    {
      if (getWidth(0)  <= 8 && getHeight(0)  <= 8)
      {
        bAllowMirrorMV = true;
      }
    }
  }
#endif

#if QC_SUB_PU_TMVP_V08
  RefPicList eColRefPicList       = eCurrRefPicList;
#else
#if QC_SUB_PU_TMVP_EXT
  RefPicList eColRefPicList = (getSlice()->getCheckLDC() || !bAllowMirrorMV) ? eCurrRefPicList :  RefPicList(getSlice()->getColFromL0Flag());
#else
  RefPicList eColRefPicList = getSlice()->getCheckLDC() ? eCurrRefPicList : RefPicList(getSlice()->getColFromL0Flag());
#endif
#endif
  // Although it might make sense to keep the unavailable motion field per direction still be unavailable, I made the MV prediction the same way as in TMVP
  // So there is an interaction between MV0 and MV1 of the corresponding blocks identified by TV.

  // Grab motion and do necessary scaling.{{
  iCurrPOC = m_pcSlice->getPOC();
  Int iColRefIdx = pcTempCU->getCUMvField(RefPicList(eColRefPicList))->getRefIdx(uiAbsPartIdx);
#if !QC_SUB_PU_TMVP_V08
#if QC_SUB_PU_TMVP_EXT
  if (iColRefIdx < 0 && bAllowMirrorMV)
#else
  if (iColRefIdx < 0 )
#endif
  {
    eColRefPicList = RefPicList(1 - eColRefPicList);
    iColRefIdx = pcTempCU->getCUMvField(RefPicList(eColRefPicList))->getRefIdx(uiAbsPartIdx);

    if (iColRefIdx < 0 )
    {
      return false;
    }
  }
#endif
  if ( iColRefIdx >=0 && iTargetRefIdx>=0 && iTargetRefIdx<m_pcSlice->getNumRefIdx(eCurrRefPicList)) 
  {
    iColPOC     = pcTempCU->getSlice()->getPOC();
    iColRefPOC  = pcTempCU->getSlice()->getRefPOC(eColRefPicList, iColRefIdx);
    //////////////////////////////////////////////////////////////
    // Set the target reference index to 0, may be changed later;//
    //////////////////////////////////////////////////////////////
    iCurrRefPOC = m_pcSlice->getRefPic(eCurrRefPicList, iTargetRefIdx)->getPOC();
    // Scale the vector.
    cColMv      = pcTempCU->getCUMvField(eColRefPicList)->getMv(uiAbsPartIdx);
    //pcMvFieldSP[2*iPartition + eCurrRefPicList].getMv();
    // Assume always short-term for now
    iScale = xGetDistScaleFactor(iCurrPOC, iCurrRefPOC, iColPOC, iColRefPOC);
    if ( iScale != 4096 )
      cColMv=cColMv.scaleMv( iScale );
#if QC_IC
    rbICFlag = pcTempCU->getICFlag( uiAbsPartIdx );
#endif
    return true;
  }
  return false;
}
TComPic * TComDataCU::getPicfromPOC(Int iPocColPic)
{
  TComPic * pColPic = NULL;
  for( UInt uiCurrRefListId = 0; uiCurrRefListId < (getSlice()->getSliceType() == B_SLICE ?  2 : 1 ) ; uiCurrRefListId++ )
  {
    RefPicList  eRefPicListX = RefPicList( uiCurrRefListId );
    for( UInt uiRefIdx =0; uiRefIdx<getSlice()->getNumRefIdx(eRefPicListX); uiRefIdx++)
    {
      if( getSlice()->getRefPic(eRefPicListX,uiRefIdx)->getPOC() == iPocColPic )
      {
        pColPic   = getSlice()->getRefPic(eRefPicListX,uiRefIdx);
        return pColPic;
      }
    }
  }
  return pColPic;
}
#endif 

/** Check whether the current PU and a spatial neighboring PU are in a same ME region.
 * \param xN, xN   location of the upper-left corner pixel of a neighboring PU
 * \param xP, yP   location of the upper-left corner pixel of the current PU
 * \returns Bool
 */
Bool TComDataCU::isDiffMER(Int xN, Int yN, Int xP, Int yP)
{

  UInt plevel = this->getSlice()->getPPS()->getLog2ParallelMergeLevelMinus2() + 2;
  if ((xN>>plevel)!= (xP>>plevel))
  {
    return true;
  }
  if ((yN>>plevel)!= (yP>>plevel))
  {
    return true;
  }
  return false;
}
/** calculate the location of upper-left corner pixel and size of the current PU.
 * \param partIdx  PU index within a CU
 * \param xP, yP   location of the upper-left corner pixel of the current PU
 * \param PSW, nPSH    size of the curren PU
 * \returns Void
 */
Void TComDataCU::getPartPosition( UInt partIdx, Int& xP, Int& yP, Int& nPSW, Int& nPSH)
{
  UInt col = m_uiCUPelX;
  UInt row = m_uiCUPelY;

  switch ( m_pePartSize[0] )
  {
  case SIZE_2NxN:
    nPSW = getWidth(0);      
    nPSH = getHeight(0) >> 1; 
    xP   = col;
    yP   = (partIdx ==0)? row: row + nPSH;
    break;
  case SIZE_Nx2N:
    nPSW = getWidth(0) >> 1; 
    nPSH = getHeight(0);      
    xP   = (partIdx ==0)? col: col + nPSW;
    yP   = row;
    break;
  case SIZE_NxN:
    nPSW = getWidth(0) >> 1; 
    nPSH = getHeight(0) >> 1; 
    xP   = col + (partIdx&0x1)*nPSW;
    yP   = row + (partIdx>>1)*nPSH;
    break;
  case SIZE_2NxnU:
    nPSW = getWidth(0);
    nPSH = ( partIdx == 0 ) ?  getHeight(0) >> 2 : ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 );
    xP   = col;
    yP   = (partIdx ==0)? row: row + getHeight(0) - nPSH;

    break;
  case SIZE_2NxnD:
    nPSW = getWidth(0);
    nPSH = ( partIdx == 0 ) ?  ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 ) : getHeight(0) >> 2;
    xP   = col;
    yP   = (partIdx ==0)? row: row + getHeight(0) - nPSH;
    break;
  case SIZE_nLx2N:
    nPSW = ( partIdx == 0 ) ? getWidth(0) >> 2 : ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 );
    nPSH = getHeight(0);
    xP   = (partIdx ==0)? col: col + getWidth(0) - nPSW;
    yP   = row;
    break;
  case SIZE_nRx2N:
    nPSW = ( partIdx == 0 ) ? ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 ) : getWidth(0) >> 2;
    nPSH = getHeight(0);
    xP   = (partIdx ==0)? col: col + getWidth(0) - nPSW;
    yP   = row;
    break;
  default:
    assert ( m_pePartSize[0] == SIZE_2Nx2N );
    nPSW = getWidth(0);      
    nPSH = getHeight(0);      
    xP   = col ;
    yP   = row ;

    break;
  }
}

/** Constructs a list of candidates for AMVP
 * \param uiPartIdx
 * \param uiPartAddr 
 * \param eRefPicList
 * \param iRefIdx
 * \param pInfo
 */
Void TComDataCU::fillMvpCand ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AMVPInfo* pInfo 
#if QC_FRUC_MERGE
  , TComPrediction * pPred
#endif
  )
{
  TComMv cMvPred;
  Bool bAddedSmvp = false;

  pInfo->iN = 0;  
  if (iRefIdx < 0)
  {
    return;
  }
  
  //-- Get Spatial MV
  UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB;
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  Bool bAdded = false;
  
  deriveLeftRightTopIdx( uiPartIdx, uiPartIdxLT, uiPartIdxRT );
  deriveLeftBottomIdx( uiPartIdx, uiPartIdxLB );
  
  TComDataCU* tmpCU = NULL;
  UInt idx;
  tmpCU = getPUBelowLeft(idx, uiPartIdxLB);
  bAddedSmvp = (tmpCU != NULL) && (tmpCU->getPredictionMode(idx) != MODE_INTRA);

  if (!bAddedSmvp)
  {
    tmpCU = getPULeft(idx, uiPartIdxLB);
    bAddedSmvp = (tmpCU != NULL) && (tmpCU->getPredictionMode(idx) != MODE_INTRA);
  }

  // Left predictor search
  bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_BELOW_LEFT);
  if (!bAdded) 
  {
    bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_LEFT );
  }
  
  if(!bAdded)
  {
    bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_BELOW_LEFT);
    if (!bAdded) 
    {
      xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_LEFT );
    }
  }
  
  // Above predictor search
  bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE_RIGHT);

  if (!bAdded) 
  {
    bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE);
  }

  if(!bAdded)
  {
    xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxLT, MD_ABOVE_LEFT);
  }

  if (!bAddedSmvp)
  {
    bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE_RIGHT);
    if (!bAdded) 
    {
      bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE);
    }

    if(!bAdded)
    {
      xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxLT, MD_ABOVE_LEFT);
    }
  }

#if QC_IMV
  if( getiMVFlag( uiPartAddr ) && getSlice()->getSPS()->getIMV() )
  {
    for( Int i = 0; i < pInfo->iN; i++ )
    {
      xRoundMV( pInfo->m_acMvCand[i] );
    }
  }
#endif

  if ( pInfo->iN == 2 )
  {
    if ( pInfo->m_acMvCand[ 0 ] == pInfo->m_acMvCand[ 1 ] )
    {
      pInfo->iN = 1;
    }
  }

  if ( getSlice()->getEnableTMVPFlag() )
  {
    // Get Temporal Motion Predictor
    Int iRefIdx_Col = iRefIdx;
    TComMv cColMv;
    UInt uiPartIdxRB;
    UInt uiAbsPartIdx;  
    UInt uiAbsPartAddr;

    deriveRightBottomIdx( uiPartIdx, uiPartIdxRB );
    uiAbsPartAddr = m_uiAbsIdxInLCU + uiPartAddr;

    //----  co-located RightBottom Temporal Predictor (H) ---//
    uiAbsPartIdx = g_auiZscanToRaster[uiPartIdxRB];
    Int uiLCUIdx = -1;
    if ( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdx] + m_pcPic->getMinCUWidth() ) >= m_pcSlice->getSPS()->getPicWidthInLumaSamples() )  // image boundary check
    {
    }
    else if ( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdx] + m_pcPic->getMinCUHeight() ) >= m_pcSlice->getSPS()->getPicHeightInLumaSamples() )
    {
    }
    else
    {
      if ( ( uiAbsPartIdx % uiNumPartInCUWidth < uiNumPartInCUWidth - 1 ) &&           // is not at the last column of LCU 
        ( uiAbsPartIdx / uiNumPartInCUWidth < m_pcPic->getNumPartInHeight() - 1 ) ) // is not at the last row    of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdx + uiNumPartInCUWidth + 1 ];
        uiLCUIdx = getAddr();
      }
      else if ( uiAbsPartIdx % uiNumPartInCUWidth < uiNumPartInCUWidth - 1 )           // is not at the last column of LCU But is last row of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ (uiAbsPartIdx + uiNumPartInCUWidth + 1) % m_pcPic->getNumPartInCU() ];
      }
      else if ( uiAbsPartIdx / uiNumPartInCUWidth < m_pcPic->getNumPartInHeight() - 1 ) // is not at the last row of LCU But is last column of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdx + 1 ];
        uiLCUIdx = getAddr() + 1;
      }
      else //is the right bottom corner of LCU                       
      {
        uiAbsPartAddr = 0;
      }
    }
    if ( uiLCUIdx >= 0 && xGetColMVP( eRefPicList, uiLCUIdx, uiAbsPartAddr, cColMv, iRefIdx_Col ) )
    {
      pInfo->m_acMvCand[pInfo->iN++] = cColMv;
    }
    else 
    {
      UInt uiPartIdxCenter;
      UInt uiCurLCUIdx = getAddr();
      xDeriveCenterIdx( uiPartIdx, uiPartIdxCenter );
      if (xGetColMVP( eRefPicList, uiCurLCUIdx, uiPartIdxCenter,  cColMv, iRefIdx_Col ))
      {
        pInfo->m_acMvCand[pInfo->iN++] = cColMv;
      }
    }
    //----  co-located RightBottom Temporal Predictor  ---//
  }

#if QC_FRUC_MERGE
  if( getSlice()->getSPS()->getUseFRUCMgrMode() )
  {
    if( pPred != NULL && pPred->deriveFRUCMV( this , getDepth( uiPartAddr ) , uiPartAddr , uiPartIdx , iRefIdx , eRefPicList ) )
    {
      const TComMv & rMV = getCUMvField( eRefPicList )->getMv( uiPartAddr );
      if( pInfo->iN == 0 )
      {
        pInfo->m_acMvCand[0] = rMV;
        pInfo->iN++;
      }
      else if( pInfo->m_acMvCand[0] != rMV )
      {
        for( Int n = min( pInfo->iN , AMVP_MAX_NUM_CANDS - 1 ) ; n > 0 ; n-- )
        {
          pInfo->m_acMvCand[n] = pInfo->m_acMvCand[n-1];
        }
        pInfo->m_acMvCand[0] = rMV;
        pInfo->iN = min( pInfo->iN + 1 , AMVP_MAX_NUM_CANDS );
      }
    }
  }
#endif

  if (pInfo->iN > AMVP_MAX_NUM_CANDS)
  {
    pInfo->iN = AMVP_MAX_NUM_CANDS;
  }
  while (pInfo->iN < AMVP_MAX_NUM_CANDS)
  {
      pInfo->m_acMvCand[pInfo->iN].set(0,0);
      pInfo->iN++;
  }

#if QC_MV_STORE_PRECISION_BIT
  for( Int i = 0 ; i < pInfo->iN ; i++ )
  {
    pInfo->m_acMvCand[i].roundMV2SignalPrecision();
  }
#endif

#if QC_IMV
  if( getiMVFlag( uiPartAddr ) && getSlice()->getSPS()->getIMV() )
  {
    for( Int i = 0 ; i < pInfo->iN ; i++ )
    {
      xRoundMV( pInfo->m_acMvCand[i] );
    }
  }
#endif

  return ;
}

Bool TComDataCU::isBipredRestriction(UInt puIdx)
{
#if QC_HEVC_MOTION_CONSTRAINT_REMOVAL
  if (getSlice()->getSPS()->getAtmvpEnableFlag())
    return false;
  else {
#endif
  Int width = 0;
  Int height = 0;
  UInt partAddr;

  getPartIndexAndSize( puIdx, partAddr, width, height );
  if ( getWidth(0) == 8 && (width < 8 || height < 8) )
  {
    return true;
  }
  return false;
#if QC_HEVC_MOTION_CONSTRAINT_REMOVAL
  }
#endif
}

Void TComDataCU::clipMv    (TComMv&  rcMv)
{
#if QC_MV_STORE_PRECISION_BIT
  Int  iMvShift = QC_MV_STORE_PRECISION_BIT;
#else
  Int  iMvShift = 2;
#endif
  Int iOffset = 8;
  Int iHorMax = ( m_pcSlice->getSPS()->getPicWidthInLumaSamples() + iOffset - m_uiCUPelX - 1 ) << iMvShift;
  Int iHorMin = (       -(Int)g_uiMaxCUWidth - iOffset - (Int)m_uiCUPelX + 1 ) << iMvShift;
  
  Int iVerMax = ( m_pcSlice->getSPS()->getPicHeightInLumaSamples() + iOffset - m_uiCUPelY - 1 ) << iMvShift;
  Int iVerMin = (       -(Int)g_uiMaxCUHeight - iOffset - (Int)m_uiCUPelY + 1 ) << iMvShift;
  
  rcMv.setHor( min (iHorMax, max (iHorMin, rcMv.getHor())) );
  rcMv.setVer( min (iVerMax, max (iVerMin, rcMv.getVer())) );
}

#if INTRA_KLT
Void TComDataCU::clipMvIntraConstraint(Int regionId, Int &iHorMin, Int &iHorMax, Int &iVerMin, Int &iVerMax, Int iRange, UInt uiTemplateSize, UInt uiBlkSize, Int iCurrY, Int iCurrX, Int offsetLCUY, Int offsetLCUX)
{
  Int  iMvShift = 0;
  Int iTemplateSize = uiTemplateSize;
  Int iBlkSize = uiBlkSize;
  if (regionId == 0) //above outside LCU
  {
    iHorMax = min((iCurrX + iRange) << iMvShift, (Int)((m_pcSlice->getSPS()->getPicWidthInLumaSamples() - iBlkSize) << iMvShift));
    iHorMin = max((iTemplateSize) << iMvShift, (iCurrX - offsetLCUX - iBlkSize + 1) << iMvShift);

    iVerMax = (iCurrY - iBlkSize - offsetLCUY) << iMvShift;
    iVerMin = max(((iTemplateSize) << iMvShift), ((iCurrY - iRange) << iMvShift));

    iHorMin = iHorMin - iCurrX; 
    iHorMax = iHorMax - iCurrX;
    iVerMax = iVerMax - iCurrY;
    iVerMin = iVerMin - iCurrY;
  }
  else if (regionId == 1) //left outside LCU
  {
    iHorMax = (iCurrX - offsetLCUX - iBlkSize) << iMvShift;
    iHorMin = max((iTemplateSize) << iMvShift, (iCurrX - iRange) << iMvShift);

    iVerMin = max((iTemplateSize) << iMvShift, (iCurrY - iBlkSize - offsetLCUY) << iMvShift);
    iVerMax = (iCurrY) << iMvShift;

    iHorMin = iHorMin - iCurrX; 
    iHorMax = iHorMax - iCurrX;
    iVerMax = iVerMax - iCurrY;
    iVerMin = iVerMin - iCurrY;
  }
  else if (regionId == 2) //left outside LCU (can reach the bottom row of LCU)
  {
    iHorMin = max((iTemplateSize) << iMvShift, (iCurrX - iRange) << iMvShift);
    iHorMax = (iCurrX - offsetLCUX - iBlkSize) << iMvShift;
    iVerMin = (iCurrY + 1) << iMvShift;
    iVerMax = min(m_pcSlice->getSPS()->getPicHeightInLumaSamples() - iBlkSize, (iCurrY - offsetLCUY + g_uiMaxCUHeight - iBlkSize) << iMvShift);

    iHorMin = iHorMin - iCurrX; 
    iHorMax = iHorMax - iCurrX;
    iVerMax = iVerMax - iCurrY;
    iVerMin = iVerMin - iCurrY;
  }
}
#endif

UInt TComDataCU::getIntraSizeIdx(UInt uiAbsPartIdx)
{
  UInt uiShift = ( m_pePartSize[uiAbsPartIdx]==SIZE_NxN ? 1 : 0 );
  
#if QC_LARGE_CTU
  UShort
#else
  UChar 
#endif
    uiWidth = m_puhWidth[uiAbsPartIdx]>>uiShift;
  UInt  uiCnt = 0;
  while( uiWidth )
  {
    uiCnt++;
    uiWidth>>=1;
  }
  uiCnt-=2;
  return uiCnt > 6 ? 6 : uiCnt;
}

Void TComDataCU::clearCbf( UInt uiIdx, TextType eType, UInt uiNumParts )
{
  ::memset( &m_puhCbf[g_aucConvertTxtTypeToIdx[eType]][uiIdx], 0, sizeof(UChar)*uiNumParts);
}

/** Set a I_PCM flag for all sub-partitions of a partition.
 * \param bIpcmFlag I_PCM flag
 * \param uiAbsPartIdx patition index
 * \param uiDepth CU depth
 * \returns Void
 */
Void TComDataCU::setIPCMFlagSubParts  (Bool bIpcmFlag, UInt uiAbsPartIdx, UInt uiDepth)
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  memset(m_pbIPCMFlag + uiAbsPartIdx, bIpcmFlag, sizeof(Bool)*uiCurrPartNumb );
}

/** Test whether the current block is skipped
 * \param uiPartIdx Block index
 * \returns Flag indicating whether the block is skipped
 */
Bool TComDataCU::isSkipped( UInt uiPartIdx )
{
  return ( getSkipFlag( uiPartIdx ) );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

Bool TComDataCU::xAddMVPCand( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir )
{
  TComDataCU* pcTmpCU = NULL;
  UInt uiIdx;
  switch( eDir )
  {
    case MD_LEFT:
    {
      pcTmpCU = getPULeft(uiIdx, uiPartUnitIdx);
      break;
    }
    case MD_ABOVE:
    {
      pcTmpCU = getPUAbove(uiIdx, uiPartUnitIdx );
      break;
    }
    case MD_ABOVE_RIGHT:
    {
      pcTmpCU = getPUAboveRight(uiIdx, uiPartUnitIdx);
      break;
    }
    case MD_BELOW_LEFT:
    {
      pcTmpCU = getPUBelowLeft(uiIdx, uiPartUnitIdx);
      break;
    }
    case MD_ABOVE_LEFT:
    {
      pcTmpCU = getPUAboveLeft(uiIdx, uiPartUnitIdx);
      break;
    }
    default:
    {
      break;
    }
  }

  if ( pcTmpCU == NULL )
  {
    return false;
  }
  
  if ( pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) >= 0 && m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getPOC() == pcTmpCU->getSlice()->getRefPOC( eRefPicList, pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) ))
  {
    TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList)->getMv(uiIdx);
    
    pInfo->m_acMvCand[ pInfo->iN++] = cMvPred;
    return true;
  }

  RefPicList eRefPicList2nd = REF_PIC_LIST_0;
  if(       eRefPicList == REF_PIC_LIST_0 )
  {
    eRefPicList2nd = REF_PIC_LIST_1;
  }
  else if ( eRefPicList == REF_PIC_LIST_1)
  {
    eRefPicList2nd = REF_PIC_LIST_0;
  }


  Int iCurrRefPOC = m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getPOC();
  Int iNeibRefPOC;


  if( pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) >= 0 )
  {
    iNeibRefPOC = pcTmpCU->getSlice()->getRefPOC( eRefPicList2nd, pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) );
    if( iNeibRefPOC == iCurrRefPOC ) // Same Reference Frame But Diff List//
    {
      TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList2nd)->getMv(uiIdx);
      pInfo->m_acMvCand[ pInfo->iN++] = cMvPred;
      return true;
    }
  }
  return false;
}

/** 
 * \param pInfo
 * \param eRefPicList 
 * \param iRefIdx
 * \param uiPartUnitIdx
 * \param eDir
 * \returns Bool
 */
Bool TComDataCU::xAddMVPCandOrder( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir )
{
  TComDataCU* pcTmpCU = NULL;
  UInt uiIdx;
  switch( eDir )
  {
  case MD_LEFT:
    {
      pcTmpCU = getPULeft(uiIdx, uiPartUnitIdx);
      break;
    }
  case MD_ABOVE:
    {
      pcTmpCU = getPUAbove(uiIdx, uiPartUnitIdx);
      break;
    }
  case MD_ABOVE_RIGHT:
    {
      pcTmpCU = getPUAboveRight(uiIdx, uiPartUnitIdx);
      break;
    }
  case MD_BELOW_LEFT:
    {
      pcTmpCU = getPUBelowLeft(uiIdx, uiPartUnitIdx);
      break;
    }
  case MD_ABOVE_LEFT:
    {
      pcTmpCU = getPUAboveLeft(uiIdx, uiPartUnitIdx);
      break;
    }
  default:
    {
      break;
    }
  }

  if ( pcTmpCU == NULL ) 
  {
    return false;
  }
  
  RefPicList eRefPicList2nd = REF_PIC_LIST_0;
  if( eRefPicList == REF_PIC_LIST_0 )
  {
    eRefPicList2nd = REF_PIC_LIST_1;
  }
  else if ( eRefPicList == REF_PIC_LIST_1)
  {
    eRefPicList2nd = REF_PIC_LIST_0;
  }

  Int iCurrPOC = m_pcSlice->getPOC();
  Int iCurrRefPOC = m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getPOC();
  Int iNeibPOC = iCurrPOC;
  Int iNeibRefPOC;

  Bool bIsCurrRefLongTerm = m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getIsLongTerm();
  Bool bIsNeibRefLongTerm = false;
  //---------------  V1 (END) ------------------//
  if( pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) >= 0)
  {
    iNeibRefPOC = pcTmpCU->getSlice()->getRefPOC( eRefPicList, pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) );
    TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList)->getMv(uiIdx);
    TComMv rcMv;

    bIsNeibRefLongTerm = pcTmpCU->getSlice()->getRefPic( eRefPicList, pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) )->getIsLongTerm();
    if ( bIsCurrRefLongTerm == bIsNeibRefLongTerm ) 
    {
      if ( bIsCurrRefLongTerm || bIsNeibRefLongTerm )
      {
        rcMv = cMvPred;
      }
      else
      {
        Int iScale = xGetDistScaleFactor( iCurrPOC, iCurrRefPOC, iNeibPOC, iNeibRefPOC );
        if ( iScale == 4096 )
        {
          rcMv = cMvPred;
        }
        else
        {
          rcMv = cMvPred.scaleMv( iScale );
        }
      }
      pInfo->m_acMvCand[ pInfo->iN++] = rcMv;
      return true;
    }
  }
  //---------------------- V2(END) --------------------//
  if( pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) >= 0)
  {
    iNeibRefPOC = pcTmpCU->getSlice()->getRefPOC( eRefPicList2nd, pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) );
    TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList2nd)->getMv(uiIdx);
    TComMv rcMv;

    bIsNeibRefLongTerm = pcTmpCU->getSlice()->getRefPic( eRefPicList2nd, pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) )->getIsLongTerm();
    if ( bIsCurrRefLongTerm == bIsNeibRefLongTerm ) 
    {
    if ( bIsCurrRefLongTerm || bIsNeibRefLongTerm )
    {
      rcMv = cMvPred;
    }
    else
    {
      Int iScale = xGetDistScaleFactor( iCurrPOC, iCurrRefPOC, iNeibPOC, iNeibRefPOC );
      if ( iScale == 4096 )
      {
        rcMv = cMvPred;
      }
      else
      {
        rcMv = cMvPred.scaleMv( iScale );
      }
    }
    pInfo->m_acMvCand[ pInfo->iN++] = rcMv;
    return true;
    }
  }
  //---------------------- V3(END) --------------------//
  return false;
}

/** 
 * \param eRefPicList
 * \param uiCUAddr 
 * \param uiPartUnitIdx
 * \param riRefIdx
 * \returns Bool
 */
#if QC_IC
Bool TComDataCU::xGetColMVP( RefPicList eRefPicList, Int uiCUAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx, Bool* bICFlag )
#else
Bool TComDataCU::xGetColMVP( RefPicList eRefPicList, Int uiCUAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx )
#endif
{
  UInt uiAbsPartAddr = uiPartUnitIdx;

  RefPicList  eColRefPicList;
  Int iColPOC, iColRefPOC, iCurrPOC, iCurrRefPOC, iScale;
  TComMv cColMv;

  // use coldir.
  TComPic *pColPic = getSlice()->getRefPic( RefPicList(getSlice()->isInterB() ? 1-getSlice()->getColFromL0Flag() : 0), getSlice()->getColRefIdx());
  TComDataCU *pColCU = pColPic->getCU( uiCUAddr );
  if(pColCU->getPic()==0||pColCU->getPartitionSize(uiPartUnitIdx)==SIZE_NONE)
  {
    return false;
  }
  iCurrPOC = m_pcSlice->getPOC();    
  iColPOC = pColCU->getSlice()->getPOC();  

  if (pColCU->isIntra(uiAbsPartAddr))
  {
    return false;
  }
  eColRefPicList = getSlice()->getCheckLDC() ? eRefPicList : RefPicList(getSlice()->getColFromL0Flag());

  Int iColRefIdx = pColCU->getCUMvField(RefPicList(eColRefPicList))->getRefIdx(uiAbsPartAddr);

  if (iColRefIdx < 0 )
  {
    eColRefPicList = RefPicList(1 - eColRefPicList);
    iColRefIdx = pColCU->getCUMvField(RefPicList(eColRefPicList))->getRefIdx(uiAbsPartAddr);

    if (iColRefIdx < 0 )
    {
      return false;
    }
  }

  // Scale the vector.
  iColRefPOC = pColCU->getSlice()->getRefPOC(eColRefPicList, iColRefIdx);
  cColMv = pColCU->getCUMvField(eColRefPicList)->getMv(uiAbsPartAddr);

  iCurrRefPOC = m_pcSlice->getRefPic(eRefPicList, riRefIdx)->getPOC();
  Bool bIsCurrRefLongTerm = m_pcSlice->getRefPic(eRefPicList, riRefIdx)->getIsLongTerm();
  Bool bIsColRefLongTerm = pColCU->getSlice()->getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);

  if ( bIsCurrRefLongTerm != bIsColRefLongTerm ) 
  {
    return false;
  }
#if QC_IC
  if( bICFlag != NULL )
  {
    *bICFlag = pColCU->getICFlag( uiAbsPartAddr ) ;
  }
#endif
  if ( bIsCurrRefLongTerm || bIsColRefLongTerm )
  {
    rcMv = cColMv;
  }
  else
  {
    iScale = xGetDistScaleFactor(iCurrPOC, iCurrRefPOC, iColPOC, iColRefPOC);
    if ( iScale == 4096 )
    {
      rcMv = cColMv;
    }
    else
    {
      rcMv = cColMv.scaleMv( iScale );
    }
  }
  return true;
}

#if QC_FRUC_MERGE
TComMv TComDataCU::scaleMV( const TComMv & rColMV , Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC )
{
  TComMv mv = rColMV;
  Int iScale = xGetDistScaleFactor(iCurrPOC, iCurrRefPOC, iColPOC, iColRefPOC);
  if ( iScale != 4096 )
  {
    mv = rColMV.scaleMv( iScale );
  }
  return( mv );
}

Bool TComDataCU::isSameMVField( RefPicList eListA , TComMvField & rMVFieldA , RefPicList eListB , TComMvField & rMVFieldB )
{
  if( rMVFieldA.getRefIdx() >= 0 && rMVFieldB.getRefIdx() >= 0 )
  {
    return( rMVFieldA.getMv() == rMVFieldB.getMv()
      && getSlice()->getRefPOC( eListA , rMVFieldA.getRefIdx() ) == getSlice()->getRefPOC( eListB , rMVFieldB.getRefIdx() ) );
  }
  else
  {
    return( false );
  }
}
#endif

UInt TComDataCU::xGetMvdBits(TComMv cMvd)
{
  return ( xGetComponentBits(cMvd.getHor()) + xGetComponentBits(cMvd.getVer()) );
}

UInt TComDataCU::xGetComponentBits(Int iVal)
{
  UInt uiLength = 1;
  UInt uiTemp   = ( iVal <= 0) ? (-iVal<<1)+1: (iVal<<1);
  
  assert ( uiTemp );
  
  while ( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  
  return uiLength;
}


Int TComDataCU::xGetDistScaleFactor(Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC)
{
  Int iDiffPocD = iColPOC - iColRefPOC;
  Int iDiffPocB = iCurrPOC - iCurrRefPOC;
  
  if( iDiffPocD == iDiffPocB )
  {
    return 4096;
  }
  else
  {
    Int iTDB      = Clip3( -128, 127, iDiffPocB );
    Int iTDD      = Clip3( -128, 127, iDiffPocD );
#if QC_FRUC_MERGE
    Int iScale = getSlice()->getScaleFactor( iTDB , iTDD );
#else
    Int iX        = (0x4000 + abs(iTDD/2)) / iTDD;
    Int iScale    = Clip3( -4096, 4095, (iTDB * iX + 32) >> 6 );
#endif
    return iScale;
  }
}

/** 
 * \param eCUMode
 * \param uiPartIdx 
 * \param ruiPartIdxCenter
 * \returns Void
 */
Void TComDataCU::xDeriveCenterIdx( UInt uiPartIdx, UInt& ruiPartIdxCenter )
{
  UInt uiPartAddr;
  Int  iPartWidth;
  Int  iPartHeight;
  getPartIndexAndSize( uiPartIdx, uiPartAddr, iPartWidth, iPartHeight);
  
  ruiPartIdxCenter = m_uiAbsIdxInLCU+uiPartAddr; // partition origin.
  ruiPartIdxCenter = g_auiRasterToZscan[ g_auiZscanToRaster[ ruiPartIdxCenter ]
                                        + ( iPartHeight/m_pcPic->getMinCUHeight()  )/2*m_pcPic->getNumPartInWidth()
                                        + ( iPartWidth/m_pcPic->getMinCUWidth()  )/2];
}

Void TComDataCU::compressMV()
{
  Int scaleFactor = 4 * AMVP_DECIMATION_FACTOR / m_unitSize;
  if (scaleFactor > 0)
  {
    m_acCUMvField[0].compress(m_pePredMode, scaleFactor);
    m_acCUMvField[1].compress(m_pePredMode, scaleFactor);    
  }
}

#if QC_USE_65ANG_MODES
Bool TComDataCU::getUseExtIntraAngModes(UInt uiWidth)
{
  Bool bUseExtIntraAngModes;
  switch (uiWidth)
  {
    case  2:
    case  4:
    case  8:
    case 16:
    case 32:
    case 64:
    default: bUseExtIntraAngModes = m_pcSlice->getSPS()->getUseExtIntraAngModes()&1; break;
  }
  return bUseExtIntraAngModes;
}
#endif

UInt TComDataCU::getCoefScanIdx(UInt uiAbsPartIdx, UInt uiWidth, Bool bIsLuma, Bool bIsIntra)
{
  UInt uiCTXIdx;
  UInt uiScanIdx;
  UInt uiDirMode;

  if ( !bIsIntra ) 
  {
    uiScanIdx = SCAN_DIAG;
    return uiScanIdx;
  }

  switch(uiWidth)
  {
    case  2: uiCTXIdx = 6; break;
    case  4: uiCTXIdx = 5; break;
    case  8: uiCTXIdx = 4; break;
    case 16: uiCTXIdx = 3; break;
    case 32: uiCTXIdx = 2; break;
    case 64: uiCTXIdx = 1; break;
    default: uiCTXIdx = 0; break;
  }

  if ( bIsLuma )
  {
    uiDirMode = getLumaIntraDir(uiAbsPartIdx);
    uiScanIdx = SCAN_DIAG;
    if (uiCTXIdx >3 && uiCTXIdx < 6) //if multiple scans supported for transform size
    {
#if QC_USE_65ANG_MODES
      uiScanIdx = abs((Int) uiDirMode - VER_IDX) < 10 ? SCAN_HOR : (abs((Int)uiDirMode - HOR_IDX) < 10 ? SCAN_VER : SCAN_DIAG);
#else
      uiScanIdx = abs((Int) uiDirMode - VER_IDX) < 5 ? SCAN_HOR : (abs((Int)uiDirMode - HOR_IDX) < 5 ? SCAN_VER : SCAN_DIAG);
#endif
    }
  }
  else
  {
    uiDirMode = getChromaIntraDir(uiAbsPartIdx);
    if( uiDirMode == DM_CHROMA_IDX )
    {
      // get number of partitions in current CU
      UInt depth = getDepth(uiAbsPartIdx);
      UInt numParts = getPic()->getNumPartInCU() >> (2 * depth);
      
      // get luma mode from upper-left corner of current CU
      uiDirMode = getLumaIntraDir((uiAbsPartIdx/numParts)*numParts);
    }
    uiScanIdx = SCAN_DIAG;
    if (uiCTXIdx >4 && uiCTXIdx < 7) //if multiple scans supported for transform size
    {
#if QC_USE_65ANG_MODES
      uiScanIdx = abs((Int) uiDirMode - VER_IDX) < 10 ? SCAN_HOR : (abs((Int)uiDirMode - HOR_IDX) < 10 ? SCAN_VER : SCAN_DIAG);
#else
      uiScanIdx = abs((Int) uiDirMode - VER_IDX) < 5 ? SCAN_HOR : (abs((Int)uiDirMode - HOR_IDX) < 5 ? SCAN_VER : SCAN_DIAG);
#endif
    }
  }

  return uiScanIdx;
}

UInt TComDataCU::getSCUAddr()
{ 
  return getPic()->getPicSym()->getInverseCUOrderMap(m_uiCUAddr)*(1<<(m_pcSlice->getSPS()->getMaxCUDepth()<<1))+m_uiAbsIdxInLCU; 
}

#if ALF_HM3_QC_REFACTOR
UInt TComDataCU::getCtxAlfCtrlFlag( UInt uiAbsPartIdx )
{
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  UInt        uiCtx = 0;

  // Get BCBP of left PU
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx    = ( pcTempCU ) ? pcTempCU->getAlfCtrlFlag( uiTempPartIdx ) : 0;

  // Get BCBP of above PU
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
  uiCtx   += ( pcTempCU ) ? pcTempCU->getAlfCtrlFlag( uiTempPartIdx ) : 0;

  return uiCtx;
}

Void TComDataCU::setAlfCtrlFlagSubParts         ( UInt uiFlag, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  for (UInt ui = 0; ui < uiCurrPartNumb; ui++ )
  {
    m_puiAlfCtrlFlag[uiAbsPartIdx + ui] = uiFlag;
  }
}

Void TComDataCU::createTmpAlfCtrlFlag()
{
  m_puiTmpAlfCtrlFlag = (UInt* )xMalloc(UInt, m_uiNumPartition);
}

Void TComDataCU::destroyTmpAlfCtrlFlag()
{
  if(m_puiTmpAlfCtrlFlag)
  {
    xFree(m_puiTmpAlfCtrlFlag);        m_puiTmpAlfCtrlFlag = NULL;
  }
}

Void TComDataCU::copyAlfCtrlFlagToTmp()
{
  memcpy( m_puiTmpAlfCtrlFlag, m_puiAlfCtrlFlag, sizeof(UInt)*m_uiNumPartition );
}

Void TComDataCU::copyAlfCtrlFlagFromTmp()
{
  memcpy( m_puiAlfCtrlFlag, m_puiTmpAlfCtrlFlag, sizeof(UInt)*m_uiNumPartition );
}
#endif

#if QC_OBMC
// Function for fetching neighboring motions.
Bool TComDataCU::getNeigMotion( UInt uiAbsPartIdx, TComMvField cNeigMvField[2], Int &irNeigPredDir, Int iDir, TComMvField cCurMvField[2], Int &iCurrDir, UInt uiZeroIdx, Bool &bTobeStored )
{
  TComDataCU* pcTmpCU = NULL;
  UInt uiIdx;
  if( iDir == 0 ) //above
  {
    pcTmpCU = getPUAbove( uiIdx, uiAbsPartIdx + uiZeroIdx );
  }  
  else if( iDir == 1 ) //left
  {
    pcTmpCU = getPULeft( uiIdx, uiAbsPartIdx + uiZeroIdx );
  }
  else if( iDir == 2 ) //below
  {
    pcTmpCU = this;
    uiIdx = g_auiRasterToZscan[g_auiZscanToRaster[uiAbsPartIdx + uiZeroIdx] + getPic()->getNumPartInWidth()] - uiZeroIdx;
  }
  else if( iDir == 3 ) //right
  {
    pcTmpCU = this;
    uiIdx = g_auiRasterToZscan[g_auiZscanToRaster[uiAbsPartIdx + uiZeroIdx] + 1] - uiZeroIdx;
  }

  if( pcTmpCU == NULL || pcTmpCU->isIntra( uiIdx ) )
  {
    return false;
  }

  irNeigPredDir = pcTmpCU->getInterDir( uiIdx );
  if(irNeigPredDir)
  { 
    if( !bTobeStored )
    {
      //backup motion information
      for(UInt iRefList = 0; iRefList < 2; iRefList ++)
      {
        TComCUMvField* pTmpMvField = getCUMvField( RefPicList(iRefList) );
        cCurMvField[iRefList].setMvField( pTmpMvField->getMv( uiAbsPartIdx ), pTmpMvField->getRefIdx( uiAbsPartIdx ) );
      }
      iCurrDir = getInterDir( uiAbsPartIdx );
      bTobeStored = true;
    }
    for(UInt iRefList = 0; iRefList < 2; iRefList ++)
    {
      TComCUMvField* pTmpMvField = pcTmpCU->getCUMvField( RefPicList(iRefList) );
      cNeigMvField[iRefList].setMvField( pTmpMvField->getMv( uiIdx ), pTmpMvField->getRefIdx( uiIdx ) );
    }

    if(irNeigPredDir != iCurrDir)
    {
      return true;
    }
    else
    {
      for(UInt iRefList = 0; iRefList < 2; iRefList ++)
      {
        if( iCurrDir & ( 1 << iRefList ) )
        {       
          if(!(cCurMvField[iRefList] == cNeigMvField[iRefList]))
          {
            return true;
          }
        }
      }
      return false;
    }
  }
  else
  {
    return false;
  }
}
#endif

#if QC_LARGE_CTU
Void TComDataCU::getMaxMinCUDepth( UChar & rucMinDepth , UChar & rucMaxDepth , UInt uiAbsPartIdx )
{
  const UChar ucMaxCUDepth = ( UChar )( g_uiMaxCUDepth - g_uiAddCUDepth );
  rucMinDepth = ucMaxCUDepth;
  rucMaxDepth = 0;

  TComDataCU * pNeighbor;
  UInt uiNeighbor; 
  
  // left
  pNeighbor = getPULeft( uiNeighbor , uiAbsPartIdx );
  if( pNeighbor != NULL )
  {
    rucMinDepth = min( rucMinDepth , pNeighbor->getDepth( uiNeighbor ) );
    rucMaxDepth = max( rucMaxDepth , pNeighbor->getDepth( uiNeighbor ) );
  }
  else
  {
    rucMinDepth = 0;
    rucMaxDepth = ucMaxCUDepth;
  }

  // bottom left
  pNeighbor = getPUBelowLeft( uiNeighbor , uiAbsPartIdx );
  if( pNeighbor != NULL )
  {
    rucMinDepth = min( rucMinDepth , pNeighbor->getDepth( uiNeighbor ) );
    rucMaxDepth = max( rucMaxDepth , pNeighbor->getDepth( uiNeighbor ) );
  }
  else
  {
    rucMinDepth = 0;
    rucMaxDepth = ucMaxCUDepth;
  }

  // top
  pNeighbor = getPUAbove( uiNeighbor , uiAbsPartIdx );
  if( pNeighbor != NULL )
  {
    rucMinDepth = min( rucMinDepth , pNeighbor->getDepth( uiNeighbor ) );
    rucMaxDepth = max( rucMaxDepth , pNeighbor->getDepth( uiNeighbor ) );
  }
  else
  {
    rucMinDepth = 0;
    rucMaxDepth = ucMaxCUDepth;
  }

  // top right
  pNeighbor = getPUAboveRight( uiNeighbor , uiAbsPartIdx );
  if( pNeighbor != NULL )
  {
    rucMinDepth = min( rucMinDepth , pNeighbor->getDepth( uiNeighbor ) );
    rucMaxDepth = max( rucMaxDepth , pNeighbor->getDepth( uiNeighbor ) );
  }
  else
  {
    rucMinDepth = 0;
    rucMaxDepth = ucMaxCUDepth;
  }

  Int nDepthInc = 1;
  rucMinDepth = rucMinDepth >= nDepthInc ? rucMinDepth - nDepthInc : 0;
  rucMaxDepth = min( ( UChar )ucMaxCUDepth , ( UChar )( rucMaxDepth + 1 ) );
}
#endif

#if HM14_CLEAN_UP
Void TComDataCU::resetCoeff( UInt uiAbsPartIdx )
{
  static const UInt uiMinCoeffSize = getPic()->getMinCUWidth()*getPic()->getMinCUHeight();
  UInt uiLumaOffset   = uiMinCoeffSize*uiAbsPartIdx;
  UInt uiChromaOffset = uiLumaOffset>>2;

  Int nSize = sizeof( TCoeff ) * getWidth( uiAbsPartIdx ) * getHeight( uiAbsPartIdx );
  memset( getCoeffY() + uiLumaOffset , 0 , nSize );
  nSize >>= 2;
  memset( getCoeffCb() + uiChromaOffset , 0 , nSize );
  memset( getCoeffCr() + uiChromaOffset , 0 , nSize );
}

Void TComDataCU::resetPCMSample( UInt uiAbsPartIdx )
{
  static const UInt uiMinCoeffSize = getPic()->getMinCUWidth()*getPic()->getMinCUHeight();
  UInt uiLumaOffset   = uiMinCoeffSize*uiAbsPartIdx;
  UInt uiChromaOffset = uiLumaOffset>>2;

  Int nSize = sizeof( Pel ) * getWidth( uiAbsPartIdx ) * getHeight( uiAbsPartIdx );
  memset( getPCMSampleY() + uiLumaOffset , 0 , nSize );
  nSize >>= 2;
  memset( getPCMSampleCb() + uiChromaOffset , 0 , nSize );
  memset( getPCMSampleCr() + uiChromaOffset , 0 , nSize );
}
#endif


//! \}
