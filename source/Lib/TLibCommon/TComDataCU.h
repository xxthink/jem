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

/** \file     TComDataCU.h
    \brief    CU data structure (header)
    \todo     not all entities are documented
*/

#ifndef _TCOMDATACU_
#define _TCOMDATACU_

#include <assert.h>

// Include files
#include "CommonDef.h"
#include "TComMotionInfo.h"
#include "TComSlice.h"
#include "TComRdCost.h"
#include "TComPattern.h"

#include <algorithm>
#include <vector>

#if QC_FRUC_MERGE
class TComPrediction;
#endif

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU data structure class
class TComDataCU
{
private:
  
  // -------------------------------------------------------------------------------------------------------------------
  // class pointers
  // -------------------------------------------------------------------------------------------------------------------
  
  TComPic*      m_pcPic;              ///< picture class pointer
  TComSlice*    m_pcSlice;            ///< slice header pointer
  TComPattern*  m_pcPattern;          ///< neighbour access class pointer
  
  // -------------------------------------------------------------------------------------------------------------------
  // CU description
  // -------------------------------------------------------------------------------------------------------------------
  
  UInt          m_uiCUAddr;           ///< CU address in a slice
  UInt          m_uiAbsIdxInLCU;      ///< absolute address in a CU. It's Z scan order
  UInt          m_uiCUPelX;           ///< CU position in a pixel (X)
  UInt          m_uiCUPelY;           ///< CU position in a pixel (Y)
  UInt          m_uiNumPartition;     ///< total number of minimum partitions in a CU
#if QC_LARGE_CTU 
  UShort*       m_puhWidth;           ///< array of widths
  UShort*       m_puhHeight;          ///< array of heights
#else
  UChar*        m_puhWidth;           ///< array of widths
  UChar*        m_puhHeight;          ///< array of heights
#endif
  UChar*        m_puhDepth;           ///< array of depths
  Int           m_unitSize;           ///< size of a "minimum partition"
  
  // -------------------------------------------------------------------------------------------------------------------
  // CU data
  // -------------------------------------------------------------------------------------------------------------------
  Bool*         m_skipFlag;           ///< array of skip flags
#if ROT_TR
  Char*         m_ROTIdx;             ///< array of ROTIdxs
#endif
#if CU_LEVEL_MPI
  Char*         m_MPIIdx;           ///< array of MPIIdxs
#endif
#if QC_IMV
  Bool*         m_iMVFlag;           ///< array of integer MV flags
  Char*         m_piMVCandNum;        ///< encoder only array
#endif
#if QC_OBMC
  Bool*         m_OBMCFlag;           ///< array of OBMC flags
#endif
  Char*         m_pePartSize;         ///< array of partition sizes
  Char*         m_pePredMode;         ///< array of prediction modes
  Bool*         m_CUTransquantBypass;   ///< array of cu_transquant_bypass flags
  Char*         m_phQP;               ///< array of QP values
#if QC_EMT
  UChar*        m_puhEmtTuIdx;        ///< array of TU-level transform indexes
  UChar*        m_puhEmtCuFlag;       ///< array of CU-level flags enabling EMT
#endif
  UChar*        m_puhTrIdx;           ///< array of transform indices
  UChar*        m_puhTransformSkip[3];///< array of transform skipping flags
#if KLT_COMMON
  UChar*        m_puhKLTFlag[3];      ///< array of KLT flags
#endif
  UChar*        m_puhCbf[3];          ///< array of coded block flags (CBF)
  TComCUMvField m_acCUMvField[2];     ///< array of motion vectors
#if QC_FRUC_MERGE
  TComCUMvField m_acFRUCUniLateralMVField[2];
#endif
  TCoeff*       m_pcTrCoeffY;         ///< transformed coefficient buffer (Y)
  TCoeff*       m_pcTrCoeffCb;        ///< transformed coefficient buffer (Cb)
  TCoeff*       m_pcTrCoeffCr;        ///< transformed coefficient buffer (Cr)
#if ADAPTIVE_QP_SELECTION
  Int*          m_pcArlCoeffY;        ///< ARL coefficient buffer (Y)
  Int*          m_pcArlCoeffCb;       ///< ARL coefficient buffer (Cb)
  Int*          m_pcArlCoeffCr;       ///< ARL coefficient buffer (Cr)
  Bool          m_ArlCoeffIsAliasedAllocation; ///< ARL coefficient buffer is an alias of the global buffer and must not be free()'d

  static Int*   m_pcGlbArlCoeffY;     ///< ARL coefficient buffer (Y)
  static Int*   m_pcGlbArlCoeffCb;    ///< ARL coefficient buffer (Cb)
  static Int*   m_pcGlbArlCoeffCr;    ///< ARL coefficient buffer (Cr)
#endif
  
  Pel*          m_pcIPCMSampleY;      ///< PCM sample buffer (Y)
  Pel*          m_pcIPCMSampleCb;     ///< PCM sample buffer (Cb)
  Pel*          m_pcIPCMSampleCr;     ///< PCM sample buffer (Cr)

  // -------------------------------------------------------------------------------------------------------------------
  // neighbour access variables
  // -------------------------------------------------------------------------------------------------------------------
  
  TComDataCU*   m_pcCUAboveLeft;      ///< pointer of above-left CU
  TComDataCU*   m_pcCUAboveRight;     ///< pointer of above-right CU
  TComDataCU*   m_pcCUAbove;          ///< pointer of above CU
  TComDataCU*   m_pcCULeft;           ///< pointer of left CU
  TComDataCU*   m_apcCUColocated[2];  ///< pointer of temporally colocated CU's for both directions
  TComMvField   m_cMvFieldA;          ///< motion vector of position A
  TComMvField   m_cMvFieldB;          ///< motion vector of position B
  TComMvField   m_cMvFieldC;          ///< motion vector of position C
  TComMv        m_cMvPred;            ///< motion vector predictor
  
  // -------------------------------------------------------------------------------------------------------------------
  // coding tool information
  // -------------------------------------------------------------------------------------------------------------------
  
  Bool*         m_pbMergeFlag;        ///< array of merge flags
  UChar*        m_puhMergeIndex;      ///< array of merge candidate indices
#if QC_FRUC_MERGE
  UChar*        m_puhFRUCMgrMode;
#endif
#if AMP_MRG
  Bool          m_bIsMergeAMP;
#endif

#if QC_SUB_PU_TMVP
  UChar*        m_peMergeType;       ///< array of merge Types flags to indicate whehter a block uses sub-PU TMVP
#endif
#if QC_IC
  Bool*         m_pbICFlag;           ///< array of IC flags
#endif
  UChar*        m_puhLumaIntraDir;    ///< array of intra directions (luma)
  UChar*        m_puhChromaIntraDir;  ///< array of intra directions (chroma)
  UChar*        m_puhInterDir;        ///< array of inter directions
  Char*         m_apiMVPIdx[2];       ///< array of motion vector predictor candidates
  Char*         m_apiMVPNum[2];       ///< array of number of possible motion vectors predictors
  Bool*         m_pbIPCMFlag;         ///< array of intra_pcm flags

  // -------------------------------------------------------------------------------------------------------------------
  // misc. variables
  // -------------------------------------------------------------------------------------------------------------------
  
  Bool          m_bDecSubCu;          ///< indicates decoder-mode
  Double        m_dTotalCost;         ///< sum of partition RD costs
  UInt          m_uiTotalDistortion;  ///< sum of partition distortion
  UInt          m_uiTotalBits;        ///< sum of partition bits
  UInt          m_uiTotalBins;       ///< sum of partition bins
  UInt*         m_sliceStartCU;    ///< Start CU address of current slice
  UInt*         m_sliceSegmentStartCU; ///< Start CU address of current slice
  Char          m_codedQP;
#if ALF_HM3_QC_REFACTOR
  UInt*         m_puiAlfCtrlFlag;     ///< array of ALF flags
  UInt*         m_puiTmpAlfCtrlFlag;  ///< temporal array of ALF flags
#endif
protected:
  
  /// add possible motion vector predictor candidates
  Bool          xAddMVPCand           ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );
  Bool          xAddMVPCandOrder      ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );

  Void          deriveRightBottomIdx        ( UInt uiPartIdx, UInt& ruiPartIdxRB );
#if QC_IC
  Bool          xGetColMVP( RefPicList eRefPicList, Int uiCUAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx, Bool* bICFlag = NULL );
#else
  Bool          xGetColMVP( RefPicList eRefPicList, Int uiCUAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx );
#endif
  /// compute required bits to encode MVD (used in AMVP)
  UInt          xGetMvdBits           ( TComMv cMvd );
  UInt          xGetComponentBits     ( Int iVal );
  
  /// compute scaling factor from POC difference
  Int           xGetDistScaleFactor   ( Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC );
  
  Void xDeriveCenterIdx( UInt uiPartIdx, UInt& ruiPartIdxCenter );

public:
  TComDataCU();
  virtual ~TComDataCU();
  
  // -------------------------------------------------------------------------------------------------------------------
  // create / destroy / initialize / copy
  // -------------------------------------------------------------------------------------------------------------------
  
  Void          create                ( UInt uiNumPartition, UInt uiWidth, UInt uiHeight, Bool bDecSubCu, Int unitSize
#if ADAPTIVE_QP_SELECTION
    , Bool bGlobalRMARLBuffer = false
#endif  
    );
  Void          destroy               ();
  
  Void          initCU                ( TComPic* pcPic, UInt uiCUAddr 
#if HM14_CLEAN_UP
  , Bool bDec = false
#endif
);
  Void          initEstData           ( UInt uiDepth, Int qp, Bool bTransquantBypass );
  Void          initSubCU             ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth, Int qp );
  Void          setOutsideCUPart      ( UInt uiAbsPartIdx, UInt uiDepth );

  Void          copySubCU             ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth );
  Void          copyInterPredInfoFrom ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList );
  Void          copyPartFrom          ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth );
#if INTER_KLT 
  Void          copySameSizeCUFrom    ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth );
#endif
  Void          copyToPic             ( UChar uiDepth );
  Void          copyToPic             ( UChar uiDepth, UInt uiPartIdx, UInt uiPartDepth );
  
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for CU description
  // -------------------------------------------------------------------------------------------------------------------
  
  TComPic*      getPic                ()                        { return m_pcPic;           }
  TComSlice*    getSlice              ()                        { return m_pcSlice;         }
  UInt&         getAddr               ()                        { return m_uiCUAddr;        }
  UInt&         getZorderIdxInCU      ()                        { return m_uiAbsIdxInLCU; }
  UInt          getSCUAddr            ();
  UInt          getCUPelX             ()                        { return m_uiCUPelX;        }
  UInt          getCUPelY             ()                        { return m_uiCUPelY;        }
  TComPattern*  getPattern            ()                        { return m_pcPattern;       }
  
  UChar*        getDepth              ()                        { return m_puhDepth;        }
  UChar         getDepth              ( UInt uiIdx )            { return m_puhDepth[uiIdx]; }
  Void          setDepth              ( UInt uiIdx, UChar  uh ) { m_puhDepth[uiIdx] = uh;   }
  
  Void          setDepthSubParts      ( UInt uiDepth, UInt uiAbsPartIdx );
  
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for CU data
  // -------------------------------------------------------------------------------------------------------------------
  
  Char*         getPartitionSize      ()                        { return m_pePartSize;        }
  PartSize      getPartitionSize      ( UInt uiIdx )            { return static_cast<PartSize>( m_pePartSize[uiIdx] ); }
  Void          setPartitionSize      ( UInt uiIdx, PartSize uh){ m_pePartSize[uiIdx] = uh;   }
  Void          setPartSizeSubParts   ( PartSize eMode, UInt uiAbsPartIdx, UInt uiDepth );
  Void          setCUTransquantBypassSubParts( Bool flag, UInt uiAbsPartIdx, UInt uiDepth );
  
  Bool*        getSkipFlag            ()                        { return m_skipFlag;          }
  Bool         getSkipFlag            (UInt idx)                { return m_skipFlag[idx];     }
  Void         setSkipFlag           ( UInt idx, Bool skip)     { m_skipFlag[idx] = skip;   }
  Void         setSkipFlagSubParts   ( Bool skip, UInt absPartIdx, UInt depth );
#if ROT_TR
  Char*        getROTIdx            ()                        { return m_ROTIdx;          }
  Char         getROTIdx            (UInt idx)                { return m_ROTIdx[idx];     }
  Void         setROTIdx           ( UInt idx, Char ROTIdx)     { m_ROTIdx[idx] = ROTIdx;   }
  Void         setROTIdxSubParts   ( Char ROTIdx, UInt absPartIdx, UInt depth );
#endif
#if CU_LEVEL_MPI
  Char*        getMPIIdx            ()                        { return m_MPIIdx;          }
  Char         getMPIIdx            (UInt idx)                { return m_MPIIdx[idx];     }
  Void         setMPIIdx           ( UInt idx, Char MPIIdx)     { m_MPIIdx[idx] = MPIIdx;   }
  Void         setMPIIdxSubParts   ( Char MPIIdx, UInt absPartIdx, UInt depth );
#endif
#if QC_IMV
  Bool*         getiMVFlag            ()                        { return m_iMVFlag;          }
  Bool          getiMVFlag            (UInt idx)                { return m_iMVFlag[idx];     }
  Void          setiMVFlag            ( UInt idx, Bool iMV)     { m_iMVFlag[idx] = iMV;      }
  Void          setiMVFlagSubParts    ( Bool iMV, UInt absPartIdx, UInt depth );
  Char*         getiMVCandNum            ()                        { return m_piMVCandNum;          }
  Char          getiMVCandNum            (UInt idx)                { return m_piMVCandNum[idx];     }
  Void          setiMVCandNum           ( UInt idx, Char ciMVCandNum)     { m_piMVCandNum[idx] = ciMVCandNum;   }
  Void          setiMVCandNumSubParts   ( Char ciMVCandNum, UInt absPartIdx, UInt depth );
#endif
#if QC_OBMC
  Bool*        getOBMCFlag            ()                        { return m_OBMCFlag;          }
  Bool         getOBMCFlag            (UInt idx)                { return m_OBMCFlag[idx];     }
  Void         setOBMCFlag            ( UInt idx, Bool OBMC )   { m_OBMCFlag[idx] = OBMC;     }
  Void         setOBMCFlagSubParts    ( Bool OBMC, UInt absPartIdx, UInt depth );
  Bool         isOBMCFlagCoded        ( UInt uiAbsPartIdx );
#endif

  Char*         getPredictionMode     ()                        { return m_pePredMode;        }
  PredMode      getPredictionMode     ( UInt uiIdx )            { return static_cast<PredMode>( m_pePredMode[uiIdx] ); }
  Bool*         getCUTransquantBypass ()                        { return m_CUTransquantBypass;        }
  Bool          getCUTransquantBypass( UInt uiIdx )             { return m_CUTransquantBypass[uiIdx]; }
  Void          setPredictionMode     ( UInt uiIdx, PredMode uh){ m_pePredMode[uiIdx] = uh;   }
  Void          setPredModeSubParts   ( PredMode eMode, UInt uiAbsPartIdx, UInt uiDepth );
  
#if QC_LARGE_CTU
  UShort*       getWidth              ()                        { return m_puhWidth;          }
  UShort        getWidth              ( UInt uiIdx )            { return m_puhWidth[uiIdx];   }
  Void          setWidth              ( UInt uiIdx, UShort uh ) { m_puhWidth[uiIdx] = uh;     }

  UShort*       getHeight             ()                        { return m_puhHeight;         }
  UShort        getHeight             ( UInt uiIdx )            { return m_puhHeight[uiIdx];  }
  Void          setHeight             ( UInt uiIdx, UShort uh ) { m_puhHeight[uiIdx] = uh;    }
#else
  UChar*        getWidth              ()                        { return m_puhWidth;          }
  UChar         getWidth              ( UInt uiIdx )            { return m_puhWidth[uiIdx];   }
  Void          setWidth              ( UInt uiIdx, UChar  uh ) { m_puhWidth[uiIdx] = uh;     }
  
  UChar*        getHeight             ()                        { return m_puhHeight;         }
  UChar         getHeight             ( UInt uiIdx )            { return m_puhHeight[uiIdx];  }
  Void          setHeight             ( UInt uiIdx, UChar  uh ) { m_puhHeight[uiIdx] = uh;    }
#endif
  
  Void          setSizeSubParts       ( UInt uiWidth, UInt uiHeight, UInt uiAbsPartIdx, UInt uiDepth );
  
  Char*         getQP                 ()                        { return m_phQP;              }
  Char          getQP                 ( UInt uiIdx )            { return m_phQP[uiIdx];       }
  Void          setQP                 ( UInt uiIdx, Char value ){ m_phQP[uiIdx] =  value;     }
  Void          setQPSubParts         ( Int qp,   UInt uiAbsPartIdx, UInt uiDepth );
  Int           getLastValidPartIdx   ( Int iAbsPartIdx );
  Char          getLastCodedQP        ( UInt uiAbsPartIdx );
  Void          setQPSubCUs           ( Int qp, TComDataCU* pcCU, UInt absPartIdx, UInt depth, Bool &foundNonZeroCbf );
  Void          setCodedQP            ( Char qp )               { m_codedQP = qp;             }
  Char          getCodedQP            ()                        { return m_codedQP;           }

  Bool          isLosslessCoded(UInt absPartIdx);
  
#if QC_EMT
  UChar*        getEmtTuIdx           ()                        { return m_puhEmtTuIdx;          }
  UChar         getEmtTuIdx           ( UInt uiIdx )            { return m_puhEmtTuIdx[uiIdx];   }
  Void          setEmtTuIdxSubParts     ( UInt uiTrMode, UInt uiAbsPartIdx, UInt uiDepth );
  UChar*        getEmtCuFlag          ()                        { return m_puhEmtCuFlag;          }
  UChar         getEmtCuFlag          ( UInt uiIdx )            { return m_puhEmtCuFlag[uiIdx];   }
  Void          setEmtCuFlagSubParts  ( UInt uiTuOptTrFlag, UInt uiAbsPartIdx, UInt uiDepth );
#endif  

  UChar*        getTransformIdx       ()                        { return m_puhTrIdx;          }
  UChar         getTransformIdx       ( UInt uiIdx )            { return m_puhTrIdx[uiIdx];   }
  Void          setTrIdxSubParts      ( UInt uiTrIdx, UInt uiAbsPartIdx, UInt uiDepth );

  UChar*        getTransformSkip      ( TextType eType)    { return m_puhTransformSkip[g_aucConvertTxtTypeToIdx[eType]];}
  UChar         getTransformSkip      ( UInt uiIdx,TextType eType)    { return m_puhTransformSkip[g_aucConvertTxtTypeToIdx[eType]][uiIdx];}
  Void          setTransformSkipSubParts  ( UInt useTransformSkip, TextType eType, UInt uiAbsPartIdx, UInt uiDepth); 
  Void          setTransformSkipSubParts  ( UInt useTransformSkipY, UInt useTransformSkipU, UInt useTransformSkipV, UInt uiAbsPartIdx, UInt uiDepth );

#if KLT_COMMON
  UChar*        getKLTFlag(TextType eType)    { return m_puhKLTFlag[g_aucConvertTxtTypeToIdx[eType]]; }
  UChar         getKLTFlag(UInt uiIdx, TextType eType)    { return m_puhKLTFlag[g_aucConvertTxtTypeToIdx[eType]][uiIdx]; }
  Void          setKLTFlagSubParts(UInt useKLT, TextType eType, UInt uiAbsPartIdx, UInt uiDepth);
  Void          setKLTFlagSubParts(UInt useKLTY, UInt useKLTU, UInt useKLTV, UInt uiAbsPartIdx, UInt uiDepth);
#endif

  UInt          getQuadtreeTULog2MinSizeInCU( UInt absPartIdx );
  
  TComCUMvField* getCUMvField         ( RefPicList e )          { return  &m_acCUMvField[e];  }
#if QC_FRUC_MERGE
  TComCUMvField* getFRUCUniLateralMVField        ( RefPicList e )          { return  &m_acFRUCUniLateralMVField[e]; }
  TComMv        scaleMV( const TComMv & rColMV , Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC );
  Bool          isSameMVField( RefPicList eListA , TComMvField & rMVFieldA , RefPicList eListB , TComMvField & rMVFieldB );
  Bool          getMvPair( RefPicList eCurRefPicList , const TComMvField & rCurMvField , TComMvField & rMvPair );
  Bool          getBlockBelowRight( UInt uiAbsPartIdx, Int nCurBlkWidth , Int nCurBlkHeight , UInt & rCUAddr , UInt & rBRAbsPartIdx );
#endif

  TCoeff*&      getCoeffY             ()                        { return m_pcTrCoeffY;        }
  TCoeff*&      getCoeffCb            ()                        { return m_pcTrCoeffCb;       }
  TCoeff*&      getCoeffCr            ()                        { return m_pcTrCoeffCr;       }
#if ADAPTIVE_QP_SELECTION
  Int*&         getArlCoeffY          ()                        { return m_pcArlCoeffY;       }
  Int*&         getArlCoeffCb         ()                        { return m_pcArlCoeffCb;      }
  Int*&         getArlCoeffCr         ()                        { return m_pcArlCoeffCr;      }
#endif
  
  Pel*&         getPCMSampleY         ()                        { return m_pcIPCMSampleY;     }
  Pel*&         getPCMSampleCb        ()                        { return m_pcIPCMSampleCb;    }
  Pel*&         getPCMSampleCr        ()                        { return m_pcIPCMSampleCr;    }

#if HM14_CLEAN_UP
  Void          resetCoeff( UInt uiAbsPartIdx );
  Void          resetPCMSample( UInt uiAbsPartIdx );
#endif

  UChar         getCbf    ( UInt uiIdx, TextType eType )                  { return m_puhCbf[g_aucConvertTxtTypeToIdx[eType]][uiIdx];  }
  UChar*        getCbf    ( TextType eType )                              { return m_puhCbf[g_aucConvertTxtTypeToIdx[eType]];         }
  UChar         getCbf    ( UInt uiIdx, TextType eType, UInt uiTrDepth )  { return ( ( getCbf( uiIdx, eType ) >> uiTrDepth ) & 0x1 ); }
  Void          setCbf    ( UInt uiIdx, TextType eType, UChar uh )        { m_puhCbf[g_aucConvertTxtTypeToIdx[eType]][uiIdx] = uh;    }
  Void          clearCbf  ( UInt uiIdx, TextType eType, UInt uiNumParts );
  UChar         getQtRootCbf          ( UInt uiIdx )                      { return getCbf( uiIdx, TEXT_LUMA, 0 ) || getCbf( uiIdx, TEXT_CHROMA_U, 0 ) || getCbf( uiIdx, TEXT_CHROMA_V, 0 ); }
  
  Void          setCbfSubParts        ( UInt uiCbfY, UInt uiCbfU, UInt uiCbfV, UInt uiAbsPartIdx, UInt uiDepth          );
  Void          setCbfSubParts        ( UInt uiCbf, TextType eTType, UInt uiAbsPartIdx, UInt uiDepth                    );
  Void          setCbfSubParts        ( UInt uiCbf, TextType eTType, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth    );
  
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for coding tool information
  // -------------------------------------------------------------------------------------------------------------------
  
  Bool*         getMergeFlag          ()                        { return m_pbMergeFlag;               }
  Bool          getMergeFlag          ( UInt uiIdx )            { return m_pbMergeFlag[uiIdx];        }
  Void          setMergeFlag          ( UInt uiIdx, Bool b )    { m_pbMergeFlag[uiIdx] = b;           }
  Void          setMergeFlagSubParts  ( Bool bMergeFlag, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

  UChar*        getMergeIndex         ()                        { return m_puhMergeIndex;                         }
  UChar         getMergeIndex         ( UInt uiIdx )            { return m_puhMergeIndex[uiIdx];                  }
  Void          setMergeIndex         ( UInt uiIdx, UInt uiMergeIndex ) { m_puhMergeIndex[uiIdx] = uiMergeIndex;  }
  Void          setMergeIndexSubParts ( UInt uiMergeIndex, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
#if QC_SUB_PU_TMVP
  Void          setMergeTypeSubParts ( UChar eMergeType, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
#endif
#if QC_FRUC_MERGE
  UChar*        getFRUCMgrMode          ()                        { return m_puhFRUCMgrMode;               }
  UChar         getFRUCMgrMode          ( UInt uiIdx )            { return m_puhFRUCMgrMode[uiIdx];        }
  Void          setFRUCMgrMode          ( UInt uiIdx, UChar b )   { m_puhFRUCMgrMode[uiIdx] = b;           }
  Void          setFRUCMgrModeSubParts  ( UChar uhFRUCMgrMode, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
#endif
  template <typename T>
  Void          setSubPart            ( T bParameter, T* pbBaseLCU, UInt uiCUAddr, UInt uiCUDepth, UInt uiPUIdx );

#if AMP_MRG
  Void          setMergeAMP( Bool b )      { m_bIsMergeAMP = b; }
  Bool          getMergeAMP( )             { return m_bIsMergeAMP; }
#endif

  UChar*        getLumaIntraDir       ()                        { return m_puhLumaIntraDir;           }
  UChar         getLumaIntraDir       ( UInt uiIdx )            { return m_puhLumaIntraDir[uiIdx];    }
  Void          setLumaIntraDir       ( UInt uiIdx, UChar  uh ) { m_puhLumaIntraDir[uiIdx] = uh;      }
  Void          setLumaIntraDirSubParts( UInt uiDir,  UInt uiAbsPartIdx, UInt uiDepth );
  
  UChar*        getChromaIntraDir     ()                        { return m_puhChromaIntraDir;         }
  UChar         getChromaIntraDir     ( UInt uiIdx )            { return m_puhChromaIntraDir[uiIdx];  }
  Void          setChromaIntraDir     ( UInt uiIdx, UChar  uh ) { m_puhChromaIntraDir[uiIdx] = uh;    }
  Void          setChromIntraDirSubParts( UInt uiDir,  UInt uiAbsPartIdx, UInt uiDepth );
  
  UChar*        getInterDir           ()                        { return m_puhInterDir;               }
  UChar         getInterDir           ( UInt uiIdx )            { return m_puhInterDir[uiIdx];        }
  Void          setInterDir           ( UInt uiIdx, UChar  uh ) { m_puhInterDir[uiIdx] = uh;          }
  Void          setInterDirSubParts   ( UInt uiDir,  UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Bool*         getIPCMFlag           ()                        { return m_pbIPCMFlag;               }
  Bool          getIPCMFlag           (UInt uiIdx )             { return m_pbIPCMFlag[uiIdx];        }
  Void          setIPCMFlag           (UInt uiIdx, Bool b )     { m_pbIPCMFlag[uiIdx] = b;           }
  Void          setIPCMFlagSubParts   (Bool bIpcmFlag, UInt uiAbsPartIdx, UInt uiDepth);

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for accessing partition information
  // -------------------------------------------------------------------------------------------------------------------
  Void          getPartIndexAndSize   ( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight );
#if QC_SUB_PU_TMVP
  Void          getPartIndexAndSize( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight, UInt uiAbsPartIdx, Bool bLCU) ;
#endif
#if QC_IC
  Bool*         getICFlag          ()                        { return m_pbICFlag;               }
  Bool          getICFlag          ( UInt uiIdx )            { return m_pbICFlag[uiIdx];        }
  Void          setICFlag          ( UInt uiIdx, Bool  uh )  { m_pbICFlag[uiIdx] = uh;          }
  Void          setICFlagSubParts  ( Bool bICFlag,  UInt uiAbsPartIdx, UInt uiDepth );
  Bool          isICFlagCoded      ( UInt uiAbsPartIdx );
#endif
  UChar         getNumPartitions       ();
  Bool          isFirstAbsZorderIdxInDepth (UInt uiAbsPartIdx, UInt uiDepth);
  
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for motion vector
  // -------------------------------------------------------------------------------------------------------------------
  
  Void          getMvField            ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, TComMvField& rcMvField );
  
  Void          fillMvpCand           ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AMVPInfo* pInfo 
#if QC_FRUC_MERGE
    , TComPrediction * pPred = NULL
#endif
    );
  Bool          isDiffMER             ( Int xN, Int yN, Int xP, Int yP);
  Void          getPartPosition       ( UInt partIdx, Int& xP, Int& yP, Int& nPSW, Int& nPSH);
  Void          setMVPIdx             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPIdx)  { m_apiMVPIdx[eRefPicList][uiIdx] = iMVPIdx;  }
  Int           getMVPIdx             ( RefPicList eRefPicList, UInt uiIdx)               { return m_apiMVPIdx[eRefPicList][uiIdx];     }
  Char*         getMVPIdx             ( RefPicList eRefPicList )                          { return m_apiMVPIdx[eRefPicList];            }

  Void          setMVPNum             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPNum ) { m_apiMVPNum[eRefPicList][uiIdx] = iMVPNum;  }
  Int           getMVPNum             ( RefPicList eRefPicList, UInt uiIdx )              { return m_apiMVPNum[eRefPicList][uiIdx];     }
  Char*         getMVPNum             ( RefPicList eRefPicList )                          { return m_apiMVPNum[eRefPicList];            }
  
  Void          setMVPIdxSubParts     ( Int iMVPIdx, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Void          setMVPNumSubParts     ( Int iMVPNum, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  
  Void          clipMv                ( TComMv&     rcMv     );
#if INTRA_KLT
  Void      clipMvIntraConstraint ( Int regionId, Int &iHorMin, Int &iHorMax, Int &iVerMin, Int &iVerMax, Int iRange, UInt uiTemplateSize, UInt uiBlkSize, Int iY, Int iCurrX, Int offsetLCUY, Int offsetLCUX);
#endif
  Void          getMvPredLeft         ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldA.getMv(); }
  Void          getMvPredAbove        ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldB.getMv(); }
  Void          getMvPredAboveRight   ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldC.getMv(); }
  
  Void          compressMV            ();
  
  // -------------------------------------------------------------------------------------------------------------------
  // utility functions for neighbouring information
  // -------------------------------------------------------------------------------------------------------------------
  
  TComDataCU*   getCULeft                   () { return m_pcCULeft;       }
  TComDataCU*   getCUAbove                  () { return m_pcCUAbove;      }
  TComDataCU*   getCUAboveLeft              () { return m_pcCUAboveLeft;  }
  TComDataCU*   getCUAboveRight             () { return m_pcCUAboveRight; }
  TComDataCU*   getCUColocated              ( RefPicList eRefPicList ) { return m_apcCUColocated[eRefPicList]; }


  TComDataCU*   getPULeft                   ( UInt&  uiLPartUnitIdx, 
                                              UInt uiCurrPartUnitIdx, 
                                              Bool bEnforceSliceRestriction=true, 
                                              Bool bEnforceTileRestriction=true );
  TComDataCU*   getPUAbove                  ( UInt&  uiAPartUnitIdx,
                                              UInt uiCurrPartUnitIdx, 
                                              Bool bEnforceSliceRestriction=true, 
                                              Bool planarAtLCUBoundary = false,
                                              Bool bEnforceTileRestriction=true );
  TComDataCU*   getPUAboveLeft              ( UInt&  uiALPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction=true );
  TComDataCU*   getPUAboveRight             ( UInt&  uiARPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction=true );
  TComDataCU*   getPUBelowLeft              ( UInt&  uiBLPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction=true );

#if QC_USE_65ANG_MODES
  TComDataCU*   getPULeftOffset             ( UInt& uiPartUnitIdx, 
                                              UInt uiCurrPartUnitIdx, 
                                              UInt uiPartOffset=0,
                                              Bool bEnforceSliceRestriction=true, 
                                              Bool bEnforceTileRestriction=true );
  TComDataCU*   getPUAboveOffset            ( UInt& uiPartUnitIdx, 
                                              UInt uiCurrPartUnitIdx, 
                                              UInt uiPartOffset=0,
                                              Bool bEnforceSliceRestriction=true, 
                                              Bool planarAtLCUBoundary = true,
                                              Bool bEnforceTileRestriction=true );
#endif

  TComDataCU*   getQpMinCuLeft              ( UInt&  uiLPartUnitIdx , UInt uiCurrAbsIdxInLCU );
  TComDataCU*   getQpMinCuAbove             ( UInt&  aPartUnitIdx , UInt currAbsIdxInLCU );
  Char          getRefQP                    ( UInt   uiCurrAbsIdxInLCU                       );

  TComDataCU*   getPUAboveRightAdi          ( UInt&  uiARPartUnitIdx, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset = 1, Bool bEnforceSliceRestriction=true );
  TComDataCU*   getPUBelowLeftAdi           ( UInt&  uiBLPartUnitIdx, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset = 1, Bool bEnforceSliceRestriction=true );
  
  Void          deriveLeftRightTopIdx       ( UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT );
  Void          deriveLeftBottomIdx         ( UInt uiPartIdx, UInt& ruiPartIdxLB );
  
  Void          deriveLeftRightTopIdxAdi    ( UInt& ruiPartIdxLT, UInt& ruiPartIdxRT, UInt uiPartOffset, UInt uiPartDepth );
  Void          deriveLeftBottomIdxAdi      ( UInt& ruiPartIdxLB, UInt  uiPartOffset, UInt uiPartDepth );
  
  Bool          hasEqualMotion              ( UInt uiAbsPartIdx, TComDataCU* pcCandCU, UInt uiCandAbsPartIdx );
#if QC_IMV
#if QC_MV_STORE_PRECISION_BIT
  Void          xRoundMV( TComMv & rMV ) 
  { 
    rMV += TComMv( 1 << ( QC_MV_STORE_PRECISION_BIT - 1 ) , 1 << ( QC_MV_STORE_PRECISION_BIT - 1 ) ); 
    rMV >>= QC_MV_STORE_PRECISION_BIT; 
    rMV <<= QC_MV_STORE_PRECISION_BIT; 
  }
#else
  Void          xRoundMV( TComMv & rMV ) { rMV += TComMv( 2 , 2 ); rMV >>= 2; rMV <<= 2; }
#endif
  Char          getMaxNeighboriMVCandNum( UInt uiAbsPartIdx );
#if QC_FRUC_MERGE
  Bool          resetMVDandMV2Int( UInt uiAbsPartIdx , UInt uiPartIdx , Bool bResetMV , TComPrediction * pPred );
  Bool          resetMVDandMV2Int( Bool bResetMV , TComPrediction * pPred );
#else
  Void          resetMVDandMV2Int( UInt uiAbsPartIdx , UInt uiPartIdx , Bool bResetMV );
  Void          resetMVDandMV2Int( Bool bResetMV );
#endif
#endif

#if QC_SUB_PU_TMVP_EXT
  Void getNeighboringMvField(TComDataCU *pcCU, UInt uiPartIdx, TComMvField *cMvField,UChar *pucInterDir);
  Void generateMvField(TComMvField *cMvField,UChar* pucInterDir, UInt uiMvNum,TComMvField* cMvFieldMedian,UChar &ucInterDirMedian);  
  Bool getInterMergeSubPURecursiveCandidate( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMvFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand
  , UChar*          peMergeTypeNeighbors  , TComMvField*    pcMvFieldSP[2] , UChar*          puhInterDirSP[2] , Int iCount );

#endif

#if QC_SUB_PU_TMVP
  Void          get1stTvFromSpatialNeighbor ( UInt uiAbsPartIdx, UInt uiPUIdx, Bool &bTvAva, Int &iPOC, TComMv &rcMv);
#endif


#if QC_SUB_PU_TMVP_EXT
  Void          getInterMergeCandidates( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMvFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand
#if QC_IC
    , Bool*         pbICFlag
#endif
#if QC_SUB_PU_TMVP
  , UChar*          peMergeTypeNeighbors
  , TComMvField*    pcMvFieldSP[2]
  , UChar*          puhInterDirSP[2]
  , UInt            uiDecCurrAbsPartIdx = 0
  , TComDataCU*     pDecCurrCU = NULL
#endif
  , Int mrgCandIdx = -1 );
#else
  Void          getInterMergeCandidates     ( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand 
#if QC_IC
    , Bool*         pbICFlag
#endif
#if QC_SUB_PU_TMVP
  , UChar*          peMergeTypeNeighbors
  , TComMvField*    pcMvFieldSP
  , UChar*          puhInterDirSP
  , UInt            uiDecCurrAbsPartIdx = 0
  , TComDataCU*     pDecCurrCU = NULL
#endif
  , Int mrgCandIdx = -1 );
#endif
#if QC_SUB_PU_TMVP
  Void          getSPPara(Int iPUWidth, Int iPUHeight, Int& iNumSP, Int& iNumSPInOneLine, Int& iSPWidth, Int& iSPHeight);
  Void          getSPAbsPartIdx(UInt uiBaseAbsPartIdx, Int iWidth, Int iHeight, Int iPartIdx, Int iNumPartLine, UInt& ruiPartAddr );
  Void          setInterDirSP( UInt uiDir, UInt uiAbsPartIdx, Int iWidth, Int iHeight );
#if QC_SUB_PU_TMVP_EXT && QC_SUB_PU_TMVP_V08==0
  Bool          deriveScaledMotionTemporalForOneDirection( TComDataCU* pcTempCU,RefPicList eCurrRefPicList, TComMv &cColMv, UInt uiAbsPartIdx, Int iTargetRefIdx, TComPic *pColPic
#if QC_IC
    , Bool& rbICFlag
#endif
    );
#else
  Bool          deriveScaledMotionTemporalForOneDirection( TComDataCU* pcTempCU,RefPicList eCurrRefPicList, TComMv &cColMv, UInt uiAbsPartIdx, Int iTargetRefIdx
#if QC_IC
    , Bool& rbICFlag
#endif
    );
#endif
  Bool          getInterMergeSubPUTmvpCandidate ( UInt uiPUIdx,  TComMvField* pcMvFieldSP, UChar* puhInterDirSP, TComMvField* pcMvFieldDefault, UChar* pcInterDirDefault, TComMv cTMv, Bool bMrgIdxMatchATMVPCan, 
#if QC_IC
    Bool& rbICFlag,
#endif
    Int iPocColPic =0, TComDataCU* pDecCurrCU = NULL, UInt uiDecCurrAbsPartIdx=0);
  TComPic *     getPicfromPOC (Int iPocColPic);
#endif
  Void          deriveLeftRightTopIdxGeneral  ( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT );
  Void          deriveLeftBottomIdxGeneral    ( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLB );
  
  
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for modes
  // -------------------------------------------------------------------------------------------------------------------
  
  Bool          isIntra   ( UInt uiPartIdx )  { return m_pePredMode[ uiPartIdx ] == MODE_INTRA; }
  Bool          isSkipped ( UInt uiPartIdx );                                                     ///< SKIP (no residual)
  Bool          isBipredRestriction( UInt puIdx );

#if QC_LARGE_CTU
  Void          getMaxMinCUDepth( UChar & rucMinDepth , UChar & rucMaxDepth , UInt uiAbsPartIdx );
#endif
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for symbol prediction (most probable / mode conversion)
  // -------------------------------------------------------------------------------------------------------------------
  
  UInt          getIntraSizeIdx                 ( UInt uiAbsPartIdx                                       );
  
  Void          getAllowedChromaDir             ( UInt uiAbsPartIdx, UInt* uiModeList );

  Int           getIntraDirLumaPredictor        ( UInt uiAbsPartIdx, Int* uiIntraDirPred
#if QC_USE_65ANG_MODES
    , Int &iAboveLeftCase
#endif
    , Int* piMode = NULL 
    );
  
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for SBAC context
  // -------------------------------------------------------------------------------------------------------------------
  
  UInt          getCtxSplitFlag                 ( UInt   uiAbsPartIdx, UInt uiDepth                   );
  UInt          getCtxQtCbf                     ( TextType eType, UInt uiTrDepth );
  UInt          getCtxSkipFlag                  ( UInt   uiAbsPartIdx                                 );
#if QC_IMV
  UInt          getCtxiMVFlag                  ( UInt   uiAbsPartIdx                                 );
  Bool          hasSubCUNonZeroMVd             ();
#endif
#if QC_FRUC_MERGE
  UInt          getCtxFRUCMgrMode              ( UInt uiAbsPartIdx );
  UInt          getCtxFRUCME                   ( UInt uiAbsPartIdx );
#endif
  UInt          getCtxInterDir                  ( UInt   uiAbsPartIdx                                 );
  
  UInt          getSliceStartCU         ( UInt pos )                  { return m_sliceStartCU[pos-m_uiAbsIdxInLCU];                                                                                          }
  UInt          getSliceSegmentStartCU  ( UInt pos )                  { return m_sliceSegmentStartCU[pos-m_uiAbsIdxInLCU];                                                                                   }
  UInt&         getTotalBins            ()                            { return m_uiTotalBins;                                                                                                  }
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for RD cost storage
  // -------------------------------------------------------------------------------------------------------------------
  
  Double&       getTotalCost()                  { return m_dTotalCost;        }
  UInt&         getTotalDistortion()            { return m_uiTotalDistortion; }
  UInt&         getTotalBits()                  { return m_uiTotalBits;       }
  UInt&         getTotalNumPart()               { return m_uiNumPartition;    }

  UInt          getCoefScanIdx(UInt uiAbsPartIdx, UInt uiWidth, Bool bIsLuma, Bool bIsIntra);

#if QC_USE_65ANG_MODES
  Bool          getUseExtIntraAngModes(UInt uiWidth);
#endif

#if QC_SUB_PU_TMVP
  UChar*        getMergeType         ()                        { return m_peMergeType;          }
  UChar         getMergeType        ( UInt uiIdx )             { return m_peMergeType[uiIdx];   }
  Void          setMergeType        ( UInt uiIdx, UChar e )    { m_peMergeType[uiIdx] = e;      }
#endif

#if ALF_HM3_QC_REFACTOR
  UInt*         getAlfCtrlFlag        ()                        { return m_puiAlfCtrlFlag;            }
  UInt          getAlfCtrlFlag        ( UInt uiIdx )            { return m_puiAlfCtrlFlag[uiIdx];     }
  Void          setAlfCtrlFlag        ( UInt uiIdx, UInt uiFlag){ m_puiAlfCtrlFlag[uiIdx] = uiFlag;   }
  Void          setAlfCtrlFlagSubParts( UInt uiFlag, UInt uiAbsPartIdx, UInt uiDepth );
  Void          createTmpAlfCtrlFlag  ();
  Void          destroyTmpAlfCtrlFlag ();
  Void          copyAlfCtrlFlagToTmp  ();
  Void          copyAlfCtrlFlagFromTmp();
  UInt          getCtxAlfCtrlFlag               ( UInt   uiAbsPartIdx                                 );
#endif
#if QC_OBMC
  Bool          getNeigMotion( UInt uiAbsPartIdx, TComMvField cNeigMvField[2], Int &irNeigPredDir, Int iDir, TComMvField cCurMvField[2], Int &iCurrDir, UInt uiZeroIdx, Bool &bTobeStored);
#endif
#if INTER_KLT
  Void          interpolatePic(TComPic* pcPic);
#endif
};

namespace RasterAddress
{
  /** Check whether 2 addresses point to the same column
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualCol( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    // addrA % numUnitsPerRow == addrB % numUnitsPerRow
    return (( addrA ^ addrB ) &  ( numUnitsPerRow - 1 ) ) == 0;
  }
  
  /** Check whether 2 addresses point to the same row
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualRow( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    // addrA / numUnitsPerRow == addrB / numUnitsPerRow
    return (( addrA ^ addrB ) &~ ( numUnitsPerRow - 1 ) ) == 0;
  }
  
  /** Check whether 2 addresses point to the same row or column
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualRowOrCol( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    return isEqualCol( addrA, addrB, numUnitsPerRow ) | isEqualRow( addrA, addrB, numUnitsPerRow );
  }
  
  /** Check whether one address points to the first column
   * \param addr           Address in raster scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isZeroCol( Int addr, Int numUnitsPerRow )
  {
    // addr % numUnitsPerRow == 0
    return ( addr & ( numUnitsPerRow - 1 ) ) == 0;
  }
  
  /** Check whether one address points to the first row
   * \param addr           Address in raster scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isZeroRow( Int addr, Int numUnitsPerRow )
  {
    // addr / numUnitsPerRow == 0
    return ( addr &~ ( numUnitsPerRow - 1 ) ) == 0;
  }
  
  /** Check whether one address points to a column whose index is smaller than a given value
   * \param addr           Address in raster scan order
   * \param val            Given column index value
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool lessThanCol( Int addr, Int val, Int numUnitsPerRow )
  {
    // addr % numUnitsPerRow < val
    return ( addr & ( numUnitsPerRow - 1 ) ) < val;
  }
  
  /** Check whether one address points to a row whose index is smaller than a given value
   * \param addr           Address in raster scan order
   * \param val            Given row index value
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool lessThanRow( Int addr, Int val, Int numUnitsPerRow )
  {
    // addr / numUnitsPerRow < val
    return addr < val * numUnitsPerRow;
  }
};

//! \}

#endif
