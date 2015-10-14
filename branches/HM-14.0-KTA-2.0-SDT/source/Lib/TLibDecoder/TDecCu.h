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

/** \file     TDecCu.h
    \brief    CU decoder class (header)
*/

#ifndef __TDECCU__
#define __TDECCU__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComPrediction.h"
#include "TDecEntropy.h"

//! \ingroup TLibDecoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU decoder class
class TDecCu
{
private:
  UInt                m_uiMaxDepth;       ///< max. number of depth
  TComYuv**           m_ppcYuvResi;       ///< array of residual buffer
  TComYuv**           m_ppcYuvReco;       ///< array of prediction & reconstruction buffer
  TComDataCU**        m_ppcCU;            ///< CU data array
#if QC_OBMC
  TComYuv**           m_ppcTmpYuv1;       ///< array of OBMC prediction buffer
  TComYuv**           m_ppcTmpYuv2;
#endif
  // access channel
  TComTrQuant*        m_pcTrQuant;
  TComPrediction*     m_pcPrediction;
  TDecEntropy*        m_pcEntropyDecoder;

  Bool                m_bDecodeDQP;
#if QC_SUB_PU_TMVP
#if QC_SUB_PU_TMVP_EXT
  TComMvField*        m_pMvFieldSP[2];
  UChar*              m_phInterDirSP[2];
#else
  TComMvField*        m_pMvFieldSP;
  UChar*              m_phInterDirSP;
#endif
#endif
  
public:
  TDecCu();
  virtual ~TDecCu();
  
  /// initialize access channels
  Void  init                    ( TDecEntropy* pcEntropyDecoder, TComTrQuant* pcTrQuant, TComPrediction* pcPrediction );
  
  /// create internal buffers
  Void  create                  ( UInt uiMaxDepth, UInt uiMaxWidth, UInt uiMaxHeight );
  
  /// destroy internal buffers
  Void  destroy                 ();
  
  /// decode CU information
  Void  decodeCU                ( TComDataCU* pcCU, UInt& ruiIsLast );
  
  /// reconstruct CU information
  Void  decompressCU            ( TComDataCU* pcCU );
  
#if INTER_KLT
  TComPrediction* getPointerPrediction() { return m_pcPrediction; }
#endif

protected:
  
  Void xDecodeCU                ( TComDataCU* pcCU,                       UInt uiAbsPartIdx, UInt uiDepth, UInt &ruiIsLast);
  Void xFinishDecodeCU          ( TComDataCU* pcCU,                       UInt uiAbsPartIdx, UInt uiDepth, UInt &ruiIsLast);
  Bool xDecodeSliceEnd          ( TComDataCU* pcCU,                       UInt uiAbsPartIdx, UInt uiDepth);
  Void xDecompressCU            ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  
  Void xReconInter              ( TComDataCU* pcCU, UInt uiDepth );
  
  Void  xReconIntraQT           ( TComDataCU* pcCU, UInt uiDepth );
  Void  xIntraRecLumaBlk        ( TComDataCU* pcCU, UInt uiTrDepth, UInt uiAbsPartIdx, TComYuv* pcRecoYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv );
  Void  xIntraRecChromaBlk      ( TComDataCU* pcCU, UInt uiTrDepth, UInt uiAbsPartIdx, TComYuv* pcRecoYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv, UInt uiChromaId );
#if INTRA_KLT
  Void  xIntraRecLumaBlkTM      ( TComDataCU* pcCU, UInt uiTrDepth, UInt uiAbsPartIdx, TComYuv* pcRecoYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv, Int genPred0genPredAndtrainKLT1);
#endif  
  Void  xReconPCM               ( TComDataCU* pcCU, UInt uiDepth );

  Void xDecodeInterTexture      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  Void xDecodePCMTexture        ( TComDataCU* pcCU, UInt uiPartIdx, Pel *piPCM, Pel* piReco, UInt uiStride, UInt uiWidth, UInt uiHeight, TextType ttText);
  
  Void xCopyToPic               ( TComDataCU* pcCU, TComPic* pcPic, UInt uiZorderIdx, UInt uiDepth );

  Void  xIntraLumaRecQT         ( TComDataCU* pcCU, UInt uiTrDepth, UInt uiAbsPartIdx, TComYuv* pcRecoYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv );
  Void  xIntraChromaRecQT       ( TComDataCU* pcCU, UInt uiTrDepth, UInt uiAbsPartIdx, TComYuv* pcRecoYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv );

  Bool getdQPFlag               ()                        { return m_bDecodeDQP;        }
  Void setdQPFlag               ( Bool b )                { m_bDecodeDQP = b;           }
  Void xFillPCMBuffer           (TComDataCU* pCU, UInt depth);

#if QC_FRUC_MERGE
  Void xDeriveCUMV( TComDataCU * pcCU , UInt uiAbsPartIdx , UInt uiDepth );
#endif
};

//! \}
#endif

