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

/** \file     TDecEntropy.cpp
    \brief    entropy decoder class
*/

#include "TDecEntropy.h"
#include "TLibCommon/TComTU.h"
#include "TLibCommon/TComPrediction.h"

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
#include "../TLibCommon/Debug.h"
static const Bool bDebugRQT = DebugOptionList::DebugRQT.getInt()!=0;
static const Bool bDebugPredEnabled = DebugOptionList::DebugPred.getInt()!=0;
#endif

//! \ingroup TLibDecoder
//! \{

Void TDecEntropy::setEntropyDecoder         ( TDecEntropyIf* p )
{
  m_pcEntropyDecoderIf = p;
}

#include "TLibCommon/TComSampleAdaptiveOffset.h"

Void TDecEntropy::decodeSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  m_pcEntropyDecoderIf->parseSkipFlag( pcCU, uiAbsPartIdx, uiDepth );
}


Void TDecEntropy::decodeCUTransquantBypassFlag(TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  m_pcEntropyDecoderIf->parseCUTransquantBypassFlag( pcCU, uiAbsPartIdx, uiDepth );
}


/** decode merge flag
 * \param pcSubCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \param uiPUIdx
 * \returns Void
 */
Void TDecEntropy::decodeMergeFlag( TComDataCU* pcSubCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx )
{
  // at least one merge candidate exists
  m_pcEntropyDecoderIf->parseMergeFlag( pcSubCU, uiAbsPartIdx, uiDepth, uiPUIdx );
}

/** decode merge index
 * \param pcCU
 * \param uiPartIdx
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TDecEntropy::decodeMergeIndex( TComDataCU* pcCU, UInt uiPartIdx, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiMergeIndex = 0;
  m_pcEntropyDecoderIf->parseMergeIndex( pcCU, uiMergeIndex );
  pcCU->setMergeIndexSubParts( uiMergeIndex, uiAbsPartIdx, uiPartIdx, uiDepth );
}

Void TDecEntropy::decodeSplitFlag   ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  m_pcEntropyDecoderIf->parseSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
}

Void TDecEntropy::decodePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  m_pcEntropyDecoderIf->parsePredMode( pcCU, uiAbsPartIdx, uiDepth );
}

Void TDecEntropy::decodePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  m_pcEntropyDecoderIf->parsePartSize( pcCU, uiAbsPartIdx, uiDepth );
}

Void TDecEntropy::decodePredInfo    ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, TComDataCU* pcSubCU )
{
  if( pcCU->isIntra( uiAbsPartIdx ) )                                 // If it is Intra mode, encode intra prediction mode.
  {
#if INTRA_NSP
  PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
  UInt uiNumPU = ( ePartSize == SIZE_2Nx2N ? 1 : ( ePartSize == SIZE_NxN ? 4 : 2 ) );
  UInt uiPUOffset = ( g_auiPUOffset[UInt( ePartSize )] << ( ( pcCU->getSlice()->getSPS()->getMaxTotalCUDepth() - uiDepth ) << 1 ) ) >> 4;
  for ( UInt uiPartIdx = 0, uiSubPartIdx = uiAbsPartIdx; uiPartIdx < uiNumPU; uiPartIdx++, uiSubPartIdx += uiPUOffset )
    {
      decodeIntraDirModeLuma  ( pcCU, uiSubPartIdx, uiDepth, uiPartIdx );
      if(pcCU->getWidth(uiSubPartIdx) > 8 || uiPartIdx == 0)
      {
        decodeIntraDirModeChroma( pcCU, uiSubPartIdx, uiDepth, uiPartIdx );
      }
    }
#else
    decodeIntraDirModeLuma  ( pcCU, uiAbsPartIdx, uiDepth );
    if (pcCU->getPic()->getChromaFormat()!=CHROMA_400)
    {
      decodeIntraDirModeChroma( pcCU, uiAbsPartIdx, uiDepth );

      if (enable4ChromaPUsInIntraNxNCU(pcCU->getPic()->getChromaFormat()) && pcCU->getPartitionSize( uiAbsPartIdx )==SIZE_NxN)
      {
        UInt uiPartOffset = ( pcCU->getPic()->getNumPartitionsInCtu() >> ( pcCU->getDepth(uiAbsPartIdx) << 1 ) ) >> 2;
        decodeIntraDirModeChroma( pcCU, uiAbsPartIdx + uiPartOffset,   uiDepth+1 );
        decodeIntraDirModeChroma( pcCU, uiAbsPartIdx + uiPartOffset*2, uiDepth+1 );
        decodeIntraDirModeChroma( pcCU, uiAbsPartIdx + uiPartOffset*3, uiDepth+1 );
      }
    }
#endif
  }
  else                                                                // if it is Inter mode, encode motion vector and reference index
  {
    decodePUWise( pcCU, uiAbsPartIdx, uiDepth, pcSubCU );
  }
}

/** Parse I_PCM information.
 * \param pcCU  pointer to CUpointer to CU
 * \param uiAbsPartIdx CU index
 * \param uiDepth CU depth
 * \returns Void
 */
Void TDecEntropy::decodeIPCMInfo( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if(!pcCU->getSlice()->getSPS()->getUsePCM()
    || pcCU->getWidth(uiAbsPartIdx) > (1<<pcCU->getSlice()->getSPS()->getPCMLog2MaxSize())
    || pcCU->getWidth(uiAbsPartIdx) < (1<<pcCU->getSlice()->getSPS()->getPCMLog2MinSize()) )
  {
    return;
  }

  m_pcEntropyDecoderIf->parseIPCMInfo( pcCU, uiAbsPartIdx, uiDepth );
}

#if INTRA_NSP
Void TDecEntropy::decodeIntraDirModeLuma  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx   )
{
  m_pcEntropyDecoderIf->parseIntraDirLumaAng( pcCU, uiAbsPartIdx, uiDepth, uiPUIdx );
}
#else
Void TDecEntropy::decodeIntraDirModeLuma  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  m_pcEntropyDecoderIf->parseIntraDirLumaAng( pcCU, uiAbsPartIdx, uiDepth );
}
#endif 

#if INTRA_NSP
Void TDecEntropy::decodeIntraDirModeChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx  )
#else
Void TDecEntropy::decodeIntraDirModeChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
#endif 
{
#if INTRA_NSP
  m_pcEntropyDecoderIf->parseIntraDirChroma( pcCU, uiAbsPartIdx, uiDepth, uiPUIdx );
#else
  m_pcEntropyDecoderIf->parseIntraDirChroma( pcCU, uiAbsPartIdx, uiDepth );
#endif 
#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  if (bDebugPredEnabled)
  {
    UInt cdir=pcCU->getIntraDir(CHANNEL_TYPE_CHROMA, uiAbsPartIdx);
    if (cdir==36)
    {
      cdir=pcCU->getIntraDir(CHANNEL_TYPE_LUMA, uiAbsPartIdx);
    }
    printf("coding chroma Intra dir: %d, uiAbsPartIdx: %d, luma dir: %d\n", cdir, uiAbsPartIdx, pcCU->getIntraDir(CHANNEL_TYPE_LUMA, uiAbsPartIdx));
  }
#endif
}


/** decode motion information for every PU block.
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \param pcSubCU
 * \returns Void
 */
Void TDecEntropy::decodePUWise( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, TComDataCU* pcSubCU )
{
  PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
  UInt uiNumPU = ( ePartSize == SIZE_2Nx2N ? 1 : ( ePartSize == SIZE_NxN ? 4 : 2 ) );
  UInt uiPUOffset = ( g_auiPUOffset[UInt( ePartSize )] << ( ( pcCU->getSlice()->getSPS()->getMaxTotalCUDepth() - uiDepth ) << 1 ) ) >> 4;

  TComMvField cMvFieldNeighbours[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];

  for ( UInt ui = 0; ui < pcCU->getSlice()->getMaxNumMergeCand(); ui++ )
  {
    uhInterDirNeighbours[ui] = 0;
  }
  Int numValidMergeCand = 0;
  Bool hasMergedCandList = false;

  pcSubCU->copyInterPredInfoFrom( pcCU, uiAbsPartIdx, REF_PIC_LIST_0 );
  pcSubCU->copyInterPredInfoFrom( pcCU, uiAbsPartIdx, REF_PIC_LIST_1 );
  for ( UInt uiPartIdx = 0, uiSubPartIdx = uiAbsPartIdx; uiPartIdx < uiNumPU; uiPartIdx++, uiSubPartIdx += uiPUOffset )
  {
    decodeMergeFlag( pcCU, uiSubPartIdx, uiDepth, uiPartIdx );
    if ( pcCU->getMergeFlag( uiSubPartIdx ) )
    {
      decodeMergeIndex( pcCU, uiPartIdx, uiSubPartIdx, uiDepth );
#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      if (bDebugPredEnabled)
      {
        std::cout << "Coded merge flag, CU absPartIdx: " << uiAbsPartIdx << " PU(" << uiPartIdx << ") absPartIdx: " << uiSubPartIdx;
        std::cout << " merge index: " << (UInt)pcCU->getMergeIndex(uiSubPartIdx) << std::endl;
      }
#endif

      UInt uiMergeIndex = pcCU->getMergeIndex(uiSubPartIdx);
      if ( pcCU->getSlice()->getPPS()->getLog2ParallelMergeLevelMinus2() && ePartSize != SIZE_2Nx2N && pcSubCU->getWidth( 0 ) <= 8 )
      {
        if ( !hasMergedCandList )
        {
          pcSubCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth ); // temporarily set.
          pcSubCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );
          pcSubCU->setPartSizeSubParts( ePartSize, 0, uiDepth ); // restore.
          hasMergedCandList = true;
        }
      }
      else
      {
        uiMergeIndex = pcCU->getMergeIndex(uiSubPartIdx);
        pcSubCU->getInterMergeCandidates( uiSubPartIdx-uiAbsPartIdx, uiPartIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand, uiMergeIndex );
      }

      pcCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeIndex], uiSubPartIdx, uiPartIdx, uiDepth );

      TComMv cTmpMv( 0, 0 );
      for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
      {
        if ( pcCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
        {
          pcCU->setMVPIdxSubParts( 0, RefPicList( uiRefListIdx ), uiSubPartIdx, uiPartIdx, uiDepth);
          pcCU->setMVPNumSubParts( 0, RefPicList( uiRefListIdx ), uiSubPartIdx, uiPartIdx, uiDepth);
          pcCU->getCUMvField( RefPicList( uiRefListIdx ) )->setAllMvd( cTmpMv, ePartSize, uiSubPartIdx, uiDepth, uiPartIdx );
          pcCU->getCUMvField( RefPicList( uiRefListIdx ) )->setAllMvField( cMvFieldNeighbours[ 2*uiMergeIndex + uiRefListIdx ], ePartSize, uiSubPartIdx, uiDepth, uiPartIdx );

        }
      }
    }
    else
    {
      decodeInterDirPU( pcCU, uiSubPartIdx, uiDepth, uiPartIdx );
      for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
      {
        if ( pcCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
        {
          decodeRefFrmIdxPU( pcCU,    uiSubPartIdx,              uiDepth, uiPartIdx, RefPicList( uiRefListIdx ) );
          decodeMvdPU      ( pcCU,    uiSubPartIdx,              uiDepth, uiPartIdx, RefPicList( uiRefListIdx ) );
          decodeMVPIdxPU   ( pcSubCU, uiSubPartIdx-uiAbsPartIdx, uiDepth, uiPartIdx, RefPicList( uiRefListIdx ) );
#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
          if (bDebugPredEnabled)
          {
            std::cout << "refListIdx: " << uiRefListIdx << std::endl;
            std::cout << "MVD horizontal: " << pcCU->getCUMvField(RefPicList(uiRefListIdx))->getMvd( uiAbsPartIdx ).getHor() << std::endl;
            std::cout << "MVD vertical:   " << pcCU->getCUMvField(RefPicList(uiRefListIdx))->getMvd( uiAbsPartIdx ).getVer() << std::endl;
            std::cout << "MVPIdxPU: " << pcCU->getMVPIdx(RefPicList( uiRefListIdx ), uiSubPartIdx) << std::endl;
            std::cout << "InterDir: " << (UInt)pcCU->getInterDir(uiSubPartIdx) << std::endl;
          }
#endif
        }
      }
    }

    if ( (pcCU->getInterDir(uiSubPartIdx) == 3) && pcSubCU->isBipredRestriction(uiPartIdx) )
    {
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( TComMv(0,0), ePartSize, uiSubPartIdx, uiDepth, uiPartIdx);
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, ePartSize, uiSubPartIdx, uiDepth, uiPartIdx);
      pcCU->setInterDirSubParts( 1, uiSubPartIdx, uiPartIdx, uiDepth);
    }
  }
  return;
}

/** decode inter direction for a PU block
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \param uiPartIdx
 * \returns Void
 */
Void TDecEntropy::decodeInterDirPU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPartIdx )
{
  UInt uiInterDir;

  if ( pcCU->getSlice()->isInterP() )
  {
    uiInterDir = 1;
  }
  else
  {
    m_pcEntropyDecoderIf->parseInterDir( pcCU, uiInterDir, uiAbsPartIdx );
  }

  pcCU->setInterDirSubParts( uiInterDir, uiAbsPartIdx, uiPartIdx, uiDepth );
}

Void TDecEntropy::decodeRefFrmIdxPU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPartIdx, RefPicList eRefList )
{
  Int iRefFrmIdx = 0;
  Int iParseRefFrmIdx = pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList );

  if ( pcCU->getSlice()->getNumRefIdx( eRefList ) > 1 && iParseRefFrmIdx )
  {
    m_pcEntropyDecoderIf->parseRefFrmIdx( pcCU, iRefFrmIdx, eRefList );
  }
  else if ( !iParseRefFrmIdx )
  {
    iRefFrmIdx = NOT_VALID;
  }
  else
  {
    iRefFrmIdx = 0;
  }

  PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
  pcCU->getCUMvField( eRefList )->setAllRefIdx( iRefFrmIdx, ePartSize, uiAbsPartIdx, uiDepth, uiPartIdx );
}

/** decode motion vector difference for a PU block
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \param uiPartIdx
 * \param eRefList
 * \returns Void
 */
Void TDecEntropy::decodeMvdPU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPartIdx, RefPicList eRefList )
{
  if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
  {
    m_pcEntropyDecoderIf->parseMvd( pcCU, uiAbsPartIdx, uiPartIdx, uiDepth, eRefList );
  }
}

Void TDecEntropy::decodeMVPIdxPU( TComDataCU* pcSubCU, UInt uiPartAddr, UInt uiDepth, UInt uiPartIdx, RefPicList eRefList )
{
  Int iMVPIdx = -1;

  TComMv cZeroMv( 0, 0 );
  TComMv cMv     = cZeroMv;
  Int    iRefIdx = -1;

  TComCUMvField* pcSubCUMvField = pcSubCU->getCUMvField( eRefList );
  AMVPInfo* pAMVPInfo = pcSubCUMvField->getAMVPInfo();

  iRefIdx = pcSubCUMvField->getRefIdx(uiPartAddr);
  cMv = cZeroMv;

  if ( (pcSubCU->getInterDir(uiPartAddr) & ( 1 << eRefList )) )
  {
    m_pcEntropyDecoderIf->parseMVPIdx( iMVPIdx );
  }
  pcSubCU->fillMvpCand(uiPartIdx, uiPartAddr, eRefList, iRefIdx, pAMVPInfo);
  pcSubCU->setMVPNumSubParts(pAMVPInfo->iN, eRefList, uiPartAddr, uiPartIdx, uiDepth);
  pcSubCU->setMVPIdxSubParts( iMVPIdx, eRefList, uiPartAddr, uiPartIdx, uiDepth );
  if ( iRefIdx >= 0 )
  {
    m_pcPrediction->getMvPredAMVP( pcSubCU, uiPartIdx, uiPartAddr, eRefList, cMv);
    cMv += pcSubCUMvField->getMvd( uiPartAddr );
  }

  PartSize ePartSize = pcSubCU->getPartitionSize( uiPartAddr );
  pcSubCU->getCUMvField( eRefList )->setAllMv(cMv, ePartSize, uiPartAddr, 0, uiPartIdx);
}

#if INTER_NSP
Void TDecEntropy::xDecodeTransform        ( Bool& bCodeDQP, Bool& isChromaQpAdjCoded, TComTU &rTu, const Int quadtreeTULog2MinSizeInCU, Int iFirstIteration )
#else
Void TDecEntropy::xDecodeTransform        ( Bool& bCodeDQP, Bool& isChromaQpAdjCoded, TComTU &rTu, const Int quadtreeTULog2MinSizeInCU )
#endif
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();
  const UInt uiDepth=rTu.GetTransformDepthTotal();
  const UInt uiTrDepth = rTu.GetTransformDepthRel();
#if INTRA_NSP  
  const UInt uiMaxSizeBit = max(g_aucConvertToBit[rTu.getRect(COMPONENT_Y).width],g_aucConvertToBit[rTu.getRect(COMPONENT_Y).height])  + 2;  
#endif 
  UInt uiSubdiv;
  const UInt numValidComponent = pcCU->getPic()->getNumberValidComponents();
  const Bool bChroma = isChromaEnabled(pcCU->getPic()->getChromaFormat());

  const UInt uiLog2TrafoSize = rTu.GetLog2LumaTrSize();
#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  if (bDebugRQT)
  {
    printf("x..codeTransform: offsetLuma=%d offsetChroma=%d absPartIdx=%d, uiDepth=%d\n width=%d, height=%d, uiTrIdx=%d, uiInnerQuadIdx=%d\n",
        rTu.getCoefficientOffset(COMPONENT_Y), rTu.getCoefficientOffset(COMPONENT_Cb), uiAbsPartIdx, uiDepth, rTu.getRect(COMPONENT_Y).width, rTu.getRect(COMPONENT_Y).height, rTu.GetTransformDepthRel(), rTu.GetSectionNumber());
    fflush(stdout);
  }
#endif
#if INTRA_NSP
  if( pcCU->isIntra(uiAbsPartIdx) && pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N && uiDepth == pcCU->getDepth(uiAbsPartIdx) )
#else
  if( pcCU->isIntra(uiAbsPartIdx) && pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_NxN && uiDepth == pcCU->getDepth(uiAbsPartIdx) )
#endif 
  {
    uiSubdiv = 1;
  }
  else if( (pcCU->getSlice()->getSPS()->getQuadtreeTUMaxDepthInter() == 1) && (pcCU->isInter(uiAbsPartIdx)) && ( pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N ) && (uiDepth == pcCU->getDepth(uiAbsPartIdx)) )
  {
    uiSubdiv = (uiLog2TrafoSize >quadtreeTULog2MinSizeInCU);
  }
#if INTRA_NSP
  else if( uiMaxSizeBit > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
#else
  else if( uiLog2TrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
#endif 
  {
    uiSubdiv = 1;
  }
  else if( uiLog2TrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    uiSubdiv = 0;
  }
  else if( uiLog2TrafoSize == quadtreeTULog2MinSizeInCU )
  {
    uiSubdiv = 0;
  }
  else
  {
    assert( uiLog2TrafoSize > quadtreeTULog2MinSizeInCU );
#if INTER_NSP
    if (pcCU->isIntra(uiAbsPartIdx) || (rTu.GetPUIdx() == 0 && (iFirstIteration || !pcCU->getInterNstFlag(uiAbsPartIdx) ) ))
    {
      m_pcEntropyDecoderIf->parseTransformSubdivFlag( uiSubdiv, 5 - uiLog2TrafoSize );
    }
    else
    {
      uiSubdiv = 0;
    }
#else
      m_pcEntropyDecoderIf->parseTransformSubdivFlag( uiSubdiv, 5 - uiLog2TrafoSize );
#endif
  }

#if INTER_NSP
  if (!uiSubdiv)
  {
    if (rTu.tryInterNst(pcCU, uiAbsPartIdx) && iFirstIteration)
    {
      m_pcEntropyDecoderIf->parseInterNstFlag( pcCU, uiAbsPartIdx ); 
    }
  }
#endif

  for(Int chan=COMPONENT_Cb; chan<numValidComponent; chan++)
  {
    const ComponentID compID=ComponentID(chan);
#if INTRA_NSP==0
    const UInt trDepthTotalAdj=rTu.GetTransformDepthTotalAdj(compID);
#endif 

    const Bool bFirstCbfOfCU = uiTrDepth == 0;

#if INTER_NSP
    if ( pcCU->getInterNstFlag (uiAbsPartIdx) )
    {
      assert (uiTrDepth == 0);
      assert (uiSubdiv == 0);
      if( rTu.ProcessComponentSection(compID) && !iFirstIteration)
      {
        m_pcEntropyDecoderIf->parseQtCbf( rTu, compID, (uiSubdiv == 0) );
      }
    }
    else
    {
      if( bFirstCbfOfCU )
      {
#if INTRA_NSP
        pcCU->setCbfSubParts ( 0, compID, rTu.GetAbsPartIdxTU(compID), pcCU->getDepth(uiAbsPartIdx),  uiTrDepth, rTu.GetPUIdx());            
#else
        pcCU->setCbfSubParts( 0, compID, rTu.GetAbsPartIdxTU(compID), trDepthTotalAdj);
#endif 
      }
      if( bFirstCbfOfCU || rTu.ProcessingAllQuadrants(compID) )
      {
        if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth - 1 ) )
        {
          m_pcEntropyDecoderIf->parseQtCbf( rTu, compID, (uiSubdiv == 0) );
        }
      }
#if INTRA_NSP
      else
      {
        pcCU->setCbfSubParts( pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth - 1 ) << uiTrDepth, compID, uiAbsPartIdx, pcCU->getDepth( uiAbsPartIdx ), uiTrDepth, rTu.GetPUIdx());
      }
#endif 
    }
#else
    if( bFirstCbfOfCU )
    {
#if INTRA_NSP
      pcCU->setCbfSubParts ( 0, compID, rTu.GetAbsPartIdxTU(compID), pcCU->getDepth(uiAbsPartIdx),  uiTrDepth, rTu.GetPUIdx());            
#else
      pcCU->setCbfSubParts( 0, compID, rTu.GetAbsPartIdxTU(compID), trDepthTotalAdj);
#endif 
    }
    if( bFirstCbfOfCU || rTu.ProcessingAllQuadrants(compID) )
    {
      if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth - 1 ) )
      {
        m_pcEntropyDecoderIf->parseQtCbf( rTu, compID, (uiSubdiv == 0) );
      }
    }
#if INTRA_NSP
    else
    {
      pcCU->setCbfSubParts( pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth - 1 ) << uiTrDepth, compID, uiAbsPartIdx, pcCU->getDepth( uiAbsPartIdx ), uiTrDepth, rTu.GetPUIdx());
    }
#endif 
#endif
  }

#if INTER_NSP
  UInt uiTempSubdiv = uiSubdiv;
  if (pcCU->getInterNstFlag(uiAbsPartIdx))
  {
    assert (uiSubdiv == 0);
    if (iFirstIteration)
    {
      uiTempSubdiv  = 1;
    }
  }
  if( uiTempSubdiv )
#else
  if( uiSubdiv )
#endif
  {
#if INTRA_NSP
    UInt uiYUVCbf[MAX_NUM_COMPONENT] = {0,0,0};
#if INTER_NSP
    if( (pcCU->isIntra(uiAbsPartIdx) && pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N && uiTrDepth == 0) || pcCU->getInterNstFlag(uiAbsPartIdx) )
#else
    if( pcCU->isIntra(uiAbsPartIdx) && pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N && uiTrDepth == 0) 
#endif
    {      
      const UInt  uiNumPU  = pcCU->getNumPartitions(uiAbsPartIdx); 
 #if INTER_NSP
      Int initTUDepth = 0;
      if (pcCU->getInterNstFlag(uiAbsPartIdx))
      {
        initTUDepth = 0;
      }
      else
      {
        assert (pcCU->isIntra(uiAbsPartIdx));
        initTUDepth = 1;
      }
      
      TComTURecurse tuRecurseCU(pcCU, uiAbsPartIdx, uiDepth, initTUDepth);
#else
      TComTURecurse tuRecurseCU(pcCU, uiAbsPartIdx, uiDepth,1);
#endif
      TComTURecurse tuRecurseWithPU(tuRecurseCU, false, TComTU::DONT_SPLIT);  
      for( UInt uiPUIdx = 0; uiPUIdx < uiNumPU; uiPUIdx++)
      {
        tuRecurseWithPU.SetPUIdx(uiPUIdx);
        do
        {
          xDecodeTransform( bCodeDQP, isChromaQpAdjCoded, tuRecurseWithPU, quadtreeTULog2MinSizeInCU );          
          UInt childTUAbsPartIdx=tuRecurseWithPU.GetAbsPartIdxTU();
          for(UInt ch=0; ch<numValidComponent; ch++)
          {
#if INTER_NSP
            uiYUVCbf[ch] |= pcCU->getCbf(childTUAbsPartIdx , ComponentID(ch),  uiTrDepth+initTUDepth );
#else
            uiYUVCbf[ch] |= pcCU->getCbf(childTUAbsPartIdx , ComponentID(ch),  uiTrDepth+1 );
#endif
          }
        } while (tuRecurseWithPU.nextSection(tuRecurseCU));
        tuRecurseWithPU.nextSectionWithPU(tuRecurseCU,uiPUIdx+((pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2NxN) ? 2:1), (uiPUIdx==(uiNumPU-2)));
      }

#if INTER_NSP
      if (!pcCU->getInterNstFlag(uiAbsPartIdx))
      {
#endif
      for(UInt ch=0; ch<numValidComponent; ch++)
      {
        TComTURecurse tuRecurseCbf(tuRecurseCU, false);
        do
        { 
          if (tuRecurseCbf.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
          {         
            const UInt flag =  1<<(uiTrDepth);        
            pcCU->UpdateCbfSubParts  (flag, ComponentID(ch), tuRecurseCbf.GetAbsPartIdxTU(), pcCU->getDepth(  tuRecurseCbf.GetAbsPartIdxTU() ),  tuRecurseCbf.GetTransformDepthRel(), tuRecurseCbf.GetPUIdx());      
          }
        } while (tuRecurseCbf.nextSection(tuRecurseCU) );
      }
#if INTER_NSP
      }
#endif

    }
    else
    { 
      TComTURecurse tuRecurseChild(rTu, true);
      do
      {
        xDecodeTransform( bCodeDQP, isChromaQpAdjCoded, tuRecurseChild, quadtreeTULog2MinSizeInCU );
        UInt childTUAbsPartIdx=tuRecurseChild.GetAbsPartIdxTU();
        for(UInt ch=0; ch<numValidComponent; ch++)
        {
          uiYUVCbf[ch] |= pcCU->getCbf(childTUAbsPartIdx , ComponentID(ch),  uiTrDepth+1 );
        }
      } while (tuRecurseChild.nextSection(rTu) );
      for(UInt ch=0; ch<numValidComponent; ch++)
      {
        TComTURecurse tuRecurseCbf(rTu, false);
        do
        { 
          if (tuRecurseCbf.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
          {         
            const UInt flag =  1<<(uiTrDepth);        
            pcCU->UpdateCbfSubParts  (flag, ComponentID(ch), tuRecurseCbf.GetAbsPartIdxTU(), pcCU->getDepth(  tuRecurseCbf.GetAbsPartIdxTU() ),  tuRecurseCbf.GetTransformDepthRel(), tuRecurseCbf.GetPUIdx());      
          }
        } while (tuRecurseCbf.nextSection(rTu) );
      }
    }
#else
    const UInt uiQPartNum = pcCU->getPic()->getNumPartitionsInCtu() >> ((uiDepth+1) << 1);
    UInt uiYUVCbf[MAX_NUM_COMPONENT] = {0,0,0};
    TComTURecurse tuRecurseChild(rTu, true);
    do
    {
      xDecodeTransform( bCodeDQP, isChromaQpAdjCoded, tuRecurseChild, quadtreeTULog2MinSizeInCU );
      UInt childTUAbsPartIdx=tuRecurseChild.GetAbsPartIdxTU();
      for(UInt ch=0; ch<numValidComponent; ch++)
      {
        uiYUVCbf[ch] |= pcCU->getCbf(childTUAbsPartIdx , ComponentID(ch),  uiTrDepth+1 );
      }
    } while (tuRecurseChild.nextSection(rTu) );
    for(UInt ch=0; ch<numValidComponent; ch++)
    {
         UChar *pBase = pcCU->getCbf( ComponentID(ch) ) + uiAbsPartIdx;
         const UChar flag = uiYUVCbf[ch] << uiTrDepth;
         for( UInt ui = 0; ui < 4 * uiQPartNum; ++ui )
         {
           pBase[ui] |= flag;
         }
    }
#endif 
  }
  else
  {
    assert( uiDepth >= pcCU->getDepth( uiAbsPartIdx ) );
#if INTRA_NSP
     pcCU->setTrIdxSubParts( uiTrDepth, uiAbsPartIdx, pcCU->getDepth( uiAbsPartIdx ), uiTrDepth, rTu.GetPUIdx());
#else
    pcCU->setTrIdxSubParts( uiTrDepth, uiAbsPartIdx, uiDepth );
#endif 
    {
      DTRACE_CABAC_VL( g_nSymbolCounter++ );
      DTRACE_CABAC_T( "\tTrIdx: abspart=" );
      DTRACE_CABAC_V( uiAbsPartIdx );
      DTRACE_CABAC_T( "\tdepth=" );
      DTRACE_CABAC_V( uiDepth );
      DTRACE_CABAC_T( "\ttrdepth=" );
      DTRACE_CABAC_V( uiTrDepth );
      DTRACE_CABAC_T( "\n" );
    }
#if INTRA_NSP
    pcCU->setCbfSubParts ( 0, COMPONENT_Y, uiAbsPartIdx, pcCU->getDepth( uiAbsPartIdx ),  uiTrDepth, rTu.GetPUIdx());            
#else
    pcCU->setCbfSubParts ( 0, COMPONENT_Y, uiAbsPartIdx, uiDepth );
#endif 
 
#if INTER_NSP
    Int cuAbsPartIdx = uiAbsPartIdx;
    if (rTu.GetPUIdx()!= 0)
    {
      cuAbsPartIdx = uiAbsPartIdx - rTu.GetRelPartIdxTU(COMPONENT_Y);
      assert (cuAbsPartIdx >= 0);
    }

    if( (!pcCU->isIntra(uiAbsPartIdx)) && uiDepth == pcCU->getDepth( uiAbsPartIdx ) && ((!bChroma) || (!pcCU->getCbf( uiAbsPartIdx, COMPONENT_Cb, 0 ) && !pcCU->getCbf( uiAbsPartIdx, COMPONENT_Cr, 0 )) )&& !pcCU->getInterNstFlag(uiAbsPartIdx) ) 
#else
    if( (!pcCU->isIntra(uiAbsPartIdx)) && uiDepth == pcCU->getDepth( uiAbsPartIdx ) && ((!bChroma) || (!pcCU->getCbf( uiAbsPartIdx, COMPONENT_Cb, 0 ) && !pcCU->getCbf( uiAbsPartIdx, COMPONENT_Cr, 0 )) ))
#endif
    {
#if INTRA_NSP
      pcCU->setCbfSubParts( 1 << uiTrDepth, COMPONENT_Y, uiAbsPartIdx, pcCU->getDepth( uiAbsPartIdx ),  uiTrDepth, rTu.GetPUIdx());            
#else
      pcCU->setCbfSubParts( 1 << uiTrDepth, COMPONENT_Y, uiAbsPartIdx, uiDepth );
#endif 
    }
    else
    {
      m_pcEntropyDecoderIf->parseQtCbf( rTu, COMPONENT_Y, true );
    }


    // transform_unit begin
    UInt cbf[MAX_NUM_COMPONENT]={0,0,0};
    Bool validCbf       = false;
    Bool validChromaCbf = false;
    const UInt uiTrIdx = rTu.GetTransformDepthRel();

    for(UInt ch=0; ch<pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      const ComponentID compID = ComponentID(ch);

      cbf[compID] = pcCU->getCbf( uiAbsPartIdx, compID, uiTrIdx );

      if (cbf[compID] != 0)
      {
        validCbf = true;
        if (isChroma(compID))
        {
          validChromaCbf = true;
        }
      }
    }

    if ( validCbf )
    {

      // dQP: only for CTU
      if ( pcCU->getSlice()->getPPS()->getUseDQP() )
      {
        if ( bCodeDQP )
        {
          const UInt uiAbsPartIdxCU=rTu.GetAbsPartIdxCU();
          decodeQP( pcCU, uiAbsPartIdxCU);
          bCodeDQP = false;
        }
      }

      if ( pcCU->getSlice()->getUseChromaQpAdj() )
      {
        if ( validChromaCbf && isChromaQpAdjCoded && !pcCU->getCUTransquantBypass(rTu.GetAbsPartIdxCU()) )
        {
          decodeChromaQpAdjustment( pcCU, rTu.GetAbsPartIdxCU() );
          isChromaQpAdjCoded = false;
        }
      }

      const UInt numValidComp=pcCU->getPic()->getNumberValidComponents();

      for(UInt ch=COMPONENT_Y; ch<numValidComp; ch++)
      {
          const ComponentID compID=ComponentID(ch);

          if( rTu.ProcessComponentSection(compID))
          {
#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
            if (bDebugRQT)
            {
              printf("Call NxN for chan %d width=%d height=%d cbf=%d\n", compID, rTu.getRect(compID).width, rTu.getRect(compID).height, 1);
            }
#endif

#if INTRA_NSP==0
            if (rTu.getRect(compID).width != rTu.getRect(compID).height)
            {
              //code two sub-TUs
              TComTURecurse subTUIterator(rTu, false, TComTU::VERTICAL_SPLIT, true, compID);

              do
              {
                const UInt subTUCBF = pcCU->getCbf(subTUIterator.GetAbsPartIdxTU(), compID, (uiTrIdx + 1));

                if (subTUCBF != 0)
                {
#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
                  if (bDebugRQT)
                  {
                    printf("Call NxN for chan %d width=%d height=%d cbf=%d\n", compID, subTUIterator.getRect(compID).width, subTUIterator.getRect(compID).height, 1);
                  }
#endif
                  m_pcEntropyDecoderIf->parseCoeffNxN( subTUIterator, compID );
                }
              } while (subTUIterator.nextSection(rTu));
            }
            else
#endif 
            {
              if(isChroma(compID) && (cbf[COMPONENT_Y] != 0))
              {
                m_pcEntropyDecoderIf->parseCrossComponentPrediction( rTu, compID );
              }

              if(cbf[compID] != 0)
              {
                m_pcEntropyDecoderIf->parseCoeffNxN( rTu, compID );
              }
            }
          }
      }
    }
    // transform_unit end  
}
}

Void TDecEntropy::decodeQP          ( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  if ( pcCU->getSlice()->getPPS()->getUseDQP() )
  {
    m_pcEntropyDecoderIf->parseDeltaQP( pcCU, uiAbsPartIdx, pcCU->getDepth( uiAbsPartIdx ) );
  }
}

Void TDecEntropy::decodeChromaQpAdjustment( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  if ( pcCU->getSlice()->getUseChromaQpAdj() )
  {
    m_pcEntropyDecoderIf->parseChromaQpAdjustment( pcCU, uiAbsPartIdx, pcCU->getDepth( uiAbsPartIdx ) );
  }
}


//! decode coefficients
Void TDecEntropy::decodeCoeff( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool& bCodeDQP, Bool& isChromaQpAdjCoded )
{
  if( pcCU->isIntra(uiAbsPartIdx) )
  {
  }
  else
  {
    UInt uiQtRootCbf = 1;
    if( !( pcCU->getPartitionSize( uiAbsPartIdx) == SIZE_2Nx2N && pcCU->getMergeFlag( uiAbsPartIdx ) ) )
    {
      m_pcEntropyDecoderIf->parseQtRootCbf( uiAbsPartIdx, uiQtRootCbf );
    }
    if ( !uiQtRootCbf )
    {
      static const UInt cbfZero[MAX_NUM_COMPONENT]={0,0,0};
      pcCU->setCbfSubParts( cbfZero, uiAbsPartIdx, uiDepth );
      pcCU->setTrIdxSubParts( 0 , uiAbsPartIdx, uiDepth );
      return;
    }

  }

 TComTURecurse tuRecurse(pcCU, uiAbsPartIdx, uiDepth);

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  if (bDebugRQT)
  {
    printf("..codeCoeff: uiAbsPartIdx=%d, PU format=%d, 2Nx2N=%d, NxN=%d\n", uiAbsPartIdx, pcCU->getPartitionSize(uiAbsPartIdx), SIZE_2Nx2N, SIZE_NxN);
  }
#endif

  Int quadtreeTULog2MinSizeInCU = pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx);

#if INTER_NSP
  xDecodeTransform( bCodeDQP, isChromaQpAdjCoded, tuRecurse, quadtreeTULog2MinSizeInCU, 1 );
#else
  xDecodeTransform( bCodeDQP, isChromaQpAdjCoded, tuRecurse, quadtreeTULog2MinSizeInCU );
#endif

}

//! \}
