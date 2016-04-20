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


#include "TComTU.h"
#include "TComRom.h"
#include "TComDataCU.h"
#include "TComPic.h"

//----------------------------------------------------------------------------------------------------------------------

/*static*/ const UInt TComTU::NUMBER_OF_SECTIONS[TComTU::NUMBER_OF_SPLIT_MODES] = { 1, 2, 4 };

static     const UInt         partIdxStepShift  [TComTU::NUMBER_OF_SPLIT_MODES] = { 0, 1, 2 };

//----------------------------------------------------------------------------------------------------------------------

TComTU::TComTU(TComDataCU *pcCU, const UInt absPartIdxCU, const UInt cuDepth, const UInt initTrDepthRelCU)
  : mChromaFormat(pcCU->getSlice()->getSPS()->getChromaFormatIdc()),
    mbProcessLastOfLevel(true), // does not matter. the top level is not 4 quadrants.
    mCuDepth(cuDepth),
    mSection(0),
    mSplitMode(DONT_SPLIT),
    mAbsPartIdxCU(absPartIdxCU),
    mAbsPartIdxTURelCU(0),
#if INTRA_NSP==0
    mAbsPartIdxStep(pcCU->getPic()->getNumPartitionsInCtu() >> (pcCU->getDepth(absPartIdxCU)<<1)),
#endif 
    mpcCU(pcCU),
    mLog2TrLumaSize(0),
    mpParent(NULL)
{
#if INTRA_NSP==0
  const TComSPS *pSPS=pcCU->getSlice()->getSPS();
#endif 
  const UInt baseOffset444=pcCU->getPic()->getMinCUWidth()*pcCU->getPic()->getMinCUHeight()*absPartIdxCU;
#if INTRA_NSP
  mPUIdx = 0;    
  mbLoopFilterModeSet = (initTrDepthRelCU==2) ? 1 : 0;
  UInt uiRoiNumPartition;
  Int iRoiWidth,iRoiHeight;    
  pcCU->getSize(absPartIdxCU, iRoiWidth, iRoiHeight, uiRoiNumPartition,mbLoopFilterModeSet  );
  UInt uiWidthBit  = g_aucConvertToBit[ iRoiWidth ] + 2;
  UInt uiHeightBit = g_aucConvertToBit[ iRoiHeight ] + 2;
  if(initTrDepthRelCU == 1)
    mLog2TrLumaSize = (uiWidthBit + uiHeightBit) >> 1;    
  else
    mLog2TrLumaSize = max(uiWidthBit, uiHeightBit);    
#else
  mLog2TrLumaSize = g_aucConvertToBit[pSPS->getMaxCUWidth() >> (mCuDepth+initTrDepthRelCU)]+2;
#endif 

  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {
#if INTRA_NSP 
    mTrDepthRelCU[i] = (initTrDepthRelCU==1) ? 1 : 0;
#else
    mTrDepthRelCU[i] = initTrDepthRelCU;
#endif
    const UInt csx=getComponentScaleX(ComponentID(i), mChromaFormat);
    const UInt csy=getComponentScaleY(ComponentID(i), mChromaFormat);
#if INTRA_NSP
    mOrigWidth[i]=mRect[i].width = (i < getNumberValidComponents(mChromaFormat)) ? (iRoiWidth >> csx) : 0;
    mRect[i].height              = (i < getNumberValidComponents(mChromaFormat)) ? (iRoiHeight >> csy) : 0;      
#else
    mOrigWidth[i]=mRect[i].width = (i < getNumberValidComponents(mChromaFormat)) ? (pcCU->getWidth( absPartIdxCU) >> csx) : 0;
    mRect[i].height              = (i < getNumberValidComponents(mChromaFormat)) ? (pcCU->getHeight(absPartIdxCU) >> csy) : 0;
#endif 
    mRect[i].x0=0;
    mRect[i].y0=0;
    mCodeAll[i]=true;
    mOffsets[i]=baseOffset444>>(csx+csy);
  }
#if INTRA_NSP
    deriveQPartInfo();  
    for(Int i=0;i<MAX_NUM_PARTS;i++)
    {
      mAbsStartPartIdxTURelCU[i] = mAbsPartIdxStep[i];
    }
#endif 
}


#if INTRA_NSP
TComTURecurse::TComTURecurse(      TComDataCU *pcCU,
                             const UInt        absPartIdxCU,      
                             const UInt        forcedDepthOfCU,
                             const UInt        initTrDepthRelCU)
  : TComTU(pcCU, absPartIdxCU, pcCU->getDepth(absPartIdxCU), initTrDepthRelCU)
{ }
#endif

TComTURecurse::TComTURecurse(      TComDataCU *pcCU,
                             const UInt        absPartIdxCU)
  : TComTU(pcCU, absPartIdxCU, pcCU->getDepth(absPartIdxCU), 0)
{ }

#if INTRA_NSP
TComTU::TComTU(TComTU &parent, Bool bProcessLastOfLevel, const TU_SPLIT_MODE splitMode, const Bool splitAtCurrentDepth, const ComponentID absPartIdxSourceComponent)
#else
TComTU::TComTU(TComTU &parent, const Bool bProcessLastOfLevel, const TU_SPLIT_MODE splitMode, const Bool splitAtCurrentDepth, const ComponentID absPartIdxSourceComponent)
#endif 
  : mChromaFormat(parent.mChromaFormat),
    mbProcessLastOfLevel(bProcessLastOfLevel),
    mCuDepth(parent.mCuDepth),
    mSection(0),
    mSplitMode(splitMode),
    mAbsPartIdxCU(parent.mAbsPartIdxCU),
    mAbsPartIdxTURelCU(parent.GetRelPartIdxTU(absPartIdxSourceComponent)),
#if INTRA_NSP==0
    mAbsPartIdxStep(std::max<UInt>(1, (parent.GetAbsPartIdxNumParts(absPartIdxSourceComponent) >> partIdxStepShift[splitMode]))),
#endif 
    mpcCU(parent.mpcCU),
    mLog2TrLumaSize(parent.mLog2TrLumaSize - ((splitMode != QUAD_SPLIT) ? 0 : 1)), //no change in width for vertical split
    mpParent(&parent)
{
#if INTRA_NSP
   mPUIdx = parent.mPUIdx;   
  mbLoopFilterModeSet = parent.mbLoopFilterModeSet;
#endif  
  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {    
    mTrDepthRelCU[i] = parent.mTrDepthRelCU[i] + ((splitAtCurrentDepth || (splitMode == DONT_SPLIT)) ? 0 : 1);
  }

  if (mSplitMode==DONT_SPLIT)
  {
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      mRect[i] = (parent.mRect[i]);
      mOffsets[i]=parent.mOffsets[i];
      mCodeAll[i]=true; // The 1 TU at this level is coded.
      mOrigWidth[i]=mRect[i].width;
    }
#if INTRA_NSP
    deriveQPartInfo(absPartIdxSourceComponent);
    for(Int i=0;i<MAX_NUM_PARTS;i++)
    {
      mAbsStartPartIdxTURelCU[i] = parent.GetAbsStartPartIdxRelNumParts(COMPONENT_Y, i);
    } 
#else
    return;
#endif 
  }
  else if (mSplitMode==VERTICAL_SPLIT)
  {
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      mRect[i].x0 = (parent.mRect[i].x0);
      mRect[i].y0 = (parent.mRect[i].y0);
      mRect[i].width  = (parent.mRect[i].width);
      mRect[i].height = (parent.mRect[i].height)>>1;
      mOffsets[i]=parent.mOffsets[i];
      mCodeAll[i]=true; // The 2 TUs at this level is coded.
      mOrigWidth[i]=mRect[i].width;
    }
#if INTRA_NSP==0
    return;
  }
#else
    deriveQPartInfo(absPartIdxSourceComponent);
    for(Int i=0;i<MAX_NUM_PARTS;i++)
    {
      mAbsStartPartIdxTURelCU[i] = parent.GetAbsStartPartIdxRelNumParts(COMPONENT_Y, parent.GetSectionNumber()) + mAbsPartIdxStep[i] ;
    }
  }
  else
  {
#endif 
  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {
    mRect[i].width = (parent.mRect[i].width >> 1);
    mRect[i].height= (parent.mRect[i].height>> 1);
    mRect[i].x0=parent.mRect[i].x0;
    mRect[i].y0=parent.mRect[i].y0;
    mOffsets[i]=parent.mOffsets[i];
#if INTRA_NSP 
      mOrigWidth[i]=mRect[i].width;    
    }
      deriveQPartInfo(absPartIdxSourceComponent);
      for(Int i=0;i<MAX_NUM_PARTS;i++)
      {
        mAbsStartPartIdxTURelCU[i] = parent.GetAbsStartPartIdxRelNumParts(COMPONENT_Y, parent.GetSectionNumber()) + mAbsPartIdxStep[i] ;
      }           
  }

  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {
#endif
    if ((mRect[i].width < MIN_TU_SIZE || mRect[i].height < MIN_TU_SIZE) && mRect[i].width!=0)
    {
      const UInt numPels=mRect[i].width * mRect[i].height;
      if (numPels < (MIN_TU_SIZE*MIN_TU_SIZE))
      {
        // this level doesn't have enough pixels to have 4 blocks of any relative dimension
#if INTRA_NSP 
#if INTER_NSP 
        if (mTrDepthRelCU[i] > 0)
        {
          mTrDepthRelCU[i]--;
        }
#else
        mTrDepthRelCU[i]--;
#endif
        if(mTrDepthRelCU[i])
        {
        mRect[i].width = parent.mRect[i].width;
        mRect[i].height= parent.mRect[i].height;        
        }
        else
        {
          mRect[i].width = MIN_TU_SIZE;
          mRect[i].height= MIN_TU_SIZE;        
        }        
        mCodeAll[i]=false; // go up a level, so only process one entry of a quadrant 
#else
        mRect[i].width = parent.mRect[i].width;
        mRect[i].height= parent.mRect[i].height;
        mCodeAll[i]=false; // go up a level, so only process one entry of a quadrant  
        mTrDepthRelCU[i]--;
#endif 
            
      }
      else if (mRect[i].width < mRect[i].height)
      {
        mRect[i].width=MIN_TU_SIZE;
        mRect[i].height=numPels/MIN_TU_SIZE;
        mCodeAll[i]=true;
      }
      else
      {
        mRect[i].height=MIN_TU_SIZE;
        mRect[i].width=numPels/MIN_TU_SIZE;
        mCodeAll[i]=true;
      }
    }
    else
    {
      mCodeAll[i]=true;
    }

    mOrigWidth[i]=mRect[i].width;
#if INTRA_NSP ==0
    if (!mCodeAll[i] && mbProcessLastOfLevel )
    {
      mRect[i].width=0;
    }
#endif 
  }
}

Bool TComTURecurse::nextSection(const TComTU &parent)
{
  if (mSplitMode==DONT_SPLIT)
  {
    mSection++;
    return false;
  }
  else
  {
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      mOffsets[i]+=mRect[i].width*mRect[i].height;
      if (mbProcessLastOfLevel)
      {
        mRect[i].width=mOrigWidth[i];
      }
      mRect[i].x0+=mRect[i].width;
      const TComRectangle &parentRect=parent.getRect(ComponentID(i));
      if (mRect[i].x0 >= parentRect.x0+parentRect.width)
      {
        mRect[i].x0=parentRect.x0;
        mRect[i].y0+=mRect[i].height;
      }
      if (!mCodeAll[i])
      {
#if INTRA_NSP
        if (1)
#else
        if (!mbProcessLastOfLevel || mSection!=2)
#endif 
        {
          mRect[i].width=0;
        }
      }
    }
    assert(mRect[COMPONENT_Cb].x0==mRect[COMPONENT_Cr].x0);
    assert(mRect[COMPONENT_Cb].y0==mRect[COMPONENT_Cr].y0);
    assert(mRect[COMPONENT_Cb].width==mRect[COMPONENT_Cr].width);
    assert(mRect[COMPONENT_Cb].height==mRect[COMPONENT_Cr].height);
   
#if INTRA_NSP
    mSection++;
    UInt uiInferSplit = ((NUMBER_OF_SECTIONS[mSplitMode]==2) && (mSection == 1))? 1 : 0;        
    mAbsPartIdxTURelCU= parent.GetAbsStartPartIdxRelNumParts(parent.GetSectionNumber()) + mAbsPartIdxStep[mSection + uiInferSplit];                
#else
    mAbsPartIdxTURelCU+=mAbsPartIdxStep;
    mSection++;
#endif 
     
#if INTRA_NSP
    return mSection< NUMBER_OF_SECTIONS[mSplitMode];
#else
    return mSection< (1<<mSplitMode);
#endif 
  }
}

#if INTRA_NSP
Void TComTURecurse::nextSectionWithPU(const TComTU &parent, UInt uiPUIdx, UInt uiLastPU,const ChannelType chType)
{
  mSection = uiPUIdx;
 {
  mAbsPartIdxTURelCU= parent.GetAbsStartPartIdxRelNumParts(parent.GetSectionNumber()) + mAbsPartIdxStep[uiPUIdx];  

  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {
    mOffsets[i]+=mRect[i].width*mRect[i].height;

#if INTER_NSP
    if (mpcCU->isInter(GetAbsPartIdxTU()))
    {
      const UInt csx=getComponentScaleX(ComponentID(i), mChromaFormat);

      mRect[i].x0+=mRect[i].width;
      const TComRectangle &parentRect=parent.getRect(ComponentID(i));
      if (mRect[i].x0 >= parentRect.x0+(mpcCU->getWidth(GetAbsPartIdxTU()) >> csx))
      {
        mRect[i].x0=parentRect.x0;
        mRect[i].y0+=mRect[i].height;
      }
    }
#endif

    if (mbProcessLastOfLevel)
    {
      mRect[i].width=mOrigWidth[i];
    }
    if (!mCodeAll[i])
    {
      if(chType == CHANNEL_TYPE_CHROMA)
        mAbsPartIdxTURelCU = 0; //PU being clubbed        
      if (!mbProcessLastOfLevel || !uiLastPU)
      {
        mRect[i].width=0;        
      }
      else
      {
        mCodeAll[i]=true;
      }
    }
  }    
 }
}    
#endif 

UInt TComTU::GetEquivalentLog2TrSize(const ComponentID compID)     const
{
#if INTRA_NSP
  return g_aucConvertToBit[ min(getRect(compID).width,getRect(compID).height) ] + 2;
#else
  return g_aucConvertToBit[ getRect(compID).height ] + 2;
#endif 
}


Bool TComTU::useDST(const ComponentID compID)
{
        TComDataCU *const pcCU       = getCU();
  const UInt              absPartIdx = GetAbsPartIdxTU(compID);

  return isLuma(compID) && pcCU->isIntra(absPartIdx);
}


Bool TComTU::isNonTransformedResidualRotated(const ComponentID compID)
{
  // rotation only for 4x4 intra, and is only used for non-transformed blocks (the latter is not checked here)
  return    getCU()->getSlice()->getSPS()->getSpsRangeExtension().getTransformSkipRotationEnabledFlag()
         && mRect[compID].width == 4
         && getCU()->isIntra(GetAbsPartIdxTU());
}


UInt TComTU::getGolombRiceStatisticsIndex(const ComponentID compID)
{
        TComDataCU *const pcCU             = getCU();
  const UInt              absPartIdx       = GetAbsPartIdxTU(compID);
  const Bool              transformSkip    = pcCU->getTransformSkip(absPartIdx, compID);
  const Bool              transquantBypass = pcCU->getCUTransquantBypass(absPartIdx);

  //--------

  const UInt channelTypeOffset    =  isChroma(compID)                   ? 2 : 0;
  const UInt nonTransformedOffset = (transformSkip || transquantBypass) ? 1 : 0;

  //--------

  const UInt selectedIndex = channelTypeOffset + nonTransformedOffset;
  assert(selectedIndex < RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS);

  return selectedIndex;
}

#if INTRA_NSP
Void TComTU::deriveQPartInfo( const ComponentID    absPartIdxSourceComponent)
{
  TComDataCU *const pcCU      = getCU();
  UInt uiCurrPartWidth        = mRect[COMPONENT_Y].width;
  UInt uiCurrPartHeight       = mRect[COMPONENT_Y].height;
  if ( absPartIdxSourceComponent == COMPONENT_Y)
  {
    mAbsPartIdxStep[4]          = uiCurrPartWidth / pcCU->getPic()->getMinCUWidth() * uiCurrPartHeight / pcCU->getPic()->getMinCUHeight() ;
  }
  else
  {
    mAbsPartIdxStep[4]          = std::max<UInt>(1, (mpParent->GetAbsPartIdxNumParts(absPartIdxSourceComponent) >> partIdxStepShift[mSplitMode]));
  }

  if(mbLoopFilterModeSet == 1)
  {
    mAbsPartIdxStep[0] = 0;    
    mAbsPartIdxStep[1] = mAbsPartIdxStep[0]  + g_auiRasterToZscan[uiCurrPartWidth / pcCU->getPic()->getMinCUWidth()];
    mAbsPartIdxStep[2] = g_auiRasterToZscan[g_auiZscanToRaster[mAbsPartIdxStep[0]]  + (uiCurrPartHeight / pcCU->getPic()->getMinCUHeight())*pcCU->getPic()->getNumPartInCtuWidth()];
    mAbsPartIdxStep[3] = mAbsPartIdxStep[2]  + g_auiRasterToZscan[uiCurrPartWidth / pcCU->getPic()->getMinCUWidth()];       
    return;
  }

  PartSize eTUPartSize = pcCU->getPartitionSize(mAbsPartIdxCU);
#if INTER_NSP
  if (pcCU->isInter(mAbsPartIdxCU) && !pcCU->getInterNstFlag(mAbsPartIdxCU))
#else
  if (pcCU->isInter(mAbsPartIdxCU))
#endif
  {
    eTUPartSize = SIZE_2Nx2N;
  }
  switch ( eTUPartSize)
  {
  case SIZE_2Nx2N:         
    mAbsPartIdxStep[0] = 0;    
    mAbsPartIdxStep[1] = mAbsPartIdxStep[0]  + g_auiRasterToZscan[uiCurrPartWidth / pcCU->getPic()->getMinCUWidth()];
    mAbsPartIdxStep[2] = g_auiRasterToZscan[g_auiZscanToRaster[mAbsPartIdxStep[0]]  + (uiCurrPartHeight / pcCU->getPic()->getMinCUHeight())*pcCU->getPic()->getNumPartInCtuWidth()];
    mAbsPartIdxStep[3] = mAbsPartIdxStep[2]  + g_auiRasterToZscan[uiCurrPartWidth / pcCU->getPic()->getMinCUWidth()];        
    break;
  case SIZE_2NxN:      
    mAbsPartIdxStep[0] = 0;
    mAbsPartIdxStep[1] = mAbsPartIdxStep[0]  + g_auiRasterToZscan[(uiCurrPartWidth) / pcCU->getPic()->getMinCUWidth()];
    mAbsPartIdxStep[2] = g_auiRasterToZscan[g_auiZscanToRaster[mAbsPartIdxStep[0]]  + (uiCurrPartHeight / pcCU->getPic()->getMinCUHeight())*pcCU->getPic()->getNumPartInCtuWidth()];    
    mAbsPartIdxStep[3] = mAbsPartIdxStep[2]  + g_auiRasterToZscan[(uiCurrPartWidth) / pcCU->getPic()->getMinCUWidth()];        
    break;
  case SIZE_Nx2N:  
    mAbsPartIdxStep[0] =  0;
    mAbsPartIdxStep[1] = mAbsPartIdxStep[0]  + g_auiRasterToZscan[uiCurrPartWidth / pcCU->getPic()->getMinCUWidth()];
    mAbsPartIdxStep[2] = g_auiRasterToZscan[g_auiZscanToRaster[mAbsPartIdxStep[0]]  + ((uiCurrPartHeight) / pcCU->getPic()->getMinCUHeight())*pcCU->getPic()->getNumPartInCtuWidth()];
    mAbsPartIdxStep[3] = g_auiRasterToZscan[g_auiZscanToRaster[mAbsPartIdxStep[1]]  + ((uiCurrPartHeight) / pcCU->getPic()->getMinCUHeight())*pcCU->getPic()->getNumPartInCtuWidth()];
    mAbsPartIdxStep[4] = uiCurrPartWidth / pcCU->getPic()->getMinCUWidth() * (uiCurrPartHeight>>1) / pcCU->getPic()->getMinCUHeight() ;    
    break;
  case SIZE_NxN:   
    mAbsPartIdxStep[0] = 0;    
    mAbsPartIdxStep[1] = mAbsPartIdxStep[0]  + g_auiRasterToZscan[uiCurrPartWidth / pcCU->getPic()->getMinCUWidth()];
    mAbsPartIdxStep[2] = g_auiRasterToZscan[g_auiZscanToRaster[mAbsPartIdxStep[0]]  + (uiCurrPartHeight /  pcCU->getPic()->getMinCUHeight())* pcCU->getPic()->getNumPartInCtuWidth()];
    mAbsPartIdxStep[3] = mAbsPartIdxStep[2]  + g_auiRasterToZscan[uiCurrPartWidth / pcCU->getPic()->getMinCUWidth()];    
    mAbsPartIdxStep[4] =(pcCU->getPic()->getNumPartitionsInCtu() >> (pcCU->getDepth(0)<<1));    
    break;
  default:
    assert (0);
    break;
  }
}

Void TComDataCU::getSize( UInt uiAbsPartIdx, Int& riWidth, Int& riHeight, UInt& uiRoiNumPartition, Bool bLoopFilterModeSet )
{  
  if(bLoopFilterModeSet == 1)
  {
     riWidth = getWidth(uiAbsPartIdx);      riHeight = getHeight(uiAbsPartIdx); 
     return;
  }

  PartSize eTUPartSize = getPartitionSize(uiAbsPartIdx);
#if INTER_NSP
  if (isInter(uiAbsPartIdx) && !getInterNstFlag(uiAbsPartIdx))
#else
  if (isInter(uiAbsPartIdx))
#endif
  {
    eTUPartSize = SIZE_2Nx2N;
  }
  switch ( eTUPartSize)
  {
     case SIZE_2NxN:
      riWidth = getWidth(uiAbsPartIdx);      riHeight = getHeight(uiAbsPartIdx) >> 1; 
      break;
    case SIZE_Nx2N:
      riWidth = getWidth(uiAbsPartIdx) >> 1; riHeight = getHeight(uiAbsPartIdx); 
      break;
    case SIZE_NxN:
      riWidth = getWidth(uiAbsPartIdx) >> 1; riHeight = getHeight(uiAbsPartIdx) >> 1;
      break;
    default:
      assert ( eTUPartSize == SIZE_2Nx2N );
      riWidth = getWidth(uiAbsPartIdx);      riHeight = getHeight(uiAbsPartIdx); 
      break;
  }
     uiRoiNumPartition = riWidth / getPic()->getMinCUWidth() * riHeight / getPic()->getMinCUHeight() ;
}

#if INTER_NSP
Bool TComTU::tryInterNst(TComDataCU *pcCU, UInt uiAbsPartIdx)
{
  Int blockSize = max (mRect[COMPONENT_Y].width, mRect[COMPONENT_Y].height);

  if (blockSize == pcCU->getWidth(uiAbsPartIdx) 
    && pcCU->isInter(uiAbsPartIdx) && (pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2NxN || pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_Nx2N ) 
    && pcCU->getWidth(uiAbsPartIdx) <= (1<<pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize()))
  {
    return true;
  }
  else
  {
    return false;
  }
}
#endif

#endif 
