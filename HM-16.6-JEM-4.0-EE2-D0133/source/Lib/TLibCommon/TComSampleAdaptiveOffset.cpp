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

/** \file     TComSampleAdaptiveOffset.cpp
    \brief    sample adaptive offset class
*/

#include "TComSampleAdaptiveOffset.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//! \ingroup TLibCommon
//! \{

SAOOffset::SAOOffset()
{
  reset();
}

SAOOffset::~SAOOffset()
{

}
#if SAO_PEAK
Void TComSampleAdaptiveOffset::initMatrix_UInt(UInt ***m2D, Int d1, Int d2)
{
  Int i;
  
  if(!(*m2D = (UInt **) calloc(d1, sizeof(UInt *))))
    FATAL_ERROR_0("initMatrix_double: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (UInt *) calloc(d1 * d2, sizeof(UInt))))
    FATAL_ERROR_0("initMatrix_double: memory allocation problem\n", -1);
  
  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}
Void TComSampleAdaptiveOffset::initMatrix_UChar(UChar ***m2D, Int d1, Int d2)
{
  Int i;
  
  if(!(*m2D = (UChar **) calloc(d1, sizeof(UChar *))))
    FATAL_ERROR_0("initMatrix_double: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (UChar *) calloc(d1 * d2, sizeof(UChar))))
    FATAL_ERROR_0("initMatrix_double: memory allocation problem\n", -1);
  
  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}
Void TComSampleAdaptiveOffset::initMatrix_Double(Double ***m2D, Int d1, Int d2)
{
  Int i;
  
  if(!(*m2D = (Double **) calloc(d1, sizeof(Double *))))
    FATAL_ERROR_0("initMatrix_double: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (Double *) calloc(d1 * d2, sizeof(Double))))
    FATAL_ERROR_0("initMatrix_double: memory allocation problem\n", -1);
  
  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

Void TComSampleAdaptiveOffset::initMatrix3D_UInt(UInt ****m3D, Int d1, Int d2, Int d3)
{
  Int  i, j;
  
  (*m3D) = (UInt ***) malloc(d1*d2*d3*sizeof(UInt) + d1*d2*sizeof(UInt*) + d1*sizeof(UInt**));
  for(i = 0; i < d1; i ++)
  {
    (*m3D)[i] = (UInt **)( (Char*) (*m3D) + d1*sizeof(UInt *) + i * d2*sizeof(UInt*) );
    for(j = 0; j < d2; j++)
    {
      (*m3D)[i][j] = (UInt *) ((Char*)(*m3D)+ d1 *sizeof(UInt *) + d1*d2*sizeof(UInt*) + (i*d2*d3+ j*d3)*sizeof(UInt)); 
    }
  }
}
Void TComSampleAdaptiveOffset::initMatrix3DContiguous_UChar(UChar ****m3D, Int d1, Int d2, Int d3)
{
  Int  i, j;
  
  (*m3D) = (UChar ***) malloc(d1*d2*d3*sizeof(UChar) + d1*d2*sizeof(UChar*) + d1*sizeof(UChar**));
  for(i = 0; i < d1; i ++)
  {
    (*m3D)[i] = (UChar **)( (Char*) (*m3D) + d1*sizeof(UChar *) + i * d2*sizeof(UChar*) );
    for(j = 0; j < d2; j++)
    {
      (*m3D)[i][j] = (UChar *) ((Char*)(*m3D)+ d1 *sizeof(UChar *) + d1*d2*sizeof(UChar*) + (i*d2*d3+ j*d3)*sizeof(UChar)); 
    }
  }
}
Void TComSampleAdaptiveOffset::initMatrix3D_Double(Double ****m3D, Int d1, Int d2, Int d3)
{
  Int  j;
  
  if(!((*m3D) = (Double ***) calloc(d1, sizeof(Double **))))
    FATAL_ERROR_0("initMatrix3D_double: memory allocation problem\n", -1);
  
  for(j = 0; j < d1; j++)
    initMatrix_Double((*m3D) + j, d2, d3);
}
Void TComSampleAdaptiveOffset::initMatrix3D_UChar(UChar ****m3D, Int d1, Int d2, Int d3)
{
  Int  j;
  
  if(!((*m3D) = (UChar ***) calloc(d1, sizeof(UChar **))))
    FATAL_ERROR_0("initMatrix4D_double: memory allocation problem\n", -1);
  
  for(j = 0; j < d1; j++)
    initMatrix_UChar((*m3D) + j, d2, d3);
}
Void TComSampleAdaptiveOffset::initMatrix4D_UChar(UChar *****m4D, Int d1, Int d2, Int d3, Int d4)
{
  Int  j;
  
  if(!((*m4D) = (UChar ****) calloc(d1, sizeof(UChar ***))))
    FATAL_ERROR_0("initMatrix4D_double: memory allocation problem\n", -1);
  
  for(j = 0; j < d1; j++)
    initMatrix3D_UChar((*m4D) + j, d2, d3, d4);
}

Void TComSampleAdaptiveOffset::initMatrix4D_Double(Double *****m4D, Int d1, Int d2, Int d3, Int d4)
{
  Int  j;
  
  if(!((*m4D) = (Double ****) calloc(d1, sizeof(Double ***))))
    FATAL_ERROR_0("initMatrix4D_double: memory allocation problem\n", -1);
  
  for(j = 0; j < d1; j++)
    initMatrix3D_Double((*m4D) + j, d2, d3, d4);
}

Void TComSampleAdaptiveOffset::initMatrix5D_UChar(UChar ******m5D, Int d1, Int d2, Int d3, Int d4, Int d5)
{
  Int  j;
  
  if(!((*m5D) = (UChar *****) calloc(d1, sizeof(UChar ****))))
    FATAL_ERROR_0("initMatrix4D_double: memory allocation problem\n", -1);
  
  for(j = 0; j < d1; j++)
    initMatrix4D_UChar((*m5D) + j, d2, d3, d4, d5);
}
Void TComSampleAdaptiveOffset::initMatrix5D_Double(Double ******m5D, Int d1, Int d2, Int d3, Int d4, Int d5)
{
  Int  j;
  
  if(!((*m5D) = (Double *****) calloc(d1, sizeof(Double ****))))
    FATAL_ERROR_0("initMatrix4D_double: memory allocation problem\n", -1);
  
  for(j = 0; j < d1; j++)
    initMatrix4D_Double((*m5D) + j, d2, d3, d4, d5);
}
Void TComSampleAdaptiveOffset::destroyMatrix_UInt(UInt **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_double: memory free problem\n", -1);
    free(m2D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix_double: memory free problem\n", -1);
  }
}
Void TComSampleAdaptiveOffset::destroyMatrix_UChar(UChar **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_double: memory free problem\n", -1);
    free(m2D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix_double: memory free problem\n", -1);
  }
}
Void TComSampleAdaptiveOffset::destroyMatrix5D_UChar(UChar *****m3D, Int d1, Int d2, Int d3)
{
  Int i;
  
  if(m3D)
  {
    for(i = 0; i < d1; i++)
      destroyMatrix4D_UChar(m3D[i], d2, d3);
    free(m3D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix3D_double: memory free problem\n", -1);
  }
}
Void TComSampleAdaptiveOffset::destroyMatrix4D_UChar(UChar ****m3D, Int d1, Int d2)
{
  Int i;
  
  if(m3D)
  {
    for(i = 0; i < d1; i++)
      destroyMatrix3D_UChar(m3D[i], d2);
    free(m3D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix3D_double: memory free problem\n", -1);
  }
}
Void TComSampleAdaptiveOffset::destroyMatrix3D_UChar(UChar ***m3D, Int d1)
{
  Int i;
  
  if(m3D)
  {
    for(i = 0; i < d1; i++)
      destroyMatrix_UChar(m3D[i]);
    free(m3D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix3D_double: memory free problem\n", -1);
  }
}
Void TComSampleAdaptiveOffset::destroyMatrix3D_UInt(UInt ***m3D, Int d1)
{
  if(m3D)
  {
    free(m3D);
  } 
}
Void TComSampleAdaptiveOffset::destroyMatrix_Double(Double **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_double: memory free problem\n", -1);
    free(m2D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix_double: memory free problem\n", -1);
  }
}

Void TComSampleAdaptiveOffset::destroyMatrix3D_Double(Double ***m3D, Int d1)
{
  Int i;
  
  if(m3D)
  {
    for(i = 0; i < d1; i++)
      destroyMatrix_Double(m3D[i]);
    free(m3D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix3D_double: memory free problem\n", -1);
  }
}


Void TComSampleAdaptiveOffset::destroyMatrix4D_Double(Double ****m4D, Int d1, Int d2)
{
  Int  j;
  
  if(m4D)
  {
    for(j = 0; j < d1; j++)
    {
      destroyMatrix3D_Double(m4D[j], d2);
    }
    free(m4D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix4D_double: memory free problem\n", -1);
  }
}

Void TComSampleAdaptiveOffset::destroyMatrix5D_Double(Double *****m5D, Int d1, Int d2, Int d3)
{
  Int  j;
  
  if(m5D)
  {
    for(j = 0; j < d1; j++)
    {
      destroyMatrix4D_Double(m5D[j], d2, d3);
    }
    free(m5D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix4D_double: memory free problem\n", -1);
  }
}
#endif
Void SAOOffset::reset()
{
  modeIdc = SAO_MODE_OFF;
  typeIdc = -1;
  typeAuxInfo = -1;
  ::memset(offset, 0, sizeof(Int)* MAX_NUM_SAO_CLASSES);
}

const SAOOffset& SAOOffset::operator= (const SAOOffset& src)
{
  modeIdc = src.modeIdc;
  typeIdc = src.typeIdc;
  typeAuxInfo = src.typeAuxInfo;
  ::memcpy(offset, src.offset, sizeof(Int)* MAX_NUM_SAO_CLASSES);

  return *this;
}


SAOBlkParam::SAOBlkParam()
{
  reset();
}

SAOBlkParam::~SAOBlkParam()
{

}

Void SAOBlkParam::reset()
{
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx].reset();
  }
}

const SAOBlkParam& SAOBlkParam::operator= (const SAOBlkParam& src)
{
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx] = src.offsetParam[compIdx];
  }
  return *this;

}

TComSampleAdaptiveOffset::TComSampleAdaptiveOffset()
{
  m_tempPicYuv = NULL;
  m_lineBufWidth = 0;
  m_signLineBuf1 = NULL;
  m_signLineBuf2 = NULL;
#if SAO_PEAK
  saoStat = NULL;
  offsetErr = NULL;
  diffError = NULL;  
#endif
}


TComSampleAdaptiveOffset::~TComSampleAdaptiveOffset()
{
  destroy();

  if (m_signLineBuf1)
  {
    delete[] m_signLineBuf1;
    m_signLineBuf1 = NULL;
  }
  if (m_signLineBuf2)
  {
    delete[] m_signLineBuf2;
    m_signLineBuf2 = NULL;
  }
}

Void TComSampleAdaptiveOffset::create( Int picWidth, Int picHeight, ChromaFormat format, UInt maxCUWidth, UInt maxCUHeight, UInt maxCUDepth, UInt lumaBitShift, UInt chromaBitShift )
{
  destroy();

  m_picWidth        = picWidth;
  m_picHeight       = picHeight;
  m_chromaFormatIDC = format;
#if JVET_C0024_QTBT
  assert(maxCUWidth == maxCUHeight);
  m_CTUSize = maxCUWidth;
  m_numCTUInWidth   = (m_picWidth/m_CTUSize) + ((m_picWidth % m_CTUSize)?1:0);
  m_numCTUInHeight  = (m_picHeight/m_CTUSize) + ((m_picHeight % m_CTUSize)?1:0);
#else
  m_maxCUWidth      = maxCUWidth;
  m_maxCUHeight     = maxCUHeight;

  m_numCTUInWidth   = (m_picWidth/m_maxCUWidth) + ((m_picWidth % m_maxCUWidth)?1:0);
  m_numCTUInHeight  = (m_picHeight/m_maxCUHeight) + ((m_picHeight % m_maxCUHeight)?1:0);
#endif
  m_numCTUsPic      = m_numCTUInHeight*m_numCTUInWidth;

  //temporary picture buffer
  if ( !m_tempPicYuv )
  {
    m_tempPicYuv = new TComPicYuv;
#if JVET_C0024_QTBT
    m_tempPicYuv->create( m_picWidth, m_picHeight, m_chromaFormatIDC, m_CTUSize, m_CTUSize, maxCUDepth, true );
#else
    m_tempPicYuv->create( m_picWidth, m_picHeight, m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, maxCUDepth, true );
#endif
  }

  //bit-depth related
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_offsetStepLog2  [compIdx] = isLuma(ComponentID(compIdx))? lumaBitShift : chromaBitShift;
  }
}

Void TComSampleAdaptiveOffset::destroy()
{
  if ( m_tempPicYuv )
  {
    m_tempPicYuv->destroy();
    delete m_tempPicYuv;
    m_tempPicYuv = NULL;
  }
}

Void TComSampleAdaptiveOffset::invertQuantOffsets(ComponentID compIdx, Int typeIdc, Int typeAuxInfo, Int* dstOffsets, Int* srcOffsets)
{
  Int codedOffset[MAX_NUM_SAO_CLASSES];

  ::memcpy(codedOffset, srcOffsets, sizeof(Int)*MAX_NUM_SAO_CLASSES);
  ::memset(dstOffsets, 0, sizeof(Int)*MAX_NUM_SAO_CLASSES);

  if(typeIdc == SAO_TYPE_START_BO)
  {
    for(Int i=0; i< 4; i++)
    {
      dstOffsets[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES] = codedOffset[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES]*(1<<m_offsetStepLog2[compIdx]);
    }
  }
  else //EO
  {
    for(Int i=0; i< NUM_SAO_EO_CLASSES; i++)
    {
      dstOffsets[i] = codedOffset[i] *(1<<m_offsetStepLog2[compIdx]);
    }
    assert(dstOffsets[SAO_CLASS_EO_PLAIN] == 0); //keep EO plain offset as zero
  }

}

Int TComSampleAdaptiveOffset::getMergeList(TComPic* pic, Int ctuRsAddr, SAOBlkParam* blkParams, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  Int ctuX = ctuRsAddr % m_numCTUInWidth;
  Int ctuY = ctuRsAddr / m_numCTUInWidth;
  Int mergedCTUPos;
  Int numValidMergeCandidates = 0;

  for(Int mergeType=0; mergeType< NUM_SAO_MERGE_TYPES; mergeType++)
  {
    SAOBlkParam* mergeCandidate = NULL;

    switch(mergeType)
    {
    case SAO_MERGE_ABOVE:
      {
        if(ctuY > 0)
        {
          mergedCTUPos = ctuRsAddr- m_numCTUInWidth;
          if( pic->getSAOMergeAvailability(ctuRsAddr, mergedCTUPos) )
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);
          }
        }
      }
      break;
    case SAO_MERGE_LEFT:
      {
        if(ctuX > 0)
        {
          mergedCTUPos = ctuRsAddr- 1;
          if( pic->getSAOMergeAvailability(ctuRsAddr, mergedCTUPos) )
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);
          }
        }
      }
      break;
    default:
      {
        printf("not a supported merge type");
        assert(0);
        exit(-1);
      }
    }

    mergeList[mergeType]=mergeCandidate;
    if (mergeCandidate != NULL)
    {
      numValidMergeCandidates++;
    }
  }

  return numValidMergeCandidates;
}


Void TComSampleAdaptiveOffset::reconstructBlkSAOParam(SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& offsetParam = recParam[component];

    if(offsetParam.modeIdc == SAO_MODE_OFF)
    {
      continue;
    }

    switch(offsetParam.modeIdc)
    {
    case SAO_MODE_NEW:
      {
        invertQuantOffsets(component, offsetParam.typeIdc, offsetParam.typeAuxInfo, offsetParam.offset, offsetParam.offset);
      }
      break;
    case SAO_MODE_MERGE:
      {
        SAOBlkParam* mergeTarget = mergeList[offsetParam.typeIdc];
        assert(mergeTarget != NULL);

        offsetParam = (*mergeTarget)[component];
      }
      break;
    default:
      {
        printf("Not a supported mode");
        assert(0);
        exit(-1);
      }
    }
  }
}

Void TComSampleAdaptiveOffset::reconstructBlkSAOParams(TComPic* pic, SAOBlkParam* saoBlkParams)
{
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_picSAOEnabled[compIdx] = false;
  }

  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);

  for(Int ctuRsAddr=0; ctuRsAddr< m_numCTUsPic; ctuRsAddr++)
  {
    SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] = { NULL };
    getMergeList(pic, ctuRsAddr, saoBlkParams, mergeList);

    reconstructBlkSAOParam(saoBlkParams[ctuRsAddr], mergeList);

    for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      if(saoBlkParams[ctuRsAddr][compIdx].modeIdc != SAO_MODE_OFF)
      {
        m_picSAOEnabled[compIdx] = true;
      }
    }
  }
}


Void TComSampleAdaptiveOffset::offsetBlock(const Int channelBitDepth, Int typeIdx, Int* offset
                                          , Pel* srcBlk, Pel* resBlk, Int srcStride, Int resStride,  Int width, Int height
                                          , Bool isLeftAvail,  Bool isRightAvail, Bool isAboveAvail, Bool isBelowAvail, Bool isAboveLeftAvail, Bool isAboveRightAvail, Bool isBelowLeftAvail, Bool isBelowRightAvail
#if JVET_D0033_ADAPTIVE_CLIPPING
                                           , ComponentID compID
#endif
                                          )
{
#if JVET_C0024_QTBT
  if(m_lineBufWidth != m_CTUSize)
  {
    m_lineBufWidth = m_CTUSize;
#else
  if(m_lineBufWidth != m_maxCUWidth)
  {
    m_lineBufWidth = m_maxCUWidth;
#endif

    if (m_signLineBuf1)
    {
      delete[] m_signLineBuf1;
      m_signLineBuf1 = NULL;
    }
    m_signLineBuf1 = new Char[m_lineBufWidth+1];

    if (m_signLineBuf2)
    {
      delete[] m_signLineBuf2;
      m_signLineBuf2 = NULL;
    }
    m_signLineBuf2 = new Char[m_lineBufWidth+1];
  }

#if !JVET_D0033_ADAPTIVE_CLIPPING
  const Int maxSampleValueIncl = (1<< channelBitDepth )-1;
#endif

  Int x,y, startX, startY, endX, endY, edgeType;
  Int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  Char signLeft, signRight, signDown;

  Pel* srcLine = srcBlk;
  Pel* resLine = resBlk;

  switch(typeIdx)
  {
  case SAO_TYPE_EO_0:
    {
      offset += 2;
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);
      for (y=0; y< height; y++)
      {
        signLeft = (Char)sgn(srcLine[startX] - srcLine[startX-1]);
        for (x=startX; x< endX; x++)
        {
          signRight = (Char)sgn(srcLine[x] - srcLine[x+1]); 
          edgeType =  signRight + signLeft;
          signLeft  = -signRight;

#if JVET_D0033_ADAPTIVE_CLIPPING
                resLine[x] = ClipA(srcLine[x] + offset[edgeType],compID);
#else
          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);
#endif
        }
        srcLine  += srcStride;
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_90:
    {
      offset += 2;
      Char *signUpLine = m_signLineBuf1;

      startY = isAboveAvail ? 0 : 1;
      endY   = isBelowAvail ? height : height-1;
      if (!isAboveAvail)
      {
        srcLine += srcStride;
        resLine += resStride;
      }

      Pel* srcLineAbove= srcLine- srcStride;
      for (x=0; x< width; x++)
      {
        signUpLine[x] = (Char)sgn(srcLine[x] - srcLineAbove[x]);
      }

      Pel* srcLineBelow;
      for (y=startY; y<endY; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for (x=0; x< width; x++)
        {
          signDown  = (Char)sgn(srcLine[x] - srcLineBelow[x]);
          edgeType = signDown + signUpLine[x];
          signUpLine[x]= -signDown;

#if JVET_D0033_ADAPTIVE_CLIPPING
                resLine[x] = ClipA(srcLine[x] + offset[edgeType],compID);
#else
          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);
#endif
        }
        srcLine += srcStride;
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_135:
    {
      offset += 2;
      Char *signUpLine, *signDownLine, *signTmpLine;

      signUpLine  = m_signLineBuf1;
      signDownLine= m_signLineBuf2;

      startX = isLeftAvail ? 0 : 1 ;
      endX   = isRightAvail ? width : (width-1);

      //prepare 2nd line's upper sign
      Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX; x< endX+1; x++)
      {
        signUpLine[x] = (Char)sgn(srcLineBelow[x] - srcLine[x- 1]);
      }

      //1st line
      Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveLeftAvail ? 0 : 1;
      firstLineEndX   = isAboveAvail? endX: 1;
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        edgeType  =  sgn(srcLine[x] - srcLineAbove[x- 1]) - signUpLine[x+1];

#if JVET_D0033_ADAPTIVE_CLIPPING
            resLine[x] = ClipA(srcLine[x] + offset[edgeType],compID);
#else
        resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);
#endif
      }
      srcLine  += srcStride;
      resLine  += resStride;


      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for (x=startX; x<endX; x++)
        {
          signDown =  (Char)sgn(srcLine[x] - srcLineBelow[x+ 1]);
          edgeType =  signDown + signUpLine[x];
#if JVET_D0033_ADAPTIVE_CLIPPING
                resLine[x] = ClipA(srcLine[x] + offset[edgeType],compID);
#else
          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);

#endif
          signDownLine[x+1] = -signDown;
        }
        signDownLine[startX] = (Char)sgn(srcLineBelow[startX] - srcLine[startX-1]);

        signTmpLine  = signUpLine;
        signUpLine   = signDownLine;
        signDownLine = signTmpLine;

        srcLine += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowAvail ? startX : (width -1);
      lastLineEndX   = isBelowRightAvail ? width : (width -1);
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        edgeType =  sgn(srcLine[x] - srcLineBelow[x+ 1]) + signUpLine[x];
#if JVET_D0033_ADAPTIVE_CLIPPING
            resLine[x] = ClipA(srcLine[x] + offset[edgeType],compID);
#else
        resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);

#endif
      }
    }
    break;
  case SAO_TYPE_EO_45:
    {
      offset += 2;
      Char *signUpLine = m_signLineBuf1+1;

      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);

      //prepare 2nd line upper sign
      Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX-1; x< endX; x++)
      {
        signUpLine[x] = (Char)sgn(srcLineBelow[x] - srcLine[x+1]);
      }


      //first line
      Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveAvail ? startX : (width -1 );
      firstLineEndX   = isAboveRightAvail ? width : (width-1);
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        edgeType = sgn(srcLine[x] - srcLineAbove[x+1]) -signUpLine[x-1];
#if JVET_D0033_ADAPTIVE_CLIPPING
            resLine[x] = ClipA(srcLine[x] + offset[edgeType],compID);
#else
        resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);
#endif
      }
      srcLine += srcStride;
      resLine += resStride;

      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for(x= startX; x< endX; x++)
        {
          signDown =  (Char)sgn(srcLine[x] - srcLineBelow[x-1]);
          edgeType =  signDown + signUpLine[x];
#if JVET_D0033_ADAPTIVE_CLIPPING
                resLine[x] = ClipA(srcLine[x] + offset[edgeType],compID);
#else
          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);
#endif
          signUpLine[x-1] = -signDown;
        }
        signUpLine[endX-1] = (Char)sgn(srcLineBelow[endX-1] - srcLine[endX]);
        srcLine  += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        edgeType = sgn(srcLine[x] - srcLineBelow[x-1]) + signUpLine[x];
#if JVET_D0033_ADAPTIVE_CLIPPING
            resLine[x] = ClipA(srcLine[x] + offset[edgeType],compID);
#else
        resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);

#endif
      }
    }
    break;
  case SAO_TYPE_BO:
    {
      const Int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
      for (y=0; y< height; y++)
      {
        for (x=0; x< width; x++)
        {
#if JVET_D0033_ADAPTIVE_CLIPPING
                resLine[x] = ClipA(srcLine[x] + offset[srcLine[x] >> shiftBits],compID);
#else
          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[srcLine[x] >> shiftBits] );
#endif
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
    break;
  default:
    {
      printf("Not a supported SAO types\n");
      assert(0);
      exit(-1);
    }
  }
}

Void TComSampleAdaptiveOffset::offsetCTU(Int ctuRsAddr, TComPicYuv* srcYuv, TComPicYuv* resYuv, SAOBlkParam& saoblkParam, TComPic* pPic)
{
  Bool isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail;

  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  Bool bAllOff=true;
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (saoblkParam[compIdx].modeIdc != SAO_MODE_OFF)
    {
      bAllOff=false;
    }
  }
  if (bAllOff)
  {
    return;
  }

  //block boundary availability
  pPic->getPicSym()->deriveLoopFilterBoundaryAvailibility(ctuRsAddr, isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail);

#if JVET_C0024_QTBT
  Int yPos   = (ctuRsAddr / m_numCTUInWidth)*m_CTUSize;
  Int xPos   = (ctuRsAddr % m_numCTUInWidth)*m_CTUSize;
  Int height = (yPos + m_CTUSize > m_picHeight)?(m_picHeight- yPos):m_CTUSize;
  Int width  = (xPos + m_CTUSize  > m_picWidth )?(m_picWidth - xPos):m_CTUSize;
#else
  Int yPos   = (ctuRsAddr / m_numCTUInWidth)*m_maxCUHeight;
  Int xPos   = (ctuRsAddr % m_numCTUInWidth)*m_maxCUWidth;
  Int height = (yPos + m_maxCUHeight > m_picHeight)?(m_picHeight- yPos):m_maxCUHeight;
  Int width  = (xPos + m_maxCUWidth  > m_picWidth )?(m_picWidth - xPos):m_maxCUWidth;
#endif

  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& ctbOffset = saoblkParam[compIdx];

    if(ctbOffset.modeIdc != SAO_MODE_OFF)
    {
      const UInt componentScaleX = getComponentScaleX(component, pPic->getChromaFormat());
      const UInt componentScaleY = getComponentScaleY(component, pPic->getChromaFormat());

      Int  blkWidth   = (width  >> componentScaleX);
      Int  blkHeight  = (height >> componentScaleY);
      Int  blkXPos    = (xPos   >> componentScaleX);
      Int  blkYPos    = (yPos   >> componentScaleY);

      Int  srcStride  = srcYuv->getStride(component);
      Pel* srcBlk     = srcYuv->getAddr(component) + blkYPos*srcStride + blkXPos;

      Int  resStride  = resYuv->getStride(component);
      Pel* resBlk     = resYuv->getAddr(component) + blkYPos*resStride + blkXPos;

      offsetBlock( pPic->getPicSym()->getSPS().getBitDepth(toChannelType(component)), ctbOffset.typeIdc, ctbOffset.offset
                  , srcBlk, resBlk, srcStride, resStride, blkWidth, blkHeight
                  , isLeftAvail, isRightAvail
                  , isAboveAvail, isBelowAvail
                  , isAboveLeftAvail, isAboveRightAvail
                  , isBelowLeftAvail, isBelowRightAvail
             #if JVET_D0033_ADAPTIVE_CLIPPING
                         , component
             #endif
                  );
    }
  } //compIdx

}

#if SAO_PEAK  
Void TComSampleAdaptiveOffset::getPeakIdxDec(Pel* src, Int srcStride, Int x, Int y, Int* idxArr, Int* iSign4, Int* iNeighSum4)
{
  Int k, diff;
  //encoder only
  //4x4 and 8x8
  for (k=0; k<4; k++)
  {
    diff = src[idxArr[k]] - src[0];
    if ( diff > 0 )
    {
      iSign4[0]++;
      iNeighSum4[0] += diff;
    }
    else if ( diff < 0 )
    {
      iSign4[1]++;
      iNeighSum4[1] -= diff;
    }
  }
}

Void TComSampleAdaptiveOffset::getPeakIdx(Pel* src, Int srcStride, Int x, Int y, Int* idxArr)
{
  Int k, diff;
  for (k=0; k<4; k++)
  {
    diff = src[idxArr[k]] - src[0];
    if ( diff > 0 )
    {
      sign4[0][y][x]++;
      neighSum4[0][y][x] += diff;
    }
    else if ( diff < 0 )
    {
      sign4[1][y][x]++;
      neighSum4[1][y][x] -= diff;
    }
  }
}
#endif

#if SAO_PEAK
Void TComSampleAdaptiveOffset::preAnalysisNeighInfo (Bool bEnc, TComPicYuv* imgRec, Int height, Int width, TComPicYuv* imgOrg, saoNeighStruct* pPeakSAO)
{
  Pel* imgY_rec = imgRec->getAddr(COMPONENT_Y);
  Int  recStride = imgRec->getStride(COMPONENT_Y);
  //for ( Int comp = COMPONENT_Y; comp < COMPONENT_Y + 1 /*MAX_NUM_COMPONENT*/; comp ++ )
  {
    Int idxArr[NEIGHBOUR_SAMPLE_NUM] = {recStride, -recStride, 1, -1};
    Pel* imgY_org;
    Int  orgStride = 0, mostFreqSign4, norm, offset, diff, class4;
    Double normDouble;
    Pel temp;
    Int diff4[NORM_MAX];
    imgY_org = imgOrg->getAddr(COMPONENT_Y);
    orgStride  = imgOrg->getStride(COMPONENT_Y);

    for (Int j = 0; j < height; j++)
    {
      for (Int i = 0; i < width; i++)
      {
        getPeakIdx(imgY_rec + i, recStride, i, j, idxArr);
        mostFreqSign4 = sign4[0][j][i] > sign4[1][j][i] ? 0 : 1;
        class4 = max(-1, sign4[mostFreqSign4][j][i] - 2);
        if (class4 > 0)
        {
          for (norm=0; norm<NORM_MAX; norm++)
          {
            normDouble = (Double)(1<<norm);
            diff = ROUND((Double)neighSum4[mostFreqSign4][j][i]/(sign4[mostFreqSign4][j][i]*normDouble));
            diff4[norm]=min(MAX_DIFF_NEIGH, diff);
          }

          //3 or 4 neighs are lgr or smaller than current
          for (offset=-OFFSET_MAX; offset<=OFFSET_MAX; offset++)
          {
            temp = imgY_org[i] - max(0, min ( imgY_rec[i] + offset, m_max_val));
            offsetErr[offset + OFFSET_MAX]=temp*temp;
          } //distortion for each offset value.

          for (norm=0; norm<NORM_MAX; norm++)
          {
            diff=diff4[norm];
            if (mostFreqSign4==0)
            {
              for (offset=0; offset<=OFFSET_MAX; offset++)
              {
                saoStat[0][0][norm][diff][offset]+=offsetErr[OFFSET_MAX+offset];
                saoStat[1][class4-1][norm][diff][offset]+=offsetErr[OFFSET_MAX+offset];
              }
            }
            else 
            {
              for (offset=-OFFSET_MAX; offset<=0; offset++)
              {
                saoStat[0][0][norm][diff][-offset]+=offsetErr[OFFSET_MAX+offset];
                saoStat[1][class4-1][norm][diff][-offset]+=offsetErr[OFFSET_MAX+offset];
              }
            }
          }
        }
      }
      imgY_rec += recStride;
      imgY_org += orgStride;
    }
  }//comp
}
Void TComSampleAdaptiveOffset::initPeakSAOVariable(Int imgHeight, Int imgWidth)
{
  initMatrix3D_UInt(&neighSum4, 2, imgHeight, imgWidth);
  initMatrix3DContiguous_UChar(&sign4    , 2, imgHeight, imgWidth);

  resetPeakSAOParam(&neighSum4, 2, imgHeight, imgWidth);
  resetPeakSAOParam(&sign4    , 2, imgHeight, imgWidth);
  initMatrix3D_UChar   (&OffsetBest, PEAKSAO_TYPE_NUM, PEAKSAO_MAX_GROUP_NUM, MAX_DIFF_NEIGH + 1);
  initMatrix5D_UChar   (&OffsetTemp, PEAKSAO_TYPE_NUM, PEAKSAO_MAX_GROUP_NUM, NORM_MAX, MAX_DIFF_NEIGH + 1, MAX_DIFF_NEIGH + 1);
  initMatrix5D_Double  (&saoStat, PEAKSAO_TYPE_NUM, PEAKSAO_MAX_GROUP_NUM, NORM_MAX, MAX_DIFF_NEIGH + 1, OFFSET_MAX + 1);
  initMatrix_Double    (&diffError, MAX_DIFF_NEIGH + 1, OFFSET_MAX + 1);
  offsetErr = (Double *) calloc((2*OFFSET_MAX + 1), sizeof(Double));
}
Void TComSampleAdaptiveOffset::freePeakSAOVariable(Int imgHeight, Int imgWidth)
{
  if(neighSum4) { free(neighSum4);  neighSum4 = NULL; }
  if(sign4)     { free(sign4);      sign4     = NULL; }
  if(offsetErr) { free(offsetErr);  offsetErr = NULL; }

  destroyMatrix5D_Double (saoStat, PEAKSAO_TYPE_NUM, PEAKSAO_MAX_GROUP_NUM, NORM_MAX);
  destroyMatrix_Double   (diffError);
  destroyMatrix5D_UChar  (OffsetTemp, PEAKSAO_TYPE_NUM, PEAKSAO_MAX_GROUP_NUM, NORM_MAX );
  destroyMatrix3D_UChar  (OffsetBest, PEAKSAO_TYPE_NUM );

}
Void TComSampleAdaptiveOffset::peakOffsetDec(TComPic* pPic, TComPicYuv* resYuv, TComPicYuv* scrYuv, Int height, Int width)
{
  UInt mostFreqSign4, offset = 0, norm, diff;
  Double normDouble;
  saoNeighStruct* saoInfo = pPic->getPicSym()->getPeakSAOParam();
  UInt maxDiffBest = saoInfo[0].derivedMaxDiff;
  UInt classBest = saoInfo[0].peakSAOType[1];
  Int iSign4[2], iNeighSum4[2];
  Int iSize = sizeof(Int) << 1;

  Pel* src     = scrYuv->getAddr(COMPONENT_Y);
  Int  srcStride  = scrYuv->getStride(COMPONENT_Y);
  Pel* res     = resYuv->getAddr(COMPONENT_Y);
  Int  resStride  = resYuv->getStride(COMPONENT_Y);
  Int idxArr[NEIGHBOUR_SAMPLE_NUM] = {srcStride, -srcStride, 1, -1};

  if(classBest == 1)
  {
    for (Int j = 0; j < height ; j++ )
    {
      for (Int i = 0; i < width ; i++ )
      {
        memset(iSign4, 0, iSize);
        memset(iNeighSum4, 0, iSize);

        getPeakIdxDec(res + i, resStride, i, j, idxArr, iSign4, iNeighSum4);
        mostFreqSign4 = iSign4[0] > iSign4[1] ? 0 : 1;
        Int class4 = iSign4[mostFreqSign4] - 2;
        if (class4 > 0 )
        {          
          Int idx = class4 -1;
          norm = saoInfo[idx].norm;
          normDouble=(Double)(1<<norm);
          diff=ROUND((Double)iNeighSum4[mostFreqSign4]/(iSign4[mostFreqSign4]*normDouble));
          diff = min(maxDiffBest, diff);
          offset = saoInfo[idx].offset[diff];
          if (mostFreqSign4 == 0)
          {
            src[i] = (res[i] + offset);
            if(src[i] > m_max_val)
            {
              src[i] = m_max_val;
            }
          }
          else 
          {
            src[i] = (res[i] - offset);
            if(src[i] < 0)
            {
              src[i] = 0;
            }
          }
        }
      }
      src += srcStride;
      res += resStride;
    }
  }
  else
  {
    norm = saoInfo[0].norm;
    normDouble=(Double)(1<<norm);
    for (Int j = 0; j < height ; j++ )
    {
      for (Int i = 0; i < width ; i++ )
      {
        memset(iSign4, 0, iSize);
        memset(iNeighSum4, 0, iSize);
        getPeakIdxDec(res + i, resStride, i, j, idxArr, iSign4, iNeighSum4);
        mostFreqSign4 = iSign4[0] > iSign4[1] ? 0 : 1;
        Int class4 = iSign4[mostFreqSign4] - 2;

        if (class4 > 0)
        {
          diff=ROUND((Double)iNeighSum4[mostFreqSign4]/(iSign4[mostFreqSign4]*normDouble));
          diff = min(maxDiffBest, diff);
          offset = saoInfo[0].offset[diff];
          if (mostFreqSign4 == 0)
          {              
            src[i] = (res[i] + offset);
            if(src[i] > m_max_val)
            {
              src[i] = m_max_val;
            }
          }
          else
          {
            src[i] = (res[i] - offset);
            if(src[i] < 0)
            {
              src[i] = 0;
            }
          }
        }
      }
      src += srcStride;
      res += resStride;
    }
  }
}
Void TComSampleAdaptiveOffset::peakOffset(TComPic* pPic, TComPicYuv* resYuv, TComPicYuv* scrYuv, Int height, Int width)
{
  UInt mostFreqSign4, offset = 0, norm, diff;
  Double normDouble;
  saoNeighStruct* saoInfo = pPic->getPicSym()->getPeakSAOParam();
  UInt maxDiffBest = saoInfo[0].derivedMaxDiff;
  UInt classBest = saoInfo[0].peakSAOType[1];

  Pel* src     = scrYuv->getAddr(COMPONENT_Y);
  Int  srcStride  = scrYuv->getStride(COMPONENT_Y);
  Pel* res     = resYuv->getAddr(COMPONENT_Y);
  Int  resStride  = resYuv->getStride(COMPONENT_Y);

  for (Int j = 0; j < height ; j++ )
  {
    for (Int i = 0; i < width ; i++ )
    {
      mostFreqSign4 = sign4[0][j][i] > sign4[1][j][i] ? 0 : 1;
      Int class4 = sign4[mostFreqSign4][j][i] - 2;
      if (class4 > 0 )
      {
        if (classBest == 0)
        {
          norm = saoInfo[0].norm;
          normDouble=(Double)(1<<norm);
          diff=ROUND((Double)neighSum4[mostFreqSign4][j][i]/(sign4[mostFreqSign4][j][i]*normDouble));
          diff = min(maxDiffBest, diff);
          offset = saoInfo[0].offset[diff];
        }
        else 
        {
          assert (classBest == 1);
          Int idx = class4 -1;
          norm = saoInfo[idx].norm;
          normDouble=(Double)(1<<norm);
          diff=ROUND((Double)neighSum4[mostFreqSign4][j][i]/(sign4[mostFreqSign4][j][i]*normDouble));
          diff = min(maxDiffBest, diff);
          offset = saoInfo[idx].offset[diff];
        }
        if (mostFreqSign4 == 0)
        {
          src[i] = (res[i] + offset);
          if(src[i] > m_max_val)
          {
            src[i] = m_max_val;
          }
        }
        else 
        {
          src[i] = (res[i] - offset);
          if(src[i] < 0)
          {
            src[i] = 0;
          }
        }
      }
    }
    src += srcStride;
    res += resStride;
  }
}

Void TComSampleAdaptiveOffset::PeakSAOProcess(TComPic* pDecPic)
{
  saoNeighStruct* pPeakSAO = pDecPic->getPicSym()->getPeakSAOParam();
  if(pPeakSAO[0].bEnabled == false)
  {
    return;
  }
  TComPicYuv* resYuv = pDecPic->getPicYuvRec();
  TComPicYuv* srcYuv = m_tempPicYuv;
  resYuv->copyToPic(srcYuv);
  srcYuv->setBorderExtension(false);
  srcYuv->extendPicBorder(PeakSAO_PADDED_SAMPLES);

  Int imgHeight = resYuv->getHeight(COMPONENT_Y);
  Int imgWidht  = resYuv->getWidth(COMPONENT_Y);
  m_max_val = (1 << pDecPic->getSlice(0)->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) - 1;
  
  peakOffsetDec(pDecPic, srcYuv, resYuv, imgHeight, imgWidht);  
}
#endif

Void TComSampleAdaptiveOffset::SAOProcess(TComPic* pDecPic)
{
  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  Bool bAllDisabled=true;
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_picSAOEnabled[compIdx])
    {
      bAllDisabled=false;
    }
  }
  if (bAllDisabled)
  {
    return;
  }

  TComPicYuv* resYuv = pDecPic->getPicYuvRec();
  TComPicYuv* srcYuv = m_tempPicYuv;
  resYuv->copyToPic(srcYuv);
  for(Int ctuRsAddr= 0; ctuRsAddr < m_numCTUsPic; ctuRsAddr++)
  {
    offsetCTU(ctuRsAddr, srcYuv, resYuv, (pDecPic->getPicSym()->getSAOBlkParam())[ctuRsAddr], pDecPic);
  } //ctu
}


/** PCM LF disable process.
 * \param pcPic picture (TComPic) pointer
 *
 * \note Replace filtered sample values of PCM mode blocks with the transmitted and reconstructed ones.
 */
Void TComSampleAdaptiveOffset::PCMLFDisableProcess (TComPic* pcPic)
{
  xPCMRestoration(pcPic);
}

/** Picture-level PCM restoration.
 * \param pcPic picture (TComPic) pointer
 */
Void TComSampleAdaptiveOffset::xPCMRestoration(TComPic* pcPic)
{
  Bool  bPCMFilter = (pcPic->getSlice(0)->getSPS()->getUsePCM() && pcPic->getSlice(0)->getSPS()->getPCMFilterDisableFlag())? true : false;

  if(bPCMFilter || pcPic->getSlice(0)->getPPS()->getTransquantBypassEnableFlag())
  {
    for( UInt ctuRsAddr = 0; ctuRsAddr < pcPic->getNumberOfCtusInFrame() ; ctuRsAddr++ )
    {
      TComDataCU* pcCU = pcPic->getCtu(ctuRsAddr);

      xPCMCURestoration(pcCU, 0, 0);
    }
  }
}

/** PCM CU restoration.
 * \param pcCU            pointer to current CU
 * \param uiAbsZorderIdx  part index
 * \param uiDepth         CU depth
 */
Void TComSampleAdaptiveOffset::xPCMCURestoration ( TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth )
{
  TComPic* pcPic     = pcCU->getPic();
  UInt uiCurNumParts = pcPic->getNumPartitionsInCtu() >> (uiDepth<<1);
  UInt uiQNumParts   = uiCurNumParts>>2;

  // go to sub-CU
  if( pcCU->getDepth(uiAbsZorderIdx) > uiDepth )
  {
    for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++, uiAbsZorderIdx+=uiQNumParts )
    {
      UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsZorderIdx] ];
      UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsZorderIdx] ];
      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
      {
        xPCMCURestoration( pcCU, uiAbsZorderIdx, uiDepth+1 );
      }
    }
    return;
  }

  // restore PCM samples
  if ((pcCU->getIPCMFlag(uiAbsZorderIdx)&& pcPic->getSlice(0)->getSPS()->getPCMFilterDisableFlag()) || pcCU->isLosslessCoded( uiAbsZorderIdx))
  {
#if JVET_C0024_QTBT
    assert(0);  //currently IPCM mode not support QTBT
#endif
    const UInt numComponents=pcPic->getNumberValidComponents();
    for(UInt comp=0; comp<numComponents; comp++)
    {
      xPCMSampleRestoration (pcCU, uiAbsZorderIdx, uiDepth, ComponentID(comp));
    }
  }
}

/** PCM sample restoration.
 * \param pcCU           pointer to current CU
 * \param uiAbsZorderIdx part index
 * \param uiDepth        CU depth
 * \param compID         texture component type
 */
Void TComSampleAdaptiveOffset::xPCMSampleRestoration (TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth, const ComponentID compID)
{
        TComPicYuv* pcPicYuvRec = pcCU->getPic()->getPicYuvRec();
        UInt uiPcmLeftShiftBit;
  const UInt uiMinCoeffSize = pcCU->getPic()->getMinCUWidth()*pcCU->getPic()->getMinCUHeight();
  const UInt csx=pcPicYuvRec->getComponentScaleX(compID);
  const UInt csy=pcPicYuvRec->getComponentScaleY(compID);
  const UInt uiOffset   = (uiMinCoeffSize*uiAbsZorderIdx)>>(csx+csy);

        Pel *piSrc = pcPicYuvRec->getAddr(compID, pcCU->getCtuRsAddr(), uiAbsZorderIdx);
  const Pel *piPcm = pcCU->getPCMSample(compID) + uiOffset;
  const UInt uiStride  = pcPicYuvRec->getStride(compID);
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());
#if JVET_C0024_QTBT
  const UInt uiWidth  = ((sps.getCTUSize()  >> uiDepth) >> csx);
  const UInt uiHeight = ((sps.getCTUSize() >> uiDepth) >> csy);
#else
  const UInt uiWidth  = ((sps.getMaxCUWidth()  >> uiDepth) >> csx);
  const UInt uiHeight = ((sps.getMaxCUHeight() >> uiDepth) >> csy);
#endif

  if ( pcCU->isLosslessCoded(uiAbsZorderIdx) && !pcCU->getIPCMFlag(uiAbsZorderIdx) )
  {
    uiPcmLeftShiftBit = 0;
  }
  else
  {
    uiPcmLeftShiftBit = sps.getBitDepth(toChannelType(compID)) - sps.getPCMBitDepth(toChannelType(compID));
  }

  for(UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for(UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      piSrc[uiX] = (piPcm[uiX] << uiPcmLeftShiftBit);
    }
    piPcm += uiWidth;
    piSrc += uiStride;
  }
}

//! \}
