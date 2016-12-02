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
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "TComBilateralFilter.h"

const int TComBilateralFilter::SpatialSigmaValues[] = {62};

const int TComBilateralFilter::spatialSigmaBlockLengthOffsets[] = {20, 10, -10};

TComBilateralFilter* TComBilateralFilter::m_bilateralFilterInstance = NULL;

TComBilateralFilter::TComBilateralFilter()
{
  int lookupTableSize = NUM_OF_SPATIAL_SIGMA * 3;

  // allocation
  m_bilateralFilterTable = new Int*[lookupTableSize];
  for(int i = 0; i < lookupTableSize; i++)
    m_bilateralFilterTable[i] = new Int[1024];

  // initialization
  for(int i = 0; i < lookupTableSize; i++)
    for(int j = 0; j < 1024; j++)
      m_bilateralFilterTable[i][j] = 0;
}

TComBilateralFilter::~TComBilateralFilter()
{
  int lookupTableSize = NUM_OF_SPATIAL_SIGMA * 3;
  for(int i = 0; i < lookupTableSize; ++i)
    delete [] m_bilateralFilterTable[i];
  delete [] m_bilateralFilterTable;
}

TComBilateralFilter* TComBilateralFilter::instance()
{
  if (m_bilateralFilterInstance == NULL)
    m_bilateralFilterInstance = new TComBilateralFilter();
  return m_bilateralFilterInstance;
}

Void TComBilateralFilter::createBilateralFilterTable(Int qp)
{
  Int spatialSigmaValue;
  Int intensitySigmaValue = (qp - 17) * 50;
  if (intensitySigmaValue < 0)
  {
    intensitySigmaValue = 1;
  }
  Int sqrtSpatialSigmaMulTwo;
  Int sqrtIntensitySigmaMulTwo = 2 * intensitySigmaValue * intensitySigmaValue;

  for (Int n = 0; n < NUM_OF_SPATIAL_SIGMA; n++)
  {
    spatialSigmaValue = SpatialSigmaValues[n];
    for (Int i = 0; i < 3; i++)
    {
      sqrtSpatialSigmaMulTwo = 2 * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]) * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]);
      for (Int j = 0; j < 1024; j++)
      {
        Int temp = j * 25;
        m_bilateralFilterTable[i + 3 * n][j] = exp(-(10000.0 / sqrtSpatialSigmaMulTwo) - (temp * temp / (sqrtIntensitySigmaMulTwo * 1.0))) * 100000;
      }
    }
  }
}

#if BILATERAL_FILTER_NO_SUBBOCK
Void TComBilateralFilter::smoothBlockBilateralFilter(TComDataCU* pcCU, UInt uiWidth, UInt uiHeight, Short block[], Int length, Int optimalSpatialSigmaIndex)
#else
Void TComBilateralFilter::smoothBlockBilateralFilter(TComDataCU* pcCU, Short block[], Int length, Int optimalSpatialSigmaIndex)
#endif
{
#if BILATERAL_FILTER_NO_SUBBOCK
  Int blockSize = uiWidth * uiHeight;
#else
  Int blockSize = length * length;
#endif
  Short filteredBlock[128*128];
  Int rightPixel, bottomPixel, centerPixel;
  Int rightWeight, bottomWeight, centerWeight;
  Int sumWeights[128*128], sum[128*128];
  Int blockLengthIndex;

  switch (length)
  {
    case 4:
      blockLengthIndex = 0;
      break;
    case 8:
      blockLengthIndex = 1;
      break;
    default:
      blockLengthIndex = 2;
      break;
  }

  Int *lookupTablePtr;

  for (int k = 0; k < blockSize; k++)
  {
    sumWeights[k] = 0;
    sum[k] = 0;
  }

  Int pixelIndex = 0;
  lookupTablePtr = m_bilateralFilterTable[blockLengthIndex + 3 * optimalSpatialSigmaIndex];

  // for each pixel in block
#if BILATERAL_FILTER_NO_SUBBOCK
  for (Int j = 0; j < (uiHeight - 1); j++)
  {
    pixelIndex = j * uiWidth;
    for (Int i = 0; i < (uiWidth - 1); i++)
    {
#else
  for (Int j = 0; j < (length - 1); j++)
  {
    pixelIndex = j * length;
    for (Int i = 0; i < (length - 1); i++)
    {
#endif
      centerPixel = block[pixelIndex];
      centerWeight = 100000;

      sumWeights[pixelIndex] += centerWeight;
      sum[pixelIndex] += centerPixel * centerWeight;

      //if (i < (length - 1))
      {
        rightPixel = block[pixelIndex + 1];
        rightWeight = lookupTablePtr[abs(rightPixel - centerPixel)];

        sumWeights[pixelIndex] += rightWeight;
        sum[pixelIndex] += rightPixel * rightWeight;

        sumWeights[pixelIndex + 1] += rightWeight; //next pixel to the right
        sum[pixelIndex + 1] += centerPixel * rightWeight; //next pixel to the right
      }

      //if (j < (length - 1))
      {
#if BILATERAL_FILTER_NO_SUBBOCK
        bottomPixel = block[pixelIndex + uiWidth];
#else
        bottomPixel = block[pixelIndex + length];
#endif
        bottomWeight = lookupTablePtr[abs(bottomPixel - centerPixel)];

        sumWeights[pixelIndex] += bottomWeight;
        sum[pixelIndex] += bottomPixel * bottomWeight;

#if BILATERAL_FILTER_NO_SUBBOCK
        sumWeights[pixelIndex + uiWidth] += bottomWeight; //next pixel to the bottom
        sum[pixelIndex + uiWidth] += centerPixel * bottomWeight; //next pixel to the bottom
#else
        sumWeights[pixelIndex + length] += bottomWeight; //next pixel to the bottom
        sum[pixelIndex + length] += centerPixel * bottomWeight; //next pixel to the bottom
#endif
      }
      filteredBlock[pixelIndex] = (sum[pixelIndex] + (sumWeights[pixelIndex] / 2) )/ sumWeights[pixelIndex];
      pixelIndex++;
    }

    // j < length - 1; i = length - 1
    {
      centerPixel = block[pixelIndex];
      centerWeight = 100000;

      sumWeights[pixelIndex] += centerWeight;
      sum[pixelIndex] += centerPixel * centerWeight;

#if BILATERAL_FILTER_NO_SUBBOCK
      bottomPixel = block[pixelIndex + uiWidth];
#else
      bottomPixel = block[pixelIndex + length];
#endif
      bottomWeight = lookupTablePtr[abs(bottomPixel - centerPixel)];

      sumWeights[pixelIndex] += bottomWeight;
      sum[pixelIndex] += bottomPixel * bottomWeight;

#if BILATERAL_FILTER_NO_SUBBOCK
      sumWeights[pixelIndex + uiWidth] += bottomWeight; //next pixel to the bottom
      sum[pixelIndex + uiWidth] += centerPixel * bottomWeight; //next pixel to the bottom
#else
      sumWeights[pixelIndex + length] += bottomWeight; //next pixel to the bottom
      sum[pixelIndex + length] += centerPixel * bottomWeight; //next pixel to the bottom
#endif
      filteredBlock[pixelIndex] = (sum[pixelIndex] + (sumWeights[pixelIndex] / 2) )/ sumWeights[pixelIndex];
      pixelIndex++;
    }
  }

  // j = length - 1; i < length - 1
#if BILATERAL_FILTER_NO_SUBBOCK
  for (Int i = 0; i < (uiWidth - 1); i++)
#else
  for (Int i = 0; i < (length - 1); i++)
#endif
  {
    centerPixel = block[pixelIndex];
    centerWeight = 100000;

    sumWeights[pixelIndex] += centerWeight;
    sum[pixelIndex] += centerPixel * centerWeight;

    rightPixel = block[pixelIndex + 1];
    rightWeight = lookupTablePtr[abs(rightPixel - centerPixel)];

    sumWeights[pixelIndex] += rightWeight;
    sum[pixelIndex] += rightPixel * rightWeight;

    sumWeights[pixelIndex + 1] += rightWeight; //next pixel to the right
    sum[pixelIndex + 1] += centerPixel * rightWeight; //next pixel to the right
    filteredBlock[pixelIndex] = (sum[pixelIndex] + (sumWeights[pixelIndex] / 2) )/ sumWeights[pixelIndex];
    pixelIndex++;
  }
  // j = length - 1; i = length - 1
  {
    centerPixel = block[pixelIndex];
    centerWeight = 100000;

    sumWeights[pixelIndex] += centerWeight;
    sum[pixelIndex] += centerPixel * centerWeight;

    filteredBlock[pixelIndex] = (sum[pixelIndex] + (sumWeights[pixelIndex] / 2) )/ sumWeights[pixelIndex];
  }

  memcpy(block, filteredBlock, blockSize * sizeof(Short));
}

Void TComBilateralFilter::bilateralFilterIntra(TComDataCU *pcCU, UInt uiWidth, UInt uiHeight, Pel *piReco, UInt uiStride, Int optimalSpatialSigmaIndex)
{
#if BILATERAL_FILTER_NO_SUBBOCK
  UInt uiMinSize = std::min(uiWidth, uiHeight);
  Short *tempblock = new Short[ uiWidth * uiHeight ];
#else
  UInt uiMinSize = std::min(uiWidth, uiHeight);
  Short *tempblock;
  tempblock = new Short[ uiMinSize * uiMinSize ];
#endif

#if BILATERAL_FILTER_NO_SUBBOCK
      for (UInt j = 0; j < uiHeight; j++)
      {
        memcpy(tempblock + j * uiWidth, piReco + j * uiStride, uiWidth * sizeof(Short));
      }
      smoothBlockBilateralFilter(pcCU, uiWidth, uiHeight, tempblock, uiMinSize, optimalSpatialSigmaIndex);
      for (UInt j = 0; j < uiHeight; j++)
      {
        memcpy(piReco + j * uiStride, tempblock + j * uiWidth, uiWidth * sizeof(Short));
      }
#else
  for (UInt j = 0; j < (uiHeight / uiMinSize); j++)
  {
    for(UInt i = 0; i < (uiWidth / uiMinSize); i++)
    {
      for (UInt k = 0; k < uiMinSize; k++)
      {
        memcpy(tempblock + k * uiMinSize, piReco + (j * uiMinSize + k) * uiStride + i * uiMinSize, uiMinSize * sizeof(Short));
        k++;
        memcpy(tempblock + k * uiMinSize, piReco + (j * uiMinSize + k) * uiStride + i * uiMinSize, uiMinSize * sizeof(Short));
      }
      smoothBlockBilateralFilter(pcCU, tempblock, uiMinSize, optimalSpatialSigmaIndex);
      for (UInt k = 0; k < uiMinSize; k++)
      {
        memcpy(piReco + (j * uiMinSize + k) * uiStride + i * uiMinSize, tempblock + k * uiMinSize, uiMinSize * sizeof(Short));
        k++;
        memcpy(piReco + (j * uiMinSize + k) * uiStride + i * uiMinSize, tempblock + k * uiMinSize, uiMinSize * sizeof(Short));
      }
    }
  }
#endif
  delete[] tempblock;
}

Void TComBilateralFilter::bilateralFilterInter(TComDataCU *pcCU, UInt uiWidth, UInt uiHeight, Pel *piResi, UInt uiStrideRes, Pel *piPred, UInt uiPredStride, Pel *piReco, UInt uiRecStride, Int clipbd, Int optimalSpatialSigmaIndex)
{
#if BILATERAL_FILTER_NO_SUBBOCK
  UInt uiMinSize = std::min(uiWidth, uiHeight);
  Short *tempblock = new Short[ uiWidth * uiHeight ];
#else
  UInt uiMinSize = std::min(uiWidth, uiHeight);
  Short *tempblock = new Short[ uiMinSize * uiMinSize ];
#endif
  Pel *piPredTemp = piPred;
  Pel *piResiTemp = piResi;
  Pel *piRecoTemp = piReco;
  // Reco = Pred + Resi
  for (UInt uiY = 0; uiY < uiHeight; ++uiY)
  {
    for (UInt uiX = 0; uiX < uiWidth; ++uiX)
    {
#if JVET_D0033_ADAPTIVE_CLIPPING
      piReco[uiX] = ClipA(piPred[uiX] + piResi[uiX], COMPONENT_Y);
#else
      piReco[uiX] = ClipBD(piPred[uiX] + piResi[uiX], clipbd);
#endif
    }
    piPred += uiPredStride;
    piResi += uiStrideRes;
    piReco += uiRecStride;
  }

  piPred = piPredTemp;
  piResi = piResiTemp;
  piReco = piRecoTemp;

  // Reco' = filter(Reco)
#if BILATERAL_FILTER_NO_SUBBOCK
      for (UInt j = 0; j < uiHeight; j++)
      {
        memcpy(tempblock + j * uiWidth, piReco + j * uiRecStride, uiWidth * sizeof(Short));
      }
      smoothBlockBilateralFilter(pcCU, uiWidth, uiHeight, tempblock, uiMinSize, optimalSpatialSigmaIndex);
      for (UInt j = 0; j < uiHeight; j++)
      {
        memcpy(piReco + j * uiRecStride, tempblock + j * uiWidth, uiWidth * sizeof(Short));
      }
#else
  for (UInt j = 0; j < uiHeight / uiMinSize; j++)
  {
    for(UInt i = 0; i < uiWidth / uiMinSize; i++)
    {
      for (UInt k = 0; k < uiMinSize; k++)
      {
        memcpy(tempblock + k * uiMinSize, piReco + (j * uiMinSize + k) * uiRecStride + i * uiMinSize, uiMinSize * sizeof(Short));
        k++;
        memcpy(tempblock + k * uiMinSize, piReco + (j * uiMinSize + k) * uiRecStride + i * uiMinSize, uiMinSize * sizeof(Short));
      }
      smoothBlockBilateralFilter(pcCU, tempblock, uiMinSize, optimalSpatialSigmaIndex);
      for (UInt k = 0; k < uiMinSize; k++)
      {
        memcpy(piReco + (j * uiMinSize + k) * uiRecStride + i * uiMinSize, tempblock + k * uiMinSize, uiMinSize * sizeof(Short));
        k++;
        memcpy(piReco + (j * uiMinSize + k) * uiRecStride + i * uiMinSize, tempblock + k * uiMinSize, uiMinSize * sizeof(Short));
      }
    }
  }
#endif
  delete[] tempblock;

  // Resi' = Reco' - Pred
  for (UInt uiY = 0; uiY < uiHeight; ++uiY)
  {
    for (UInt uiX = 0; uiX < uiWidth; ++uiX)
    {
      piResi[uiX] = piReco[uiX] - piPred[uiX];
    }
    piPred += uiPredStride;
    piResi += uiStrideRes;
    piReco += uiRecStride;
  }


}

#if BILATERAL_FILTER_TEST == 2 || BILATERAL_FILTER_TEST == 3

  Void TComBilateralFilter::bilateralFilterCU(TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight)
  {
#if JVET_C0024_QTBT 
  if(pcCU->getPic()==0)
#else
  if(pcCU->getPic()==0||pcCU->getPartitionSize(uiAbsZorderIdx)==NUMBER_OF_PART_SIZES)
#endif
  {
    return;
  }

  TComPic* pcPic     = pcCU->getPic();
  UInt uiCurNumParts = pcPic->getNumPartitionsInCtu() >> (uiDepth<<1);
  UInt uiQNumParts   = uiCurNumParts>>2;
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());

  if( pcCU->getDepth(uiAbsZorderIdx) > uiDepth )
  {
    for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++, uiAbsZorderIdx+=uiQNumParts )
    {
      UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsZorderIdx] ];
      UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsZorderIdx] ];
      if( ( uiLPelX < sps.getPicWidthInLumaSamples() ) && ( uiTPelY < sps.getPicHeightInLumaSamples() ) )
      {
        bilateralFilterCU( pcCU, uiAbsZorderIdx, uiDepth+1, uiWidth>>1, uiHeight>>1 );
      }
    }
    return;
  }

#if JVET_C0024_QTBT
  UInt uiBTDepth = pcCU->getBTDepth(uiAbsZorderIdx, uiWidth, uiHeight);
  UInt uiMinCUW = pcCU->getPic()->getMinCUWidth();
  UInt uiMinCUH = pcCU->getPic()->getMinCUHeight();

  if (pcCU->getBTSplitModeForBTDepth(uiAbsZorderIdx, uiBTDepth)==1)
  {
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 2; uiPartUnitIdx++ )
    {
      if (uiPartUnitIdx==1)
      {
        uiAbsZorderIdx = g_auiRasterToZscan[g_auiZscanToRaster[uiAbsZorderIdx] 
        + (uiHeight>>1)/uiMinCUH*pcCU->getPic()->getNumPartInCtuWidth()];
      }
      bilateralFilterCU(pcCU, uiAbsZorderIdx, uiDepth, uiWidth, uiHeight>>1);
    }
    return;
  }
  else if (pcCU->getBTSplitModeForBTDepth(uiAbsZorderIdx, uiBTDepth)==2)
  {
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 2; uiPartUnitIdx++ )
    {
      if (uiPartUnitIdx==1)
      {
        uiAbsZorderIdx = g_auiRasterToZscan[g_auiZscanToRaster[uiAbsZorderIdx] 
        + (uiWidth>>1)/uiMinCUW];
      }
      bilateralFilterCU(pcCU, uiAbsZorderIdx, uiDepth, uiWidth>>1, uiHeight);
    }
    return;
  }
#endif

    // At the block. Do filtering!

    if (pcCU->getCbf(uiAbsZorderIdx, COMPONENT_Y) != 0)
    {
      Pel* piReco = pcPic->getPicYuvRec()->getAddr(COMPONENT_Y, pcCU->getCtuRsAddr(), uiAbsZorderIdx);
      UInt uiRecStride = pcPic->getPicYuvRec()->getStride(COMPONENT_Y);

      //No need to calculated reconstruction or update residual like bilateralFilterInter does so calls bilateralFilterIntra independent on picture type
      bilateralFilterIntra(pcCU, uiWidth, uiHeight, piReco, uiRecStride, 0);
    }
  }

  Void TComBilateralFilter::bilateralFilterPic(TComPic* pcPic)
  {
    for ( UInt ctuRsAddr = 0; ctuRsAddr < pcPic->getNumberOfCtusInFrame(); ctuRsAddr++ )
    {
      TComDataCU* pCtu = pcPic->getCtu(ctuRsAddr);
      UInt uiCTUSize = pCtu->getSlice()->getSPS()->getCTUSize() ;
      bilateralFilterCU(pCtu, 0, 0, uiCTUSize, uiCTUSize);
    } 
  }
  
#endif

