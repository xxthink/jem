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
#include <algorithm>
#include "TComBilateralFilter.h"

const int TComBilateralFilter::SpatialSigmaValues[] = {62};

#if BILATERAL_FILTER_REUSE_INTRA_PART
const int TComBilateralFilter::spatialSigmaBlockLengthOffsets[] = {20, 10, -10, 0, -30};
#else
const int TComBilateralFilter::spatialSigmaBlockLengthOffsets[] = {20, 10, -10, 0, -10, -30};
#endif
TComBilateralFilter* TComBilateralFilter::m_bilateralFilterInstance = NULL;

#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
#if (BILATERAL_FILTER_REDUCE_RANGE>0) && (BILATERAL_FILTER_REDUCE_RANGE <= 536)
#if BILATERAL_FILTER_REDUCE_RANGE<=65
const UShort maxPosList[34] = {6, 12, 18, 23, 29, 35, 41, 46, 52, 58, 64, 69, 75, 81, 87, 92, 98, 104, 110, 115, 121, 127, 133, 138, 144, 150, 156, 161, 167, 173, 179, 184, 190, 196};
#else
const UShort maxPosList[34] = {8, 15, 22, 29, 36, 43, 50, 57, 64, 71, 78, 85, 92, 99, 106, 113, 121, 128, 135, 142, 149, 156, 163, 170, 177, 184, 191, 198, 205, 212, 219, 226, 234, 241};
#endif
#else
const UShort maxPosList[34] = {11,    20,    29,    39,    48,    57,    66,    76,    85,    94,   104,   113,   122,   131,   141,   150,   159,   169,   178,   187,   196,  206,   215,   224,   234,   243,   252,   261,   271,   280,   289,   299,   308,   317};
#endif
#endif

#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
const Short shiftEntryList[] = {0, 0, 0, 0, 0, 2};
#endif

TComBilateralFilter::TComBilateralFilter()
{
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE == 0
#if BILATERAL_FILTER_REUSE_INTRA_PART
  int lookupTableSize = 5;
#else
  int lookupTableSize = 6;
#endif
#endif
  int numQP = MAX_QP-18+1;
  // allocation
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE
  m_bilateralFilterTable = new UShort*[numQP];
#else
  m_bilateralFilterTable = new UShort**[numQP];
#endif
  for(int i = 0; i < numQP; i++)
  {
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE
    m_bilateralFilterTable[i] = new UShort[maxPosList[i]+1];
#else
    m_bilateralFilterTable[i] = new UShort*[lookupTableSize];
#endif
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE == 0
    for(int j = 0; j < lookupTableSize; j++)
#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
      m_bilateralFilterTable[i][j] = new UShort[maxPosList[i]+1];
#else
      m_bilateralFilterTable[i][j] = new UShort[1024];
#endif
#endif
  }

  // initialization
  for(int i = 0; i < numQP; i++)
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE == 0
    for(int j = 0; j < lookupTableSize; j++)
#endif
#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
      for(int k = 0; k < (maxPosList[i]+1); k++)
#else
      for(int k = 0; k < 1024; k++)
#endif
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE
        m_bilateralFilterTable[i][k] = 0;
#else
        m_bilateralFilterTable[i][j][k] = 0;
#endif
}

TComBilateralFilter::~TComBilateralFilter()
{
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE == 0
#if BILATERAL_FILTER_REUSE_INTRA_PART
  int lookupTableSize = 5;
#else
  int lookupTableSize = 6;
#endif
#endif
  int numQP = MAX_QP-18+1;
  for(int i = 0; i < numQP; ++i)
  {
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE == 0
    for(int j = 0; j < lookupTableSize; ++j)
    {
      delete [] m_bilateralFilterTable[i][j];
    }
#endif
    delete [] m_bilateralFilterTable[i];
  }
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
#if BILATERAL_FILTER_REUSE_INTRA_PART
  int lookupTableSize = 5;
#else
  int lookupTableSize = 6;
#endif
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE
  for (Int n = 0; n < NUM_OF_SPATIAL_SIGMA; n++)
  {
    spatialSigmaValue = SpatialSigmaValues[n];
    for (Int i = 0; i < lookupTableSize; i++)
    {
      sqrtSpatialSigmaMulTwo = 2 * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]) * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]);

      // Calculate the multiplication factor that we will use to convert the first table (with the strongest filter) to one of the
      // tables that gives weaker filtering (such as when TU = 8 or 16 or when we have inter filtering). 
      Int sqrtSpatialSigmaMulTwoStrongestFiltering = 2 * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[0]) * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[0]);

      // multiplication factor equals exp(-1/stronger)/exp(-1/weaker)
      double centerWeightMultiplier = exp(-(10000.0 / sqrtSpatialSigmaMulTwoStrongestFiltering))/exp(-(10000.0 / sqrtSpatialSigmaMulTwo));
#if BILATERAL_FILTER_REDUCE_RANGE
      m_bilateralCenterWeightTable[i] = (centerWeightMultiplier*BILATERAL_FILTER_REDUCE_RANGE + 0.5);
#else

      m_bilateralCenterWeightTable[i] = centerWeightMultiplier*100000;

#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
      if(i==5)
        m_bilateralCenterWeightTable[i] = m_bilateralCenterWeightTable[i] >> 2;
#endif

#endif
    }
  }

#endif
  for (Int n = 0; n < NUM_OF_SPATIAL_SIGMA; n++)
  {
    spatialSigmaValue = SpatialSigmaValues[n];
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE == 0
    for (Int i = 0; i < lookupTableSize; i++)
    {
#else
    Int i = 0;
#endif
    sqrtSpatialSigmaMulTwo = 2 * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]) * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]);
#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF     
    for (Int j = 0; j < (maxPosList[qp-18]+1); j++)
#else
    for (Int j = 0; j < 1024; j++)
#endif      
    {        
      Int temp = j * 25;
#if BILATERAL_FILTER_REDUCE_RANGE
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE     
      m_bilateralFilterTable[qp-18][j] = UShort(exp(-(10000.0 / sqrtSpatialSigmaMulTwo) - (temp * temp / (sqrtIntensitySigmaMulTwo * 1.0))) * BILATERAL_FILTER_REDUCE_RANGE + 0.5);
#else        
      m_bilateralFilterTable[qp-18][i + 3 * n][j] = UShort(exp(-(10000.0 / sqrtSpatialSigmaMulTwo) - (temp * temp / (sqrtIntensitySigmaMulTwo * 1.0))) * BILATERAL_FILTER_REDUCE_RANGE + 0.5);
#endif
#else
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE
      m_bilateralFilterTable[qp-18][j] = UShort(exp(-(10000.0 / sqrtSpatialSigmaMulTwo) - (temp * temp / (sqrtIntensitySigmaMulTwo * 1.0))) * 100000);
#else
      m_bilateralFilterTable[qp-18][i + 3 * n][j] = UShort(exp(-(10000.0 / sqrtSpatialSigmaMulTwo) - (temp * temp / (sqrtIntensitySigmaMulTwo * 1.0))) * 100000);
#endif
#endif
    }
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE == 0
    }
#endif
  }
}

Void TComBilateralFilter::smoothBlockBilateralFilter(TComDataCU* pcCU, UInt uiWidth, UInt uiHeight, Short block[], Int length, Int optimalSpatialSigmaIndex, Int qp)
{
  Int rightPixel, bottomPixel, centerPixel;
  Int rightWeight, bottomWeight, centerWeight;
  Int sumWeights[128], sum[128];
  Int blockLengthIndex;
#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
  Short shiftEntry;
#endif

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

  UShort *lookupTablePtr;

#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE
  centerWeight = m_bilateralCenterWeightTable[blockLengthIndex + 3 * optimalSpatialSigmaIndex];
#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
  shiftEntry = shiftEntryList[blockLengthIndex + 3 * optimalSpatialSigmaIndex];
#endif
#else
#if BILATERAL_FILTER_REDUCE_RANGE
  centerWeight = BILATERAL_FILTER_REDUCE_RANGE;
#else
  centerWeight = 100000;
#endif
#endif

#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF 
  Int theMaxPos = maxPosList[qp-18];
#endif

#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE
  lookupTablePtr = m_bilateralFilterTable[qp-18];
#else
#if BILATERAL_FILTER_REUSE_INTRA_PART
  if((optimalSpatialSigmaIndex==1))
  {
    switch (length)
    {
      case 4:
        lookupTablePtr = m_bilateralFilterTable[qp-18][3];
        break;
      case 8:
        lookupTablePtr = m_bilateralFilterTable[qp-18][2];
        break;
      default:
        lookupTablePtr = m_bilateralFilterTable[qp-18][4];
        break;
    }
  }
  else
  {
    lookupTablePtr = m_bilateralFilterTable[qp-18][blockLengthIndex];
  }
#else
  lookupTablePtr = m_bilateralFilterTable[qp-18][blockLengthIndex + 3 * optimalSpatialSigmaIndex];
#endif
#endif
  // for each pixel in block

  // These are the types of pixels:
  //
  // A BB C
  //
  // D EE F
  // D EE F
  //
  // G HH I
  //
  // If the block is larger than 4x4, the E-part is larger.
  //
  // Filter types:
  //
  // AA  BBB   CC
  // A    B     C
  //
  // D    E     F
  // DD  EEE   FF
  // D    E     F
  //
  // G    H     I
  // GG  HHH   II
  // C uses a filter of type x
  Int currentPixelSum;
  Int currentPixelSumWeights;
  Int rightPixelSum;
  Int rightPixelSumWeights;
  
  Short *blockCurrentPixelPtr = block;
  Short *blockRightPixelPtr = blockCurrentPixelPtr+1;
  Short *blockNextLinePixelPtr = blockCurrentPixelPtr + uiWidth;
  Int *sumWeightsPtr = sumWeights;
  Int *sumPtr = sum;
  
  // A pixel. uses filter type xx
  //                           x
  //
  // No information from previous row
  // No information from previous pixel
  // Storing information to next row
  // Storing information to next pixel
  
  // top left pixel; i = 0, j = 0;
  
  centerPixel = *(blockCurrentPixelPtr);
  rightPixel = *(blockRightPixelPtr++);
  bottomPixel = *(blockNextLinePixelPtr++);
#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
  rightWeight = lookupTablePtr[std::min(theMaxPos, abs(rightPixel - centerPixel))];
  bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(bottomPixel - centerPixel))];
#else
  rightWeight = lookupTablePtr[abs(rightPixel - centerPixel)];
  bottomWeight = lookupTablePtr[abs(bottomPixel - centerPixel)];
#endif  
#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
  currentPixelSumWeights = centerWeight + ((rightWeight + bottomWeight) >> shiftEntry);
  currentPixelSum = centerPixel * centerWeight + ((rightPixel * rightWeight + bottomPixel * bottomWeight) >> shiftEntry);
#else
  currentPixelSumWeights = centerWeight + rightWeight + bottomWeight;
  currentPixelSum = centerPixel * centerWeight + rightPixel * rightWeight + bottomPixel * bottomWeight;
#endif
  
  rightPixelSumWeights = rightWeight; //next pixel to the right
  rightPixelSum = centerPixel * rightWeight; //next pixel to the right
  
  *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
  *(sumPtr++) = centerPixel * bottomWeight; //next pixel to the bottom
  *(blockCurrentPixelPtr++) = (currentPixelSum + (currentPixelSumWeights >> 1) )/currentPixelSumWeights;

  for (Int i = 1; i < (uiWidth - 1); i++)
  {
    // B pixel. uses filter type xxx
    //                            x
    //
    // No information from previous row
    // Information reused from previous pixel
    // Storing information to next row
    // Storing information to next pixel

    centerPixel = rightPixel;
    rightPixel = *(blockRightPixelPtr++);
    bottomPixel = *(blockNextLinePixelPtr++);
#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
    rightWeight = lookupTablePtr[std::min(theMaxPos, abs(rightPixel - centerPixel))];
    bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(bottomPixel - centerPixel))];
#else
    rightWeight = lookupTablePtr[abs(rightPixel - centerPixel)];
    bottomWeight = lookupTablePtr[abs(bottomPixel - centerPixel)];
#endif    
#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
    currentPixelSumWeights = centerWeight + ((rightPixelSumWeights + rightWeight + bottomWeight) >> shiftEntry);
    currentPixelSum = centerPixel * centerWeight + ((rightPixelSum + rightPixel * rightWeight + bottomPixel * bottomWeight) >> shiftEntry);
#else
    currentPixelSumWeights = centerWeight + rightPixelSumWeights + rightWeight + bottomWeight;
    currentPixelSum = centerPixel * centerWeight + rightPixelSum + rightPixel * rightWeight + bottomPixel * bottomWeight;
#endif
    
    rightPixelSumWeights = rightWeight; //next pixel to the right
    rightPixelSum = centerPixel * rightWeight; //next pixel to the right

    *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
    *(sumPtr++) = centerPixel * bottomWeight; //next pixel to the bottom
    *(blockCurrentPixelPtr++) = (currentPixelSum + (currentPixelSumWeights >> 1) )/currentPixelSumWeights;
  }
  
  // C pixel. uses filter type xx
  //                            x
  //
  // No information from previous row
  // Information reused from previous pixel
  // Storing information to next row
  // No information to store to next pixel
  
  centerPixel = rightPixel;
  blockRightPixelPtr++;
  bottomPixel = *(blockNextLinePixelPtr++);
#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
  bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(bottomPixel - centerPixel))];
#else
  bottomWeight = lookupTablePtr[abs(bottomPixel - centerPixel)];
#endif
#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
  currentPixelSumWeights = centerWeight + ((rightPixelSumWeights + bottomWeight) >> shiftEntry);
  currentPixelSum = centerPixel * centerWeight + ((rightPixelSum + bottomPixel * bottomWeight) >> shiftEntry);
#else
  currentPixelSumWeights = centerWeight + rightPixelSumWeights + bottomWeight;
  currentPixelSum = centerPixel * centerWeight + rightPixelSum + bottomPixel * bottomWeight;
#endif
  
  *(sumWeightsPtr) = bottomWeight; //next pixel to the bottom
  *(sumPtr) = centerPixel * bottomWeight; //next pixel to the bottom
  *(blockCurrentPixelPtr++) = (currentPixelSum + (currentPixelSumWeights >> 1) )/currentPixelSumWeights;
  
  for (Int j = 1; j < (uiHeight - 1); j++)
  {
    sumWeightsPtr = sumWeights;
    sumPtr = sum;
    
    //                           x
    // D pixel. uses filter type xx
    //                           x
    //
    // Uses information from previous row
    // No information from previous pixel
    // Storing information to next row
    // Storing information to next pixel
    
    centerPixel = *(blockCurrentPixelPtr);
    rightPixel = *(blockRightPixelPtr++);
    bottomPixel = *(blockNextLinePixelPtr++);

#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
    rightWeight = lookupTablePtr[std::min(theMaxPos, abs(rightPixel - centerPixel))];
    bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(bottomPixel - centerPixel))];
#else
    rightWeight = lookupTablePtr[abs(rightPixel - centerPixel)];
    bottomWeight = lookupTablePtr[abs(bottomPixel - centerPixel)];
#endif    

#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
    currentPixelSumWeights = centerWeight + ((*(sumWeightsPtr) + rightWeight + bottomWeight) >> shiftEntry);
    currentPixelSum = centerPixel * centerWeight + ((*(sumPtr) + rightPixel * rightWeight + bottomPixel * bottomWeight) >> shiftEntry);
#else
    currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightWeight + bottomWeight;
    currentPixelSum = centerPixel * centerWeight + *(sumPtr) + rightPixel * rightWeight + bottomPixel * bottomWeight;
#endif
    
    rightPixelSumWeights = rightWeight; //next pixel to the right
    rightPixelSum = centerPixel * rightWeight; //next pixel to the right
    
    *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
    *(sumPtr++) = centerPixel * bottomWeight; //next pixel to the bottom
    *(blockCurrentPixelPtr++) = (currentPixelSum + (currentPixelSumWeights >> 1) )/currentPixelSumWeights;

    for (Int i = 1; i < (uiWidth - 1); i++)
    {
      //                            x
      // E pixel. uses filter type xxx
      //                            x
      //
      // Uses information from previous row
      // Uses information from previous pixel
      // Storing information to next row
      // No information to store to next pixel

      centerPixel = rightPixel;
      rightPixel = *(blockRightPixelPtr++);
      bottomPixel = *(blockNextLinePixelPtr++);

#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
      rightWeight = lookupTablePtr[std::min(theMaxPos, abs(rightPixel - centerPixel))];
      bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(bottomPixel - centerPixel))];
#else
      rightWeight = lookupTablePtr[abs(rightPixel - centerPixel)];
      bottomWeight = lookupTablePtr[abs(bottomPixel - centerPixel)];
#endif      

#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
      currentPixelSumWeights = centerWeight + ((*(sumWeightsPtr) + rightPixelSumWeights + rightWeight + bottomWeight) >> shiftEntry);
      currentPixelSum = centerPixel * centerWeight + ((*(sumPtr) + rightPixelSum + rightPixel * rightWeight + bottomPixel * bottomWeight) >> shiftEntry);
#else
      currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightPixelSumWeights + rightWeight + bottomWeight;
      currentPixelSum = centerPixel * centerWeight + *(sumPtr) + rightPixelSum + rightPixel * rightWeight + bottomPixel * bottomWeight;
#endif      
      rightPixelSumWeights = rightWeight; //next pixel to the right
      rightPixelSum = centerPixel * rightWeight; //next pixel to the right
      
      *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
      *(sumPtr++) = centerPixel * bottomWeight; //next pixel to the bottom
      *(blockCurrentPixelPtr++) = (currentPixelSum + (currentPixelSumWeights >> 1) )/currentPixelSumWeights;
    }

    //                            x
    // F pixel. uses filter type xx
    //                            x
    //
    // Uses information from previous row
    // Uses information from previous pixel
    // Storing information to next row
    // Storing information to next pixel
    
    centerPixel = rightPixel;
    blockRightPixelPtr++;
    bottomPixel = *(blockNextLinePixelPtr++);
#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
    bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(bottomPixel - centerPixel))];
#else
    bottomWeight = lookupTablePtr[abs(bottomPixel - centerPixel)];
#endif    

#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
    currentPixelSumWeights = centerWeight + ((*(sumWeightsPtr) + rightPixelSumWeights + bottomWeight) >> shiftEntry);
    currentPixelSum = centerPixel * centerWeight + ((*(sumPtr) + rightPixelSum +  bottomPixel * bottomWeight) >> shiftEntry);
#else
    currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightPixelSumWeights + bottomWeight;
    currentPixelSum = centerPixel * centerWeight + *(sumPtr) + rightPixelSum +  bottomPixel * bottomWeight;
#endif
    
    *(sumWeightsPtr) = bottomWeight; //next pixel to the bottom
    *(sumPtr) = centerPixel * bottomWeight; //next pixel to the bottom
    *(blockCurrentPixelPtr++) = (currentPixelSum + (currentPixelSumWeights >> 1) )/currentPixelSumWeights;

  }

  sumWeightsPtr = sumWeights;
  sumPtr = sum;
  
  //                           x
  // G pixel. uses filter type xx
  //
  // Uses information from previous row
  // No information from previous pixel
  // No information to store to next row
  // Storing information to next pixel
  
  
  centerPixel = *(blockCurrentPixelPtr);
  rightPixel = *(blockRightPixelPtr++);
#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
  rightWeight = lookupTablePtr[std::min(theMaxPos, abs(rightPixel - centerPixel))];
#else
  rightWeight = lookupTablePtr[abs(rightPixel - centerPixel)];
#endif  
#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
  currentPixelSumWeights = centerWeight + ((*(sumWeightsPtr++) + rightWeight) >> shiftEntry);
  currentPixelSum = centerPixel * centerWeight + ((*(sumPtr++) + rightPixel * rightWeight) >> shiftEntry);
#else
  currentPixelSumWeights = centerWeight + *(sumWeightsPtr++) + rightWeight;
  currentPixelSum = centerPixel * centerWeight + *(sumPtr++) + rightPixel * rightWeight;
#endif
  
  rightPixelSumWeights = rightWeight; //next pixel to the right
  rightPixelSum = centerPixel * rightWeight; //next pixel to the right
  
  *(blockCurrentPixelPtr++) = (currentPixelSum + (currentPixelSumWeights >> 1) )/currentPixelSumWeights;
  
  for (Int i = 1; i < (uiWidth - 1); i++)
  {
    //                            x
    // H pixel. uses filter type xxx
    //
    // Uses information from previous row
    // Uses information from previous pixel
    // No information to store to next row
    // Storing information to next pixel

    centerPixel = rightPixel;
    rightPixel = *(blockRightPixelPtr++);
#if BILATERAL_FILTER_ONLY_REF_NZFILTERCOEFF
    rightWeight = lookupTablePtr[std::min(theMaxPos, abs(rightPixel - centerPixel))];
#else
    rightWeight = lookupTablePtr[abs(rightPixel - centerPixel)];
#endif    

#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
    currentPixelSumWeights = centerWeight + ((*(sumWeightsPtr++) + rightWeight + rightPixelSumWeights) >> shiftEntry);
    currentPixelSum = centerPixel * centerWeight + ((*(sumPtr++) + rightPixelSum + rightPixel * rightWeight) >> shiftEntry);
#else
    currentPixelSumWeights = centerWeight + *(sumWeightsPtr++) + rightWeight + rightPixelSumWeights;
    currentPixelSum = centerPixel * centerWeight + *(sumPtr++) + rightPixelSum + rightPixel * rightWeight;
#endif

    rightPixelSumWeights = rightWeight; //next pixel to the right
    rightPixelSum = centerPixel * rightWeight; //next pixel to the right
    
    *(blockCurrentPixelPtr++) = (currentPixelSum + (currentPixelSumWeights >> 1) )/currentPixelSumWeights;
  }

  //                            x
  // I pixel. uses filter type xx
  //
  // Uses information from previous row
  // Uses information from previous pixel
  // No information to store to next row
  // No information to store to nex pixel
  
  centerPixel = rightPixel;
#if BILATERAL_FILTER_FIX_OVERFLOW_FOR_16BITS
  currentPixelSumWeights = centerWeight + ((*(sumWeightsPtr) + rightPixelSumWeights) >> shiftEntry);
  currentPixelSum = centerPixel * centerWeight + ((*(sumPtr) + rightPixelSum) >> shiftEntry);
#else
  currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightPixelSumWeights;
  currentPixelSum = centerPixel * centerWeight + *(sumPtr) + rightPixelSum;
#endif
  *(blockCurrentPixelPtr) = (currentPixelSum + (currentPixelSumWeights >> 1) )/currentPixelSumWeights;

}

Void TComBilateralFilter::bilateralFilterIntra(TComDataCU *pcCU, UInt uiWidth, UInt uiHeight, Pel *piReco, UInt uiStride, Int qp)
{
  UInt uiMinSize = std::min(uiWidth, uiHeight);
  Short *tempblock = new Short[ uiWidth * uiHeight ];
  
  for (UInt j = 0; j < uiHeight; j++)   
  {
    memcpy(tempblock + j * uiWidth, piReco + j * uiStride, uiWidth * sizeof(Short));
  }
  smoothBlockBilateralFilter(pcCU, uiWidth, uiHeight, tempblock, uiMinSize, 0, qp);
  for (UInt j = 0; j < uiHeight; j++)
  {
    memcpy(piReco + j * uiStride, tempblock + j * uiWidth, uiWidth * sizeof(Short));
  }
  delete[] tempblock;
}

Void TComBilateralFilter::bilateralFilterInter(TComDataCU *pcCU, UInt uiWidth, UInt uiHeight, Pel *piResi, UInt uiStrideRes, Pel *piPred, UInt uiPredStride, Pel *piReco, UInt uiRecStride, Int clipbd, Int qp)
{
  UInt uiMinSize = std::min(uiWidth, uiHeight);
  Short *tempblock = new Short[ uiWidth * uiHeight ];
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
  for (UInt j = 0; j < uiHeight; j++)
  {
    memcpy(tempblock + j * uiWidth, piReco + j * uiRecStride, uiWidth * sizeof(Short));
  }
  smoothBlockBilateralFilter(pcCU, uiWidth, uiHeight, tempblock, uiMinSize, 1, qp);
  for (UInt j = 0; j < uiHeight; j++)
  {
    memcpy(piReco + j * uiRecStride, tempblock + j * uiWidth, uiWidth * sizeof(Short));
  }
  delete[] tempblock;

  // need to be performed if residual  is used
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

