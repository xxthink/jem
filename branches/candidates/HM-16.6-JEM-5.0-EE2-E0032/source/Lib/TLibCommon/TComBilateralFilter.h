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
#ifndef TComBilateralFilter_h
#define TComBilateralFilter_h

#include "TComPattern.h"
#include "CommonDef.h"


class TComBilateralFilter
{
private:
  static TComBilateralFilter* m_bilateralFilterInstance;
  TComBilateralFilter();

public:
  static const Int NUM_OF_SPATIAL_SIGMA = 1;
  static const Int SpatialSigmaValues[NUM_OF_SPATIAL_SIGMA];
#if BILATERAL_FILTER_REUSE_INTRA_PART
  static const Int spatialSigmaBlockLengthOffsets[5];
#else
  static const Int spatialSigmaBlockLengthOffsets[6];
#endif
  ~TComBilateralFilter();
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE
  UShort** m_bilateralFilterTable;
#else
  UShort*** m_bilateralFilterTable;
#endif
#if BILATERAL_FILTER_MULTIPLY_CENTER_VALUE
  Int m_bilateralCenterWeightTable[6];
#endif

  static TComBilateralFilter* instance();
  Void createBilateralFilterTable(Int qp);
  Void smoothBlockBilateralFilter(TComDataCU* pcCU, UInt uiWidth, UInt uiHeight, Short block[], Int length, Int optimalSpatialSigmaIndex, Int qp);
  Void bilateralFilterInter(TComDataCU *pcCU, UInt uiWidth, UInt uiHeight, Pel *piResi, UInt uiStrideRes, Pel *piPred, UInt uiPredStride, Pel *piReco, UInt uiRecStride, Int clipbd, Int qp);
  Void bilateralFilterIntra(TComDataCU *pcCU, UInt uiWidth, UInt uiHeight, Pel *piReco, UInt uiStride, Int qp);
};

#endif /* TComBilateralFilter_h */
