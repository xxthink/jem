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

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()
: m_pLumaRecBuffer(0)
, m_iLumaRecStride(0)
{
  m_piYuvExt = NULL;
#if BIO 
#define BIO_TEMP_BUFFER_SIZE      (MAX_CU_SIZE+4)*(MAX_CU_SIZE+4) 
    m_pGradX0 = new Pel [BIO_TEMP_BUFFER_SIZE];
  m_pGradY0 = new Pel [BIO_TEMP_BUFFER_SIZE];
  m_pGradX1 = new Pel [BIO_TEMP_BUFFER_SIZE];
  m_pGradY1 = new Pel [BIO_TEMP_BUFFER_SIZE];
  m_pPred0  = new Pel [BIO_TEMP_BUFFER_SIZE];
  m_pPred1  = new Pel [BIO_TEMP_BUFFER_SIZE];
  iRefListIdx = -1;  
#endif
#if QC_FRUC_MERGE
  m_cFRUCRDCost.init();
#endif

#if QC_SUB_PU_TMVP_EXT
  m_cMvFieldSP[0] = new TComMvField[MAX_NUM_SPU_W*MAX_NUM_SPU_W*2];
  m_cMvFieldSP[1] = new TComMvField[MAX_NUM_SPU_W*MAX_NUM_SPU_W*2];
  m_uhInterDirSP[0] = new UChar[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
  m_uhInterDirSP[1] = new UChar[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
#endif

}

TComPrediction::~TComPrediction()
{
#if BIO 
  if( m_pGradX0 != NULL )     {delete [] m_pGradX0 ; m_pGradX0= NULL;}
  if( m_pGradY0 != NULL )     {delete [] m_pGradY0 ; m_pGradY0= NULL;}
  if( m_pGradX1 != NULL )     {delete [] m_pGradX1 ; m_pGradX1= NULL;}
  if( m_pGradY1 != NULL )     {delete [] m_pGradY1 ; m_pGradY1= NULL;}
  if( m_pPred0  != NULL )     {delete [] m_pPred0  ; m_pPred0 = NULL;}
  if( m_pPred1  != NULL )     {delete [] m_pPred1  ; m_pPred1 = NULL;}
#endif
#if QC_SUB_PU_TMVP_EXT
  for (UInt ui=0;ui<2;ui++)
  {
    if( m_cMvFieldSP[ui] != NULL )
    {
      delete [] m_cMvFieldSP[ui];
      m_cMvFieldSP[ui] = NULL;
    }
    if( m_uhInterDirSP[ui] != NULL )
    {
      delete [] m_uhInterDirSP[ui];
      m_uhInterDirSP[ui] = NULL;
    }
  }
#endif
  
  delete[] m_piYuvExt;

  m_acYuvPred[0].destroy();
  m_acYuvPred[1].destroy();

  m_cYuvPredTemp.destroy();

#if QC_FRUC_MERGE
  m_cYuvPredFrucTemplate[0].destroy();
  m_cYuvPredFrucTemplate[1].destroy();
#endif

  if( m_pLumaRecBuffer )
  {
    delete [] m_pLumaRecBuffer;
  }
  
  Int i, j;
  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 4; j++)
    {
      m_filteredBlock[i][j].destroy();
    }
    m_filteredBlockTmp[i].destroy();
  }
}

Void TComPrediction::initTempBuff()
{
  if( m_piYuvExt == NULL )
  {
#if QC_LARGE_CTU
    Int extWidth  = g_uiMaxCUWidth + 16; 
    Int extHeight = g_uiMaxCUHeight + 1;
#else
    Int extWidth  = MAX_CU_SIZE + 16; 
    Int extHeight = MAX_CU_SIZE + 1;
#endif
    Int i, j;
    for (i = 0; i < 4; i++)
    {
#if BIO
        m_filteredBlockTmp[i].create(extWidth+ 4 , extHeight + 7 + 4);
#else
    m_filteredBlockTmp[i].create(extWidth, extHeight + 7);
#endif
      for (j = 0; j < 4; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight);
      }
    }
#if QC_LARGE_CTU
    m_iYuvExtHeight  = ((g_uiMaxCUHeight + 2) << 4);
    m_iYuvExtStride = ((g_uiMaxCUWidth  + 8) << 4);
#else
    m_iYuvExtHeight  = ((MAX_CU_SIZE + 2) << 4);
    m_iYuvExtStride = ((MAX_CU_SIZE  + 8) << 4);
#endif
    m_piYuvExt = new Int[ m_iYuvExtStride * m_iYuvExtHeight ];

    // new structure
#if QC_LARGE_CTU
#if BIO
    m_acYuvPred[0] .create( g_uiMaxCUWidth + 4, g_uiMaxCUHeight + 4);
    m_acYuvPred[1] .create( g_uiMaxCUWidth + 4, g_uiMaxCUHeight + 4);
#else
  m_acYuvPred[0] .create( g_uiMaxCUWidth, g_uiMaxCUHeight );
    m_acYuvPred[1] .create( g_uiMaxCUWidth, g_uiMaxCUHeight );
#endif

    m_cYuvPredTemp.create( g_uiMaxCUWidth, g_uiMaxCUHeight );
#else
    m_acYuvPred[0] .create( MAX_CU_SIZE, MAX_CU_SIZE );
    m_acYuvPred[1] .create( MAX_CU_SIZE, MAX_CU_SIZE );

    m_cYuvPredTemp.create( MAX_CU_SIZE, MAX_CU_SIZE );
#endif
#if QC_FRUC_MERGE
    m_cYuvPredFrucTemplate[0].create( g_uiMaxCUWidth, g_uiMaxCUHeight );
    m_cYuvPredFrucTemplate[1].create( g_uiMaxCUWidth, g_uiMaxCUHeight );
#endif
  }

#if QC_LARGE_CTU
  if (m_iLumaRecStride != (g_uiMaxCUWidth>>1) + 1)
  {
    m_iLumaRecStride =  (g_uiMaxCUWidth>>1) + 1;
#else
  if (m_iLumaRecStride != (MAX_CU_SIZE>>1) + 1)
  {
    m_iLumaRecStride =  (MAX_CU_SIZE>>1) + 1;
#endif
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Int[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }

#if QC_LMCHROMA
  Int shift = g_bitDepthY + 4; //??????????
  for( Int i = 32; i < 64; i++ )
  {
    m_uiaLMShift[i-32] = ( ( 1 << shift ) + i/2 ) / i;
  }
#endif

#if QC_IC
  m_uiaICShift[0] = 0;
  for( Int i = 1; i < 64; i++ )
  {
    m_uiaICShift[i] = ( (1 << 15) + i/2 ) / i;
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
Pel TComPrediction::predIntraGetPredValDC( Int* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft )
{
  assert(iWidth > 0 && iHeight > 0);
  Int iInd, iSum = 0;
  Pel pDcVal;

  if (bAbove)
  {
    for (iInd = 0;iInd < iWidth;iInd++)
    {
      iSum += pSrc[iInd-iSrcStride];
    }
  }
  if (bLeft)
  {
    for (iInd = 0;iInd < iHeight;iInd++)
    {
      iSum += pSrc[iInd*iSrcStride-1];
    }
  }

  if (bAbove && bLeft)
  {
    pDcVal = (iSum + iWidth) / (iWidth + iHeight);
  }
  else if (bAbove)
  {
    pDcVal = (iSum + iWidth/2) / iWidth;
  }
  else if (bLeft)
  {
    pDcVal = (iSum + iHeight/2) / iHeight;
  }
  else
  {
    pDcVal = pSrc[-1]; // Default DC value already calculated and placed in the prediction array if no neighbors are available
  }
  
  return pDcVal;
}

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 * \param dirMode the intra prediction mode index
 * \param blkAboveAvailable boolean indication if the block above is available
 * \param blkLeftAvailable boolean indication if the block to the left is available
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
Void TComPrediction::xPredIntraAng(Int bitDepth, Int* pSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter
#if QC_INTRA_4TAP_FILTER
                                   , Bool bLuma 
                                   , Bool bUse4TapFilter
#endif
                                   )
{
  Int k,l;
  Int blkSize        = width;
  Pel* pDst          = rpDst;

  // Map the mode index to main prediction direction and angle
  assert( dirMode > 0 ); //no planar
  Bool modeDC        = dirMode < 2;
#if QC_USE_65ANG_MODES
  Bool modeHor       = !modeDC && (dirMode < DIA_IDX);
#else
  Bool modeHor       = !modeDC && (dirMode < 18);
#endif
  Bool modeVer       = !modeDC && !modeHor;
  Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
  Int absAng         = abs(intraPredAngle);
  Int signAng        = intraPredAngle < 0 ? -1 : 1;

  // Set bitshifts and scale the angle parameter to block size
#if QC_USE_65ANG_MODES
  Int angTable[17]    = {0,    1,    2,    3,    5,    7,    9,   11,   13,   15,   17,   19,   21,   23,   26,   29,   32};
  Int invAngTable[17] = {0, 8192, 4096, 2731, 1638, 1170,  910,  745,  630,  546,  482,  431,  390,  356,  315,  282,  256}; // (256 * 32) / Angle
#else
  Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
  Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
#endif
  Int invAngle       = invAngTable[absAng];
  absAng             = angTable[absAng];
  intraPredAngle     = signAng * absAng;

  // Do the DC prediction
  if (modeDC)
  {
    Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

    for (k=0;k<blkSize;k++)
    {
      for (l=0;l<blkSize;l++)
      {
        pDst[k*dstStride+l] = dcval;
      }
    }
  }

  // Do angular predictions
  else
  {
    Pel* refMain;
    Pel* refSide;
    Pel  refAbove[2*MAX_CU_SIZE+1];
    Pel  refLeft[2*MAX_CU_SIZE+1];

    // Initialise the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      for (k=0;k<blkSize+1;k++)
      {
        refAbove[k+blkSize-1] = pSrc[k-srcStride-1];
      }
      for (k=0;k<blkSize+1;k++)
      {
        refLeft[k+blkSize-1] = pSrc[(k-1)*srcStride-1];
      }
      refMain = (modeVer ? refAbove : refLeft) + (blkSize-1);
      refSide = (modeVer ? refLeft : refAbove) + (blkSize-1);

      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)
      for (k=-1; k>blkSize*intraPredAngle>>5; k--)
      {
        invAngleSum += invAngle;
        refMain[k] = refSide[invAngleSum>>8];
      }
    }
    else
    {
      for (k=0;k<2*blkSize+1;k++)
      {
        refAbove[k] = pSrc[k-srcStride-1];
      }
      for (k=0;k<2*blkSize+1;k++)
      {
        refLeft[k] = pSrc[(k-1)*srcStride-1];
      }
      refMain = modeVer ? refAbove : refLeft;
      refSide = modeVer ? refLeft  : refAbove;
    }

    if (intraPredAngle == 0)
    {
      for (k=0;k<blkSize;k++)
      {
        for (l=0;l<blkSize;l++)
        {
          pDst[k*dstStride+l] = refMain[l+1];
        }
      }

      if ( bFilter )
      {
        for (k=0;k<blkSize;k++)
        {
          pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 1) );
        }
      }
    }
    else
    {
      Int deltaPos=0;
      Int deltaInt;
      Int deltaFract;
      Int refMainIndex;

      for (k=0;k<blkSize;k++)
      {
        deltaPos += intraPredAngle;
        deltaInt   = deltaPos >> 5;
        deltaFract = deltaPos & (32 - 1);

        if (deltaFract)
        {
          // Do linear filtering
#if QC_INTRA_4TAP_FILTER
          if (bUse4TapFilter)
          {
            Int p[4];
            const Pel nMin = 0, nMax = (1 << bitDepth)-1;
            Int *f = (width<=8) ? g_aiIntraCubicFilter[deltaFract]:g_aiIntraGaussFilter[deltaFract];
            
            for (l=0;l<blkSize;l++)
            {
              refMainIndex        = l+deltaInt+1;

              p[1] = refMain[refMainIndex];
              p[2] = refMain[refMainIndex+1];

              p[0] = l==0 ? p[1] : refMain[refMainIndex-1];
              p[3] = l==(blkSize-1) ? p[2] : refMain[refMainIndex+2];

              pDst[k*dstStride+l] =  (Pel)( ( f[0]*p[0] + f[1]*p[1] + f[2]*p[2] + f[3]*p[3] + 128 ) >> 8 );

              if( width<=8 )
              {
                pDst[k*dstStride+l] =  Clip3( nMin, nMax, pDst[k*dstStride+l] );
              }
            }
          }
          else
          {
#endif
          for (l=0;l<blkSize;l++)
          {
            refMainIndex        = l+deltaInt+1;
            pDst[k*dstStride+l] = (Pel) ( ((32-deltaFract)*refMain[refMainIndex]+deltaFract*refMain[refMainIndex+1]+16) >> 5 );
          }
#if QC_INTRA_4TAP_FILTER
          }
#endif
        }
        else
        {
          // Just copy the integer samples
          for (l=0;l<blkSize;l++)
          {
            pDst[k*dstStride+l] = refMain[l+deltaInt+1];
          }
        }
      }
#if QC_USE_65ANG_MODES
      if ( bFilter && absAng<=1 )
      {
        for (k=0;k<blkSize;k++)
        {
          pDst[k*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[k*dstStride] + (( refSide[k+1] - refSide[0] ) >> 2) );
        }
      }
#endif
    }

    // Flip the block if this is the horizontal mode
    if (modeHor)
    {
      Pel  tmp;
      for (k=0;k<blkSize-1;k++)
      {
        for (l=k+1;l<blkSize;l++)
        {
          tmp                 = pDst[k*dstStride+l];
          pDst[k*dstStride+l] = pDst[l*dstStride+k];
          pDst[l*dstStride+k] = tmp;
        }
      }
    }
  }
}

Void TComPrediction::predIntraLumaAng(TComPattern* pcTComPattern, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft 
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
                                      )
{
  Pel *pDst = piPred;
  Int *ptrSrc;

  assert( g_aucConvertToBit[ iWidth ] >= 0 ); //   4x  4
#if !QC_LARGE_CTU
  assert( g_aucConvertToBit[ iWidth ] <= 5 ); // 128x128
#endif
  assert( iWidth == iHeight  );

  ptrSrc = pcTComPattern->getPredictorPtr( uiDirMode, g_aucConvertToBit[ iWidth ] + 2, m_piYuvExt 
#if QC_USE_65ANG_MODES
    , bUseExtIntraAngModes
#endif
    );

  // get starting pixel in block
  Int sw = 2 * iWidth + 1;

  // Create the prediction
  if ( uiDirMode == PLANAR_IDX )
  {
    xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
  }
  else
  {
    if ( (iWidth > 16) || (iHeight > 16) )
    {
      xPredIntraAng(g_bitDepthY, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false
#if QC_INTRA_4TAP_FILTER
        , true 
        , bUse4TapFilter
#endif
        );
    }
    else
    {
      xPredIntraAng(g_bitDepthY, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, true
#if QC_INTRA_4TAP_FILTER
        , true
        , bUse4TapFilter
#endif
        );
#if CU_LEVEL_MPI
    if( !pcCU->getMPIIdx(uiAbsPartIdx) && (uiDirMode == DC_IDX ) && bAbove && bLeft )
#else
      if( (uiDirMode == DC_IDX ) && bAbove && bLeft )
#endif
      {
        xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight);
      }
    }
#if INTRA_BOUNDARY_FILTER
    if( bUseBoundaryFilter )
  {
#if QC_USE_65ANG_MODES
      if( uiDirMode == VDIA_IDX )
#else
      if( uiDirMode == 34 )
#endif
      {
        xIntraPredFilteringMode34( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight);
      }
      else  if( uiDirMode == 2 )
      {
        xIntraPredFilteringMode02( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight);
      }
#if QC_USE_65ANG_MODES
      else if( ( uiDirMode<=10 && uiDirMode>2 ) || ( uiDirMode>=(VDIA_IDX-8) && uiDirMode<VDIA_IDX ) )
#else
      else if( ( uiDirMode<=6 && uiDirMode>2 ) || ( uiDirMode>=30 && uiDirMode<34 ) )
#endif
      {
        xIntraPredFilteringModeDGL( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode );
      }
    }
#endif
  }
#if CU_LEVEL_MPI
  if( pcCU->getMPIIdx(uiAbsPartIdx) && pcCU->getCUPelX() && pcCU->getCUPelY() )
  {
      Pel* pRec = pcCU->getPic()->getPicYuvRec()->getLumaAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
    Int iStrideRec= pcCU->getPic()->getPicYuvRec()->getStride();
    PartSize eSize         = pcCU->getPartitionSize( uiAbsPartIdx );
      Int idexMPI = pcCU->getMPIIdx(uiAbsPartIdx) ;
    if (idexMPI>3) idexMPI = 0;
      idexMPI += (eSize == SIZE_NxN?4:0) ;
    xMPIredFiltering( pRec, iStrideRec, pDst, uiStride, iWidth, iHeight, idexMPI );
  }
#endif
}

// Angular chroma
Void TComPrediction::predIntraChromaAng( Int* piSrc, UInt uiDirMode, Pel* piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft 
#if QC_INTRA_4TAP_FILTER
                                        , Bool bUse4TapFilter
#endif
#if CU_LEVEL_MPI
, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiChromaIdx
#endif
                                        )
{
  Pel *pDst = piPred;
  Int *ptrSrc = piSrc;

  // get starting pixel in block
  Int sw = 2 * iWidth + 1;

  if ( uiDirMode == PLANAR_IDX )
  {
    xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
  }
  else
  {
    // Create the prediction
    xPredIntraAng(g_bitDepthC, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false
#if QC_INTRA_4TAP_FILTER
      , false 
      , bUse4TapFilter
#endif
      );
  }
#if CU_LEVEL_MPI
  if( pcCU->getMPIIdx(uiAbsPartIdx) && pcCU->getCUPelX() && pcCU->getCUPelY() )
  { 
      Pel* pRec = pcCU->getPic()->getPicYuvRec()->getCbAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
    if (uiChromaIdx == 1)
      pRec = pcCU->getPic()->getPicYuvRec()->getCrAddr(pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);
    Int iStrideRec= pcCU->getPic()->getPicYuvRec()->getCStride();
    PartSize eSize         = pcCU->getPartitionSize( uiAbsPartIdx );
      Int idexMPI = pcCU->getMPIIdx(uiAbsPartIdx) ;
    if (idexMPI>3) idexMPI = 0;
      idexMPI += (eSize == SIZE_NxN?4:0) ;
    xMPIredFiltering( pRec, iStrideRec, pDst, uiStride, iWidth, iHeight, idexMPI );
  }
#endif
}

/** Function for checking identical motion.
 * \param TComDataCU* pcCU
 * \param UInt PartAddr
 */
Bool TComPrediction::xCheckIdenticalMotion ( TComDataCU* pcCU, UInt PartAddr )
{
  if( pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred() )
  {
#if QC_FRUC_MERGE
    if( pcCU->getFRUCMgrMode( PartAddr ) )
      return false;
#endif
    if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)
    {
      Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
      Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
      if(RefPOCL0 == RefPOCL1 && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr))
      {
        return true;
      }
    }
  }
  return false;
}

#if QC_SUB_PU_TMVP
Void TComPrediction::xGetSubPUAddrAndMerge(TComDataCU* pcCU, UInt uiPartAddr, Int iSPWidth, Int iSPHeight, Int iNumSPInOneLine, Int iNumSP, UInt* uiMergedSPW, UInt* uiMergedSPH, UInt* uiSPAddr )
{
  for (Int i = 0; i < iNumSP; i++)
  {
    uiMergedSPW[i] = iSPWidth;
    uiMergedSPH[i] = iSPHeight;
    pcCU->getSPAbsPartIdx(uiPartAddr, iSPWidth, iSPHeight, i, iNumSPInOneLine, uiSPAddr[i]);
  }

  // horizontal sub-PU merge
  for (Int i=0; i<iNumSP; i++)
  {
    if (i % iNumSPInOneLine == iNumSPInOneLine - 1 || uiMergedSPW[i]==0 || uiMergedSPH[i]==0)
    {
      continue;
    }
    for (Int j=i+1; j<i+iNumSPInOneLine-i%iNumSPInOneLine; j++)
    {
      if (xCheckTwoSPMotion(pcCU, uiSPAddr[i], uiSPAddr[j]))
      {
        uiMergedSPW[i] += iSPWidth;
        uiMergedSPW[j] = uiMergedSPH[j] = 0;
      }
      else
      {
        break;
      }
    }
  }
  //vertical sub-PU merge
  for (Int i=0; i<iNumSP-iNumSPInOneLine; i++)
  {
    if (uiMergedSPW[i]==0 || uiMergedSPH[i]==0)
    {
      continue;
    }
    for (Int j=i+iNumSPInOneLine; j<iNumSP; j+=iNumSPInOneLine)
    {
      if (xCheckTwoSPMotion(pcCU, uiSPAddr[i], uiSPAddr[j]) && uiMergedSPW[i]==uiMergedSPW[j])
      {
        uiMergedSPH[i] += iSPHeight;
        uiMergedSPH[j] = uiMergedSPW[j] = 0;
      }
      else
      {
        break;
      }
    }
  }
}
Bool TComPrediction::xCheckTwoSPMotion ( TComDataCU* pcCU, UInt PartAddr0, UInt PartAddr1 )
{
  if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr1))
  {
    return false;
  }
  if( pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr1))
  {
    return false;
  }

  if (pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr0) >= 0)
  {
    if (pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr1))
    {
      return false;
    }
  }

  if (pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr0) >= 0)
  {
    if (pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr1))
    {
      return false;
    }
  }
  return true;
}
#endif

Void TComPrediction::motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList, Int iPartIdx )
{
  Int         iWidth;
  Int         iHeight;
  UInt        uiPartAddr;
  if ( iPartIdx >= 0 )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );
    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP()
#if QC_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred
#if BIO
    ,  false 
#endif
    , true   );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred  );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() 
#if QC_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
#if QC_SUB_PU_TMVP
#if QC_SUB_PU_TMVP_EXT
      if ( pcCU->getMergeType(uiPartAddr)== MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType(uiPartAddr)== MGR_TYPE_SUBPU_TMVP_EXT)  
#else
      if ( pcCU->getMergeType(uiPartAddr)== MGR_TYPE_SUBPU_TMVP )  
#endif
      {
        Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;

        pcCU->getSPPara(iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);

#if QC_LARGE_CTU
        UInt uiW[MAX_NUM_SPU_W*MAX_NUM_SPU_W], uiH[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
        UInt uiSPAddr[MAX_NUM_SPU_W*MAX_NUM_SPU_W];

#else
        UInt uiW[256], uiH[256];
        UInt uiSPAddr[256];
#endif

        xGetSubPUAddrAndMerge(pcCU, uiPartAddr, iSPWidth, iSPHeight, iNumSPInOneLine, iNumSP, uiW, uiH, uiSPAddr);

        //MC
        for (Int i = 0; i < iNumSP; i++)
        {
          if (uiW[i]==0 || uiH[i]==0)
          {
            continue;
          }

          if(xCheckIdenticalMotion( pcCU, uiSPAddr[i] )/*0*/)
          {
            xPredInterUni (pcCU, uiSPAddr[i], uiW[i], uiH[i], REF_PIC_LIST_0, pcYuvPred  );
          }
          else
          {
            xPredInterBi  (pcCU, uiSPAddr[i], uiW[i], uiH[i], pcYuvPred);
          }
        }
      }
      else
      {
#endif
        if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
        {
          xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred  );
        }
        else
        {
          xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
        }
#if QC_SUB_PU_TMVP
      }
#endif
    }
    return;
  }

  for ( iPartIdx = 0; iPartIdx < pcCU->getNumPartitions(); iPartIdx++ )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );

    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP()
#if QC_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred 
#if BIO
    ,  false 
#endif
    , true    );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred  );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP()
#if QC_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    { 
#if QC_SUB_PU_TMVP
#if QC_SUB_PU_TMVP_EXT
      if ( pcCU->getMergeType(uiPartAddr)== MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType(uiPartAddr)== MGR_TYPE_SUBPU_TMVP_EXT)  
#else
      if (pcCU->getMergeType(uiPartAddr) == MGR_TYPE_SUBPU_TMVP)  
#endif
      {
        Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;

        pcCU->getSPPara(iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);

#if QC_LARGE_CTU
        UInt uiW[MAX_NUM_SPU_W*MAX_NUM_SPU_W], uiH[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
        UInt uiSPAddr[MAX_NUM_SPU_W*MAX_NUM_SPU_W];
#else
        UInt uiW[256], uiH[256];
        UInt uiSPAddr[256];
#endif

        xGetSubPUAddrAndMerge(pcCU, uiPartAddr, iSPWidth, iSPHeight, iNumSPInOneLine, iNumSP, uiW, uiH, uiSPAddr);
        //MC
        for (Int i = 0; i < iNumSP; i++)
        {
          if (uiW[i]==0 || uiH[i]==0)
          {
            continue;
          }
          if( xCheckIdenticalMotion( pcCU, uiSPAddr[i] )/*0*/)
          {
            xPredInterUni (pcCU, uiSPAddr[i], uiW[i], uiH[i], REF_PIC_LIST_0, pcYuvPred   );
          }
          else
          {
            xPredInterBi  (pcCU, uiSPAddr[i], uiW[i], uiH[i], pcYuvPred);
          }
        }
      }
      else
      {
#endif
        if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
        {
          xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred   );
        }
        else
        {
          xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
        }
#if QC_SUB_PU_TMVP
      }
#endif
    }
  }
  return;
}

#if QC_OBMC
/** Function for sub-block based Overlapped Block Motion Compensation (OBMC).
 *
 * This function can:
 * 1. Perform sub-block OBMC for a CU.
 * 2. Before motion estimation, subtract (scaled) predictors generated by applying neighboring motions to current CU/PU from the original signal of current CU/PU,
 *    to make the motion estimation biased to OBMC.
 */
Void TComPrediction::subBlockOBMC( TComDataCU*  pcCU, UInt uiAbsPartIdx, TComYuv *pcYuvPred, TComYuv *pcYuvTmpPred1, TComYuv *pcYuvTmpPred2, Bool bOBMC4ME )
{
  if( !pcCU->getSlice()->getSPS()->getOBMC() || !pcCU->getOBMCFlag( uiAbsPartIdx ) )
  {
    return;
  }
  
  PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
  UInt uiWidth           = pcCU->getWidth ( uiAbsPartIdx );
  UInt uiHeight          = pcCU->getHeight( uiAbsPartIdx );
  UInt uiMinCUW          = pcCU->getPic()->getMinCUWidth();
  UInt uiOBMCBlkSize     = pcCU->getSlice()->getSPS()->getOBMCBlkSize();
  UInt uiMaxWidthInBlock = pcCU->getPic()->getNumPartInWidth();

  UInt uiHeightInBlock   = uiHeight / uiMinCUW;
  UInt uiWidthInBlock    = uiWidth / uiMinCUW;
  UInt uiStep            = uiOBMCBlkSize / uiMinCUW;
  UInt uiMaxCUDepth      = pcCU->getSlice()->getSPS()->getMaxCUDepth();
  UInt uiDepth           = uiMaxCUDepth - pcCU->getDepth( uiAbsPartIdx );

  UInt uiSubPartIdx      = 0;
  UInt uiZeroIdx         = pcCU->getZorderIdxInCU();
  UInt uiAbsPartIdxLCURaster = g_auiZscanToRaster[uiAbsPartIdx + uiZeroIdx];
  Bool bOBMCSimp             = ( uiWidth == 8 && ePartSize != SIZE_2Nx2N );

  Int  i1stPUWidth  = -1, i1stPUHeight = -1;
  UInt uiPartAddr   = 0;
#if QC_SUB_PU_TMVP
#if QC_SUB_PU_TMVP_EXT
  Bool bATMVP       = (pcCU->getMergeType( uiAbsPartIdx ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiAbsPartIdx ) == MGR_TYPE_SUBPU_TMVP_EXT);
#else
  Bool bATMVP       = pcCU->getMergeType( uiAbsPartIdx ) == MGR_TYPE_SUBPU_TMVP;
#endif
  Bool bNormal2Nx2N = (ePartSize == SIZE_2Nx2N && !bATMVP);
  Bool bSubMotion   = ePartSize == SIZE_NxN   || (ePartSize == SIZE_2Nx2N && bATMVP);
#else
  Bool bNormal2Nx2N = ePartSize == SIZE_2Nx2N;
  Bool bSubMotion   = ePartSize == SIZE_NxN;
#endif
#if QC_FRUC_MERGE
  Int nFrucRefineSize = max( pcCU->getWidth( 0 ) >> pcCU->getSlice()->getSPS()->getFRUCSmallBlkRefineDepth(), QC_FRUC_MERGE_REFINE_MINBLKSIZE );
  if( pcCU->getFRUCMgrMode( uiAbsPartIdx ) && ePartSize == SIZE_2Nx2N )
  {
    bNormal2Nx2N = false;
    bSubMotion = true;
  }
#endif
  Bool bVerticalPU  = ( ePartSize == SIZE_2NxN || ePartSize == SIZE_2NxnU || ePartSize == SIZE_2NxnD );
  Bool bHorizonalPU = ( ePartSize == SIZE_Nx2N || ePartSize == SIZE_nLx2N || ePartSize == SIZE_nRx2N );
  Bool bAtmvpPU = false, bNormalTwoPUs = false;
#if QC_FRUC_MERGE
  Bool bFrucPU = false;
#endif
  Bool bTwoPUs  = ( bVerticalPU || bHorizonalPU );
  Int  iNeigPredDir = 0, iCurPredDir = 0;

  if( bTwoPUs )
  {
#if QC_SUB_PU_TMVP
    pcCU->getPartIndexAndSize( 1, uiPartAddr, i1stPUWidth, i1stPUHeight );
#if QC_SUB_PU_TMVP_EXT
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP_EXT);
#else
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP);
#endif
#endif
#if QC_FRUC_MERGE
    bFrucPU |= (pcCU->getFRUCMgrMode( uiPartAddr ) != QC_FRUC_MERGE_OFF );
#endif
    pcCU->getPartIndexAndSize( 0, uiPartAddr, i1stPUWidth, i1stPUHeight );
#if QC_SUB_PU_TMVP
#if QC_SUB_PU_TMVP_EXT
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP_EXT);
#else
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP );
#endif
#endif
#if QC_FRUC_MERGE
    bFrucPU |= (pcCU->getFRUCMgrMode( uiPartAddr ) != QC_FRUC_MERGE_OFF );
#endif
    i1stPUWidth  /= pcCU->getPic()->getMinCUWidth();
    i1stPUHeight /= pcCU->getPic()->getMinCUWidth();    

    bNormalTwoPUs = !bAtmvpPU;
#if QC_FRUC_MERGE
    bNormalTwoPUs &= !bFrucPU;
#endif
  }

  Bool bCurrMotStored = false, bDiffMot[4]= { false, false, false, false };
  TComMvField cCurMvField[2], cNeigMvField[2];

  Int maxDir = bNormal2Nx2N ? 2 : 4;
  for( Int iSubX = 0; iSubX < uiWidthInBlock; iSubX += uiStep )
  {
    for( Int iSubY = 0; iSubY < uiHeightInBlock; iSubY += uiStep )
    {
      if( bNormal2Nx2N && iSubX && iSubY )
      {
        continue;
      }
      Bool bCURBoundary = ( iSubX == uiWidthInBlock  - 1 );
      Bool bCUBBoundary = ( iSubY == uiHeightInBlock - 1 );

      bCurrMotStored    = false;
      uiSubPartIdx      = g_auiRasterToZscan[uiAbsPartIdxLCURaster + iSubX + iSubY*uiMaxWidthInBlock] - uiZeroIdx;

      for( Int iDir = 0; iDir < maxDir; iDir++ ) //iDir: 0 - above, 1 - left, 2 - below, 3 - right
      {
        if( ( iDir == 3 && bCURBoundary ) || ( iDir == 2 && bCUBBoundary ) )
        {
          continue;
        }
 
        Bool bVerPUBound  = false;
        Bool bHorPUBound  = false;

        if( bNormal2Nx2N ) //skip unnecessary check for CU boundary
        {
          if( ( iDir == 1 && !iSubY && iSubX ) || ( iDir == 0 && !iSubX && iSubY ) )
          {
            continue;
          }
        }
        else
        {
          Bool bCheckNeig = bSubMotion || ( iSubX == 0 && iDir == 1 ) || ( iSubY == 0 && iDir == 0 ); //CU boundary or NxN or 2nx2n_ATMVP
          if( !bCheckNeig && bTwoPUs )
          {
#if QC_SUB_PU_TMVP
            if( bAtmvpPU )
            {
              //sub-PU boundary
#if QC_SUB_PU_TMVP_EXT
              bCheckNeig |= (pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP_EXT);
#else
              bCheckNeig |= ( pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP );
#endif
            }
#endif
#if QC_FRUC_MERGE
            if( bFrucPU )
            {
              //sub-PU boundary
              bCheckNeig |= ( pcCU->getFRUCMgrMode( uiSubPartIdx ) != QC_FRUC_MERGE_OFF );
            }
#endif
            if( !bCheckNeig )
            {
              //PU boundary
              bVerPUBound = bVerticalPU  && ( ( iDir == 2 && iSubY == i1stPUHeight - 1 ) || ( iDir == 0 && iSubY == i1stPUHeight ) );
              bHorPUBound = bHorizonalPU && ( ( iDir == 3 && iSubX == i1stPUWidth  - 1 ) || ( iDir == 1 && iSubX == i1stPUWidth ) );

              bCheckNeig  |= ( bVerPUBound || bHorPUBound );
            }
          }
          if( !bCheckNeig )
          {
            continue;
          }
        }
        
        Bool bCurSubBkFetched  = bNormalTwoPUs && ( ( bVerPUBound && iSubX ) || ( bHorPUBound && iSubY ) );

#if QC_SUB_PU_TMVP
#if QC_SUB_PU_TMVP_EXT
        Bool bSubBlockOBMCSimp = (bOBMCSimp || (( pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP_EXT) && ( 1 << pcCU->getSlice()->getSPS()->getSubPUTLog2Size() ) == 4 ));
#else
        Bool bSubBlockOBMCSimp = bOBMCSimp || ( pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP && ( 1 << pcCU->getSlice()->getSPS()->getSubPUTLog2Size() ) == 4 );
#endif
#else
        Bool bSubBlockOBMCSimp = bOBMCSimp;
#endif
#if QC_FRUC_MERGE
        bSubBlockOBMCSimp |= ( bOBMCSimp || ( pcCU->getFRUCMgrMode( uiSubPartIdx ) != QC_FRUC_MERGE_OFF && nFrucRefineSize == 4 ) );
#endif
        if( ( bCurSubBkFetched && bDiffMot[iDir] ) || pcCU->getNeigMotion( uiSubPartIdx, cNeigMvField, iNeigPredDir, iDir, cCurMvField, iCurPredDir, uiZeroIdx, bCurrMotStored ) )
        {
          Bool bFeAllSubBkIn1Line = false; //Fetch all sub-blocks in one row/column
          if( !bCurSubBkFetched )
          {
            //store temporary motion information
            pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiSubPartIdx, uiMaxCUDepth);
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cNeigMvField[0], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cNeigMvField[1], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->setInterDirSubParts( iNeigPredDir, uiSubPartIdx, 0, uiMaxCUDepth );
            if( bNormalTwoPUs )
            {
              bFeAllSubBkIn1Line = ( bHorPUBound || bVerPUBound );
              if( bFeAllSubBkIn1Line )
              {
                bDiffMot[iDir] = true;
              }
            }
          }
          UInt uiSubBlockWidth  = ( bFeAllSubBkIn1Line && bVerticalPU  ) ? uiWidth  : uiOBMCBlkSize;
          UInt uiSubBlockHeight = ( bFeAllSubBkIn1Line && !bVerticalPU ) ? uiHeight : uiOBMCBlkSize;
          //motion compensation and OBMC
          if( !bCurSubBkFetched )
          {
            xSubBlockMotionCompensation( pcCU, bFeAllSubBkIn1Line ? pcYuvTmpPred2 : pcYuvTmpPred1, uiSubPartIdx, uiSubBlockWidth, uiSubBlockHeight );
          }

          if( bOBMC4ME )
          {
            xSubtractOBMC( pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiOBMCBlkSize, uiOBMCBlkSize, iDir, bSubBlockOBMCSimp );
          }
          else
          {
            xSubblockOBMC( pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiOBMCBlkSize, uiOBMCBlkSize, iDir, bSubBlockOBMCSimp );
          }
          //recover motion information
          if( !bCurSubBkFetched )
          {
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cCurMvField[0], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cCurMvField[1], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->setInterDirSubParts( iCurPredDir, uiSubPartIdx, 0, uiMaxCUDepth );
            pcCU->setPartSizeSubParts( ePartSize, uiSubPartIdx,      uiMaxCUDepth );
          }
        }
      }
    }
  }
}

// Function for (weighted) averaging predictors of current block and predictors generated by applying neighboring motions to current block.
Void TComPrediction::xSubblockOBMC( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp )
{
  Int iDstStride  = pcYuvPredDst->getWidth();
  Int iDstCStride = pcYuvPredDst->getCStride();
  Int iSrcStride  = pcYuvPredSrc->getWidth();
  Int iSrcCStride = pcYuvPredSrc->getCStride();

  Pel *pDst   = pcYuvPredDst->getLumaAddr( uiAbsPartIdx );
  Pel *pSrc   = pcYuvPredSrc->getLumaAddr( uiAbsPartIdx );

  Pel *pDstCb = pcYuvPredDst->getCbAddr( uiAbsPartIdx ); 
  Pel *pDstCr = pcYuvPredDst->getCrAddr( uiAbsPartIdx );
  Pel *pSrcCb = pcYuvPredSrc->getCbAddr( uiAbsPartIdx ); 
  Pel *pSrcCr = pcYuvPredSrc->getCrAddr( uiAbsPartIdx );

  Int iDstPtrOffset = iDstStride, iScrPtrOffset = iSrcStride, iDstPtrOffsetC = iDstCStride, iScrPtrOffsetC = iSrcCStride, ioffsetDst = 1, ioffsetSrc = 1, ioffsetDstC = 1, ioffsetSrcC = 1;

  if( iDir ) //0: above; 1:left; 2: below; 3:right
  {
    if( iDir == 1 )  
    {
      //luma
      iDstPtrOffset = iScrPtrOffset = iDstPtrOffsetC = iScrPtrOffsetC = 1;
      ioffsetDst = iDstStride; ioffsetSrc = iSrcStride; ioffsetDstC = iDstCStride; ioffsetSrcC = iSrcCStride;
      iWidth = iHeight;
    }
    else if( iDir == 2 )
    {
      Int iHMinus1 = iHeight - 1;
      pDst += iHMinus1*iDstStride;
      pSrc += iHMinus1*iSrcStride;
      iDstPtrOffset = -iDstStride; iScrPtrOffset = -iSrcStride; iDstPtrOffsetC = -iDstCStride; iScrPtrOffsetC = -iSrcCStride; ioffsetDst = ioffsetSrc = ioffsetDstC = ioffsetSrcC = 1;
      //chroma
      Int iCHMinus1   = (iHeight >> 1) - 1;
      Int iDstCOffset = iCHMinus1 * iDstCStride, iSrcCOffset = iCHMinus1 * iSrcCStride;
      pDstCb  += iDstCOffset;
      pDstCr  += iDstCOffset;
      pSrcCb  += iSrcCOffset;
      pSrcCr  += iSrcCOffset;
    }
    else
    {
      Int iWMinus1 = iWidth - 1 , iCWMinus1 = (iWidth>>1) - 1;
      //luma
      pDst += iWMinus1;
      pSrc += iWMinus1;
      iDstPtrOffset = iScrPtrOffset = iDstPtrOffsetC = iScrPtrOffsetC = -1;
      ioffsetDst = iDstStride; ioffsetSrc = iSrcStride; ioffsetDstC = iDstCStride; ioffsetSrcC = iSrcCStride;

      //chroma
      pDstCb  += iCWMinus1;
      pDstCr  += iCWMinus1;
      pSrcCr  += iCWMinus1;
      pSrcCb  += iCWMinus1;
      iWidth  = iHeight;
    }
  }

  //luma
  Pel *pDst1 = pDst  + iDstPtrOffset;
  Pel *pDst2 = pDst1 + iDstPtrOffset;
  Pel *pDst3 = pDst2 + iDstPtrOffset;
  Pel *pSrc1 = pSrc  + iScrPtrOffset;
  Pel *pSrc2 = pSrc1 + iScrPtrOffset;
  Pel *pSrc3 = pSrc2 + iScrPtrOffset;

  for( Int i = 0; i < iWidth; i++ )
  {
    *pDst  = ( (*pDst) * 3  + (*pSrc)  + 2 ) >> 2;
    *pDst1 = ( (*pDst1) * 7 + (*pSrc1) + 4 ) >> 3;
    pDst += ioffsetDst; pDst1 += ioffsetDst; pSrc += ioffsetSrc; pSrc1 += ioffsetSrc;

    if( !bOBMCSimp )
    {
      *pDst2 = ( (*pDst2) * 15 + (*pSrc2) +  8 ) >> 4;
      *pDst3 = ( (*pDst3) * 31 + (*pSrc3) + 16 ) >> 5;
      pDst2 += ioffsetDst; pDst3 += ioffsetDst; pSrc2 += ioffsetSrc; pSrc3 += ioffsetSrc;
    }
  }

  //chroma
  iWidth >>= 1;
  Pel *pDstCb1 = pDstCb + iDstPtrOffsetC; 
  Pel *pDstCr1 = pDstCr + iDstPtrOffsetC;
  Pel *pSrcCb1 = pSrcCb + iScrPtrOffsetC; 
  Pel *pSrcCr1 = pSrcCr + iScrPtrOffsetC; 

  for( Int i = 0; i < iWidth; i++ )
  {
    *pDstCb = ( (*pDstCb)* 3 + (*pSrcCb) + 2 ) >> 2;
    *pDstCr = ( (*pDstCr)* 3 + (*pSrcCr) + 2 ) >> 2;
    pDstCb += ioffsetDstC; pDstCr += ioffsetDstC; pSrcCb += ioffsetSrcC; pSrcCr += ioffsetSrcC;

    if( !bOBMCSimp )
    {
      *pDstCb1 = ( (*pDstCb1)* 7 + (*pSrcCb1) + 4 ) >> 3;
      *pDstCr1 = ( (*pDstCr1)* 7 + (*pSrcCr1) + 4 ) >> 3;
      pDstCb1 += ioffsetDstC; pDstCr1 += ioffsetDstC; pSrcCb1 += ioffsetSrcC; pSrcCr1 += ioffsetSrcC;
    }
  }
}

// Function for subtracting (scaled) predictors generated by applying neighboring motions to current block from the original signal of current block.
Void TComPrediction::xSubtractOBMC( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp )
{
  Int iDstStride = pcYuvPredDst->getWidth();
  Int iSrcStride = pcYuvPredSrc->getWidth();
  Pel *pDst      = pcYuvPredDst->getLumaAddr( uiAbsPartIdx );
  Pel *pSrc      = pcYuvPredSrc->getLumaAddr( uiAbsPartIdx );

  if( iDir == 0 ) //above
  {
    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 2 ) >> 2;
    }

    pDst += iDstStride;
    pSrc += iSrcStride;

    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 4 ) >> 3;
    }

    if( !bOBMCSimp )
    {
      pDst += iDstStride;
      pSrc += iSrcStride;

      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pDst[i] + 8 ) >> 4;
      }

      pDst += iDstStride;
      pSrc += iSrcStride;

      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pSrc[i] + 16 ) >> 5;
      }
    }
  }

  if( iDir == 1 ) //left
  {
    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 2 ) >> 2;
    }

    pDst++;
    pSrc++;

    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 4 ) >> 3;
    }

    if( !bOBMCSimp )
    {
      pDst++;
      pSrc++;

      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 8 ) >> 4;
      }
      pDst++;
      pSrc++;

      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 16 ) >> 5;
      }
    }
  }

  if( iDir == 2 ) //below
  {
    pDst += ( iHeight - 4 )*iDstStride;
    pSrc += ( iHeight - 4 )*iSrcStride;
    if( !bOBMCSimp )
    {
      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pSrc[i] + 16 ) >> 5;
      }

      pDst += iDstStride;
      pSrc += iSrcStride;

      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pSrc[i] + 8 ) >> 4;
      }
    }
    else
    {
      pDst += iDstStride;
      pSrc += iSrcStride;
    }

    pDst += iDstStride;
    pSrc += iSrcStride;

    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 4 ) >> 3;
    }

    pDst += iDstStride;
    pSrc += iSrcStride;

    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 2 ) >> 2;
    }
  }

  if( iDir == 3 ) //right
  {
    pDst += iWidth - 4;
    pSrc += iWidth - 4;
    if( !bOBMCSimp )
    {
      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 16 ) >> 5;
      }

      pDst++;
      pSrc++;

      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 8 ) >> 4;
      }
    }
    else
    {
      pDst++;
      pSrc++;
    }

    pDst++;
    pSrc++;

    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 4 ) >> 3;
    }
    pDst++;
    pSrc++;

    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 2 ) >> 2;
    }
  }
}

Void TComPrediction::xSubBlockMotionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, Int uiPartAddr, Int iWidth, Int iHeight )
{
  if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
  {
    xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred
#if BIO 
    ,  false 
#endif
#if QC_FRUC_MERGE
      , false 
    , true
#endif
      );
  }
  else
  {
    xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred 
#if QC_FRUC_MERGE
      , true
#endif
      );
  }
}
#endif

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv*& rpcYuvPred
#if BIO                  
   ,Bool bBIOapplied
#endif
   , Bool bi 
#if QC_FRUC_MERGE
  , Bool bOBMC
#endif
  )
{ 
#if QC_FRUC_MERGE
  Int nBlkWidth = iWidth;
  Int nBlkHeight = iHeight;
  Int nBlkStepX = iWidth;
  Int nBlkStepY = iHeight;
  Int xRasterOffsetStep = 0;
  Int yRasterOffsetStep = 0;
  UInt uiIdxRasterStart = g_auiZscanToRaster[pcCU->getZorderIdxInCU() + uiPartAddr];
  Int nBlkMCWidth = iWidth;
  if( !bOBMC && pcCU->getFRUCMgrMode( uiPartAddr ) 
#if BIO                  
    && !bBIOapplied
#endif
   )
  {
    Int nRefineBlkSize = xFrucGetSubBlkSize( pcCU , uiPartAddr , nBlkWidth , nBlkHeight );
    nBlkMCWidth = iWidth = iHeight = nRefineBlkSize;
    nBlkStepX = nBlkStepY = nRefineBlkSize;
    xRasterOffsetStep = nRefineBlkSize >> 2;
    yRasterOffsetStep = xRasterOffsetStep * pcCU->getPic()->getNumPartInWidth();
  }
  for( Int y = 0 , yRasterOffset = 0 ; y < nBlkHeight ; y += nBlkStepY , yRasterOffset += yRasterOffsetStep )
  {
    for( Int x = 0 , xRasterOffset = 0 ; x < nBlkWidth ; x += nBlkStepX , xRasterOffset += xRasterOffsetStep )
    {
      uiPartAddr = g_auiRasterToZscan[uiIdxRasterStart+yRasterOffset+xRasterOffset] - pcCU->getZorderIdxInCU();
#endif
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );
  pcCU->clipMv(cMv);
#if QC_FRUC_MERGE
    // check whether later blocks have the same MV, refidx must been the same
    iWidth = nBlkMCWidth;
    for( Int xLater = x + nBlkStepX , xRasterOffsetLater = xRasterOffset + xRasterOffsetStep ; xLater < nBlkWidth ; xLater += nBlkStepX , xRasterOffsetLater += xRasterOffsetStep )
    {
      UInt uiPartAddrLater = g_auiRasterToZscan[uiIdxRasterStart+yRasterOffset+xRasterOffsetLater] - pcCU->getZorderIdxInCU();
      if( pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddrLater ) == cMv )
      {
        iWidth += nBlkStepX;
        x += nBlkStepX;
        xRasterOffset += xRasterOffsetStep;
      }
      else
        break;
    }
#endif
#if QC_IC
  Bool bICFlag = pcCU->getICFlag( uiPartAddr ) ;

#if QC_FRUC_MERGE
  xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi
#if BIO                  
,bBIOapplied
#endif
, false, bICFlag );
#else
  xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi
#if BIO                  
,bBIOapplied
#endif
, bICFlag );
#endif
  xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi, bICFlag );
#else
  xPredInterLumaBlk  ( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi 
#if BIO                  
,bBIOapplied
#endif
);
  xPredInterChromaBlk( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi );
#endif
#if QC_FRUC_MERGE
    }
  }
#endif
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv*& rpcYuvPred 
#if QC_FRUC_MERGE
  , Bool bOBMC
#endif
  )
{
  TComYuv* pcMbYuv;
  Int      iRefIdx[2] = {-1, -1};
#if BIO 
  Int      FrameNumber[3] = {-1, -1,-1};
  FrameNumber[2] = pcCU->getSlice()->getPOC();
  bool bBIOcheck0 = pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE; 
  bool bBIOcheck1 =  pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE;
  bool bBIOapplied = false;

  for ( Int iRefList = 0; iRefList < 2; iRefList++ )
  {
    RefPicList eRefPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[iRefList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );
   if ( iRefIdx[iRefList] >= 0 )
     FrameNumber[iRefList] = pcCU->getSlice()->getRefPic(eRefPicList,iRefIdx[iRefList])->getPOC() ;
  }
 
if ( iRefIdx[0] >= 0 && iRefIdx[1] >= 0 ) // applied for only EL,  only if Bi-pred is from different "time directions"
  {  
      int d1 = FrameNumber[1] - FrameNumber[2], d0 = FrameNumber[2] - FrameNumber[0];
      if (d1 * d0 > 0&&!bBIOcheck0&&! bBIOcheck1 )
    {
        bBIOapplied = true;
    }
  }

#if QC_FRUC_MERGE 
 if (pcCU->getFRUCMgrMode( uiPartAddr ) ) bBIOapplied = false;
#endif
#endif
  for ( Int iRefList = 0; iRefList < 2; iRefList++ )
  {
    RefPicList eRefPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[iRefList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );

    if ( iRefIdx[iRefList] < 0 )
    {
      continue;
    }

    assert( iRefIdx[iRefList] < pcCU->getSlice()->getNumRefIdx(eRefPicList) );
#if BIO 
  iRefListIdx = iRefList;
#endif
    pcMbYuv = &m_acYuvPred[iRefList];
    if( pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 )
    {
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv 
#if BIO                  
   ,bBIOapplied
#endif
   , true 
#if QC_FRUC_MERGE
        , bOBMC
#endif
        );
    }
    else
    {
#if QC_IC
      if ( ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) || 
             ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE ) ) 
            && !pcCU->getICFlag( uiPartAddr ) )
#else
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) || 
           ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE ) )
#endif
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv
#if BIO                  
   ,bBIOapplied
#endif
   , true 
#if QC_FRUC_MERGE
          , bOBMC
#endif
          );
      }
      else
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv 
#if QC_FRUC_MERGE
#if BIO                  
   ,bBIOapplied
#endif 
   , false 
   , bOBMC
#else
#if BIO                  
    ,bBIOapplied,false
#endif 
#endif
          );
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE 
#if QC_IC
    && !pcCU->getICFlag( uiPartAddr )
#endif
    )
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[0], &m_acYuvPred[1], iRefIdx[0], iRefIdx[1], uiPartAddr, iWidth, iHeight, rpcYuvPred );
  }  
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE 
#if QC_IC
    && !pcCU->getICFlag( uiPartAddr )
#endif
    )
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, rpcYuvPred ); 
  }
  else
  {
    xWeightedAverage( &m_acYuvPred[0], &m_acYuvPred[1], iRefIdx[0], iRefIdx[1], uiPartAddr, iWidth, iHeight, rpcYuvPred 
#if BIO                  
   ,bBIOapplied
#endif
                  );
  }
}
#if BIO 
Void  TComPrediction::xGradFilterX(Pel*  piRefY, Int iRefStride,Pel*  piDstY,Int iDstStride,
                                  Int iWidth, Int iHeight,Int iMVyFrac,Int iMVxFrac)
{
if ( iMVyFrac == 0 )
  {
    xCTI_Filter1DHorG (piRefY, iRefStride,  iWidth, iHeight, iDstStride,  piDstY, iMVxFrac );
    return;
  }

    Int tmpStride = m_filteredBlockTmp[0].getStride();
    Pel *tmp    = m_filteredBlockTmp[0].getLumaAddr();

  xCTI_Filter2DVerGG (piRefY - BIO_FILTER_HALF_LENGTH_MINUS_1,  iRefStride,  iWidth +BIO_FILTER_LENGTH_MINUS_1 , iHeight, tmpStride,  tmp,  iMVyFrac        );
  xCTI_Filter2DHorG  (tmp +   BIO_FILTER_HALF_LENGTH_MINUS_1,  tmpStride,  iWidth            , iHeight, iDstStride,  piDstY ,iMVxFrac );
  }
Void  TComPrediction::xGradFilterY(Pel*  piRefY, Int iRefStride,Pel*  piDstY,Int iDstStride,
                                  Int iWidth, Int iHeight,Int iMVyFrac,Int iMVxFrac)
{
  if ( iMVxFrac == 0 )
  {
    xCTI_Filter1DVerG (piRefY, iRefStride,  iWidth, iHeight, iDstStride,  piDstY, iMVyFrac );
    return;
  }

    Int tmpStride = m_filteredBlockTmp[0].getStride();
    Pel *tmp    = m_filteredBlockTmp[0].getLumaAddr();
  
  xCTI_Filter2DVerG (piRefY  - BIO_FILTER_HALF_LENGTH_MINUS_1,  iRefStride,  iWidth +BIO_FILTER_LENGTH_MINUS_1 , iHeight, tmpStride,  tmp,  iMVyFrac );
  xCTI_Filter2DHorGG (tmp  + BIO_FILTER_HALF_LENGTH_MINUS_1,  tmpStride,  iWidth            , iHeight, iDstStride,  piDstY , iMVxFrac );
}
#endif
/**
 * \brief Generate motion-compensated luma block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */
Void TComPrediction::xPredInterLumaBlk( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi 
#if BIO                  
   ,bool bBIOapplied
#endif
#if QC_FRUC_MERGE
  , Int nFRUCMode
#endif
#if QC_IC
  , Bool bICFlag
#endif
  )
{
#if QC_FRUC_MERGE
  cu->clipMv( *mv );
  Int nFilterIdx = nFRUCMode ? cu->getSlice()->getSPS()->getFRUCRefineFilter() : 0;
#endif
  Int refStride = refPic->getStride();  
#if QC_MV_STORE_PRECISION_BIT 
  Int refOffset = ( mv->getHor() >> QC_MV_STORE_PRECISION_BIT ) + ( mv->getVer() >> QC_MV_STORE_PRECISION_BIT ) * refStride;
#else
  Int refOffset = ( mv->getHor() >> 2 ) + ( mv->getVer() >> 2 ) * refStride;
#endif
  Pel *ref      = refPic->getLumaAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  
  Int dstStride = dstPic->getStride();
  Pel *dst      = dstPic->getLumaAddr( partAddr ); 
#if QC_FRUC_MERGE
  if( nFRUCMode )
    dst = dstPic->getLumaAddr( 0 );
#endif
  
#if QC_MV_STORE_PRECISION_BIT == 3
  Int xFrac = mv->getHor() & 0x7;
  Int yFrac = mv->getVer() & 0x7;
#else
  Int xFrac = mv->getHor() & 0x3;
  Int yFrac = mv->getVer() & 0x3;
#endif
#if BIO 
  if ( bBIOapplied)
  { 
      Pel* pGradY= m_pGradY0;  Pel* pGradX= m_pGradX0;  Pel *pPred= m_pPred0;
      Int iWidthG   = width + 4;
      Int iHeightG = height + 4;
      if (iRefListIdx == 0)
      {    
       pGradY = m_pGradY0;
       pGradX = m_pGradX0;
       pPred = m_pPred0;
      }
      else
      {
       pGradY = m_pGradY1;
       pGradX = m_pGradX1;
       pPred  = m_pPred1 ;
      }

  ref -=(2+2*refStride);
#if QC_MV_STORE_PRECISION_BIT == 3
         xGradFilterY(ref , refStride,pGradY,iWidthG,iWidthG,iHeightG, yFrac>>1,  xFrac>>1);
     xGradFilterX(ref , refStride,pGradX,iWidthG,iWidthG,iHeightG, yFrac>>1,  xFrac>>1);
#else
         xGradFilterY(ref , refStride,pGradY,iWidthG,iWidthG,iHeightG, yFrac,  xFrac);
     xGradFilterX(ref , refStride,pGradX,iWidthG,iWidthG,iHeightG, yFrac,  xFrac);
#endif
     xPredInterFrac( ref , pPred, iWidthG, refStride, xFrac, yFrac, iWidthG, iHeightG,bi);
    ref +=(2+2*refStride);

    }
  else
  {   
#endif
  if ( yFrac == 0 )
  {
    m_if.filterHorLuma( ref, refStride, dst, dstStride, width, height, xFrac,       
#if QC_IC
      !bi || bICFlag
#else
      !bi 
#endif
#if QC_FRUC_MERGE
      , nFilterIdx
#endif
      );
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVerLuma( ref, refStride, dst, dstStride, width, height, yFrac, true, 
#if QC_IC
      !bi || bICFlag
#else
      !bi 
#endif
#if QC_FRUC_MERGE
      , nFilterIdx
#endif
      );
  }
  else
  {
    Int tmpStride = m_filteredBlockTmp[0].getStride();
    Short *tmp    = m_filteredBlockTmp[0].getLumaAddr();

#if QC_FRUC_MERGE
    Int filterSize = NTAPS_LUMA;
    if( nFilterIdx == 1 )
      filterSize = NTAPS_LUMA_FRUC;
    else if( nFilterIdx != 0 )
      assert( 0 );
#else
    Int filterSize = NTAPS_LUMA;
#endif
    Int halfFilterSize = ( filterSize >> 1 );

    m_if.filterHorLuma(ref - (halfFilterSize-1)*refStride, refStride, tmp, tmpStride, width, height+filterSize-1, xFrac, false     
#if QC_FRUC_MERGE
      , nFilterIdx
#endif
      );
    m_if.filterVerLuma(tmp + (halfFilterSize-1)*tmpStride, tmpStride, dst, dstStride, width, height,              yFrac, false, 
#if QC_IC
      !bi || bICFlag
#else
      !bi
#endif
#if QC_FRUC_MERGE
      , nFilterIdx
#endif
      );    
  }
#if BIO 
}
#endif
#if QC_IC
  if( bICFlag )
  {
    Int a, b, i, j;
    const Int iShift = IC_CONST_SHIFT;
    xGetLLSICPrediction( cu, mv, refPic, a, b, TEXT_LUMA );
    dst = dstPic->getLumaAddr( partAddr );

    for ( i = 0; i < height; i++ )
    {
      for ( j = 0; j < width; j++ )
      {
        dst[j] = Clip3( 0, ( 1 << g_bitDepthY ) - 1, ( ( a*dst[j] ) >> iShift ) + b );
      }
      dst += dstStride;
    }

    if(bi)
    {
      Pel *dst2      = dstPic->getLumaAddr( partAddr );
      Int shift = IF_INTERNAL_PREC - g_bitDepthY;
      for (i = 0; i < height; i++)
      {
        for (j = 0; j < width; j++)
        {
          Short val = dst2[j] << shift;
          dst2[j] = val - (Short)IF_INTERNAL_OFFS;
        }
        dst2 += dstStride;
      }
    }
  }
#endif
}
#if BIO 
Void TComPrediction::xPredInterFrac(Pel* ref,Pel* dst,Int dstStride,Int refStride,Int xFrac,Int yFrac,Int width, Int height,Bool bi  )
{
  if ( yFrac == 0 )
  {
    m_if.filterHorLuma( ref, refStride, dst, dstStride, width, height, xFrac,       !bi  );
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVerLuma( ref, refStride, dst, dstStride, width, height, yFrac, true,  !bi  );
  }
  else
  {
    Int tmpStride = m_filteredBlockTmp[0].getStride();
    Short *tmp    = m_filteredBlockTmp[0].getLumaAddr();
    Int filterSize = NTAPS_LUMA;
    Int halfFilterSize = ( filterSize >> 1 );

    m_if.filterHorLuma(ref - (halfFilterSize-1)*refStride, refStride, tmp, tmpStride, width, height+filterSize-1, xFrac, false      );
    m_if.filterVerLuma(tmp + (halfFilterSize-1)*tmpStride, tmpStride, dst, dstStride, width, height,              yFrac, false,  !bi);    
  }
}
#endif
/**
 * \brief Generate motion-compensated chroma block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */
Void TComPrediction::xPredInterChromaBlk( TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi
#if QC_IC
  , Bool bICFlag 
#endif
  )
{
#if QC_FRUC_MERGE
  cu->clipMv( *mv );
#endif
  Int     refStride  = refPic->getCStride();
  Int     dstStride  = dstPic->getCStride();
  
#if QC_MV_STORE_PRECISION_BIT
  Int     refOffset  = (mv->getHor() >> (QC_MV_STORE_PRECISION_BIT+1)) + (mv->getVer() >> (QC_MV_STORE_PRECISION_BIT+1)) * refStride;
#else
  Int     refOffset  = (mv->getHor() >> 3) + (mv->getVer() >> 3) * refStride;
#endif
  
  Pel*    refCb     = refPic->getCbAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  Pel*    refCr     = refPic->getCrAddr( cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  
  Pel* dstCb = dstPic->getCbAddr( partAddr );
  Pel* dstCr = dstPic->getCrAddr( partAddr );
  
#if QC_MV_STORE_PRECISION_BIT == 3
  Int     xFrac  = mv->getHor() & 0xF;
  Int     yFrac  = mv->getVer() & 0xF;
#else
  Int     xFrac  = mv->getHor() & 0x7;
  Int     yFrac  = mv->getVer() & 0x7;
#endif
  UInt    cxWidth  = width  >> 1;
  UInt    cxHeight = height >> 1;
  
  Int     extStride = m_filteredBlockTmp[0].getStride();
  Short*  extY      = m_filteredBlockTmp[0].getLumaAddr();


  if ( yFrac == 0 )
  {
#if QC_IC
    m_if.filterHorChroma(refCb, refStride, dstCb,  dstStride, cxWidth, cxHeight, xFrac, !bi || bICFlag);
    m_if.filterHorChroma(refCr, refStride, dstCr,  dstStride, cxWidth, cxHeight, xFrac, !bi || bICFlag);   
#else
    m_if.filterHorChroma(refCb, refStride, dstCb,  dstStride, cxWidth, cxHeight, xFrac, !bi);    
    m_if.filterHorChroma(refCr, refStride, dstCr,  dstStride, cxWidth, cxHeight, xFrac, !bi);    
#endif
  }
  else if ( xFrac == 0 )
  {
#if QC_IC
    m_if.filterVerChroma(refCb, refStride, dstCb, dstStride, cxWidth, cxHeight, yFrac, true, !bi || bICFlag);
    m_if.filterVerChroma(refCr, refStride, dstCr, dstStride, cxWidth, cxHeight, yFrac, true, !bi || bICFlag);    
#else
    m_if.filterVerChroma(refCb, refStride, dstCb, dstStride, cxWidth, cxHeight, yFrac, true, !bi);    
    m_if.filterVerChroma(refCr, refStride, dstCr, dstStride, cxWidth, cxHeight, yFrac, true, !bi);    
#endif
  }
  else
  {
    Int filterSize = NTAPS_CHROMA;
    Int halfFilterSize = (filterSize>>1);

    m_if.filterHorChroma(refCb - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth, cxHeight+filterSize-1, xFrac, false);
#if QC_IC
    m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCb, dstStride, cxWidth, cxHeight  , yFrac, false, !bi || bICFlag);
#else
    m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCb, dstStride, cxWidth, cxHeight  , yFrac, false, !bi);
#endif

    m_if.filterHorChroma(refCr - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth, cxHeight+filterSize-1, xFrac, false);
#if QC_IC
    m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCr, dstStride, cxWidth, cxHeight  , yFrac, false, !bi || bICFlag);
#else
    m_if.filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCr, dstStride, cxWidth, cxHeight  , yFrac, false, !bi);    
#endif
  }

#if QC_IC
  if( bICFlag )
  {
    Int a, b, i, j;
    const Int iShift = IC_CONST_SHIFT;
    xGetLLSICPrediction( cu, mv, refPic, a, b, TEXT_CHROMA_U ); // Cb

    dstCb = dstPic->getCbAddr( partAddr );
    dstCr = dstPic->getCrAddr( partAddr );

    for ( i = 0; i < cxHeight; i++ )
    {
      for ( j = 0; j < cxWidth; j++ )
      {
        dstCb[j] = Clip3(  0, ( 1 << g_bitDepthC ) - 1, ( ( a*dstCb[j] ) >> iShift ) + b );
      }
      dstCb += dstStride;
    }

    xGetLLSICPrediction( cu, mv, refPic, a, b, TEXT_CHROMA_V ); // Cr

    for ( i = 0; i < cxHeight; i++ )
    {
      for ( j = 0; j < cxWidth; j++ )
      {
        dstCr[j] = Clip3( 0, ( 1 << g_bitDepthC ) - 1, ( ( a*dstCr[j] ) >> iShift ) + b );
      }
      dstCr += dstStride;
    }

    if(bi)
    {
      Pel* dstCb2 = dstPic->getCbAddr( partAddr );
      Pel* dstCr2 = dstPic->getCrAddr( partAddr );
      Int shift = IF_INTERNAL_PREC - g_bitDepthC;
      for (i = 0; i < cxHeight; i++)
      {
        for (j = 0; j < cxWidth; j++)
        {
          Short val = dstCb2[j] << shift;
          dstCb2[j] = val - (Short)IF_INTERNAL_OFFS;

          val = dstCr2[j] << shift;
          dstCr2[j] = val - (Short)IF_INTERNAL_OFFS;
        }
        dstCb2 += dstStride;
        dstCr2 += dstStride;
      }
    }
  }
#endif
}

#if QC_IC
/** Function for deriving the position of first non-zero binary bit of a value
 * \param x input value
 *
 * This function derives the position of first non-zero binary bit of a value
 */
Int GetMSB( UInt x )
{
  Int iMSB = 0, bits = ( sizeof( Int ) << 3 ), y = 1;

  while( x > 1 )
  {
    bits >>= 1;
    y = x >> bits;

    if( y )
    {
      x = y;
      iMSB += bits;
    }
  }

  iMSB+=y;

  return iMSB;
}

/** Function for deriving LM illumination compensation.
 */
Void TComPrediction::xGetLLSICPrediction( TComDataCU* pcCU, TComMv *pMv, TComPicYuv *pRefPic, Int &a, Int &b, TextType eType )
{
  TComPicYuv *pRecPic = pcCU->getPic()->getPicYuvRec();
  Pel *pRec = NULL, *pRef = NULL;
  UInt uiWidth, /*uiHeight,*/ uiTmpPartIdx;
  Int iRecStride = ( eType == TEXT_LUMA ) ? pRecPic->getStride() : pRecPic->getCStride();
  Int iRefStride = ( eType == TEXT_LUMA ) ? pRefPic->getStride() : pRefPic->getCStride();
  Int iRefOffset, iRecOffset, iHor, iVer;
#if QC_MV_STORE_PRECISION_BIT
  UInt uiMvPres = ( eType == TEXT_LUMA ) ? QC_MV_STORE_PRECISION_BIT : QC_MV_STORE_PRECISION_BIT+1, uiMvOffset = 1 << ( uiMvPres - 1 );
#else
  UInt uiMvPres = ( eType == TEXT_LUMA ) ? 2 : 3, uiMvOffset = 1 << ( uiMvPres - 1 );
#endif

  iHor = ( pMv->getHor() + uiMvOffset ) >> uiMvPres;
  iVer = ( pMv->getVer() + uiMvOffset ) >> uiMvPres;

  uiWidth  = ( eType == TEXT_LUMA ) ? pcCU->getWidth( 0 )  : ( pcCU->getWidth( 0 )  >> 1 );
  //uiHeight = ( eType == TEXT_LUMA ) ? pcCU->getHeight( 0 ) : ( pcCU->getHeight( 0 ) >> 1 );

  Int j, iCountShift = 0;

  // LLS parameters estimation -->

  Int x = 0, y = 0, xx = 0, xy = 0;
  Int precShift = std::max(0, ( ( eType == TEXT_LUMA ) ? g_bitDepthY : g_bitDepthC ) - 12 );
  Int iTmpRec, iTmpRef;
  Int iRefStep, iRecStep;
  UInt uiStep = 2;//uiWidth > 8 ? 2 : 1;
  TComDataCU* pNeigCu = NULL;
  TComMv cMv;
  Int iMaxNumMinus1 = 30 - 2*min( ( ( eType == TEXT_LUMA ) ? g_bitDepthY : g_bitDepthC ), 12 ) - 1;
  while( uiWidth/uiStep > ( 1 << iMaxNumMinus1 ) ) //make sure log2(2*uiWidth/uiStep) + 2*min(g_bitDepthY, 12) <= 30
  {
    uiStep <<= 1;
  }

  for( Int iDir = 0; iDir < 2; iDir++ ) //iDir: 0 - above, 1 - left
  {
    if( !iDir )
    {
      pNeigCu = pcCU->getPUAbove( uiTmpPartIdx, pcCU->getZorderIdxInCU() );
    }
    else
    {
      pNeigCu =  pcCU->getPULeft( uiTmpPartIdx, pcCU->getZorderIdxInCU() );
    }

    if( pNeigCu == NULL )
    {
      continue;
    }

    cMv.setHor( iHor << uiMvPres ); cMv.setVer( iVer << uiMvPres );
    pNeigCu->clipMv( cMv );

    if( iDir )
    {
      iRefOffset = ( cMv.getHor() >> uiMvPres ) + ( cMv.getVer() >> uiMvPres ) * iRefStride - 1;
      iRecOffset = -1;
      iRefStep   = iRefStride*uiStep;
      iRecStep   = iRecStride*uiStep;
    }
    else
    {
      iRefOffset = ( cMv.getHor() >> uiMvPres ) + ( cMv.getVer() >> uiMvPres ) * iRefStride - iRefStride;
      iRecOffset = -iRecStride;
      iRefStep   = uiStep;
      iRecStep   = uiStep;
    }

    if( eType == TEXT_LUMA )
    {
      pRef = pRefPic->getLumaAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() ) + iRefOffset;
      pRec = pRecPic->getLumaAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() ) + iRecOffset;
    }
    else if( eType == TEXT_CHROMA_U )
    {
      pRef = pRefPic->getCbAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() ) + iRefOffset;
      pRec = pRecPic->getCbAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() ) + iRecOffset;
    }
    else
    {
      assert( eType == TEXT_CHROMA_V );
      pRef = pRefPic->getCrAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() ) + iRefOffset;
      pRec = pRecPic->getCrAddr( pcCU->getAddr(), pcCU->getZorderIdxInCU() ) + iRecOffset;
    }

    for( j = 0; j < uiWidth; j+=uiStep )
    {
      iTmpRef = pRef[0] >> precShift;
      iTmpRec = pRec[0] >> precShift;

      x  += iTmpRef;
      y  += iTmpRec;
      xx += iTmpRef*iTmpRef;
      xy += iTmpRef*iTmpRec;

      pRef += iRefStep;
      pRec += iRecStep;
    }

    iCountShift += ( iCountShift ? 1 : g_aucConvertToBit[ uiWidth/uiStep ] + 2 );
  }

  if( iCountShift == 0 )
  {
    a = ( 1 << IC_CONST_SHIFT );
    b = 0;
    return;
  }

  xy += xx >> IC_REG_COST_SHIFT;
  xx += xx >> IC_REG_COST_SHIFT;

  Int  iCropShift = max( 0, ( ( ( eType == TEXT_LUMA ) ? g_bitDepthY : g_bitDepthC ) - precShift + iCountShift ) - 15 );
  Int  x1 = x, y1 = y;

  x  >>= iCropShift;
  y  >>= iCropShift;
  xy >>= ( iCropShift << 1 );
  xx >>= ( iCropShift << 1 );

  Int a1 = ( xy << iCountShift ) - ( y * x );
  Int a2 = ( xx << iCountShift ) - ( x * x );

  x = x1 << precShift;
  y = y1 << precShift;

  const Int iShift = IC_CONST_SHIFT;
  const Int iShiftA2 = 6;
  const Int iAccuracyShift = 15;
  Int iScaleShiftA2 = 0;
  Int iScaleShiftA1 = 0;
  Int a1s = a1;
  Int a2s = a2;

  iScaleShiftA2 = GetMSB( abs( a2 ) ) - iShiftA2;
  iScaleShiftA1 = iScaleShiftA2 - IC_SHIFT_DIFF;

  if( iScaleShiftA1 < 0 )
  {
    iScaleShiftA1 = 0;
  }

  if( iScaleShiftA2 < 0 )
  {
    iScaleShiftA2 = 0;
  }

  Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

  a2s = a2 >> iScaleShiftA2;
  a1s = a1 >> iScaleShiftA1;

  a2s = Clip3( 0 , 63, a2s );
  Int64 aI64 = ( (Int64) a1s * m_uiaICShift[ a2s ] ) >> iScaleShiftA;
  a = (Int) aI64;
  a = Clip3( 0, 1 << ( iShift + 2 ), a );
  b = (  y - ( ( a * x ) >> iShift ) + ( 1 << ( iCountShift - 1 ) ) ) >> iCountShift;
  Int iOffset = 1 << ( eType == TEXT_LUMA ? g_bitDepthY - 1 : g_bitDepthC - 1 );
  b = Clip3( -iOffset, iOffset - 1, b );
}
#endif
#if BIO
Pel optical_flow_averaging( Int64 s1,Int64 s2,Int64 s3,Int64 s5,Int64 s6,
              Pel pGradX0 , Pel pGradX1,Pel pGradY0 , Pel pGradY1,
              Pel pSrcY0Temp, Pel pSrcY1Temp, int shiftNum , int  offset     , Int64 limit ,   Int64 denom_min_1 ,   Int64 denom_min_2   )
{
Int64 vx = 0;  Int64 vy = 0;
Int64 b=0;
if (s1>denom_min_1)         
{
vx =  s3  / s1;
vx =vx>limit?limit:vx<-limit?-limit:vx; 
}
if (s5>denom_min_2)
{
vy = (s6-vx*s2)/ s5;
vy =vy>limit?limit:vy<-limit?-limit:vy; 
}
  
b = vx * (pGradX0 - pGradX1) +vy * (pGradY0 - pGradY1);       
b = (b>0)?((b +32)>> 6):(-((-b+32) >> 6));
return( ClipY((Short)((pSrcY0Temp + pSrcY1Temp + b +offset) >> shiftNum)));
}
#endif
Void TComPrediction::xWeightedAverage( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv*& rpcYuvDst 
#if BIO                  
   ,bool bBIOapplied
#endif
)
{

  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
#if BIO 
    if (bBIOapplied)
    {
      static Int64 m_piDotProduct1[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct2[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct3[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct5[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct6[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS1temp[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS2temp[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS3temp[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS5temp[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS6temp[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS1[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS2[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS3[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS5[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS6[BIO_TEMP_BUFFER_SIZE];
      Int x=0, y=0;

      Int iHeightG = iHeight + 4;
      Int iWidthG  = iWidth  + 4;
      Int iStrideTemp = 2+2*iWidthG;
      Pel* pGradX0 = m_pGradX0; Pel* pGradX1 = m_pGradX1; 
      Pel* pGradY0 = m_pGradY0; Pel* pGradY1 = m_pGradY1;    
      Pel* pSrcY0 = m_pPred0; 
      Pel* pSrcY1 = m_pPred1;
      Int iSrc0Stride = iWidthG;
      Int iSrc1Stride = iWidthG;   
      Pel* pDstY = rpcYuvDst->getLumaAddr(uiPartIdx); 
      Int iDstStride = rpcYuvDst->getStride(); 
      Pel* pSrcY0Temp = pSrcY0; Pel* pSrcY1Temp = pSrcY1;

      static const int  shiftNum    = IF_INTERNAL_PREC + 1 - g_bitDepthY;
      static const int  offset      = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;
      static const Int64 limit = (12<<(IF_INTERNAL_PREC-1- g_bitDepthY)); 
      static const Int64 regularizator_1 = 500* (1<< (g_bitDepthY-8))* (1<< (g_bitDepthY-8));
      static const Int64 regularizator_2 =regularizator_1<<1;
      static const Int64 denom_min_1 = 700* (1<< (g_bitDepthY-8))* (1<< (g_bitDepthY-8));
      static const Int64 denom_min_2 = denom_min_1<<1;

      Int64* m_piDotProductTemp1 = m_piDotProduct1;Int64* m_piDotProductTemp2 = m_piDotProduct2;Int64* m_piDotProductTemp3 = m_piDotProduct3;Int64* m_piDotProductTemp5 = m_piDotProduct5;Int64* m_piDotProductTemp6 = m_piDotProduct6;
      Int64* m_pS1loc=m_piS1temp;Int64* m_pS2loc=m_piS2temp;Int64* m_pS3loc=m_piS3temp;Int64* m_pS5loc=m_piS5temp;Int64* m_pS6loc=m_piS6temp;
      Int64* m_pS1loc_1=m_piS1temp;Int64* m_pS2loc_1=m_piS2temp;Int64* m_pS3loc_1=m_piS3temp;Int64* m_pS5loc_1=m_piS5temp;Int64* m_pS6loc_1=m_piS6temp;
      Int64* m_pS1loc_2=m_piS1temp;Int64* m_pS2loc_2=m_piS2temp;Int64* m_pS3loc_2=m_piS3temp;Int64* m_pS5loc_2=m_piS5temp;Int64* m_pS6loc_2=m_piS6temp;
      Int64* m_pS1loc_3=m_piS1temp;Int64* m_pS2loc_3=m_piS2temp;Int64* m_pS3loc_3=m_piS3temp;Int64* m_pS5loc_3=m_piS5temp;Int64* m_pS6loc_3=m_piS6temp;

      Int64* m_pS1loc1=m_piS1temp;Int64* m_pS2loc1=m_piS2temp;Int64* m_pS3loc1=m_piS3temp;Int64* m_pS5loc1=m_piS5temp;Int64* m_pS6loc1=m_piS6temp;
      Int64* m_pS1loc2=m_piS1temp;Int64* m_pS2loc2=m_piS2temp;Int64* m_pS3loc2=m_piS3temp;Int64* m_pS5loc2=m_piS5temp;Int64* m_pS6loc2=m_piS6temp;
      Int64* m_piSS1loc=m_piS1;Int64* m_piSS2loc=m_piS2;Int64* m_piSS3loc=m_piS3;Int64* m_piSS5loc=m_piS5;Int64* m_piSS6loc=m_piS6;
      Int64* m_piSS1loc_1=m_piS1;Int64* m_piSS2loc_1=m_piS2;Int64* m_piSS3loc_1=m_piS3;Int64* m_piSS5loc_1=m_piS5;Int64* m_piSS6loc_1=m_piS6;

      Int64 temp=0, tempX=0, tempY=0;
      for (y = 0; y < iHeightG; y ++)    
      {
        for (x =0; x < iWidthG ; x ++)
        {
          temp =(Int64) (pSrcY0Temp[x ] - pSrcY1Temp[x ]);
          tempX =(Int64) (pGradX0[x] +  pGradX1[x]);
          tempY =(Int64) (pGradY0[x] +  pGradY1[x]);
          m_piDotProductTemp1[x] =  tempX*tempX;
          m_piDotProductTemp2[x] =  tempX*tempY;
          m_piDotProductTemp3[x] = -tempX*temp<<5;
          m_piDotProductTemp5[x] =  tempY*tempY<<1;
          m_piDotProductTemp6[x] = -tempY*temp<<6;
        }
        pSrcY0Temp+=iSrc0Stride;
        pSrcY1Temp+=iSrc1Stride;
        pGradX0+=iWidthG;
        pGradX1+=iWidthG;
        pGradY0+=iWidthG;
        pGradY1+=iWidthG;
        m_piDotProductTemp1+=iWidthG;
        m_piDotProductTemp2+=iWidthG;
        m_piDotProductTemp3+=iWidthG;
        m_piDotProductTemp5+=iWidthG;
        m_piDotProductTemp6+=iWidthG;
      }

      m_piDotProductTemp1 = m_piDotProduct1+2;m_piDotProductTemp2 = m_piDotProduct2+2;m_piDotProductTemp3 = m_piDotProduct3+2;m_piDotProductTemp5 = m_piDotProduct5+2;m_piDotProductTemp6 = m_piDotProduct6+2;
      m_pS1loc=m_piS1temp+2;m_pS2loc=m_piS2temp+2;m_pS3loc=m_piS3temp+2;m_pS5loc=m_piS5temp+2;m_pS6loc=m_piS6temp+2;      

      for (y = 0;y < iHeightG ; y ++)
      {  
        x=0;
        m_pS1loc[x] =  m_piDotProductTemp1[-2] + m_piDotProductTemp1[-1] + m_piDotProductTemp1[0] + m_piDotProductTemp1[1] +  m_piDotProductTemp1[2] ;
        m_pS2loc[x] =  m_piDotProductTemp2[-2] + m_piDotProductTemp2[-1] + m_piDotProductTemp2[0] + m_piDotProductTemp2[1] +  m_piDotProductTemp2[2] ;
        m_pS3loc[x] =  m_piDotProductTemp3[-2] + m_piDotProductTemp3[-1] + m_piDotProductTemp3[0] + m_piDotProductTemp3[1] +  m_piDotProductTemp3[2] ;
        m_pS5loc[x] =  m_piDotProductTemp5[-2] + m_piDotProductTemp5[-1] + m_piDotProductTemp5[0] + m_piDotProductTemp5[1] +  m_piDotProductTemp5[2] ;
        m_pS6loc[x] =  m_piDotProductTemp6[-2] + m_piDotProductTemp6[-1] + m_piDotProductTemp6[0] + m_piDotProductTemp6[1] +  m_piDotProductTemp6[2] ;

        for ( x=1;    x < iWidth  ; x++)
        {
          m_pS1loc[x] =  -m_piDotProductTemp1[x-3]  +  m_piDotProductTemp1[x+2] + m_pS1loc[x-1];
          m_pS2loc[x] =  -m_piDotProductTemp2[x-3]  +  m_piDotProductTemp2[x+2] + m_pS2loc[x-1];  
          m_pS3loc[x] =  -m_piDotProductTemp3[x-3]  +  m_piDotProductTemp3[x+2] + m_pS3loc[x-1];
          m_pS5loc[x] =  -m_piDotProductTemp5[x-3]  +  m_piDotProductTemp5[x+2] + m_pS5loc[x-1];  
          m_pS6loc[x] =  -m_piDotProductTemp6[x-3]  +  m_piDotProductTemp6[x+2] + m_pS6loc[x-1];
        }
        m_piDotProductTemp1+=iWidthG;m_piDotProductTemp2+=iWidthG;m_piDotProductTemp3+=iWidthG;m_piDotProductTemp5+=iWidthG;m_piDotProductTemp6+=iWidthG;
        m_pS1loc+=iWidthG;m_pS2loc+=iWidthG;m_pS3loc+=iWidthG;m_pS5loc+=iWidthG;m_pS6loc+=iWidthG;
      }
      m_pS1loc=m_piS1temp+iStrideTemp;m_pS2loc=m_piS2temp+iStrideTemp;m_pS3loc=m_piS3temp+iStrideTemp;m_pS5loc=m_piS5temp+iStrideTemp;m_pS6loc=m_piS6temp+iStrideTemp;
      m_pS1loc_1=m_pS1loc- iWidthG;m_pS2loc_1=m_pS2loc- iWidthG;m_pS3loc_1=m_pS3loc- iWidthG;m_pS5loc_1=m_pS5loc- iWidthG;m_pS6loc_1=m_pS6loc- iWidthG;
      m_pS1loc_2=m_pS1loc_1- iWidthG;m_pS2loc_2=m_pS2loc_1- iWidthG;m_pS3loc_2=m_pS3loc_1- iWidthG;m_pS5loc_2=m_pS5loc_1- iWidthG;m_pS6loc_2=m_pS6loc_1- iWidthG;
      m_pS1loc1=m_pS1loc+ iWidthG;m_pS2loc1=m_pS2loc+ iWidthG;m_pS3loc1=m_pS3loc+ iWidthG;m_pS5loc1=m_pS5loc+ iWidthG;m_pS6loc1=m_pS6loc+ iWidthG;
      m_pS1loc2=m_pS1loc1+ iWidthG;m_pS2loc2=m_pS2loc1+ iWidthG;m_pS3loc2=m_pS3loc1+ iWidthG;m_pS5loc2=m_pS5loc1+ iWidthG;m_pS6loc2=m_pS6loc1+ iWidthG;
      m_piSS1loc=m_piS1+iStrideTemp;m_piSS2loc=m_piS2+iStrideTemp;m_piSS3loc=m_piS3+iStrideTemp;m_piSS5loc=m_piS5+iStrideTemp;m_piSS6loc=m_piS6+iStrideTemp;
      pGradX0 = m_pGradX0+iStrideTemp ;  pGradX1 = m_pGradX1+iStrideTemp ;
      pGradY0 = m_pGradY0+iStrideTemp  ; pGradY1 = m_pGradY1+iStrideTemp ;
      pSrcY0Temp = pSrcY0 +iStrideTemp;
      pSrcY1Temp = pSrcY1 +iStrideTemp;

      y = 0;
      for (x = 0; x < iWidth ; x ++)
      { 
        m_piSS1loc[x]=m_pS1loc_2[x]+m_pS1loc_1[x]+m_pS1loc[x]+m_pS1loc1[x]+m_pS1loc2[x]+regularizator_1;
        m_piSS2loc[x]=m_pS2loc_2[x]+m_pS2loc_1[x]+m_pS2loc[x]+m_pS2loc1[x]+m_pS2loc2[x];
        m_piSS3loc[x]=m_pS3loc_2[x]+m_pS3loc_1[x]+m_pS3loc[x]+m_pS3loc1[x]+m_pS3loc2[x];
        m_piSS5loc[x]=m_pS5loc_2[x]+m_pS5loc_1[x]+m_pS5loc[x]+m_pS5loc1[x]+m_pS5loc2[x]+regularizator_2;
        m_piSS6loc[x]=m_pS6loc_2[x]+m_pS6loc_1[x]+m_pS6loc[x]+m_pS6loc1[x]+m_pS6loc2[x];
        pDstY[x]=optical_flow_averaging(
          m_piSS1loc[x],m_piSS2loc[x],m_piSS3loc[x],m_piSS5loc[x],m_piSS6loc[x],
          pGradX0[x] , pGradX1[x],pGradY0[x] , pGradY1[x],pSrcY0Temp[x ], pSrcY1Temp[x ]
        ,  shiftNum ,  offset , limit, denom_min_1, denom_min_2);
      }
      m_pS1loc2+=iWidthG;m_pS2loc2+=iWidthG;m_pS3loc2+=iWidthG;m_pS5loc2+=iWidthG;m_pS6loc2+=iWidthG;
      pDstY += iDstStride;pSrcY0Temp+=iSrc0Stride;pSrcY1Temp+=iSrc1Stride;pGradX0+=iWidthG;pGradX1+=iWidthG;pGradY0+=iWidthG;pGradY1+=iWidthG;
      m_piSS1loc_1=m_piSS1loc;m_piSS2loc_1=m_piSS2loc;m_piSS3loc_1=m_piSS3loc;m_piSS5loc_1=m_piSS5loc;m_piSS6loc_1=m_piSS6loc;
      m_pS1loc+=iWidthG;m_pS2loc+=iWidthG;m_pS3loc+=iWidthG;m_pS5loc+=iWidthG;m_pS6loc+=iWidthG;
      m_pS1loc_3=m_pS1loc_2;m_pS2loc_3=m_pS2loc_2;m_pS3loc_3=m_pS3loc_2;m_pS5loc_3=m_pS5loc_2;m_pS6loc_3=m_pS6loc_2;
      m_piSS1loc+=iWidthG;  m_piSS2loc+=iWidthG;  m_piSS3loc+=iWidthG;  m_piSS5loc+=iWidthG;  m_piSS6loc+=iWidthG;
      for (y = 1;  y < iHeight ;   y ++)
      {
        for (x = 0; x < iWidth ; x ++)
        { 
          m_piSS1loc[x]=m_piSS1loc_1[x]-m_pS1loc_3[x]+m_pS1loc2[x];
          m_piSS2loc[x]=m_piSS2loc_1[x]-m_pS2loc_3[x]+m_pS2loc2[x];
          m_piSS3loc[x]=m_piSS3loc_1[x]-m_pS3loc_3[x]+m_pS3loc2[x];
          m_piSS5loc[x]=m_piSS5loc_1[x]-m_pS5loc_3[x]+m_pS5loc2[x];
          m_piSS6loc[x]=m_piSS6loc_1[x]-m_pS6loc_3[x]+m_pS6loc2[x];

          pDstY[x]=optical_flow_averaging(m_piSS1loc[x],m_piSS2loc[x],m_piSS3loc[x],m_piSS5loc[x],m_piSS6loc[x],
            pGradX0[x] , pGradX1[x],pGradY0[x] , pGradY1[x],pSrcY0Temp[x ], pSrcY1Temp[x ]
          ,  shiftNum,  offset, limit, denom_min_1, denom_min_2 );
        }

        m_piSS1loc_1=m_piSS1loc;m_piSS2loc_1=m_piSS2loc;m_piSS3loc_1=m_piSS3loc;m_piSS5loc_1=m_piSS5loc;m_piSS6loc_1=m_piSS6loc;
        m_piSS1loc+=iWidthG;m_piSS2loc+=iWidthG;m_piSS3loc+=iWidthG;m_piSS5loc+=iWidthG;m_piSS6loc+=iWidthG;
        m_pS1loc2+=iWidthG;m_pS2loc2+=iWidthG;m_pS3loc2+=iWidthG;m_pS5loc2+=iWidthG;m_pS6loc2+=iWidthG;
        m_pS1loc_3+=iWidthG;m_pS2loc_3+=iWidthG;m_pS3loc_3+=iWidthG;m_pS5loc_3+=iWidthG;m_pS6loc_3+=iWidthG;
        pDstY += iDstStride;pSrcY0Temp+=iSrc0Stride;pSrcY1Temp+=iSrc1Stride;
        pGradX0+=iWidthG;pGradX1+=iWidthG;pGradY0+=iWidthG;pGradY1+=iWidthG;
      }
    } //bBIOapplied
    rpcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight, bBIOapplied);    
#else
  rpcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight );
#endif
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )
  {
    pcYuvSrc0->copyPartToPartYuv( rpcYuvDst, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )
  {
    pcYuvSrc1->copyPartToPartYuv( rpcYuvDst, uiPartIdx, iWidth, iHeight );
  }
}
#if BIO 
const Short m_lumaGradientFilter[4][BIO_FILTER_LENGTH] =
{
  {  8,      -39,       -3,       46,      -17,        5 },
  {  4,      -17,      -36,       60,      -15,        4 },
  { -1,        4,      -57,       57,       -4,        1 },
  { -4,       15,      -60,       36,       17,       -4 }
};
const Short m_lumaInterpolationFilter[4][BIO_FILTER_LENGTH] =
{
  {      0,    0,  64,   0,    0,    0  },
  {      2,   -9,  57,  19,   -7,    2  },
  {      1,   -7,  38,  38,   -7,    1  },
  {      2,   -7,  19,  57,   -9,    2  },
};


__inline Void TComPrediction::xCTI_Filter2DVerG (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  
   Pel*& rpiDst, Int iMV)
{
  Pel*   piDst = rpiDst;
  Int   iSum =0;
  Pel*  piSrcTmp  = piSrc-BIO_FILTER_HALF_LENGTH_MINUS_1*iSrcStride;
  Pel*  piSrcTmp1  = piSrcTmp+iSrcStride;
  Pel*  piSrcTmp2  = piSrcTmp1+iSrcStride;
  Pel*  piSrcTmp3  = piSrcTmp2+iSrcStride;
  Pel*  piSrcTmp4  = piSrcTmp3+iSrcStride;
  Pel*  piSrcTmp5  = piSrcTmp4+iSrcStride;
   Int iShift = g_bitDepthY-8; // MC interpolation filter bit-depth + gradient's filter bit-depth
   Int iOffSet = iShift>0?(1<<(iShift-1)):0;
  for ( Int y = iHeight; y != 0; y-- )
      {

      for ( Int x = 0; x < iWidth; x++ )
      {
        iSum      = m_lumaGradientFilter[iMV][0]* piSrcTmp[x] 
                   +m_lumaGradientFilter[iMV][1]*piSrcTmp1[x]
               +m_lumaGradientFilter[iMV][2]*piSrcTmp2[x]
               +m_lumaGradientFilter[iMV][3]*piSrcTmp3[x] 
               +m_lumaGradientFilter[iMV][4]*piSrcTmp4[x] 
               +m_lumaGradientFilter[iMV][5]*piSrcTmp5[x];
        piDst[x ] = (Pel) ((iSum>=0)?((iSum +iOffSet)>> iShift):(-((-iSum +iOffSet)>> iShift)));

      }
      piSrcTmp+=iSrcStride;
      piSrcTmp1+=iSrcStride;
      piSrcTmp2+=iSrcStride;
      piSrcTmp3+=iSrcStride;
      piSrcTmp4+=iSrcStride;
      piSrcTmp5+=iSrcStride;
      piSrc += iSrcStride;
      piDst += iDstStride;
      }
  return;
}
__inline Void TComPrediction::xCTI_Filter1DVerG (Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, 
  Pel*& rpiDst, Int iMV)
{
  Pel*  piDst = rpiDst;
  Pel*  piSrcTmp  = piSrc-BIO_FILTER_HALF_LENGTH_MINUS_1*iSrcStride;
  Pel*  piSrcTmp1  = piSrcTmp+iSrcStride;
  Pel*  piSrcTmp2  = piSrcTmp1+iSrcStride;
  Pel*  piSrcTmp3  = piSrcTmp2+iSrcStride;
  Pel*  piSrcTmp4  = piSrcTmp3+iSrcStride;
  Pel*  piSrcTmp5  = piSrcTmp4+iSrcStride;
  Int iSum = 0;
   Int iShift = 4;       
   Int iOffSet = 8;  
  for ( Int y = iHeight; y != 0; y-- )
        {
        for ( Int x = 0; x < iWidth; x++ )
        {
        iSum      = m_lumaGradientFilter[iMV][0]* piSrcTmp[x] 
        +m_lumaGradientFilter[iMV][1]*piSrcTmp1[x]
        +m_lumaGradientFilter[iMV][2]*piSrcTmp2[x]
        +m_lumaGradientFilter[iMV][3]*piSrcTmp3[x] 
        +m_lumaGradientFilter[iMV][4]*piSrcTmp4[x] 
        +m_lumaGradientFilter[iMV][5]*piSrcTmp5[x];
      piDst   [x ] =  (Pel) (  (iSum>=0)? ((iSum + iOffSet) >>  iShift):(- ((-iSum + iOffSet) >>  iShift ))   );
        }
      piSrcTmp+=iSrcStride;
      piSrcTmp1+=iSrcStride;
      piSrcTmp2+=iSrcStride;
      piSrcTmp3+=iSrcStride;
      piSrcTmp4+=iSrcStride;
      piSrcTmp5+=iSrcStride;
      piSrc += iSrcStride;
        piDst += iDstStride;
        }
  return;
}
__inline Void TComPrediction::xCTI_Filter1DHorG(Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, 
  Pel*& rpiDst, Int iMV)
{
  Pel*  piDst    = rpiDst;
  Int   iSum =0;
  Pel*  piSrcTmp;
   Int iShift = 4;       
   Int iOffSet = 8;  
   for ( Int y = iHeight; y != 0; y-- )
        {
          piSrcTmp = &piSrc[ -BIO_FILTER_HALF_LENGTH_MINUS_1 ];
        for ( Int x = 0; x < iWidth; x++ )
        {
          iSum      = m_lumaGradientFilter[iMV][0]* piSrcTmp[0] 
          +m_lumaGradientFilter[iMV][1]*piSrcTmp[1]
          +m_lumaGradientFilter[iMV][2]*piSrcTmp[2]
          +m_lumaGradientFilter[iMV][3]*piSrcTmp[3] 
          +m_lumaGradientFilter[iMV][4]*piSrcTmp[4] 
          +m_lumaGradientFilter[iMV][5]*piSrcTmp[5];

      piDst   [x ] =  (Pel) (  (iSum>=0)? ((iSum + iOffSet) >>  iShift):(- ((-iSum + iOffSet) >>  iShift ))   );
            piSrcTmp++;
        }
        piSrc += iSrcStride;
        piDst += iDstStride;
        }

  return;
}
__inline Void TComPrediction::xCTI_Filter2DHorG( Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  
  Pel*& rpiDst, Int iMV)
{
  Pel*  piDst    = rpiDst;
  Int   iSum=0;
  Pel*  piSrcTmp;
   Int iShift = 6+4-(g_bitDepthY-8); // MC interpolation filter bit-depth + gradient's filter bit-depth
   Int iOffSet = iShift>0?(1<<(iShift-1)):0;

     for ( Int y = iHeight; y != 0; y-- )
        {
          piSrcTmp = &piSrc[ -BIO_FILTER_HALF_LENGTH_MINUS_1 ];
        for ( Int x = 0; x < iWidth; x++ )
        {

          iSum      = m_lumaGradientFilter[iMV][0]* piSrcTmp[0] 
          +m_lumaGradientFilter[iMV][1]*piSrcTmp[1]
          +m_lumaGradientFilter[iMV][2]*piSrcTmp[2]
          +m_lumaGradientFilter[iMV][3]*piSrcTmp[3] 
          +m_lumaGradientFilter[iMV][4]*piSrcTmp[4] 
          +m_lumaGradientFilter[iMV][5]*piSrcTmp[5];

          piDst[x ] = (iSum>=0)?  Pel((iSum +  iOffSet) >>  iShift ): Pel(- ((-iSum +  iOffSet) >>  iShift  ));
          piSrcTmp++;
        }
        piSrc += iSrcStride;
        piDst += iDstStride;
        }

  return;
}
__inline Void TComPrediction::xCTI_Filter2DVerGG (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  
   Pel*& rpiDst, Int iMV)
{
    Pel* piDst = rpiDst;
  Int   iSum=0;
  Pel*  piSrcTmp  = piSrc-BIO_FILTER_HALF_LENGTH_MINUS_1*iSrcStride;
  Pel*  piSrcTmp1  = piSrcTmp+iSrcStride;
  Pel*  piSrcTmp2  = piSrcTmp1+iSrcStride;
  Pel*  piSrcTmp3  = piSrcTmp2+iSrcStride;
  Pel*  piSrcTmp4  = piSrcTmp3+iSrcStride;
  Pel*  piSrcTmp5  = piSrcTmp4+iSrcStride;

   Int iShift = g_bitDepthY-8; // MC interpolation filter bit-depth + gradient's filter bit-depth
   Int iOffSet = iShift>0?(1<<(iShift-1)):0;

    for ( Int y = iHeight; y != 0; y-- )
      {
      for ( Int x = 0; x < iWidth; x++ )
      {
        iSum      = m_lumaInterpolationFilter[iMV][0]* piSrcTmp[x] 
        +m_lumaInterpolationFilter[iMV][1]*piSrcTmp1[x]
        +m_lumaInterpolationFilter[iMV][2]*piSrcTmp2[x]
          +m_lumaInterpolationFilter[iMV][3]*piSrcTmp3[x] 
          +m_lumaInterpolationFilter[iMV][4]*piSrcTmp4[x] 
          +m_lumaInterpolationFilter[iMV][5]*piSrcTmp5[x];
        piDst[x ] = (Pel) ((iSum>=0)?((iSum +iOffSet)>> iShift):(-((-iSum +iOffSet)>> iShift)));

      }
      piSrcTmp+=iSrcStride;
      piSrcTmp1+=iSrcStride;
      piSrcTmp2+=iSrcStride;
      piSrcTmp3+=iSrcStride;
      piSrcTmp4+=iSrcStride;
      piSrcTmp5+=iSrcStride;
      piSrc += iSrcStride;
      piDst += iDstStride;
      }

  return;
}
__inline Void TComPrediction::xCTI_Filter2DHorGG( Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMV)
{
  Pel*  piDst    = rpiDst;
  Int   iSum=0;
   Pel*  piSrcTmp;
   Int iShift = 6+4-(g_bitDepthY-8); // MC interpolation filter bit-depth + gradient's filter bit-depth
   Int iOffSet = iShift>0?(1<<(iShift-1)):0;
      for ( Int y = iHeight; y != 0; y-- )
      {
        piSrcTmp = &piSrc[ -BIO_FILTER_HALF_LENGTH_MINUS_1 ];
      for ( Int x = 0; x < iWidth; x++ )
      {
        iSum      = m_lumaInterpolationFilter[iMV][0]* piSrcTmp[0] 
        +m_lumaInterpolationFilter[iMV][1]*piSrcTmp[1]
        +m_lumaInterpolationFilter[iMV][2]*piSrcTmp[2]
          +m_lumaInterpolationFilter[iMV][3]*piSrcTmp[3]
          + m_lumaInterpolationFilter[iMV][4]*piSrcTmp[4] 
          +m_lumaInterpolationFilter[iMV][5]*piSrcTmp[5];

        piDst[x ] =(iSum>=0)?  Pel((iSum +  iOffSet) >>  iShift ) :  Pel(- ((-iSum +  iOffSet) >>  iShift  ));
          piSrcTmp++;
      }
      piSrc += iSrcStride;
      piDst += iDstStride;
      }
      
  return;
}
#endif
// AMVP
Void TComPrediction::getMvPredAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
  if( pcAMVPInfo->iN <= 1 )
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[0];

    pcCU->setMVPIdxSubParts( 0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }

  assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
  rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
  return;
}

/** Function for deriving planar intra prediction.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 *
 * This function derives the prediction samples for planar mode (intra coding).
 */
Void TComPrediction::xPredIntraPlanar( Int* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height )
{
  assert(width == height);

  Int k, l, bottomLeft, topRight;
  Int horPred;
  Int leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  UInt blkSize = width;
  UInt offset2D = width;
  UInt shift1D = g_aucConvertToBit[ width ] + 2;
  UInt shift2D = shift1D + 1;

  // Get left and above reference column and row
  for(k=0;k<blkSize+1;k++)
  {
    topRow[k] = pSrc[k-srcStride];
    leftColumn[k] = pSrc[k*srcStride-1];
  }

  // Prepare intermediate variables used in interpolation
  bottomLeft = leftColumn[blkSize];
  topRight   = topRow[blkSize];
  for (k=0;k<blkSize;k++)
  {
    bottomRow[k]   = bottomLeft - topRow[k];
    rightColumn[k] = topRight   - leftColumn[k];
    topRow[k]      <<= shift1D;
    leftColumn[k]  <<= shift1D;
  }

  // Generate prediction signal
  for (k=0;k<blkSize;k++)
  {
    horPred = leftColumn[k] + offset2D;
    for (l=0;l<blkSize;l++)
    {
      horPred += rightColumn[k];
      topRow[l] += bottomRow[l];
      rpDst[k*dstStride+l] = ( (horPred + topRow[l]) >> shift2D );
    }
  }
}

/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void TComPrediction::xDCPredFiltering( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight )
{
  Pel* pDst = rpDst;
  Int x, y, iDstStride2, iSrcStride2;

  // boundary pixels processing
  pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);

  for ( x = 1; x < iWidth; x++ )
  {
    pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
  }

  for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
  {
    pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
  }

  return;
}

#if INTRA_BOUNDARY_FILTER
Void TComPrediction::xIntraPredFilteringMode34( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight )
{
  Pel* pDst = rpDst;

  for ( Int y = 0, iDstStride2 = 0, iSrcStride2 = -1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
  {
    pDst[iDstStride2  ] = (  8 * pDst[iDstStride2  ] + 8 * pSrc[iSrcStride2+iSrcStride  ] + 8 ) >> 4;
#if INTRA_BOUNDARY_FILTER_MULTI_LINE
    pDst[iDstStride2+1] = ( 12 * pDst[iDstStride2+1] + 4 * pSrc[iSrcStride2+iSrcStride*2] + 8 ) >> 4;     
    pDst[iDstStride2+2] = ( 14 * pDst[iDstStride2+2] + 2 * pSrc[iSrcStride2+iSrcStride*3] + 8 ) >> 4;    
    pDst[iDstStride2+3] = ( 15 * pDst[iDstStride2+3] +     pSrc[iSrcStride2+iSrcStride*4] + 8 ) >> 4;
#endif
  }
  return;
}

Void TComPrediction::xIntraPredFilteringMode02( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight )
{
  Pel* pDst = rpDst;

  for ( Int x = 0; x < iWidth; x++ )
  {
    pDst[x             ] = (  8 * pDst[x             ] + 8 * pSrc[x - iSrcStride + 1] + 8 ) >> 4;
#if INTRA_BOUNDARY_FILTER_MULTI_LINE
    pDst[x+iDstStride  ] = ( 12 * pDst[x+iDstStride  ] + 4 * pSrc[x - iSrcStride + 2] + 8 ) >> 4;
    pDst[x+iDstStride*2] = ( 14 * pDst[x+iDstStride*2] + 2 * pSrc[x - iSrcStride + 3] + 8 ) >> 4;
    pDst[x+iDstStride*3] = ( 15 * pDst[x+iDstStride*3] +     pSrc[x - iSrcStride + 4] + 8 ) >> 4; 
#endif
  }
  return;
}

Void TComPrediction::xIntraPredFilteringModeDGL( Int* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight, UInt uiMode )
{
  Pel* pDst = rpDst;

#if QC_USE_65ANG_MODES
  const Int aucAngPredFilterCoef[8][3] = {
    { 12, 3, 1 }, 
    { 12, 3, 1 }, 
    { 12, 1, 3 }, 
    { 12, 2, 2 }, 
    { 12, 2, 2 },
    { 12, 3, 1 },
    {  8, 6, 2 },  
    {  8, 7, 1 },  
  };
  const Int aucAngPredPosiOffset[8][2] = {
    { 2, 3 }, 
    { 2, 3 }, 
    { 1, 2 },
    { 1, 2 },
    { 1, 2 },
    { 1, 2 },
    { 1, 2 },
    { 1, 2 },
  };
  assert( ( uiMode>=(VDIA_IDX-8) && uiMode<VDIA_IDX ) || ( uiMode>2 && uiMode<=(2+8) ) );
#else
  const Int aucAngPredFilterCoef[4][3] = {
    { 12, 3, 1 }, 
    { 12, 1, 3 }, 
    { 12, 2, 2 },
    {  8, 6, 2 },  
  };
  const Int aucAngPredPosiOffset[4][2] = {
    { 2, 3 }, 
    { 1, 2 },
    { 1, 2 },
    { 1, 2 },
  };
  assert( ( uiMode>=30 && uiMode<34 ) || ( uiMode>2 && uiMode<=6 ) );
#endif

#if QC_USE_65ANG_MODES
  Bool bHorz = (uiMode < DIA_IDX);
  UInt deltaAng = bHorz ? ((2+8)-uiMode): (uiMode-(VDIA_IDX-8));
#else
  Bool bHorz = (uiMode < 18);
  UInt deltaAng = bHorz ? (6-uiMode): (uiMode-30);
#endif
  const Int *offset = aucAngPredPosiOffset[deltaAng];
  const Int *filter = aucAngPredFilterCoef[deltaAng];

  if(bHorz)
  {
    for ( Int x = 0; x < iWidth; x++ )
    {
      pDst[x] = ( filter[0] * pDst[x] 
      + filter[1] * pSrc[x - iSrcStride + offset[0]] 
      + filter[2] * pSrc[x - iSrcStride + offset[1]] + 8) >> 4;
    }
  }
  else
  {
    for ( Int y = 0; y < iHeight; y++ )
    {         
      pDst[y * iDstStride] = ( filter[0] * pDst[y * iDstStride] 
      + filter[1] * pSrc[(y + offset[0] ) * iSrcStride -1 ] 
      + filter[2] * pSrc[(y + offset[1] ) * iSrcStride -1 ] + 8) >> 4;        
    }
  }

  return;
}
#endif

#if QC_LMCHROMA
/** Function for deriving chroma LM intra prediction.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param piSrc pointer to reconstructed chroma sample array
 * \param pPred pointer for the prediction sample array
 * \param uiPredStride the stride of the prediction sample array
 * \param uiCWidth the width of the chroma block
 * \param uiCHeight the height of the chroma block
 *
 * This function derives the prediction samples for chroma LM mode (chroma intra coding)
 */
Void TComPrediction::predLMIntraChroma( TComPattern* pcPattern, UInt uiChromaId, Pel* pPred, UInt uiPredStride, UInt uiCWidth, UInt uiCHeight )
{
  // LLS parameters estimation -->
  Int a, b, iShift;
  xGetLMParameters( pcPattern, uiCWidth, uiCHeight, 0, uiChromaId, a, b, iShift );

  // get prediction -->
  Int  iLumaStride = m_iLumaRecStride;
  Int  *pLuma = m_pLumaRecBuffer + iLumaStride + 1;

  for( Int i = 0; i < uiCHeight; i++ )
  {
    for( Int j = 0; j < uiCWidth; j++ )
    {
      pPred[j] = ClipC( ( ( a * pLuma[j] ) >> iShift ) + b );
    }

    pPred += uiPredStride;
    pLuma += iLumaStride;
  }
  // <-- end of get prediction
}

/** Function for deriving downsampled luma sample of current chroma block and its above, left causal pixel
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param uiCWidth the width of the chroma block
 * \param uiCHeight the height of the chroma block
 * \param bLeftPicBoundary indication of the chroma block located on the left picture boundary
 *
 * This function derives downsampled luma sample of current chroma block and its above, left causal pixel
 */
Void TComPrediction::getLumaRecPixels( TComPattern* pcPattern, UInt uiCWidth, UInt uiCHeight, Bool bLeftPicBoundary )
{
  Pel* pRecSrc0 = pcPattern->getROIY();
  Int* pDst0 = m_pLumaRecBuffer + m_iLumaRecStride + 1;

  Int iRecSrcStride = pcPattern->getPatternLStride();
  Int iRecSrcStride2 = iRecSrcStride << 1;
  Int iDstStride = m_iLumaRecStride;

  UInt uiWidth  = 2 * uiCWidth;
  UInt uiHeight = 2 * uiCHeight;  
  Int* ptrSrc = pcPattern->getAdiOrgBuf( uiWidth, uiHeight, m_piYuvExt );
  Int iSrcStride = ( max( uiWidth, uiHeight ) << 1 ) + 1;

  iSrcStride = ( max( uiWidth, uiHeight ) << 1 ) + 3;
  Int iSrcStride2 = iSrcStride << 1;
  Int* pDst = pDst0 - 1 - iDstStride;  
  Int* piSrc = ptrSrc;

  // top row downsampled from ADI buffer
  pDst++;     
  piSrc += 3;
  for (Int i = 0; i < uiCWidth; i++)
  {
    pDst[i] = ( ((piSrc[2*i]              * 2 ) + piSrc[2*i - 1]              + piSrc[2*i + 1]             )
      + ((piSrc[2*i + iSrcStride] * 2 ) + piSrc[2*i - 1 + iSrcStride] + piSrc[2*i + 1 + iSrcStride])
      + 4) >> 3;
  }

  // left column downsampled from ADI buffer
  pDst = pDst0 - 1; 
  piSrc = ptrSrc + iSrcStride2;
  for (Int j = 0; j < uiCHeight; j++)
  {
    pDst[0] = (  (piSrc[1]              *2 + piSrc[0]          + piSrc[2]             ) 
      + (piSrc[1 + iSrcStride] *2 + piSrc[iSrcStride] + piSrc[2 + iSrcStride])
      + 4) >> 3;
    piSrc += iSrcStride2; 
    pDst += iDstStride;    
  }

  // inner part from reconstructed picture buffer
  for( Int j = 0; j < uiCHeight; j++ )
  {
    for (Int i = 0; i < uiCWidth; i++)
    {
      if(i==0 && bLeftPicBoundary)
      {
        pDst0[i] = ( pRecSrc0[2*i] + pRecSrc0[2*i + iRecSrcStride] + 1) >> 1;
      }
      else
      {
        pDst0[i] = ( pRecSrc0[2*i]              * 2 + pRecSrc0[2*i + 1]                 + pRecSrc0[2*i -1 ] 
        + pRecSrc0[2*i + iRecSrcStride]* 2 + pRecSrc0[2*i + 1 + iRecSrcStride] + pRecSrc0[2*i -1 + iRecSrcStride]
        + 4) >> 3;
      }
    }
    pDst0 += iDstStride;
    pRecSrc0 += iRecSrcStride2;
  }
}

/** Function for deriving LM parameter for predciton of Cr from Cb.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param uiWidth the width of the chroma block
 * \param uiHeight the height of the chroma block
 * \param a the weight of the linear prediction model
 * \param b the offset of the linear prediction model
 * \param iShift the shifting bits of of the linear prediction model
 *
 * This function derives the parameters of linear prediction model
 */
Void TComPrediction::addCrossColorResi( TComPattern* pcPattern, Pel* piPred, UInt uiPredStride, UInt uiWidth, UInt uiHeight, Pel* piResi, UInt uiResiStride )
{
  Int a, b, iShift;

  xGetLMParameters( pcPattern, uiWidth, uiHeight, 1, 1, a, b, iShift );

  Int offset = 1 << (iShift - 1);

  if (a >= 0)
  {
    return;
  }

  Pel*  pPred   = piPred;
  Pel*  pResi   = piResi;

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      pPred[ uiX ] = ClipC(pPred[ uiX ] + (( pResi[ uiX ] * a + offset) >> iShift ) );
    }
    pPred += uiPredStride;
    pResi += uiResiStride;
  }
}

/** Function for deriving the positon of first non-zero binary bit of a value
 * \param x input value
 *
 * This function derives the positon of first non-zero binary bit of a value
 */
Int GetFloorLog2( UInt x )
{
  int bits = -1;
  while( x > 0 )
  {
    bits ++;
    x >>= 1;
  }
  return bits;
}

/** Function for deriving the parameters of linear prediction model.
 * \param x, y, xx, yy sum of reference samples of source component, target component, square of source component and multiplication of source component and target component
 * \param iCountShift, count of reference samples
 * \param iPredType indication of the cross-componennt preidciton type, 0: chroma from luma, 1: Cr from Cb
 * \param a the weight of the linear prediction model
 * \param b the offset of the linear prediction model
 * \param iShift the shifting bits of of the linear prediction model
 *
 * This function derives the parameters of linear prediction model
 */

Void TComPrediction::xCalcLMParameters( Int x, Int y, Int xx, Int xy, Int iCountShift, Int iPredType, Int &a, Int &b, Int &iShift )
{
  Int avgX =  x  >> iCountShift;
  Int avgY =  y  >> iCountShift;

  Int RErrX = x & ( ( 1 << iCountShift ) - 1 );
  Int RErrY =  y & ( ( 1 << iCountShift ) - 1 );

  Int iB = 7;
  iShift = 13 - iB;

  UInt uiInternalBitDepth = g_bitDepthC; // need consider different bit depth later ????????????

  if( iCountShift == 0 )
  {
    a = 0;
    b = 1 << (uiInternalBitDepth - 1);
    iShift = 0;
  }
  else
  {
    Int a1 = xy - ( avgX*avgY << iCountShift ) - avgX*RErrY - avgY*RErrX;
    Int a2 = xx - ( avgX*avgX << iCountShift ) - 2*avgX*RErrX ;

    if ( iPredType == 1) // Cr residual predicted from Cb residual, Cr from Cb
    {
      a1 += -1*( xx >> (CR_FROM_CB_REG_COST_SHIFT + 1 ));
      a2 += xx >> CR_FROM_CB_REG_COST_SHIFT;
    }

    const Int iShiftA1 = uiInternalBitDepth - 2;
    const Int iShiftA2 = 5;
    const Int iAccuracyShift = uiInternalBitDepth + 4;

    Int iScaleShiftA2 = 0;
    Int iScaleShiftA1 = 0;
    Int a1s = a1;
    Int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : GetFloorLog2( abs( a1 ) ) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : GetFloorLog2( abs( a2 ) ) - iShiftA2;

    if( iScaleShiftA1 < 0 )
    {
      iScaleShiftA1 = 0;
    }
    
    if( iScaleShiftA2 < 0 )
    {
      iScaleShiftA2 = 0;
    }

    Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if (a2s >= 32)
    {
      UInt a2t = m_uiaLMShift[ a2s - 32 ] ;
      a2t = ClipC( a2t );  //????????????
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }
    
    if( iScaleShiftA < 0 )
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3(-( 1 << (15-iB) ), ( 1 << (15-iB )) - 1, a);
    a = a << iB;
   
    Short n = 0;
    if (a != 0)
    {
      n = GetFloorLog2(abs( a ) + ( (a < 0 ? -1 : 1) - 1)/2 ) - 5;
    }
    
    iShift =(iShift+iB)-n;
    a = a>>n;

    b =  avgY - ( (  a * avgX ) >> iShift );
  }   

}
/** Function for deriving LM parameter for predciton of Cr from Cb.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param uiWidth the width of the chroma block
 * \param uiHeight the height of the chroma block
 * \param a the weight of the linear prediction model
 * \param b the offset of the linear prediction model
 * \param iShift the shifting bits of of the linear prediction model
 *
 * This function derives the parameters of linear prediction model
 */
Void TComPrediction::xGetLMParameters( TComPattern* pcPattern, UInt uiWidth, UInt uiHeight, Int iPredType, UInt uiChromaId, Int &a, Int &b, Int &iShift )
{
  Int *pSrcColor0, *pCurChroma0; 
  Int iSrcStride, iCurStride;
  UInt uiInternalBitDepth = g_bitDepthC; 

  if (iPredType == 0) //chroma from luma
  {
    iSrcStride = m_iLumaRecStride;
    pSrcColor0 = m_pLumaRecBuffer + iSrcStride + 1;

    pCurChroma0  = ( uiChromaId > 0 ? pcPattern->getAdiCrBuf( uiWidth, uiHeight, m_piYuvExt ) : pcPattern->getAdiCbBuf( uiWidth, uiHeight, m_piYuvExt ) );
    iCurStride   = 2 * uiWidth+ 1;
    pCurChroma0 += iCurStride + 1;
  }
  else
  {
    assert (uiChromaId == 1);

    pSrcColor0   = pcPattern->getAdiCbBuf( uiWidth, uiHeight, getPredicBuf() );
    pCurChroma0  = pcPattern->getAdiCrBuf( uiWidth, uiHeight, getPredicBuf() ); 

    iSrcStride = 2 * uiWidth+ 1;
    iCurStride = 2 * uiWidth+ 1;

    pSrcColor0  += iSrcStride + 1;
    pCurChroma0 += iCurStride + 1;
  }

  Int x = 0, y = 0, xx = 0, xy = 0;
  Int i, j;
  Int iCountShift = 0;

  Int *pSrc = pSrcColor0 - iSrcStride;
  Int *pCur = pCurChroma0 - iCurStride;

  for( j = 0; j < uiWidth; j++ )
  {
    x += pSrc[j];
    y += pCur[j];
    xx += pSrc[j] * pSrc[j];
    xy += pSrc[j] * pCur[j];
  }

  pSrc  = pSrcColor0 - 1;
  pCur = pCurChroma0 - 1;

  for( i = 0; i < uiHeight; i++ )
  {
    x += pSrc[0];
    y += pCur[0];
    xx += pSrc[0] * pSrc[0];
    xy += pSrc[0] * pCur[0];

    pSrc += iSrcStride;
    pCur += iCurStride;
  }

  iCountShift = g_aucConvertToBit[ uiWidth ] + 3;

  Int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if( iTempShift > 0 )
  {
    x  = ( x +  ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    y  = ( y +  ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xx = ( xx + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xy = ( xy + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    iCountShift -= iTempShift;
  }

  // LLS parameters estimation -->
  xCalcLMParameters( x, y, xx, xy, iCountShift, iPredType, a, b, iShift );
}

#endif

#if QC_FRUC_MERGE
/**
 * \brief calculate the cost of template matching in FRUC
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param nWidth          Width of the block
 * \param nHeight         Height of the block
 * \param eCurRefPicList  Reference picture list
 * \param rCurMvField     Mv to be checked
 * \param uiMVCost        Cost of the Mv
 */
UInt TComPrediction::xFrucGetTempMatchCost( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nWidth , Int nHeight , RefPicList eCurRefPicList , const TComMvField & rCurMvField , UInt uiMVCost )
{
#if QC_MV_STORE_PRECISION_BIT
  const Int nMVUnit = QC_MV_STORE_PRECISION_BIT;
#else
  const Int nMVUnit = 2;
#endif

  UInt uiCost = uiMVCost;
  DistParam cDistParam;
  cDistParam.bApplyWeight = false;
  TComPicYuv * pRefPicYuv = pcCU->getSlice()->getRefPic( eCurRefPicList , rCurMvField.getRefIdx() )->getPicYuvRec();
  TComYuv * pYuvPredRefTop = &m_acYuvPred[0] , * pYuvPredRefLeft = &m_acYuvPred[1];
  TComYuv * pYuvPredCurTop = &m_cYuvPredFrucTemplate[0] , * pYuvPredCurLeft = &m_cYuvPredFrucTemplate[1];
  if( m_bFrucTemplateAvailabe[0] )
  {
    TComMv mvTop( 0 , - ( QC_FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) );
    mvTop += rCurMvField.getMv();
    xPredInterLumaBlk( pcCU , pRefPicYuv , uiAbsPartIdx , &mvTop , nWidth , QC_FRUC_MERGE_TEMPLATE_SIZE , pYuvPredRefTop , false , 
#if BIO
false,
#endif
QC_FRUC_MERGE_TEMPLATE );
    m_cFRUCRDCost.setDistParam( cDistParam , g_bitDepthY , pYuvPredCurTop->getLumaAddr( 0 ) , pYuvPredCurTop->getStride() ,
      pYuvPredRefTop->getLumaAddr( 0 ) , pYuvPredRefTop->getStride() , nWidth , QC_FRUC_MERGE_TEMPLATE_SIZE , false );
#if QC_IC
    cDistParam.bMRFlag = pcCU->getICFlag( uiAbsPartIdx );
#endif
    uiCost += cDistParam.DistFunc( &cDistParam );

  }
  if( m_bFrucTemplateAvailabe[1] )
  {
    TComMv mvLeft( - ( QC_FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) , 0 );
    mvLeft += rCurMvField.getMv();
    xPredInterLumaBlk( pcCU , pRefPicYuv , uiAbsPartIdx , &mvLeft , QC_FRUC_MERGE_TEMPLATE_SIZE , nHeight , pYuvPredRefLeft , false , 
#if BIO
false,
#endif
QC_FRUC_MERGE_TEMPLATE );
    m_cFRUCRDCost.setDistParam( cDistParam , g_bitDepthY , pYuvPredCurLeft->getLumaAddr( 0 ) , pYuvPredCurLeft->getStride() ,
      pYuvPredRefLeft->getLumaAddr( 0 ) , pYuvPredRefLeft->getStride() , QC_FRUC_MERGE_TEMPLATE_SIZE , nHeight , false );
#if QC_IC
    cDistParam.bMRFlag = pcCU->getICFlag( uiAbsPartIdx );
#endif
    uiCost += cDistParam.DistFunc( &cDistParam );

  }

  return( uiCost );
}

/**
 * \brief calculate the cost of bilateral matching in FRUC
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param nWidth          Width of the block
 * \param nHeight         Height of the block
 * \param eCurRefPicList  Reference picture list
 * \param rCurMvField     Mv to be checked
 * \param rPairMVField    paired Mv based on bilateral assumption
 * \param uiMVCost        Cost of the Mv
 */
UInt TComPrediction::xFrucGetBilaMatchCost( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nWidth , Int nHeight , RefPicList eCurRefPicList , const TComMvField & rCurMvField , TComMvField & rPairMVField , UInt uiMVCost )
{
  UInt uiCost = MAX_UINT;
  if( pcCU->getMvPair( eCurRefPicList , rCurMvField , rPairMVField ) )
  {
    RefPicList eTarRefPicList = ( RefPicList )( 1 - ( Int )eCurRefPicList );
    TComPicYuv * pRefPicYuvA = pcCU->getSlice()->getRefPic( eCurRefPicList , rCurMvField.getRefIdx() )->getPicYuvRec();
    TComPicYuv * pRefPicYuvB = pcCU->getSlice()->getRefPic( eTarRefPicList , rPairMVField.getRefIdx() )->getPicYuvRec();
    TComYuv * pYuvPredA = &m_acYuvPred[0];
    TComYuv * pYuvPredB = &m_acYuvPred[1];
    TComMv mvOffset( 0 , 0 );
    TComMv mvAp = rCurMvField.getMv() + mvOffset;
    xPredInterLumaBlk( pcCU , pRefPicYuvA , uiAbsPartIdx , &mvAp , nWidth , nHeight , pYuvPredA , false 
#if BIO                  
,false
#endif
, QC_FRUC_MERGE_BILATERALMV );
    TComMv mvBp = rPairMVField.getMv() + mvOffset;
    xPredInterLumaBlk( pcCU , pRefPicYuvB , uiAbsPartIdx , &mvBp , nWidth , nHeight , pYuvPredB , false 
#if BIO                  
,false
#endif
, QC_FRUC_MERGE_BILATERALMV );
    DistParam cDistParam;
    cDistParam.bApplyWeight = false;
    m_cFRUCRDCost.setDistParam( cDistParam , g_bitDepthY , pYuvPredA->getLumaAddr( 0 ) , pYuvPredA->getStride() ,
      pYuvPredB->getLumaAddr( 0 ) , pYuvPredB->getStride() , nWidth , nHeight , false );
#if QC_IC
    cDistParam.bMRFlag = pcCU->getICFlag( uiAbsPartIdx );
#endif
    uiCost = cDistParam.DistFunc( &cDistParam ) + uiMVCost;
  }

  return( uiCost );
}

/**
 * \brief refine Mv for a block with bilateral matching or template matching and return the min cost so far
 *
 * \param pBestMvField    Pointer to the best Mv (Mv pair) so far
 * \param eCurRefPicList  Reference picture list
 * \param uiMinCost       Min cost so far
 * \param nSearchMethod   Search pattern to be used
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param rMvStart        Searching center
 * \param nBlkWidth       Width of the block
 * \param nBlkHeight      Height of the block
 * \param bTM             Whether is template matching
 */
UInt TComPrediction::xFrucRefineMv( TComMvField * pBestMvField , RefPicList eCurRefPicList , UInt uiMinCost , Int nSearchMethod , TComDataCU * pCU , UInt uiAbsPartIdx , const TComMvField & rMvStart , Int nBlkWidth , Int nBlkHeight , Bool bTM )
{
  Int nSearchStepShift = QC_MV_STORE_PRECISION_BIT - QC_MV_SIGNAL_PRECISION_BIT;

  switch( nSearchMethod )
  {
  case 0:
    // cross
    uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift );
    if( nSearchStepShift > 0 )
    {
      uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , 0 , 1 );
    }
    break;
  case 1:
    // square
    uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_SQUARE>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift );
    if( nSearchStepShift > 0 )
    {
      uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , 0 , 1 );
    }
    break;
  case 2:
    // diamond
    uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift );
    uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift , 1 );
    if( nSearchStepShift > 0 )
    {
      uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , 0 , 1 );
    }
    break;
  case 3:
    // no refinement
    break;
  case 4:
    // hexagon
    uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift );
    uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift , 1 );
    if( nSearchStepShift > 0 )
    {
      uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , 0 , 1 );
    }
    break;
  case 5:
    // adaptive cross 
    if( rMvStart.getRefIdx() != pBestMvField[eCurRefPicList].getRefIdx() || rMvStart.getMv() != pBestMvField[eCurRefPicList].getMv() )
    {
      uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift );
      if( nSearchStepShift > 0 )
      {
        uiMinCost = xFrucRefineMvSearch<QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , 0 , 1 );
      }
    }
    break;
  default:
    assert( 0 );
  }
  
  return( uiMinCost );
}

/**
 * \brief refine Mv for a block with bilateral matching or template matching and return the min cost so far
 *
 * \param pBestMvField    Pointer to the best Mv (Mv pair) so far
 * \param eCurRefPicList  Reference picture list
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param rMvStart        Searching center
 * \param nBlkWidth       Width of the block
 * \param nBlkHeight      Height of the block
 * \param uiMinDist       Min cost so far
 * \param bTM             Whether is template matching
 * \param nSearchStepShift Indicate the searching step, 0 for 1/8 pel, 1 for 1/4 pel
 * \param uiMaxSearchRounds Max rounds of pattern searching
 */
template<Int SearchPattern>
UInt TComPrediction::xFrucRefineMvSearch( TComMvField * pBestMvField , RefPicList eCurRefPicList , TComDataCU * pCU , UInt uiAbsPartIdx , TComMvField const & rMvStart , Int nBlkWidth , Int nBlkHeight , UInt uiMinDist , Bool bTM , Int nSearchStepShift , UInt uiMaxSearchRounds )
{
  const TComMv mvSearchOffsetCross[4] = { TComMv( 0 , 1 ) , TComMv( 1 , 0 ) , TComMv( 0 , -1 ) , TComMv( -1 , 0 ) };
  const TComMv mvSearchOffsetSquare[8] = { TComMv( -1 , 1 ) , TComMv( 0 , 1 ) , TComMv( 1 , 1 ) , TComMv( 1 , 0 ) , TComMv( 1 , -1 ) , TComMv( 0 , -1 ) , TComMv( -1 , -1 ) , TComMv( -1 , 0 )  };
  const TComMv mvSearchOffsetDiamond[8] = { TComMv( 0 , 2 ) , TComMv( 1 , 1 ) , TComMv( 2 , 0 ) , TComMv( 1 , -1 ) , TComMv( 0 , -2 ) , TComMv( -1 , -1 ) , TComMv( -2 , 0 ) , TComMv( -1 , 1 ) };
  const TComMv mvSearchOffsetHexagon[6] = { TComMv( 2 , 0 ) , TComMv( 1 , 2 ) , TComMv( -1 , 2 ) , TComMv( -2 , 0 ) , TComMv( -1 , -2 ) , TComMv( 1 , -2 ) };

  Int nDirectStart = 0 , nDirectEnd = 0 , nDirectRounding = 0 , nDirectMask = 0;
  const TComMv * pSearchOffset;
  if( SearchPattern == QC_FRUC_MERGE_MV_SEARCHPATTERN_CROSS )
  {
    nDirectEnd = 3;
    nDirectRounding = 4;
    nDirectMask = 0x03;
    pSearchOffset = mvSearchOffsetCross;
  }
  else if( SearchPattern == QC_FRUC_MERGE_MV_SEARCHPATTERN_SQUARE )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetSquare;
  }
  else if( SearchPattern == QC_FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetDiamond;
  }
  else if( SearchPattern == QC_FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON )
  {
    nDirectEnd = 5;
    pSearchOffset = mvSearchOffsetHexagon;
  }
  else
  {
    assert( 0 );
  }

  Int nBestDirect;
  const Int & rSearchRange = pCU->getSlice()->getSPS()->getFRUCRefineRange();
  for( UInt uiRound = 0 ; uiRound < uiMaxSearchRounds ; uiRound++ )
  {
    nBestDirect = -1;
    TComMvField mvCurCenter = pBestMvField[eCurRefPicList];
    for( Int nIdx = nDirectStart ; nIdx <= nDirectEnd ; nIdx++ )
    {
      Int nDirect;
      if( SearchPattern == QC_FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON )
      {
        nDirect = nIdx < 0 ? nIdx + 6 : nIdx >= 6 ? nIdx - 6 : nIdx;
      }
      else
      {
        nDirect = ( nIdx + nDirectRounding ) & nDirectMask;
      }

      TComMv mvOffset = pSearchOffset[nDirect];
      mvOffset <<= nSearchStepShift;
      TComMvField mvCand = mvCurCenter;
      mvCand.getMv() += mvOffset;
      UInt uiCost = xFrucGetMvCost( rMvStart.getMv() , mvCand.getMv() , rSearchRange , QC_FRUC_MERGE_REFINE_MVWEIGHT );
      if( uiCost > uiMinDist )
        continue;
      TComMvField mvPair;

      if( bTM )
      {
        if( pCU->isSameMVField( eCurRefPicList , mvCand , ( RefPicList )( !eCurRefPicList ) , pBestMvField[!eCurRefPicList] ) )
          continue;
        uiCost = xFrucGetTempMatchCost( pCU , uiAbsPartIdx , nBlkWidth , nBlkHeight , eCurRefPicList , mvCand , uiCost );
      }
      else
      {
        uiCost = xFrucGetBilaMatchCost( pCU , uiAbsPartIdx , nBlkWidth , nBlkHeight , eCurRefPicList , mvCand , mvPair , uiCost ); 
      }
      if( uiCost < uiMinDist )
      {
        uiMinDist = uiCost;
        pBestMvField[eCurRefPicList] = mvCand;
        if( !bTM )
          pBestMvField[!eCurRefPicList] = mvPair;
        nBestDirect = nDirect;
      }
    }

    if( nBestDirect == -1 )
      break;
    Int nStep = 1;
    if( SearchPattern == QC_FRUC_MERGE_MV_SEARCHPATTERN_SQUARE || SearchPattern == QC_FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND )
    {
      nStep = 2 - ( nBestDirect & 0x01 );
    }
    nDirectStart = nBestDirect - nStep;
    nDirectEnd = nBestDirect + nStep;
  }

  return( uiMinDist );
}

/**
 * \brief Find Mv predictor for a block based on template matching
 *
 * \param pcCU            Pointer to current CU
 * \param uiPUIdx         PU Index
 * \param eTargetRefPicList The reference list for the Mv predictor
 * \param nTargetRefIdx   The reference index for the Mv predictor
 */
Bool TComPrediction::xFrucFindBlkMv4Pred( TComDataCU * pCU , UInt uiPUIdx , RefPicList eTargetRefPicList , Int nTargetRefIdx )
{
  Bool bAvailable = false;
  if( pCU->getSlice()->getSPS()->getUseFRUCMgrMode() )
  {
    UInt uiAbsPartIdx = 0;
    Int nWidth = 0 , nHeight = 0;
    pCU->getPartIndexAndSize( uiPUIdx , uiAbsPartIdx , nWidth , nHeight );
    if( xFrucGetCurBlkTemplate( pCU , uiAbsPartIdx , nWidth , nHeight ) )
    {
      // find best start
      xFrucCollectBlkStartMv( pCU , uiPUIdx , eTargetRefPicList , nTargetRefIdx );
      TComMvField mvStart[2] , mvFinal[2];
      UInt uiMinCost = xFrucFindBestMvFromList( mvStart , eTargetRefPicList , pCU , uiAbsPartIdx , mvStart[eTargetRefPicList] , nWidth , nHeight , true , false );
      if( mvStart[eTargetRefPicList].getRefIdx() >= 0 )
      {
        // refine Mv
        mvFinal[eTargetRefPicList] = mvStart[eTargetRefPicList];
        uiMinCost = xFrucRefineMv( mvFinal , eTargetRefPicList , uiMinCost , 2 , pCU , uiAbsPartIdx , mvStart[eTargetRefPicList] , nWidth , nHeight , true );
        bAvailable = true;
        // save Mv
        pCU->getCUMvField( eTargetRefPicList )->setAllMv( mvFinal[eTargetRefPicList].getMv(), pCU->getPartitionSize( 0 ) , uiAbsPartIdx , 0 , uiPUIdx ); 
      }
    }
  }

  return( bAvailable );
}

/**
 * \brief Find Mv for a block based on template matching or bilateral matching
 *
 * \param pcCU            Pointer to current CU
 * \param uiPUIdx         PU Index
 */
Bool TComPrediction::xFrucFindBlkMv( TComDataCU * pCU , UInt uiPUIdx )
{
  Bool bAvailable = false;
  UInt uiAbsPartIdx = 0;
  Int nWidth = 0 , nHeight = 0;
  pCU->getPartIndexAndSize( uiPUIdx , uiAbsPartIdx , nWidth , nHeight );
  TComMvField mvStart[2] , mvFinal[2];

  const Int nSearchMethod = 2;
  if( pCU->getFRUCMgrMode( uiAbsPartIdx ) == QC_FRUC_MERGE_BILATERALMV )
  {
    if( !pCU->getSlice()->getSPS()->getUseFRUCMgrMode() || pCU->getSlice()->isInterP() )
      return( false );

    xFrucCollectBlkStartMv( pCU , uiPUIdx );
    RefPicList eBestRefPicList = REF_PIC_LIST_0;
    UInt uiMinCost = xFrucFindBestMvFromList( mvStart , eBestRefPicList , pCU , uiAbsPartIdx , mvStart[eBestRefPicList] , nWidth , nHeight , false , false );
    if( mvStart[eBestRefPicList].getRefIdx() >= 0 )
    {
      mvFinal[0] = mvStart[0];
      mvFinal[1] = mvStart[1];
      uiMinCost = xFrucRefineMv( mvFinal , eBestRefPicList , uiMinCost , nSearchMethod , pCU , uiAbsPartIdx , mvStart[eBestRefPicList] , nWidth , nHeight , false );
      bAvailable = true;
    }
  }
  else if( pCU->getFRUCMgrMode( uiAbsPartIdx ) == QC_FRUC_MERGE_TEMPLATE )
  {
    if( !pCU->getSlice()->getSPS()->getUseFRUCMgrMode() )
      return( false );
    if( !xFrucGetCurBlkTemplate( pCU , uiAbsPartIdx , nWidth , nHeight ) )
      return( false );

    xFrucCollectBlkStartMv( pCU , uiPUIdx );
    UInt uiMinCost[2];
    // find the best Mvs from the two lists first and then refine Mvs: try to avoid duplicated Mvs
    for( Int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
    {
      RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
      uiMinCost[nRefPicList] = xFrucFindBestMvFromList( mvStart , eCurRefPicList , pCU , uiAbsPartIdx , mvStart[nRefPicList] , nWidth , nHeight , true , false );
    }
    mvFinal[0] = mvStart[0];
    mvFinal[1] = mvStart[1];
    for( Int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
    {
      if( mvStart[nRefPicList].getRefIdx() >= 0 )
      {
        uiMinCost[nRefPicList] = xFrucRefineMv( mvFinal , ( RefPicList )nRefPicList , uiMinCost[nRefPicList] , nSearchMethod , pCU , uiAbsPartIdx , mvStart[nRefPicList] , nWidth , nHeight , true );
        bAvailable = true;
      }
    }
  }
  else
  {
    assert( 0 );
  }

  if( bAvailable )
  {
    // save Mv
    pCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( mvFinal[0] , pCU->getPartitionSize( uiAbsPartIdx ) , uiAbsPartIdx , 0 , uiPUIdx ); 
    pCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( mvFinal[1] , pCU->getPartitionSize( uiAbsPartIdx ) , uiAbsPartIdx , 0 , uiPUIdx ); 
    UInt uiDir = ( mvFinal[0].getRefIdx() >= 0 ) + ( ( mvFinal[1].getRefIdx() >=0 ) << 1 );
    pCU->setInterDirSubParts( uiDir , uiAbsPartIdx , uiPUIdx , pCU->getDepth( uiAbsPartIdx ) );
  }

  return( bAvailable );
}

/**
 * \brief Refine Mv for each sub-block of the block based on bilateral matching or template matching
 *
 * \param pcCU            Pointer to current CU
 * \param uiDepth         CU depth
 * \param uiPUIdx         PU Index
 * \param bTM             Whether is template matching
 */
Bool TComPrediction::xFrucRefineSubBlkMv( TComDataCU * pCU , UInt uiDepth , UInt uiPUIdx , Bool bTM )
{
  TComCUMvField * pCuMvField0 = pCU->getCUMvField( REF_PIC_LIST_0 );
  TComCUMvField * pCuMvField1 = pCU->getCUMvField( REF_PIC_LIST_1 );
  UInt uiAbsPartIdx = 0;
  Int nWidth = 0 , nHeight = 0;
  pCU->getPartIndexAndSize( uiPUIdx , uiAbsPartIdx , nWidth , nHeight );
  Int nRefineBlockSize = xFrucGetSubBlkSize( pCU , uiAbsPartIdx , nWidth , nHeight );
  UInt uiIdxStep = ( nRefineBlockSize * nRefineBlockSize ) >> 4;
  Int nRefineBlkStep = nRefineBlockSize >> 2;
  const Int nSearchMethod = 5;
#if QC_SUB_PU_TMVP_EXT
  UInt uiSubBlkRasterIdx = 0;
  UInt uiSubBlkRasterStep = nRefineBlkStep * nRefineBlkStep;
#endif
  for( Int y = 0 , yBlk4Offset = 0 ; y < nHeight ; y += nRefineBlockSize , yBlk4Offset += pCU->getPic()->getNumPartInWidth() * nRefineBlkStep )
  {
    for( Int x = 0 , xBlk4Offset = 0 ; x < nWidth ; x += nRefineBlockSize , xBlk4Offset += nRefineBlkStep )
    {
      UInt uiRasterOrder = g_auiZscanToRaster[uiAbsPartIdx+pCU->getZorderIdxInCU()] + yBlk4Offset + xBlk4Offset;
      UInt uiSubBlkIdx = g_auiRasterToZscan[uiRasterOrder] - pCU->getZorderIdxInCU();

      // start from the best Mv of the full block
      TComMvField mvStart[2] , mvFinal[2];
      mvStart[0].setMvField( pCuMvField0->getMv( uiSubBlkIdx ) , pCuMvField0->getRefIdx( uiSubBlkIdx ) );
      mvStart[1].setMvField( pCuMvField1->getMv( uiSubBlkIdx ) , pCuMvField1->getRefIdx( uiSubBlkIdx ) );
      mvFinal[0] = mvStart[0];
      mvFinal[1] = mvStart[1];

      // refinement
      if( bTM )
      {
        if( !xFrucGetCurBlkTemplate( pCU , uiSubBlkIdx , nRefineBlockSize , nRefineBlockSize ) )
          continue;
        for( Int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
        {
          if( mvStart[nRefPicList].getRefIdx() >= 0 )
          {
            RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
            xFrucCollectSubBlkStartMv( pCU , uiSubBlkIdx , eCurRefPicList , mvStart[eCurRefPicList] , nRefineBlockSize , nRefineBlockSize 
#if QC_SUB_PU_TMVP_EXT
              , uiSubBlkRasterIdx , uiSubBlkRasterStep
#endif
              );
            UInt uiMinCost = xFrucFindBestMvFromList( mvFinal , eCurRefPicList , pCU , uiSubBlkIdx , mvStart[eCurRefPicList] , nRefineBlockSize , nRefineBlockSize , bTM , true );
            uiMinCost = xFrucRefineMv( mvFinal , eCurRefPicList , uiMinCost , nSearchMethod , pCU , uiSubBlkIdx , mvStart[eCurRefPicList] , nRefineBlockSize , nRefineBlockSize , bTM );
          }
        }
      }
      else
      {
        xFrucCollectSubBlkStartMv( pCU , uiSubBlkIdx , REF_PIC_LIST_0 , mvStart[REF_PIC_LIST_0] , nRefineBlockSize , nRefineBlockSize 
#if QC_SUB_PU_TMVP_EXT
          , uiSubBlkRasterIdx , uiSubBlkRasterStep
#endif
          );
        RefPicList eBestRefPicList = REF_PIC_LIST_0;
        UInt uiMinCost = xFrucFindBestMvFromList( mvFinal , eBestRefPicList , pCU , uiSubBlkIdx , mvStart[eBestRefPicList] , nRefineBlockSize , nRefineBlockSize , bTM , true );
        uiMinCost = xFrucRefineMv( mvFinal , eBestRefPicList , uiMinCost , nSearchMethod , pCU , uiSubBlkIdx , mvStart[eBestRefPicList] , nRefineBlockSize , nRefineBlockSize , bTM );
      }

      // save Mv
      if( !( mvFinal[0] == mvStart[0] && mvFinal[1] == mvStart[1] ) )
      {
        UInt uiDir = ( mvFinal[0].getRefIdx() >= 0 ) + ( ( mvFinal[1].getRefIdx() >=0 ) << 1 );
        for( UInt n = 0 , uiOffset = uiSubBlkIdx ; n < uiIdxStep ; n++ , uiOffset++ )
        {
          pCU->setInterDir( uiOffset , uiDir );
          pCuMvField0->setMv( mvFinal[0].getMv() , uiOffset );
          pCuMvField0->setRefIdx( mvFinal[0].getRefIdx() , uiOffset );
          pCuMvField1->setMv( mvFinal[1].getMv() , uiOffset );
          pCuMvField1->setRefIdx( mvFinal[1].getRefIdx() , uiOffset );
        }
      }
#if QC_SUB_PU_TMVP_EXT
      uiSubBlkRasterIdx += uiSubBlkRasterStep;
#endif
    }
  }

  return( true );
}

/**
 * \brief Whether a Mv has been checked (in a temp list)
 *
 * \param rMvField        Mv info
 * \param rList           Temp list of Mv
 */
Bool TComPrediction::xFrucIsInList( const TComMvField & rMvField , std::list<TComMvField> & rList )
{
  std::list<TComMvField>::iterator pos = rList.begin();
  while( pos != rList.end() )
  {
    if( rMvField == *pos )
      return( true );
    pos++;
  }
  return( false );
}

/**
 * \brief Insert a Mv to the list to be checked
 *
 * \param rMvField        Mv info
 * \param rList           Temp list of Mv
 */
Void TComPrediction::xFrucInsertMv2StartList( const TComMvField & rMvField , std::list<TComMvField> & rList )
{
  // do not use zoom in FRUC for now
  if( xFrucIsInList( rMvField , rList ) == false )
    rList.push_back( rMvField );
}


/**
 * \brief Collect Mv candidates for a block
 *
 * \param pcCU            Pointer to current CU
 * \param uiPUIdx         PU Index
 * \param eTargetRefPicList The reference list for the Mv predictor
 * \param nTargetRefIdx   The reference index for the Mv predictor
 */
Void TComPrediction::xFrucCollectBlkStartMv( TComDataCU * pCU , UInt uiPUIdx , RefPicList eTargetRefList , Int nTargetRefIdx )
{
  // get merge candidates
  TComMvField  cMvFieldNeighbours[ 2 * MRG_MAX_NUM_CANDS ]; // double length for mv of both lists
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
  Int numValidMergeCand = 0;
  UInt uiAddrOffset = 0;
  Int nWidth = 0 , nHeight = 0;
#if QC_IC
  Bool abICFlag[MRG_MAX_NUM_CANDS];
#endif
  pCU->getPartIndexAndSize( uiPUIdx , uiAddrOffset , nWidth , nHeight );
  pCU->getInterMergeCandidates( uiAddrOffset, uiPUIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand 
#if QC_IC
    , abICFlag
#endif
#if QC_SUB_PU_TMVP
    , m_eMergeCandTypeNieghors , m_cMvFieldSP , m_uhInterDirSP 
#endif
    );

  // add merge candidates to the list
  m_listMVFieldCand[0].clear();
  m_listMVFieldCand[1].clear();
  for( Int nMergeIndex = 0; nMergeIndex < numValidMergeCand + numValidMergeCand ; nMergeIndex++ )
  {
    if( cMvFieldNeighbours[nMergeIndex].getRefIdx() >= 0 
#if QC_SUB_PU_TMVP_EXT
     && m_eMergeCandTypeNieghors[nMergeIndex>>1] != MGR_TYPE_SUBPU_TMVP_EXT
#endif
#if QC_SUB_PU_TMVP
      && m_eMergeCandTypeNieghors[nMergeIndex>>1] != MGR_TYPE_SUBPU_TMVP
#endif
      )
    {
      if( nTargetRefIdx >= 0 
        && ( cMvFieldNeighbours[nMergeIndex].getRefIdx() != nTargetRefIdx 
        || ( nMergeIndex & 0x01 ) != ( Int )eTargetRefList ) )
        continue;
      xFrucInsertMv2StartList( cMvFieldNeighbours[nMergeIndex] , m_listMVFieldCand[nMergeIndex&0x01] );
    }
  }

  // add uni-lateral candidates to the list
  if( pCU->getSlice()->getSPS()->getUseFRUCMgrMode() )
  {
    TComDataCU * pCUFRUC = pCU->getPic()->getCU( pCU->getAddr() );
    TComMvField mvCand;
    UInt uiRasterBase = g_auiZscanToRaster[pCU->getZorderIdxInCU() + uiAddrOffset];
    for( Int y = 0 , yOffset = 0 ; y < nHeight ; y += MIN_PU_SIZE , yOffset += pCU->getPic()->getNumPartInWidth() )
    {
      for( Int x = 0 , xOffset = 0 ; x < nWidth ; x += MIN_PU_SIZE , xOffset++ )
      {
        if( x != 0 && x + x != nWidth )
          continue;
        if( y != 0 && y + y != nHeight )
          continue;
        UInt idx = g_auiRasterToZscan[uiRasterBase+yOffset+xOffset];
        for( Int nList = 0 ; nList < 2 ; nList++ )
        {
          RefPicList eCurList = ( RefPicList )nList;
          if( pCUFRUC->getFRUCUniLateralMVField( eCurList )->getRefIdx( idx ) >= 0 )
          {
            if( nTargetRefIdx >= 0 
              && ( pCUFRUC->getFRUCUniLateralMVField( eCurList )->getRefIdx( idx ) != nTargetRefIdx || eCurList != eTargetRefList ) )
              continue;
            mvCand.setMvField( pCUFRUC->getFRUCUniLateralMVField( eCurList )->getMv( idx ) , pCUFRUC->getFRUCUniLateralMVField( eCurList )->getRefIdx( idx ) );
            xFrucInsertMv2StartList( mvCand , m_listMVFieldCand[nList&0x01] );
          }
        }
      }
    }
  }
}

/**
 * \brief Collect Mv candidates for a sub-block
 *
 * \param pcCU            Pointer to current CU
 * \param uiPUIdx         PU Index
 * \param eRefPicList     The reference list for the Mv predictor
 * \param rMvStart        The searching center
 * \param nSubBlkWidth    Block width
 * \param nSubBlkHeight   Block height
 * \param uiSubBlkRasterIdx Sub-block index in raster scan
 * \param uiSubBlkRasterStep Sub-block step in raster scan
 */
Void TComPrediction::xFrucCollectSubBlkStartMv( TComDataCU * pCU , UInt uiAbsPartIdx , RefPicList eRefPicList , const TComMvField & rMvStart , Int nSubBlkWidth , Int nSubBlkHeight 
#if QC_SUB_PU_TMVP_EXT
  , UInt uiSubBlkRasterIdx , UInt uiSubBlkRasterStep
#endif
  )
{
  std::list<TComMvField> & rStartMvList = m_listMVFieldCand[eRefPicList];
  rStartMvList.clear();

  // start Mv
  xFrucInsertMv2StartList( rMvStart , rStartMvList );

  // zero Mv
  TComMvField mvZero;
  mvZero.setRefIdx( rMvStart.getRefIdx() );
  mvZero.getMv().setZero();
  xFrucInsertMv2StartList( mvZero , rStartMvList );

  Int nCurPOC = pCU->getSlice()->getPOC();
  Int nCurRefPOC = pCU->getSlice()->getRefPOC( eRefPicList , rMvStart.getRefIdx() );
  
  // scaled TMVP, collocated positions and bottom right positions
  UInt uiCUAddr[2] = { pCU->getAddr() , 0 };
  UInt uiColBlockIdx[2] = { uiAbsPartIdx + pCU->getZorderIdxInCU() , 0 };
  Int nMaxPositions = 1 + pCU->getBlockBelowRight( uiAbsPartIdx , nSubBlkWidth , nSubBlkHeight , uiCUAddr[1] , uiColBlockIdx[1] );
  for( Int n = 0 ; n < nMaxPositions ; n++ )
  {
    for( Int nRefIdx = pCU->getSlice()->getNumRefIdx( eRefPicList ) - 1 ; nRefIdx >= 0 ; nRefIdx-- )
    {
      TComMvField mvCand;
      TComPic * pColPic = pCU->getSlice()->getRefPic( eRefPicList , nRefIdx );
      Int nColPOC = pColPic->getPOC();
      TComDataCU * pColCU = pColPic->getCU( uiCUAddr[n] );
      for( Int nRefListColPic = 0 ; nRefListColPic < 2 ; nRefListColPic++ )
      {
        Int nRefIdxColPic = pColCU->getCUMvField( ( RefPicList )nRefListColPic )->getRefIdx( uiColBlockIdx[n] );
        if( nRefIdxColPic >= 0 )
        {
          const TComMv & rColMv = pColCU->getCUMvField( ( RefPicList )nRefListColPic )->getMv( uiColBlockIdx[n] );
          mvCand.setRefIdx( rMvStart.getRefIdx() );
          mvCand.getMv() = pCU->scaleMV( rColMv , nCurPOC , nCurRefPOC , nColPOC , pColCU->getSlice()->getRefPOC( ( RefPicList )nRefListColPic , nRefIdxColPic ) );
          xFrucInsertMv2StartList( mvCand , rStartMvList );
        }
      }
    }
  }

#if QC_SUB_PU_TMVP_EXT
  if( pCU->getSlice()->getSPS()->getAtmvpEnableFlag() )
  {
    for( UInt n = 0 ; n < uiSubBlkRasterStep ; n++ )
    {
      UInt uiIdx = ( ( n + uiSubBlkRasterIdx ) << 1 ) + eRefPicList;
      if( rMvStart.getRefIdx() == m_cMvFieldSP[0][uiIdx].getRefIdx() )
      {
        xFrucInsertMv2StartList( m_cMvFieldSP[0][uiIdx] , rStartMvList );
      }
      if( rMvStart.getRefIdx() == m_cMvFieldSP[1][uiIdx].getRefIdx() )
      {
        xFrucInsertMv2StartList( m_cMvFieldSP[1][uiIdx] , rStartMvList );
      }
    }
  }
#endif

  // scaled interpolated MV 
  if( pCU->getSlice()->getSPS()->getUseFRUCMgrMode() ) 
  {
    TComCUMvField * pFRUCUniLateralMVField = pCU->getPic()->getCU( pCU->getAddr() )->getFRUCUniLateralMVField( eRefPicList );
    Int nRefIdx = pFRUCUniLateralMVField->getRefIdx( uiAbsPartIdx );
    if( nRefIdx >= 0 )
    {
      TComMvField mvCand;
      const TComMv & rMv = pFRUCUniLateralMVField->getMv( uiAbsPartIdx );
      mvCand.setRefIdx( rMvStart.getRefIdx() );
      mvCand.getMv() = pCU->scaleMV( rMv , nCurPOC , nCurRefPOC , nCurPOC , pCU->getSlice()->getRefPOC( eRefPicList , nRefIdx ) );
      xFrucInsertMv2StartList( mvCand , rStartMvList );
    }
  }

  // top neighbor
  UInt uiAbsPartIdxTop = 0;
  TComDataCU * pTop = pCU->getPUAbove( uiAbsPartIdxTop , uiAbsPartIdx + pCU->getZorderIdxInCU() );
  if( pTop != NULL && pTop->getCUMvField( eRefPicList )->getRefIdx( uiAbsPartIdxTop ) == rMvStart.getRefIdx() )
  {
    TComMvField mvCand;
    mvCand.setMvField( pTop->getCUMvField( eRefPicList )->getMv( uiAbsPartIdxTop ) , pTop->getCUMvField( eRefPicList )->getRefIdx( uiAbsPartIdxTop ) );
    xFrucInsertMv2StartList( mvCand , rStartMvList );
  }

  // left neighbor
  UInt uiAbsPartIdxLeft = 0;
  TComDataCU * pLeft = pCU->getPULeft( uiAbsPartIdxLeft , uiAbsPartIdx + pCU->getZorderIdxInCU() );
  if( pLeft != NULL && pLeft->getCUMvField( eRefPicList )->getRefIdx( uiAbsPartIdxLeft ) == rMvStart.getRefIdx() )
  {
    TComMvField mvCand;
    mvCand.setMvField( pLeft->getCUMvField( eRefPicList )->getMv( uiAbsPartIdxLeft ) , pLeft->getCUMvField( eRefPicList )->getRefIdx( uiAbsPartIdxLeft ) );
    xFrucInsertMv2StartList( mvCand , rStartMvList );
  }
}

/**
 * \brief Find the best Mv for Mv candidate list
 *
 * \param pBestMvField    Pointer to the best Mv (Mv pair)
 * \param rBestRefPicList Best reference list
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param rMvStart        Searching center
 * \param nBlkWidth       Width of the block
 * \param nBlkHeight      Height of the block
 * \param bTM             Whether is template matching
 * \param bMvCost         Whether count Mv cost
 */
UInt TComPrediction::xFrucFindBestMvFromList( TComMvField * pBestMvField , RefPicList & rBestRefPicList , TComDataCU * pCU , UInt uiAbsPartIdx , const TComMvField & rMvStart , Int nBlkWidth , Int nBlkHeight , Bool bTM , Bool bMvCost )
{
  UInt uiMinCost = MAX_UINT;

  Int nRefPicListStart = 0;
  Int nRefPicListEnd = 1;
  if( bTM )
  {
    nRefPicListStart = nRefPicListEnd = rBestRefPicList;
  }
  for( Int nRefPicList = nRefPicListStart ; nRefPicList <= nRefPicListEnd ; nRefPicList++ )
  {
    RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
    for( std::list<TComMvField>::iterator pos = m_listMVFieldCand[eCurRefPicList].begin() ; pos != m_listMVFieldCand[eCurRefPicList].end() ; pos++ )
    {
      TComMvField mvPair;

      if( !bTM && eCurRefPicList == REF_PIC_LIST_1 && !pCU->getSlice()->getCheckLDC() )
      {       
        // for normal B picture
        if( !pCU->getMvPair( REF_PIC_LIST_1 , *pos , mvPair ) || xFrucIsInList( mvPair , m_listMVFieldCand[0] ) )
          // no paired MV or the pair has been checked in list0
          continue;
      }

      UInt uiCost = 0;
      if( bMvCost )
      {
        uiCost = xFrucGetMvCost( rMvStart.getMv() , pos->getMv() , MAX_INT , QC_FRUC_MERGE_REFINE_MVWEIGHT );
        if( uiCost > uiMinCost )
          continue;
      }

      if( bTM )
      {
        uiCost = xFrucGetTempMatchCost( pCU , uiAbsPartIdx , nBlkWidth , nBlkHeight , eCurRefPicList , *pos , uiCost );
      }
      else
      {
        uiCost = xFrucGetBilaMatchCost( pCU , uiAbsPartIdx , nBlkWidth , nBlkHeight , eCurRefPicList , *pos , mvPair , uiCost ); 
      }

      if( uiCost < uiMinCost )
      {
        uiMinCost = uiCost;
        pBestMvField[eCurRefPicList] = *pos;
        if( !bTM )
        {
          rBestRefPicList = eCurRefPicList;
          pBestMvField[!eCurRefPicList] = mvPair;
        }
      }
    }
  }

  return( uiMinCost );
}

/**
 * \brief Interface of FRUC. Derive Mv information for a block and its sub-blocks
 *
 * \param pcCU            Pointer to current CU
 * \param uiDepth         CU depth
 * \param uiAbsPartIdx    Address of block within CU
 * \param uiPUIdx         PU index
 * \param nTargetRefIdx   The target reference index for Mv predictor
 * \param eTargetRefPicList The target reference list for Mv predictor
 */
Bool TComPrediction::deriveFRUCMV( TComDataCU * pCU , UInt uiDepth , UInt uiAbsPartIdx , UInt uiPUIdx , Int nTargetRefIdx , RefPicList eTargetRefList )
{
  Bool bAvailable = false;

  if( pCU->getMergeFlag( uiAbsPartIdx ) )
  {
    bAvailable = xFrucFindBlkMv( pCU , uiPUIdx );
    if( bAvailable )
      xFrucRefineSubBlkMv( pCU , uiDepth , uiPUIdx , pCU->getFRUCMgrMode( uiAbsPartIdx ) == QC_FRUC_MERGE_TEMPLATE );
  }
  else
  {
    // for AMVP
    bAvailable = xFrucFindBlkMv4Pred( pCU , uiPUIdx , eTargetRefList , nTargetRefIdx );
  }

  return( bAvailable );
}

/**
 * \brief Check whether top template is available
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 */
Bool TComPrediction::xFrucIsTopTempAvailable( TComDataCU * pCU , UInt uiAbsPartIdx )
{
  // must be used in sub-CU mode, namely getZorderIdxInCU indicates the CU position
  UInt uiAbsPartIdxTop = 0;
  TComDataCU * pPUTop = pCU->getPUAbove( uiAbsPartIdxTop , uiAbsPartIdx + pCU->getZorderIdxInCU() );
  return( pPUTop != NULL && ( pPUTop->getAddr() < pCU->getAddr() || pPUTop->getZorderIdxInCU() < pCU->getZorderIdxInCU() ) );
}

/**
 * \brief Check whether left template is available
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 */
Bool TComPrediction::xFrucIsLeftTempAvailable( TComDataCU * pCU , UInt uiAbsPartIdx )
{
  // must be used in sub-CU mode, namely getZorderIdxInCU indicates the CU position
  UInt uiAbsPartIdxLeft = 0;
  TComDataCU * pPULeft = pCU->getPULeft( uiAbsPartIdxLeft , uiAbsPartIdx + pCU->getZorderIdxInCU() );
  return( pPULeft != NULL && ( pPULeft->getAddr() < pCU->getAddr() || pPULeft->getZorderIdxInCU() < pCU->getZorderIdxInCU() ) );
}

/**
 * \brief Get sub-block size for further refinement
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param nBlkWidth       Width of the block
 * \param nBlkHeight      Height of the block
 */
Int TComPrediction::xFrucGetSubBlkSize( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nBlkWidth , Int nBlkHeight )
{
  Int nRefineBlkSize = max( pcCU->getWidth( uiAbsPartIdx ) >> pcCU->getSlice()->getSPS()->getFRUCSmallBlkRefineDepth() , QC_FRUC_MERGE_REFINE_MINBLKSIZE );
  while( true ) 
  {
    Int nMask = nRefineBlkSize - 1;
    if( nRefineBlkSize > min( nBlkWidth , nBlkHeight ) || ( nBlkWidth & nMask ) || ( nBlkHeight & nMask ) )
      nRefineBlkSize >>= 1;
    else
      break;
  }
  assert( nRefineBlkSize >= QC_FRUC_MERGE_REFINE_MINBLKSIZE );
  return( nRefineBlkSize );
}

/**
 * \brief Get the top and left templates for the current block
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param nCurBlkWidth    Width of the block
 * \param nCurBlkHeight   Height of the block
 */
Bool TComPrediction::xFrucGetCurBlkTemplate( TComDataCU * pCU , UInt uiAbsPartIdx , Int nCurBlkWidth , Int nCurBlkHeight )
{
  m_bFrucTemplateAvailabe[0] = xFrucIsTopTempAvailable( pCU , uiAbsPartIdx );
  m_bFrucTemplateAvailabe[1] = xFrucIsLeftTempAvailable( pCU , uiAbsPartIdx );
  if( !m_bFrucTemplateAvailabe[0] && !m_bFrucTemplateAvailabe[1] )
    return false;

  const Int nMVUnit = QC_MV_STORE_PRECISION_BIT;
  TComPicYuv * pCurPicYuv = pCU->getPic()->getPicYuvRec();
  if( m_bFrucTemplateAvailabe[0] )
  {
    TComYuv * pTemplateTop = &m_cYuvPredFrucTemplate[0];
    TComMv mvTop( 0 , - ( QC_FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) );
    xPredInterLumaBlk( pCU , pCurPicYuv , uiAbsPartIdx , &mvTop , nCurBlkWidth , QC_FRUC_MERGE_TEMPLATE_SIZE , pTemplateTop , false ,
#if BIO
false,
#endif
QC_FRUC_MERGE_TEMPLATE );
  }
  if( m_bFrucTemplateAvailabe[1] )
  {
    TComYuv * pTemplateLeft = &m_cYuvPredFrucTemplate[1];
    TComMv mvLeft( - ( QC_FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) , 0 );
    xPredInterLumaBlk( pCU , pCurPicYuv , uiAbsPartIdx , &mvLeft , QC_FRUC_MERGE_TEMPLATE_SIZE , nCurBlkHeight , pTemplateLeft , false , 
#if BIO
false,
#endif
QC_FRUC_MERGE_TEMPLATE );
  }

  return( true );
}

/**
 * \brief calculate the Mv cost
 *
 * \param rMvStart        Searching center
 * \param rMvCur          Current Mv
 * \param nSearchRange    Search range
 * \param nWeighting      Weighting factor
 */
UInt TComPrediction::xFrucGetMvCost( const TComMv & rMvStart , const TComMv & rMvCur , Int nSearchRange , Int nWeighting )
{
  TComMv mvDist = rMvStart - rMvCur;
  UInt uiCost = MAX_UINT;
  if( mvDist.getAbsHor() <= nSearchRange && mvDist.getAbsVer() <= nSearchRange )
  {
    uiCost = ( mvDist.getAbsHor() + mvDist.getAbsVer() ) * nWeighting;
    uiCost >>= ( QC_MV_STORE_PRECISION_BIT - QC_MV_SIGNAL_PRECISION_BIT );
  }

  return( uiCost );
}

#endif
#if CU_LEVEL_MPI
Void TComPrediction::xMPIredFiltering( Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight , Int idxMPI)
{
  Pel* pDst = rpDst;
  Int x, y;
  switch(idxMPI)
  {
  case 7:
     // 0* 0*
     // 1* 1*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((                    pSrc[-1] +                      3*pDst[0] + 2) >> 2);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((                     pDst[x-1] +                     3*pDst[x] + 2) >> 2); //upper-line
      }
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                                   top                left                    top-left                   current  
        pDst[iDstStride*y] = ((                         pSrc[iSrcStride*y-1]  +                               3*pDst[iDstStride*y]+2) >> 2);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstLeft    = pDst-1;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
         pDstLeft[x]          // left neighbour
        +3*pDst[x] + 2) >> 2); 
        }
        pDst+=iDstStride;
        pDstLeft+=iDstStride;
      }
    }
    break;
  case 6:
    // 0* 1*
    // 0* 3*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((pSrc[-iSrcStride] +                                    3*pDst[0]  +2) >> 2);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((pSrc[x - iSrcStride]  +                                  3*pDst[x]  +2) >> 2); //upper-line
      }     
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                   top                left                top-left        current  
        pDst[iDstStride*y] = 
          ((pDst[iDstStride*(y-1)]   +                                     3*pDst[iDstStride*y]+2) >> 2);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstTop     = pDst-iDstStride;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
        +pDstTop[x]          // top neighbour
        +3*pDst[x] +2) >> 2); 
        }
        pDst+=iDstStride;
        pDstTop+=iDstStride;
      }
    }
    break;
  case 5: 
    // 0* 1*
    // 1* 6*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((pSrc[-iSrcStride] + pSrc[-1] +                      6*pDst[0] + 4) >> 3);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((pSrc[x - iSrcStride]  +pDst[x-1] +                         6*pDst[x] + 4) >> 3); //upper-line
      }
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                                   top                left                    top-left                   current  
        pDst[iDstStride*y] = ((pDst[iDstStride*(y-1)] +  pSrc[iSrcStride*y-1]  +                               6*pDst[iDstStride*y]+ 4) >> 3);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstLeft    = pDst-1;
      Pel* pDstTop     = pDst-iDstStride;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
         pDstLeft[x]          // left neighbour
        +pDstTop[x]          // top neighbour
        +6*pDst[x] + 4) >> 3); 
        }
        pDst+=iDstStride;
        pDstLeft+=iDstStride;
        pDstTop+=iDstStride;
      }
    }
    break;
  case 3:
    // 0* 0*
    // 1* 1*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((                    pSrc[-1] +                      pDst[0] + 1) >> 1);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((                     pDst[x-1] +                     pDst[x] + 1) >> 1); //upper-line
      }
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                                   top                left                    top-left                   current  
        pDst[iDstStride*y] = ((                         pSrc[iSrcStride*y-1]  +                               pDst[iDstStride*y]+1) >> 1);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstLeft    = pDst-1;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
         pDstLeft[x]          // left neighbour
        +pDst[x] + 1) >> 1); 
        }
        pDst+=iDstStride;
        pDstLeft+=iDstStride;
      }
    }
    break;
  case 2:
    // 0* 1*
    // 0* 1*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((pSrc[-iSrcStride] +                                    pDst[0]  +1) >> 1);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((pSrc[x - iSrcStride]  +                                   pDst[x]  +1) >> 1); //upper-line
      }     
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                   top                left                top-left        current  
        pDst[iDstStride*y] = 
          ((pDst[iDstStride*(y-1)]   +                                      pDst[iDstStride*y]+1) >> 1);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstTop     = pDst-iDstStride;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
        +pDstTop[x]          // top neighbour
        +pDst[x] +1) >> 1); 
        }
        pDst+=iDstStride;
        pDstTop+=iDstStride;
      }
    }
    break;
  case 1: 
    // 0* 1*
    // 1* 2*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((pSrc[-iSrcStride] + pSrc[-1] +                      2*pDst[0] + 2) >> 2);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((pSrc[x - iSrcStride]  +pDst[x-1] +                         2*pDst[x] + 2) >> 2); //upper-line
      }
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                                   top                left                    top-left                   current  
        pDst[iDstStride*y] = ((pDst[iDstStride*(y-1)] +  pSrc[iSrcStride*y-1]  +                               2*pDst[iDstStride*y]+ 2) >> 2);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstLeft    = pDst-1;
      Pel* pDstTop     = pDst-iDstStride;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
         pDstLeft[x]          // left neighbour
        +pDstTop[x]          // top neighbour
        +2*pDst[x] + 2) >> 2); 
        }
        pDst+=iDstStride;
        pDstLeft+=iDstStride;
        pDstTop+=iDstStride;
      }
    }
    break;
  case 0:
  case 4:
    {

    }
    break;
  }
  return;
}
#endif
//! \}
