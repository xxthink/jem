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

/**
 * \file
 * \brief Implementation of TComInterpolationFilter class
 */

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "TComRom.h"
#include "TComInterpolationFilter.h"
#include <assert.h>

#if QC_SIMD_OPT
#include <emmintrin.h>  
#endif

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

#if QC_MV_STORE_PRECISION_BIT == 3
// from SHVC upsampling filter
const Short TComInterpolationFilter::m_lumaFilter[8][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0,  0,  0 },
//  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
//  { -1, 3,  -8, 60, 13,  -4,  1,  0 },
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
//  { -1, 4, -11, 52, 26,  -8,  3, -1 }, 
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
//  { -1, 4, -11, 45, 34, -10,  4, -1 },
  { -1, 4, -11, 40, 40, -11,  4, -1 }, 
//  { -1, 4, -10, 34, 45, -11,  4, -1 },
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
//  { -1, 3,  -8, 26, 52, -11,  4, -1 }, 
  {  0, 1,  -5, 17, 58, -10,  4, -1 },
//  {  0, 1,  -4, 13, 60,  -8,  3, -1 },
  {  0, 1,  -3,  8, 62,  -5,  2, -1 },
//  {  0, 1,  -2,  4, 63,  -3,  1,  0 }
};

const Short TComInterpolationFilter::m_chromaFilter[16][NTAPS_CHROMA] =
{
  {  0, 64,  0,  0 },
  { -2, 62,  4,  0 },
  { -2, 58, 10, -2 },
  { -4, 56, 14, -2 },
  { -4, 54, 16, -2 }, 
  { -6, 52, 20, -2 }, 
  { -6, 46, 28, -4 }, 
  { -4, 42, 30, -4 },
  { -4, 36, 36, -4 }, 
  { -4, 30, 42, -4 }, 
  { -4, 28, 46, -6 },
  { -2, 20, 52, -6 }, 
  { -2, 16, 54, -4 },
  { -2, 14, 56, -4 },
  { -2, 10, 58, -2 }, 
  {  0,  4, 62, -2 }  
};
#else
const Short TComInterpolationFilter::m_lumaFilter[4][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0, 0,  0 },
  { -1, 4, -10, 58, 17,  -5, 1,  0 },
  { -1, 4, -11, 40, 40, -11, 4, -1 },
  {  0, 1,  -5, 17, 58, -10, 4, -1 }
};

const Short TComInterpolationFilter::m_chromaFilter[8][NTAPS_CHROMA] =
{
  {  0, 64,  0,  0 },
  { -2, 58, 10, -2 },
  { -4, 54, 16, -2 },
  { -6, 46, 28, -4 },
  { -4, 36, 36, -4 },
  { -4, 28, 46, -6 },
  { -2, 16, 54, -4 },
  { -2, 10, 58, -2 }
};
#endif

#if QC_FRUC_MERGE
#if QC_MV_STORE_PRECISION_BIT == 3
const Short TComInterpolationFilter::m_lumaFilterBilinear[8][NTAPS_LUMA_FRUC] =
{
  { 64,  0, },
  { 56,  8, },
  { 48, 16, },
  { 40, 24, },
  { 32, 32, },
  { 24, 40, },
  { 16, 48, },
  {  8, 56, },
};
#else
const Short TComInterpolationFilter::m_lumaFilterBilinear[4][NTAPS_LUMA_FRUC] =
{
  { 64,  0, },
  { 48, 16, },
  { 32, 32, },
  { 16, 48, },
};
#endif

#endif

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

/**
 * \brief Apply unit FIR filter to a block of samples
 *
 * \param bitDepth   bitDepth of samples
 * \param src        Pointer to source samples
 * \param srcStride  Stride of source samples
 * \param dst        Pointer to destination samples
 * \param dstStride  Stride of destination samples
 * \param width      Width of block
 * \param height     Height of block
 * \param isFirst    Flag indicating whether it is the first filtering operation
 * \param isLast     Flag indicating whether it is the last filtering operation
 */
Void TComInterpolationFilter::filterCopy(Int bitDepth, const Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast)
{
  Int row, col;
  
  if ( isFirst == isLast )
  {
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        dst[col] = src[col];
      }
      
      src += srcStride;
      dst += dstStride;
    }              
  }
  else if ( isFirst )
  {
    Int shift = IF_INTERNAL_PREC - bitDepth;
    
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        Short val = src[col] << shift;
        dst[col] = val - (Short)IF_INTERNAL_OFFS;
      }
      
      src += srcStride;
      dst += dstStride;
    }          
  }
  else
  {
    Int shift = IF_INTERNAL_PREC - bitDepth;
    Short offset = IF_INTERNAL_OFFS;
    offset += shift?(1 << (shift - 1)):0;
    Short maxVal = (1 << bitDepth) - 1;
    Short minVal = 0;
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        Short val = src[ col ];
        val = ( val + offset ) >> shift;
        if (val < minVal) val = minVal;
        if (val > maxVal) val = maxVal;
        dst[col] = val;
      }
      
      src += srcStride;
      dst += dstStride;
    }              
  }
}

#if QC_SIMD_OPT
inline __m128i simdInterpolateLuma4( Short const *src , Int srcStride , __m128i *mmCoeff , const __m128i & mmOffset , Int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( Int n = 0 ; n < 8 ; n++ )
  {
    __m128i mmPix = _mm_loadl_epi64( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix , mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix , mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi , _mm_unpackhi_epi16( lo , hi ) );
    sumLo = _mm_add_epi32( sumLo , _mm_unpacklo_epi16( lo , hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi , mmOffset ) , shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo , mmOffset ) , shift );
  return( _mm_packs_epi32( sumLo , sumHi ) );
}

inline __m128i simdInterpolateChroma4( Short const *src , Int srcStride , __m128i *mmCoeff , const __m128i & mmOffset , Int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( Int n = 0 ; n < 4 ; n++ )
  {
    __m128i mmPix = _mm_loadl_epi64( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix , mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix , mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi , _mm_unpackhi_epi16( lo , hi ) );
    sumLo = _mm_add_epi32( sumLo , _mm_unpacklo_epi16( lo , hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi , mmOffset ) , shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo , mmOffset ) , shift );
  return( _mm_packs_epi32( sumLo , sumHi ) );
}

inline __m128i simdInterpolateLuma8( Short const *src , Int srcStride , __m128i *mmCoeff , const __m128i & mmOffset , Int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( Int n = 0 ; n < 8 ; n++ )
  {
    __m128i mmPix = _mm_loadu_si128( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix , mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix , mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi , _mm_unpackhi_epi16( lo , hi ) );
    sumLo = _mm_add_epi32( sumLo , _mm_unpacklo_epi16( lo , hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi , mmOffset ) , shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo , mmOffset ) , shift );
  return( _mm_packs_epi32( sumLo , sumHi ) );
}

#if QC_FRUC_MERGE
inline __m128i simdInterpolateLuma2P8( Short const *src , Int srcStride , __m128i *mmCoeff , const __m128i & mmOffset , Int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( Int n = 0 ; n < 2 ; n++ )
  {
    __m128i mmPix = _mm_loadu_si128( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix , mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix , mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi , _mm_unpackhi_epi16( lo , hi ) );
    sumLo = _mm_add_epi32( sumLo , _mm_unpacklo_epi16( lo , hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi , mmOffset ) , shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo , mmOffset ) , shift );
  return( _mm_packs_epi32( sumLo , sumHi ) );
}

inline __m128i simdInterpolateLuma2P4( Short const *src , Int srcStride , __m128i *mmCoeff , const __m128i & mmOffset , Int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( Int n = 0 ; n < 2 ; n++ )
  {
    __m128i mmPix = _mm_loadl_epi64( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix , mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix , mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi , _mm_unpackhi_epi16( lo , hi ) );
    sumLo = _mm_add_epi32( sumLo , _mm_unpacklo_epi16( lo , hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi , mmOffset ) , shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo , mmOffset ) , shift );
  return( _mm_packs_epi32( sumLo , sumHi ) );
}
#endif

inline __m128i simdClip3( __m128i mmMin , __m128i mmMax , __m128i mmPix )
{
  __m128i mmMask = _mm_cmpgt_epi16( mmPix , mmMin );
  mmPix = _mm_or_si128( _mm_and_si128( mmMask , mmPix ) , _mm_andnot_si128( mmMask , mmMin ) );
  mmMask = _mm_cmplt_epi16( mmPix , mmMax );
  mmPix = _mm_or_si128( _mm_and_si128( mmMask , mmPix ) , _mm_andnot_si128( mmMask , mmMax ) );
  return( mmPix );
}
#endif

/**
 * \brief Apply FIR filter to a block of samples
 *
 * \tparam N          Number of taps
 * \tparam isVertical Flag indicating filtering along vertical direction
 * \tparam isFirst    Flag indicating whether it is the first filtering operation
 * \tparam isLast     Flag indicating whether it is the last filtering operation
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  coeff      Pointer to filter taps
 */
template<Int N, Bool isVertical, Bool isFirst, Bool isLast>
Void TComInterpolationFilter::filter(Int bitDepth, Short const *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Short const *coeff)
{
  Int row, col;
  
  Short c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if ( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if ( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if ( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }
  
  Int cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  Int offset;
  Short maxVal;
  Int headRoom = IF_INTERNAL_PREC - bitDepth;
  Int shift = IF_FILTER_PREC;
  if ( isLast )
  {
    shift += (isFirst) ? 0 : headRoom;
    offset = 1 << (shift - 1);
    offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
    maxVal = (1 << bitDepth) - 1;
  }
  else
  {
    shift -= (isFirst) ? headRoom : 0;
    offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
    maxVal = 0;
  }
  
#if QC_SIMD_OPT
  if( N == 8 && !( width & 0x07 ) )
  {
    Short minVal = 0;
    __m128i mmOffset = _mm_set1_epi32( offset );
    __m128i mmCoeff[8];
    __m128i mmMin = _mm_set1_epi16( minVal );
    __m128i mmMax = _mm_set1_epi16( maxVal );
    for( Int n = 0 ; n < 8 ; n++ )
      mmCoeff[n] = _mm_set1_epi16( c[n] );
    for( row = 0 ; row < height ; row++ )
    {
      for( col = 0 ; col < width ; col += 8 )
      {
        __m128i mmFiltered = simdInterpolateLuma8( src + col , cStride , mmCoeff , mmOffset , shift );
        if( isLast )
        {
          mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
        }
        _mm_storeu_si128( ( __m128i * )( dst + col ) , mmFiltered );
      }
      src += srcStride;
      dst += dstStride;
    }
    return;
  }
  else if( N == 8 && !( width & 0x03 ) )
  {
    Short minVal = 0;
    __m128i mmOffset = _mm_set1_epi32( offset );
    __m128i mmCoeff[8];
    __m128i mmMin = _mm_set1_epi16( minVal );
    __m128i mmMax = _mm_set1_epi16( maxVal );
    for( Int n = 0 ; n < 8 ; n++ )
      mmCoeff[n] = _mm_set1_epi16( c[n] );
    for( row = 0 ; row < height ; row++ )
    {
      for( col = 0 ; col < width ; col += 4 )
      {
        __m128i mmFiltered = simdInterpolateLuma4( src + col , cStride , mmCoeff , mmOffset , shift );
        if( isLast )
        {
          mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
        }
        _mm_storel_epi64( ( __m128i * )( dst + col ) , mmFiltered );
      }
      src += srcStride;
      dst += dstStride;
    }
    return;
  }
  else if( N == 4 && !( width & 0x03 ) )
  {
    Short minVal = 0;
    __m128i mmOffset = _mm_set1_epi32( offset );
    __m128i mmCoeff[8];
    __m128i mmMin = _mm_set1_epi16( minVal );
    __m128i mmMax = _mm_set1_epi16( maxVal );
    for( Int n = 0 ; n < 4 ; n++ )
      mmCoeff[n] = _mm_set1_epi16( c[n] );
    for( row = 0 ; row < height ; row++ )
    {
      for( col = 0 ; col < width ; col += 4 )
      {
        __m128i mmFiltered = simdInterpolateChroma4( src + col , cStride , mmCoeff , mmOffset , shift );
        if( isLast )
        {
          mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
        }
        _mm_storel_epi64( ( __m128i * )( dst + col ) , mmFiltered );
      }
      src += srcStride;
      dst += dstStride;
    }
    return;
  }
#if QC_FRUC_MERGE
  else if( N == 2 && !( width & 0x07 ) )
  {
    Short minVal = 0;
    __m128i mmOffset = _mm_set1_epi32( offset );
    __m128i mmCoeff[2];
    __m128i mmMin = _mm_set1_epi16( minVal );
    __m128i mmMax = _mm_set1_epi16( maxVal );
    for( Int n = 0 ; n < 2 ; n++ )
      mmCoeff[n] = _mm_set1_epi16( c[n] );
    for( row = 0 ; row < height ; row++ )
    {
      for( col = 0 ; col < width ; col += 8 )
      {
        __m128i mmFiltered = simdInterpolateLuma2P8( src + col , cStride , mmCoeff , mmOffset , shift );
        if( isLast )
        {
          mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
        }
        _mm_storeu_si128( ( __m128i * )( dst + col ) , mmFiltered );
      }
      src += srcStride;
      dst += dstStride;
    }
    return;
  }
  else if( N == 2 && !( width & 0x03 ) )
  {
    Short minVal = 0;
    __m128i mmOffset = _mm_set1_epi32( offset );
    __m128i mmCoeff[8];
    __m128i mmMin = _mm_set1_epi16( minVal );
    __m128i mmMax = _mm_set1_epi16( maxVal );
    for( Int n = 0 ; n < 2 ; n++ )
      mmCoeff[n] = _mm_set1_epi16( c[n] );
    for( row = 0 ; row < height ; row++ )
    {
      for( col = 0 ; col < width ; col += 4 )
      {
        __m128i mmFiltered = simdInterpolateLuma2P4( src + col , cStride , mmCoeff , mmOffset , shift );
        if( isLast )
        {
          mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
        }
        _mm_storel_epi64( ( __m128i * )( dst + col ) , mmFiltered );
      }
      src += srcStride;
      dst += dstStride;
    }
    return;
  }
#endif
#endif

  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col++)
    {
      Int sum;
      
      sum  = src[ col + 0 * cStride] * c[0];
      sum += src[ col + 1 * cStride] * c[1];
      if ( N >= 4 )
      {
        sum += src[ col + 2 * cStride] * c[2];
        sum += src[ col + 3 * cStride] * c[3];
      }
      if ( N >= 6 )
      {
        sum += src[ col + 4 * cStride] * c[4];
        sum += src[ col + 5 * cStride] * c[5];
      }
      if ( N == 8 )
      {
        sum += src[ col + 6 * cStride] * c[6];
        sum += src[ col + 7 * cStride] * c[7];        
      }
      
      Short val = ( sum + offset ) >> shift;
      if ( isLast )
      {
        val = ( val < 0 ) ? 0 : val;
        val = ( val > maxVal ) ? maxVal : val;        
      }
      dst[col] = val;
    }
    
    src += srcStride;
    dst += dstStride;
  }    
}

/**
 * \brief Filter a block of samples (horizontal)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<Int N>
Void TComInterpolationFilter::filterHor(Int bitDepth, Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Bool isLast, Short const *coeff)
{
  if ( isLast )
  {
    filter<N, false, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
  }
  else
  {
    filter<N, false, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
  }
}

/**
 * \brief Filter a block of samples (vertical)
 *
 * \tparam N          Number of taps
 * \param  bitDpeth   Sample bit depth
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<Int N>
Void TComInterpolationFilter::filterVer(Int bitDepth, Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast, Short const *coeff)
{
  if ( isFirst && isLast )
  {
    filter<N, true, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
  }
  else if ( isFirst && !isLast )
  {
    filter<N, true, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
  }
  else if ( !isFirst && isLast )
  {
    filter<N, true, false, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
  }
  else
  {
    filter<N, true, false, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
  }      
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 * \brief Filter a block of luma samples (horizontal)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void TComInterpolationFilter::filterHorLuma(Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int frac, Bool isLast 
#if QC_FRUC_MERGE
  , Int nFilterIdx
#endif
  )
{
#if QC_MV_STORE_PRECISION_BIT
  assert(frac >= 0 && frac < ( 1 << QC_MV_STORE_PRECISION_BIT ) );
#else
  assert(frac >= 0 && frac < 4);
#endif
  
  if ( frac == 0 )
  {
    filterCopy(g_bitDepthY, src, srcStride, dst, dstStride, width, height, true, isLast );
  }
  else
  {
#if QC_FRUC_MERGE
    if( nFilterIdx == 1 )
      filterHor<NTAPS_LUMA_FRUC>(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilterBilinear[frac]);
    else
#endif
    filterHor<NTAPS_LUMA>(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter[frac]);
  }
}

/**
 * \brief Filter a block of luma samples (vertical)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void TComInterpolationFilter::filterVerLuma(Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast 
#if QC_FRUC_MERGE
  , Int nFilterIdx
#endif
  )
{
#if QC_MV_STORE_PRECISION_BIT
  assert(frac >= 0 && frac < (1 << QC_MV_STORE_PRECISION_BIT ) );
#else
  assert(frac >= 0 && frac < 4);
#endif
  
  if ( frac == 0 )
  {
    filterCopy(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isFirst, isLast );
  }
  else
  {
#if QC_FRUC_MERGE
    if( nFilterIdx == 1 )
      filterVer<NTAPS_LUMA_FRUC>(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilterBilinear[frac]);
    else
#endif
    filterVer<NTAPS_LUMA>(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter[frac]);
  }
}

/**
 * \brief Filter a block of chroma samples (horizontal)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void TComInterpolationFilter::filterHorChroma(Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int frac, Bool isLast )
{
#if QC_MV_STORE_PRECISION_BIT 
  assert(frac >= 0 && frac < ( 1 << ( QC_MV_STORE_PRECISION_BIT + 1 ) ) );
#else
  assert(frac >= 0 && frac < 8);
#endif
  
  if ( frac == 0 )
  {
    filterCopy(g_bitDepthC, src, srcStride, dst, dstStride, width, height, true, isLast );
  }
  else
  {
    filterHor<NTAPS_CHROMA>(g_bitDepthC, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilter[frac]);
  }
}

/**
 * \brief Filter a block of chroma samples (vertical)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void TComInterpolationFilter::filterVerChroma(Pel *src, Int srcStride, Short *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast )
{
#if QC_MV_STORE_PRECISION_BIT
  assert(frac >= 0 && frac < ( 1 << ( QC_MV_STORE_PRECISION_BIT + 1 ) ) );
#else
  assert(frac >= 0 && frac < 8);
#endif
  
  if ( frac == 0 )
  {
    filterCopy(g_bitDepthC, src, srcStride, dst, dstStride, width, height, isFirst, isLast );
  }
  else
  {
    filterVer<NTAPS_CHROMA>(g_bitDepthC, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilter[frac]);
  }
}

//! \}
