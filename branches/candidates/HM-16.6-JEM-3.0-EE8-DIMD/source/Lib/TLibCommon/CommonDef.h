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

/** \file     CommonDef.h
    \brief    Defines version information, constants and small in-line functions
*/

#ifndef __COMMONDEF__
#define __COMMONDEF__

#include <algorithm>
#include <iostream>
#include <assert.h>
#include <limits>

#if _MSC_VER > 1000
// disable "signed and unsigned mismatch"
#pragma warning( disable : 4018 )
// disable Bool coercion "performance warning"
#pragma warning( disable : 4800 )
#endif // _MSC_VER > 1000
#include "TypeDef.h"

#ifdef _MSC_VER
#if _MSC_VER <= 1500
inline Int64 abs (Int64 x) { return _abs64(x); };
#endif
#endif

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Version information
// ====================================================================================================================

#define NV_VERSION        "HM-16.6-JEM-3.0rc1"                 ///< Current software version

// ====================================================================================================================
// Platform information
// ====================================================================================================================

#ifdef __GNUC__
#define NVM_COMPILEDBY  "[GCC %d.%d.%d]", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__
#ifdef __IA64__
#define NVM_ONARCH    "[on 64-bit] "
#else
#define NVM_ONARCH    "[on 32-bit] "
#endif
#endif

#ifdef __INTEL_COMPILER
#define NVM_COMPILEDBY  "[ICC %d]", __INTEL_COMPILER
#elif  _MSC_VER
#define NVM_COMPILEDBY  "[VS %d]", _MSC_VER
#endif

#ifndef NVM_COMPILEDBY
#define NVM_COMPILEDBY "[Unk-CXX]"
#endif

#ifdef _WIN32
#define NVM_ONOS        "[Windows]"
#elif  __linux
#define NVM_ONOS        "[Linux]"
#elif  __CYGWIN__
#define NVM_ONOS        "[Cygwin]"
#elif __APPLE__
#define NVM_ONOS        "[Mac OS X]"
#else
#define NVM_ONOS "[Unk-OS]"
#endif

#define NVM_BITS          "[%d bit] ", (sizeof(Void*) == 8 ? 64 : 32) ///< used for checking 64-bit O/S

#ifndef NULL
#define NULL              0
#endif

// ====================================================================================================================
// Common constants
// ====================================================================================================================

static const UInt   MAX_UINT =                            0xFFFFFFFFU; ///< max. value of unsigned 32-bit integer
static const Int    MAX_INT =                              2147483647; ///< max. value of signed 32-bit integer
static const Double MAX_DOUBLE =                             1.7e+308; ///< max. value of Double-type value

// ====================================================================================================================
// Coding tool configuration
// ====================================================================================================================
// Most of these should not be changed - they resolve the meaning of otherwise magic numbers.

static const Int MAX_GOP =                                         64; ///< max. value of hierarchical GOP size
static const Int MAX_NUM_REF_PICS =                                16; ///< max. number of pictures used for reference
//intra KLT
#if VCEG_AZ08_INTRA_KLT
static const Int TMPRED0_TMPREDKLT1_ORI2 =                          1; ///< (default 1) 0: Template matching prediction; 1: TM prediction + KLT; 2: Original method
static const Int TMPRED_CANDI_NUM =                                 8; ///< Candidate number for intra prediction (should <= 32)
static const Int SEARCHRANGEINTRA =                                64; ///< Intra search range (-SEARCHRANGE,+SEARCHRANGE)
#endif
//inter KLT
#if VCEG_AZ08_INTER_KLT
static const Int SEARCHRANGE =                                     32; ///< (default 32) Search range for inter coding (-SEARCHRANGE,+SEARCHRANGE)
static const Int SEARCH_SIZE =                                     ((SEARCHRANGE << 1) + 1)*((SEARCHRANGE << 1) + 1);
#endif

#if VCEG_AZ08_KLT_COMMON
static const Int USE_MORE_BLOCKSIZE_DEPTH_MIN =                     1; ///< (default 1) To indicate minimum block size for KLT. 1~4 means 4x4, 8x8, 16x16, 32x32 respectively.
static const Int USE_MORE_BLOCKSIZE_DEPTH_MAX =                     4; ///< (default 4) To indicate maximum block size for KLT. 1~4 means 4x4, 8x8, 16x16, 32x32 respectively.
static const Int MAX_CANDI_NUM =                                  100; ///< Max allowable candidate number. The candidate number for different size blocks can be set respectively.
#if VCEG_AZ08_FAST_DERIVE_KLT
static const Int FAST_KLT_CANDINUM =                    MAX_CANDI_NUM; ///If MAX_CANDI_NUM > blkSize, fast algorithm will be performed.
#endif
static const Int KLTBASIS_SHIFTBIT =                               10; ///< KLT scale factor is BLOCK_SIZE*(1<<KLTBASIS_SHIFTBIT); (log2(width)+KLTBASIS_SHIFTBIT <= 15); If 6, then the first base vector is {64,...,64}. We use 10.
static const Int INIT_THRESHOULD_SHIFTBITS =                        2;  ///< (default 2) Early skip threshold for checking distance.
static const Int MAX_NUM_REF_IDS =                                 ((MAX_NUM_REF_PICS << 1) + (MAX_CANDI_NUM << 4)); 
static const Double IGNORE_THRESHOULD_OF_LARGEST =               1e-6;
#if VCEG_AZ08_FORCE_USE_GIVENNUM_BASIS
static const Int FORCE_BASIS_NUM =                                 32; /// Forced number of basis utilized (for speeding up).
#endif
#endif

static const Int MAX_NUM_REF =                                     16; ///< max. number of entries in picture reference list
static const Int MAX_QP =                                          51;
static const Int NOT_VALID =                                       -1;

static const Int AMVP_MAX_NUM_CANDS =                               2; ///< AMVP: advanced motion vector prediction - max number of final candidates
static const Int AMVP_MAX_NUM_CANDS_MEM =                           3; ///< AMVP: advanced motion vector prediction - max number of candidates
static const Int AMVP_DECIMATION_FACTOR =                           4;
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
static const Int MRG_MAX_NUM_CANDS =                                7; ///< MERGE
#else
static const Int MRG_MAX_NUM_CANDS =                                5; ///< MERGE
#endif

static const Int MAX_TLAYER =                                       7; ///< Explicit temporal layer QP offset - max number of temporal layer

static const Int ADAPT_SR_SCALE =                                   1; ///< division factor for adaptive search range

static const Int MAX_NUM_PICS_IN_SOP =                           1024;

static const Int MAX_NESTING_NUM_OPS =                           1024;
static const Int MAX_NESTING_NUM_LAYER =                           64;

static const Int MAX_VPS_NUM_HRD_PARAMETERS =                       1;
static const Int MAX_VPS_OP_SETS_PLUS1 =                         1024;
static const Int MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1 =         1;

static const Int MAXIMUM_INTRA_FILTERED_WIDTH =                    16;
static const Int MAXIMUM_INTRA_FILTERED_HEIGHT =                   16;

static const Int MAX_CPB_CNT =                                     32; ///< Upper bound of (cpb_cnt_minus1 + 1)
static const Int MAX_NUM_LAYER_IDS =                               64;

static const Int COEF_REMAIN_BIN_REDUCTION =                        3; ///< indicates the level at which the VLC transitions from Golomb-Rice to TU+EG(k)

static const Int CU_DQP_TU_CMAX =                                   5; ///< max number bins for truncated unary
static const Int CU_DQP_EG_k =                                      0; ///< expgolomb order

static const Int SBH_THRESHOLD =                                    4; ///< value of the fixed SBH controlling threshold

static const Int C1FLAG_NUMBER =                                    8; // maximum number of largerThan1 flag coded in one chunk:  16 in HM5
static const Int C2FLAG_NUMBER =                                    1; // maximum number of largerThan2 flag coded in one chunk:  16 in HM5

static const Int MAX_NUM_VPS =                                     16;
static const Int MAX_NUM_SPS =                                     16;
static const Int MAX_NUM_PPS =                                     64;

#if QTBT_NSST
static const Int NSST_SIG_NZ_LUMA =                              1;
static const Int NSST_SIG_NZ_CHROMA =                            1;
#endif

#if COM16_C806_T64
#if JVET_C0024_QTBT
static const Int MLS_GRP_NUM =                                    1024; ///< Max number of coefficient groups, max(16, 256)
#else
static const Int MLS_GRP_NUM =                                    256; ///< Max number of coefficient groups, max(16, 256)
#endif
#else
static const Int MLS_GRP_NUM =                                     64; ///< Max number of coefficient groups, max(16, 64)
#endif
static const Int MLS_CG_LOG2_WIDTH =                                2;
static const Int MLS_CG_LOG2_HEIGHT =                               2;
static const Int MLS_CG_SIZE =                                      4; ///< Coefficient group size of 4x4; = MLS_CG_LOG2_WIDTH + MLS_CG_LOG2_HEIGHT

#if COM16_C983_RSAF
static const Int SCAN_SET_SIZE =                                   (1 << MLS_CG_SIZE); ///< Coefficient group size of 4x4; = MLS_CG_LOG2_WIDTH + MLS_CG_LOG2_HEIGHT
#endif

#if JVET_C0046_ZO_ASSERT
static const Int TH_LOG2TBAREASIZE =                               10; ///< Threshold for zeroing out
#endif

#if ADAPTIVE_QP_SELECTION
static const Int ARL_C_PRECISION =                                  7; ///< G382: 7-bit arithmetic precision
static const Int LEVEL_RANGE =                                     30; ///< G382: max coefficient level in statistics collection
#endif

static const Int RVM_VCEGAM10_M =                                   4;

#if VCEG_AZ07_INTRA_65ANG_MODES
static const Int FAST_UDI_MAX_RDMODE_NUM =                         67; ///< maximum number of RD comparison in fast-UDI estimation loop
#else
static const Int FAST_UDI_MAX_RDMODE_NUM =                         35; ///< maximum number of RD comparison in fast-UDI estimation loop
#endif

#if VCEG_AZ07_INTRA_65ANG_MODES
static const Int NUM_INTRA_MODE =                                  68;
static const Int NUM_DIR =                (((NUM_INTRA_MODE-4)>>2)+1);
static const Int PLANAR_IDX =                                       0;
static const Int DC_IDX =                                           1; ///< index for intra DC mode
static const Int VER_IDX =                          (3*(NUM_DIR-1)+2); ///< index for intra VERTICAL   mode
static const Int HOR_IDX =                          (1*(NUM_DIR-1)+2); ///< index for intra HORIZONTAL mode
static const Int DIA_IDX =                          (2*(NUM_DIR-1)+2); ///< index for intra Diagonal mode
static const Int VDIA_IDX =                         (4*(NUM_DIR-1)+2); ///< index for intra DC mode
#if COM16_C806_LMCHROMA
static const Int NUM_CHROMA_MODE =                                  6; ///< total number of chroma modes
static const Int LM_CHROMA_IDX =                 (NUM_INTRA_MODE - 1); ///< chroma mode index for derived from LM mode
#else
static const Int NUM_CHROMA_MODE =                                  5; ///< total number of chroma modes
#endif
static const Int DM_CHROMA_IDX =                       NUM_INTRA_MODE; ///< chroma mode index for derived from luma intra mode
#else
static const Int NUM_INTRA_MODE =                                  36;
static const Int PLANAR_IDX =                                       0;
static const Int VER_IDX =                                         26; ///< index for intra VERTICAL   mode
static const Int HOR_IDX =                                         10; ///< index for intra HORIZONTAL mode
static const Int DC_IDX =                                           1; ///< index for intra DC mode
#if COM16_C1044_NSST
static const Int DIA_IDX =                                         18; ///< index for intra Diagonal mode
#endif
#if COM16_C806_LMCHROMA
static const Int NUM_CHROMA_MODE =                                  6; ///< total number of chroma modes
static const Int LM_CHROMA_IDX =                                   35; ///< chroma mode index for derived from LM mode
#else
static const Int NUM_CHROMA_MODE =                                  5; ///< total number of chroma modes
#endif
static const Int DM_CHROMA_IDX =                                   36; ///< chroma mode index for derived from luma intra mode
#endif

#if DIMD_NUM_INTRA_DIR_INC
static const Int EXT_HOR_IDX =                                     34;
static const Int EXT_DIA_IDX =                                     66;
static const Int EXT_VER_IDX =                                     98;
static const Int EXT_VDIA_IDX =                                   130;
#if VCEG_AZ07_INTRA_65ANG_MODES
#define MAP131TO67( mode )                 (mode<2?mode:((mode>>1)+1))
#define MAP67TO131( mode )                 (mode<2?mode:((mode<<1)-2))
#else
#define MAP131TO35( mode )       (mode<2?mode:(MAP131TO67(mode)>>1)+1)
#define MAP35TO131( mode )       MAP67TO131( (mode<2?mode:((mode<<1)-2)) )
#endif
#endif

#if COM16_C806_EMT
static const UChar INTER_MODE_IDX =                               255; ///< index for inter modes
#if JVET_C0024_QTBT
static const UInt  EMT_INTRA_MAX_CU =                              64; ///< Max Intra CU size applying EMT, supported values: 8, 16, 32, 64, 128
static const UInt  EMT_INTER_MAX_CU =                              64; ///< Max Inter CU size applying EMT, supported values: 8, 16, 32, 64, 128
#else
static const UInt  EMT_INTRA_MAX_CU =                              32; ///< Max Intra CU size applying EMT, supported values: 8, 16, 32
static const UInt  EMT_INTER_MAX_CU =                              32; ///< Max Inter CU size applying EMT, supported values: 8, 16, 32
#endif
#endif

#if VCEG_AZ07_INTRA_65ANG_MODES
static const Int MDCS_ANGLE_LIMIT =                                 9; ///< 0 = Horizontal/vertical only, 1 = Horizontal/vertical +/- 1, 2 = Horizontal/vertical +/- 2 etc...
#else
static const Int MDCS_ANGLE_LIMIT =                                 4; ///< 0 = Horizontal/vertical only, 1 = Horizontal/vertical +/- 1, 2 = Horizontal/vertical +/- 2 etc...
#endif
#if DIMD_NUM_INTRA_DIR_INC
static const Int MDCS_EXT_ANGLE_LIMIT =                            19;
#endif
static const Int MDCS_MAXIMUM_WIDTH =                               8; ///< (measured in pixels) TUs with width greater than this can only use diagonal scan
static const Int MDCS_MAXIMUM_HEIGHT =                              8; ///< (measured in pixels) TUs with height greater than this can only use diagonal scan


static const Int LOG2_MAX_NUM_COLUMNS_MINUS1 =                      7;
static const Int LOG2_MAX_NUM_ROWS_MINUS1 =                         7;

static const Int CABAC_INIT_PRESENT_FLAG =                          1;

static const Int LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS =   4;
static const Int CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS = 8;

static const Int MAX_NUM_LONG_TERM_REF_PICS =                      33;
static const Int NUM_LONG_TERM_REF_PIC_SPS =                        0;


static const Int MAX_QP_OFFSET_LIST_SIZE =                          6; ///< Maximum size of QP offset list is 6 entries

// Cost mode support
static const Int LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP =      0; ///< QP to use for lossless coding.
static const Int LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME =4; ///< QP' to use for mixed_lossy_lossless coding.
#if COM16_C806_LMCHROMA
static const Int CR_FROM_CB_REG_COST_SHIFT =                        9;     
#endif

static const Int RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS =     4;
static const Int RExt__GOLOMB_RICE_INCREMENT_DIVISOR =              4;

static const Int RExt__PREDICTION_WEIGHTING_ANALYSIS_DC_PRECISION = 0; ///< Additional fixed bit precision used during encoder-side weighting prediction analysis. Currently only used when high_precision_prediction_weighting_flag is set, for backwards compatibility reasons.

static const Int MAX_TIMECODE_SEI_SETS =                            3; ///< Maximum number of time sets

#if COM16_C806_LARGE_CTU
#if JVET_C0024_QTBT && !JVET_C0024_CTU_256
static const Int MAX_CU_DEPTH =                                     7; ///< log2(CTUSize)
static const Int MAX_CU_SIZE =                                    128; ///< = 1<<(MAX_CU_DEPTH)
#else
static const Int MAX_CU_DEPTH =                                     8; ///< log2(CTUSize)
static const Int MAX_CU_SIZE =                                    256; ///< = 1<<(MAX_CU_DEPTH)
#endif
#else
static const Int MAX_CU_DEPTH =                                     6; ///< log2(CTUSize)
static const Int MAX_CU_SIZE =                                     64; ///< = 1<<(MAX_CU_DEPTH)
#endif
#if JVET_C0024_QTBT
static const Int MIN_PU_SIZE =                                      1<<MIN_CU_LOG2;
static const Int MIN_TU_SIZE =                                      1<<MIN_CU_LOG2;
#else
static const Int MIN_PU_SIZE =                                      4;
static const Int MIN_TU_SIZE =                                      4;
#endif
#if COM16_C806_T64
#if JVET_C0024_QTBT
static const Int MAX_TU_SIZE =                                      128; 
static const Int MAX_LOG2_TU_SIZE_PLUS_ONE =                        8; ///< log2(MAX_TU_SIZE) + 1 
#else
static const Int MAX_TU_SIZE =                                     64;
static const Int MAX_LOG2_TU_SIZE_PLUS_ONE =                        7; ///< log2(MAX_TU_SIZE) + 1
#endif
#else
static const Int MAX_TU_SIZE =                                     32;
#endif
static const Int MAX_NUM_PART_IDXS_IN_CTU_WIDTH = MAX_CU_SIZE/MIN_PU_SIZE; ///< maximum number of partition indices across the width of a CTU (or height of a CTU)
static const Int SCALING_LIST_REM_NUM =                             6;

static const Int QUANT_SHIFT =                                     14; ///< Q(4) = 2^14
static const Int IQUANT_SHIFT =                                     6;
static const Int SCALE_BITS =                                      15; ///< For fractional bit estimates in RDOQ

static const Int SCALING_LIST_NUM = MAX_NUM_COMPONENT * NUMBER_OF_PREDICTION_MODES; ///< list number for quantization matrix

static const Int SCALING_LIST_START_VALUE =                        8 ; ///< start value for dpcm mode
static const Int MAX_MATRIX_COEF_NUM =                            64 ; ///< max coefficient number for quantization matrix
static const Int MAX_MATRIX_SIZE_NUM =                             8 ; ///< max size number for quantization matrix
static const Int SCALING_LIST_BITS =                               8 ; ///< bit depth of scaling list entries
static const Int LOG2_SCALING_LIST_NEUTRAL_VALUE =                 4 ; ///< log2 of the value that, when used in a scaling list, has no effect on quantisation
static const Int SCALING_LIST_DC =                                16 ; ///< default DC value

static const Int CONTEXT_STATE_BITS =                              6 ;
#if COM16_C806_T64
#if JVET_C0024_QTBT
static const Int LAST_SIGNIFICANT_GROUPS =                        14 ;
#else
static const Int LAST_SIGNIFICANT_GROUPS =                        12 ;
#endif
#else
static const Int LAST_SIGNIFICANT_GROUPS =                        10 ;
#endif
#if VCEG_AZ07_FRUC_MERGE
static const Int FRUC_MERGE_OFF =                                0x0 ;
static const Int FRUC_MERGE_BILATERALMV =                        0x01;
static const Int FRUC_MERGE_TEMPLATE =                           0x02;
static const Int FRUC_MERGE_TEMPLATE_SIZE =                        4 ;
static const Int FRUC_MERGE_REFINE_MVWEIGHT =                      4 ;
static const Int FRUC_MERGE_REFINE_MINBLKSIZE =                    4 ;
#endif
#if VCEG_AZ07_CTX_RESIDUALCODING
static const Int MAX_GR_ORDER_RESIDUAL =                          10 ;
#endif
#if VCEG_AZ07_BAC_ADAPT_WDOW
#if !VCEG_AZ05_MULTI_PARAM_CABAC
static const Int ALPHA0 =                                           6; ///< 2^ALPHA0 is "window size" for probability up-date
#endif
static const Int CABAC_NUM_BINS =                              100000; ///< max number of bins for window calculation
static const Int NUM_WDOW =                                         4; ///< could be 16, 32, 64, 128
#endif
#if VCEG_AZ05_MULTI_PARAM_CABAC
static const Int ALPHA0 =                                           4; ///< 2^ALPHA0 is "window size" for probability up-date
#endif

#if COM16_C1016_AFFINE
static const Int AFFINE_MAX_NUM_V0 =                                3; ///< max number of motion candidates in top-left corner
static const Int AFFINE_MAX_NUM_V1 =                                2; ///< max number of motion candidates in top-right corner
static const Int AFFINE_MAX_NUM_V2 =                                2; ///< max number of motion candidates in left-bottom corner
static const Int AFFINE_MAX_NUM_COMB =                             12; ///< max number of combined motion candidates
static const Int AFFINE_MIN_BLOCK_SIZE =                            4; ///< Minimum affine MC block size
#endif

#if JVET_C0024_QTBT

#if JVET_C0024_AMAX_BT
static const Double AMAXBT_TH32 =                                   15.0;
static const Double AMAXBT_TH64 =                                   30.0;
#endif
 
static const Int    SKIP_DEPTH =                                    3;
static const Int    SKIPHORNOVERQT_DEPTH_TH =                       2;

#if JVET_C0024_FAST_MRG
static const Int    NUM_MRG_SATD_CAND =                             4;
static const Double MRG_FAST_RATIO    =                             1.25;
#endif

#if JVET_C0024_PBINTRA_FAST
static const Double PBINTRA_RATIO     =                             1.1;
#endif

//QTBT high level parameters
//for I slice luma CTB configuration para.
static const Int    MAX_BT_DEPTH  =                                 4;      ///<  <=7
static const Int    MAX_BT_SIZE   =                                 32;     ///<  [1<<MIN_QT_SIZE, 1<<CTU_LOG2]
static const Int    MIN_BT_SIZE   =                                 4;      ///<  can be set down to 1<<MIN_CU_LOG2

//for I slice chroma CTB configuration para. (in luma samples)
static const Int    MAX_BT_DEPTH_C =                                0;      ///< <=7   
static const Int    MAX_BT_SIZE_C  =                                16;     ///< [1<<MIN_QT_SIZE_C, 1<<CTU_LOG2]
static const Int    MIN_BT_SIZE_C  =                                4;      ///< can be set down to 4

//for P/B slice CTU config. para.
static const Int    MAX_BT_DEPTH_INTER =                            4;      ///< <=7
static const Int    MAX_BT_SIZE_INTER  =                          128;      ///< for initialization, [1<<MIN_BT_SIZE_INTER, 1<<CTU_LOG2]
static const Int    MIN_BT_SIZE_INTER  =                            4;      ///<

#endif

// ====================================================================================================================
// Macro functions
// ====================================================================================================================

#if COM16_C1016_AFFINE
#define SIGN(x)                       ( x >= 0 ? 1 : -1 )
#endif

template <typename T> inline T Clip3 (const T minVal, const T maxVal, const T a) { return std::min<T> (std::max<T> (minVal, a) , maxVal); }  ///< general min/max clip
template <typename T> inline T ClipBD(const T x, const Int bitDepth)             { return Clip3(T(0), T((1 << bitDepth)-1), x);           }

template <typename T> inline Void Check3( T minVal, T maxVal, T a)
{
  if ((a > maxVal) || (a < minVal))
  {
    std::cerr << "ERROR: Range check " << minVal << " >= " << a << " <= " << maxVal << " failed" << std::endl;
    assert(false);
    exit(1);
  }
}  ///< general min/max clip

#define DATA_ALIGN                  1                                                                 ///< use 32-bit aligned malloc/free
#if     DATA_ALIGN && _WIN32 && ( _MSC_VER > 1300 )
#define xMalloc( type, len )        _aligned_malloc( sizeof(type)*(len), 32 )
#define xFree( ptr )                _aligned_free  ( ptr )
#else
#define xMalloc( type, len )        malloc   ( sizeof(type)*(len) )
#define xFree( ptr )                free     ( ptr )
#endif

#define FATAL_ERROR_0(MESSAGE, EXITCODE)                      \
{                                                             \
  printf(MESSAGE);                                            \
  exit(EXITCODE);                                             \
}

template <typename ValueType> inline ValueType leftShift       (const ValueType value, const Int shift) { return (shift >= 0) ? ( value                                  << shift) : ( value                                   >> -shift); }
template <typename ValueType> inline ValueType rightShift      (const ValueType value, const Int shift) { return (shift >= 0) ? ( value                                  >> shift) : ( value                                   << -shift); }
template <typename ValueType> inline ValueType leftShift_round (const ValueType value, const Int shift) { return (shift >= 0) ? ( value                                  << shift) : ((value + (ValueType(1) << (-shift - 1))) >> -shift); }
template <typename ValueType> inline ValueType rightShift_round(const ValueType value, const Int shift) { return (shift >= 0) ? ((value + (ValueType(1) << (shift - 1))) >> shift) : ( value                                   << -shift); }
#if O0043_BEST_EFFORT_DECODING
// when shift = 0, returns value
// when shift = 1, (value + 0 + value[1]) >> 1
// when shift = 2, (value + 1 + value[2]) >> 2
// when shift = 3, (value + 3 + value[3]) >> 3
template <typename ValueType> inline ValueType rightShiftEvenRounding(const ValueType value, const UInt shift) { return (shift == 0) ? value : ((value + (1<<(shift-1))-1 + ((value>>shift)&1)) >> shift) ; }
#endif

//! \}

#endif // end of #ifndef  __COMMONDEF__

