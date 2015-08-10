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

/** \file     TypeDef.h
    \brief    Define basic types, new types and enumerations
*/

#ifndef _TYPEDEF__
#define _TYPEDEF__

//! \ingroup TLibCommon
//! \{

///////////////////////////////////////////////////////////
// Contribution COM16–C806 (QUALCOMM) defines section start
///////////////////////////////////////////////////////////

#define QC_LARGE_CTU                        1  ///< CTU size larger than 64x64, supporting up to 512x512 CTU size in the software,
#if QC_LARGE_CTU
#define QC_LARGE_CTU_FAST                   1
#endif

#define QC_T64                              1  ///< Enable 64x64 Transform

#define QC_EMT                              1  ///< Enhanced Multiple Transform (EMT)
#if QC_EMT
#define QC_EMT_INTRA                        1  ///< EMT for Intra prediction residual
#define QC_EMT_INTER                        1  ///< EMT for Inter prediction residual
#if QC_EMT_INTRA 
#define QC_EMT_INTRA_MAX_CU                 32 ///< Max Intra CU size applying EMT, supported values: 8, 16, 32
#define QC_EMT_INTRA_FAST                   1  //   Fast encoder methods of Intra EMT
#endif
#if QC_EMT_INTER 
#define QC_EMT_INTER_MAX_CU                 32 ///< Max Inter CU size applying EMT, supported values: 8, 16, 32
#define QC_EMT_INTER_FAST                   1  //   Fast encoder methods of Inter EMT
#endif
#endif

#if QC_EMT || QC_T64
#define QC_TRANS_PREC                       2  //   Integer transform matrix precision
#endif

#define QC_SUB_PU_TMVP                      1  ///< CY: sub-block level temporal motion prediction (a.k.a. ATMVP)
#if QC_SUB_PU_TMVP                     
#define QC_HEVC_MOTION_CONSTRAINT_REMOVAL   1
#define QC_DISABLE_4X4_PU                   1
#define GEN_MRG_IMPROVEMENT                 1
#endif

#define QC_OBMC                             1  ///< Overlapped Block Motion Compensation (OBMC)
#if QC_OBMC
#define QC_AOBMC_MAXCUSIZE                  16 //   Maximum CU size which can apply OBMC adaptively, larger CUs always apply OBMC
#endif

#define QC_LMCHROMA                         1  ///< Cross component prediction: predict chroma from luma or Cr from Cb with linear model
#if QC_LMCHROMA
#define CR_FROM_CB_REG_COST_SHIFT           9     
#define CR_FROM_CB_LAMBDA_ADJUSTMENT        1
#define LM_DOWNSAMPLE_NUM_ROWS              2  //  [1, 2, 1; 1, 2, 1]/8 filter is used for downsampling luma signal
#define LM_DOWNSAMPLE_NUM_COLUMNS           3  
#endif

#define ALF_HM3_QC_REFACTOR                 1  ///< Adaptive loop filter with 4x4 block activity adaptation 
#if ALF_HM3_QC_REFACTOR
#define ALF_HM3_VAR_SIZE_H                  4
#define ALF_HM3_VAR_SIZE_W                  4
#define ALF_HM3_NO_VAR_BIN                  16
#define ALF_WIN_VERSIZE                     32
#define ALF_WIN_HORSIZE                     32
#define QC_ALF_TMEPORAL_NUM                 6  //   0: no temporal prediction
#endif

// code cleaning and optimization
#define QC_SIMD_OPT                         1
#define HM14_CLEAN_UP                       1
#if HM14_CLEAN_UP
#define MLS_CG_BITS                         2
#endif
///////////////////////////////////////////////////////////
// Contribution COM16–C806 (QUALCOMM) defines section end
///////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////
// Contribution VCEG-AZ05 (SAMSUNG) defines section starts
///////////////////////////////////////////////////////////
#define MULTI_PARAM_CABAC                   1
#if MULTI_PARAM_CABAC
#define ALPHA0                              4       // 2^ALPHA0 is 1st "window size" for probability up-date (4,5,6,7; could be adaptive if ENABLE_ADAPTIVE_W==1)
#endif
#define BIO                                 1  // bi-directional optical flow
#define ROT_TR                              1  // rotational transform for 4x4 coefficients sub-blocks
#define CU_LEVEL_MPI                        1 // multi-parameter Intra prediction
#if CU_LEVEL_MPI
  #define MPI_DICT_SIZE_INTRA         4
  #define MPI_DICT_SIZE_INTER         2
#endif
///////////////////////////////////////////////////////////
// Contribution VCEG-AZ05 (SAMSUNG) defines section ends
///////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
// Contribution VCEG-AZ07 (QUALCOMM) defines section starts
///////////////////////////////////////////////////////////
#define QC_ECABAC                           1  /// CABAC improvements
#if QC_ECABAC
#define DEBUG                               1  //for CABAC debug
#define QC_CTX_RESIDUALCODING               1  //new ctx for residual coding

#define QC_AC_ADAPT_WDOW                    1
#if QC_AC_ADAPT_WDOW
#if !MULTI_PARAM_CABAC
#define ALPHA0                              6       // 2^ALPHA0 is "window size" for probability up-date
#endif
#define CABAC_NUM_BINS                      100000
#define NUM_WDOW                            4       //could be 16, 32, 64, 128
#define ENABLE_ADAPTIVE_W                   1       //0: always use ALPHA0
#define INIT_PREVFRAME                      1       //initilized probabilities are from previously coded frames
#endif
#endif

#define QC_FRUC_MERGE                       1
#if QC_FRUC_MERGE
#define QC_FRUC_MERGE_OFF                   0x0
#define QC_FRUC_MERGE_BILATERALMV           0x01
#define QC_FRUC_MERGE_TEMPLATE              0x02
#define QC_FRUC_MERGE_TEMPLATE_SIZE         4
#define QC_FRUC_MERGE_REFINE_MVWEIGHT       4
#define QC_FRUC_MERGE_REFINE_MINBLKSIZE     4
#define QC_MV_STORE_PRECISION_BIT           3
#define QC_MV_SIGNAL_PRECISION_BIT          2
#endif

#define QC_IMV                              1

#define QC_SUB_PU_TMVP_V08                  QC_FRUC_MERGE
#define MERGE_CAND_NUM_PATCH                1

#if QC_SUB_PU_TMVP
#define QC_SUB_PU_TMVP_EXT                  1
#endif

#define QC_INTRA_4TAP_FILTER                1 ///< Intra 4-tap interpolation filters
#define INTRA_BOUNDARY_FILTER               1 ///< Intra Boundary Filtering
#if INTRA_BOUNDARY_FILTER
#define INTRA_BOUNDARY_FILTER_MULTI_LINE    1 /// 0: Filter one boundary line, 1: Filter 4 boundary lines
#endif

#define QC_USE_65ANG_MODES                  1 ///< Extended angular intra prediction, including 65 angular modes & 6 MPMs

#define QC_IC                               1 ///< Illumination Compensation
#if QC_IC
#define IC_REG_COST_SHIFT                   7
#define IC_CONST_SHIFT                      5
#define IC_SHIFT_DIFF                       12
#define QC_IC_SPDUP                         1 //speedup of IC
#endif
///////////////////////////////////////////////////////////
// Contribution VCEG-AZ07 (QUALCOMM) defines section ends
///////////////////////////////////////////////////////////

#define HARMONIZE_GOP_FIRST_FIELD_COUPLE  1
#define FIX_FIELD_DEPTH                 1
#define EFFICIENT_FIELD_IRAP            1
#define ALLOW_RECOVERY_POINT_AS_RAP     1
#define BUGFIX_INTRAPERIOD 1
#define SAO_ENCODE_ALLOW_USE_PREDEBLOCK 1

#define SAO_SGN_FUNC 1

#define FIX1172 1 ///< fix ticket #1172

#define SETTING_PIC_OUTPUT_MARK     1
#define SETTING_NO_OUT_PIC_PRIOR    1
#define FIX_EMPTY_PAYLOAD_NAL       1
#define FIX_WRITING_OUTPUT          1
#define FIX_OUTPUT_EOS              1

#define FIX_POC_CRA_NORASL_OUTPUT   1

#define MAX_NUM_PICS_IN_SOP           1024

#define MAX_NESTING_NUM_OPS         1024
#define MAX_NESTING_NUM_LAYER       64

#define MAX_VPS_NUM_HRD_PARAMETERS                1
#define MAX_VPS_OP_SETS_PLUS1                     1024
#define MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1  1

#define MAX_CPB_CNT                     32  ///< Upper bound of (cpb_cnt_minus1 + 1)
#define MAX_NUM_LAYER_IDS                64

#if !QC_CTX_RESIDUALCODING
#define COEF_REMAIN_BIN_REDUCTION        3 ///< indicates the level at which the VLC 
#endif                                     ///< transitions from Golomb-Rice to TU+EG(k)

#define CU_DQP_TU_CMAX 5                   ///< max number bins for truncated unary
#define CU_DQP_EG_k 0                      ///< expgolomb order

#define SBH_THRESHOLD                    4  ///< I0156: value of the fixed SBH controlling threshold
  
#define SEQUENCE_LEVEL_LOSSLESS           0  ///< H0530: used only for sequence or frame-level lossless coding

#define DISABLING_CLIP_FOR_BIPREDME         1  ///< Ticket #175
  
#define C1FLAG_NUMBER               8 // maximum number of largerThan1 flag coded in one chunk :  16 in HM5
#define C2FLAG_NUMBER               1 // maximum number of largerThan2 flag coded in one chunk:  16 in HM5 
#define SAO_ENCODING_CHOICE              1  ///< I0184: picture early termination
#if SAO_ENCODING_CHOICE
#define SAO_ENCODING_RATE                0.75
#define SAO_ENCODING_CHOICE_CHROMA       1 ///< J0044: picture early termination Luma and Chroma are handled separately
#if SAO_ENCODING_CHOICE_CHROMA
#define SAO_ENCODING_RATE_CHROMA         0.5
#endif
#endif

#define MAX_NUM_VPS                16
#define MAX_NUM_SPS                16
#define MAX_NUM_PPS                64

#define RDOQ_CHROMA_LAMBDA          1   ///< F386: weighting of chroma for RDOQ

#define MIN_SCAN_POS_CROSS          4

#define FAST_BIT_EST                1   ///< G763: Table-based bit estimation for CABAC

#if QC_T64
#define MLS_GRP_NUM                         256    ///< Max number of coefficient groups, max(16, 256)
#else
#define MLS_GRP_NUM                         64     ///< G644 : Max number of coefficient groups, max(16, 64)
#endif
#define MLS_CG_SIZE                         4      ///< G644 : Coefficient group size of 4x4

#define ADAPTIVE_QP_SELECTION               1      ///< G382: Adaptive reconstruction levels, non-normative part for adaptive QP selection
#if ADAPTIVE_QP_SELECTION
#define ARL_C_PRECISION                     7      ///< G382: 7-bit arithmetic precision
#define LEVEL_RANGE                         30     ///< G382: max coefficient level in statistics collection
#endif

#define HHI_RQT_INTRA_SPEEDUP             1           ///< tests one best mode with full rqt
#define HHI_RQT_INTRA_SPEEDUP_MOD         0           ///< tests two best modes with full rqt

#if HHI_RQT_INTRA_SPEEDUP_MOD && !HHI_RQT_INTRA_SPEEDUP
#error
#endif

#define VERBOSE_RATE 0 ///< Print additional rate information in encoder

#define AMVP_DECIMATION_FACTOR            4

#define SCAN_SET_SIZE                     16
#define LOG2_SCAN_SET_SIZE                4

#define FAST_UDI_MAX_RDMODE_NUM               35          ///< maximum number of RD comparison in fast-UDI estimation loop 

#define ZERO_MVD_EST                          0           ///< Zero Mvd Estimation in normal mode

#if QC_USE_65ANG_MODES
#define NUM_INTRA_MODE 68
#else
#define NUM_INTRA_MODE 36
#endif

#define WRITE_BACK                      1           ///< Enable/disable the encoder to replace the deltaPOC and Used by current from the config file with the values derived by the refIdc parameter.
#define AUTO_INTER_RPS                  1           ///< Enable/disable the automatic generation of refIdc from the deltaPOC and Used by current from the config file.
#define PRINT_RPS_INFO                  0           ///< Enable/disable the printing of bits used to send the RPS.
                                                    // using one nearest frame as reference frame, and the other frames are high quality (POC%4==0) frames (1+X)
                                                    // this should be done with encoder only decision
                                                    // but because of the absence of reference frame management, the related code was hard coded currently

#define RVM_VCEGAM10_M 4

#define PLANAR_IDX             0
#if QC_USE_65ANG_MODES
#define NUM_DIR                (((NUM_INTRA_MODE-4)>>2)+1)
#define HOR_IDX                (1*(NUM_DIR-1)+2)       // index for intra HORIZONTAL mode
#define DIA_IDX                (2*(NUM_DIR-1)+2)       // index for intra DIAGONAL   mode
#define VER_IDX                (3*(NUM_DIR-1)+2)       // index for intra VERTICAL   mode
#define VDIA_IDX               (4*(NUM_DIR-1)+2)       // index for intra VERTICAL DIAGONAL   mode
#else
#define VER_IDX                26                    // index for intra VERTICAL   mode
#define HOR_IDX                10                    // index for intra HORIZONTAL mode
#endif
#define DC_IDX                 1                     // index for intra DC mode
#if QC_LMCHROMA
#define NUM_CHROMA_MODE        6                     // total number of chroma modes
#if QC_USE_65ANG_MODES
#define LM_CHROMA_IDX          (NUM_INTRA_MODE-1)
#else
#define LM_CHROMA_IDX          35
#endif
#else
#define NUM_CHROMA_MODE        5                     // total number of chroma modes
#endif
#if QC_USE_65ANG_MODES
#define DM_CHROMA_IDX          NUM_INTRA_MODE        // chroma mode index for derived from luma intra mode
#else
#define DM_CHROMA_IDX          36                    // chroma mode index for derived from luma intra mode
#endif

#define FAST_UDI_USE_MPM 1

#define RDO_WITHOUT_DQP_BITS              0           ///< Disable counting dQP bits in RDO-based mode decision

#define FULL_NBIT 0 ///< When enabled, compute costs using full sample bitdepth.  When disabled, compute costs as if it is 8-bit source video.
#if FULL_NBIT
# define DISTORTION_PRECISION_ADJUSTMENT(x) 0
#else
# define DISTORTION_PRECISION_ADJUSTMENT(x) (x)
#endif

#define LOG2_MAX_NUM_COLUMNS_MINUS1        7
#define LOG2_MAX_NUM_ROWS_MINUS1           7
#define LOG2_MAX_COLUMN_WIDTH              13
#define LOG2_MAX_ROW_HEIGHT                13

#define REG_DCT                           65535
#if QC_EMT
#define INTER_MODE                        65534
#endif

#define AMP_SAD                               1           ///< dedicated SAD functions for AMP
#define AMP_ENC_SPEEDUP                       1           ///< encoder only speed-up by AMP mode skipping
#if AMP_ENC_SPEEDUP
#define AMP_MRG                               1           ///< encoder only force merge for AMP partition (no motion search for AMP)
#endif

#define CABAC_INIT_PRESENT_FLAG     1

// ====================================================================================================================
// Basic type redefinition
// ====================================================================================================================

typedef       void                Void;
typedef       bool                Bool;

#ifdef __arm__
typedef       signed char         Char;
#else
typedef       char                Char;
#endif
typedef       unsigned char       UChar;
typedef       short               Short;
typedef       unsigned short      UShort;
typedef       int                 Int;
typedef       unsigned int        UInt;
typedef       double              Double;
typedef       float               Float;

// ====================================================================================================================
// 64-bit integer type
// ====================================================================================================================

#ifdef _MSC_VER
typedef       __int64             Int64;

#if _MSC_VER <= 1200 // MS VC6
typedef       __int64             UInt64;   // MS VC6 does not support unsigned __int64 to double conversion
#else
typedef       unsigned __int64    UInt64;
#endif

#else

typedef       long long           Int64;
typedef       unsigned long long  UInt64;

#endif

// ====================================================================================================================
// Type definition
// ====================================================================================================================

typedef       UChar           Pxl;        ///< 8-bit pixel type
typedef       Short           Pel;        ///< 16-bit pixel type
typedef       Int             TCoeff;     ///< transform coefficient

/// parameters for adaptive loop filter
class TComPicSym;

// Slice / Slice segment encoding modes
enum SliceConstraint
{
  NO_SLICES              = 0,          ///< don't use slices / slice segments
  FIXED_NUMBER_OF_LCU    = 1,          ///< Limit maximum number of largest coding tree blocks in a slice / slice segments
  FIXED_NUMBER_OF_BYTES  = 2,          ///< Limit maximum number of bytes in a slice / slice segment
  FIXED_NUMBER_OF_TILES  = 3,          ///< slices / slice segments span an integer number of tiles
};

enum SAOComponentIdx
{
  SAO_Y =0,
  SAO_Cb,
  SAO_Cr,
  NUM_SAO_COMPONENTS
};

enum SAOMode //mode
{
  SAO_MODE_OFF = 0,
  SAO_MODE_NEW,
  SAO_MODE_MERGE,
  NUM_SAO_MODES
};

enum SAOModeMergeTypes 
{
  SAO_MERGE_LEFT =0,
  SAO_MERGE_ABOVE,
  NUM_SAO_MERGE_TYPES
};


enum SAOModeNewTypes 
{
  SAO_TYPE_START_EO =0,
  SAO_TYPE_EO_0 = SAO_TYPE_START_EO,
  SAO_TYPE_EO_90,
  SAO_TYPE_EO_135,
  SAO_TYPE_EO_45,
  
  SAO_TYPE_START_BO,
  SAO_TYPE_BO = SAO_TYPE_START_BO,

  NUM_SAO_NEW_TYPES
};
#define NUM_SAO_EO_TYPES_LOG2 2

enum SAOEOClasses 
{
  SAO_CLASS_EO_FULL_VALLEY = 0,
  SAO_CLASS_EO_HALF_VALLEY = 1,
  SAO_CLASS_EO_PLAIN       = 2,
  SAO_CLASS_EO_HALF_PEAK   = 3,
  SAO_CLASS_EO_FULL_PEAK   = 4,
  NUM_SAO_EO_CLASSES,
};


#define NUM_SAO_BO_CLASSES_LOG2  5
enum SAOBOClasses
{
  //SAO_CLASS_BO_BAND0 = 0,
  //SAO_CLASS_BO_BAND1,
  //SAO_CLASS_BO_BAND2,
  //...
  //SAO_CLASS_BO_BAND31,

  NUM_SAO_BO_CLASSES = (1<<NUM_SAO_BO_CLASSES_LOG2),
};
#define MAX_NUM_SAO_CLASSES  32  //(NUM_SAO_EO_GROUPS > NUM_SAO_BO_GROUPS)?NUM_SAO_EO_GROUPS:NUM_SAO_BO_GROUPS

struct SAOOffset
{
  Int modeIdc; //NEW, MERGE, OFF
  Int typeIdc; //NEW: EO_0, EO_90, EO_135, EO_45, BO. MERGE: left, above
  Int typeAuxInfo; //BO: starting band index
  Int offset[MAX_NUM_SAO_CLASSES];

  SAOOffset();
  ~SAOOffset();
  Void reset();

  const SAOOffset& operator= (const SAOOffset& src);
};

struct SAOBlkParam
{

  SAOBlkParam();
  ~SAOBlkParam();
  Void reset();
  const SAOBlkParam& operator= (const SAOBlkParam& src);
  SAOOffset& operator[](Int compIdx){ return offsetParam[compIdx];}
private:
  SAOOffset offsetParam[NUM_SAO_COMPONENTS];

};

/// parameters for deblocking filter
typedef struct _LFCUParam
{
  Bool bInternalEdge;                     ///< indicates internal edge
  Bool bLeftEdge;                         ///< indicates left edge
  Bool bTopEdge;                          ///< indicates top edge
} LFCUParam;

// ====================================================================================================================
// Enumeration
// ====================================================================================================================

/// supported slice type
enum SliceType
{
  B_SLICE,
  P_SLICE,
  I_SLICE
};

/// chroma formats (according to semantics of chroma_format_idc)
enum ChromaFormat
{
  CHROMA_400  = 0,
  CHROMA_420  = 1,
  CHROMA_422  = 2,
  CHROMA_444  = 3
};

/// supported partition shape
enum PartSize
{
  SIZE_2Nx2N,           ///< symmetric motion partition,  2Nx2N
  SIZE_2NxN,            ///< symmetric motion partition,  2Nx N
  SIZE_Nx2N,            ///< symmetric motion partition,   Nx2N
  SIZE_NxN,             ///< symmetric motion partition,   Nx N
  SIZE_2NxnU,           ///< asymmetric motion partition, 2Nx( N/2) + 2Nx(3N/2)
  SIZE_2NxnD,           ///< asymmetric motion partition, 2Nx(3N/2) + 2Nx( N/2)
  SIZE_nLx2N,           ///< asymmetric motion partition, ( N/2)x2N + (3N/2)x2N
  SIZE_nRx2N,           ///< asymmetric motion partition, (3N/2)x2N + ( N/2)x2N
#if QC_IMV
  NUMBER_OF_PART_SIZES = 8,
#endif
  SIZE_NONE = 15
};

/// supported prediction type
enum PredMode
{
  MODE_INTER,           ///< inter-prediction mode
  MODE_INTRA,           ///< intra-prediction mode
  MODE_NONE = 15
};

/// texture component type
enum TextType
{
  TEXT_LUMA,            ///< luma
  TEXT_CHROMA,          ///< chroma (U+V)
  TEXT_CHROMA_U,        ///< chroma U
  TEXT_CHROMA_V,        ///< chroma V
  TEXT_ALL,             ///< Y+U+V
  TEXT_NONE = 15
};

/// reference list index
enum RefPicList
{
  REF_PIC_LIST_0 = 0,   ///< reference list 0
  REF_PIC_LIST_1 = 1,   ///< reference list 1
  REF_PIC_LIST_X = 100  ///< special mark
};

/// distortion function index
enum DFunc
{
  DF_DEFAULT  = 0,
  DF_SSE      = 1,      ///< general size SSE
  DF_SSE4     = 2,      ///<   4xM SSE
  DF_SSE8     = 3,      ///<   8xM SSE
  DF_SSE16    = 4,      ///<  16xM SSE
  DF_SSE32    = 5,      ///<  32xM SSE
  DF_SSE64    = 6,      ///<  64xM SSE
  DF_SSE16N   = 7,      ///< 16NxM SSE
  
  DF_SAD      = 8,      ///< general size SAD
  DF_SAD4     = 9,      ///<   4xM SAD
  DF_SAD8     = 10,     ///<   8xM SAD
  DF_SAD16    = 11,     ///<  16xM SAD
  DF_SAD32    = 12,     ///<  32xM SAD
  DF_SAD64    = 13,     ///<  64xM SAD
  DF_SAD16N   = 14,     ///< 16NxM SAD
  
  DF_SADS     = 15,     ///< general size SAD with step
  DF_SADS4    = 16,     ///<   4xM SAD with step
  DF_SADS8    = 17,     ///<   8xM SAD with step
  DF_SADS16   = 18,     ///<  16xM SAD with step
  DF_SADS32   = 19,     ///<  32xM SAD with step
  DF_SADS64   = 20,     ///<  64xM SAD with step
  DF_SADS16N  = 21,     ///< 16NxM SAD with step
  
  DF_HADS     = 22,     ///< general size Hadamard with step
  DF_HADS4    = 23,     ///<   4xM HAD with step
  DF_HADS8    = 24,     ///<   8xM HAD with step
  DF_HADS16   = 25,     ///<  16xM HAD with step
  DF_HADS32   = 26,     ///<  32xM HAD with step
  DF_HADS64   = 27,     ///<  64xM HAD with step
  DF_HADS16N  = 28,     ///< 16NxM HAD with step
  
#if AMP_SAD
  DF_SAD12    = 43,
  DF_SAD24    = 44,
  DF_SAD48    = 45,

  DF_SADS12   = 46,
  DF_SADS24   = 47,
  DF_SADS48   = 48,

  DF_SSE_FRAME = 50     ///< Frame-based SSE
#else
  DF_SSE_FRAME = 33     ///< Frame-based SSE
#endif
};

/// index for SBAC based RD optimization
enum CI_IDX
{
  CI_CURR_BEST = 0,     ///< best mode index
  CI_NEXT_BEST,         ///< next best index
  CI_TEMP_BEST,         ///< temporal index
  CI_CHROMA_INTRA,      ///< chroma intra index
  CI_QT_TRAFO_TEST,
  CI_QT_TRAFO_ROOT,
  CI_NUM,               ///< total number
};

/// motion vector predictor direction used in AMVP
enum MVP_DIR
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

/// coefficient scanning type used in ACS
enum COEFF_SCAN_TYPE
{
  SCAN_DIAG = 0,         ///< up-right diagonal scan
  SCAN_HOR,              ///< horizontal first scan
  SCAN_VER               ///< vertical first scan
};

#if QC_EMT || QC_T64
enum TRANS_TYPE
{
  DCT2, DCT5, DCT8, DST1, DST7, NUM_TRANS_TYPE,
  DCT2_HEVC, DCT2_EMT
};
#endif

namespace Profile
{
  enum Name
  {
    NONE = 0,
    MAIN = 1,
    MAIN10 = 2,
    MAINSTILLPICTURE = 3,
  };
}

namespace Level
{
  enum Tier
  {
    MAIN = 0,
    HIGH = 1,
  };

  enum Name
  {
    NONE     = 0,
    LEVEL1   = 30,
    LEVEL2   = 60,
    LEVEL2_1 = 63,
    LEVEL3   = 90,
    LEVEL3_1 = 93,
    LEVEL4   = 120,
    LEVEL4_1 = 123,
    LEVEL5   = 150,
    LEVEL5_1 = 153,
    LEVEL5_2 = 156,
    LEVEL6   = 180,
    LEVEL6_1 = 183,
    LEVEL6_2 = 186,
  };
}

#if ALF_HM3_QC_REFACTOR
struct _AlfParam
{
  Int alf_flag;                           ///< indicates use of ALF
  Int cu_control_flag;                    ///< coding unit based control flag
  Int chroma_idc;                         ///< indicates use of ALF for chroma
  Int tap;                                ///< number of filter taps - horizontal
  Int tapV;                               ///< number of filter taps - vertical
  Int num_coeff;                          ///< number of filter coefficients
  Int *coeff;                             ///< filter coefficient array
  Int tap_chroma;                         ///< number of filter taps (chroma)
  Int num_coeff_chroma;                   ///< number of filter coefficients (chroma)
  Int *coeff_chroma;                      ///< filter coefficient array (chroma)
  //CodeAux related
  Int realfiltNo;
  Int filtNo;
  Int filterPattern[ALF_HM3_NO_VAR_BIN];
  Int startSecondFilter;
  Int noFilters;
  Int varIndTab[ALF_HM3_NO_VAR_BIN];
#if QC_ALF_TMEPORAL_NUM
  Bool temproalPredFlag; //indicate whether reuse previous ALF coefficients
  Int  prevIdx;          //index of the reused ALF coefficients
  Int  **alfCoeffLuma;    
  Int  *alfCoeffChroma;
#endif
  //Coeff send related
  Int filters_per_group_diff; //this can be updated using codedVarBins
  Int filters_per_group;
  Int codedVarBins[ALF_HM3_NO_VAR_BIN]; 
  Int forceCoeff0;
  Int predMethod;
  Int **coeffmulti;
  Int minKStart;
  Int maxScanVal;
  Int kMinTab[42];
  UInt num_alf_cu_flag;
  UInt num_cus_in_frame;
  UInt alf_max_depth;
  UInt *alf_cu_flag;

};
#endif

#if QC_AC_ADAPT_WDOW
typedef struct _CABACState
{
  Bool bActived;
  UChar uiWdow; 
  UInt uiQP;
} CABACState;

typedef struct _QPFLAG
{
  UInt uiQP;       
  Bool bUsed;      //same QP, same type has appearaed
  Bool bFirstUsed; //same QP, same type was firstly signaled
#if INIT_PREVFRAME
  UInt uiResetInit; //for the first B/P frame after intra slice, no init update
#endif
} QPFlag;
#endif
//! \}

#endif
