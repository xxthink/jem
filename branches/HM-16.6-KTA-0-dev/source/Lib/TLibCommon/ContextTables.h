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

/** \file     ContextTables.h
    \brief    Defines constants and tables for SBAC
    \todo     number of context models is not matched to actual use, should be fixed
*/

#ifndef __CONTEXTTABLES__
#define __CONTEXTTABLES__

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define MAX_NUM_CTX_MOD             512       ///< maximum number of supported contexts

#if COM16_C806_LARGE_CTU
#define NUM_SPLIT_FLAG_CTX            5       ///< number of context models for split flag
#else
#define NUM_SPLIT_FLAG_CTX            3       ///< number of context models for split flag
#endif
#define NUM_SKIP_FLAG_CTX             3       ///< number of context models for skip flag

#define NUM_MERGE_FLAG_EXT_CTX        1       ///< number of context models for merge flag of merge extended
#if COM16_C806_GEN_MRG_IMPROVEMENT
#define NUM_MERGE_IDX_EXT_CTX         5       ///< number of context models for merge index of merge extended
#else
#define NUM_MERGE_IDX_EXT_CTX         1       ///< number of context models for merge index of merge extended
#endif
#if COM16_C806_OBMC
#define NUM_OBMC_FLAG_CTX             1       ///< number of context models for OBMC flag
#endif

#if VCEG_AZ07_IMV
#define NUM_IMV_FLAG_CTX              3       ///< number of context models for iMV flag
#endif
#if VCEG_AZ07_FRUC_MERGE
#define NUM_FRUCMGRMODE_CTX           3
#define NUM_FRUCME_CTX                3
#endif
#if VCEG_AZ06_IC
#define NUM_IC_FLAG_CTX               1       ///< number of context models for illumination compensation flag
#endif

#define NUM_PART_SIZE_CTX             4       ///< number of context models for partition size
#define NUM_PRED_MODE_CTX             1       ///< number of context models for prediction mode

#if VCEG_AZ07_INTRA_65ANG_MODES
#define NUM_INTRA_PREDICT_CTX         9       ///< number of context models for intra prediction
#else
#define NUM_INTRA_PREDICT_CTX         1       ///< number of context models for intra prediction
#endif

#define NUM_CHROMA_PRED_CTX           2       ///< number of context models for intra prediction (chroma)
#define NUM_INTER_DIR_CTX             5       ///< number of context models for inter prediction direction
#define NUM_MV_RES_CTX                2       ///< number of context models for motion vector difference
#define NUM_CHROMA_QP_ADJ_FLAG_CTX    1       ///< number of context models for chroma_qp_adjustment_flag
#define NUM_CHROMA_QP_ADJ_IDC_CTX     1       ///< number of context models for chroma_qp_adjustment_idc

#define NUM_REF_NO_CTX                2       ///< number of context models for reference index
#if COM16_C806_T64
#define NUM_TRANS_SUBDIV_FLAG_CTX     4       ///< number of context models for transform subdivision flags
#else
#define NUM_TRANS_SUBDIV_FLAG_CTX     3       ///< number of context models for transform subdivision flags
#endif
#define NUM_QT_ROOT_CBF_CTX           1       ///< number of context models for QT ROOT CBF
#define NUM_DELTA_QP_CTX              3       ///< number of context models for dQP

#define NUM_SIG_CG_FLAG_CTX           2       ///< number of context models for MULTI_LEVEL_SIGNIFICANCE
#define NUM_EXPLICIT_RDPCM_FLAG_CTX   1       ///< number of context models for the flag which specifies whether to use RDPCM on inter coded residues
#define NUM_EXPLICIT_RDPCM_DIR_CTX    1       ///< number of context models for the flag which specifies which RDPCM direction is used on inter coded residues

//--------------------------------------------------------------------------------------------------

// context size definitions for significance map

#define NUM_SIG_FLAG_CTX_LUMA        28       ///< number of context models for luma sig flag
#define NUM_SIG_FLAG_CTX_CHROMA      16       ///< number of context models for chroma sig flag

//                                                                                                           |----Luma-----|  |---Chroma----|
static const UInt significanceMapContextSetStart         [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {0,  9, 21, 27}, {0,  9, 12, 15} };
static const UInt significanceMapContextSetSize          [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {9, 12,  6,  1}, {9,  3,  3,  1} };
static const UInt nonDiagonalScan8x8ContextOffset        [MAX_NUM_CHANNEL_TYPE]                          = {  6,               0              };
static const UInt notFirstGroupNeighbourhoodContextOffset[MAX_NUM_CHANNEL_TYPE]                          = {  3,               0              };

//------------------

#define NEIGHBOURHOOD_00_CONTEXT_1_THRESHOLD_4x4  3
#define NEIGHBOURHOOD_00_CONTEXT_2_THRESHOLD_4x4  1

//------------------

#define FIRST_SIG_FLAG_CTX_LUMA                   0
#define FIRST_SIG_FLAG_CTX_CHROMA     (FIRST_SIG_FLAG_CTX_LUMA + NUM_SIG_FLAG_CTX_LUMA)

#define NUM_SIG_FLAG_CTX              (NUM_SIG_FLAG_CTX_LUMA + NUM_SIG_FLAG_CTX_CHROMA)       ///< number of context models for sig flag

//--------------------------------------------------------------------------------------------------

// context size definitions for last significant coefficient position

#define NUM_CTX_LAST_FLAG_SETS         2

#if COM16_C806_T64
#define NUM_CTX_LAST_FLAG_XY          19      ///< number of context models for last coefficient position
#else
#define NUM_CTX_LAST_FLAG_XY          15      ///< number of context models for last coefficient position
#endif

//--------------------------------------------------------------------------------------------------

// context size definitions for greater-than-one and greater-than-two maps

#define NUM_ONE_FLAG_CTX_PER_SET       4      ///< number of context models for greater than 1 flag in a set
#define NUM_ABS_FLAG_CTX_PER_SET       1      ///< number of context models for greater than 2 flag in a set

//------------------

#define NUM_CTX_SETS_LUMA              4      ///< number of context model sets for luminance
#define NUM_CTX_SETS_CHROMA            2      ///< number of context model sets for combined chrominance

#define FIRST_CTX_SET_LUMA             0      ///< index of first luminance context set

//------------------

#define NUM_ONE_FLAG_CTX_LUMA         (NUM_ONE_FLAG_CTX_PER_SET * NUM_CTX_SETS_LUMA)           ///< number of context models for greater than 1 flag of luma
#define NUM_ONE_FLAG_CTX_CHROMA       (NUM_ONE_FLAG_CTX_PER_SET * NUM_CTX_SETS_CHROMA)         ///< number of context models for greater than 1 flag of chroma

#define NUM_ABS_FLAG_CTX_LUMA         (NUM_ABS_FLAG_CTX_PER_SET * NUM_CTX_SETS_LUMA)           ///< number of context models for greater than 2 flag of luma
#define NUM_ABS_FLAG_CTX_CHROMA       (NUM_ABS_FLAG_CTX_PER_SET * NUM_CTX_SETS_CHROMA)         ///< number of context models for greater than 2 flag of chroma

#define NUM_ONE_FLAG_CTX              (NUM_ONE_FLAG_CTX_LUMA + NUM_ONE_FLAG_CTX_CHROMA)        ///< number of context models for greater than 1 flag
#define NUM_ABS_FLAG_CTX              (NUM_ABS_FLAG_CTX_LUMA + NUM_ABS_FLAG_CTX_CHROMA)        ///< number of context models for greater than 2 flag

#define FIRST_CTX_SET_CHROMA          (FIRST_CTX_SET_LUMA + NUM_CTX_SETS_LUMA)                 ///< index of first chrominance context set

//--------------------------------------------------------------------------------------------------

// context size definitions for CBF

#define NUM_QT_CBF_CTX_SETS           2

#define NUM_QT_CBF_CTX_PER_SET        5       ///< number of context models for QT CBF

#define FIRST_CBF_CTX_LUMA            0       ///< index of first luminance CBF context

#define FIRST_CBF_CTX_CHROMA          (FIRST_CBF_CTX_LUMA + NUM_QT_CBF_CTX_PER_SET)  ///< index of first chrominance CBF context


//--------------------------------------------------------------------------------------------------

#define NUM_MVP_IDX_CTX               1       ///< number of context models for MVP index

#define NUM_SAO_MERGE_FLAG_CTX        1       ///< number of context models for SAO merge flags
#define NUM_SAO_TYPE_IDX_CTX          1       ///< number of context models for SAO type index

#define NUM_TRANSFORMSKIP_FLAG_CTX    1       ///< number of context models for transform skipping

#define NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX  1

#define NUM_CROSS_COMPONENT_PREDICTION_CTX 10

#if ALF_HM3_REFACTOR
#define NUM_ALF_CTRL_FLAG_CTX         3       ///< number of context models for ALF control flag
#define NUM_ALF_FLAG_CTX              1       ///< number of context models for ALF flag
#define NUM_ALF_UVLC_CTX              2       ///< number of context models for ALF UVLC (filter length)
#define NUM_ALF_SVLC_CTX              3       ///< number of context models for ALF SVLC (filter coeff.)
#endif

#if COM16_C806_EMT
#define NUM_EMT_TU_IDX_CTX            4       ///< number of context models for EMT TU-level transform index
#if COM16_C806_LARGE_CTU
#define NUM_EMT_CU_FLAG_CTX           6       ///< number of context models for EMT CU-level flag
#else
#define NUM_EMT_CU_FLAG_CTX           4       ///< number of context models for EMT CU-level flag
#endif
#endif

#define CNU                          154      ///< dummy initialization value for unused context models 'Context model Not Used'


// ====================================================================================================================
// Tables
// ====================================================================================================================

// initial probability for cu_transquant_bypass flag
static const UChar
INIT_CU_TRANSQUANT_BYPASS_FLAG[NUMBER_OF_SLICE_TYPES][NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX] =
{
  { 154 },
  { 154 },
  { 154 },
};

// initial probability for split flag
static const UChar
INIT_SPLIT_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SPLIT_FLAG_CTX] =  
{
#if COM16_C806_LARGE_CTU
  { 107,  139,  126, 255, 0, },
  { 107,  139,  126, 255, 0, }, 
  { 139,  141,  157, 255, 0, }, 
#else
  { 107,  139,  126, },
  { 107,  139,  126, }, 
  { 139,  141,  157, }, 
#endif
};

static const UChar
INIT_SKIP_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SKIP_FLAG_CTX] =
{
  { 197,  185,  201, },
  { 197,  185,  201, },
  { CNU,  CNU,  CNU, },
};

static const UChar
INIT_MERGE_FLAG_EXT[NUMBER_OF_SLICE_TYPES][NUM_MERGE_FLAG_EXT_CTX] =
{
  { 154, },
  { 110, },
  { CNU, },
};

static const UChar
INIT_MERGE_IDX_EXT[NUMBER_OF_SLICE_TYPES][NUM_MERGE_IDX_EXT_CTX] =
{
#if COM16_C806_GEN_MRG_IMPROVEMENT
  { 137, CNU, CNU, CNU, CNU}, 
  { 122, CNU, CNU, CNU, CNU}, 
  { CNU, CNU, CNU, CNU, CNU}, 
#else
  { 137, },
  { 122, },
  { CNU, },
#endif
};

#if VCEG_AZ07_FRUC_MERGE
static const UChar
  INIT_FRUCMGRMODEBIN1[NUMBER_OF_SLICE_TYPES][NUM_FRUCMGRMODE_CTX] = 
{
  { 197,  185,  201 }, 
  { 197,  185,  201 }, 
  { CNU,  CNU,  CNU }, 
};

static const UChar
  INIT_FRUCMGRMODEBIN2[NUMBER_OF_SLICE_TYPES][NUM_FRUCME_CTX] = 
{
  { 197,  185,  201 }, 
  { 197,  185,  201 }, 
  { CNU,  CNU,  CNU }, 
};
#endif

#if VCEG_AZ07_IMV
static const UChar 
  INIT_IMV_FLAG[NUMBER_OF_SLICE_TYPES][NUM_IMV_FLAG_CTX] =
{
  { 197,  185,  201, }, 
  { 197,  185,  201, }, 
  { CNU,  CNU,  CNU, }, 
};
#endif

#if COM16_C806_OBMC
static const UChar 
INIT_OBMC_FLAG[NUMBER_OF_SLICE_TYPES][NUM_OBMC_FLAG_CTX] =  
{
  { 201, }, 
  { 201, }, 
  { CNU, }, 
};
#endif

#if VCEG_AZ06_IC
static const UChar 
INIT_IC_FLAG[NUMBER_OF_SLICE_TYPES][NUM_IC_FLAG_CTX] =  
{
  { 154 },
  { 154 },
  { CNU },
};
#endif

static const UChar
INIT_PART_SIZE[NUMBER_OF_SLICE_TYPES][NUM_PART_SIZE_CTX] =
{
  { 154,  139,  154, 154 },
  { 154,  139,  154, 154 },
  { 184,  CNU,  CNU, CNU },
};

static const UChar
INIT_PRED_MODE[NUMBER_OF_SLICE_TYPES][NUM_PRED_MODE_CTX] =
{
  { 134, },
  { 149, },
  { CNU, },
};

static const UChar
INIT_INTRA_PRED_MODE[NUMBER_OF_SLICE_TYPES][NUM_INTRA_PREDICT_CTX] =
{
#if VCEG_AZ07_INTRA_65ANG_MODES
  { 183, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU }, 
  { 154, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU }, 
  { 184, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU },
#else
  { 183, },
  { 154, },
  { 184, },
#endif
};

static const UChar
INIT_CHROMA_PRED_MODE[NUMBER_OF_SLICE_TYPES][NUM_CHROMA_PRED_CTX] =
{
  { 152,  139, },
  { 152,  139, },
  {  63,  139, },
};

static const UChar
INIT_INTER_DIR[NUMBER_OF_SLICE_TYPES][NUM_INTER_DIR_CTX] =
{
  {  95,   79,   63,   31,  31, },
  {  95,   79,   63,   31,  31, },
  { CNU,  CNU,  CNU,  CNU, CNU, },
};

static const UChar
INIT_MVD[NUMBER_OF_SLICE_TYPES][NUM_MV_RES_CTX] =
{
  { 169,  198, },
  { 140,  198, },
  { CNU,  CNU, },
};

static const UChar
INIT_REF_PIC[NUMBER_OF_SLICE_TYPES][NUM_REF_NO_CTX] =
{
  { 153,  153 },
  { 153,  153 },
  { CNU,  CNU },
};

static const UChar
INIT_DQP[NUMBER_OF_SLICE_TYPES][NUM_DELTA_QP_CTX] =
{
  { 154,  154,  154, },
  { 154,  154,  154, },
  { 154,  154,  154, },
};

static const UChar
INIT_CHROMA_QP_ADJ_FLAG[NUMBER_OF_SLICE_TYPES][NUM_CHROMA_QP_ADJ_FLAG_CTX] =
{
  { 154, },
  { 154, },
  { 154, },
};

static const UChar
INIT_CHROMA_QP_ADJ_IDC[NUMBER_OF_SLICE_TYPES][NUM_CHROMA_QP_ADJ_IDC_CTX] =
{
  { 154, },
  { 154, },
  { 154, },
};

//--------------------------------------------------------------------------------------------------

//Initialisation for CBF

//                                 |---------Luminance---------|
#define BSLICE_LUMA_CBF_CONTEXT     153,  111,  CNU,  CNU,  CNU
#define PSLICE_LUMA_CBF_CONTEXT     153,  111,  CNU,  CNU,  CNU
#define ISLICE_LUMA_CBF_CONTEXT     111,  141,  CNU,  CNU,  CNU
//                                 |--------Chrominance--------|
#define BSLICE_CHROMA_CBF_CONTEXT   149,   92,  167,  154,  154
#define PSLICE_CHROMA_CBF_CONTEXT   149,  107,  167,  154,  154
#define ISLICE_CHROMA_CBF_CONTEXT    94,  138,  182,  154,  154


static const UChar
INIT_QT_CBF[NUMBER_OF_SLICE_TYPES][NUM_QT_CBF_CTX_SETS * NUM_QT_CBF_CTX_PER_SET] =
{
  { BSLICE_LUMA_CBF_CONTEXT, BSLICE_CHROMA_CBF_CONTEXT },
  { PSLICE_LUMA_CBF_CONTEXT, PSLICE_CHROMA_CBF_CONTEXT },
  { ISLICE_LUMA_CBF_CONTEXT, ISLICE_CHROMA_CBF_CONTEXT },
};


//--------------------------------------------------------------------------------------------------

static const UChar
INIT_QT_ROOT_CBF[NUMBER_OF_SLICE_TYPES][NUM_QT_ROOT_CBF_CTX] =
{
  {  79, },
  {  79, },
  { CNU, },
};


//--------------------------------------------------------------------------------------------------

//Initialisation for last-significant-position

//                                           |------------------------------Luminance----------------------------------|
#define BSLICE_LUMA_LAST_POSITION_CONTEXT     125, 110, 124, 110,  95,  94, 125, 111, 111,  79, 125, 126, 111, 111,  79
#define PSLICE_LUMA_LAST_POSITION_CONTEXT     125, 110,  94, 110,  95,  79, 125, 111, 110,  78, 110, 111, 111,  95,  94
#define ISLICE_LUMA_LAST_POSITION_CONTEXT     110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,  79
//                                           |------------------------------Chrominance--------------------------------|
#define BSLICE_CHROMA_LAST_POSITION_CONTEXT   108, 123,  93, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU
#define PSLICE_CHROMA_LAST_POSITION_CONTEXT   108, 123, 108, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU
#define ISLICE_CHROMA_LAST_POSITION_CONTEXT   108, 123,  63, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU


#if COM16_C806_T64
static const UChar 
INIT_LAST[NUMBER_OF_SLICE_TYPES][NUM_CTX_LAST_FLAG_SETS * NUM_CTX_LAST_FLAG_XY] =  
{
  { BSLICE_LUMA_LAST_POSITION_CONTEXT,    126,  111,  111,   79,
    BSLICE_CHROMA_LAST_POSITION_CONTEXT,  CNU,  CNU,  CNU,  CNU,
  }, 
  { PSLICE_LUMA_LAST_POSITION_CONTEXT,    111,  111,   95,   94,
    PSLICE_CHROMA_LAST_POSITION_CONTEXT,  CNU,  CNU,  CNU,  CNU,
  }, 
  { ISLICE_LUMA_LAST_POSITION_CONTEXT,    143,  127,  111,   79, 
    ISLICE_CHROMA_LAST_POSITION_CONTEXT,  CNU,  CNU,  CNU,  CNU,
  }, 
};
#else
static const UChar
INIT_LAST[NUMBER_OF_SLICE_TYPES][NUM_CTX_LAST_FLAG_SETS * NUM_CTX_LAST_FLAG_XY] =
{
  { BSLICE_LUMA_LAST_POSITION_CONTEXT, BSLICE_CHROMA_LAST_POSITION_CONTEXT },
  { PSLICE_LUMA_LAST_POSITION_CONTEXT, PSLICE_CHROMA_LAST_POSITION_CONTEXT },
  { ISLICE_LUMA_LAST_POSITION_CONTEXT, ISLICE_CHROMA_LAST_POSITION_CONTEXT },
};
#endif

//--------------------------------------------------------------------------------------------------

static const UChar
INIT_SIG_CG_FLAG[NUMBER_OF_SLICE_TYPES][2 * NUM_SIG_CG_FLAG_CTX] =
{
  { 121,  140,
    61,  154,
  },
  { 121,  140,
    61,  154,
  },
  {  91,  171,
    134,  141,
  },
};


//--------------------------------------------------------------------------------------------------

//Initialisation for significance map

//                                          |-DC-|  |-----------------4x4------------------|  |------8x8 Diagonal Scan------|  |----8x8 Non-Diagonal Scan----|  |-NxN First group-|  |-NxN Other group-| |-Single context-|
//                                          |    |  |                                      |  |-First Group-| |-Other Group-|  |-First Group-| |-Other Group-|  |                 |  |                 | |                |
#define BSLICE_LUMA_SIGNIFICANCE_CONTEXT     170,    154, 139, 153, 139, 123, 123,  63, 124,   166, 183, 140,  136, 153, 154,   166, 183, 140,  136, 153, 154,   166,   183,   140,   136,   153,   154,        140
#define PSLICE_LUMA_SIGNIFICANCE_CONTEXT     155,    154, 139, 153, 139, 123, 123,  63, 153,   166, 183, 140,  136, 153, 154,   166, 183, 140,  136, 153, 154,   166,   183,   140,   136,   153,   154,        140
#define ISLICE_LUMA_SIGNIFICANCE_CONTEXT     111,    111, 125, 110, 110,  94, 124, 108, 124,   107, 125, 141,  179, 153, 125,   107, 125, 141,  179, 153, 125,   107,   125,   141,   179,   153,   125,        141

//                                          |-DC-|  |-----------------4x4------------------|  |-8x8 Any group-|  |-NxN Any group-| |-Single context-|
#define BSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 138, 138, 122, 121, 122, 121, 167,   151,  183,  140,   151,  183,  140,        140
#define PSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 123, 123, 107, 121, 107, 121, 167,   151,  183,  140,   151,  183,  140,        140
#define ISLICE_CHROMA_SIGNIFICANCE_CONTEXT   140,    139, 182, 182, 152, 136, 152, 136, 153,   136,  139,  111,   136,  139,  111,        111

//------------------------------------------------

static const UChar
INIT_SIG_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SIG_FLAG_CTX] =
{
  { BSLICE_LUMA_SIGNIFICANCE_CONTEXT, BSLICE_CHROMA_SIGNIFICANCE_CONTEXT },
  { PSLICE_LUMA_SIGNIFICANCE_CONTEXT, PSLICE_CHROMA_SIGNIFICANCE_CONTEXT },
  { ISLICE_LUMA_SIGNIFICANCE_CONTEXT, ISLICE_CHROMA_SIGNIFICANCE_CONTEXT },
};


//--------------------------------------------------------------------------------------------------

//Initialisation for greater-than-one flags and greater-than-two flags

//                                 |------Set 0-------| |------Set 1-------| |------Set 2-------| |------Set 3-------|
#define BSLICE_LUMA_ONE_CONTEXT     154, 196, 167, 167,  154, 152, 167, 182,  182, 134, 149, 136,  153, 121, 136, 122
#define PSLICE_LUMA_ONE_CONTEXT     154, 196, 196, 167,  154, 152, 167, 182,  182, 134, 149, 136,  153, 121, 136, 137
#define ISLICE_LUMA_ONE_CONTEXT     140,  92, 137, 138,  140, 152, 138, 139,  153,  74, 149,  92,  139, 107, 122, 152

#define BSLICE_LUMA_ABS_CONTEXT     107,                 167,                  91,                 107
#define PSLICE_LUMA_ABS_CONTEXT     107,                 167,                  91,                 122
#define ISLICE_LUMA_ABS_CONTEXT     138,                 153,                 136,                 167

//                                 |------Set 4-------| |------Set 5-------|
#define BSLICE_CHROMA_ONE_CONTEXT   169, 208, 166, 167,  154, 152, 167, 182
#define PSLICE_CHROMA_ONE_CONTEXT   169, 194, 166, 167,  154, 167, 137, 182
#define ISLICE_CHROMA_ONE_CONTEXT   140, 179, 166, 182,  140, 227, 122, 197

#define BSLICE_CHROMA_ABS_CONTEXT   107,                 167
#define PSLICE_CHROMA_ABS_CONTEXT   107,                 167
#define ISLICE_CHROMA_ABS_CONTEXT   152,                 152


//------------------------------------------------

static const UChar
INIT_ONE_FLAG[NUMBER_OF_SLICE_TYPES][NUM_ONE_FLAG_CTX] =
{
  { BSLICE_LUMA_ONE_CONTEXT, BSLICE_CHROMA_ONE_CONTEXT },
  { PSLICE_LUMA_ONE_CONTEXT, PSLICE_CHROMA_ONE_CONTEXT },
  { ISLICE_LUMA_ONE_CONTEXT, ISLICE_CHROMA_ONE_CONTEXT },
};

static const UChar
INIT_ABS_FLAG[NUMBER_OF_SLICE_TYPES][NUM_ABS_FLAG_CTX] =
{
  { BSLICE_LUMA_ABS_CONTEXT, BSLICE_CHROMA_ABS_CONTEXT },
  { PSLICE_LUMA_ABS_CONTEXT, PSLICE_CHROMA_ABS_CONTEXT },
  { ISLICE_LUMA_ABS_CONTEXT, ISLICE_CHROMA_ABS_CONTEXT },
};


//--------------------------------------------------------------------------------------------------

static const UChar
INIT_MVP_IDX[NUMBER_OF_SLICE_TYPES][NUM_MVP_IDX_CTX] =
{
  { 168, },
  { 168, },
  { CNU, },
};

static const UChar
INIT_SAO_MERGE_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SAO_MERGE_FLAG_CTX] =
{
  { 153,  },
  { 153,  },
  { 153,  },
};

static const UChar
INIT_SAO_TYPE_IDX[NUMBER_OF_SLICE_TYPES][NUM_SAO_TYPE_IDX_CTX] =
{
  { 160, },
  { 185, },
  { 200, },
};

static const UChar
INIT_TRANS_SUBDIV_FLAG[NUMBER_OF_SLICE_TYPES][NUM_TRANS_SUBDIV_FLAG_CTX] =
{
  { 224,  167,  122,
#if COM16_C806_T64
  122
#endif
  },
  { 124,  138,   94,
#if COM16_C806_T64
  94
#endif
   },
  { 153,  138,  138,
#if COM16_C806_T64
  138
#endif
   },
};

static const UChar
INIT_TRANSFORMSKIP_FLAG[NUMBER_OF_SLICE_TYPES][2*NUM_TRANSFORMSKIP_FLAG_CTX] =
{
  { 139,  139},
  { 139,  139},
  { 139,  139},
};

static const UChar
INIT_EXPLICIT_RDPCM_FLAG[NUMBER_OF_SLICE_TYPES][2*NUM_EXPLICIT_RDPCM_FLAG_CTX] =
{
  {139, 139},
  {139, 139},
  {CNU, CNU}
};

static const UChar
INIT_EXPLICIT_RDPCM_DIR[NUMBER_OF_SLICE_TYPES][2*NUM_EXPLICIT_RDPCM_DIR_CTX] =
{
  {139, 139},
  {139, 139},
  {CNU, CNU}
};

static const UChar
INIT_CROSS_COMPONENT_PREDICTION[NUMBER_OF_SLICE_TYPES][NUM_CROSS_COMPONENT_PREDICTION_CTX] =
{
  { 154, 154, 154, 154, 154, 154, 154, 154, 154, 154 },
  { 154, 154, 154, 154, 154, 154, 154, 154, 154, 154 },
  { 154, 154, 154, 154, 154, 154, 154, 154, 154, 154 },
};

#if ALF_HM3_REFACTOR
static const UChar
  INIT_ALF_CTRL_FLAG[NUMBER_OF_SLICE_TYPES][NUM_ALF_CTRL_FLAG_CTX] =
{
  { CNU, CNU, CNU },
  { CNU, CNU, CNU },
  { CNU, CNU, CNU },
};

// initial probability for ALF flag
static const UChar
  INIT_ALF_FLAG[NUMBER_OF_SLICE_TYPES][NUM_ALF_FLAG_CTX] =
{
  {240},
  {224},
  {224}
};

// initial probability for ALF side information (unsigned)
static const UChar
  INIT_ALF_UVLC[NUMBER_OF_SLICE_TYPES][NUM_ALF_UVLC_CTX] =
{
  {154, 140},
  {140, 110},
  {139, 139},
};

// initial probability for ALF side information (signed)
static const UChar
  INIT_ALF_SVLC[NUMBER_OF_SLICE_TYPES][NUM_ALF_SVLC_CTX] =
{
  { 185, 185, CNU },
  { CNU, CNU, CNU },
  { CNU, CNU, CNU },
};
#endif

#if COM16_C806_EMT
static const UChar 
INIT_EMT_TU_IDX[NUMBER_OF_SLICE_TYPES][NUM_EMT_TU_IDX_CTX] =  
{
  { CNU,  CNU, CNU,  CNU }, 
  { CNU,  CNU, CNU,  CNU }, 
  { CNU,  CNU, CNU,  CNU }, 
};

static const UChar 
INIT_EMT_CU_FLAG[NUMBER_OF_SLICE_TYPES][NUM_EMT_CU_FLAG_CTX] = 
{
#if COM16_C806_LARGE_CTU
  { CNU,  CNU, CNU,  CNU, CNU,  CNU },
  { CNU,  CNU, CNU,  CNU, CNU,  CNU },
  { CNU,  CNU, CNU,  CNU, CNU,  CNU }, 
#else
  { CNU,  CNU, CNU,  CNU },
  { CNU,  CNU, CNU,  CNU },
  { CNU,  CNU, CNU,  CNU }, 
#endif
};
#endif

//! \}

#endif
