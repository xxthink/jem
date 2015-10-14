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
#if QC_AC_ADAPT_WDOW
#define MAX_NUM_CTX_MOD             384   ///< number of context models for last coefficient position
#else
#define MAX_NUM_CTX_MOD             512       ///< maximum number of supported contexts
#endif

#if QC_LARGE_CTU
#define NUM_SPLIT_FLAG_CTX            5       ///< number of context models for split flag
#else
#define NUM_SPLIT_FLAG_CTX            3       ///< number of context models for split flag
#endif
#define NUM_SKIP_FLAG_CTX             3       ///< number of context models for skip flag
#if CU_LEVEL_MPI
 #define NUM_MPI_CTX                  2       /// < number of context models for MPI Idx coding
#endif
#if ROT_TR
 #define NUM_ROT_TR_CTX               7       /// < number of context models for ROT Idx coding
#endif
#if QC_IMV
#define NUM_IMV_FLAG_CTX              3       ///< number of context models for iMV flag
#endif
#if QC_OBMC
#define NUM_OBMC_FLAG_CTX             1       ///< number of context models for OBMC flag
#endif
#define NUM_MERGE_FLAG_EXT_CTX        1       ///< number of context models for merge flag of merge extended
#if GEN_MRG_IMPROVEMENT
#if QC_SUB_PU_TMVP_EXT
#define NUM_MERGE_IDX_EXT_CTX         5       ///< number of context models for merge index of merge extended
#else
#define NUM_MERGE_IDX_EXT_CTX         4       ///< number of context models for merge index of merge extended
#endif
#else
#define NUM_MERGE_IDX_EXT_CTX         1       ///< number of context models for merge index of merge extended
#endif

#if QC_FRUC_MERGE
#define NUM_FRUCMGRMODE_CTX           3
#define NUM_FRUCME_CTX                3
#endif

#if QC_EMT
#define NUM_EMT_TU_IDX_CTX            4
#if QC_LARGE_CTU
#define NUM_EMT_CU_FLAG_CTX           6
#else
#define NUM_EMT_CU_FLAG_CTX           4
#endif
#endif
#if QC_IC
#define NUM_IC_FLAG_CTX               1       ///< number of context models for illumination compensation flag
#endif

#define NUM_PART_SIZE_CTX             4       ///< number of context models for partition size
#define NUM_PRED_MODE_CTX             1       ///< number of context models for prediction mode

#if QC_USE_65ANG_MODES
#define NUM_ADI_CTX                   9       ///< number of context models for intra prediction
#else
#define NUM_ADI_CTX                   1       ///< number of context models for intra prediction
#endif

#define NUM_CHROMA_PRED_CTX           2       ///< number of context models for intra prediction (chroma)
#define NUM_INTER_DIR_CTX             5       ///< number of context models for inter prediction direction
#define NUM_MV_RES_CTX                2       ///< number of context models for motion vector difference

#define NUM_REF_NO_CTX                2       ///< number of context models for reference index
#if QC_T64
#define NUM_TRANS_SUBDIV_FLAG_CTX     4       ///< number of context models for transform subdivision flags
#else
#define NUM_TRANS_SUBDIV_FLAG_CTX     3       ///< number of context models for transform subdivision flags
#endif
#define NUM_QT_CBF_CTX                4       ///< number of context models for QT CBF
#define NUM_QT_ROOT_CBF_CTX           1       ///< number of context models for QT ROOT CBF
#define NUM_DELTA_QP_CTX              3       ///< number of context models for dQP

#define NUM_SIG_CG_FLAG_CTX           2       ///< number of context models for MULTI_LEVEL_SIGNIFICANCE

#if QC_CTX_RESIDUALCODING
#define NUM_SIG_FLAG_CTX              66      ///< number of context models for sig flag
#define NUM_SIG_FLAG_CTX_LUMA         54      ///< number of context models for luma sig flag
#define NUM_SIG_FLAG_CTX_CHROMA       12      ///< number of context models for chroma sig flag
#define NUM_SIG_FLAG_CTX_LUMA_TU      18      ///< number of context models for luma sig flag per TU
#else
#define NUM_SIG_FLAG_CTX              42      ///< number of context models for sig flag
#define NUM_SIG_FLAG_CTX_LUMA         27      ///< number of context models for luma sig flag
#define NUM_SIG_FLAG_CTX_CHROMA       15      ///< number of context models for chroma sig flag
#endif

#if QC_T64
#define NUM_CTX_LAST_FLAG_XY          19      ///< number of context models for last coefficient position
#else
#define NUM_CTX_LAST_FLAG_XY          15      ///< number of context models for last coefficient position
#endif

#if QC_CTX_RESIDUALCODING
#define NUM_ONE_FLAG_CTX              22      ///< number of context models for greater than one
#define NUM_ONE_FLAG_CTX_LUMA         16      ///< number of context models for greater than one of luma
#define NUM_ONE_FLAG_CTX_CHROMA        6      ///< number of context models for greater than one of chroma
#else
#define NUM_ONE_FLAG_CTX              24      ///< number of context models for greater than 1 flag
#define NUM_ONE_FLAG_CTX_LUMA         16      ///< number of context models for greater than 1 flag of luma
#define NUM_ONE_FLAG_CTX_CHROMA        8      ///< number of context models for greater than 1 flag of chroma

#define NUM_ABS_FLAG_CTX               6      ///< number of context models for greater than 2 flag
#define NUM_ABS_FLAG_CTX_LUMA          4      ///< number of context models for greater than 2 flag of luma
#define NUM_ABS_FLAG_CTX_CHROMA        2      ///< number of context models for greater than 2 flag of chroma
#endif

#define NUM_MVP_IDX_CTX               1       ///< number of context models for MVP index

#define NUM_SAO_MERGE_FLAG_CTX        1       ///< number of context models for SAO merge flags
#define NUM_SAO_TYPE_IDX_CTX          1       ///< number of context models for SAO type index

#define NUM_TRANSFORMSKIP_FLAG_CTX    1       ///< number of context models for transform skipping 
#if KLT_COMMON
#define NUM_KLT_FLAG_CTX              1       ///< number of context models for KLT 
#endif
#define NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX  1 

#if ALF_HM3_QC_REFACTOR
#define NUM_ALF_CTRL_FLAG_CTX         3       ///< number of context models for ALF control flag
#define NUM_ALF_FLAG_CTX              1       ///< number of context models for ALF flag
#define NUM_ALF_UVLC_CTX              2       ///< number of context models for ALF UVLC (filter length)
#define NUM_ALF_SVLC_CTX              3       ///< number of context models for ALF SVLC (filter coeff.)
#endif
#if QC_AC_ADAPT_WDOW
#define NUM_QP_PROB                  5
#if !ALF_HM3_QC_REFACTOR
#define NUM_ALF_CTX                  0
#else
#define NUM_ALF_CTX                   (NUM_ALF_CTRL_FLAG_CTX+NUM_ALF_FLAG_CTX+NUM_ALF_UVLC_CTX+NUM_ALF_SVLC_CTX)
#endif
#define NUM_CTX_PBSLICE              MAX_NUM_CTX_MOD //could be set to the exact number of used contexts later
#endif
#define CNU                          154      ///< dummy initialization value for unused context models 'Context model Not Used'

// ====================================================================================================================
// Tables
// ====================================================================================================================

// initial probability for cu_transquant_bypass flag
static const UChar
INIT_CU_TRANSQUANT_BYPASS_FLAG[3][NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX] =
{
  { 154 }, 
  { 154 }, 
  { 154 }, 
};

// initial probability for split flag
static const UChar 
#if QC_LARGE_CTU
  INIT_SPLIT_FLAG[3][NUM_SPLIT_FLAG_CTX] =  
{
  { 107,  139,  126, 255, 0, },
  { 107,  139,  126, 255, 0, }, 
  { 139,  141,  157, 255, 0, }, 
};
#else
INIT_SPLIT_FLAG[3][NUM_SPLIT_FLAG_CTX] =  
{
  { 107,  139,  126, },
  { 107,  139,  126, }, 
  { 139,  141,  157, }, 
};
#endif

static const UChar 
INIT_SKIP_FLAG[3][NUM_SKIP_FLAG_CTX] =  
{
  { 197,  185,  201, }, 
  { 197,  185,  201, }, 
  { CNU,  CNU,  CNU, }, 
};
#if CU_LEVEL_MPI
static const UChar 
INIT_MPIIdx_FLAG[3][NUM_MPI_CTX] =  
{
  { 107,107 }, 
  { 107,107 }, 
  { 139,139 }, 
};
#endif
#if ROT_TR
static const UChar 
INIT_ROT_TR_IDX[3][NUM_ROT_TR_CTX] =  
{
  { 107,107,107,107,107,107,107 }, 
  { 107,107,107,107,107,107,107 }, 
  { 139,139,139,139,139,139,139 }, 
};
#endif

#if KLT_COMMON
static const UChar
INIT_KLT_FLAG[3][2 * NUM_KLT_FLAG_CTX] =
{
  { 139, 139 },
  { 139, 139 },
  { 139, 139 },
};
#endif

#if QC_IMV
static const UChar 
  INIT_IMV_FLAG[3][NUM_IMV_FLAG_CTX] =
{
  { 197,  185,  201, }, 
  { 197,  185,  201, }, 
  { CNU,  CNU,  CNU, }, 
};
#endif

#if QC_OBMC
static const UChar 
INIT_OBMC_FLAG[3][NUM_OBMC_FLAG_CTX] =  
{
  { 201, }, 
  { 201, }, 
  { CNU, }, 
};
#endif

#if QC_IC
static const UChar 
INIT_IC_FLAG[3][NUM_IC_FLAG_CTX] =  
{
  { 154 },
  { 154 },
  { CNU },
};
#endif

static const UChar
INIT_MERGE_FLAG_EXT[3][NUM_MERGE_FLAG_EXT_CTX] = 
{
  { 154, }, 
  { 110, }, 
  { CNU, }, 
};

static const UChar 
INIT_MERGE_IDX_EXT[3][NUM_MERGE_IDX_EXT_CTX] =  
{
#if GEN_MRG_IMPROVEMENT
#if QC_SUB_PU_TMVP_EXT
  { 137, CNU, CNU, CNU, CNU}, 
  { 122, CNU, CNU, CNU, CNU}, 
  { CNU, CNU, CNU, CNU, CNU}, 
#else
  { 137, CNU, CNU, CNU}, 
  { 122, CNU, CNU, CNU}, 
  { CNU, CNU, CNU, CNU}, 
#endif 
#else
  { 137, }, 
  { 122, }, 
  { CNU, }, 
#endif
};

#if QC_FRUC_MERGE
static const UChar
  INIT_FRUCMGRMODEBIN1[3][NUM_FRUCMGRMODE_CTX] = 
{
  { 197,  185,  201 }, 
  { 197,  185,  201 }, 
  { CNU,  CNU,  CNU }, 
};

static const UChar
  INIT_FRUCMGRMODEBIN2[3][NUM_FRUCME_CTX] = 
{
  { 197,  185,  201 }, 
  { 197,  185,  201 }, 
  { CNU,  CNU,  CNU }, 
};
#endif

#if QC_EMT
static const UChar 
INIT_EMT_TU_IDX[3][NUM_EMT_TU_IDX_CTX] =  
{
  { CNU,  CNU, CNU,  CNU },
  { CNU,  CNU, CNU,  CNU },
  { CNU,  CNU, CNU,  CNU }, 
};

static const UChar 
INIT_EMT_CU_FLAG[3][NUM_EMT_CU_FLAG_CTX] = 
{
#if QC_LARGE_CTU
  { CNU,  CNU, CNU,  CNU,  CNU,  CNU },
  { CNU,  CNU, CNU,  CNU,  CNU,  CNU },
  { CNU,  CNU, CNU,  CNU,  CNU,  CNU }, 
#else
  { CNU,  CNU, CNU,  CNU },
  { CNU,  CNU, CNU,  CNU },
  { CNU,  CNU, CNU,  CNU }, 
#endif
};
#endif

static const UChar 
INIT_PART_SIZE[3][NUM_PART_SIZE_CTX] =  
{
  { 154,  139,  154,  154 },
  { 154,  139,  154,  154 },
  { 184,  CNU,  CNU,  CNU },
};

static const UChar
INIT_PRED_MODE[3][NUM_PRED_MODE_CTX] = 
{
  { 134, }, 
  { 149, }, 
  { CNU, }, 
};

static const UChar 
INIT_INTRA_PRED_MODE[3][NUM_ADI_CTX] = 
{
#if QC_USE_65ANG_MODES
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
INIT_CHROMA_PRED_MODE[3][NUM_CHROMA_PRED_CTX] = 
{
  { 152,  139, }, 
  { 152,  139, }, 
  {  63,  139, }, 
};

static const UChar 
INIT_INTER_DIR[3][NUM_INTER_DIR_CTX] = 
{
  {  95,   79,   63,   31,  31, }, 
  {  95,   79,   63,   31,  31, }, 
  { CNU,  CNU,  CNU,  CNU, CNU, }, 
};

static const UChar 
INIT_MVD[3][NUM_MV_RES_CTX] =  
{
  { 169,  198, }, 
  { 140,  198, }, 
  { CNU,  CNU, }, 
};

static const UChar 
INIT_REF_PIC[3][NUM_REF_NO_CTX] =  
{
  { 153,  153 }, 
  { 153,  153 }, 
  { CNU,  CNU }, 
};

static const UChar 
INIT_DQP[3][NUM_DELTA_QP_CTX] = 
{
  { 154,  154,  154, }, 
  { 154,  154,  154, }, 
  { 154,  154,  154, }, 
};

static const UChar 
INIT_QT_CBF[3][2*NUM_QT_CBF_CTX] =  
{
  { 153,  111,  CNU,  CNU,   149,   92,  167,  154 },
  { 153,  111,  CNU,  CNU,   149,  107,  167,  154 },
  { 111,  141,  CNU,  CNU,    94,  138,  182,  154 },
};

static const UChar 
INIT_QT_ROOT_CBF[3][NUM_QT_ROOT_CBF_CTX] = 
{
  {  79, }, 
  {  79, }, 
  { CNU, }, 
};

#if QC_T64
static const UChar 
INIT_LAST[3][2*NUM_CTX_LAST_FLAG_XY] =  
{
  { 125,  110,  124,  110,   95,   94,  125,  111,  111,   79,  125,  126,  111,  111,   79,  126,  111,  111,   79,
    108,  123,   93,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,
  }, 
  { 125,  110,   94,  110,   95,   79,  125,  111,  110,   78,  110,  111,  111,   95,   94,  111,  111,   95,   94,
    108,  123,  108,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,
  }, 
  { 110,  110,  124,  125,  140,  153,  125,  127,  140,  109,  111,  143,  127,  111,   79,  143,  127,  111,   79, 
    108,  123,   63,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,
  }, 
};
#else
static const UChar 
INIT_LAST[3][2*NUM_CTX_LAST_FLAG_XY] =  
{
#if QC_CTX_RESIDUALCODING
  {
    110, 110,  94, 110, 140, 140, 111, 126, 126, 125, 126, 127, 143, 126, 125,
    108, 108,  62, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU,    
  },
  {
    111, 125, 124, 111, 111, 111, 111, 126, 126, 110, 111, 141, 127, 111, 125,
    109,  94,  63, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU,
    
  },
  {
    125,  95, 109, 110, 125, 110, 125, 125, 110, 110, 154, 140, 140, 111, 111,
    123,  93,  77, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU,    
  },
#else
  { 125,  110,  124,  110,   95,   94,  125,  111,  111,   79,  125,  126,  111,  111,   79,
    108,  123,   93,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, 
  }, 
  { 125,  110,   94,  110,   95,   79,  125,  111,  110,   78,  110,  111,  111,   95,   94,
    108,  123,  108,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,
  }, 
  { 110,  110,  124,  125,  140,  153,  125,  127,  140,  109,  111,  143,  127,  111,   79, 
    108,  123,   63,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, 
  }, 
#endif
};
#endif

static const UChar 
INIT_SIG_CG_FLAG[3][2 * NUM_SIG_CG_FLAG_CTX] =  
{
#if QC_CTX_RESIDUALCODING
  {
    122, 143,
    91, 141,
  },
  {
    78, 111,
    60, 140,
  },
  {
    135, 155,
    104, 139,
  },
#else
  { 121,  140,  
    61,  154, 
  }, 
  { 121,  140, 
    61,  154, 
  }, 
  {  91,  171,  
    134,  141, 
  }, 
#endif
};

#if QC_CTX_RESIDUALCODING
static const UChar 
INIT_SIG_FLAG[3][NUM_SIG_FLAG_CTX] = 
{
  {
    107, 139, 154, 140, 140, 141, 108, 154, 125, 155, 126, 127, 139, 155, 155, 141, 156, 143, //4x4
    107, 139, 154, 140, 140, 141, 108, 154, 125, 155, 126, 127, 139, 155, 155, 141, 156, 143, //8x8 
    107, 139, 154, 140, 140, 141, 108, 154, 125, 155, 126, 127, 139, 155, 155, 141, 156, 143, //16x16
    137, 154, 154, 155, 155, 156, 124, 185, 156, 171, 142, 158,
  },
  {
    121, 167, 153, 139, 154, 140, 137, 168, 139, 154, 169, 155, 167, 169, 169, 184, 199, 156, 
    121, 167, 153, 139, 154, 140, 137, 168, 139, 154, 169, 155, 167, 169, 169, 184, 199, 156,
    121, 167, 153, 139, 154, 140, 137, 168, 139, 154, 169, 155, 167, 169, 169, 184, 199, 156,

    136, 153, 139, 154, 125, 140, 122, 154, 184, 185, 171, 157,
  },
  {
    152, 139, 154, 154, 169, 155, 182, 154, 169, 184, 155, 141, 168, 214, 199, 170, 170, 171, 
    152, 139, 154, 154, 169, 155, 182, 154, 169, 184, 155, 141, 168, 214, 199, 170, 170, 171, 
    152, 139, 154, 154, 169, 155, 182, 154, 169, 184, 155, 141, 168, 214, 199, 170, 170, 171, 

    167, 154, 169, 140, 155, 141, 153, 171, 185, 156, 171, 172,
  },
};
static const UChar 
INIT_ONE_FLAG[3][NUM_ONE_FLAG_CTX] = 
{
  {
    121, 135, 123, 124, 139, 125,  92, 124, 154, 125, 155, 138, 169, 155, 170, 156, 166, 152, 140, 170, 171, 157,
  },
  {
    165,  75, 152, 153, 139, 154, 121, 138, 139, 154, 140, 167, 183, 169, 170, 156, 193, 181, 169, 170, 171, 172,
  },
  {
    196, 105, 152, 153, 139, 154, 136, 138, 139, 169, 140, 196, 183, 169, 170, 171, 195, 181, 169, 170, 156, 157,
  },
};
#else
static const UChar 
INIT_SIG_FLAG[3][NUM_SIG_FLAG_CTX] = 
{
  { 170,  154,  139,  153,  139,  123,  123,   63,  124,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  170,  153,  138,  138,  122,  121,  122,  121,  167,  151,  183,  140,  151,  183,  140,  }, 
  { 155,  154,  139,  153,  139,  123,  123,   63,  153,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  170,  153,  123,  123,  107,  121,  107,  121,  167,  151,  183,  140,  151,  183,  140,  }, 
  { 111,  111,  125,  110,  110,   94,  124,  108,  124,  107,  125,  141,  179,  153,  125,  107,  125,  141,  179,  153,  125,  107,  125,  141,  179,  153,  125,  140,  139,  182,  182,  152,  136,  152,  136,  153,  136,  139,  111,  136,  139,  111,  }, 
};

static const UChar 
INIT_ONE_FLAG[3][NUM_ONE_FLAG_CTX] = 
{
  { 154,  196,  167,  167,  154,  152,  167,  182,  182,  134,  149,  136,  153,  121,  136,  122,  169,  208,  166,  167,  154,  152,  167,  182, }, 
  { 154,  196,  196,  167,  154,  152,  167,  182,  182,  134,  149,  136,  153,  121,  136,  137,  169,  194,  166,  167,  154,  167,  137,  182, }, 
  { 140,   92,  137,  138,  140,  152,  138,  139,  153,   74,  149,   92,  139,  107,  122,  152,  140,  179,  166,  182,  140,  227,  122,  197, }, 
};

static const UChar 
INIT_ABS_FLAG[3][NUM_ABS_FLAG_CTX] =  
{
  { 107,  167,   91,  107,  107,  167, }, 
  { 107,  167,   91,  122,  107,  167, }, 
  { 138,  153,  136,  167,  152,  152, }, 
};
#endif
static const UChar 
INIT_MVP_IDX[3][NUM_MVP_IDX_CTX] =  
{
  { 168 },
  { 168 },
  { CNU }, 
};

static const UChar 
INIT_SAO_MERGE_FLAG[3][NUM_SAO_MERGE_FLAG_CTX] = 
{
  { 153,  }, 
  { 153,  }, 
  { 153,  }, 
};

static const UChar 
INIT_SAO_TYPE_IDX[3][NUM_SAO_TYPE_IDX_CTX] = 
{
  { 160, },
  { 185, },
  { 200, },
};

static const UChar
INIT_TRANS_SUBDIV_FLAG[3][NUM_TRANS_SUBDIV_FLAG_CTX] =
{
#if QC_T64
  { 224,  167,  122, 122 },
  { 124,  138,   94,  94 },
  { 153,  138,  138, 138 },
#else
  { 224,  167,  122, },
  { 124,  138,   94, },
  { 153,  138,  138, },
#endif
};

static const UChar
INIT_TRANSFORMSKIP_FLAG[3][2*NUM_TRANSFORMSKIP_FLAG_CTX] = 
{
  { 139,  139}, 
  { 139,  139}, 
  { 139,  139}, 
};

#if ALF_HM3_QC_REFACTOR
static const UChar
INIT_ALF_CTRL_FLAG[3][NUM_ALF_CTRL_FLAG_CTX] =
{
  { CNU, CNU, CNU },
  { CNU, CNU, CNU },
  { CNU, CNU, CNU },
};

// initial probability for ALF flag
static const UChar
INIT_ALF_FLAG[3][NUM_ALF_FLAG_CTX] =
{
  {240},
  {224},
  {224}
};

// initial probability for ALF side information (unsigned)
static const UChar
INIT_ALF_UVLC[3][NUM_ALF_UVLC_CTX] =
{
  {154, 140},
  {140, 110},
  {139, 139},
};

// initial probability for ALF side information (signed)
static const UChar
INIT_ALF_SVLC[3][NUM_ALF_SVLC_CTX] =
{
  { 185, 185, CNU },
  { CNU, CNU, CNU },
  { CNU, CNU, CNU },
};
#endif
//! \}


#endif
