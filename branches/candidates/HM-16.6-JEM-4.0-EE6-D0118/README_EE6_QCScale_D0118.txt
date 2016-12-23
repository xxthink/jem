This code is for testing of the in-loop adaptive scaling method described in D0118.
The code was modifed to be aligned with EE6 HDR Anchor and D0124 codebase with orignal Sharp algorithm implementation and weighted PSNR and RDO.
For conducting simulations, the following macro to be used:
#define QC_EE6_ENABLE_DC_SCALE               1 or 2             
QC_EE6_ENABLE_DC_SCALE               1  tests exclusion of the DC coefficient from adaptive scale derivation and enable adaptive scaling of DC component
QC_EE6_ENABLE_DC_SCALE               2  exclusion of the DC coefficient from adaptive scale derivation only.


The rest of EE6 relevant macros are derived from EE6-Anchor and D0124 branch and remain valid.
These settings are provided below for your information.



*** This luma adaptive QP control code is built on top of  HM-16.6-JEM-4.0-EE6-Anchor *** 

This support luma adaptive coeffiicent scaling (either use DC + prediction to scale AC, or use prediction only to scale all coefficients. 
Comparing to the luma adaptive QP control with explict delta QP signalling supported in HM-16.6-JEM-4.0-EE6-Anchor, methods in this EE6 branch does not signal delta QP.

Below are example config parameters to control the luma adaptive QP and coefficient scaling:

### example: enable luma adaptive coefficient scaling
LumaLevelToDeltaQPMode        : 2                    # Luma activity based deltaQP control  -- 0: disabled;  1: send dQP; 2: coefficient scaling
LumaDQPFile                   : lumaDQP_SDR_6.txt    # File containing luma change points used to derive the luma QP LUT. If file is not exist, use Default luma DQP LUT. 
isSDR                         : 1           # 1 (default): SDR in PQ2100 container,   0: HDR

Note: 
* the per-sequence config files have been modifed to use the converted SDR files, ChromaQpScale etc. parameters.
* For coding SDR in ST-2084 with luma adaptive coefficient scaling, use CTC QP -6 (i.e. 11, 16, 21, 26, 31) and LumaDQPFile=lumaDQP_SDR_6.txt (in cfg directory) to get proper DC quantization. Make sure your path to the LumaDQPFile is correct.
* For HDR test, use CTC QP and default luma DQP LUT defined in the code.

Related Macros:

(1) Code for luma adaptive coefficient scaling are under
    #define SHARP_LUMA_DELTA_QP                1            ///< enable luma adaptive QP
    #define SHARP_LUMA_RES_SCALING             1            ///< enable coefficient scaling based on luma predciton and DC 

    Here the target QP for each block is similar to the adaptive QP approach, But the delta QP is not signalled. Transform coefficients are scaled based on reconstructed luma pixel. 
    Refer JVET-C0095 for details.

There are two default LUT table defined in the code,  parameter isSDR specified in sequence config file is used to select which LUT is used. 
This is also used to get the correct weight to compute the WPSNR inside the code. The LUT table can also be input from a file through command line or config file parameter LumaDQPFile

(3) 
#define SHARP_WEIGHT_DISTORTION              1            ///< use weighted distortion in RD decision
#define SHARP_WEIGHT_DISTORTION_OUTPUT       1            ///< printout weighted PSNR

To make RD decision align with the luma adaptive QP adjustment, JVET-C0095 proposed a weighted PSNR metric.
They are incorporated into encoder RD decision, and are enabled by the above macros.
The weighted PSNR is printed out in the encoder log file as:

WPSNR SUMMARY---------------------------------------------------
	Total Frames |   Bitrate     Y-PSNR    U-PSNR    V-PSNR    YUV-PSNR
        7    w     210.9357   36.6616   40.2324   40.4726   37.5797




  
  
  