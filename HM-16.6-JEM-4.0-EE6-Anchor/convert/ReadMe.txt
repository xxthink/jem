This directory provides scripts and configurations files that may be useful for creating content described in the "JVET Common Test Conditions and Evaluation Procedures for HDR/Video" (JVET-D1020).  Note that the information in this directory may be updated in the future, and any updates will be announced on the JVET e-mail reflector.

Files:

+ HDRToolsV0.13-EE6-changes.patch : Patch that maybe applied to the HDRTools v0.13 to enable creation of the Table 1 data.  The patch has two additions: (i) for content denoted as conforming to the broadcast legal range, the sample values are clipped to this range before conversion to PQ BT.2100 and (ii) content may now be mapped to a range with a non-zero minimum value, e.g. 0.01cd/m2 to 100 cd/m2.

+ md5sum.txt: MD5 sums for the test sequences

+ per-sequence/*.cfg : Per-sequence configuration files that may be used in combination with HDRTools v0.13 after patching with HDRToolsV0.13-EE6-changes.patch.  The configuration files contain sequence specific information, including if the input sequence corresponds to broadcast legal or full range data.

+ HDRConvert.sh: Example shell script for creating the PQ BT.2100 content listed in Table A.

To create the Table A data, please do the following:
1.  Obtain HDRTools and the HDRConvert.sh script
2.  Apply the HDRToolsV0.13-EE6-changes.patch
3.  Update the ‘execDir’ and ‘inDir’ variables in the HDRConvert.sh script
4.  Run the HDRConvert.sh script
