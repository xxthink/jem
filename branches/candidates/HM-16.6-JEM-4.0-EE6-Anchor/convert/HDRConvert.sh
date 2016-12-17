#!/bin/sh
#
# convert from SDR BT709 to ST2084 BT2100
# Note: HDRTools by Alexis Tourapis v0.13 + an EE6 patch is used in the data conversion 
# https://gitlab.com/standards/HDRTools/
# 
execDir="/nas_home/HDRTools/HDRTools_v0.13_EE6/bin"         # change this to proper directory
exec="$execDir/HDRConvert"
cfgDir="$execDir/CfE_cfgFiles"                              # default config file from HDRTools/bin/CfE_cfgFiles are used
inDir="/nas_share/Sequences"    

seqCfgDir="./per-sequence"
exrDir="./EXR"
outDir="./PQ_BT2100"

seqcfgnames=("A10_Tango" "A11_Drums100" "A12_CampfireParty" "A13_ToddlerFountain" "A20_CatRobot" "A21_TrafficFlow" "A22_DaylightRoad" "A23_Rollercoaster" "S05Kimono" "S06ParkScene" "S07Cactus" "S08BasketballDrive" "S09BQTerrace" "S10BasketballDrill" "S11BQMall" "S12PartyScene" "S13RaceHorsesC" "S14BasketballPass" "S15BQSquare" "S16BlowingBubbles" "S17RaceHorses" "S18FourPeople" "S19Johnny" "S20KristenAndSara" "S21BasketballDrillText" "S22ChinaSpeed" "S23SlideEditing" "S24SlideShow")
 
# ClassA1 (0 - 3): A10_Tango A11_Drums100 A12_CampfireParty A13_ToddlerFountain  
# ClassA2 (4 - 7) A20_CatRobot A21_TrafficFlow A22_DaylightRoad A23_Rollercoaster 
# ClassB (8 -12): S05Kimono S06ParkScene S07Cactus S08BasketballDrive S09BQTerrace 
# ClassC (13-16): S10BasketballDrill S11BQMall S12PartyScene S13RaceHorsesC
# ClassD (17-20): S14BasketballPass S15BQSquare S16BlowingBubbles S17RaceHorses 
# ClassE (21-23): S18FourPeople S19Johnny S20KristenAndSara
# ClassF (24-27): S21BasketballDrillText S22ChinaSpeed S23SlideEditing S24SlideShow

for ((idx=0;idx<=27;idx++)); 
do
    # parse seeunce config
    seqcfg=${seqcfgnames[${idx}]}
    seqcfgfile=$seqCfgDir/$seqcfg.cfg
    
    seq=$(grep SourceFile $seqcfgfile | cut -f2 -d=)
    width=$(grep SourceWidth $seqcfgfile | cut -f2 -d=)
    height=$(grep SourceHeight $seqcfgfile| cut -f2 -d=)
    nframe=$(grep NumberOfFrames $seqcfgfile| cut -f2 -d=)
    echo "$seq width $width, Height $height $nframe"
   
    # input/output
    tmpExr=$exrDir/"$seq"_"%03d".exr
    outseq=PQ2100_"$seq"    
    
    # set correct bitdepth
    if (( $idx < 8 )); then
        srcBitdepth=10  # class A
    else
        srcBitdepth=8
    fi       
    
    # Run YUV to EXR  (Convert from Gamma to linear light )
    $exec -f $cfgDir/HDRConvertYCbCr420ToEXR2020.cfg -f $seqcfgfile -p SourceFile=$inDir/"$seq"  -p OutputFile=$tmpExr -p NumberOfFrames=$nframe
          
    # EXR to YUV with ST2084
    $exec -f $cfgDir/HDRConvertEXR2020ToYCbCr420.cfg -p SourceFile=$tmpExr -p OutputFile=$outDir/$outseq -p SourceWidth=$width -p SourceHeight=$height -p OutputTransferMinBrightness=0.01  -p NumberOfFrames=$nframe
    
done  # loop sequence
