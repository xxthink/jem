#!/bin/sh
#
# convert from SDR BT709 to ST2084
# Jane Zhao @ sharplabs
#
# Note: HDRTools by Alexis Tourapis v0.10.1 is used in our test 
# https://gitlab.com/standards/HDRTools/tags/v0.10_1
# Note: HDRTools v0.12 has bug doing this conversion. 
# 
exec="/nas_share/Jane/HDRTools/HDRTools_Git/bin/HDRConvert"   
inDir="/nas_share/Sequences"
exrDir="./EXR"
outDir="./PQ_BT709"

# Idx     0     1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23  24   25   26    27
#        A10   A11  A12  A13  A20  A21  A22  A23  S05  S06  S07  S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20  S21  S22  S23  S24
nframes=(294   300  300  300  300  300  300  300  240  240  500  500  600  500  600  500  300  500  600  500  300  600  600  600  500  500  300  500)
widths=(4096  3840 3840 4096 3840 3840 3840 4096 1920 1920 1920 1920 1920 832  832  832  832  416  416  416  416  1280 1280 1280 832  1024 1280 1280)
heights=(2160 2160 2160 2160 2160 2160 2160 2160 1080 1080 1080 1080 1080 480  480  480  480  240  240  240  240  720  720  720  480  768  720  720)
  
# ClassA1: A10Tango  A11Drums A12CampfireParty A13ToddlerFountain
# classA2: A20CatRobot  A21TrafficFlow  A22DaylightRoad A23RollerCoaster  
# ClassB: S05Kimono S06ParkScene S07Cactus S08BasketballDrive S09BQTerrace 
# ClassC: S10BasketballDrill S11BQMall S12PartyScene S13RaceHorsesC
# ClassD: S14BasketballPass S15BQSquare S16BlowingBubbles S17RaceHorses 
# ClassE: S18FourPeople S19Johnny S20KristenAndSara
# ClassF: S21BasketballDrillText S22ChinaSpeed S23SlideEditing S24SlideShow

inseqnames=("Tango_4096x2160_60fps_10bit_420_jvet" "Drums_3840x2160_100fps_10bit_420_jvet" "CampfireParty_3840x2160_30fps_10bit_420_jvet"  "ToddlerFountain_4096x2160_60fps_10bit_420_jvet" "CatRobot_3840x2160_60fps_10bit_420_jvet"  "TrafficFlow_3840x2160_30fps_10bit_420_jvet" "DaylightRoad_3840x2160_60fps_10bit_420_jvet"   "RollerCoaster_4096x2160_60fps_10bit_420_jvet"   "Kimono1_1920x1080_24"  "ParkScene_1920x1080_24" "Cactus_1920x1080_50" "BasketballDrive_1920x1080_50" "BQTerrace_1920x1080_60" "BasketballDrill_832x480_50" "BQMall_832x480_60" "PartyScene_832x480_50" "RaceHorses_832x480_30" "BasketballPass_416x240_50" "BQSquare_416x240_60" "BlowingBubbles_416x240_50" "RaceHorses_416x240_30" "FourPeople_1280x720_60" "Johnny_1280x720_60" "KristenAndSara_1280x720_60" "BasketballDrillText_832x480_50" "ChinaSpeed_1024x768_30" "SlideEditing_1280x720_30" "SlideShow_1280x720_20" )

for ((idx=0;idx<=27;idx++)); 
do

    seq=${inseqnames[${idx}]}
    width=${widths[${idx}]}
    height=${heights[${idx}]}
    nframe=${nframes[${idx}]}
    
    # input
    tmpExr=$exrDir/"$seq"_"%03d".exr 
    outseq="$seq"_PQ_BT709.yuv


    # set correct bitdepth
    if (( $idx < 8 )); then
        srcBitdepth=10  # class A
    else
        srcBitdepth=8
    fi       
    
    # Run YUV to EXR  (Convert from Gamma to linear light )
    $exec -f HDRConvertYCbCr420ToEXR709.cfg -p SourceFile=$inDir/"$seq".yuv  -p OutputFile=$tmpExr  -p SourceWidth=$width -p SourceHeight=$height -p NumberOfFrames=$nframe -p SourceBitDepthCmp0=$srcBitdepth -p SourceBitDepthCmp1=$srcBitdepth -p SourceBitDepthCmp2=$srcBitdepth -p SourceTransferFunction=6  -p SourceNormalizationScale=100  -p SourceSystemGamma=2.4 -p SourceSampleRange=1
          
    # EXR to YUV with ST2084
    $exec -f HDRConvertEXR709ToYCbCr420.cfg -p SourceFile=$tmpExr -p OutputFile=$outDir/$outseq -p SourceWidth=$width -p SourceHeight=$height -p NumberOfFrames=$nframe -p OutputSampleRange=1              
    
    echo ""
done  # loop sequence
