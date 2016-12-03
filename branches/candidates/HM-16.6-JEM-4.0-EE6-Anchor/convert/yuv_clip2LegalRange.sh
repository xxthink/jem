#!/bin/sh
#
# Make sure YUVs noted as broadcast legal range are indeed within legal range. If not, clip them. 
# This step can be omitted if later HDRTools can have the clipping step inside HDRTools
#
ffmpeg_exe=/nas_share/Jane/ffmpeg-2.8.2-64bit-static/ffmpeg           # change this to your directory

inDir="/nas_share/Sequences/"
outDir="."

# ClassA1 (0 - 3): A10_Tango A11_Drums100 A12_CampfireParty A13_ToddlerFountain  
# ClassA2 (4 - 7) A20_CatRobot A21_TrafficFlow A22_DaylightRoad A23_Rollercoaster 
# ClassB (8 -12): S05Kimono S06ParkScene S07Cactus S08BasketballDrive S09BQTerrace 
# ClassC (13-16): S10BasketballDrill S11BQMall S12PartyScene S13RaceHorsesC
# ClassD (17-20): S14BasketballPass S15BQSquare S16BlowingBubbles S17RaceHorses 
# ClassE (21-23): S18FourPeople S19Johnny S20KristenAndSara
# ClassF (24-27): S21BasketballDrillText S22ChinaSpeed S23SlideEditing S24SlideShow
inseqnames=("Tango_4096x2160_60fps_10bit_420_jvet" "Drums_3840x2160_100fps_10bit_420_jvet" "CampfireParty_3840x2160_30fps_10bit_420_jvet"  "ToddlerFountain_4096x2160_60fps_10bit_420_jvet" "CatRobot_3840x2160_60fps_10bit_420_jvet"  "TrafficFlow_3840x2160_30fps_10bit_420_jvet" "DaylightRoad_3840x2160_60fps_10bit_420_jvet"   "RollerCoaster_4096x2160_60fps_10bit_420_jvet"   "Kimono1_1920x1080_24"  "ParkScene_1920x1080_24" "Cactus_1920x1080_50" "BasketballDrive_1920x1080_50" "BQTerrace_1920x1080_60" "BasketballDrill_832x480_50" "BQMall_832x480_60" "PartyScene_832x480_50" "RaceHorses_832x480_30" "BasketballPass_416x240_50" "BQSquare_416x240_60" "BlowingBubbles_416x240_50" "RaceHorses_416x240_30" "FourPeople_1280x720_60" "Johnny_1280x720_60" "KristenAndSara_1280x720_60" "BasketballDrillText_832x480_50" "ChinaSpeed_1024x768_30" "SlideEditing_1280x720_30" "SlideShow_1280x720_20" )

# Idx     0     1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23  24   25   26    27
#        A10   A11  A12  A13  A20  A21  A22  A23  S05  S06  S07  S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20  S21  S22  S23  S24
nframes=(294   300  300  300  300  300  300  300  240  240  500  500  600  500  600  500  300  500  600  500  300  600  600  600  500  500  300  500)
widths=(4096  3840 3840 4096 3840 3840 3840 4096 1920 1920 1920 1920 1920 832  832  832  832  416  416  416  416  1280 1280 1280 832  1024 1280 1280)
heights=(2160 2160 2160 2160 2160 2160 2160 2160 1080 1080 1080 1080 1080 480  480  480  480  240  240  240  240  720  720  720  480  768  720  720)

# 0: standard/limited video; 1: full range
#             A10 A11 A12 A13 A20 A21 A22 A23 S05 S06 S07 S08 S09 S10 S11 S12 S13 S14 S15 S16 S17 S18 S19 S20 S21 S22 S23 S24
sampleRanges=( 0   0   0   0   0   0   0   0   0   0   1   0   0   0   0   0   0   0   0   0   0   0    0   0   1   1  0  1 );  

for ((idx=0;idx<=27;idx++));
do
  width=${widths[${idx}]}
  height=${heights[${idx}]}
  seqName=${inseqnames[${idx}]} 
  range=${sampleRanges[${idx}]}  
  resolution="$width"x"$height"
  
  if (( $range==0 )); then
    # clip YUV to legal range
    #
    if (( $idx < 8 )); then
        # srcBitdepth=10  # class A
        $ffmpeg_exe  -s "$resolution" -pix_fmt yuv420p10le -i $inDir/"$seqName".yuv -vcodec rawvideo -vf lut=y=clipval:u=clipval:v=clipval $outDir/"$seqName".yuv  

    else
      # srcBitdepth=8
      $ffmpeg_exe  -s "$resolution" -pix_fmt yuv420p -i $inDir/"$seqName".yuv -vcodec rawvideo -vf lut=y=clipval:u=clipval:v=clipval $outDir/"$seqName".yuv  
    fi   
  fi
  
done
