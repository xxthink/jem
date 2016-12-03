For content denoted as conforming to the broadcast legal range, the content should first be clipped to the broadcast legal range value.

Currently it is not sure if HDRTools v0.13 has done the clipping already, so we do it in an extra step using ffmpeg first, refer yuv_clip2LegalRange.sh
Then the YUV files are converted to PQ2020 using HDRTools v0.13, refer HDRConvert_to_PQ2020.sh

# md5sum of the converted YUV in PQ2020 are 
#
01a7a06f98150ae694dd512ec4ded7e3  PQ2020_BasketballDrill_832x480_50.yuv
a788cbf935f572315dc4482bada2ceae  PQ2020_BasketballDrillText_832x480_50.yuv                    # full range
ff20d31683beefa97a838e84646333d0  PQ2020_BasketballDrive_1920x1080_50.yuv
4ca351d00bd7af35bacf252771cdcea3  PQ2020_BasketballPass_416x240_50.yuv
280d6fff1e516e98b51e065fe32d7982  PQ2020_BlowingBubbles_416x240_50.yuv
f03aeb8db54e692be45feb35c5ae6b2e  PQ2020_BQMall_832x480_60.yuv
0e8d354d495490b16335904e9f9af3bf  PQ2020_BQSquare_416x240_60.yuv
296bbb81306379537e25320785e65a58  PQ2020_BQTerrace_1920x1080_60.yuv
30eb4f63030b32df73e21d5f3d243304  PQ2020_Cactus_1920x1080_50.yuv                               # full range
a9e9e2f6b5591ab883b3963e0dcf4947  PQ2020_CampfireParty_3840x2160_30fps_10bit_420_jvet.yuv
e0904202b71aaf6dbe7f0091caf37773  PQ2020_CatRobot_3840x2160_60fps_10bit_420_jvet.yuv
ee7bf37b8888e86cb760a95fb465e73f  PQ2020_ChinaSpeed_1024x768_30.yuv                            # full range
f02c43f22fbdaf3269c8941ce0ecb2ee  PQ2020_DaylightRoad_3840x2160_60fps_10bit_420_jvet.yuv
9f671887836a65a1596e75b0d23d58a8  PQ2020_Drums_3840x2160_100fps_10bit_420_jvet.yuv
9ed031930fedf54e679565e952c50cac  PQ2020_FourPeople_1280x720_60.yuv
c8e6048e854723e9ab8033c44ee8c4d6  PQ2020_Johnny_1280x720_60.yuv
2503d8c43d96036aabdb31f648024e67  PQ2020_Kimono1_1920x1080_24.yuv
3b761bd58e4ac4c446978484a9b6cd9a  PQ2020_KristenAndSara_1280x720_60.yuv
6e854935f990f56530d59e890049c182  PQ2020_ParkScene_1920x1080_24.yuv
bf576940633155335f9a46069da68e81  PQ2020_PartyScene_832x480_50.yuv
0a746bae1819604e05780cf3925e1bd9  PQ2020_RaceHorses_416x240_30.yuv
bf7a2d32c927e4419e52a99cd7f0c4bd  PQ2020_RaceHorses_832x480_30.yuv
469dd72823cd909490df45bd3f43246b  PQ2020_RollerCoaster_4096x2160_60fps_10bit_420_jvet.yuv
6bc4e2a684e6fb0ea712ec6ff0f3139c  PQ2020_SlideEditing_1280x720_30.yuv
b0483d43c5bb30424c8b2b45205765dc  PQ2020_SlideShow_1280x720_20.yuv                              # full range
c7db45f4b8bafcb6bfa8f36f9573f58f  PQ2020_Tango_4096x2160_60fps_10bit_420_jvet.yuv
c001f28f1c6c61e1b304b15c0c7e8476  PQ2020_ToddlerFountain_4096x2160_60fps_10bit_420_jvet.yuv
933792754e37e7dc8692078a29321986  PQ2020_TrafficFlow_3840x2160_30fps_10bit_420_jvet.yuv



# if use HDRTools v0.13 to do conversion directly without the clipping the Legal range sequences first
# Among the converted PQ2020 sequences, only the following three sequences have different md5sum . 
# The rest are identical to the above with extra ffmpeg clipping step.
#
3ae81c1e8e6012a09d4804748efccdc4  PQ2020_CampfireParty_3840x2160_30fps_10bit_420_jvet.yuv         *** 
614d666a0639219f1a489d7ac02280f3  PQ2020_CatRobot_3840x2160_60fps_10bit_420_jvet.yuv              *** 
b13c68b946cad36a29f933c588ad76a9  PQ2020_DaylightRoad_3840x2160_60fps_10bit_420_jvet.yuv          ***


# md5sum of the intermediate cliiped Standard Range YUVs by ffmpeg are
#
bd215136fed04067d82c10b2e49b2c7c  BasketballDrill_832x480_50.yuv
141adbdde68ed886b68771ccf58b9138  BasketballDrive_1920x1080_50.yuv
bfd9abbdc677790130dc4023b4e409f0  BasketballPass_416x240_50.yuv
50a520722f0e906b7884b6b9fea48699  BlowingBubbles_416x240_50.yuv
f889efea02b0c9a7d174b0f7a99cb51b  BQMall_832x480_60.yuv
713ef64958345859b9bae986c3a3f763  BQSquare_416x240_60.yuv
f5b69197a694bf4dd3b722110c5be2e4  BQTerrace_1920x1080_60.yuv
422970ea48907f0384e6e566fcedd678  Cactus_1920x1080_50.yuv
843c2557d930d5c39d6aed529732195c  CampfireParty_3840x2160_30fps_10bit_420_jvet.yuv
479dcc8fe3ee43571aad09afda4c5009  CatRobot_3840x2160_60fps_10bit_420_jvet.yuv
7b0111bd859907a545cdee650b7f5f32  DaylightRoad_3840x2160_60fps_10bit_420_jvet.yuv
f1d760282da8baf851693436f63da31d  Drums_3840x2160_100fps_10bit_420_jvet.yuv
d36a92fb7265a612920d90eb7e001f32  FourPeople_1280x720_60.yuv
7a30ffc20460658a20dea05952195d4f  Johnny_1280x720_60.yuv
98e71a6564536bef4331e5d91f8ebff6  Kimono1_1920x1080_24.yuv
29490d5a7ab8c3d876f3f149564d1e7d  KristenAndSara_1280x720_60.yuv
80ef1e0e9cbd4867261450c67d3e36cb  ParkScene_1920x1080_24.yuv
4766c455665b6d228a6390e3d3ff2647  PartyScene_832x480_50.yuv
290a63e86213abc4459fce1dbd39edbe  RaceHorses_416x240_30.yuv
0a351df99f22d837bc528bd4901c6968  RaceHorses_832x480_30.yuv
0f55a84c7c190ef79263fbf453e620d0  RollerCoaster_4096x2160_60fps_10bit_420_jvet.yuv
dbf1d0e765ba9f775c5208adb4dc6c0c  SlideEditing_1280x720_30.yuv
c5bc3a0943a067c5074f2bbc792a4bac  Tango_4096x2160_60fps_10bit_420_jvet.yuv
c675a227a10052a3cb50a0a9542c6dd3  ToddlerFountain_4096x2160_60fps_10bit_420_jvet.yuv
e3640f0b3a325456eb2c242601473ee3  TrafficFlow_3840x2160_30fps_10bit_420_jvet.yuv

