This software provides updates to the JEM3 in order to provide answers to question 1-5 for 
EE2 (improvement of Adaptive Primary Transform).

The switches can be activated in Typedef.h (line 153-167), Q3 is activated by default.

The following modifications have been performed:

Q1: Replace DST-VII in JEM by DST-IV. 
    A set of tables has been provided, those tables (in TComRom.cpp) include for each 
    block-size vertical transforms using DCT2,DCT5,DCT8,DST1 and DST4.

Q2: Add ID on top of JEM Adaptive Multiple Core Transforms. 
    The ID transform is active on the top of the JEM3 transforms (DCT2,DCT5,DCT8,DST1 and 
    DST7)
    
Q3: Add DST-IV on top of JEM Adaptive Multiple Core Transforms on top of test 2.
    As originally proposed in JVET-C0022 DST4 and ID transforms are used in conjunction 
    with the JEM3 core transforms. 
    This version makes use of symmetry, that is the same transform cores are used for
    symmetrical angular IPMs (eg IPM 18 and 50 share the same transform cores).

Q4: Test 3 but mapping table between Intra Prediction Mode and Core Transform is the same for all block sizes
    Q4 use the same logic as Q3, but a single table is used for all the block sizes
    
Q5: Test 3 but mapping table between Intra Prediction Mode and Core Transform is the symmetrical. 
    Identical to Q3, as Q3 makes use of symetry, Q5 is answered through Q3, there is no
    separate test
