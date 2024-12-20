#!/bin/bash
# TUTORIAL 3. Detailed Code pseudorange modelling and Prefit-residual computation
# ===============================================================================

if [ -e "WORK_FILES" ]
then
  cd "WORK_FILES" 
fi

# Detailed Code Pseudorange Modelling and Prefit-residual computation  [Ex. 2 session 5.2, GNNS BOOK]
# ====================================================================

# Compute the values at time: 0h 5m 0s, satellite PRN25:

#  Previous: Computing solution with gLAB
# ---------------------------------------

./gLAB_linux -input:cfg gLAB_TUT3.cfg -input:obs UPC11490.05O -input:nav UPC11490.05N


# 1) Selection of orbital elements:
# ---------------------------------

  grep -A7 ^25 UPC11490.05N | head -8  

#........................ eph.dat  (in a single line) ..........................
# 25 5  5 29  2  0  0.0 9.401096031070E-05 9.094947017729E-13 0.000000000000E+00
# 8.400000000000E+01 -1.061875000000E+02  4.825915304457E-09 -2.255215633503E+00
#-5.284324288368E-06  1.204112719279E-02  5.686655640602E-06  5.153704689026E+03
# 7.200000000000E+03  2.011656761169E-07 -2.689273653419E+00  1.396983861923E-07
# 9.492799505545E-01  2.625625000000E+02 -1.460408709553E+00 -8.100337411567E-09
#-3.643008888800E-11  1.000000000000E+00  1.325000000000E+03  0.000000000000E+00
# 2.800000000000E+00  0.000000000000E+00 -7.450580596924E-09  8.520000000000E+02
# 1.800000000000E+01  0.000000000000E+00  1.000000000000E+00  0.000000000000E+00
# ..............................................................................

#  Transmission time:
#  sec_of_GPS_week=1.800000000000E+01
#  GPS_WEEK = 1.325000000000E+03

 echo 1325 18| ./nwsw2cal  
#  [2005  5 29  0  0 18.0000  149 18.000 1325 18.000 2453519.500  53519.000]
#    ==> Transmitted at: 0H 0M 18 sec of DOY=149


# 2) Satellite clock offset computation:
# -------------------------------------

# Using Octave or MATLAB compute: 

# /////////////////////////////
 octave

    format long

    c=299792458

    sec= 300
    toc= 2*3600+ 0*60 +0

    a0= 9.401096031070E-05
    a1= 9.094947017729E-13
    a2= 0 

    dt_sat0=a0+a1*(sec-toc)+a2*(sec-toc)**2

    c*dt_sat0 
 exit
# /////////////////////////////

#           ==>  9.40046848e-05
#           ==> c*dt_sat0   ===> 28181.89551

# - Cross-checking with gLAB values:

  grep MODEL gLAB.out | grep -v INFO | gawk '{if ($6==25) print $4,$6,$18}' |head -1  

#  ==>[300.00 25 -28181.89550]



# 3. Satellite Instrumental delay (TGD)
# ------------------------------------

#  TGD= -7.450580596924E-09 (in seconds) (from ephemeris)

#  TGD*c =-2.23363  (in meters)


# - Cross-checking gLAB values:
    grep MODEL gLAB.out | grep -v INFO | gawk '{if ($6==25) print $4,$6,$27}' |head -1  

#   ==>[300.00 25 -2.23363]



# 4. Satellite-receiver geometric range computation
# -------------------------------------------------

# 4.1) Emission time computation from receiver time-tag and code pseudorange:


#   C1=22857303.996 [5  5 29  0  5]
#   sec_ems=sec-C1/c-dt_sat0 
    
#   sec_ems=299.923662236054


# 4.2) Satellite coordinates at transmission time:
# ....

#  A) Computation of satellite coordinates in an Earth-Fixed reference 
#     frame (CTS) at t=T[emission]= 299.92366 s:
#
#  The ephemeris file "eph.dat" provided in "FILES/" directory
#  can be used as well.

#  Execute:
   echo "2005 149 299.92366224" > time.dat
   cat time.dat eph.dat | GPSxyz
#
#  ==>[25. 299.92366224  6364868.618075 -14298233.062153  21851197.940638 3.022976]
  
#  B) Transform these coordinates to CTS [reception]
#
# Using Octave or MATLAB compute: 

# /////////////////////////////
  octave
    format long
    c=299792458

# - Signal flight time:
    r0_rcv=[4789032.6277      176595.0498    4195013.2503]   
    r_sat= [6364868.61807  -14298233.06215  21851197.94064]
    dt_fight=norm(r_sat-r0_rcv,2)/c
#   ==> dt_fight= 0.076337713

# - Reference system Rotation:
    we= 7.2921151467e-5   
    theta=we*dt_fight
    R=[cos(theta) sin(theta) 0 ; -sin(theta) cos(theta) 0 ; 0 0 1]

    r_sat_ems=(R*r_sat')'
  exit
# /////////////////////////////

#   ==> [6364789.02494202  -14298268.49282210   21851197.94064000]

# - Checking gLAB values:
    grep MODEL gLAB.out | grep -v INFO | gawk '{if ($6==25) print $4,$6,$11,$12,$13}'|head -1  
#   ==>[300.00 25 6364789.0249 -14298268.4928 21851197.9406]



# 4.3) Geometric range computation: sat[emission]-rcv[reception]
# ------------------------------------------------------------

# Using Octave or MATLAB compute: 

# /////////////////////////////
  octave
    format long
    r0_rcv=[4789032.6277 176595.0498 4195013.250]
    r_sat_ems=[6364789.0249 -14298268.4928 21851197.9406]
    rho=norm(r_sat_ems-r0_rcv,2)
  exit
# /////////////////////////////

#     ===> rho=22885487.5549721

# - Checking gLAB values:
   grep MODEL gLAB.out | grep -v INFO | gawk '{if ($6==25) print $4,$6,$17}' |head -1  
#  ==>[300.00 25 22885487.5548]



# 5) Relativistic clock correction:
# --------------------------------

# Using Octave or MATLAB compute: 

# /////////////////////////////
  octave
    format long

   a12= 5.153704689026E+03  
   a=a12*a12 
#  ==> 26560672.0216886 (from ephemeris)
   mu= 3986004.418e8   
   c= 299792458 
   e= 1.204112719279E-02  

#  From previous computations, we have the eccentric anomaly  value:
   E= 3.022976   

   dt_rel= -2*sqrt(mu*a)/c*e*sin(E) 
  exit
# /////////////////////////////

# ==> -0.9781  


# -Cross-checking gLAB values:
   grep MODEL gLAB.out | grep -v INFO | gawk '{if ($6==25) print $4,$6,$22}' |head -1  
#  ==> [300.00 25 0.98343]



# 6. Ionospheric correction
# --------------------.......

# ------------  iono.dat (in a single line) ----------------------
#  300 4789032.6277    176595.0498   4195013.2503
#         6364789.0249 -14298268.4928  21851197.9406
#     1.0245E-08  2.2352E-08 -5.9605E-08 -1.1921E-07
#     9.6256E+04  1.3107E+05 -6.5536E+04 -5.8982E+05
# --------------------------------------------------------------

#  Use iono.dat file provided for this exercise.

  cat iono.dat | ./iono  
  # ==> 2.4726365 

# - Cross-checking gLAB values:
  grep MODEL gLAB.out | grep -v INFO | gawk '{if ($6==25) print $4,$6,$25}' |head -1 

#  ==> [300.00 25 2.47264]


# 7. Tropospheric correction:
# ---------------------------

#     ................. tropo.dat .....................
#     149.00 4789032.6277    176595.0498   4195013.2503 
#            6364789.0249 -14298268.4928  21851197.9406   
#     .................................................        

#  Use the tropo.dat file provided provided for this exercise.

     cat tropo.dat | ./tropo  
#    ==> 4.46583

# - Cross-checking gLAB values:
  grep MODEL gLAB.out | grep -v INFO | gawk '{if ($6==25) print $4,$6,$24}' |head -1  

#  ==>[300.00 25 4.46584]



8. Compute the modeled pseudorange
# --------------------------------

#  C1_mod= rho -dt_sat + Tropo + Iono + TGD =
#        = 22857311.34754 (m)

#          rho= 22885487.55479 (m)
#          dt_sat= dt_sat0 +dt_rel
#                = 28181.89551 -0.98342 = 28180.91209 (m)
#          Tropo= 4.46583 (m)
#          Iono=  2.47264 (m)
#          TGD=  -2.23363 (m)

# - Checking gLAB values:
  grep MODEL gLAB.out | grep -v INFO | gawk '{if ($6==25) print $4,$6,$10}' |head -1  

#  ==> [300.00 25 22857311.3476]


# 9) Calculate the Prefit residual: [Measured - Modelled]
# -------------------------------------------------------

# - Measured Value
  grep MODEL gLAB.out | grep -v INFO | gawk '{if ($6==25) print $4,$6,$9}' |head -1 
#  ==> [300.00 25 22857303.9960]

#  - Prefit-residual calculation:
#
#  Prefit= 22857303.9960 -  22857311.34754 = -7.35154 (m)

# - Cross-checking with gLAB values:
#   Prefit residual: [Measured - Modelled]
    grep PREFIT gLAB.out | grep -v INFO | gawk '{if ($6==25) print $4,$6,$8}' |head -1 

#   ==> [300.00 25 -7.3516]


# Summary:
# ========
#
#  (metres)|        gLAB     |  Hand-calculated 
#  ---------------------------------------------
#  rho     |  22885487.5548  |  22885487.55479      
#  dt_sat0 |    -28181.89550 |    -28181.89551  
#  dt_rel  |         0.98343 |         0.98342       
#  Tropo   |         4.46584 |         4.46583       
#  Iono    |         2.47264 |         2.47264 
#  TGD     |        -2.23363 |        -2.23363  
#  ---------------------------------------------
#  C1_mod  |  22857311.3476  |  22857311.34754
#  ---------------------------------------------
#  Prefit  |        -7.3516  |        -7.35154
#  ---------------------------------------------


# Cleaning temporary files
rm time.dat
##############################################################################



