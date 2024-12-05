#!/bin/bash
# TUTORIAL 4. Solving Navigation Equations: Least squares and Kalman filter
#==========================================================================

if [ -e "WORK_FILES" ]
then
  cd "WORK_FILES"
fi


# Ex 1: Navigation Equations System and LS solution (ENU)  [Ex. 6 session 5.2, GNNS BOOK]
# =======================================================

#Previous:

./gLAB_linux -input:cfg gLAB_TUT4_ex1.cfg -input:obs UPC11490.05O -input:nav UPC11490.05N


# a) Prefit residual vector (Y) and Geometry matrix (G) generation
# ----------------------------------------------------------------

awk '{if ($1=="PREFIT" && $4==300) print $6,$8,$15,$16}' gLAB.out > dat

cat dat| gawk 'BEGIN{g2r=atan2(1,1)/45}{e=$3*g2r;a=$4*g2r;print $2,-cos(e)*sin(a),-cos(e)*cos(a),-sin(e),1}' > M.dat




# b) LS solution with Octave or MATLAB
# ------------------------------------

# /////////////////////////////
 octave

 load M.dat
 
  Y=M(:,1)
  G=M(:,2:5)

  x=inv(G'*G)*G'*Y

  exit
# /////////////////////////////
   ==> [-0.16688  1.08547  4.49547  -4.09169]
 


# - Cross-hecking with gLAB values:

./gLAB_linux -input:cfg gLAB_TUT4_ex1b.cfg -input:obs UPC11490.05O -input:nav UPC11490.05N

grep OUTPUT gLAB.out | grep -v INFO | gawk '{if ($4==300) print $19, $18, $20}'  
#  ==> [-0.1669  1.0855  4.4955]  


grep FILTER gLAB.out | grep -v INFO | gawk '{if ($4==300) print $8}' 
#   ==> -4.0917




# Ex 2: Navigation Equations System and LS solution (XYZ) [Ex. 7 session 5.2, GNNS BOOK]
# =======================================================


# a) Prefit residual vector (Y) and Geometry matrix (G) generation
# ----------------------------------------------------------------

./gLAB_linux -input:cfg gLAB_TUT4_ex2.cfg -input:obs UPC11490.05O -input:nav UPC11490.05N

# Write in a single line:
grep "MODEL " gLAB.out | grep -v INFO | grep C1C | 
   gawk 'BEGIN{x=4789032.6277;y=176595.0498;z=4195013.2503} 
     {if ($4==300) 
       {r1=x-$11;r2=y-$12;r3=z-$13;r=sqrt(r1*r1+r2*r2+r3*r3); print $9-$10,r1/r,r2/r,r3/r,1}}' > M.dat


# - Cross-checking gLAB values:
grep "PREFIT " gLAB.out | grep -v INFO | gawk '{if ($4==300) print $8, $11, $12, $13, $14}'  


# b) LS solution with Octave or MATLAB
# ------------------------------------

# /////////////////////////////
  octave

   load M.dat
   Y=M(:,1)
   G=M(:,2:5)

   x=inv(G'*G)*G'*Y

  exit
# /////////////////////////////

#   ==> [2.659322  -0.068950  3.786600  -4.091713]



# - Cross-checking gLAB values:
grep OUTPUT gLAB.out | grep -v INFO | gawk '{if ($4==300) print $9, $10, $11}'  
#   ==> [2.6593  -0.0689  3.7866]

# - Checking clock:
grep FILTER gLAB.out | grep -v INFO | gawk '{if ($4==300) print $8}'  
#   ==> -4.0917




# Ex 3: Solving with Kalman filter [Ex. 8 session 5.2, GNNS BOOK]
# ================================


# a) SPS solution in Static mode
# ------------------------------

./gLAB_linux -input:cfg gLAB_TUT4_ex3.cfg -input:obs UPC11490.05O -input:nav UPC11490.05N



# b) Data vectors and matrices:
 
grep "PREFIT " gLAB.out | grep -v INFO |gawk '{if ($4==300) print $8,$11,$12,$13,$14 }' > M300.dat
grep "PREFIT " gLAB.out | grep -v INFO |gawk '{if ($4==600) print $8,$11,$12,$13,$14 }' > M600.dat
grep "PREFIT " gLAB.out | grep -v INFO |gawk '{if ($4==900) print $8,$11,$12,$13,$14 }' > M900.dat


# c) Kalman filter with Octave or MATLAB
# --------------------------------------

# /////////////////////////////
  octave

   load M300.dat
   load M600.dat
   load M900.dat

   x0=[0 0 0 0]'
   sigma0=3e5
   P0=(sigma0)^2*eye(4,4)

   sig_dt=3e5
   Q=zeros(4,4)
   Q(4,4)=(sig_dt)^2

   fi=eye(4,4)
   fi(4,4)=0


   Y1=M300(:,1);
   G1=M300(:,2:5);
   sigma_y=1
   R1=(sigma_y)^2*eye(size(Y1),size(Y1))

   Y2=M600(:,1);
   G2=M600(:,2:5);
   sigma_y=1
   R2=(sigma_y)^2*eye(size(Y2),size(Y2))

   Y3=M900(:,1);
   G3=M900(:,2:5);
   sigma_y=1
   R3=(sigma_y)^2*eye(size(Y3),size(Y3))



# Iterations:
#............

# k=1

   x1_=fi*x0
   P1_=fi*P0*fi'+Q

   P1=inv(G1'*inv(R1)*G1+inv(P1_))
   x1=P1*(G1'*inv(R1)*Y1+inv(P1_)*x1_)

# k=2

   x2_=fi*x1
   P2_=fi*P1*fi'+Q

   P2=inv(G2'*inv(R2)*G2+inv(P2_))
   x2=P2*(G2'*inv(R2)*Y2+inv(P2_)*x2_)


# k=3

   x3_=fi*x2
   P3_=fi*P2*fi'+Q

   P3=inv(G3'*inv(R3)*G3+inv(P3_))
   x3=P3*(G3'*inv(R3)*Y3+inv(P3_)*x3_)

  exit
# /////////////////////////////



# d) Cross-checking results with gLAB 
#....

grep OUTPUT gLAB.out | grep -v INFO |gawk '{if ($4==300) print $9,$10,$11}'  
grep OUTPUT gLAB.out | grep -v INFO |gawk '{if ($4==600) print $9,$10,$11}'  
grep OUTPUT gLAB.out | grep -v INFO |gawk '{if ($4==900) print $9,$10,$11}'  

grep FILTER gLAB.out | grep -v INFO |gawk '{if ($4==300) print $8}'  
grep FILTER gLAB.out | grep -v INFO |gawk '{if ($4==600) print $8}'   
grep FILTER gLAB.out | grep -v INFO |gawk '{if ($4==900) print $8}'   


# Removing temporal files
rm dat M.dat gLAB.out M300.dat M600.dat M900.dat
