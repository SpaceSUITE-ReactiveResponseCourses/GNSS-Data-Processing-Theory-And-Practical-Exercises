#!/bin/bash
# TUTORIAL 1: GNSS Data Processing Lab Exercises.
#             Introduction to gLAB tool suite
#================================================

if [ -e "WORK_FILES" ]
then
 cd "WORK_FILES"
fi

# Examples of GNSS Positioning using gLAB
# =======================================

# Example 1:
#----------

./gLAB_linux -input:cfg gLAB_example_1.cfg -input:obs roap1810.09o -input:nav brdc1810.09n 

./graph.py -f gLAB.out -x4 -y18 -s.- -c '($1=="OUTPUT")'  -l "North error" --cl b  -f gLAB.out -x4 -y19 -s.- -c '($1=="OUTPUT")'  -l "East error" --cl g  -f gLAB.out -x4 -y20 -s.- -c '($1=="OUTPUT")'  -l "UP error" --cl r --yn -8 --yx 8 --xl "time (s)" --yl "error (m)"  -t "NEU positioning error [SPP]" --sv FIG/exemple1a.png

./graph.py -f gLAB.out -c '($1=="PREFIT")' -x'(math.sin($16*math.pi/180)*(90-$15)/90)' -y'(math.cos($16*math.pi/180)*(90-$15)/90)' --cl b -f gLAB.out -c '($1=="PREFIT") & ($6==10)'  -x'(math.sin($16*math.pi/180)*(90-$15)/90)' -y'(math.cos($16*math.pi/180)*(90-$15)/90)' --cl r -so --xn -1 --xx 1 --yn -1 --yx 1 -t "Satellite skyplot" --sv FIG/exemple1b.png

./graph.py -f gLAB.out -x19 -y18 -so --cl b -c '($1=="OUTPUT")' --xl "East error (m)" --yl "North error (m)" -t "Horizontal Kinematic positioning error [SPP]" --xn -5 --xx 5 --yn -5 --yx 5 --sv FIG/exemple1c.png


# Example 2:
#----------

./gLAB_linux -input:cfg gLAB_example_2.cfg -input:obs roap1810.09o -input:sp3 igs15382.sp3 -input:ant igs05_1525.atx -input:snx igs09P1538.snx

./graph.py -f gLAB.out -x4 -y18 -s.- -c '($1=="OUTPUT")'  -l "North error" --cl b  -f gLAB.out -x4 -y19 -s.- -c '($1=="OUTPUT")'  -l "East error" --cl g  -f gLAB.out -x4 -y20 -s.- -c '($1=="OUTPUT")'  -l "UP error" --cl r --yn -0.2 --yx 0.2 --xl "time (s)" --yl "error (m)"  -t "NEU positioning error [Static PPP]" --sv FIG/exemple2.png


# Example 3:
#----------

./gLAB_linux -input:cfg gLAB_example_3.cfg -input:obs roap1810.09o -input:sp3 igs15382.sp3 -input:ant igs05_1525.atx -input:snx igs09P1538.snx

./graph.py -f gLAB.out -x4 -y18 -s.- -c '($1=="OUTPUT")'  -l "North error" --cl b  -f gLAB.out -x4 -y19 -s.- -c '($1=="OUTPUT")'  -l "East error" --cl g  -f gLAB.out -x4 -y20 -s.- -c '($1=="OUTPUT")'  -l "UP error" --cl r --yn -0.2 --yx 0.2 --xl "time (s)" --yl "error (m)"  -t "NEU positioning error [Kinematic PPP]" --sv FIG/exemple3.png
