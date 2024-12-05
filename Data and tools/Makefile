# Compilators
F77 = f77       
F77 = gfortran
CC = gcc
CXXFLAGS += -O3
CXXFLAGS += -fno-stack-protector
CXXFLAGS += -fcommon
CXXFLAGS += -std=gnu99

all: gLAB out2sp3 gpsxyz iono nwsw2cal tropo

# C programs [gLAB]


gLAB: source/core/gLAB.c source/core/dataHandling.c source/core/filter.c source/core/input.c source/core/model.c source/core/preprocessing.c source/core/output.c
	 cd source/core && ${CC} ${CXXFLAGS} -o gLAB_linux gLAB.c dataHandling.c filter.c input.c model.c preprocessing.c output.c -lm 


# FORTRAN programs

out2sp3: source/F77_src/out2sp3.f
	 cd source/F77_src && $(F77) -o out2sp3 out2sp3.f

gpsxyz: source/F77_src/GPSxyz.f
	 cd source/F77_src && $(F77) -c sub_orbit.f 
	 cd source/F77_src && $(F77) -o GPSxyz GPSxyz.f sub_orbit.o

iono: source/F77_src/iono.f
	 cd source/F77_src && $(F77) -c sub_klob.f 
	 cd source/F77_src && $(F77) -o iono iono.f sub_klob.o

nwsw2cal: source/F77_src/nwsw2cal.f
	 cd source/F77_src && $(F77) -o nwsw2cal nwsw2cal.f

tropo: source/F77_src/tropo.f
	 cd source/F77_src && $(F77) -c sub_trpUNB3.f
	 cd source/F77_src && $(F77) -o tropo tropo.f sub_trpUNB3.o

