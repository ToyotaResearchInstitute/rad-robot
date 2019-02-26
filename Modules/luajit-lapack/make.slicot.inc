####################################################################
#  SLICOT make include file.                                       #
#  SLICOT, Release 4.0                          ./slicot/make.inc  #
#  Vasile Sima, KU Leuven                                          #
#  October 31, 1996.                                               #
#  Revised December 7, 1999.                                       #
####################################################################
#
#  The machine (platform) identifier to append to the library names
#
#  PLAT = _sun4
#  
#  Modify the FORTRAN and OPTS definitions to refer to the
#  compiler and desired compiler options for your machine.  NOOPT
#  refers to the compiler options desired when NO OPTIMIZATION is
#  selected.  Define LOADER and LOADOPTS to refer to the loader and 
#  desired load options for your machine.
#
FORTRAN  = gfortran
#OPTS     = -O4 -native -u
OPTS     = -O4 -dynamiclib -shared -fPIC -frecursive
NOOPT    = -dynamiclib -shared -fPIC -frecursive
LOADER   = gfortran
LOADOPTS = $(SLICOTLIB) $(LAPACKLIB)
#
#  The archiver and the flag(s) to use when building archive (library)
#  If your system has no ranlib, set RANLIB = echo.
#
#ARCH     = ar
#ARCHFLAGS= r
ARCH     = ld
ARCHFLAGS= -dylib -undefined dynamic_lookup $(LAPACKLIB) $(GFORTRANLIB)
#ARCHFLAGS= -dylib -undefined dynamic_lookup -flat_namespace $(LAPACKLIB) $(GFORTRANLIB)
# ARCHFLAGS= cr
# RANLIB   = ranlib
#
#  The location of the libraries to which you will link.  (The 
#  machine-specific, optimized BLAS library should be used whenever
#  possible.)
#
#BLASLIB     = -L/software/lib
#LAPACKLIB    = -L/software/lib -llapack -lblas
LAPACKLIB    = -L/usr/local/opt/openblas/lib -llapack -lopenblas
GFORTRANLIB    = -L/usr/local/lib/gcc/8 -lgfortran
#SLICOTLIB    = ../slicot.a
SLICOTLIB    = -o ../libslicot.dylib
