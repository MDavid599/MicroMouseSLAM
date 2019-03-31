#ifndef LinearSLAM_CONFIG_H
#define LinearSLAM_CONFIG_H

/* #undef LinearSLAM_SHARED_LIBS */
#define LinearSLAM_LGPL_SHARED_LIBS 1

// available sparse matrix libraries
#define LinearSLAM_HAVE_CHOLMOD 1

#define LinearSLAM_CXX_COMPILER "GNU /usr/bin/c++"

// give a warning if Eigen defaults to row-major matrices.
// We internally assume column-major matrices throughout the code.
#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
#  error "LinearSLAM requires column major Eigen matrices (see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=422)"
#endif

#endif
