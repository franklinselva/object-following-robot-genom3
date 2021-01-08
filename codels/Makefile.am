lib_LTLIBRARIES = libcnam_codels.la

libcnam_codels_la_SOURCES  =	cnam_c_types.h
libcnam_codels_la_SOURCES +=	cnam_codels.cc
libcnam_codels_la_SOURCES +=	cnam_track_codels.cc

libcnam_codels_la_SOURCES+=	DetectionCnam_codels.cpp DetectionCnam_codels.hpp

libcnam_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libcnam_codels_la_LIBADD   =	$(requires_LIBS)
libcnam_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libcnam_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libcnam_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	cnam_c_types.h
CLEANFILES=	${BUILT_SOURCES}
cnam_c_types.h: ${top_srcdir}/cnam.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/cnam_c_types.h.d
