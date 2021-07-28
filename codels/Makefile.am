lib_LTLIBRARIES = libCT_robot_codels.la

libCT_robot_codels_la_SOURCES  =	CT_robot_c_types.h
libCT_robot_codels_la_SOURCES +=	CT_robot_codels.cc
libCT_robot_codels_la_SOURCES +=	CT_robot_track_codels.cc

libCT_robot_codels_la_SOURCES +=	Detection_codels.cpp Detection_codels.hpp

libCT_robot_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libCT_robot_codels_la_LIBADD   =	$(requires_LIBS)
libCT_robot_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libCT_robot_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libCT_robot_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	CT_robot_c_types.h
CLEANFILES=	${BUILT_SOURCES}
CT_robot_c_types.h: ${top_srcdir}/CT_robot.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/CT_robot_c_types.h.d
