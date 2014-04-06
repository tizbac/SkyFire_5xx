remove_definitions( "-D_LARGEFILE_SOURCE -O3 -pipe -msse2 -fno-strength-reduce -fno-delete-null-pointer-checks -fno-strict-aliasing -frename-registers" )
add_subdirectory(Recast)
add_subdirectory(Detour)

########### next target ###############

#SET(pathfind_STAT_SRCS
#   ChunkyTriMesh.h
#   ChunkyTriMesh.cpp
##   MeshLoaderObj.h
#   MeshLoaderObj.cpp
#)
#add_library(pathfind STATIC ${pathfind_STAT_SRCS})
