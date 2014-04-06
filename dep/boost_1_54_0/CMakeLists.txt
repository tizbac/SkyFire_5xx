#  Copyright Troy D. Straszheim 2009.
#  Distributed under the Boost Software License, Version 1.0.
#      (See accompanying file LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)
include_directories ( . )
find_package(PythonLibs)
#remove_definitions( "-D_LARGEFILE_SOURCE -O3 -pipe -msse2 -fno-strength-reduce -fno-delete-null-pointer-checks -fno-strict-aliasing -frename-registers" )
remove_definitions( -Wall -Wfatal-errors -Wextra -DSCRIPTS -fno-delete-null-pointer-checks)
add_definitions ( --no-warnings -ftemplate-depth-128 )
if ( NOT MINGW )
SET ( BOOST_COMPILE_FLAGS "-fno-inline -pthread  -DBOOST_ALL_NO_LIB=1 -DBOOST_THREAD_BUILD_DLL=1 -DBOOST_THREAD_POSIX" )
SET ( BOOST_THREAD_SRC ./libs/thread/src/pthread/once.cpp ./libs/thread/src/pthread/thread.cpp)

else ( NOT MINGW )
SET ( BOOST_COMPILE_FLAGS "-fno-inline  -DBOOST_ALL_NO_LIB=1 -DBOOST_THREAD_BUILD_DLL=1 -DBOOST_THREAD_WIN32")
SET ( BOOST_THREAD_SRC ./libs/thread/src/win32/thread.cpp ./libs/thread/src/win32/tss_dll.cpp ./libs/thread/src/win32/tss_pe.cpp)
endif ()
if (STATIC_LINK)
add_library(pfboostthread STATIC ${BOOST_THREAD_SRC})

else ( STATIC_LINK )
add_library(pfboostthread SHARED ${BOOST_THREAD_SRC})
endif ( STATIC_LINK )
if (STATIC_LINK)
add_library(pfboostsystem STATIC ./libs/system/src/error_code.cpp )
add_library(pfboostregex STATIC libs/regex/src/cpp_regex_traits.cpp
libs/regex/src/cregex.cpp
libs/regex/src/c_regex_traits.cpp
libs/regex/src/fileiter.cpp
libs/regex/src/icu.cpp
libs/regex/src/instances.cpp
libs/regex/src/posix_api.cpp
libs/regex/src/regex.cpp
libs/regex/src/regex_debug.cpp
libs/regex/src/regex_raw_buffer.cpp
libs/regex/src/regex_traits_defaults.cpp
libs/regex/src/static_mutex.cpp
libs/regex/src/usinstances.cpp
libs/regex/src/w32_regex_traits.cpp
libs/regex/src/wc_regex_traits.cpp
libs/regex/src/wide_posix_api.cpp
libs/regex/src/winstances.cpp
)
else ( STATIC_LINK )
add_library(pfboostsystem SHARED ./libs/system/src/error_code.cpp )
add_library(pfboostregex SHARED libs/regex/src/cpp_regex_traits.cpp
libs/regex/src/cregex.cpp
libs/regex/src/c_regex_traits.cpp
libs/regex/src/fileiter.cpp
libs/regex/src/icu.cpp
libs/regex/src/instances.cpp
libs/regex/src/posix_api.cpp
libs/regex/src/regex.cpp
libs/regex/src/regex_debug.cpp
libs/regex/src/regex_raw_buffer.cpp
libs/regex/src/regex_traits_defaults.cpp
libs/regex/src/static_mutex.cpp
libs/regex/src/usinstances.cpp
libs/regex/src/w32_regex_traits.cpp
libs/regex/src/wc_regex_traits.cpp
libs/regex/src/wide_posix_api.cpp
libs/regex/src/winstances.cpp
)
endif ( STATIC_LINK )
set_target_properties(pfboostthread PROPERTIES COMPILE_FLAGS ${BOOST_COMPILE_FLAGS} )
set_target_properties(pfboostsystem PROPERTIES COMPILE_FLAGS ${BOOST_COMPILE_FLAGS} )
set_target_properties(pfboostregex PROPERTIES COMPILE_FLAGS ${BOOST_COMPILE_FLAGS} )
IF ( PYTHONLIBS_FOUND AND NOT MINGW )
	include_directories( ${PYTHON_INCLUDE_DIRS} ${PYTHON_INCLUDE_PATH})
        add_library(pfboostpython
	./libs/python/src/converter/arg_to_python_base.cpp
	./libs/python/src/converter/builtin_converters.cpp
	./libs/python/src/converter/from_python.cpp
	./libs/python/src/converter/registry.cpp
	./libs/python/src/converter/type_id.cpp
	./libs/python/src/dict.cpp
	./libs/python/src/errors.cpp
	./libs/python/src/exec.cpp
	./libs/python/src/import.cpp
	./libs/python/src/list.cpp
	./libs/python/src/long.cpp
	./libs/python/src/module.cpp
	./libs/python/src/numeric.cpp
	./libs/python/src/object/class.cpp
	./libs/python/src/object/enum.cpp
	./libs/python/src/object/function.cpp
	./libs/python/src/object/function_doc_signature.cpp
	./libs/python/src/object/inheritance.cpp
	./libs/python/src/object/iterator.cpp
	./libs/python/src/object/life_support.cpp
	./libs/python/src/object/pickle_support.cpp
	./libs/python/src/object/stl_iterator.cpp
	./libs/python/src/object_operators.cpp
	./libs/python/src/object_protocol.cpp
	./libs/python/src/slice.cpp
	./libs/python/src/str.cpp
	./libs/python/src/tuple.cpp
	./libs/python/src/wrapper.cpp
	)
        set_target_properties(pfboostpython PROPERTIES COMPILE_FLAGS ${BOOST_COMPILE_FLAGS} )
ELSE ( PYTHONLIBS_FOUND  AND NOT MINGW )
        message("Impossibile trovare le librerie python , pfboostpython non verrà compilato!")

ENDIF ( PYTHONLIBS_FOUND  AND NOT MINGW)
IF ( STATIC_LINK )
add_library(pfboostprogramopts STATIC
./libs/program_options/src/cmdline.cpp
./libs/program_options/src/config_file.cpp
./libs/program_options/src/convert.cpp
./libs/program_options/src/options_description.cpp
./libs/program_options/src/parsers.cpp
./libs/program_options/src/positional_options.cpp
./libs/program_options/src/split.cpp
./libs/program_options/src/utf8_codecvt_facet.cpp
./libs/program_options/src/value_semantic.cpp
./libs/program_options/src/variables_map.cpp
./libs/program_options/src/winmain.cpp
)


ELSE ( STATIC_LINK )
add_library(pfboostprogramopts SHARED
./libs/program_options/src/cmdline.cpp
./libs/program_options/src/config_file.cpp
./libs/program_options/src/convert.cpp
./libs/program_options/src/options_description.cpp
./libs/program_options/src/parsers.cpp
./libs/program_options/src/positional_options.cpp
./libs/program_options/src/split.cpp
./libs/program_options/src/utf8_codecvt_facet.cpp
./libs/program_options/src/value_semantic.cpp
./libs/program_options/src/variables_map.cpp
./libs/program_options/src/winmain.cpp
)
ENDIF ( STATIC_LINK )

set_target_properties(pfboostprogramopts PROPERTIES COMPILE_FLAGS ${BOOST_COMPILE_FLAGS} )
IF ( NOT STATIC_LINK )
install(TARGETS pfboostthread DESTINATION lib)
install(TARGETS pfboostsystem DESTINATION lib)
install(TARGETS pfboostpython DESTINATION lib)
install(TARGETS pfboostprogramopts DESTINATION lib)
install(TARGETS pfboostregex DESTINATION lib)
ENDIF ( NOT STATIC_LINK )
