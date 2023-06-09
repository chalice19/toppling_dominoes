# App_BasicExample is a minimal sample creating, stepping and deleting a Bullet dynamics world

set(BULLET_PHYSICS_SOURCE_DIR ../bullet3)

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src
)

LINK_LIBRARIES(
 BulletDynamics BulletCollision LinearMath
)

IF (WIN32)
	ADD_EXECUTABLE(App_BasicExample
		BasicExample.cpp
		main.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(App_BasicExample
		BasicExample.cpp
		main.cpp
	)
ENDIF()




IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)





#################
# Standalone BasicExampleGui using OpenGL (but not the example browser)


INCLUDE_DIRECTORIES(
        ${BULLET_PHYSICS_SOURCE_DIR}/src
        ${BULLET_PHYSICS_SOURCE_DIR}/btgui
        ${BULLET_PHYSICS_SOURCE_DIR}/examples
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/glad
)


SET(AppBasicExampleGui_SRCS
	BasicExample.cpp
        ${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
	../bullet3/examples/StandaloneMain/main_opengl_single_example.cpp
        ../bullet3/examples/ExampleBrowser/OpenGLGuiHelper.cpp
        ../bullet3/examples/ExampleBrowser/GL_ShapeDrawer.cpp
        ../bullet3/examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp
		../bullet3/examples/Utils/b3Clock.cpp
)

#this define maps StandaloneExampleCreateFunc to the right 'CreateFunc'
ADD_DEFINITIONS(-DB3_USE_STANDALONE_EXAMPLE)

LINK_LIBRARIES(
        BulletDynamics BulletCollision LinearMath OpenGLWindow Bullet3Common 
)

#some code to support OpenGL and Glew cross platform
IF (WIN32)
        INCLUDE_DIRECTORIES(
             ${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
        )
        ADD_DEFINITIONS(-DGLEW_STATIC)
	LINK_LIBRARIES( ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )
ELSE(WIN32)
        IF(APPLE)
                find_library(COCOA NAMES Cocoa)
                MESSAGE(${COCOA})
                link_libraries(${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

        ELSE(APPLE)
                INCLUDE_DIRECTORIES(
                        ${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
                )
                ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
                ADD_DEFINITIONS("-DGLEW_STATIC")
                ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

                FIND_PACKAGE(Threads)
                LINK_LIBRARIES( ${CMAKE_THREAD_LIBS_INIT} ${DL} )
        ENDIF(APPLE)
ENDIF(WIN32)


ADD_EXECUTABLE(AppBasicExampleGui
                ${AppBasicExampleGui_SRCS}
)


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)


