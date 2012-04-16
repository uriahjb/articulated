/*
* Articulated: an articulated rigid body dynamics simulator
* Copyright (C) 2012 Uriah Baalke
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
*(at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <GL/glfw.h>
#include "Camera.h"
#include "World.h"
#include "RigidBody.h"
#include "WorldLoader.h"

void artInit();
void updateCam();
void artRun();

double curtime;
double prevtime;
double dt;
Camera artCam;
//World artWorld;

void 
artInit() {
    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        exit( EXIT_FAILURE );
    }

    // Open a window and create its OpenGL context
    if( !glfwOpenWindow( 640, 480, 8,8,8,8, 24,0, GLFW_WINDOW ) )
    {
        fprintf( stderr, "Failed to open GLFW window\n" );

        glfwTerminate();
        exit( EXIT_FAILURE );
    }

    glfwSetWindowTitle("Loading polytope test");

    // Ensure we can capture the escape key being pressed below
    glfwEnable( GLFW_STICKY_KEYS );

    // Enable vertical sync (on cards that support it)
    glfwSwapInterval( 1 );

    artCam.setPosition( Vector3f( 0.0, 0.0, 100.0 ) );
    artCam.setTarget( Vector3f( 0.0, 0.0, 0.0 ));
	prevtime = glfwGetTime();
}

void 
updateCam() {
	glfwGetMousePos( &x, &y );

	// Special case: avoid division by zero below
    	height = height > 0 ? height : 1;

	artCam.setViewport( width, height );
        //glViewport( 0, 0, width, height );
        // Initialie camera position, etc 
        //mCamera.setPosition( Vector3f( .0, -1000.0, -1000.0) );
        //mCamera.setTarget( Vector3f(0.0, 0.0, 0.0) );

	// Compute motion for camera
 	float drx = float(x) / float(mCamera.vpWidth()) - 0.5;
	float dry = float(y) / float(mCamera.vpHeight()) - 0.5;
	float drz = 0;
	float dx = 0;
	float dy = 0;
	float dz = 0;
	if ( glfwGetKey( GLFW_KEY_LSHIFT ) != GLFW_PRESS ) {
		if ( glfwGetKey( 'W' ) == GLFW_PRESS ) dz = -10;
		if ( glfwGetKey( 'S' ) == GLFW_PRESS ) dz = 10;
	} else {
		if ( glfwGetKey( 'W' ) == GLFW_PRESS ) dy = 10;
		if ( glfwGetKey( 'S' ) == GLFW_PRESS ) dy = -10;
	}
	if ( glfwGetKey( 'A' ) == GLFW_PRESS ) dx = -10;
	if ( glfwGetKey( 'D' ) == GLFW_PRESS ) dx = 10;
	if ( glfwGetKey( 'Q' ) == GLFW_PRESS ) drz = 0.01;
	if ( glfwGetKey( 'E' ) == GLFW_PRESS ) drz = -0.01;
	//cout << "dx, dy: " << drx << "," << dry << "\n";	
	artCam.localTranslate( Vector3f( dx, dy, dz ) );
	//cout << "Camera translations: \n" << mCamera.position() << "\n";
	// Rotate camera
	float dtest = -0.1;
	Quaternionf q = AngleAxisf( -drx*M_PI/50, Vector3f::UnitY())
		      * AngleAxisf( -dry*M_PI/50, Vector3f::UnitX())
		      * AngleAxisf( drz*M_PI, Vector3f::UnitZ());
	artCam.localRotate( q );
}

void
artRun() { 
	curtime = glfwGetTime();
	dt = curtime - prevtime;
	prevtime = curtime;

	// Get window size (may be different than the requested size)
        glfwGetWindowSize( &width, &height );
	
	glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
	// Apply camera transforms 
	artCam.activateGL();

	//artWorld.Update( dt )
	artWorld.Display()
	glfwSwapBuffers();
}

int
main() {
	artInit();
	do {
		updateCam();
		artRun();
	} while ( glfwGetKey( GLFW_KEY_ESC ) != GLFW_PRESS &&
              glfwGetWindowParam( GLFW_OPENED ) );
	glfwTerminate();
	exit( EXIT_SUCCESS );
}
	

