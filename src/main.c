/*
 * Draw the stanford bunny inside of a glfw window
 *
 * This is a nice test to get me started. 
 * - Next one should implement 
 *   something like boomblox but using vcollide 
 * - Next should integrate in joints etc using Fedkiw paper
 *
 * Hopefully each new test will have increasingly useful code
 *
 */

#define OGL_GRAPHICS 

#include <GL/glfw.h>
#include <stdlib.h>
#include <stdio.h>
#include "VCollide.H"
#include "polytope.h"
#include "polyObject.h"
#include "graphics.h"
#include "gpuhelper.h"
#include "camera.h"
#include "Eigen/OpenGLSupport"
#include "Eigen/Geometry"
using namespace Eigen;

int WallDist;
polyObject *list=NULL;
Polytope *polytope;
long num_of_polytopes = 0;
VCollide vc;
Camera mCamera;
extern void UpdatePolytope( int id, double trans[4][4]);

const GLfloat bunny_specular[] = {0.7, 0.7, 0.7, 1.0};
const GLfloat bunny_diffuse[] = {0.5, 0.5, 0.5, 1.0};
const GLfloat bunny_shininess[] = {100.0};

int main( int argc, char *argv[] )
{
    if ( argc != 3 ) {
	    fprintf( stderr, "USAGE: %s <bunny-file> <number-of-instances> \n", argv[0]);
	    exit(1);
    }

    polytope = new Polytope(argv[1]);
    int no_of_instances = atoi(argv[2]);

    WallDist = -10;
    // Lets make some bunnies
    for (int i=0; i<no_of_instances; i++) {
        polyObject *t = new polyObject(polytope);
    	vc.NewObject(&(t->id));
    	Polytope *p = t->p;
    	for (int j=0; j<p->num_tris; j++) 
		vc.AddTri(p->vertex[(p->tris[j])[0]].v, \	
                	  p->vertex[(p->tris[j])[1]].v, \
		     	  p->vertex[(p->tris[j])[2]].v);
    	vc.EndObject();
    	cout << "created polytope with id = " << t->id << "\n";
    	t->next = list;
    	list = t;
    	num_of_polytopes++;
    }

    int width, height, x, y;
    double t;

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

    mCamera.setPosition( Vector3f( 0.0, 0.0, 100.0 ) );
    mCamera.setTarget( Vector3f( 0.0, 0.0, 0.0 ));

    do
    {
        t = glfwGetTime();
        glfwGetMousePos( &x, &y );


        // Special case: avoid division by zero below
        height = height > 0 ? height : 1;

	mCamera.setViewport( width, height );
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
	mCamera.localTranslate( Vector3f( dx, dy, dz ) );
	//cout << "Camera translations: \n" << mCamera.position() << "\n";
	// Rotate camera
	float dtest = -0.1;
	Quaternionf q = AngleAxisf( -drx*M_PI/50, Vector3f::UnitY())
		      * AngleAxisf( -dry*M_PI/50, Vector3f::UnitX())
		      * AngleAxisf( drz*M_PI, Vector3f::UnitZ());
	mCamera.localRotate( q );

        // Clear color buffer to black
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glDisable(GL_DITHER);
        glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
        //glClear( GL_COLOR_BUFFER_BIT );

  	glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
  	glLoadIdentity();
  	GLfloat light_position[] = {100, 0.0, 0.0, 1.0};
  
  	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  	glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, 0.0);
  	glPopMatrix();
  
  	glEnable(GL_LIGHTING);
  	glEnable(GL_LIGHT0);

        // Select and setup the projection matrix
	/*
        glMatrixMode( GL_PROJECTION );
        glLoadIdentity();
        gluPerspective( 60.0f, (GLfloat)width/(GLfloat)height, 0.1f, 1000.0f );
 	*/
        // Select and setup the modelview matrix
        //glMatrixMode( GL_MODELVIEW );
        //glLoadIdentity();
	/*
        gluLookAt( 0.0f, 1.0f, 0.0f,    // Eye-position
                   0.0f, 20.0f, 0.0f,   // View-point
                   0.0f, 0.0f, 1.0f );  // Up-vector
	*/
	/*
	gluLookAt( 60.0f, 0.0f, 0.0f,
		    0.0f, 0.0f, 0.0f, 
		    0.0f, 0.0f, 1.0f );
		    */
        // Draw a rotating colorful triangle
	/*
        glTranslatef( 0.0f, 14.0f, 0.0f );
        glRotatef( 0.3f*(GLfloat)x + (GLfloat)t*100.0f, 0.0f, 0.0f, 1.0f );
        glBegin( GL_TRIANGLES );
          glColor3f( 1.0f, 0.0f, 0.0f );
          glVertex3f( -5.0f, 0.0f, -4.0f );
          glColor3f( 0.0f, 1.0f, 0.0f );
          glVertex3f( 5.0f, 0.0f, -4.0f );
          glColor3f( 0.0f, 0.0f, 1.0f );
          glVertex3f( 0.0f, 0.0f, 6.0f );
        glEnd();
        */
	// Draw bunnies
	

	
        // Get window size (may be different than the requested size)
        glfwGetWindowSize( &width, &height );
	
	glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
	mCamera.activateGL();

	//cout << "Camera projection matrix: \n" << mCamera.projectionMatrix() << "\n";
	//cout << "Camera view matrix: \n" << mCamera.viewMatrix().matrix() << "\n";

	/*
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glLoadMatrix( mCamera.projectionMatrix() );
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	*/
	/*
	Eigen::Matrix4f em = mCamera.viewMatrix().matrix();
	float m[16];
	m[0] = em(0,0); m[4] = em(0,1); m[8] = em(0,2); m[12] = em(0,3);
	m[1] = em(1,0); m[5] = em(1,1); m[9] = em(1,2); m[13] = em(1,3);
	m[2] = em(2,0); m[6] = em(2,1); m[10] = em(2,2); m[14] = em(2,3);
	m[3] = em(3,0); m[7] = em(3,1); m[11] = em(3,2); m[15] = em(3,3);
	cout << "em: \n" << em(0,3) << "\n";
	cout << "m[12]: \n" << m[12] << "\n";
	glLoadMatrixf( m );
	*/
	glLoadMatrix( mCamera.viewMatrix().matrix() );
	
	// Draw a rotating colorful triangle
        glMaterialfv(GL_FRONT, GL_SPECULAR, bunny_specular);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, bunny_diffuse);
        glMaterialfv(GL_FRONT, GL_SHININESS, bunny_shininess);
 	
	polyObject *curr;
	for (curr=list; curr != NULL; curr = curr->next) {
		double ogl_trans[16];
		curr->UpdateOneStep(ogl_trans);
		double trans[4][4];

		int i,j;
		for (i=0; i<4; i++)
			for (j=0; j<4; j++)
				//cout << "ogl_trans[" << i << "][" << j << "]" << ogl_trans[4*j+i] << "\n";
				trans[i][j] = ogl_trans[4*j+i];

		//UpdatePolytope(curr->id, trans);
		curr->p->Display(ogl_trans);
	}

        // Swap buffers
        glfwSwapBuffers();

    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey( GLFW_KEY_ESC ) != GLFW_PRESS &&
           glfwGetWindowParam( GLFW_OPENED ) );

    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    exit( EXIT_SUCCESS );
}

