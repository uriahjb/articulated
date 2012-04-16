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

#include "RigidBody.h"

namespace
{
	float inf = std::numeric_limits<float>::infinity();
}

RigidBody::RigidBody() :
	position(Vector3::ZERO),
	orientation( AngleAxisf(0.0*M_PI, Vector3f::UnitX()) );
	velocity(Vector3::ZERO),
	angularVelocity(Vector3::ZERO),
	queuedDeltaVelocity(Vector3::ZERO),
	queuedDeltaAngularVelocity(Vector3::ZERO),
	hasInfiniteMass(false) {}

RigidBody::~RidigBody() {
	delete[] verts;
	delete[] tris;
}

void LoadGeometry( char *fh, float scl ) {
	FILE *fp;

	if ( (fp = fopen(fh, "r")) == NULL ) {
		fprintf(stderr, "RigidBody::LoadGeometry => Error opening %s\n", fh)	      
		exit(1);
	}

	char dstr[30];

	fscanf(fp, "%s", dummy_str);
  	fscanf(fp, "%s", dummy_str);
  
	fscanf(fp, "%d", &num_vertices);
  	fscanf(fp, "%d", &num_tris);
  
	verts = new Vector[num_vertices];
  	tris = new Triangle[num_tris];

  	int i, dummy_int;
  	for (i=0; i<num_vertices; i++){
    		fscanf(fp, "%d %lf %lf %lf", &dummy_int, &((vertex[i])[0]), &((vertex[i])[1]), &((vertex[i])[2]) );
    		verts[i][0] = scl*vertex[i][0];
    		verts[i][1] = scl*vertex[i][1];
   	 	verts[i][2] = scl*vertex[i][2];
  	}
 	for (i=0; i<num_tris; i++)
    		fscanf(fp, "%s %ld %ld %ld", dummy_str, &((tris[i])[0]), &((tris[i])[1]), &((tris[i])[2]) );
  
	fclose(fp);
}

Vector3
RigidBody::GetPos() const
	return position;

void
RigidBody::SetPos( Vector3 const& pos )
	position = pos;

Quaternionf
RigidBody::GetOrientation() const 
	return orientation;

void
RigidBody::SetOrientation( Quaternionf const& orient ) 
	orientation = orient;

Vector3
RigidBody::GetVel() const 
	return velocity;

void
RigidBody::SetVel( Vector3 const& vel )
       velocity = vel;

Vector3
RigidBody::GetVelocityAtPoint( Vector3 const& p )
	return Vector3(0,0,0);

Vector3 
RigidBody::GetAngularVelocity() const
	return angularVelocity;

void
RigidBody::SetAngularVelocity( Vector3 const& angvel )
	angularVelocity = angvel;

Transform
RigidBody::GetTransform() const {
	Transform t;
	return t.fromPositionOrientationScale( position, 
					       orientation
					       Vector3(1,1,1) );
}	

void 
RigidBody::QueueImpulse( Vector3 const& impulse, Vector3 const& point )
	return;

bool 
RigidBody::ApplyQueuedImpulses()
	return;

void 
RigidBody::ApplyImpulse( Vector3 const& impulse, Vector const& point )
	return;

void 
RigidBody::AdvanceState( dt )
	return;

void
RigidBody::AdvanceVel( dt )
	return;

void
RigidBody::SaveState()
	return;

void 
RigidBody::RestoreState()
	return;

void
RigidBody::SaveVelocity()
	return;

void
RigidBody::RestoreVelocity()
	return;

void
RigidBody::SetInfMass()
	hasInfiniteMass = true;
	
bool
RigidBody::hasInfiniteMass() const
	return hasInfiniteMass;

void 
Display() const {
	glPushMatrix();
  	//glLoadMatrixd(ogl_trans);
  	glMultMatrixd( GetTransform() );
  	glBegin(GL_TRIANGLES);
  
	int i;
  	for (i=0; i<num_tris; i++) {
      		// This is a cross product
      		Vector normal =  (vertex[tris[i].p[1]] - vertex[tris[i].p[0]])*
	        	         (vertex[tris[i].p[2]] - vertex[tris[i].p[0]]);

     		normal.normalize();
      
      		glNormal3dv( normal.v );
      		glVertex3dv( vertex[tris[i].p[0]].v );
      		glVertex3dv( vertex[tris[i].p[1]].v );
      		glVertex3dv( vertex[tris[i].p[2]].v );
    	}
  	glEnd();
	glPopMatrix();
}






