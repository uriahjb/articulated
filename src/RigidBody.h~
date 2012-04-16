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
#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <GL/glfw.h>
#include <Eigen/Geometry>
#include "vector.h" // I hope to replace this
#include "triangle.h" // Also this

using namespace Eigen;

class
RigidBody {
	public:	
	
	// Virtual clone
	virtual RigidBody* Clone() const = 0;

	// Load geometry
	void LoadGeometry( char *fh );
	
	// Parameter Access
	Vector3f GetPos() const;
	void SetPos( Vector3f const& pos );
	Quaternionf GetOrientation() const;
	void SetOrientation( Quaternionf const& orientation );
	Vector3f GetVel() const;
	void SetVel( Vector3f const& vel );
	Vector3f GetVelocityAtPoint( Vector3f const& p );
	Vector3f const& GetAngularVelocity() const;
	void SetAngularVelocity( Vector3f const& angvel );

	Transform GetTransform() const;

	// Queue them to avoid processing order, ... sometimes
	void QueueImpulse( Vector3f const& impulse, Vector3f const& point );
	// Apply impulses, return true if significant effects
	bool ApplyQueuedImpulses();
	// Apply impulse immediately
	void ApplyImpulse( Vector3 const& impulse, Vector const& point );
	
	// Integration steps, these are Euler steps atm, it might make sense
	// to move responsibility to the World, but I'm not sure yet 
	// Advance position and orientation
	void AdvanceState( float dt );
	// Advance velocity 
	void AdvanceVel( float dt );

	// Save and Restore state
	void SaveState();
	// Restores saved info.
	void RestoreState();

	// The same as the above, but for velocity.
	void SaveVelocity();
	void RestoreVelocity();

	void setInfMass();
	virtual bool hasInfiniteMass() const;

	void Display() const;
		
	private:
	// RigidBody state
	Quaternionf orientation; 
	Vector3f position;
	Vector3f velocity;
	Vector3f angularVelocity;
	Vector3f savedVelocity;
	Vector3f savedAngularVelocity;	
	Vector3f queuedDeltaVelocity;
	Vector3f queuedDeltaAngularVelocity;

	int num_verts;
	int num_tris;
	Vector *verts;
	Triangle *tris;	

	bool hasInfiniteMass;
};

#endif
	
