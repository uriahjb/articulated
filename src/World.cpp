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
#include <limits>

World::World() {};

World::~World() Clear();

void
World::Clear()
	for( int i=0; i<GetNumBodies(); i++ )
		delete GetBody(i);

void 
World::AddBody( RigidBody* body )
	bodies.push_back( body );

int
World::GetNumBodies() const
	return int( bodies.size() );

RigidBody
World::GetBody( int body )
	return bodies[body];

void
World::Display() const {
	for( int i=0; i<GetNumBodies(); i++ )
		GetBody(i)->Display();
}

void 
World::Update( double dt ) {
	/*
	ResolveCollisions( dt );
	AdvanceVelocities( dt );
	ResolveContacts( dt );
	AdvancePositions( dt );
	... And all that other stuff we need to do for articulated bodies
	*/
}


