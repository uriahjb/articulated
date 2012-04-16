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

#ifndef WORLD_H
#define WORLD_H

#include <vector>

class
RigidBody;

class
World { 
	public:
	World();
	~World();
	
	// Add body to World
	void AddBody( RigidBody* body );
	RigidBody GetBody( int body );

	// Remove all bodies
	void Clear(); 

	// Draw our world
	void Display();
	
	// Update the world
	void Update( double dt );
		
	private:
	std:vector<RigidBody*> bodies;

	/*
	void ResolveCollisions(float dT);
	void AdvanceVelocities(float dT);
	void ResolveContacts(float dT);
	void AdvancePositions(float dT);

	static void ResolveIntersection(Intersection &i, float epsilon, bool immediate = false);
	void FindIntersections(std::vector<Intersection> & intersections);
	*/
}
#endif 
	
