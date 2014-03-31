/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  dominos_t::dominos_t()
    {		
		//m_hz = 10.0f;
		//m_zeta = 0.7f;
		//m_speed = 200.0f;

		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 0.0f;
			fd.friction = 0.6f;

			shape.Set(b2Vec2(-200.0f, 0.0f), b2Vec2(200.0f, 0.0f));
			ground->CreateFixture(&fd);

			//float32 hs[10] = {0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

			//float32 x = 20.0f, y1 = 0.0f, dx = 5.0f;
			
		}
		
		{
			//body and fixture defs - the common parts
			  b2BodyDef bodyDef;
			  //bodyDef.type = b2_dynamicBody;
			  b2FixtureDef fixtureDef;
			  fixtureDef.density = 1;
			  
			  b2BodyDef bodyDef2;
			  bodyDef2.type = b2_dynamicBody;
			  
			  //two boxes
			  b2PolygonShape squareShapeA;
			  squareShapeA.SetAsBox(10,0.5);
			  
			  b2PolygonShape squareShapeB;
			  squareShapeB.SetAsBox(0.5,2);
			  
			  
			  //large box a little to the left
			  bodyDef.position.Set(-20, 6);
			  fixtureDef.shape = &squareShapeA;
			  m_bodyA = m_world->CreateBody( &bodyDef );
			  m_bodyA->CreateFixture( &fixtureDef );
			  
			  //smaller box a little to the right
			  bodyDef2.position.Set( -19, 6);
			  fixtureDef.shape = &squareShapeB;
			  m_bodyB = m_world->CreateBody( &bodyDef2 );
			  m_bodyB->CreateFixture( &fixtureDef );
			  
			  b2PrismaticJointDef prismaticJointDef;
			  prismaticJointDef.bodyA = m_bodyA;
			  prismaticJointDef.bodyB = m_bodyB;
			  prismaticJointDef.collideConnected = false;
			  prismaticJointDef.localAxisA.Set(1,0);
			  prismaticJointDef.localAnchorA.Set( 0,-0.5);//a little outside the bottom right corner
			  prismaticJointDef.localAnchorB.Set( 0, 2);//bottom left corner
			  prismaticJointDef.enableLimit = true;
			  prismaticJointDef.lowerTranslation = 0;
			  prismaticJointDef.upperTranslation = 10;
			  prismaticJointDef.enableMotor = true;
			  prismaticJointDef.maxMotorForce = 1000;
			  m_joint = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
		}
		
				{
			//body and fixture defs - the common parts
			  b2BodyDef bodyDef;
			  //bodyDef.type = b2_dynamicBody;
			  b2FixtureDef fixtureDef;
			  fixtureDef.density = 1;
			  

			  
			  //two boxes
			  b2PolygonShape squareShapeA;
			  squareShapeA.SetAsBox(10,0.5);
			  

			  
			  
			  //large box a little to the left
			  bodyDef.position.Set(-20, 0.5);
			  fixtureDef.shape = &squareShapeA;
			  m_conebase = m_world->CreateBody( &bodyDef );
			  m_conebase->CreateFixture( &fixtureDef );
			  

				}
				
			{
			//body and fixture defs - the common parts
			  b2BodyDef bodyDef;
			  bodyDef.type = b2_dynamicBody;
			  b2FixtureDef fixtureDef;
			  fixtureDef.density = 1;
			  fixtureDef.friction=0.5;
			  b2Vec2 vertices[5];
			  vertices[0].Set(-2, -1);
			  vertices[1].Set(-5, -1);
			  vertices[2].Set(-5,  1);
			  vertices[3].Set(-2, 1);
			  vertices[4].Set( 0.5, 0);
			  
			  b2PolygonShape polygonShape;
			  polygonShape.Set(vertices, 5); //pass array to the shape			  

			  
			  //two boxes
			 // b2PolygonShape squareShapeA;
			  //squareShapeA.SetAsBox(10,0.5);
			  

			  
			  
			  //large box a little to the left
			  bodyDef.position.Set(-13, 2);
			  fixtureDef.shape = &polygonShape;
			  m_conebase = m_world->CreateBody( &bodyDef );
			  m_conebase->CreateFixture( &fixtureDef );
			  

		}
		
				{
			//body and fixture defs - the common parts
			  b2BodyDef bodyDef;
			  //bodyDef.type = b2_dynamicBody;
			  b2FixtureDef fixtureDef;
			  fixtureDef.density = 1;
			  
			  b2BodyDef bodyDef2;
			  bodyDef2.type = b2_dynamicBody;
			  b2FixtureDef fixtureDef2;
			  fixtureDef2.density = 1;
			  //two boxes
			  b2PolygonShape squareShapeA;
			  squareShapeA.SetAsBox(0.5,10);
			  
			  b2PolygonShape squareShapeB;
			  squareShapeB.SetAsBox(6,0.5);
			  
			  
			  //large box a little to the left
			  bodyDef.position.Set(3.5, 10);
			  fixtureDef2.shape = &squareShapeA;
			  m_lift = m_world->CreateBody( &bodyDef );
			  m_lift->CreateFixture( &fixtureDef2 );
			  
			  //smaller box a little to the right
			  bodyDef2.position.Set( 8.5, 10);
			  fixtureDef.shape = &squareShapeB;
			  m_wall = m_world->CreateBody( &bodyDef2 );
			  m_wall->CreateFixture( &fixtureDef );
			  
			  b2PrismaticJointDef prismaticJointDef;
			  prismaticJointDef.bodyA = m_lift;
			  prismaticJointDef.bodyB = m_wall;
			  prismaticJointDef.collideConnected = false;
			  prismaticJointDef.localAxisA.Set(0,1);
			  prismaticJointDef.localAnchorA.Set( -0.5,-10);//a little outside the bottom right corner
			  prismaticJointDef.localAnchorB.Set( 6, 0);//bottom left corner
			  prismaticJointDef.enableLimit = true;
			  prismaticJointDef.lowerTranslation = 0;
			  prismaticJointDef.upperTranslation = 20;
			  prismaticJointDef.enableMotor = true;
			  prismaticJointDef.maxMotorForce = 400;
			  m_joint_lift = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
		}
		
		
		
	}
    

    void dominos_t::keyboard(unsigned char key)
    {
        switch (key)
		{
		case 'a':
			m_joint->SetMotorSpeed(-6.0f);
			break;

		case 'd':
			m_joint->SetMotorSpeed(6.0f);
			break;
		case 'w':
			m_joint_lift->SetMotorSpeed(2.0f);
			break;

		case 's':
			m_joint_lift->SetMotorSpeed(-2.0f);
			break;
		
		}
    }
    
    void dominos_t::keyboardUp(unsigned char key)
    {
        switch (key)
		{
		case 'w':
			m_joint_lift->SetMotorSpeed(0.0f);
			break;

		case 's':
			m_joint_lift->SetMotorSpeed(0.0f);
			break;
		
		}
    }


  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}

