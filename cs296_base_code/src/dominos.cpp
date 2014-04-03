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

#include <math.h> 
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
			  squareShapeA.SetAsBox(10.3,0.5);
			  
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
			  prismaticJointDef.upperTranslation = 10.3;
			  prismaticJointDef.enableMotor = true;
			  prismaticJointDef.maxMotorForce = 10000;
			  m_joint = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
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
			  bodyDef.position.Set(-20, 12);
			  fixtureDef.shape = &squareShapeA;
			  m_bodyA1 = m_world->CreateBody( &bodyDef );
			  m_bodyA1->CreateFixture( &fixtureDef );
			  
			  //smaller box a little to the right
			  bodyDef2.position.Set( -19, 12);
			  fixtureDef.shape = &squareShapeB;
			  m_bodyB1 = m_world->CreateBody( &bodyDef2 );
			  m_bodyB1->CreateFixture( &fixtureDef );
			  
			  b2PrismaticJointDef prismaticJointDef;
			  prismaticJointDef.bodyA = m_bodyA1;
			  prismaticJointDef.bodyB = m_bodyB1;
			  prismaticJointDef.collideConnected = false;
			  prismaticJointDef.localAxisA.Set(1,0);
			  prismaticJointDef.localAnchorA.Set( 0,-0.5);//a little outside the bottom right corner
			  prismaticJointDef.localAnchorB.Set( 0, 2);//bottom left corner
			  prismaticJointDef.enableLimit = true;
			  prismaticJointDef.lowerTranslation = 0;
			  prismaticJointDef.upperTranslation = 10.3;
			  prismaticJointDef.enableMotor = true;
			  prismaticJointDef.maxMotorForce = 10000;
			  m_joint1 = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
		}
		
		
				{
			//body and fixture defs - the common parts
			  b2BodyDef bodyDef;
			  //bodyDef.type = b2_dynamicBody;
			  b2FixtureDef fixtureDef;
			  fixtureDef.density = 1;
			  

			  
			  //two boxes
			  b2PolygonShape squareShapeA;
			  squareShapeA.SetAsBox(10.3,0.5);
			  

			  
			  
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
			  fixtureDef.density = 2;
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
			  m_cone = m_world->CreateBody( &bodyDef );
			  m_cone->CreateFixture( &fixtureDef );
			  

		}
		
		    {
			  b2PolygonShape shape;
			  shape.SetAsBox(0.5f, 1.0f);
			
			  b2FixtureDef fd;
			  fd.shape = &shape;
			  fd.density = 4.0f;
			  fd.friction = 0.1f;
				
			  for (int i = 0; i < 4; ++i)
			{
			  b2BodyDef bd;
			  bd.type = b2_dynamicBody;
			  bd.position.Set(-16.0f + 1.0f * i, 7.0f);
			  domin[i] = m_world->CreateBody(&bd);
			  domin[i]->CreateFixture(&fd);
			}
			}
			
			{
			  b2PolygonShape shape;
			  shape.SetAsBox(0.5f, 3.0f);
			
			  b2FixtureDef fd;
			  fd.shape = &shape;
			  b2BodyDef bd;
			  bd.position.Set(-9.8, 16.0f);
			  m_wall2 = m_world->CreateBody(&bd);
			  m_wall2->CreateFixture(&fd);
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
			  squareShapeB.SetAsBox(6.2,0.5);
			  
			  
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
			  prismaticJointDef.maxMotorForce = 10000;
			  m_joint_lift = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
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
			  squareShapeA.SetAsBox(0.5,15);
			  
			  b2PolygonShape squareShapeB;
			  squareShapeB.SetAsBox(10.3,0.5);
			  
			  
			 
			  bodyDef.position.Set(-30.5, 15);
			  fixtureDef2.shape = &squareShapeA;
			  m_lift3 = m_world->CreateBody( &bodyDef );
			  m_lift3->CreateFixture( &fixtureDef2 );
			  
			
			  bodyDef2.position.Set( -28, 20);
			  fixtureDef.shape = &squareShapeB;
			  m_wall3 = m_world->CreateBody( &bodyDef2 );
			  m_wall3->CreateFixture( &fixtureDef );
			  
			  b2PrismaticJointDef prismaticJointDef;
			  prismaticJointDef.bodyA = m_lift3;
			  prismaticJointDef.bodyB = m_wall3;
			  prismaticJointDef.collideConnected = false;
			  prismaticJointDef.localAxisA.Set(0,1);
			  prismaticJointDef.localAnchorA.Set( 0.5,-13.5);//a little outside the bottom right corner
			  prismaticJointDef.localAnchorB.Set( -10.3, 0);//bottom left corner
			  prismaticJointDef.enableLimit = true;
			  prismaticJointDef.lowerTranslation = 18;
			  prismaticJointDef.upperTranslation = 28;
			  prismaticJointDef.enableMotor = true;
			  prismaticJointDef.maxMotorForce = 10000;
			  m_joint_lift3 = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
		}
		
			{
			//body and fixture defs - the common parts
			  //~ b2BodyDef bodyDef;
			  //~ //bodyDef.type = b2_dynamicBody;
			   b2FixtureDef fixtureDef;
			   fixtureDef.density = 3;
			  //~ 
			  b2BodyDef bodyDef2;
			  bodyDef2.type = b2_dynamicBody;
			  
			  //two boxes
			  //~ b2PolygonShape squareShapeA;
			  //~ squareShapeA.SetAsBox(10,0.5);
			  	
			   
			  b2PolygonShape squareShapeB;
			  squareShapeB.SetAsBox(3,1);

			  bodyDef2.position.Set( 6, 21);
			  fixtureDef.shape = &squareShapeB;
			  m_bodyB2 = m_world->CreateBody( &bodyDef2 );
			  m_bodyB2->CreateFixture( &fixtureDef );
			  
			  b2PrismaticJointDef prismaticJointDef;
			  prismaticJointDef.bodyA = m_lift;
			  prismaticJointDef.bodyB = m_bodyB2;
			  prismaticJointDef.collideConnected = false;
			  prismaticJointDef.localAxisA.Set(1,0);
			  prismaticJointDef.localAnchorA.Set( 0,14);//a little outside the bottom right corner
			  prismaticJointDef.localAnchorB.Set( 0, 2);//bottom left corner
			  prismaticJointDef.enableLimit = true;
			  prismaticJointDef.lowerTranslation = -15;
			  prismaticJointDef.upperTranslation = 3;
			  prismaticJointDef.enableMotor = true;
			  prismaticJointDef.maxMotorForce = 100000;
			  m_joint2 = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
		}
		
		{
			//body and fixture defs - the common parts
			  //~ b2BodyDef bodyDef;
			  //~ //bodyDef.type = b2_dynamicBody;
			   b2FixtureDef fixtureDef;
			   fixtureDef.density = 1;
			   //fixtureDef.friction = 100;
			  //~ 
			  b2BodyDef bodyDef2;
			  bodyDef2.type = b2_dynamicBody;
			  
			  //two boxes
			  //~ b2PolygonShape squareShapeA;
			  //~ squareShapeA.SetAsBox(10,0.5);
			  	
			   
			  b2PolygonShape squareShapeB;
			  squareShapeB.SetAsBox(3,1);

			  bodyDef2.position.Set(-30.5,15);
			  fixtureDef.shape = &squareShapeB;
			  m_bodyB3 = m_world->CreateBody( &bodyDef2 );
			  m_bodyB3->CreateFixture( &fixtureDef );
			  
			  b2PrismaticJointDef prismaticJointDef;
			  prismaticJointDef.bodyA = m_lift3;
			  prismaticJointDef.bodyB = m_bodyB3;
			  prismaticJointDef.collideConnected = false;
			  prismaticJointDef.localAxisA.Set(1,0);
			  prismaticJointDef.localAnchorA.Set( 0,16);//a little outside the bottom right corner
			  prismaticJointDef.localAnchorB.Set( -2, -0.5);//bottom left corner
			  prismaticJointDef.enableLimit = true;
			  prismaticJointDef.lowerTranslation = -5;
			  prismaticJointDef.upperTranslation = 20;
			  prismaticJointDef.enableMotor = true;
			  prismaticJointDef.maxMotorForce = 100000;
			  m_joint3 = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
		}
		
		{
			//body and fixture defs - the common parts
			  b2BodyDef bodyDef;
			  bodyDef.type = b2_dynamicBody;
			  b2FixtureDef fixtureDef;
			  fixtureDef.density = 1;
			  
			  b2BodyDef bodyDef2;
			  //bodyDef2.type = b2_dynamicBody;
			  b2FixtureDef fixtureDef2;
			  fixtureDef2.density = 1000;
			  fixtureDef2.friction = 2;
			  //two boxes
			  b2PolygonShape squareShapeA;
			  squareShapeA.SetAsBox(1,1);
			  
			  b2PolygonShape squareShapeB;
			  squareShapeB.SetAsBox(10,0.1);
			  
			  
			 
			  bodyDef.position.Set(1, 29.8);//dynamic base
			  fixtureDef2.shape = &squareShapeB;
			  gunbase = m_world->CreateBody( &bodyDef );
			  gunbase->CreateFixture( &fixtureDef2 );
			  
			
			  bodyDef2.position.Set( 0, 31.8);
			  fixtureDef.shape = &squareShapeA;
			  gunanch = m_world->CreateBody( &bodyDef2 );
			  //gunanch->CreateFixture( &fixtureDef );
			  
			  b2RevoluteJointDef revoluteJointDef;
			  revoluteJointDef.bodyA = gunanch;
			  revoluteJointDef.bodyB = gunbase;
			  revoluteJointDef.collideConnected = false;
			  revoluteJointDef.localAnchorA.Set( 0,0);//a little outside the bottom right corner
			  revoluteJointDef.localAnchorB.Set( -1, 2);//bottom left corner
			  revoluteJointDef.referenceAngle = 0;
			  revoluteJointDef.enableLimit = true;
			  revoluteJointDef.lowerAngle = 0 * DEGTORAD;
			  revoluteJointDef.upperAngle =  20 * DEGTORAD;
			  revoluteJointDef.enableMotor = true;
			  revoluteJointDef.maxMotorTorque = 200000;
			  m_base = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef );
		}
		
		{
			//body and fixture defs - the common parts
			  b2BodyDef bodyDef;
			  bodyDef.type = b2_dynamicBody;
			  b2FixtureDef fixtureDef;
			  fixtureDef.density = 1000;
			  
			  
			  //b2BodyDef bodyDef2;
			  //bodyDef2.type = b2_dynamicBody;
			  //b2FixtureDef fixtureDef2;
			  //fixtureDef2.density = 1;
			  
			  //b2PolygonShape squareShapeA;
			  //squareShapeA.SetAsBox(1,1);
			  
			  b2PolygonShape squareShapeB;
			  squareShapeB.SetAsBox(10,0.1);
			  
			  
			 
			  bodyDef.position.Set(1, 33.8);//dynamic base
			  fixtureDef.shape = &squareShapeB;
			  guntop = m_world->CreateBody( &bodyDef );
			  guntop->CreateFixture( &fixtureDef );
			  
			
			  //bodyDef2.position.Set( 0, 37);
			  //fixtureDef.shape = &squareShapeB;
			  //gunanch1 = m_world->CreateBody( &bodyDef2 );
			  //gunanch1->CreateFixture( &fixtureDef );
			  
			  b2RevoluteJointDef revoluteJointDef;
			  revoluteJointDef.bodyA = gunanch;
			  revoluteJointDef.bodyB = guntop;
			  revoluteJointDef.collideConnected = false;
			  revoluteJointDef.localAnchorA.Set( 0,0);//a little outside the bottom right corner
			  revoluteJointDef.localAnchorB.Set( -1, -2);//bottom left corner
			  revoluteJointDef.referenceAngle = 0;
			  revoluteJointDef.enableLimit = true;
			  revoluteJointDef.lowerAngle = 0 * DEGTORAD;
			  revoluteJointDef.upperAngle =  20 * DEGTORAD;
			  revoluteJointDef.enableMotor = true;
			  revoluteJointDef.maxMotorTorque = 200000;
			  m_top = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef );
		}
		
		
		
		
	}
    

    void dominos_t::keyboard(unsigned char key)
    {
        switch (key)
		{
		case 'a':
			m_joint->SetMotorSpeed(-4.0f);
			break;

		case 'd':
			m_joint->SetMotorSpeed(4.0f);
			break;
		case 'q':
			m_joint1->SetMotorSpeed(-4.0f);
			break;

		case 'e':
			m_joint1->SetMotorSpeed(4.0f);
			break;			
		case 'w':
			m_joint_lift->SetMotorSpeed(2.0f);
			break;

		case 's':
			m_joint_lift->SetMotorSpeed(-2.0f);
			break;
		case '1':
			m_joint_lift->SetMotorSpeed(0.0f);
			break;
		case 'i':
			m_joint_lift3->SetMotorSpeed(2.0f);
			break;
		case 'k':
			m_joint_lift3->SetMotorSpeed(-2.0f);
			break;
		case 'j':
			m_joint2->SetMotorSpeed(-2.0f);
			break;
		case 'l':
		m_joint2->SetMotorSpeed(2.0f);
		break;		
		
		case 'o':
		m_joint3->SetMotorSpeed(2.0f);
		break;
		case 'u':
		m_joint3->SetMotorSpeed(-2.0f);
		break;
		
		case '=':
		m_base->SetMotorSpeed(0.5f);
		m_top->SetMotorSpeed(0.5f);
		break;
		case '-':
		m_base->SetMotorSpeed(-0.5f);
		m_top->SetMotorSpeed(-0.5f);
		break;
		
		 case ' ':
        //apply immediate force upwards
       //const b2Vec2 im=(0,50);
       // m_cone->ApplyLinearImpulse( b2Vec2(0.9396926*50,0.342020*50), m_cone->GetWorldCenter(),true );
       m_cone->ApplyLinearImpulse( b2Vec2(cos(20 * DEGTORAD)*50,sin(20 * DEGTORAD)*50), m_cone->GetWorldCenter(),true );
        for(int i=0;i<4;i++)
		{
				domin[i]->ApplyLinearImpulse( b2Vec2(cos(20 * DEGTORAD)*50,sin(20 * DEGTORAD)*50), domin[i]->GetWorldCenter(),true );
		}
          break;					
					//~ 
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

