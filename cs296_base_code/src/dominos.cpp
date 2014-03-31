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
  /*! This is the constructor 
    
	This is the documentation block for the constructor.
   */ 
  
  dominos_t::dominos_t()
  {
    //!<b> Ground: </b>
     
    /** b1 is a pointer to the body ground */  
    b2Body* b1;  	
    {
     /** \brief shape is an object of class b2Edgeshape
	
	The endpoints of shape (edge) are specified using Set function
	*/
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
          
    //! <b> Top horizontal shelf: </b>
    {
	/*! \brief shape is an object of class b2Polygonshape
	
	The shape is set to a box whose dimensions are specified by calling Set function
	*/
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	//! bd contains the definition of a body
	//! position of bd is specified using Set function
      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
	//! CreateBody(&bd) creates a body with definition bd
      b2Body* ground = m_world->CreateBody(&bd);
	//CreateFixture attches the specified fixture to the specified body
      ground->CreateFixture(&shape, 0.0f);
    }

    //! <b> Dominos: </b>
    {
	
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	/** \brief fd is an object of class b2FixtureDef
	
	It is used to define a fixture.
	In the definition, various properties linke density, friction and restitution can be set	
	*/	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
	/** In the loop 10 dynamic bodies are created whose positions are specified.
	    Fixture fd is attached to all the bodies	
	*/	
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }
      
    //! <b> Another horizontal shelf: </b>
    {
	/** \brief shape is an object of class b2Polygonshape
	
	The shape is set to a box whose dimensions are specified by calling Set function
	*/		
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
	
      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


    //! <b>The pendulum that knocks the dominos off: </b>
    {
	//! b2 & b4 are a pointer to the objects of type b2Body 			
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);
	  
	b2BodyDef bd;
	bd.position.Set(-36.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }
	//! \brief jd is an object of type b2RevoluteJointDef
	//! jd is used to define a Revolute joint
      b2RevoluteJointDef jd;
	//! \brief anchor is a variable of type b2Vec2
	//! It stores two floating point numbers which are specified using Set()
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
	//! \brief jd.initialise(b2,b4,anchor) initialises the joint jd
	
	//! b2 and b4 are the bodies attached to the joint at the coordinates specified by anchor
      jd.Initialize(b2, b4, anchor);
	//!CreateJoint(&jd) makes a joint with the definition specified in jd
      m_world->CreateJoint(&jd);
    }
      
    //! <b> The train of small spheres: </b>
    {
	//! spherebody is a pointer to objects of class b2Body
      b2Body* spherebody;
	//! \brief circle is an object of Class b2CircleShape
	
	//! circle.m_radius is used to set the radius of circle
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	//! In the loop 10 circular dunamic bodies are created with body definition ballbd
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }

    //! <b>The pulley system: </b>
    {
	//! \brief bd is a pointer to object of class b2BodyDef
	
	//! It contains the definition of body. 
	//! Setting fixedRotation to be true in the definition will disable any rotational motion of dynamic body
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = true;
      //! The open box:
	
	/** \brief fd1, fd2 and fd3 are pointers to object of class b2FixtureDef
	 
	These are used to define fixtures
	 
	SetAsBox() function is used to make box with parameters width,height,center coordinates,angle	*/
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       //! box1 is a pointer to an object of class b2Body. 
	//! This body has Fixtures fd1,fd2 and fd3 attached to it
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //! The bar:
	
	//! density is used to define density in kg/m^2
      bd->position.Set(10,15);	
      fd1->density = 34.0;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      //! The pulley joint:

	//! myjoint is a pointer to object of class b2PulleyJointDef
	
	//! It contains the definition for a pulley
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-10, 15); //! worldAnchorOnBody1 specifies the coordinates of anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 15); //! worldAnchorOnBody2 specifies the coordinates of anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-10, 20); //! worldAnchorGround1 specifies the coordinates of anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(10, 20); //! worldAnchorGround2 specifies the coordinates of anchor point for ground 2 in world axis
      float32 ratio = 1.0f; //! ratio is used to set the ratio of speeds of the 2 bodies on either side of the pulley
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);//! Initialise is used to initialise the pulley joint with the given parameters
      m_world->CreateJoint(myjoint);
    }

    //! <b> The revolving horizontal platform: </b>
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
	
      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);
	//! body2 is a static body which holds the joint. It is not having any fixture attached to it
	
	//! 'body' is the body of revolving platform with fixture fd attached to it
      b2RevoluteJointDef jointDef;//! jointDef contains the definition of a revolute joint
      jointDef.bodyA = body;//! .bodyA and .bodyB specifies the 2 bodies attached to the joint
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);//! localAnchorA.Set() and localAnchorB.Set() specifies the points in body A and B respectively around which they will rotate
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;//! collideConnected is set to False which will restrict the connected bodies to collide with each other.
      m_world->CreateJoint(&jointDef);
    }

    //! <b>The heavy sphere on the platform: </b>
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;//! ballfd.friction is used to set the friction value of the fixture
      ballfd.restitution = 0.0f;//! ballfd.restitution is used to set the restitution calue of the fixture 
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


    //! <b>The see-saw system at the bottom:</b>
    {
      //! The triangle wedge:
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);//! poly.Set() function is used to make a triangle here and takes an array of coordinates of vertices and number of vertices as parameters
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(15.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(30.0f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;	
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(30.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
	//! All the other elements uses the commands that have been already explained above
    }
      //! <b> New Wall: </b>
      {
	b2Body* newWall;//! newWall is a pointer to object of class b2Body
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 4.0f);//! New Wall has a rectangular shape of specified dimension at a specified position
	  
	b2BodyDef bdnew;//! bdnew contains the definition of a body
	bdnew.position.Set(-40.0f, 4.0f);
	
	b2FixtureDef wallfd;
      	wallfd.shape = &shape;
      	wallfd.density = 10.0f;
      	wallfd.restitution = 1.0f;//! Restitution coefficient of wallfd fixture is set to 1
	newWall = m_world->CreateBody(&bdnew);
	newWall->CreateFixture(&wallfd);//! wallfd fixture is attached to newWall body
      }
       //! <b> New Spehere: </b>
    {
      b2Body* newsphere;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	//! This sphere has same properties as the above specified heavy sphere except that it has coefficient of restitution=1
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 1.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-25.0f, 0.0f);
      newsphere = m_world->CreateBody(&ballbd);
      newsphere->CreateFixture(&ballfd);
    }
    //! <b>New wedge: </b>
    {
      b2Body* newWedge;//! newWedge is a pointer to object of class b2Body
      b2PolygonShape poly; 
      b2Vec2 vertices[3];
      vertices[0].Set(-7,0);
      vertices[1].Set(7,0);
      vertices[2].Set(0,0.25);
      poly.Set(vertices, 3);//! poly is a shape (triangle) with specified vertices (coordinates) and no. of sides
      b2FixtureDef wedgefd;//! wedgefd is a fixture definition
      wedgefd.shape = &poly;//! we have set the density, friction and restitution value in the definition of wedgefd
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-5.0f, 0.0f);
      newWedge = m_world->CreateBody(&wedgebd);
      newWedge->CreateFixture(&wedgefd);//! fixture with definition wedgefd is attached to newWedge
    }

  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
