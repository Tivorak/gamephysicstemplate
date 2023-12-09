#include "RigidBodySystemSimulator.h"
#include "RigidBody.h"
#include "collisionDetect.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	this->m_iTestCase = 0;
	//notifyCaseChanged(this->m_iTestCase);
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo1,Demo2,Demo3,Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
}

Vec3 inputLoc, inputDirection;

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1), 2000.0, Vec3(0.5));
	Vec3 loc(0.0, 0.0f, 0.0);
	Vec3 force(0, 0, 200);
	/*
	DUC->beginLine();
	// Intended
	DUC->drawLine(loc, Vec3(1.), loc + force, Vec3(1.));
	DUC->drawLine(inputLoc, Vec3(0, 1, 0), inputLoc + 5 * inputDirection, Vec3(0, 0, 1));

	// Actual
	Vec3 temp_loc(0.1, 0.2, -0.1);
	Vec3 temp_force(0, 0, 200);
	DUC->drawLine(temp_loc, Vec3(1, 0, 0), temp_loc + temp_force, Vec3(1, 0, 0));
	DUC->endLine();
	*/

	for (RigidBody& body : bodies)
	{
		DUC->drawRigidBody(body.getBodyToWorld());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	this->m_iTestCase = testCase;
	bodies.clear();

	switch (testCase)
	{
	case 0:
	{
		//simulate Timestep of 2
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		this->bodies[0].print();
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI_2));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		simulateTimestep(2);
		this->bodies[0].print();

		//reset and simulate normally
		bodies.clear();
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI_2));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		break;
	}
	case 1:
	{
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI_2));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		break;
	}
	case 2:
	{
		addRigidBody(Vec3(0.25, 1, 0), Vec3(1, 0.5, 1), 1);
		addRigidBody(Vec3(-0.25, 0, 0), Vec3(1, 0.5, 1), 1);
		applyForceOnBody(0, Vec3(0.25, 1, 0), Vec3(0, -50, 0));
		break;
	}
	case 3:
	{
		Vec3 size(0.5, 0.5, 0.5);
		double mass = 1;
		//middle top
		addRigidBody(Vec3(0, 2, 0),  size, mass);
		//left
		addRigidBody(Vec3(-1, 1, -1), size, mass);
		addRigidBody(Vec3(-1, 1, 1), size, mass);
		//right
		addRigidBody(Vec3(1, 1, -1), size, mass);
		addRigidBody(Vec3(1, 1, 1), size, mass);
		Vec3 pushRightBack(0.5, 0, -0.5);
		Vec3 pushLeftBack(-0.5, 0, -0.5);
		Vec3 pushRightFront(0.5, 0, 0.5);
		Vec3 pushLeftFront(-0.5, 0, 0.5);
		Vec3 pushDown(0, -0.5, 0);
		double factor = 20;
		applyForceOnBody(0, this->bodies[0].center_of_mass - pushDown, pushDown*factor);
		applyForceOnBody(1, this->bodies[1].center_of_mass - pushRightFront, pushRightFront * factor);
		applyForceOnBody(2, this->bodies[2].center_of_mass - pushRightBack, pushRightBack * factor);
		applyForceOnBody(3, this->bodies[3].center_of_mass - pushLeftFront, pushLeftFront * factor);
		applyForceOnBody(4, this->bodies[4].center_of_mass - pushLeftBack, pushLeftBack * factor);
		break;
	}
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	if (this->m_iTestCase != 3)
		return;

	Vec3 gravity(0, -0.981, 0);

	for (int i = 0;i < bodies.size();i++)
	{
		RigidBody& body = bodies[i];
		Vec3 loc = body.center_of_mass + Vec3(0,0.5,0);
		applyForceOnBody(i, loc, gravity * body.mass * timeElapsed);
	}
}

ostream& operator<<(ostream& os, const RigidBody& body)
{
	os << "RigidBody at " << body.center_of_mass << " with linear velocity " << body.linear_velocity << ", angular velocity " << body.angular_velocity << " and mass " << body.mass;
	return os;
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (RigidBody& body : bodies)
	{
		body.do_step(timeStep);
	}

	for (int i = 0;i < bodies.size();i++)
	{
		RigidBody& body_a = bodies[i];
		for (int j = i + 1;j < bodies.size();j++)
		{
			RigidBody& body_b = bodies[j];

			CollisionInfo info = checkCollisionSAT(body_a.getBodyToWorld(), body_b.getBodyToWorld());

			if (!info.isValid) // Bodies are not colliding: do nothing
				continue;

			Vec3 vel_a = body_a.getWorldVelocityAt(info.collisionPointWorld);
			Vec3 vel_b = body_b.getWorldVelocityAt(info.collisionPointWorld);

			Vec3 v_rel = vel_a - vel_b;

			float v_rel_dot_n = dot(v_rel, info.normalWorld);

			if (v_rel_dot_n > 0) // Bodies are already separating: do nothing
				continue;

			Vec3 x_a = body_a.getWorldToBody().transformVector(info.collisionPointWorld);
			Vec3 x_b = body_b.getWorldToBody().transformVector(info.collisionPointWorld);

			Vec3 bodyAImpulse = body_a.calcImpulse(info.collisionPointWorld, info.normalWorld);
			Vec3 bodyBImpulse = body_b.calcImpulse(info.collisionPointWorld, info.normalWorld);

			float impulse = -v_rel_dot_n / (1 / body_a.mass + 1 / body_b.mass + dot(bodyAImpulse + bodyBImpulse, info.normalWorld));

			body_a.linear_velocity = body_a.linear_velocity + (info.normalWorld * impulse) / body_a.mass;
			body_b.linear_velocity -= (impulse * info.normalWorld) / body_b.mass;

			body_a.angular_momentum += cross(x_a, impulse * info.normalWorld);
			body_b.angular_momentum -= cross(x_b, impulse * info.normalWorld);
		}

		if(this->m_iTestCase != 3 || body_a.center_of_mass.y > -1)
			continue;

		body_a.linear_velocity = -body_a.linear_velocity;
	}

}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	if (this->m_iTestCase != 1)
		return;
	//use jedi powers
	//apply force to all bodies in the direction of the camera
	Vec3 eyePoint = Vec3(DUC->g_camera.GetEyePt());

	for (int i = 0;i < bodies.size();i++)
	{
		RigidBody& body = bodies[i];
		Vec3 center = body.center_of_mass;
		Vec3 force = (center - eyePoint);
		double size = sqrt(force.x * force.x + force.y * force.y + force.z * force.z);
		force /= size;
		Vec3 loc = center + force * 0.5;
		applyForceOnBody(i, loc, force);
	}
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return bodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return bodies[i].center_of_mass;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return bodies[i].linear_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return bodies[i].angular_velocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	bodies[i].add_force(force, loc);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	bodies.emplace_back(RigidBody::makeQuad(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	bodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	bodies[i].linear_velocity = velocity;
}
