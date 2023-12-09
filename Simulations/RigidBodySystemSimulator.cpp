#include "RigidBodySystemSimulator.h"
#include "RigidBody.h"
#include "collisionDetect.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	this->m_iTestCase = 1;
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

	DUC->beginLine();
	// Intended
	DUC->drawLine(loc, Vec3(1.), loc + force, Vec3(1.));
	DUC->drawLine(inputLoc, Vec3(0, 1, 0), inputLoc + 5 * inputDirection, Vec3(0, 0, 1));

	// Actual
	Vec3 temp_loc(0.1, 0.2, -0.1);
	Vec3 temp_force(0, 0, 200);
	DUC->drawLine(temp_loc, Vec3(1, 0, 0), temp_loc + temp_force, Vec3(1, 0, 0));
	DUC->endLine();

	for(RigidBody& body : bodies)
	{
		DUC->drawRigidBody(body.getBodyToWorld());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	bodies.clear();

	switch (testCase)
	{
	case 0:
		{
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI_2));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));

		// addRigidBody(Vec3(0.), Vec3(1, 1, 1), 2);
		// setOrientationOf(0, Quat(Vec3(0, 1, 0), M_PI_2));
		// applyForceOnBody(0, Vec3(-1, 0, 0), Vec3(0, 0, 100));


		//addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		//addRigidBody(Vec3(0.), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		//Vec3 loc(0.0, 0.0f, 0.0);
		//Vec3 force(0, 0, 200);
		//applyForceOnBody(0, loc, force);
		break;
		}
	case 1:
		{
		addRigidBody(Vec3(0.25, 1, 0), Vec3(1, 0.5, 1), 1);
		addRigidBody(Vec3(-0.25, 0, 0), Vec3(1, 0.5, 1), 1);
		applyForceOnBody(0, Vec3(0.25, 1, 0), Vec3(0, -50, 0));
		break;
		}
	case 2:
		{
		for (int i = 0;i < 5;i++)
		{
			for (int j = 0;j < 5;j++)
			{
				addRigidBody(Vec3(0.5 * i, 0.5 * j, 0), Vec3(0.25, 0.25, 0.1), 1);
			}
		}
		break;
		}
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
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
		for (int j = i + 1;j < bodies.size();j++)
		{
			RigidBody& body_a = bodies[i];
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
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	cout << "World: " << Mat4(DUC->g_camera.GetWorldMatrix()) << endl;
	cout << "View: " << Mat4(DUC->g_camera.GetViewMatrix()) << endl;
	cout << "Proj: " << Mat4(DUC->g_camera.GetProjMatrix()) << endl;
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
