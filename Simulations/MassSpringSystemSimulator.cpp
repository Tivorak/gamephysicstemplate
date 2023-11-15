#include "MassSpringSystemSimulator.h"



MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iIntegrator = none;
	this->mass_points = std::vector<MassPoint>();
	this->springs = std::vector<Spring>();
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo1,Demo2,Demo3,Demo4,Test";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	constexpr TwEnumVal integrators[5] = {
		{
			euler,
			"Euler"
		},
		{
			midpoint,
			"MidPoint"
		},
		{
			leapfrog,
			"LeapFrog"
		},
		{
			rk4,
			"RK4"
		},
		{
			none,
			"None"
		}
	};
	const TwType integratorsType = TwDefineEnum("Integrators", integrators, 5);

	switch (m_iTestCase)
	{
	case 1:
	case 2: // Demos that use a fixed time step
		TwAddVarRO(DUC->g_pTweakBar, "Fixed Time Step", TW_TYPE_FLOAT, &this->timestep_override, nullptr);
		TwAddVarRO(DUC->g_pTweakBar, "Integrator", integratorsType, &this->m_iIntegrator, nullptr);
		break;
	case 3: // Demos that can have their time step and integrator changed
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", integratorsType, &this->m_iIntegrator, nullptr);
		TwAddVarRW(DUC->g_pTweakBar, "horizontal points", TW_TYPE_UINT16, &this->numHorizontalPoints, nullptr);
		TwAddVarRW(DUC->g_pTweakBar, "vertical points", TW_TYPE_UINT16, &this->numVerticalPoints, nullptr);
		TwAddVarRW(DUC->g_pTweakBar, "external force", TW_TYPE_DIR3F, &this->m_externalForce, nullptr);
		TwAddVarRW(DUC->g_pTweakBar, "gravity", TW_TYPE_FLOAT, &this->gravity, nullptr);
		TwAddVarRW(DUC->g_pTweakBar, "damping factor", TW_TYPE_FLOAT, &this->m_fDamping, nullptr);
		TwAddVarRW(DUC->g_pTweakBar, "stiffness", TW_TYPE_FLOAT, &this->m_fStiffness, nullptr);
		TwAddVarRW(DUC->g_pTweakBar, "mass", TW_TYPE_FLOAT, &this->m_fMass, nullptr);
		break;
	}
}

void MassSpringSystemSimulator::reset()
{
	this->mass_points.clear();
	this->springs.clear();
	this->m_iIntegrator = none;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (auto& p : this->mass_points) {
		DUC->setUpLighting(Vec3(), Vec3(0.4), 100, p.is_fixed ? Vec3(0, 0, 0.3) : Vec3(0.3, 0, 0));
		DUC->drawSphere(p.position, this->mass_points.size() > 10 ? Vec3(0.01) : Vec3(0.1));
	}

	for (auto& s : this->springs) {
		DUC->beginLine();
		DUC->drawLine(this->mass_points.at(s.mass_point_1).position, Vec3(1), this->mass_points.at(s.mass_point_2).position, Vec3(1));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	this->reset();

	m_iTestCase = testCase;
	switch (testCase)
	{
	case 0: // Demo 1
	{
		cout << "Selecting Demo 1" << std::endl;
		this->m_fDamping = 1;
		this->m_fStiffness = 40;
		this->m_fMass = 10;

		int p0 = this->addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		int p1 = this->addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		this->addSpring(p0, p1, 1);

		std::cout << this->mass_points.at(p0) << std::endl;
		std::cout << this->mass_points.at(p1) << std::endl;
		std::cout << this->springs.at(0) << std::endl;

		this->stepEuler(0.1);
		std::cout << "Euler Result:" << std::endl;
		std::cout << this->mass_points.at(p0) << std::endl;
		std::cout << this->mass_points.at(p1) << std::endl;

		this->mass_points = std::vector<MassPoint>();
		this->springs = std::vector<Spring>();

		p0 = this->addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		p1 = this->addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		this->addSpring(p0, p1, 1);

		this->stepMidpoint(0.1);
		std::cout << "MidPoint Result:" << std::endl;
		std::cout << this->mass_points.at(p0) << std::endl;
		std::cout << this->mass_points.at(p1) << std::endl;

		break;
	}
	case 1: // Demo 2
	{
		cout << "Selecting Demo 2" << std::endl;
		int p0 = this->addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		int p1 = this->addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		this->addSpring(p0, p1, 1);
		this->m_iIntegrator = euler;
		this->timestep_override = 0.1;
		this->m_fDamping = 1;
		this->m_fStiffness = 40;
		this->m_fMass = 10;
		break;
	}
	case 2: // Demo 3
	{
		cout << "Selecting Demo 3" << std::endl;
		int p0 = this->addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		int p1 = this->addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		this->addSpring(p0, p1, 1);
		this->m_iIntegrator = midpoint;
		this->timestep_override = 0.1;
		this->m_fDamping = 1;
		this->m_fStiffness = 40;
		this->m_fMass = 10;
		break;
	}
	case 3: // Demo 4
	{
		this->m_fDamping = 1;
		this->m_fStiffness = 500;
		this->m_fMass = 1;
		this->gravity = -9.81;
		const Vec3 start = Vec3(-0.5, 1.5, 0);

		const float horizontalSize = 1;
		const float verticalSize = 2;

		Vec3 horizontalStep = Vec3(horizontalSize / this->numHorizontalPoints, 0, 0);
		Vec3 verticalStep = -Vec3(0, 0, verticalSize / this->numVerticalPoints);

		for (int i = 0; i < this->numHorizontalPoints;i++)
		{
			for (int j = 0;j < this->numVerticalPoints;j++)
			{
				int thisIndex =  this->addMassPoint(start + horizontalStep * i + verticalStep * j, Vec3(0, 0, 0), j == 0);

				int leftIndex = (i - 1) * this->numVerticalPoints + j; // The point to the left of this, already present as i - 1 < i

				if (leftIndex >= 0 && leftIndex < this->mass_points.size())
				{
					this->addSpring(leftIndex, thisIndex, norm(horizontalStep));
				}

				int upIndex = i * this->numVerticalPoints + (j - 1); // The point to the top of this, already present as (j - 1) < j

				if (j != 0 && upIndex >= 0 && upIndex < this->mass_points.size())
				{
					this->addSpring(upIndex, thisIndex, norm(verticalStep));
				}

				int leftUpperIndex = (i - 1) * this->numVerticalPoints + (j - 1); // The point to the upper left of this, already present as (i - 1) < i

				if (j != 0 && leftUpperIndex >= 0 && leftUpperIndex < this->mass_points.size())
				{
					this->addSpring(leftUpperIndex, thisIndex, norm(horizontalStep + verticalStep));
				}

			}
		}

		// 	// Connect the 4 corners
		// this->addSpring(0, this->numVerticalPoints - 1); // top left to bottom left
		// this->addSpring(0, (this->numHorizontalPoints - 1) * this->numVerticalPoints); // top left to top right
		// this->addSpring((this->numHorizontalPoints - 1) * this->numVerticalPoints, (this->numHorizontalPoints * this->numVerticalPoints) - 1); // top right to bottom right
		// this->addSpring(this->numVerticalPoints - 1, (this->numHorizontalPoints* this->numVerticalPoints) - 1); // bottom left to bottom right

	}
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}



void MassSpringSystemSimulator::simulateTimestep(float time_step)
{
	if (this->m_iTestCase == 1 || this->m_iTestCase == 2)
	{
		time_step = this->timestep_override;
	}

	switch (this->m_iIntegrator)
	{
	case euler:
		this->stepEuler(time_step);
		break;
	case midpoint:
		this->stepMidpoint(time_step);
		break;
	case rk4:
		this->stepRK4(time_step);
	default:
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
}

void MassSpringSystemSimulator::setMass(float mass)
{
	this->m_fMass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	this->m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	this->m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	this->mass_points.push_back(MassPoint(position, velocity, 10, isFixed));
	return this->mass_points.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	this->springs.push_back(Spring(masspoint1, masspoint2, initialLength));
}

void MassSpringSystemSimulator::addSpring(int p0, int p1)
{
	this->springs.push_back(Spring(p0, p1, this->getPointDistance(p0, p1))); 
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return this->mass_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return this->springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return this->mass_points.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return this->mass_points.at(index).velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}

void MassSpringSystemSimulator::calculateForcesForPoints(std::vector<MassPoint>& points)
{
	// Spring force calculation:
	for (const auto& s : this->springs) {
		MassPoint& p0 = points.at(s.mass_point_1);
		MassPoint& p1 = points.at(s.mass_point_2);

		const float current_length = norm(p1.position - p0.position);

		const Vec3 force = this->m_fDamping * this->m_fStiffness * ((p1.position - p0.position) / current_length) * (current_length - s.initial_length);
		p0.force += force;
		p1.force -= force;
	}
}

Vec3 clampPointPosition(const Vec3& position)
{
	return Vec3(position.x, max(0., position.y), position.z);
}

void MassSpringSystemSimulator::stepEuler(float time_step)
{
	this->calculateForcesForPoints(this->mass_points);

	// Euler: 
	for (auto& p : this->mass_points) {

		if (!p.is_fixed) {
			p.position = clampPointPosition(p.position + time_step * p.velocity);
			p.velocity = p.velocity + time_step * (p.force / this->m_fMass);
		}

		// Reset accumulated force per frame
		p.force = Vec3(0.f, this->gravity, 0.f) + this->m_externalForce;
	}
}

void MassSpringSystemSimulator::stepMidpoint(float time_step)
{
	std::vector<MassPoint> original_points = std::vector<MassPoint>(this->mass_points);

	// Spring force calculation:
	this->calculateForcesForPoints(this->mass_points);

	// Midpoint for the half step: 
	for (auto& p : this->mass_points) {

		if (!p.is_fixed) {
			p.position = clampPointPosition(p.position + (time_step / 2) * p.velocity);
			p.velocity = p.velocity + (time_step / 2) * (p.force / this->m_fMass);
		}

		// Reset accumulated force per frame
		p.force = Vec3(0.f, this->gravity, 0.f) + this->m_externalForce;
	}

	// this->massPoints now has y~, the state of the system at x + h/2
	// now we recalculate forces for x + h/2
	// 
	// Spring force calculation:
	this->calculateForcesForPoints(this->mass_points);

	// Whole step for midpoint: 
	for (int i = 0; i < this->mass_points.size(); i++) {
		MassPoint& p = this->mass_points.at(i);
		const MassPoint& p_original = original_points.at(i);

		if (!p.is_fixed) {
			p.position = clampPointPosition(p_original.position + time_step * p.velocity);
			p.velocity = p_original.velocity + time_step * (p.force / this->m_fMass);
		}

		// Reset accumulated force per frame
		p.force = Vec3(0.f, this->gravity, 0.f) + this->m_externalForce;
	}
}

void MassSpringSystemSimulator::stepRK4(float time_step)
{
	// this is k1
	std::vector<MassPoint> original_points = std::vector<MassPoint>(this->mass_points);

	this->calculateForcesForPoints(this->mass_points);

	// Do a half-step to k2
	for (auto& p : this->mass_points) {

		if (!p.is_fixed) {
			p.position = clampPointPosition(p.position + (time_step / 2) * p.velocity);
			p.velocity = p.velocity + (time_step / 2) * (p.force / this->m_fMass);
		}

		// Reset accumulated force per frame
		p.force = Vec3(0.f, this->gravity, 0.f) + this->m_externalForce;
	}

	this->calculateForcesForPoints(this->mass_points);
	// "this" now holds k2

	std::vector<MassPoint> k2_points = std::vector<MassPoint>(this->mass_points);

	// Do a half-step using k2, from the original

	for (int i = 0; i < this->mass_points.size(); i++) {
		MassPoint& p = this->mass_points.at(i);
		const MassPoint& p_original = original_points.at(i);

		if (!p.is_fixed) {
			p.position = clampPointPosition(p_original.position + (time_step / 2) * p.velocity);
			p.velocity = p_original.velocity + (time_step / 2) * (p.force / this->m_fMass);
		}

		// Reset accumulated force per frame
		p.force = Vec3(0.f, this->gravity, 0.f) + this->m_externalForce;
	}

	this->calculateForcesForPoints(this->mass_points);

	// "this" now holds k3

	std::vector<MassPoint> k3_points = std::vector<MassPoint>(this->mass_points);

	for (int i = 0; i < this->mass_points.size(); i++) {
		MassPoint& p = this->mass_points.at(i);
		const MassPoint& p_original = original_points.at(i);

		if (!p.is_fixed) {
			p.position = clampPointPosition(p_original.position + time_step * p.velocity);
			p.velocity = p_original.velocity + time_step * (p.force / this->m_fMass);
		}

		// Reset accumulated force per frame
		p.force = Vec3(0.f, this->gravity, 0.f) + this->m_externalForce;
	}

	this->calculateForcesForPoints(this->mass_points);

	// "this" now holds k4

	std::vector<MassPoint> k4_points = std::vector<MassPoint>(this->mass_points);

	for (int i = 0; i < this->mass_points.size(); i++) {
		MassPoint& p = this->mass_points.at(i);
		const MassPoint& k1 = original_points.at(i);
		const MassPoint& k2 = k2_points.at(i);
		const MassPoint& k3 = k3_points.at(i);
		const MassPoint& k4 = k4_points.at(i);


		if (!p.is_fixed) {
			p.position = clampPointPosition(k1.position + (time_step / 6) * (k1.velocity + 2 * k2.velocity + 2 * k3.velocity + k4.velocity));
			p.velocity = k1.velocity + (time_step / 6) * (k1.force + 2 * k2.force + 2 * k3.force + k4.force) / this->m_fMass;
		}

		// Reset accumulated force per frame
		p.force = Vec3(0.f, this->gravity, 0.f) + this->m_externalForce;
	}
}

float MassSpringSystemSimulator::getPointDistance(int p0, int p1) const
{
	return norm(this->mass_points.at(p0).position - this->mass_points.at(p1).position);
}

std::ostream& operator<<(std::ostream& os, const MassPoint& p)
{
	return os << "MassPoint at " << p.position << " with velocity " << p.velocity << '.';
}

std::ostream& operator<<(std::ostream& os, const Spring& p)
{
	return os << "Spring between mass points [" << p.mass_point_1 << ", " << p.mass_point_2 << "], with initial length " << p.initial_length << '.';
}
