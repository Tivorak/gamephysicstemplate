#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include "Simulator.h"

class RigidBody {

public:

	Vec3 center_of_mass;
	Vec3 linear_velocity;
	Vec3 angular_velocity;
	Vec3 angular_momentum;
	Quat orientation;
	float mass;

	void add_force(Vec3 force);
	std::vector<Vec3> get_world_points();

	void do_step(float h);

	static RigidBody makeQuad(float width, float height, float depth, Vec3 position, Mat4 initialRotation);
	Mat4 getBodyToWorld();

private:
	RigidBody(std::vector<Vec3> mass_points, std::vector<Vec3> point_masses, Mat4 scale_matrix);
	RigidBody();

	Mat4 inverse_base_inertia_tensor;
	std::vector<Vec3> body_points;
	Vec3 forces;
	Vec3 torque;
	Mat4 scale_matrix;

};
#endif // RIGIDBODY_H
