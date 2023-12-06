#include "RigidBody.h"

void RigidBody::add_force(Vec3 force)
{
	this->forces += force;

	for (Vec3 point : this->body_points) {
		this->torque += GamePhysics::cross(point, force);
	}
}

std::vector<Vec3> RigidBody::get_world_points()
{
	std::vector<Vec3> world_points;
	Mat4 rotational_matrix = this->orientation.getRotMat();
	
	for (Vec3 point : this->body_points) {
		world_points.emplace_back(rotational_matrix.transformVector(point));
	}

	return world_points;
}

RigidBody RigidBody::makeQuad(float width, float height, float depth, Vec3 position, Mat4 initialRotation)
{
	RigidBody body;

	body.body_points = std::vector<Vec3>{
		Vec3(position.x + width / 2, position.y + height / 2, position.z + depth / 2),
		Vec3(position.x - width / 2, position.y + height / 2, position.z + depth / 2),
		Vec3(position.x + width / 2, position.y - height / 2, position.z + depth / 2),
		Vec3(position.x - width / 2, position.y - height / 2, position.z + depth / 2),
		Vec3(position.x + width / 2, position.y + height / 2, position.z - depth / 2),
		Vec3(position.x - width / 2, position.y + height / 2, position.z - depth / 2),
		Vec3(position.x + width / 2, position.y - height / 2, position.z - depth / 2),
		Vec3(position.x - width / 2, position.y - height / 2, position.z - depth / 2)
	};

	body.scale_matrix.initScaling(width, height, depth);
	body.center_of_mass = position;
	body.orientation = Quaternion(initialRotation);

	body.forces = 0;
	body.torque = 0;
	body.linear_velocity = 0;
	body.angular_velocity = 0;
	body.angular_momentum = 0;

	return body;
}

Mat4 RigidBody::getBodyToWorld()
{
	Mat4 translation_matrix;
	translation_matrix.initTranslation(this->center_of_mass.x, this->center_of_mass.y, this->center_of_mass.z);

	return this->scale_matrix * this->orientation.getRotMat() * translation_matrix;
}

void RigidBody::do_step(float h)
{
	this->center_of_mass += h * this->linear_velocity;
	this->linear_velocity += h * (this->forces / this->mass);

	Quat temp = Quat(0, this->angular_velocity[0], this->angular_velocity[1], this->angular_velocity[2]);

	this->orientation += (h / 2) * temp * this->orientation;

	this->angular_momentum += h * this->torque;

	Mat4 rotational_matrix = this->orientation.getRotMat();
	Mat4 inverse_rotational_matrix = this->orientation.getRotMat();
	inverse_rotational_matrix.transpose();
	Mat4 inverse_inertia_tensor = rotational_matrix * this->inverse_base_inertia_tensor * inverse_rotational_matrix;
	this->angular_velocity = inverse_inertia_tensor.transformVector(this->angular_momentum);
}