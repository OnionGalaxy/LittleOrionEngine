#include "ComponentCollider.h"
#include "Component/ComponentMeshRenderer.h"
#include "Main/Application.h"
#include "Main/GameObject.h"
#include "Module/ModulePhysics.h"
#include "Module/ModuleTime.h"

ComponentCollider::ComponentCollider(ColliderType collider_type) : Component(ComponentType::COLLIDER), collider_type(collider_type)
{

}

ComponentCollider::ComponentCollider(GameObject* owner, ColliderType collider_type) : Component(owner, ComponentType::COLLIDER), collider_type(collider_type)
{
	if (owner->aabb.global_bounding_box.IsFinite() && owner->aabb.global_bounding_box.Size().x != 0)
	{
		ComponentMeshRenderer* mesh = static_cast<ComponentMeshRenderer*>(owner->GetComponent(Component::ComponentType::MESH_RENDERER));
		if (mesh)
		{
			box_size = btVector3(
				owner->aabb.original_box.Size().x / 2,
				owner->aabb.original_box.Size().y / 2,
				owner->aabb.original_box.Size().z / 2);
			
		}
		else
		{
			box_size = btVector3(owner->aabb.global_bounding_box.Size().x / 2,
				owner->aabb.global_bounding_box.Size().y / 2,
				owner->aabb.global_bounding_box.Size().z / 2);
		}
		is_attached = true;
	}
}


void ComponentCollider::Copy(Component* component_to_copy) const
{
	*component_to_copy = *this;
	*static_cast<ComponentCollider*>(component_to_copy) = *this;
}


void ComponentCollider::CommonAssign(const ComponentCollider& component_to_copy)
{
	mass = component_to_copy.mass;
	scale = component_to_copy.scale;
	box_size = component_to_copy.box_size;
	freeze_rotation_x = component_to_copy.freeze_rotation_x;
	freeze_rotation_y = component_to_copy.freeze_rotation_y;
	freeze_rotation_z = component_to_copy.freeze_rotation_z;
	friction = component_to_copy.friction;
	rolling_friction = component_to_copy.rolling_friction;
	visualize = component_to_copy.visualize;
	detect_collision = component_to_copy.detect_collision;
	is_attached = component_to_copy.is_attached;
	is_static = component_to_copy.is_static;
	active_physics = component_to_copy.active_physics;
	center = component_to_copy.center;
	AddBody();
}

void ComponentCollider::Delete()
{
	App->physics->world->removeRigidBody(body);
	App->physics->RemoveComponentCollider(this);
}

void ComponentCollider::SpecializedSave(Config & config) const
{
	config.AddUInt((unsigned int)collider_type, "ColliderType");
	config.AddFloat(mass, "Mass");
	config.AddFloat3(scale, "Scale");
	config.AddBool(is_static, "Static");
	config.AddBool(detect_collision, "Collision");
	config.AddBool(visualize, "Visualize");
	config.AddBool(is_attached, "Attached");
	config.AddBool(freeze_rotation_x, "Freeze_rotation_x");
	config.AddBool(freeze_rotation_y, "Freeze_rotation_y");
	config.AddBool(freeze_rotation_z, "Freeze_rotation_z");
	config.AddBool(active_physics, "Active_Physics");
	config.AddFloat(friction, "Friction");
	config.AddFloat(rolling_friction, "Rolling_friction");
	config.AddFloat3(center, "Center");
}

void ComponentCollider::SpecializedLoad(const Config & config)
{
	mass = config.GetFloat("Mass", 1.0F);
	config.GetFloat3("Scale", scale, float3::one);
	is_static = config.GetBool("Static", false);
	detect_collision = config.GetBool("Collision", true);
	visualize = config.GetBool("Visualize", true);
	is_attached = config.GetBool("Attached", false);
	freeze_rotation_x = config.GetBool("Freeze_rotation_x", true);
	freeze_rotation_y = config.GetBool("Freeze_rotation_y", true);
	freeze_rotation_z = config.GetBool("Freeze_rotation_z", true);
	active_physics = config.GetBool("Active_Physics", false);
	friction = config.GetFloat("Friction", 1.0F);
	rolling_friction = config.GetFloat("Rolling_friction", 1.0F);
	config.GetFloat3("Center", center, float3::zero);
	AddBody();
	SetConfiguration();

}

void ComponentCollider::Disable()
{
	active = false;
	SwitchPhysics();
	SetCollisionDetection();
}

void ComponentCollider::Enable()
{
	active = true;
	SwitchPhysics();
	SetCollisionDetection();
}

btRigidBody* ComponentCollider::AddBody()
{
	if (body)
	{
		App->physics->world->removeRigidBody(body);
	}

	assert(col_shape);

	float3 global_scale = owner->transform.GetGlobalScale();
	col_shape->setLocalScaling(btVector3(global_scale.x * scale.x, global_scale.y * scale.y, global_scale.z * scale.z));

	float3 global_translation = owner->transform.GetGlobalTranslation();
	Quat global_rotation = owner->transform.GetGlobalRotation();

	btTransform new_body_transformation = btTransform(
		btQuaternion(global_rotation.x, global_rotation.y, global_rotation.z, global_rotation.w),
		btVector3(global_translation.x, global_translation.y, global_translation.z)
	);

	btTransform center_of_mass = btTransform(
		btQuaternion::getIdentity(),
		btVector3(center.x, center.y, center.z)
	);

	motion_state = new btDefaultMotionState(new_body_transformation * center_of_mass);

	if (mass != 0.f) col_shape->calculateLocalInertia(mass, local_inertia);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motion_state, col_shape, local_inertia);
	body = new btRigidBody(rbInfo);
	body->setActivationState(DISABLE_DEACTIVATION);
	body->setFriction(friction);
	App->physics->world->addRigidBody(body);
	
	if (collider_type == ComponentCollider::ColliderType::MESH) {
		SetVisualization();
	}
	
	return body;
}


void ComponentCollider::MoveBody()
{
	btTransform trans = body->getWorldTransform();

	float4x4 center_offset = float4x4::Translate(center);

	float4x4 new_global_matrix = float4x4::FromTRS(
		float3(
			trans.getOrigin().getX(),
			trans.getOrigin().getY(),
			trans.getOrigin().getZ()
		),
		Quat(
			trans.getRotation().x(),
			trans.getRotation().y(),
			trans.getRotation().z(),
			trans.getRotation().w()
		),
		float3::one
	) * center_offset.Inverted();

	owner->transform.SetGlobalModelMatrix(new_global_matrix);
}

void ComponentCollider::UpdateCommonDimensions()
{
	float3 global_translation = owner->transform.GetGlobalTranslation();
	Quat global_rotation = owner->transform.GetGlobalRotation();

	btTransform new_body_transformation = btTransform(
		btQuaternion(global_rotation.x, global_rotation.y, global_rotation.z, global_rotation.w), 
		btVector3(global_translation.x, global_translation.y, global_translation.z)
	);

	btTransform center_of_mass = btTransform(
		btQuaternion::getIdentity(),
		btVector3(center.x, center.y, center.z)
	);

	motion_state->setWorldTransform(new_body_transformation * center_of_mass);
	body->setMotionState(motion_state);
	
	App->physics->world->updateSingleAabb(body);
}

void ComponentCollider::SetMass(float new_mass)
{
	App->physics->world->removeRigidBody(body);
	body->getCollisionShape()->calculateLocalInertia(new_mass, local_inertia);
	body->setMassProps(new_mass, local_inertia);
	App->physics->world->addRigidBody(body);
}

void ComponentCollider::SetVisualization()
{
	int flags = body->getCollisionFlags();
	flags |= body->CF_DISABLE_VISUALIZE_OBJECT;
	if (visualize)
	{
		flags -= body->CF_DISABLE_VISUALIZE_OBJECT;
		
	}
	body->setCollisionFlags(flags);
}

void ComponentCollider::SetCollisionDetection()
{
	int flags = body->getCollisionFlags();
	flags |= body->CF_NO_CONTACT_RESPONSE;

	if (detect_collision && active)
	{		
		flags &= ~(body->CF_NO_CONTACT_RESPONSE);
	}

	body->setCollisionFlags(flags);
}

bool ComponentCollider::DetectCollision()
{

	int numManifolds = App->physics->world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* contactManifold = App->physics->world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();
		
		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++)
		{
			btManifoldPoint pt = contactManifold->getContactPoint(j);
			
			if (obA->getWorldArrayIndex() == body->getWorldArrayIndex() || obB->getWorldArrayIndex() == body->getWorldArrayIndex())
			{
				if (pt.getDistance() < 0.0f)
				{
					return true;
				}
			}
		}
	}
	return false;
}

bool ComponentCollider::DetectCollisionWith(ComponentCollider * collider)
{
	btVector3 body_minim;
	btVector3 body_maxim;
	btVector3 collider_minim;
	btVector3 collider_maxim;

	if (!active_physics && !collider->active_physics) 
	{	
		App->physics->world->updateAabbs();
		body->getAabb(body_minim, body_maxim);
		collider->body->getAabb(collider_minim, collider_maxim);
		
		if (!(body_maxim.getX() < collider_minim.getX() || collider_maxim.getX() < body_minim.getX()))
		{
			if (!(body_maxim.getY() < collider_minim.getY() || collider_maxim.getY() < body_minim.getY())) 
			{
				if (!(body_maxim.getZ() < collider_minim.getZ() || collider_maxim.getZ() < body_minim.getZ())) 
				{
					return true;
				}
			}
		}
	}

	/*int numManifolds = App->physics->world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* contactManifold = App->physics->world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();

		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++)
		{
			btManifoldPoint pt = contactManifold->getContactPoint(j);
			if ((obA->getWorldArrayIndex() == body->getWorldArrayIndex() && obB->getWorldArrayIndex() == collider->body->getWorldArrayIndex()) || (obB->getWorldArrayIndex() == body->getWorldArrayIndex() && obA->getWorldArrayIndex() == collider->body->getWorldArrayIndex()))
			{
				if (pt.getDistance() < 0.0f)
				{
					return true;
				}
			}
		}
	}*/
	return false;
}

ENGINE_API void ComponentCollider::ClearForces() const
{
	body->clearForces();
}

void ComponentCollider::SetStatic()
{
	int flags = body->getCollisionFlags();
	flags |= body->CF_KINEMATIC_OBJECT;
	float new_mass = 0.0F;
	if (!is_static)
	{
		flags -= body->CF_KINEMATIC_OBJECT;
		new_mass = mass;
	}
	mass = new_mass;
	body->setCollisionFlags(flags);
	
}

void ComponentCollider::SetRotationAxis()
{
	body->setAngularFactor(btVector3(int(!freeze_rotation_x), int(!freeze_rotation_y), int(!freeze_rotation_z)));
}

void ComponentCollider::AddForce(float3& force)
{
	velocity.x = force.x;
	velocity.z = force.z;
}

void ComponentCollider::SwitchPhysics(bool active)
{
	active_physics = active;
	SwitchPhysics();
}

ENGINE_API bool ComponentCollider::RaycastHit(btVector3& origin, btVector3& end)
{
	//Vector normal to the surface
	btVector3 normal;

	
	return App->physics->RaycastWorld(origin, end, normal);
	/*return App->physics->RaycastWorld(origin, end, normal);*/
}


void ComponentCollider::SwitchPhysics()
{
	if (active_physics && active)
	{
		body->forceActivationState(ACTIVE_TAG);
		body->setActivationState(DISABLE_DEACTIVATION);
	}
	else
	{
		body->forceActivationState(DISABLE_SIMULATION);
	}
}

void ComponentCollider::UpdateFriction()
{
	body->setFriction(friction);
}

void ComponentCollider::UpdatePosition()
{
	float3 transform = owner->transform.GetTranslation();

	//bottom of the model
	btVector3 bottom = body->getWorldTransform().getOrigin();
	bottom.setY(bottom.getY() - 5 * box_size.getY());
	//Vector normal to the surface
	btVector3 Normal;
	App->physics->RaycastWorld(body->getWorldTransform().getOrigin(), bottom, Normal);
	float3 normal = float3(Normal);
	normal.Normalize();

	float2 normal_2D = float2(normal.x, normal.y);
	float2 vector_vel = normal_2D.Perp();

	vector_vel.Normalize();
	IsGrounded();
	if (velocity.Length() >0)
	{
		/*velocity.Normalize();*/
		
		float3 orientation = float3(velocity.x, 0, velocity.z) * speed + transform;
		owner->transform.LookAt(orientation);

		float3 direction = transform + speed * float3(velocity.x, -SignOrZero(velocity.x)* SignOrZero(normal.x)*abs(vector_vel.y), velocity.z);

		btVector3 horizontal_ray = btVector3(direction.x - SignOrZero(body->getWorldTransform().getOrigin().getX() - direction.x), body->getWorldTransform().getOrigin().getY() + abs(vector_vel.y), direction.z - SignOrZero(body->getWorldTransform().getOrigin().getZ() - direction.z));
		bool touched = App->physics->RaycastWorld(body->getWorldTransform().getOrigin(), horizontal_ray, Normal);
		if (velocity.x == 0 && velocity.z == 0) { touched = false; }
		if (!touched || (touched && body->getWorldTransform().getOrigin().distance(horizontal_ray) > 2))
		{
			if (grounded && velocity.y==0) {
				btVector3 final_pos = btVector3(direction.x, direction.y - 3 * box_size.getY(), direction.z);
				App->physics->RaycastWorld(btVector3(direction.x, direction.y + box_size.getY(), direction.z), final_pos, Normal);
				if (abs(final_pos.getY() - transform.y) < 1) {
					owner->transform.SetGlobalMatrixTranslation(float3(direction.x, final_pos.getY(), direction.z));
				}
				else {
					owner->transform.SetGlobalMatrixTranslation(float3(direction.x, transform.y, direction.z));
				}
				UpdateDimensions();
			}
			else {
				if (!grounded || velocity.y > 0) {
					btVector3 vertical_ray = btVector3(direction.x, body->getWorldTransform().getOrigin().getY() + velocity.y + SignOrZero(velocity.y), direction.z);
					bool touched = App->physics->RaycastWorld(body->getWorldTransform().getOrigin(), vertical_ray, Normal);
					if (!touched || (touched && abs(vertical_ray.getY()- (body->getWorldTransform().getOrigin().getY()+velocity.y))> 0.3 )) {
						direction = transform + float3(speed*velocity.x*0.1, velocity.y, speed*velocity.z*0.1);
						owner->transform.SetGlobalMatrixTranslation(direction);
						UpdateDimensions();
						velocity.y = velocity.y + App->physics->gravity.y / 100;
						if (IsGrounded() && velocity.y < 0) {
							velocity.y = 0;
						}
					}
					else {
						velocity.y = 0;
					}
				}
				
			}
			
		}
		else {
			if (!grounded) {
				velocity.y = velocity.y + App->physics->gravity.y / 100;
				direction = transform + float3(0, velocity.y, 0);
				owner->transform.SetGlobalMatrixTranslation(direction);
				UpdateDimensions();
			}
			else {
				velocity.y = 0;
			}

		}
		velocity.x = 0;
		velocity.z = 0;
	}
	else {
		if (!grounded) {
			velocity.y = velocity.y + App->physics->gravity.y / 100;
			direction = transform + float3(0, velocity.y, 0);
			owner->transform.SetGlobalMatrixTranslation(direction);
			UpdateDimensions();
		}
	}
}

void ComponentCollider::SetRollingFriction()
{
	body->setRollingFriction(rolling_friction);
}

void ComponentCollider::SetConfiguration()
{
	SetStatic(); 
	SetVisualization(); 
	SetRotationAxis();
	SetCollisionDetection(); 
	SwitchPhysics(); 
	UpdateFriction();
	SetRollingFriction();
}

void ComponentCollider::SetColliderCenter(float3& new_center)
{
	UpdateCommonDimensions();
}

float3 ComponentCollider::GetColliderCenter() const
{
	return center;
}

bool ComponentCollider::IsGrounded()
{
	
	btVector3 origin = body->getWorldTransform().getOrigin();

	btVector3 end = body->getWorldTransform().getOrigin();
	end.setY(end.getY() - box_size.getY() * 3);


	if (RaycastHit(origin, end)) {
		origin.setY(origin.getY() - box_size.getY()* 0.5);
		if (abs(origin.getY() - end.getY()) < 0.3) {
			grounded = true;
			return true;
		}
	}
	grounded = false;
	return false;
	
}

void ComponentCollider::SetVelocity(float3& velocity, float speed)
{
	this->velocity = velocity;
	this->speed = speed;
	
}

void ComponentCollider::SetVelocityEnemy(float3 & velocity, float speed)
{
	//bottom of the model
	btVector3 bottom = body->getWorldTransform().getOrigin();
	bottom.setY(bottom.getY() - 200 * box_size.getY());

	//Vector normal to the surface
	btVector3 Normal;
	App->physics->RaycastWorld(body->getWorldTransform().getOrigin(), bottom, Normal);
	float3 normal = float3(Normal);
	normal.Normalize();

	float2 normal_2D = float2(normal.x, normal.y);
	float2 vector_vel = normal_2D.Perp();

	if (abs(velocity.x) > 0 || abs(velocity.z) > 0)
	{
		velocity.Normalize();
		body->setLinearVelocity(speed*btVector3(velocity.x, -SignOrZero(velocity.x)* SignOrZero(normal.x)*abs(vector_vel.y), velocity.z));

		//rotate collider

		/*float3 direction = float3(velocity.x, -SignOrZero(velocity.x)* SignOrZero(normal.x)*abs(vector_vel.y), velocity.z);*/
		float3 direction = float3(velocity.x, 0, velocity.z);
		Quat new_rotation = owner->transform.GetRotation().LookAt(float3::unitZ, direction.Normalized(), float3::unitY, float3::unitY);

		btTransform trans = body->getWorldTransform();
		btQuaternion transrot = trans.getRotation();

		transrot = btQuaternion(new_rotation.x, new_rotation.y, new_rotation.z, new_rotation.w);
		//trans.setRotation(transrot);
		body->setWorldTransform(trans);
	}
}

void ComponentCollider::LookAt(float3& velocity, float speed)
{
	//bottom of the model
	btVector3 bottom = body->getWorldTransform().getOrigin();
	bottom.setY(bottom.getY() - 200 * box_size.getY());

	//Vector normal to the surface
	btVector3 Normal;
	App->physics->RaycastWorld(body->getWorldTransform().getOrigin(), bottom, Normal);
	float3 normal = float3(Normal);
	normal.Normalize();

	float2 normal_2D = float2(normal.x, normal.y);
	float2 vector_vel = normal_2D.Perp();

	if (abs(velocity.x) > 0 || abs(velocity.z) > 0)
	{
		velocity.Normalize();
		//body->setLinearVelocity(speed*btVector3(velocity.x, -SignOrZero(velocity.x)* SignOrZero(normal.x)*abs(vector_vel.y), velocity.z));

		//rotate collider

		/*float3 direction = float3(velocity.x, -SignOrZero(velocity.x)* SignOrZero(normal.x)*abs(vector_vel.y), velocity.z);*/
		float3 direction = float3(velocity.x, 0, velocity.z);
		Quat new_rotation = owner->transform.GetRotation().LookAt(float3::unitZ, direction.Normalized(), float3::unitY, float3::unitY);

		btTransform trans = body->getWorldTransform();
		btQuaternion transrot = trans.getRotation();

		transrot = btQuaternion(new_rotation.x, new_rotation.y, new_rotation.z, new_rotation.w);
		trans.setRotation(transrot);
		body->setWorldTransform(trans);
	}
}

float3 ComponentCollider::GetCurrentVelocity() const
{
	btVector3 velocity = body->getLinearVelocity();

	return float3(velocity.getX(), velocity.getY(), velocity.getZ());
}
