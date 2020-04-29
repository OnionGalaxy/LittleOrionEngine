#ifndef _COMPONENTCYLINDERCOLLIDER_H
#define _COMPONENTCYLINDERCOLLIDER_H

#include "ComponentCollider.h"


class ComponentCylinderCollider : public ComponentCollider
{
public:

	ComponentCylinderCollider();
	ComponentCylinderCollider(GameObject* owner);
	~ComponentCylinderCollider() = default;

	void UpdateDimensions();
	void Scale();
};

#endif
