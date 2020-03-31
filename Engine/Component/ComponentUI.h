#ifndef _COMPONENTUI_H_
#define _COMPONENTUI_H_

#include "Component.h"

class ComponentUI : public Component
{
public:
	enum class UIType {
		CANVAS,
		IMAGE,
		TEXT,
		BUTTON,
		PROGRESSBAR
	};
	ComponentUI(UIType);
	ComponentUI(GameObject * owner, UIType);
	~ComponentUI();

	virtual void Delete();

	virtual void Save(Config& config) const;
	virtual void Load(const Config& config);
	virtual Component* Clone(bool original_prefab = false) const { return nullptr; };
	virtual void Copy(Component* component_to_copy) const {};
	virtual void Render(float4x4* projection);
	virtual void Render(float4x4* projection, float4x4* model, unsigned int texture = 0, float3* color = &float3(0.0f, 1.0f, 0.0f));

public:
	float3 color = float3::unitY;
	UIType ui_type;
	
protected:
	unsigned int shader_program, vao, vbo;
	unsigned int ui_texture = 2;
	void InitData();
	friend class PanelComponent;
};
#endif
