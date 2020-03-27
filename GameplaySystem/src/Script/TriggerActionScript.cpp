#include "TriggerActionScript.h"

#include "Component/ComponentAABB.h"
#include "Component/ComponentScript.h"
#include "Component/ComponentTransform.h"

#include "Main/Application.h"
#include "Main/GameObject.h"
#include "Module/ModuleInput.h"
#include "Module/ModuleScene.h"

#include "UI/Panel/InspectorSubpanel/PanelComponent.h"

#include "imgui.h"
#include <MathGeoLib/MathGeoLib.h>



TriggerActionScript* TriggerActionScriptDLL()
{
	TriggerActionScript *instance = new TriggerActionScript();
	return instance;
}

TriggerActionScript::TriggerActionScript()
{
	panel = new PanelComponent();
}

// Use this for initialization before Start()
void TriggerActionScript::Awake()
{
	movement_component = trigger_go->GetComponentScript("PlayerController");
	movement_script = (PlayerController*)movement_component->script;
	start_position = trigger_go->transform.GetGlobalTranslation();
}

// Use this for initialization
void TriggerActionScript::Start()
{

}

// Update is called once per frame
void TriggerActionScript::Update()
{
	if(OnTriggerEnter())
	{
		if(movement_script)
		{
			//Do something if inside aabb of an object
			//movement_script->speed *= 5.f;
			trigger_go->transform.SetTranslation(start_position);
		}
	}

}

// Use this for showing variables on inspector
void TriggerActionScript::OnInspector(ImGuiContext* context)
{
	//Necessary to be able to write with imgui
	ImGui::SetCurrentContext(context);
	//Example to show text
	ImGui::Text("TriggerActionScript Inspector");
	//Example to Drag and drop and link GOs in the Editor, Unity-like (WIP)
	ImGui::Text("Variables: ");
	ShowDraggedObjects();
}

bool TriggerActionScript::OnTriggerEnter() const
{
	return trigger_go && owner->aabb.global_bounding_box.Intersects(trigger_go->aabb.global_bounding_box);
}

void TriggerActionScript::InitPublicGameObjects()
{
	//IMPORTANT, public gameobjects, name_gameobjects and go_uuids MUST have same size

	public_gameobjects.push_back(&trigger_go);
	public_gameobjects.push_back(&player);

	variable_names.push_back(GET_VARIABLE_NAME(trigger_go));
	variable_names.push_back(GET_VARIABLE_NAME(player));

	for(int i = 0; i < public_gameobjects.size(); ++i)
	{
		name_gameobjects.push_back(is_object);
		go_uuids.push_back(0);
	}
}
