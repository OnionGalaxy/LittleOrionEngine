#include "PlayerAttack.h"

#include "Component/ComponentAnimation.h"
#include "Component/ComponentCollider.h"
#include "Component/ComponentScript.h"
#include "Component/ComponentTransform.h"
#include "Component/ComponentAudioSource.h"
#include "Main/Application.h"
#include "Main/GameObject.h"
#include "Module/ModuleInput.h"
#include "Module/ModulePhysics.h"
#include "Module/ModuleScene.h"
#include "EditorUI/Panel/InspectorSubpanel/PanelComponent.h"

#include "imgui.h"

#include "EnemyController.h"


PlayerAttack* PlayerAttackDLL()
{
	PlayerAttack* instance = new PlayerAttack();

	return instance;
}



PlayerAttack::PlayerAttack()
{
	panel = new PanelComponent();
}
// Use this for initialization before Start()

void PlayerAttack::Awake()
{
	GameObject* enemy_manager_go = App->scene->GetGameObjectByName("EnemyManager");
	ComponentScript* enemy_manager_component = enemy_manager_go->GetComponentScript("EnemyManager");
	enemy_manager = static_cast<EnemyManager*>(enemy_manager_component->script);

	animation = (ComponentAnimation*) owner->GetComponent(Component::ComponentType::ANIMATION);
	collider_component = static_cast<ComponentCollider*>(collider->GetComponent(ComponentCollider::ColliderType::BOX));
	audio_source = static_cast<ComponentAudioSource*>(owner->GetComponent(Component::ComponentType::AUDIO_SOURCE));
}
// Use this for initialization

void PlayerAttack::Start()
{

}
// Update is called once per frame

bool PlayerAttack::Attack(int player)
{
	is_attacking = animation->IsOnState("Punch") || animation->IsOnState("Kick");

	if(!is_attacking && !raycast_cast)
	{
		if (App->input->GetGameInputDown("Punch", static_cast<PlayerID>(player - 1)))
		{
			audio_source->PlayEvent("play_punch1_player");
			animation->ActiveAnimation("punch");
			//Active colliders of hands
			raycast_cast = true;
			current_damage_power = PUNCH_DAMAGE;
		}
		else if (App->input->GetGameInputDown("Kick", static_cast<PlayerID>(player - 1)))
		{
			audio_source->PlayEvent("play_kick1_player");
			animation->ActiveAnimation("kick");
			//Active colliders of kick
			raycast_cast = true;
			current_damage_power = KICK_DAMAGE;
		}
	}

	if (raycast_cast && animation->GetCurrentClipPercentatge() > 0.5f)
	{
		ComputeCollisions();
		raycast_cast = false;
	}

	return is_attacking;
}
void PlayerAttack::ComputeCollisions() const
{
	for (auto& enemy : enemy_manager->enemies)
	{
		if (!enemy->is_alive)
		{
			continue;
		}

		if (collider_component->DetectCollisionWith(enemy->collider))
		{
			if (current_damage_power == PUNCH_DAMAGE)
				audio_source->PlayEvent("play_punch1_hit_player");
			else
				audio_source->PlayEvent("play_kick1_hit_player");
			
			enemy->TakeDamage(current_damage_power);
		}
	}
}
// Use this for showing variables on inspector

void PlayerAttack::OnInspector(ImGuiContext* context)
{
	//Necessary to be able to write with imgui
	ImGui::SetCurrentContext(context);
	ShowDraggedObjects();
}

//Use this for linking JUST GO automatically 
void PlayerAttack::InitPublicGameObjects()
{
	public_gameobjects.push_back(&collider);
	variable_names.push_back(GET_VARIABLE_NAME(collider));

	//IMPORTANT, public gameobjects, name_gameobjects and go_uuids MUST have same size
	for (int i = 0; i < public_gameobjects.size(); ++i)
	{
		name_gameobjects.push_back(is_object);
		go_uuids.push_back(0);
	}
}

