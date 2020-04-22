#include "SceneManager.h"

#include "Helper/Config.h"

#include "Main/Application.h"
#include "Main/GameObject.h"

#include "Module/ModuleAnimation.h"
#include "Module/ModuleFileSystem.h"
#include "Module/ModuleRender.h"
#include "Module/ModuleResourceManager.h"
#include "Module/ModuleScene.h"
#include "Module/ModuleScriptManager.h"

#include "ResourceManagement/Resources/Prefab.h"

#include <stack>
#include <unordered_map>
#include "Brofiler/Brofiler.h"

//THIS CLASS INCLUDE IMPORT AND LOAD FOR SCENE UNTIL SCENE IS CHANGE TO BE A RESOURCE
void SceneManager::Save(const std::string &path, GameObject* gameobject_to_save) const
{
	Config scene_config;

	std::vector<Config> game_objects_config;
	std::vector<Config> prefabs_config;
	std::vector<Config> prefabs_components_config;
	std::stack<GameObject*> pending_objects;

	for (auto& child_game_object : gameobject_to_save->children)
	{
		pending_objects.push(child_game_object);
	}

	while (!pending_objects.empty())
	{
		GameObject* current_game_object = pending_objects.top();
		pending_objects.pop();


		if (current_game_object->is_prefab_parent)
		{
			Config current_prefab;
			SavePrefab(current_prefab, current_game_object);
			prefabs_config.push_back(current_prefab);
		}
		else if(!current_game_object->prefab_reference)
		{
			Config current_gameobject;
			current_game_object->Save(current_gameobject);
			game_objects_config.push_back(current_gameobject);
		}
		
		if(current_game_object->prefab_reference)
		{
			Config current_prefab_modified_component;
			bool modified = SaveModifiedPrefabComponents(current_prefab_modified_component, current_game_object);
			if (modified) { prefabs_components_config.push_back(current_prefab_modified_component); };
		}

		for (auto& child_game_object : current_game_object->children)
		{
			pending_objects.push(child_game_object);
		}
	}
	scene_config.AddChildrenConfig(prefabs_config, "Prefabs");
	scene_config.AddChildrenConfig(prefabs_components_config, "PrefabsComponents");
	scene_config.AddChildrenConfig(game_objects_config, "GameObjects");

	std::string serialized_scene_string;
	scene_config.GetSerializedString(serialized_scene_string);

	App->filesystem->Save(path.c_str(), serialized_scene_string);
}



void SceneManager::Load(const std::string& path) const
{
	BROFILER_CATEGORY("Scene: Load", Profiler::Color::NavajoWhite);
	Path* scene_path = App->filesystem->GetPath(path);
	FileData scene_data = scene_path->GetFile()->Load();

	size_t readed_bytes = scene_data.size;
	char* scene_file_data = (char*)scene_data.buffer;
	std::string serialized_scene_string = scene_file_data;
	free(scene_file_data);

	Config scene_config(serialized_scene_string);

	std::unordered_map<int64_t, std::vector<GameObject*>> prefab_parents;
	std::vector<Config> prefabs_config;
	scene_config.GetChildrenConfig("Prefabs", prefabs_config);
	for (unsigned int i = 0; i < prefabs_config.size(); ++i)
	{
		uint64_t parent_UUID = prefabs_config[i].GetUInt("ParentUUID", 0);
		GameObject * loaded_gameobject = LoadPrefab(prefabs_config[i]);
		if (parent_UUID != 0)
		{
			prefab_parents[parent_UUID].push_back(loaded_gameobject);
		}
	}

	std::vector<Config> prefabs_modified_components;
	scene_config.GetChildrenConfig("PrefabsComponents", prefabs_modified_components);
	for (unsigned int i = 0; i < prefabs_modified_components.size(); ++i)
	{
		LoadPrefabModifiedComponents(prefabs_modified_components[i]);
	}


	std::vector<Config> game_objects_config;
	scene_config.GetChildrenConfig("GameObjects", game_objects_config);
	for (unsigned int i = 0; i < game_objects_config.size(); ++i)
	{
		GameObject* created_game_object = App->scene->CreateGameObject();
		created_game_object->Load(game_objects_config[i]);

		if (!created_game_object->IsStatic())
		{
			App->renderer->InsertAABBTree(created_game_object);
		}
		if (prefab_parents.find(created_game_object->UUID) != prefab_parents.end())
		{
			for (const auto&  prefab_child : prefab_parents[created_game_object->UUID])
			{
				ComponentTransform previous_transform = prefab_child->transform;
				prefab_child->SetParent(created_game_object);
				prefab_child->transform = previous_transform;
				prefab_child->aabb.GenerateBoundingBox();
			}

		}
	}
	App->scripts->ReLink();
	App->animations->UpdateAnimationMeshes();
}


void SceneManager::SavePrefab(Config& config, GameObject* gameobject_to_save) const
{
	if (gameobject_to_save->parent != nullptr)
	{
		config.AddUInt(gameobject_to_save->parent->UUID, "ParentUUID");
	}
	config.AddUInt(gameobject_to_save->prefab_reference->GetUUID(), "Prefab");
	config.AddString(gameobject_to_save->name, "Name");

	Config transform_config;
	gameobject_to_save->transform.Save(transform_config);
	config.AddChildConfig(transform_config, "Transform");

	std::vector<Config> original_UUIDS;
	SavePrefabUUIDS(original_UUIDS, gameobject_to_save);
	config.AddChildrenConfig(original_UUIDS, "UUIDS");
}

void SceneManager::SavePrefabUUIDS(std::vector<Config>& original_UUIDS, GameObject* gameobject_to_save) const
{
	Config config;
	config.AddUInt(gameobject_to_save->UUID, "UUID");
	config.AddUInt(gameobject_to_save->original_UUID, "OriginalUUID");

	original_UUIDS.push_back(config);
	for (const auto&  child : gameobject_to_save->children)
	{
		SavePrefabUUIDS(original_UUIDS, child);
	}

}
GameObject* SceneManager::LoadPrefab(const Config & config) const
{
	BROFILER_CATEGORY("Scene: Load Prefab", Profiler::Color::Snow);
	uint32_t prefab_uuid;
	prefab_uuid = config.GetUInt("Prefab", 0);

	std::shared_ptr<Prefab> prefab = App->resources->Load<Prefab>(prefab_uuid);
	std::unordered_map<int64_t, int64_t> UUIDS_pairs;
	std::vector<Config> original_UUIDS;
	config.GetChildrenConfig("UUIDS", original_UUIDS);
	for (const auto&  child_UUIDS : original_UUIDS)
	{
		int64_t UUID = child_UUIDS.GetUInt("UUID", 0);
		int64_t original = child_UUIDS.GetUInt("OriginalUUID", 0);
		UUIDS_pairs[original] = UUID;
	}

	if (prefab == nullptr)
	{
		GameObject * missing_prefab = App->scene->CreateGameObject();
		missing_prefab->name = "Missing prefab";
		return missing_prefab;
	}

	GameObject * instance = prefab->Instantiate(App->scene->GetRoot(), &UUIDS_pairs);
	Config transform_config;
	config.GetChildConfig("Transform", transform_config);
	instance->transform.Load(transform_config);
	config.GetString("Name", instance->name, instance->name);

	return instance;
}

bool SceneManager::SaveModifiedPrefabComponents(Config& config, GameObject* gameobject_to_save) const
{
	bool modified = false;
	if (gameobject_to_save->transform.modified_by_user)
	{
		Config transform_config;
		gameobject_to_save->transform.Save(transform_config);
		config.AddChildConfig(transform_config, "Transform");
		modified = true;
	}
	if (gameobject_to_save->modified_by_user)
	{
		config.AddString(gameobject_to_save->name, "Name");
		modified = true;
	}
	std::vector<Config> gameobject_components_config;
	for (const auto&  component : gameobject_to_save->components)
	{
		if (component->modified_by_user || component->added_by_user)
		{
			Config component_config;
			component->Save(component_config);
			gameobject_components_config.push_back(component_config);
			modified = true;
		}
	}
	if (modified)
	{
		config.AddChildrenConfig(gameobject_components_config, "Components");
		config.AddUInt(gameobject_to_save->UUID, "UUID");
	}
	return modified;
}

void SceneManager::LoadPrefabModifiedComponents(const Config& config) const
{
	BROFILER_CATEGORY("Scene: Load Modiefied", Profiler::Color::Aquamarine);
	GameObject * prefab_child = App->scene->GetGameObject(config.GetUInt("UUID", 0));
	if (prefab_child == nullptr)
	{
		return;
	}
	if (config.config_document.HasMember("Transform"))
	{
		Config transform_config;
		config.GetChildConfig("Transform", transform_config);

		prefab_child->transform.Load(transform_config);
		prefab_child->transform.modified_by_user = true;
	}
	if (config.config_document.HasMember("Name"))
	{
		config.GetString("Name", prefab_child->name, prefab_child->name);
		prefab_child->modified_by_user = true;
	}
	std::vector<Config> prefab_components_config;
	config.GetChildrenConfig("Components", prefab_components_config);
	for (const auto& component_config : prefab_components_config)
	{
		uint64_t component_type_uint = component_config.GetUInt("ComponentType", 0);
		assert(component_type_uint != 0);

		uint64_t UUID = component_config.GetUInt("UUID", 0);

		Component * component = prefab_child->GetComponent(static_cast<Component::ComponentType>(component_type_uint));
		if (component != nullptr && component->UUID == UUID)
		{
			component->Load(component_config);
			component->modified_by_user = true;
		}
		else 
		{
			Component* created_component = prefab_child->CreateComponent(static_cast<Component::ComponentType>(component_type_uint));
			created_component->Load(component_config);
			created_component->added_by_user = true;
		}
	}
}