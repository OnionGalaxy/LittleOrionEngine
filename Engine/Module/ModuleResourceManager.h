#ifndef _MODULERESOURCEMANAGER_H_
#define _MODULERESOURCEMANAGER_H_

#include "Main/Application.h"
#include "Module.h"
#include "ModuleFileSystem.h"

#include "ResourceManagement/Resources/Animation.h"
#include "ResourceManagement/Resources/Material.h"
#include "ResourceManagement/Resources/Mesh.h"
#include "ResourceManagement/Resources/Prefab.h"
#include "ResourceManagement/Resources/Skeleton.h"
#include "ResourceManagement/Resources/Skybox.h"
#include "ResourceManagement/Resources/StateMachine.h"
#include "ResourceManagement/Resources/Texture.h"

#include "ResourceManagement/ResourcesDB/ResourceDataBase.h"

#include <Brofiler/Brofiler.h>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

class Path;
class Timer;

class AnimationImporter;
class MaterialImporter;
class MeshImporter;
class ModelImporter;
class PrefabImporter;
class SceneManager;
class SkeletonImporter;
class SkyboxImporter;
class StateMachineImporter;
class TextureImporter;

class AnimationManager;
class MaterialManager;
class MeshManager;
class PrefabManager;
class SceneManager;
class SkeletonManager;
class SkyboxManager;
class StateMachineManager;
class TextureManager;

class MetafileManager;

class ModuleResourceManager : public Module
{
public:

	ModuleResourceManager();
	~ModuleResourceManager() = default;

	bool Init() override;

	update_status PreUpdate() override;
	bool CleanUp() override;

	uint32_t Import(Path& file);

	template<typename T>
	uint32_t Create(Path& path, const std::string& resource_name)
	{
		BROFILER_CATEGORY("Create Resource", Profiler::Color::Brown);
		APP_LOG_INFO("Creating Resource %s.", resource_name.c_str())

		FileData created_resource_data = ResourceManagement::Create<T>();

		APP_LOG_SUCCESS("Resource %s created correctly.", resource_name.c_str())

		return CreateFromData(created_resource_data, path, resource_name);
	}

	template<typename T>
	void Save(std::shared_ptr<Resource> modified_resource)
	{
		APP_LOG_INFO("Saving Resource %u.", modified_resource->GetUUID());

		FileData resource_data = ResourceManagement::Binarize<T>(*modified_resource.get());
		std::string modified_resource_path = modified_resource->resource_metafile->imported_file_path;
		Path* saved_resource_assets_path = App->filesystem->Save(modified_resource_path, resource_data);

		InternalImport(saved_resource_assets_path);

		APP_LOG_SUCCESS("Resource %u saved corrrectly.", modified_resource->GetUUID());
	}

	template<typename T>
	std::shared_ptr<T> Load(uint32_t uuid)
	{
		BROFILER_CATEGORY("Load Resource", Profiler::Color::Brown);
		APP_LOG_INFO("Loading Resource %u.", uuid);

		std::shared_ptr<Resource> loaded_resource;
		loaded_resource = RetrieveFromCacheIfExist(uuid);
		if (loaded_resource != nullptr)
		{
			APP_LOG_SUCCESS("Resource %u loaded correctly from cache.", uuid);
			return std::static_pointer_cast<T>(loaded_resource);
		}

		Metafile* metafile = resource_DB->GetEntry(uuid);
		assert(metafile != nullptr);
		if (!App->filesystem->Exists(metafile->exported_file_path))
		{
			APP_LOG_ERROR("Error loading Resource %u. File %s doesn't exist", uuid, metafile->exported_file_path);
			return nullptr;
		}

		Path* resource_exported_file_path = App->filesystem->GetPath(metafile->exported_file_path);
		FileData exported_file_data = resource_exported_file_path->GetFile()->Load();
		loaded_resource = ResourceManagement::Load<T>(metafile, exported_file_data);

		free((char*)exported_file_data.buffer);

		if (loaded_resource != nullptr)
		{
			resource_cache.push_back(loaded_resource);
		}

		APP_LOG_SUCCESS("Resource %u loaded correctly.", uuid);
		return std::static_pointer_cast<T>(loaded_resource);
	}

	void CleanInconsistenciesInDirectory(const Path& directory_path);
	void ImportAssetsInDirectory(const Path& directory_path);

	uint32_t CreateFromData(FileData data, Path& creation_folder_path, const std::string& created_resource_name);

	void CleanResourceCache();

private:
	void StartThread();
	uint32_t InternalImport(Path& file_path) const;

	std::shared_ptr<Resource> RetrieveFromCacheIfExist(uint32_t uuid) const;

public:
	struct ThreadComunication
	{
		mutable std::mutex thread_mutex;
		std::atomic_bool stop_thread = false;
		std::atomic_bool finished_loading = false;
		std::atomic_uint thread_importing_hash = 0;
		std::atomic_uint main_importing_hash = 0;
		std::atomic_uint loaded_items = 0;
		std::atomic_uint total_items = 0;
	} thread_comunication;

	//Importers
	std::unique_ptr<AnimationImporter> animation_importer = nullptr;
	std::unique_ptr<MaterialImporter> material_importer = nullptr;
	std::unique_ptr<MeshImporter> mesh_importer = nullptr;
	std::unique_ptr<ModelImporter> model_importer = nullptr;
	std::unique_ptr<PrefabImporter> prefab_importer = nullptr;
	std::unique_ptr<SkeletonImporter> skeleton_importer = nullptr;
	std::unique_ptr<SkyboxImporter> skybox_importer = nullptr;
	std::unique_ptr<StateMachineImporter> state_machine_importer = nullptr;
	std::unique_ptr<TextureImporter> texture_importer = nullptr;

	std::unique_ptr<MetafileManager> metafile_manager = nullptr;
	std::unique_ptr<SceneManager> scene_manager = nullptr;
	std::unique_ptr<ResourceDataBase> resource_DB = nullptr;

private:
	const size_t importer_interval_millis = 30000;
	float last_imported_time = 0;
	std::thread importing_thread;
	std::unique_ptr<Timer> thread_timer = std::make_unique<Timer>();

	mutable std::vector<std::shared_ptr<Resource>> resource_cache;

	friend class MaterialImporter;
};

#endif // _MODULERESOURCEMANAGER_H_