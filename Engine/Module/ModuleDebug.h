#ifndef _MODULEDEBUG_H_
#define _MODULEDEBUG_H_

#define ENGINE_EXPORTS

#include "Module.h"

#include <GL/glew.h>

class ModuleDebug : public Module
{
public:
	enum class CullingMode
	{
		NONE,
		FRUSTUM_CULLING,
		QUADTREE_CULLING,
		OCTTREE_CULLING,
		AABBTREE_CULLING,
		COMBINED_CULLING
	};

	ModuleDebug() = default;
	~ModuleDebug() = default;

	bool Init() override;
	bool CleanUp() override;
	
	void CreateFrustumCullingDebugScene() const;
	ENGINE_API void ToggleRenderAABB();
public:
	bool show_debug_metrics = true;
	bool show_bounding_boxes = false;
	bool show_global_bounding_boxes = false;
	bool show_grid = true;
	bool show_camera_frustum = true;
	bool show_quadtree = false;
	bool show_octtree = false;
	bool show_aabbtree = false;
	bool show_navmesh = false;
	bool show_pathfind_points = true;
	bool show_axis = false;

#if !GAME
	bool culling_scene_mode = false;
#else
	bool culling_scene_mode = true;
#endif
	CullingMode culling_mode = CullingMode::FRUSTUM_CULLING;
	float rendering_time = 0;

private:
	int num_houses = 20;
	int max_dispersion_x = 40;
	int max_dispersion_z = 40;

	friend class PanelDebug;
	friend class PanelConfiguration;
};

#endif //_MODULEDEBUG_H_