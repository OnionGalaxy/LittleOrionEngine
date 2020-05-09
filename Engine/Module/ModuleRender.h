#ifndef _MODULERENDER_H_
#define _MODULERENDER_H_

#define ENGINE_EXPORTS

#include "Module.h"
#include "Helper/Timer.h"
#include "Main/Globals.h"

#include <MathGeoLib/Geometry/LineSegment.h>
#include <GL/glew.h>
#include <list>

class ComponentCamera;
class ComponentMeshRenderer;
class GameObject;

struct SDL_Texture;
struct SDL_Renderer;
struct SDL_Rect;

class ModuleRender : public Module
{
public:
	enum class DrawMode
	{
		SHADED,
		WIREFRAME
	};

	ModuleRender() = default;
	~ModuleRender() = default;

	bool Init();
	update_status PreUpdate();
	bool CleanUp();
	
	void Render() const;
	void RenderFrame(const ComponentCamera& camera);


	ComponentMeshRenderer* CreateComponentMeshRenderer();
	void RemoveComponentMesh(ComponentMeshRenderer* mesh_to_remove);

	ENGINE_API int GetRenderedTris() const;
	ENGINE_API int GetRenderedVerts() const;

	GameObject* GetRaycastIntertectedObject(const LineSegment& ray);
	bool GetRaycastIntertectedObject(const LineSegment& ray, float3& position);

	ENGINE_API void SetDrawMode(DrawMode draw_mode);

private:
	void SetVSync(bool vsync);
	void SetAlphaTest(bool gl_alpha_test);
	void SetDepthTest(bool gl_depth_test);
	void SetScissorTest(bool gl_scissor_test);
	void SetStencilTest(bool gl_stencil_test);
	void SetBlending(bool gl_blend);
	void SetFaceCulling(bool gl_cull_face);
	void SetCulledFaces(GLenum culled_faces) const;
	void SetFrontFaces(GLenum front_faces) const;
	void SetDithering(bool gl_dither);
	void SetMinMaxing(bool gl_minmax);

	std::string GetDrawMode() const;

	void GetMeshesToRender(const ComponentCamera* camera);
	void SetListOfMeshesToRender(const ComponentCamera* camera);

public:
	bool anti_aliasing = false;

private:
	void* context = nullptr;


	bool vsync = false;
	bool gl_alpha_test = false;
	bool gl_depth_test = false;
	bool gl_scissor_test = false;
	bool gl_stencil_test = false;
	bool gl_blend = true;
	bool gl_cull_face = false;
	int culled_faces = 0;
	int front_faces = 0;
	int filling_mode = 0;
	bool gl_dither = false;
	bool gl_minmax = false;

	DrawMode draw_mode = DrawMode::SHADED;

	std::vector<ComponentMeshRenderer*> meshes;
	std::vector<ComponentMeshRenderer*> meshes_to_render;
	typedef std::pair<float, ComponentMeshRenderer*> ipair;
	std::list <ipair> opaque_mesh_to_render, transparent_mesh_to_render;
	int num_rendered_tris = 0;
	int num_rendered_verts = 0;
	Timer * rendering_measure_timer = new Timer();

	friend class ModuleDebugDraw;
	friend class ModuleSpacePartitioning;
	friend class PanelConfiguration;
	friend class PanelScene;
	friend class NavMesh;
};

#endif //_MODULERENDER_H_