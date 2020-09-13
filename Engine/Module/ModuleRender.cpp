#include "ModuleRender.h"

#include "Main/Globals.h"
#include "Main/Application.h"
#include "ModuleCamera.h"
#include "ModuleDebug.h"
#include "ModuleDebugDraw.h"
#include "ModuleEditor.h"
#include "ModuleEffects.h"
#include "ModuleProgram.h"
#include "ModuleSpacePartitioning.h"
#include "ModuleUI.h"
#include "ModuleWindow.h"
#include "ModuleLight.h"

#include "Component/ComponentCamera.h"
#include "Component/ComponentMeshRenderer.h"
#include "Component/ComponentLight.h"

#include "EditorUI/DebugDraw.h"
#include "ModuleResourceManager.h"
#include "Rendering/Viewport.h"

#include <algorithm>
#include <assimp/scene.h>
#include <MathGeoLib.h>
#include <SDL/SDL.h>
#include <Brofiler/Brofiler.h>

static void APIENTRY openglCallbackFunction(
	GLenum source,
	GLenum type,
	GLuint id,
	GLenum severity,
	GLsizei length,
	const GLchar* message,
	const void* userParam
) {
	(void)source; (void)type; (void)id;
	(void)severity; (void)length; (void)userParam;

	char error_source[256];
	switch (source)
	{
		case GL_DEBUG_SOURCE_API:             sprintf_s(error_source, "Source: API"); break;
		case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   sprintf_s(error_source, "Source: Window System"); break;
		case GL_DEBUG_SOURCE_SHADER_COMPILER: sprintf_s(error_source, "Source: Shader Compiler"); break;
		case GL_DEBUG_SOURCE_THIRD_PARTY:     sprintf_s(error_source, "Source: Third Party"); break;
		case GL_DEBUG_SOURCE_APPLICATION:     sprintf_s(error_source, "Source: Application"); break;
		case GL_DEBUG_SOURCE_OTHER:           sprintf_s(error_source, "Source: Other"); break;
	}

	char error_type[256];
	switch (type)
	{
		case GL_DEBUG_TYPE_ERROR:               sprintf_s(error_type, "Type: Error"); break;
		case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: sprintf_s(error_type, "Type: Deprecated Behaviour"); break;
		case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  sprintf_s(error_type, "Type: Undefined Behaviour"); break;
		case GL_DEBUG_TYPE_PORTABILITY:         sprintf_s(error_type, "Type: Portability"); break;
		case GL_DEBUG_TYPE_PERFORMANCE:         sprintf_s(error_type, "Type: Performance"); break;
		case GL_DEBUG_TYPE_MARKER:              sprintf_s(error_type, "Type: Marker"); break;
		case GL_DEBUG_TYPE_PUSH_GROUP:          sprintf_s(error_type, "Type: Push Group"); break;
		case GL_DEBUG_TYPE_POP_GROUP:           sprintf_s(error_type, "Type: Pop Group"); break;
		case GL_DEBUG_TYPE_OTHER:               sprintf_s(error_type, "Type: Other"); break;
	}

	char error_message[4096];
	sprintf_s(error_message, "%s %s %s", error_source, error_type, message);
	switch (severity)
	{
	case GL_DEBUG_SEVERITY_HIGH:
		OPENGL_LOG_ERROR(error_message);
		break;
	case GL_DEBUG_SEVERITY_MEDIUM:
		//OPENGL_LOG_INIT(error_message); // Actually not an itialization entry, I use this type of entry because the yellow color
		break;
	case GL_DEBUG_SEVERITY_LOW:
		//OPENGL_LOG_INFO(error_message); Too many messages in update
	case GL_DEBUG_SEVERITY_NOTIFICATION:
		return;
	}
}

// Called before render is available
bool ModuleRender::Init()
{
	APP_LOG_SECTION("************ Module Render Init ************");
	// Init GLEW library
	GLenum err = glewInit();
	// … check for errors
	if (GLEW_OK != err)
	{
		APP_LOG_ERROR("Error initializing Glew");
		return false;

	}

	APP_LOG_INFO("Using Glew %s", glewGetString(GLEW_VERSION));
	APP_LOG_INFO("Vendor: %s", glGetString(GL_VENDOR));
	APP_LOG_INFO("Renderer: %s", glGetString(GL_RENDERER));
	APP_LOG_INFO("OpenGL version supported %s", glGetString(GL_VERSION));
	APP_LOG_INFO("GLSL: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));

#if OPENGL_MAJOR_VERSION >= 4

	glEnable(GL_DEBUG_OUTPUT);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	glDebugMessageCallback(openglCallbackFunction, nullptr);
	glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, true);

#endif // OPENGL_MAJOR_VERSION >= 4

	SetVSync(VSYNC);
	SetDepthTest(true);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBlendEquation(GL_FUNC_ADD);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	APP_LOG_INFO("Glew initialized correctly.");

	testing_viewport = new Viewport(true);

	return true;
}

update_status ModuleRender::PreUpdate()
{
	// CLEAR WINDOW COLOR AND DEPTH BUFFER
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	return update_status::UPDATE_CONTINUE;
}

// Called before quitting
bool ModuleRender::CleanUp()
{
	APP_LOG_INFO("Destroying renderer");
	delete rendering_measure_timer;
	for (auto& mesh : mesh_renderers)
	{
		mesh->owner->RemoveComponent(mesh);
	}

	delete testing_viewport;

	return true;
}

void ModuleRender::Render() const
{
	BROFILER_CATEGORY("Module Render Render",Profiler::Color::Aqua);

#if GAME
	if (App->cameras->main_camera != nullptr) 
	{

		App->lights->RecordShadowsFrameBuffers(App->window->GetWidth(), App->window->GetHeight());

		App->cameras->main_camera->RecordFrame(App->window->GetWidth(), App->window->GetHeight());

		App->cameras->main_camera->RecordDebugDraws(false);
	}
#endif

	App->editor->Render();

	BROFILER_CATEGORY("Swap Window (VSYNC)", Profiler::Color::Aquamarine);
	SDL_GL_SwapWindow(App->window->window);
}

void ModuleRender::RenderFrame(const ComponentCamera &camera)
{
	BROFILER_CATEGORY("Render Frame", Profiler::Color::Azure);
  
	rendering_measure_timer->Start();
	glBindBuffer(GL_UNIFORM_BUFFER, App->program->uniform_buffer.ubo);

	static size_t projection_matrix_offset = App->program->uniform_buffer.MATRICES_UNIFORMS_OFFSET + sizeof(float4x4);
	glBufferSubData(GL_UNIFORM_BUFFER, projection_matrix_offset, sizeof(float4x4), camera.GetProjectionMatrix().Transposed().ptr());

	static size_t view_matrix_offset = App->program->uniform_buffer.MATRICES_UNIFORMS_OFFSET + 2 * sizeof(float4x4);
	glBufferSubData(GL_UNIFORM_BUFFER, view_matrix_offset, sizeof(float4x4), camera.GetViewMatrix().Transposed().ptr());

	glBindBuffer(GL_UNIFORM_BUFFER, 0);

	num_rendered_tris = 0;
	num_rendered_verts = 0;

	
	GetMeshesToRender(&camera);
	for (auto &mesh : opaque_mesh_to_render)
	{
		BROFILER_CATEGORY("Render Mesh Opaque", Profiler::Color::Aquamarine);
		if (mesh.second->mesh_uuid != 0 && mesh.second->IsEnabled())
		{
			mesh.second->Render();
			if(mesh.second->mesh_to_render)
			{
				num_rendered_tris += mesh.second->mesh_to_render->GetNumTriangles();
				num_rendered_verts += mesh.second->mesh_to_render->GetNumVerts();
				App->lights->UpdateLightAABB(*mesh.second->owner);			
			}
			glUseProgram(0);

		}
	}

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBlendEquation(GL_FUNC_ADD);
	for (auto &mesh : transparent_mesh_to_render)
	{
		BROFILER_CATEGORY("Render Mesh Transparent", Profiler::Color::Aquamarine);
		if (mesh.second->mesh_uuid != 0 && mesh.second->IsEnabled())
		{
			mesh.second->Render();
			if(mesh.second->mesh_to_render)
			{
				num_rendered_tris += mesh.second->mesh_to_render->GetNumTriangles();
				num_rendered_verts += mesh.second->mesh_to_render->GetNumVerts();
				App->lights->UpdateLightAABB(*mesh.second->owner);			
			}

			glUseProgram(0);
			
		}
	}
	glDisable(GL_BLEND);
	
	App->effects->Render();
	
	rendering_measure_timer->Stop();
	App->debug->rendering_time = rendering_measure_timer->Read();
	
}

void ModuleRender::RenderZBufferFrame(const ComponentCamera & camera)
{
	BROFILER_CATEGORY("Render Z buffer Frame", Profiler::Color::Azure);

	glBindBuffer(GL_UNIFORM_BUFFER, App->program->uniform_buffer.ubo);

	static size_t projection_matrix_offset = App->program->uniform_buffer.MATRICES_UNIFORMS_OFFSET + sizeof(float4x4);
	glBufferSubData(GL_UNIFORM_BUFFER, projection_matrix_offset, sizeof(float4x4), camera.GetProjectionMatrix().Transposed().ptr());

	static size_t view_matrix_offset = App->program->uniform_buffer.MATRICES_UNIFORMS_OFFSET + 2 * sizeof(float4x4);
	glBufferSubData(GL_UNIFORM_BUFFER, view_matrix_offset, sizeof(float4x4), camera.GetViewMatrix().Transposed().ptr());

	glBindBuffer(GL_UNIFORM_BUFFER, 0);

	for (ComponentMeshRenderer* mesh : meshes_to_render)
	{
		if (mesh->shadow_caster)
		{
			mesh->Render();
		}
	}

}

void ModuleRender::GetMeshesToRender(const ComponentCamera* camera)
{
	BROFILER_CATEGORY("Get meshes to render", Profiler::Color::Aquamarine);

	meshes_to_render.clear();
	if (camera == App->cameras->scene_camera && !App->debug->culling_scene_mode)
	{
		meshes_to_render = mesh_renderers;
	}
	else
	{
		App->space_partitioning->GetCullingMeshes(App->cameras->main_camera);
	}
	SetListOfMeshesToRender(camera);
}

void ModuleRender::SetListOfMeshesToRender(const ComponentCamera* camera)
{
	opaque_mesh_to_render.clear();
	transparent_mesh_to_render.clear();
	float3 camera_pos = camera->camera_frustum.pos;
	for (ComponentMeshRenderer* mesh_to_render : meshes_to_render)
	{
		if (mesh_to_render->mesh_to_render == nullptr || mesh_to_render->material_to_render == nullptr)
		{
			continue;
		}

		if (
			mesh_to_render->material_to_render->material_type == Material::MaterialType::MATERIAL_TRANSPARENT 
			|| mesh_to_render->material_to_render->material_type == Material::MaterialType::MATERIAL_LIQUID
			|| mesh_to_render->material_to_render->material_type == Material::MaterialType::MATERIAL_DISSOLVING
		)
		{
			mesh_to_render->owner->aabb.bounding_box;
			float3 center_bounding_box = (mesh_to_render->owner->aabb.bounding_box.minPoint + mesh_to_render->owner->aabb.bounding_box.maxPoint) / 2;
			float distance = center_bounding_box.Distance(camera_pos);
			transparent_mesh_to_render.push_back(std::make_pair(distance, mesh_to_render));
			transparent_mesh_to_render.sort([](const ipair & a, const ipair & b) { return a.first > b.first; });
		}

		if (mesh_to_render->material_to_render->material_type == Material::MaterialType::MATERIAL_OPAQUE)
		{
			mesh_to_render->owner->aabb.bounding_box;
			float3 center_bounding_box = (mesh_to_render->owner->aabb.bounding_box.minPoint + mesh_to_render->owner->aabb.bounding_box.maxPoint) / 2;
			float distance = center_bounding_box.Distance(camera_pos);
			opaque_mesh_to_render.push_back(std::make_pair(distance, mesh_to_render));
			opaque_mesh_to_render.sort([](const ipair & a, const ipair & b) { return a.first < b.first; });
		}
	}
}

void ModuleRender::SetVSync(bool vsync)
{
	this->vsync = vsync;
	vsync ? SDL_GL_SetSwapInterval(1) : SDL_GL_SetSwapInterval(0);
}

void ModuleRender::SetAlphaTest(bool gl_alpha_test)
{
	this->gl_alpha_test = gl_alpha_test;
	gl_alpha_test ? glEnable(GL_ALPHA_TEST) : glDisable(GL_ALPHA_TEST);
}

void ModuleRender::SetDepthTest(bool gl_depth_test)
{
	this->gl_depth_test = gl_depth_test;
	gl_depth_test ? glEnable(GL_DEPTH_TEST) : glDisable(GL_DEPTH_TEST);
}

void ModuleRender::SetScissorTest(bool gl_scissor_test)
{
	this->gl_scissor_test = gl_scissor_test;
	gl_scissor_test ? glEnable(GL_SCISSOR_TEST) : glDisable(GL_SCISSOR_TEST);
}

void ModuleRender::SetStencilTest(bool gl_stencil_test)
{
	this->gl_stencil_test = gl_stencil_test;
	gl_stencil_test ? glEnable(GL_STENCIL_TEST) : glDisable(GL_STENCIL_TEST);
}

void ModuleRender::SetBlending(bool gl_blend)
{
	this->gl_blend = gl_blend;

}


void ModuleRender::SetFaceCulling(bool gl_cull_face)
{
	this->gl_cull_face = gl_cull_face;
	gl_cull_face ? glEnable(GL_CULL_FACE) : glDisable(GL_CULL_FACE);
}

void ModuleRender::SetCulledFaces(GLenum culled_faces) const
{
	glCullFace(culled_faces);
}

void ModuleRender::SetFrontFaces(GLenum front_faces) const
{
	glFrontFace(front_faces);
}

void ModuleRender::SetDithering(bool gl_dither)
{
	this->gl_dither = gl_dither;
	gl_dither ? glEnable(GL_DITHER) : glDisable(GL_DITHER);
}

void ModuleRender::SetMinMaxing(bool gl_minmax)
{
	this->gl_minmax = gl_minmax;
	gl_minmax ? glEnable(GL_MINMAX) : glDisable(GL_MINMAX);
}

void ModuleRender::SetDrawMode(DrawMode draw_mode)
{
	this->draw_mode = draw_mode;
	switch (draw_mode)
	{
	case ModuleRender::DrawMode::SHADED:
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			threshold_brightness = false;
		break;
	case ModuleRender::DrawMode::WIREFRAME:
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			threshold_brightness = false;
		break;
	case ModuleRender::DrawMode::BRIGHTNESS:
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			threshold_brightness = true;
		break;
	default:
		break;
	}
}

std::string ModuleRender::GetDrawMode() const
{
	switch (draw_mode)
	{
	case ModuleRender::DrawMode::SHADED:
		return "Shaded";
		break;
	case ModuleRender::DrawMode::WIREFRAME:
		return "Wireframe";
		break;
	case ModuleRender::DrawMode::BRIGHTNESS:
		return "Brightness";
		break;
	default:
		return "Unknown";
		break;
	}
}

ComponentMeshRenderer* ModuleRender::CreateComponentMeshRenderer()
{
	ComponentMeshRenderer* created_mesh = new ComponentMeshRenderer();
	mesh_renderers.push_back(created_mesh);
	return created_mesh;
}

void ModuleRender::RemoveComponentMesh(ComponentMeshRenderer* mesh_to_remove)
{
	const auto it = std::find(mesh_renderers.begin(), mesh_renderers.end(), mesh_to_remove);
	if (it != mesh_renderers.end())
	{
		delete *it;
		mesh_renderers.erase(it);
	}
}

RaycastHit* ModuleRender::GetRaycastIntersection(const LineSegment& ray, const ComponentCamera* cam)
{
	BROFILER_CATEGORY("Do Raycast", Profiler::Color::HotPink);
	App->space_partitioning->GetCullingMeshes(cam);
	std::vector<ComponentMeshRenderer*> intersected_meshes;
	for (const auto&  mesh : meshes_to_render)
	{
		if (mesh->owner->aabb.bounding_box.Intersects(ray))
		{
			//Allow non touchable meshes to be ignored from mouse picking in game mode
			if (cam != App->cameras->scene_camera && !mesh->is_raycastable) continue;

			intersected_meshes.push_back(mesh);
		}
	}

	BROFILER_CATEGORY("Intersect", Profiler::Color::HotPink);
	std::vector<GameObject*> intersected;
	GameObject* selected = nullptr;
	float min_distance = INFINITY;

	RaycastHit* result = new RaycastHit();

	for (const auto&  mesh : intersected_meshes)
	{
		LineSegment transformed_ray = ray;
		transformed_ray.Transform(mesh->owner->transform.GetGlobalModelMatrix().Inverted());
		BROFILER_CATEGORY("Triangles", Profiler::Color::HotPink);
		std::vector<Mesh::Vertex> &vertices = mesh->mesh_to_render->vertices;
		std::vector<uint32_t> &indices = mesh->mesh_to_render->indices;
		for (size_t i = 0; i < indices.size(); i += 3)
		{
			float3 first_point = vertices[indices[i]].position;
			float3 second_point = vertices[indices[i + 1]].position;
			float3 third_point = vertices[indices[i + 2]].position;
			Triangle triangle(first_point, second_point, third_point);

			float distance;
			float3 intersected_point;
			bool intersected = triangle.Intersects(transformed_ray, &distance, &intersected_point);
			if (intersected && distance < min_distance)
			{
				selected = mesh->owner;
				min_distance = distance;

				result->game_object = mesh->owner;
				result->hit_distance = distance;
				result->hit_point = intersected_point;
			}
		}
	}
	return result;
}

int ModuleRender::GetRenderedTris() const
{
	return num_rendered_tris;
}

int ModuleRender::GetRenderedVerts() const
{
	return num_rendered_verts;
}

void ModuleRender::RenderQuad()
{
	BROFILER_CATEGORY("Post Processing", Profiler::Color::MediumAquaMarine);
	
	unsigned int quadVAO = 0;
	unsigned int quadVBO;
	if (quadVAO == 0)
	{
		// setup plane VAO
		glGenVertexArrays(1, &quadVAO);
		glGenBuffers(1, &quadVBO);
		glBindVertexArray(quadVAO);
		glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

	}
	glBindVertexArray(quadVAO);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindVertexArray(0);
}

void ModuleRender::SetHDRType(const HDRType type)
{
	switch (type)
	{
		case HDRType::REINHARD:
			hdr_type = HDRType::REINHARD;
		break;	
		case HDRType::FILMIC:
			hdr_type = HDRType::FILMIC;
		break;
		case HDRType::EXPOSURE:
			hdr_type = HDRType::EXPOSURE;
		break;
	}
}
std::string ModuleRender::GetHDRType(const HDRType type) const
{
	switch (type)
	{
		case HDRType::REINHARD:
			return "Reinhard";
		case HDRType::FILMIC:
			return "Filmic";
		case HDRType::EXPOSURE:
			return "Exposure";
	}
}

void ModuleRender::RenderPostProcessingEffects(const ComponentCamera &camera)
{
	if (hdr_active)
	{
		horizontal = true;
		bool first_iteration = true;
	
		GLuint blur = App->program->UseProgram("Blur", 0);
		for (unsigned int i = 0; i < amount_of_blur; i++)
		{
			glBindFramebuffer(GL_FRAMEBUFFER, camera.pingpongFBO[horizontal]);
			glUniform1f(glGetUniformLocation(blur, "horizontal"), horizontal);
			glBindTexture(GL_TEXTURE_2D, first_iteration ? camera.color_buffers[1] : camera.pingpongColorbuffers[!horizontal]);
			RenderQuad();
			horizontal = !horizontal;
			if (first_iteration)
				first_iteration = false;
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	
		GLuint program = App->program->UseProgram("HDR", 0);
		glBindFramebuffer(GL_FRAMEBUFFER, camera.fbo);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, camera.color_buffers[0]);
		glUniform1i(glGetUniformLocation(program, "hdr_uniform.scene_texture"), 0);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, camera.pingpongColorbuffers[!horizontal]);
		glUniform1i(glGetUniformLocation(program, "hdr_uniform.bloom_texture"), 1);
		glUniform1f(glGetUniformLocation(program, "exposure"), exposure);
		glUniform1f(glGetUniformLocation(program, "bloom"), bloom);
		glUniform1f(glGetUniformLocation(program, "hdr_active"), hdr_active);
		glUniform1i(glGetUniformLocation(program, "hdr_type"), static_cast<int>(hdr_type));
		RenderQuad();
	}
}