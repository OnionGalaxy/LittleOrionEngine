#include "Main/Application.h"
#include "ModuleAI.h"
#include "ModuleInput.h"


bool ModuleAI::Init()
{
	return true;
}

update_status ModuleAI::Update()
{
	nav_mesh.Update();

	if (App->input->GetKeyDown(KeyCode::P))
	{
		nav_mesh.CreateNavMesh();
	}

	return update_status::UPDATE_CONTINUE;
}

bool ModuleAI::CleanUp()
{
	return true;
}

void ModuleAI::RenderNavMesh(ComponentCamera& camera)
{
	nav_mesh.RenderNavMesh(camera);
}

bool ModuleAI::FindPath(float3 & start, float3 & end, std::vector<float3>& path) const
{
	return nav_mesh.FindPath(start, end ,path);
}
