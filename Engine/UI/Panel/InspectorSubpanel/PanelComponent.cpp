#include "PanelComponent.h"

#include "Actions/EditorActionTranslate.h"
#include "Actions/EditorActionRotation.h"
#include "Actions/EditorActionScale.h"
#include "Actions/EditorAction.h"

#include "Component/ComponentCamera.h"
#include "Component/ComponentMaterial.h"
#include "Component/ComponentMesh.h"
#include "Component/ComponentTransform.h"
#include "Component/ComponentLight.h"
#include "Component/ComponentScript.h"

#include "Main/Application.h"
#include "Main/GameObject.h"
#include "Module/ModuleEditor.h"
#include "Module/ModuleActions.h"
#include "Module/ModuleFileSystem.h"
#include "Module/ModuleProgram.h"
#include "Module/ModuleTexture.h"
#include "Module/ModuleScriptManager.h"


#include "Helper/Utils.h"

#include "UI/Panel/PanelPopups.h"

#include <imgui.h>
#include <FontAwesome5/IconsFontAwesome5.h>

void PanelComponent::ShowComponentTransformWindow(ComponentTransform *transform)
{
	if (ImGui::CollapsingHeader(ICON_FA_RULER_COMBINED " Transform", ImGuiTreeNodeFlags_DefaultOpen))
	{

		if (ImGui::DragFloat3("Translation", transform->translation.ptr(), 0.01f))
		{
			transform->OnTransformChange();
			transform->modified_by_user = true;
		}
		//UndoRedo
		CheckClickForUndo(ModuleActions::UndoActionType::TRANSLATION, transform);

		if (ImGui::DragFloat3("Rotation", transform->rotation_degrees.ptr(), 0.1f, -180.f, 180.f))
		{
			transform->rotation = Utils::GenerateQuatFromDegFloat3(transform->rotation_degrees);
			transform->rotation_radians = Utils::Float3DegToRad(transform->rotation_degrees);
			transform->OnTransformChange();
			transform->modified_by_user = true;
		}
		//UndoRedo
		CheckClickForUndo(ModuleActions::UndoActionType::ROTATION, transform);

		if (ImGui::DragFloat3("Scale", transform->scale.ptr(), 0.01f))
		{
			transform->OnTransformChange();
			transform->modified_by_user = true;
		}

		//UndoRedo
		CheckClickForUndo(ModuleActions::UndoActionType::SCALE, transform);
	}
}

void PanelComponent::ShowComponentMeshWindow(ComponentMesh *mesh)
{
	if (ImGui::CollapsingHeader(ICON_FA_SHAPES " Mesh", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if(ImGui::Checkbox("Active", &mesh->active))
		{
			//UndoRedo
			App->actions->action_component = mesh;
			App->actions->AddUndoAction(ModuleActions::UndoActionType::ENABLE_DISABLE_COMPONENT);
			mesh->modified_by_user = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete"))
		{
			App->actions->DeleteComponentUndo(mesh);
			return;
		}
		ImGui::Separator();


		char tmp_string[16];
		ImGui::AlignTextToFramePadding();
		ImGui::Text("Triangles");
		ImGui::SameLine();
		sprintf(tmp_string, "%d", mesh->mesh_to_render->vertices.size() / 3);
		ImGui::Button(tmp_string);

		ImGui::AlignTextToFramePadding();
		ImGui::Text("Vertices");
		ImGui::SameLine();
		sprintf(tmp_string, "%d", mesh->mesh_to_render->vertices.size());
		ImGui::Button(tmp_string);
	}
}

void PanelComponent::ShowComponentMaterialWindow(ComponentMaterial *material)
{
	if (ImGui::CollapsingHeader(ICON_FA_IMAGE " Material", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::BeginCombo("Shader", material->shader_program.c_str()))
		{
			for (auto & program : App->program->names)
			{
				bool is_selected = (material->shader_program == program);
				if (ImGui::Selectable(program, is_selected))
				{
					material->shader_program = program;
					if (is_selected)
						ImGui::SetItemDefaultFocus();
				}

			}
			ImGui::EndCombo();
		}

		float window_width = ImGui::GetWindowWidth();
		for (size_t i = 0; i < material->textures.size(); ++i)
		{
			Texture::TextureType type = static_cast<Texture::TextureType>(i);
			if (ImGui::CollapsingHeader(GetTypeName(type).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
			{
				if (material->textures[i].get() != nullptr) {
					ImGui::PushID(i);
					char tmp_string[256];
					std::shared_ptr<Texture> & texture = material->textures[i];
					ImGui::Image((void*)(intptr_t)texture->opengl_texture, ImVec2(window_width * 0.2f, window_width * 0.2f), ImVec2(0, 1), ImVec2(1, 0));
					DropTarget(material, type);
					ImGui::SameLine();
					ImGui::BeginGroup();
					ImGui::Text("Texture:");
					ImGui::SameLine();
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), texture->exported_file.c_str());
					sprintf_s(tmp_string, "(%dx%d px)", texture->width, texture->height);
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), tmp_string);

					bool mipmap = texture->IsMipMapped();
					ImGui::Checkbox("Mipmap", &mipmap);
					ImGui::SameLine();
					ImGui::Checkbox("Checker Texture", &material->show_checkerboard_texture);
					ImGui::Spacing();

					if (ImGui::Button(ICON_FA_TIMES) )
					{
						//UndoRedo
						App->actions->type_texture = Texture::TextureType(i);
						App->actions->action_component = material;
						App->actions->AddUndoAction(ModuleActions::UndoActionType::EDIT_COMPONENTMATERIAL);

						material->RemoveMaterialTexture(i);
						material->modified_by_user = true;
					}
					ImGui::SameLine(); ImGui::Text("Remove Texture");
					ImGui::EndGroup();
					ImGui::PopID();
				}
				else
				{
					ImGui::Image((void*)0, ImVec2(window_width * 0.2f, window_width * 0.2f), ImVec2(0, 1), ImVec2(1, 0), ImVec4(1.f,1.f,1.f,1.f), ImVec4(1.f, 1.f, 1.f, 1.f));
					DropTarget(material, type);
				}
				if (type == Texture::TextureType::DIFUSSE)
				{
					if (ImGui::ColorEdit3("Diffuse Color", material->diffuse_color)) { material->modified_by_user = true; };
					if (ImGui::SliderFloat("k diffuse", &material->k_diffuse, 0, 1)) { material->modified_by_user = true; };
				}
				if (type == Texture::TextureType::EMISSIVE)
				{
					if (ImGui::ColorEdit3("Emissive Color", material->emissive_color)) {material->modified_by_user = true;}
				}
				if (type == Texture::TextureType::OCLUSION)
				{
					if (ImGui::SliderFloat("k ambient", &material->k_ambient, 0, 1)){	material->modified_by_user = true;}
				}
				if (type == Texture::TextureType::SPECULAR)
				{
					if(ImGui::ColorEdit3("Specular Color", material->specular_color)) { material->modified_by_user = true; }
					if (ImGui::SliderFloat("k specular", &material->k_specular, 0, 1)) { material->modified_by_user = true; }
					if (ImGui::SliderFloat("Shininess", &material->shininess, 0, 1)) { material->modified_by_user = true; }
				}

				ImGui::Separator();
			}
		}
	}
}

void PanelComponent::DropTarget(ComponentMaterial *material, Texture::TextureType type)
{
	if (ImGui::BeginDragDropTarget())
	{
		if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("DND_File"))
		{
			assert(payload->DataSize == sizeof(File*));
			File *incoming_file = *(File**)payload->Data;
			if (incoming_file->file_type == FileType::TEXTURE)
			{
				//UndoRedo
				App->actions->type_texture = type;
				App->actions->action_component = material;
				App->actions->AddUndoAction(ModuleActions::UndoActionType::EDIT_COMPONENTMATERIAL);

				material->SetMaterialTexture(type, App->texture->LoadTexture(incoming_file->file_path.c_str()));
				material->modified_by_user = true;
			}
		}
		ImGui::EndDragDropTarget();
	}
}

ENGINE_API void PanelComponent::DropGOTarget(GameObject*& go)
{
	if (ImGui::BeginDragDropTarget())
	{
		if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("DND_GameObject"))
		{
			assert(payload->DataSize == sizeof(GameObject*));
			GameObject *incoming_game_object = *(GameObject**)payload->Data;
			go = incoming_game_object;
		}
		ImGui::EndDragDropTarget();
	}
}

std::string PanelComponent::GetTypeName(Texture::TextureType type)
{
	switch (type)
	{
	case Texture::TextureType::DIFUSSE:
		return "Difusse";
		break;
	case Texture::TextureType::SPECULAR:
		return "Specular";
		break;
	case Texture::TextureType::EMISSIVE:
		return "Emissive";
		break;
	case Texture::TextureType::OCLUSION:
		return "Oclusion";
		break;
	default:
		return "";
	}
}

void PanelComponent::ShowComponentCameraWindow(ComponentCamera *camera)
{
	if (ImGui::CollapsingHeader(ICON_FA_VIDEO " Camera", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if(ImGui::Checkbox("Active", &camera->active))
		{
			//UndoRedo
			App->actions->action_component = camera;
			App->actions->AddUndoAction(ModuleActions::UndoActionType::ENABLE_DISABLE_COMPONENT);
			camera->modified_by_user = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete"))
		{
			App->actions->DeleteComponentUndo(camera);
			return;
		}
		ImGui::Separator();

		if (ImGui::InputFloat3("Front", &camera->camera_frustum.front[0], 3, ImGuiInputTextFlags_ReadOnly)) { camera->modified_by_user = true; };
		if (ImGui::InputFloat3("Up", &camera->camera_frustum.up[0], 3, ImGuiInputTextFlags_ReadOnly)) { camera->modified_by_user = true; };

		ImGui::Separator();

		if (ImGui::DragFloat("Mov Speed", &camera->camera_movement_speed, 0.01f, camera->CAMERA_MINIMUN_MOVEMENT_SPEED, camera->CAMERA_MAXIMUN_MOVEMENT_SPEED)) { camera->modified_by_user = true; };
		
		//UndoRedo
		CheckClickedCamera(camera);

		if (ImGui::DragFloat("FOV", &camera->camera_frustum.verticalFov, 0.01f, 0, 2 * 3.14f))
		{
			camera->SetFOV(camera->camera_frustum.verticalFov);
			camera->modified_by_user = true;
		}

		//UndoRedo
		CheckClickedCamera(camera);

		if (ImGui::DragFloat("Aspect Ratio", &camera->aspect_ratio, 0.01f , 0, 10))
		{
			camera->SetAspectRatio(camera->aspect_ratio);
			camera->modified_by_user = true;
		}

		//UndoRedo
		CheckClickedCamera(camera);

		if (ImGui::DragFloat("Near plane", &camera->camera_frustum.nearPlaneDistance, 0.01f, 1, camera->camera_frustum.farPlaneDistance + 1))
		{
			camera->SetNearDistance(camera->camera_frustum.nearPlaneDistance);
			camera->modified_by_user = true;
		}

		//UndoRedo
		CheckClickedCamera(camera);

		if (ImGui::DragFloat("Far plane", &camera->camera_frustum.farPlaneDistance, 0.01f, camera->camera_frustum.nearPlaneDistance + 1, camera->camera_frustum.nearPlaneDistance + 1000))
		{
			camera->SetFarDistance(camera->camera_frustum.farPlaneDistance);
			camera->modified_by_user = true;
		}

		//UndoRedo
		CheckClickedCamera(camera);

		ImGui::Separator();
		int camera_clear_mode = static_cast<int>(camera->camera_clear_mode);
		if (ImGui::Combo("Clear Mode", &camera_clear_mode, "Color\0Skybox\0"))
		{
			switch (camera_clear_mode)
			{
			case 0:
				camera->SetClearMode(ComponentCamera::ClearMode::COLOR);
				camera->modified_by_user = true;
				break;
			case 1:
				camera->SetClearMode(ComponentCamera::ClearMode::SKYBOX);
				camera->modified_by_user = true;
				break;
			}
		}		
		if (ImGui::ColorEdit3("Clear Color", camera->camera_clear_color)) { camera->modified_by_user = true; };
		ImGui::Separator();

		if (ImGui::DragFloat("Orthographic Size", &camera->camera_frustum.orthographicHeight, 0.01f, 0, 100))
		{
			camera->SetOrthographicSize(float2(camera->camera_frustum.orthographicHeight * camera->aspect_ratio, camera->camera_frustum.orthographicHeight));
			camera->modified_by_user = true;
		}

		//UndoRedo
		CheckClickedCamera(camera);

		if (ImGui::Combo("Front faces", &camera->perpesctive_enable, "Perspective\0Orthographic\0"))
		{
			switch (camera->perpesctive_enable)
			{
			case 0:
				camera->SetPerpesctiveView();
				break;
			case 1:
				camera->SetOrthographicView();
				break;
			}
		}
		ImGui::Separator();

		if (ImGui::DragInt("Depth", &camera->depth, 0.05f)) { camera->modified_by_user = true; };

		//UndoRedo
		CheckClickedCamera(camera);

	}
}

void PanelComponent::CheckClickedCamera(ComponentCamera* camera)
{
	//UndoRedo
	if (ImGui::IsItemActive() && !ImGui::IsItemActiveLastFrame())
	{
		//Push new action
		App->actions->action_component = camera;
		App->actions->AddUndoAction(ModuleActions::UndoActionType::EDIT_COMPONENTCAMERA);
	}
}

void PanelComponent::CheckClickForUndo(ModuleActions::UndoActionType  type, Component* component)
{
	if (ImGui::IsItemActive() && !ImGui::IsItemActiveLastFrame())
	{
		switch (type)
		{
		case ModuleActions::UndoActionType::TRANSLATION:
			App->actions->previous_transform = ((ComponentTransform*)component)->GetTranslation();
			break;
		case ModuleActions::UndoActionType::ROTATION:
			App->actions->previous_transform = ((ComponentTransform*)component)->GetRotationRadiants();
			break;
		case ModuleActions::UndoActionType::SCALE:
			App->actions->previous_transform = ((ComponentTransform*)component)->GetScale();
			break;
		case ModuleActions::UndoActionType::EDIT_COMPONENTLIGHT:
			App->actions->previous_light_color[0] = ((ComponentLight*)component)->light_color[0];
			App->actions->previous_light_color[1] = ((ComponentLight*)component)->light_color[1];
			App->actions->previous_light_color[2] = ((ComponentLight*)component)->light_color[2];
			App->actions->previous_light_intensity = ((ComponentLight*)component)->light_intensity;
			App->actions->action_component = component;
			break;
		default:
			break;
		}


		App->actions->clicked = true;
	}

	if (ImGui::IsItemDeactivatedAfterChange())
	{
		App->actions->AddUndoAction(type);
		App->actions->clicked = false;
	}

}

void PanelComponent::ShowComponentLightWindow(ComponentLight *light)
{
	if (ImGui::CollapsingHeader(ICON_FA_LIGHTBULB " Light", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if(ImGui::Checkbox("Active", &light->active))
		{
			//UndoRedo
			App->actions->action_component = light;
			App->actions->AddUndoAction(ModuleActions::UndoActionType::ENABLE_DISABLE_COMPONENT);
			light->modified_by_user = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete"))
		{
			App->actions->DeleteComponentUndo(light);

			return;
		}
		ImGui::Separator();

		if (ImGui::ColorEdit3("Color", light->light_color)) { light->modified_by_user = true; };
		
		CheckClickForUndo(ModuleActions::UndoActionType::EDIT_COMPONENTLIGHT, light);
		
		if (ImGui::DragFloat("Intensity ", &light->light_intensity, 0.01f, 0.f, 1.f)) { light->modified_by_user = true; };

		CheckClickForUndo(ModuleActions::UndoActionType::EDIT_COMPONENTLIGHT, light);

		int light_type = static_cast<int>(light->light_type);

		if (ImGui::Combo("Light Type", &light_type, "Point\0Spot\0Directional"))
		{
			switch (light_type)
			{
			case 0:
				light->light_type = ComponentLight::LightType::POINT_LIGHT;
				break;
			case 1:
				light->light_type = ComponentLight::LightType::SPOT_LIGHT;
				break;
			case 2:
				light->light_type = ComponentLight::LightType::DIRECTIONAL_LIGHT;
				break;
			}
		}
		if (light->light_type == ComponentLight::LightType::POINT_LIGHT)
		{
			if (ImGui::DragFloat("Range", &light->point_light_parameters.range, 1.f, 1.f, 100.f))
			{
				light->point_light_parameters.ChangePointLightAttenuationValues(light->point_light_parameters.range);
			}
		}
		if (light->light_type == ComponentLight::LightType::SPOT_LIGHT)
		{
			if (ImGui::DragFloat("Spot Angle", &light->spot_light_parameters.spot_angle, 1.f, 1.f, 179.f))
			{
				light->spot_light_parameters.SetSpotAngle(light->spot_light_parameters.spot_angle);
			}
			if (ImGui::DragFloat("Edge Softness", &light->spot_light_parameters.edge_softness, 0.01f, 0.f, 1.f))
			{
				light->spot_light_parameters.SetEdgeSoftness(light->spot_light_parameters.edge_softness);
			}
			if (ImGui::DragFloat("Range", &light->spot_light_parameters.range, 1.f, 1.f, 100.f))
			{
				light->spot_light_parameters.ChangeSpotLightAttenuationValues(light->spot_light_parameters.range);
			}
		}
		
	}
}

void PanelComponent::ShowComponentScriptWindow(ComponentScript* component_script)
{
	if (ImGui::CollapsingHeader(ICON_FA_EDIT " Script", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::Checkbox("Active", &component_script->active))
		{
			//UndoRedo TODO
			//App->editor->action_component = component_script;
			//App->editor->AddUndoAction(ModuleEditor::UndoActionType::ENABLE_DISABLE_COMPONENT);
		}
		
		
		if (ImGui::Button("Delete"))
		{
			component_script->owner->RemoveComponent(component_script);
			App->scripts->RemoveComponentScript(component_script);
			return;
		}
		ImGui::SameLine();
		if (ImGui::Button("Refresh"))
		{
			App->scripts->Refresh();
			return;
		}
		ShowScriptsCreated(component_script);
		ImGui::Separator();

		component_script->ShowComponentWindow();

		// to implement CheckClickForUndo(ModuleEditor::UndoActionType::EDIT_COMPONENTSCRIPT, component_script);

	}
}

void PanelComponent::ShowAddNewComponentButton()
{
	float window_width = ImGui::GetWindowWidth();
	float button_width = 0.5f * window_width;
	ImGui::SetCursorPosX((window_width - button_width) / 2.f);
	ImGui::Button("Add Component", ImVec2(button_width, 25));

	//UndoRedo
	Component* component = nullptr;

	if (ImGui::BeginPopupContextItem("Add component", 0))
	{
		char tmp_string[128];

		sprintf_s(tmp_string, "%s Material", ICON_FA_IMAGE);
		if (ImGui::Selectable(tmp_string))
		{
			component = App->editor->selected_game_object->CreateComponent(Component::ComponentType::MATERIAL);

		}

		sprintf_s(tmp_string, "%s Camera", ICON_FA_VIDEO);
		if (ImGui::Selectable(tmp_string))
		{
			component = App->editor->selected_game_object->CreateComponent(Component::ComponentType::CAMERA);

		}

		sprintf_s(tmp_string, "%s Light", ICON_FA_LIGHTBULB);
		if (ImGui::Selectable(tmp_string))
		{
			component = App->editor->selected_game_object->CreateComponent(Component::ComponentType::LIGHT);

		}
		sprintf_s(tmp_string, "%s Script", ICON_FA_EDIT);
		if (ImGui::Selectable(tmp_string))
		{
			App->editor->selected_game_object->CreateComponent(Component::ComponentType::SCRIPT);

		}

		ImGui::EndPopup();
	}

	if(component != nullptr)
	{
		App->actions->action_component = component;
		App->actions->AddUndoAction(ModuleActions::UndoActionType::ADD_COMPONENT);
	}
}

void PanelComponent::ShowScriptsCreated(ComponentScript* component_script) 
{

	if (ImGui::BeginCombo("Add Script", component_script->name.c_str()))
	{
		for (auto script_name : App->scripts->scripts_list) {
			if (ImGui::Selectable(script_name.c_str()))
			{
				component_script->LoadName(script_name);
		
			}
		}
		ImGui::Separator();
		if (ImGui::Selectable("Create new Script")) 
		{
			App->editor->popups->create_script_shown = true;
		}

		ImGui::EndCombo();
	}

}



