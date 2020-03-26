#include "PanelMenuBar.h"

#include "EditorUI/Panel/PanelAbout.h"
#include "EditorUI/Panel/PanelConfiguration.h"
#include "EditorUI/Panel/PanelConsole.h"
#include "EditorUI/Panel/PanelDebug.h"
#include "EditorUI/Panel/PanelGame.h"
#include "EditorUI/Panel/PanelHierarchy.h"
#include "EditorUI/Panel/PanelInspector.h"
#include "EditorUI/Panel/PanelPopups.h"
#include "EditorUI/Panel/PanelNavMesh.h"
#include "EditorUI/Panel/PanelProjectExplorer.h"
#include "EditorUI/Panel/PanelScene.h"
#include "EditorUI/Panel/PanelResourceDatabase.h"
#include "EditorUI/Panel/PopupsPanel/PanelPopupSceneManagement.h"

#include "Filesystem/PathAtlas.h"
#include "Main/Application.h"
#include "Module/ModuleEditor.h"
#include "Module/ModuleFileSystem.h"
#include "Module/ModuleScene.h"

#include <FontAwesome5/IconsFontAwesome5.h>
#include <FontAwesome5/IconsFontAwesome5Brands.h>
#include <imgui.h>
#include <SDL/SDL.h>

PanelMenuBar::PanelMenuBar()
{
	enabled = true;
	window_name = "MainMenuBar";
}

void PanelMenuBar::Render()
{
	if (ImGui::BeginMainMenuBar())
	{
		ShowFileMenu();
		ShowGameObjectMenu();
		ShowWindowMenu();
		ShowHelpMenu();
		ImGui::EndMainMenuBar();
	}
}


void PanelMenuBar::ShowFileMenu()
{
	if (ImGui::BeginMenu("File"))
	{
		if (ImGui::MenuItem(ICON_FA_FILE " New Scene"))
		{
			App->editor->OpenScene(DEFAULT_SCENE_PATH);
		}
		if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN " Load Scene"))
		{
			App->editor->popups->scene_management_popup.load_scene_shown = true;
		}
		ImGui::Separator();
		if (App->editor->current_scene_path != "" && ImGui::MenuItem(ICON_FA_SAVE " Save Scene"))
		{
			App->editor->SaveScene(App->editor->current_scene_path);
		}
		if (ImGui::MenuItem(ICON_FA_SAVE " Save Scene as"))
		{
			App->editor->popups->scene_management_popup.save_scene_shown = true;
		}
		if (ImGui::MenuItem(ICON_FA_SIGN_OUT_ALT " Exit"))
		{
			SDL_Event quit_event;
			quit_event.type = SDL_QUIT;
			SDL_PushEvent(&quit_event);
		}

		ImGui::EndMenu();
	}
}

void PanelMenuBar::ShowGameObjectMenu()
{
	if (ImGui::BeginMenu("GameObject"))
	{
		if (ImGui::Selectable("Create Empty"))
		{
			App->scene->CreateGameObject();
		}

		if (ImGui::Selectable("Create Empty Child"))
		{
			if (App->editor->selected_game_object != nullptr)
			{
				App->scene->CreateChildGameObject(App->editor->selected_game_object);
			}
			else
			{
				App->scene->CreateGameObject();
			}
		}
		
		if (ImGui::BeginMenu("3D Object"))
		{
			/* TODO: This
			if (ImGui::Selectable("Cube"))
			{
				App->model_loader->LoadCoreModel(PRIMITIVE_CUBE_PATH);
			}
			if (ImGui::Selectable("Cylinder"))
			{
				App->model_loader->LoadCoreModel(PRIMITIVE_CYLINDER_PATH);
			}
			if (ImGui::Selectable("Sphere"))
			{
				App->model_loader->LoadCoreModel(PRIMITIVE_SPHERE_PATH);
			}
			if (ImGui::Selectable("Torus"))
			{
				App->model_loader->LoadCoreModel(PRIMITIVE_TORUS_PATH);
			}
			*/
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Light"))
		{
			if (ImGui::Selectable("Point Light"))
			{
				GameObject* created_game_object = App->scene->CreateGameObject();
				created_game_object->name = "Point Light";
				created_game_object->CreateComponent(Component::ComponentType::LIGHT);
			}
			
			ImGui::EndMenu();
		}

		if (ImGui::Selectable("Camera"))
		{
			GameObject* created_game_object = App->scene->CreateGameObject();
			created_game_object->name = "Camera";
			created_game_object->CreateComponent(Component::ComponentType::CAMERA);
		}

		ImGui::EndMenu();
	}
}

void PanelMenuBar::ShowWindowMenu()
{
	if (ImGui::BeginMenu("Window"))
	{
		if (ImGui::BeginMenu("General"))
		{
			if (ImGui::MenuItem((ICON_FA_SITEMAP " Hierarchy"), (const char*)0, App->editor->hierarchy->IsOpened()))
			{
				App->editor->hierarchy->SwitchOpen();
			}
			if (ImGui::MenuItem((ICON_FA_TH " Scene"), (const char*)0, App->editor->scene_panel->IsOpened()))
			{
				App->editor->scene_panel->SwitchOpen();
			}
			if (ImGui::MenuItem((ICON_FA_GHOST " Game"), (const char*)0, App->editor->game_panel->IsOpened()))
			{
				App->editor->game_panel->SwitchOpen();
			}
			if (ImGui::MenuItem((ICON_FA_INFO " Inspector"), (const char*)0, App->editor->inspector->IsOpened()))
			{
				App->editor->inspector->SwitchOpen();
			}
			if (ImGui::MenuItem((ICON_FA_FOLDER_OPEN " Explorer"), (const char*)0, App->editor->project_explorer->IsOpened()))
			{
				App->editor->project_explorer->SwitchOpen();
			}
			if (ImGui::MenuItem((ICON_FA_TERMINAL " Console"), (const char*)0, App->editor->console->IsOpened()))
			{
				App->editor->console->SwitchOpen();
			}
			if (ImGui::MenuItem((ICON_FA_BRAIN " AI"), (const char*)0, App->editor->nav_mesh->IsOpened()))
			{
				App->editor->nav_mesh->SwitchOpen();
			}
			if (ImGui::MenuItem((ICON_FA_DATABASE " Resource Database"), (const char*)0, App->editor->resource_database->IsOpened()))
			{
				App->editor->resource_database->SwitchOpen();
			}

			ImGui::EndMenu();
		}

		if (ImGui::MenuItem((ICON_FA_COGS " Config"), (const char*)0, App->editor->configuration->IsOpened()))
		{
			App->editor->configuration->SwitchOpen();
		}

		if (ImGui::MenuItem((ICON_FA_BUG " Debug"), (const char*)0, App->editor->debug_panel->IsOpened()))
		{
			App->editor->debug_panel->SwitchOpen();
		}
		
		ImGui::EndMenu();
	}
}

void PanelMenuBar::ShowHelpMenu()
{
	if (ImGui::BeginMenu("Help"))
	{
		if (ImGui::MenuItem(ICON_FA_QUESTION_CIRCLE " About", (const char*)0, App->editor->about->IsOpened()))
		{
			App->editor->about->SwitchOpen();
		}
		ImGui::PushFont(App->editor->GetFont(Fonts::FONT_FAB));
		if (ImGui::MenuItem(ICON_FA_GITHUB_ALT " Repository"))
		{
			ShellExecuteA(NULL, "open", "https://github.com/unnamed-company/LittleOrionEngine/", NULL, NULL, SW_SHOWNORMAL);
		}
		ImGui::PopFont();
		ImGui::EndMenu();
	}
}