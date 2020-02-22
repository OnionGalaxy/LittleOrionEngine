#include "ModuleInput.h"

#include "Main/Globals.h"
#include "Main/Application.h"
#include "ModuleWindow.h"
#include "ModuleModelLoader.h"
#include "ModuleCamera.h"
#include "ModuleEditor.h"
#include "ModuleFileSystem.h"
#include "ModuleRender.h"
#include "ModuleScene.h"
#include "ModuleUI.h"
#include "Component/ComponentCamera.h"
#include "UI/Panel/PanelProjectExplorer.h"
#include "UI/Panel/PanelScene.h"

#include <SDL/SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl.h"
#include <GL/glew.h>
#include "Brofiler/Brofiler.h"

// Called before render is available
bool ModuleInput::Init()
{
	APP_LOG_SECTION("************ Module Input Init ************");

	APP_LOG_INIT("Init SDL input event system");
	bool ret = true;
	SDL_Init(0);

	if (SDL_InitSubSystem(SDL_INIT_EVENTS) < 0)
	{
		APP_LOG_ERROR("SDL_EVENTS could not initialize! SDL_Error: %s\n", SDL_GetError());
		ret = false;
	}

	for (int i = 0; i < MAX_KEYS; ++i)
	{
		key_bible[(KeyCode)i] = KeyState::IDLE;
	}

	for (int i = 0; i < NUM_MOUSE_BUTTONS; ++i)
	{
		mouse_bible[(MouseCode)i] = KeyState::IDLE;
	}

	APP_LOG_SUCCESS("SDL input event system initialized correctly.");

	return ret;
}

// Called every draw update
update_status ModuleInput::PreUpdate()
{
	BROFILER_CATEGORY("Inputs PreUpdate", Profiler::Color::BlueViolet);

	for (auto& mouse : mouse_bible)
	{
		if (mouse.second == KeyState::DOWN)
		{
			mouse.second = KeyState::REPEAT;
		}
		else if (mouse.second == KeyState::UP)
		{
			mouse.second = KeyState::IDLE;
		}
	}

	SDL_PumpEvents();

	SDL_Event event;
	while (SDL_PollEvent(&event) != 0)
	{
		ImGui_ImplSDL2_ProcessEvent(&event);

		// Esc button is pressed
		switch (event.type)
		{
		case SDL_QUIT:
			return update_status::UPDATE_STOP;
			break;

		case SDL_WINDOWEVENT:
			if (event.window.event == SDL_WINDOWEVENT_RESIZED || event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
				App->window->WindowResized(event.window.data1, event.window.data2);
			break;

		case SDL_MOUSEMOTION:

			mouse_position = { event.motion.x, event.motion.y };
			mouse_motion = { event.motion.xrel , event.motion.yrel};
			/*if (event.motion.state & SDL_BUTTON_RMASK && App->editor->scene_panel->IsHovered())
			{
				float2 motion(event.motion.xrel, event.motion.yrel);
				App->cameras->scene_camera->RotateCameraWithMouseMotion(motion);
			}
			else if (event.motion.state & SDL_BUTTON_LMASK && App->editor->scene_panel->IsHovered() && App->cameras->IsOrbiting())
			{
				float2 motion(event.motion.xrel, event.motion.yrel);
				if (App->editor->selected_game_object != nullptr)
				{
					App->cameras->scene_camera->OrbitCameraWithMouseMotion(motion, App->editor->selected_game_object->transform.GetGlobalTranslation());
				}
				else
				{
					App->cameras->scene_camera->RotateCameraWithMouseMotion(motion);
				}
			}*/
			break;

		case SDL_MOUSEWHEEL:
			mouse_wheel_motion = event.wheel.y;

			//if (event.wheel.y > 0 && App->editor->scene_panel->IsHovered())
			//{
			//	App->cameras->scene_camera->MoveFoward();
			//}
			//else if (event.wheel.y < 0 && App->editor->scene_panel->IsHovered())
			//{
			//	App->cameras->scene_camera->MoveBackward();
			//}
			break;

		case SDL_MOUSEBUTTONDOWN:

			mouse_bible[(MouseCode)event.button.button] = KeyState::DOWN;

			//if (event.button.button == SDL_BUTTON_RIGHT && App->editor->scene_panel->IsHovered())
			//{
			//	App->cameras->SetMovement(true);
			//}
			//if (event.button.button == SDL_BUTTON_LEFT && App->editor->scene_panel->IsHovered() && !App->cameras->IsOrbiting())
			//{
			//	float2 mouse_position = float2(event.button.x, event.button.y);
			//	App->editor->scene_panel->MousePicking(mouse_position);

			//	if (event.button.clicks == 2 && App->editor->selected_game_object != nullptr)
			//	{
			//		App->cameras->scene_camera->Center(App->editor->selected_game_object->aabb.global_bounding_box);
			//	}
			//}
			break;

		case SDL_MOUSEBUTTONUP:

			mouse_bible[(MouseCode)event.button.button] = KeyState::UP;

			//if (event.button.button == SDL_BUTTON_RIGHT)
			//{
			//	App->cameras->SetMovement(false);
			//}
			break;

		case SDL_KEYDOWN:
			if (event.key.keysym.sym == SDLK_LALT)
			{
				App->cameras->SetOrbit(true);
			}
			else if (event.key.keysym.sym == SDLK_LSHIFT)
			{
				App->cameras->scene_camera->SetSpeedUp(true);
			}
			else if (event.key.keysym.sym == SDLK_f)
			{
				if (App->editor->selected_game_object != nullptr)
				{
					App->cameras->scene_camera->Center(App->editor->selected_game_object->aabb.global_bounding_box);
				}
			}

			//Undo-Redo
			if (event.key.keysym.sym == SDLK_z)
			{
				controlKeyDown = true;
			}

			break;

		case SDL_KEYUP:
			if (event.key.keysym.sym == SDLK_LALT)
			{
				App->cameras->SetOrbit(false);
			}
			else if (event.key.keysym.sym == SDLK_LSHIFT)
			{
				App->cameras->scene_camera->SetSpeedUp(false);
			}

			if (event.key.keysym.sym == SDLK_LCTRL)
			{
				controlKeyDown = false;
			}
			break;

		case SDL_DROPFILE:
			char *dropped_filedir = event.drop.file;
			App->editor->project_explorer->CopyFileToSelectedFolder(dropped_filedir);
			SDL_free(dropped_filedir);

			break;
		}
	}

	/*keyboard = SDL_GetKeyboardState(NULL);

	if (App->cameras->IsMovementEnabled())
	{
		if (keyboard[SDL_SCANCODE_Q])
		{
			App->cameras->scene_camera->MoveUp();
		}

		if (keyboard[SDL_SCANCODE_E])
		{
			App->cameras->scene_camera->MoveDown();
		}

		if (keyboard[SDL_SCANCODE_W])
		{
			App->cameras->scene_camera->MoveFoward();
		}

		if (keyboard[SDL_SCANCODE_S])
		{
			App->cameras->scene_camera->MoveBackward();
		}

		if (keyboard[SDL_SCANCODE_A])
		{
			App->cameras->scene_camera->MoveLeft();
		}

		if (keyboard[SDL_SCANCODE_D])
		{
			App->cameras->scene_camera->MoveRight();
		}
	}

	if (keyboard[SDL_SCANCODE_UP])
	{
		App->cameras->scene_camera->RotatePitch(-1.f);
	}

	if (keyboard[SDL_SCANCODE_DOWN])
	{
		App->cameras->scene_camera->RotatePitch(1.f);
	}

	if (keyboard[SDL_SCANCODE_LEFT])
	{
		App->cameras->scene_camera->RotateYaw(-1.f);
	}

	if (keyboard[SDL_SCANCODE_RIGHT])
	{
		App->cameras->scene_camera->RotateYaw(1.f);
	}

	if(controlKeyDown && keyboard[SDL_SCANCODE_LCTRL] && !keyboard[SDL_SCANCODE_LSHIFT])
	{
		App->editor->Undo();
		controlKeyDown = false;
	}

	if (controlKeyDown && keyboard[SDL_SCANCODE_LSHIFT] && keyboard[SDL_SCANCODE_LCTRL])
	{
		App->editor->Redo();
		controlKeyDown = false;
	}*/

	const Uint8* keys = SDL_GetKeyboardState(nullptr);

	for (int i = 0; i < MAX_KEYS; ++i)
	{
		if (keys[i] == 1)
		{
			if (key_bible[(KeyCode)i] == KeyState::IDLE)
			{
				key_bible[(KeyCode)i] = KeyState::DOWN;
			}
			else if (key_bible[(KeyCode)i] == KeyState::DOWN)
			{
				key_bible[(KeyCode)i] = KeyState::REPEAT;
			}
		}
		else if (keys[i] == 0)
		{
			if (key_bible[(KeyCode)i] == KeyState::REPEAT || key_bible[(KeyCode)i] == KeyState::DOWN)
			{
				key_bible[(KeyCode)i] = KeyState::UP;
			}
			else if (key_bible[(KeyCode)i] == KeyState::UP)
			{
				key_bible[(KeyCode)i] = KeyState::IDLE;
			}
		}
	}


	return update_status::UPDATE_CONTINUE;
}

// Called before quitting
bool ModuleInput::CleanUp()
{
	APP_LOG_INFO("Quitting SDL input event subsystem");
	SDL_QuitSubSystem(SDL_INIT_EVENTS);
	return true;
}

bool ModuleInput::GetKey(KeyCode key)
{
	//Returns true while the user holds down the key identified by name.
	return key_bible[key] == KeyState::REPEAT;
}

bool ModuleInput::GetKeyDown(KeyCode key)
{
	//Returns true during the frame the user starts pressing down the key identified by name.
	return key_bible[key] == KeyState::DOWN;
}

bool ModuleInput::GetKeyUp(KeyCode key)
{
	//Returns true during the frame the user releases the key identified by name.
	return key_bible[key] == KeyState::UP;
}

bool ModuleInput::GetMouseButton(MouseCode mouse)
{
	//Returns whether the given mouse button is held down.
	return mouse_bible[mouse] == KeyState::REPEAT;
}

bool ModuleInput::GetMouseButtonDown(MouseCode mouse)
{
	//Returns true during the frame the user pressed the given mouse button.
	return mouse_bible[mouse] == KeyState::DOWN;
}

bool ModuleInput::GetMouseButtonUp(MouseCode mouse)
{
	//Returns true during the frame the user releases the given mouse button.
	return mouse_bible[mouse] == KeyState::UP;
}

int2 ModuleInput::GetMousePosition() const
{
	return mouse_position;
}

int2 ModuleInput::GetMouseMotion() const
{
	return mouse_motion;
}

Sint32 ModuleInput::GetMouseWheelMotion() const
{
	return mouse_wheel_motion;
}
