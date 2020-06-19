#ifndef _ANIMCONTROLLER_H_
#define _ANIMCONTROLLER_H_

#include "ResourceManagement/Resources/Animation.h"
#include "EditorUI/Panel/InspectorSubpanel/PanelComponent.h"

#include <vector>

class StateMachine;
class Skeleton;
struct State;
struct Clip;
struct Transition;

struct PlayingClip
{
	std::shared_ptr<Clip> clip;
	float current_time = 0;
	bool playing = false;
	float time = 0;
	void Update(float speed);
};

enum ClipType
{
	ACTIVE = 0,
	NEXT
};
class AnimController
{
public:
	AnimController() { playing_clips.resize(2); };
	~AnimController() = default;

	AnimController(const AnimController& controller_to_copy) = default;
	AnimController(AnimController&& controller_to_move) = default;

	AnimController & operator=(const AnimController& controller_to_copy) = default;
	AnimController & operator=(AnimController&& controller_to_move) = default;

	bool Update();
	void SetStateMachine(uint32_t state_machine_uuid);
	void GetClipTransform(uint32_t skeleton_uuid, std::vector<math::float4x4>& pose);
	void StartNextState(const std::string& trigger);
	std::shared_ptr<Transition> active_transition;
	bool IsOnState(const std::string& state);

	bool apply_transition = false;
private:
	void SetActiveState(std::shared_ptr<State> & state);
	void FinishActiveState();
public:
	std::shared_ptr<StateMachine> state_machine = nullptr;
	std::vector<PlayingClip> playing_clips;

private:
	std::shared_ptr<State> active_state = nullptr;
	friend class PanelComponent;
};

#endif //_ANIMCONTROLLER_H_

