#include "AnimController.h"

#include "Component/ComponentMeshRenderer.h"
#include "Main/Application.h"
#include "Module/ModuleResourceManager.h"
#include "Module/ModuleTime.h"
#include "ResourceManagement/Resources/StateMachine.h"
#include "Helper/Utils.h"
#include <cmath>

#include <math.h>

void AnimController::GetClipTransform(uint32_t skeleton_uuid, std::vector<math::float4x4>& pose, uint64_t mesh_hash_name, std::vector<float>& morph_weights)
{
	for (size_t j = 0; j < playing_clips.size(); j++)
	{
		const std::shared_ptr<Clip> clip = playing_clips[j].clip;
		if (!clip || clip->skeleton_channels_joints_map.find(skeleton_uuid) == clip->skeleton_channels_joints_map.end())
		{
			continue;
		}

		float weight = j != ClipType::ACTIVE ? playing_clips[j].current_time / (active_transition->interpolation_time * 1.0f) : 0.0f;
		float current_keyframe = ((playing_clips[j].current_time*(clip->animation->frames - 1)) / clip->animation_time) + 1;
		size_t first_keyframe_index = static_cast<size_t>(std::floor(current_keyframe));
		size_t second_keyframe_index = static_cast<size_t>(std::ceil(current_keyframe));
		if (second_keyframe_index == clip->animation->frames)
		{
			second_keyframe_index = clip->loop ? 0 : clip->animation->frames - 1;
		}

		float interpolation_lambda = current_keyframe - std::floor(current_keyframe);
		const std::vector<Animation::Channel>& current_pose = clip->animation->keyframes[first_keyframe_index].channels;
		const std::vector<Animation::Channel>& next_pose = clip->animation->keyframes[second_keyframe_index].channels;

		auto & morph_channels = clip->animation->morph_channels;
		auto it = std::find_if(morph_channels.begin(), morph_channels.end(), [&mesh_hash_name](const auto & channel) { return channel.mesh_hash == mesh_hash_name; });
		if (it != morph_channels.end() && second_keyframe_index < (*it).keyframes.size())
		{
			auto& keyframes = (*it).keyframes;
			
			auto& current_weights = keyframes[first_keyframe_index].morph_targets;
			auto& next_weights = keyframes[second_keyframe_index].morph_targets;
			assert(current_weights.size() == next_weights.size());
			for (size_t i = 0; i < current_weights.size(); i++)
			{
				morph_weights[i] = Utils::Lerp(current_weights[i].weight, next_weights[i].weight, interpolation_lambda);
			}
		}

		auto& joint_channels_map = clip->skeleton_channels_joints_map[skeleton_uuid];
		for (size_t i = 0; i < joint_channels_map.size(); ++i)
		{
			size_t joint_index = joint_channels_map[i];
			if (joint_index < pose.size())
			{
				const float3& last_translation = current_pose[i].translation;
				const float3& next_translation = next_pose[i].translation;

				const Quat& last_rotation = current_pose[i].rotation;
				const Quat& next_rotation = next_pose[i].rotation;

				float3 position = Utils::Interpolate(last_translation, next_translation, interpolation_lambda);
				Quat rotation = Utils::Interpolate(last_rotation, next_rotation, interpolation_lambda);
				if (j != ClipType::ACTIVE)
				{
					pose[joint_index] = Utils::Interpolate(pose[joint_index], float4x4::FromTRS(position, rotation, float3::one), weight);
				}
				else
				{
					pose[joint_index] = float4x4::FromTRS(position, rotation, float3::one);

				}	
			}
		}
		if (weight > 1.0f)
		{
			apply_transition = true;
		}
	}
}
bool AnimController::Update()
{
	for (auto & playing_clip : playing_clips)
	{
		 playing_clip.Update();
	}
	if (active_transition && active_transition->automatic)
	{
		float animation_time_with_interpolation = playing_clips[ClipType::ACTIVE].current_time + active_transition->interpolation_time + 100;
		if (animation_time_with_interpolation >= playing_clips[ClipType::ACTIVE].clip->animation_time)
		{
			playing_clips[ClipType::NEXT].clip = state_machine->GetState(active_transition->target_hash)->clip;
			playing_clips[ClipType::NEXT].playing = true;
		}
	}
	if (apply_transition && active_transition)
	{
		FinishActiveState();
	}
	return playing_clips[ClipType::ACTIVE].playing;
}

void AnimController::SetStateMachine(uint32_t state_machine_uuid)
{
	this->state_machine = App->resources->Load<StateMachine>(state_machine_uuid);
	SetActiveState(state_machine->GetDefaultState());
}

void AnimController::SetActiveState(std::shared_ptr<State> & state)
{
	if (state != nullptr && state->clip != nullptr)
	{
		active_state = state;
		playing_clips[ClipType::ACTIVE] = { active_state->clip };
		active_transition = state_machine->GetAutomaticTransition(active_state->name_hash);
	}
}

void AnimController::StartNextState(const std::string& trigger)
{
	std::shared_ptr<Transition> next_transition = state_machine->GetTriggerTransition(trigger, active_state->name_hash);
	if (!next_transition || next_transition == active_transition)
	{
		return;
	}
	std::shared_ptr<State> next_state;
	active_transition = next_transition;
	next_state = state_machine->GetState(active_transition->target_hash);
	playing_clips[ClipType::NEXT] = { next_state->clip, 0, true };
	if (!playing_clips[ClipType::ACTIVE].playing)
	{
		FinishActiveState();
	}
}

bool AnimController::IsOnState(const std::string& state)
{
	uint64_t state_hash = std::hash<std::string>{}(state);
	return active_state.get()->name_hash == state_hash;
}

void AnimController::FinishActiveState()
{
	std::shared_ptr<State> next_state = state_machine->GetState(active_transition->target_hash);
	SetActiveState(next_state);
	playing_clips[ClipType::ACTIVE] = playing_clips[ClipType::NEXT];
	playing_clips[ClipType::NEXT] = {};
	apply_transition = false;
}

void PlayingClip::Update()
{
	if (!playing || !clip || clip->animation == nullptr)
	{
		return;
	}
	current_time = current_time + static_cast<int>(App->time->delta_time);
	if (current_time >= clip->animation_time)
	{
		if (clip->loop)
		{

			current_time = current_time % clip->animation_time;
		}
		else
		{
			playing = false;
		}
	}
}
