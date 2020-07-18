#include "AnimController.h"

#include "Component/ComponentMeshRenderer.h"
#include "Main/Application.h"
#include "Module/ModuleResourceManager.h"
#include "Module/ModuleTime.h"
#include "ResourceManagement/Resources/StateMachine.h"
#include "Helper/Utils.h"

#include <math.h>
void AnimController::GetClipTransform(const std::shared_ptr<Skeleton>& skeleton, std::vector<math::float4x4>& pose, uint64_t mesh_hash_name, std::vector<float>& morph_weights)
{
	pose.resize(skeleton->skeleton.size());
	for (size_t j = 0; j < playing_clips.size(); j++)
	{
		const std::shared_ptr<Clip> clip = playing_clips[j].clip;
		uint32_t skeleton_uuid = skeleton->GetUUID();
		if (!clip || clip->skeleton_channels_joints_map.find(skeleton_uuid) == clip->skeleton_channels_joints_map.end())
		{
			continue;
		}

		float weight = j != ClipType::ACTIVE ? playing_clips[j].current_time / (playing_clips[j].interpolation_time) : 0.0f;
		float current_percentage = playing_clips[j].current_time / clip->animation_time;
		float current_keyframe = current_percentage * (clip->animation->frames - 1);
		//Get current Keyframe

		size_t first_keyframe_index = static_cast<size_t>(std::floor(current_keyframe));
		size_t second_keyframe_index = static_cast<size_t>(std::ceil(current_keyframe));
		if (second_keyframe_index >= clip->animation->frames || first_keyframe_index >= clip->animation->frames)
		{
			second_keyframe_index = clip->loop ? 0 : clip->animation->frames - 1;
			first_keyframe_index = clip->loop ? 0 : clip->animation->frames - 1;
		}
		float interpolation_lambda = current_keyframe - std::floor(current_keyframe);
		const std::vector<Animation::Channel>& current_pose = clip->animation->keyframes[first_keyframe_index].channels;
		const std::vector<Animation::Channel>& next_pose = clip->animation->keyframes[second_keyframe_index].channels;
		assert(current_pose.size() == next_pose.size());


		//Morphing
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
				morph_weights[i] = math::Lerp(current_weights[i].weight, next_weights[i].weight, interpolation_lambda);
			}
		}

		//Position
		auto& joint_channels_map = clip->skeleton_channels_joints_map[skeleton_uuid];
		assert(joint_channels_map.size() == skeleton->skeleton.size());
		for (size_t i = 0; i < joint_channels_map.size(); ++i)
		{
			float4x4 current_transform = skeleton->skeleton[i].transform_local;
			
			size_t channel_index = joint_channels_map[i];
			if (channel_index < current_pose.size())
			{
				const float3& last_translation = current_pose[channel_index].translation;
				const float3& next_translation = next_pose[channel_index].translation;

				const Quat& last_rotation = current_pose[channel_index].rotation;
				const Quat& next_rotation = next_pose[channel_index].rotation;

				float3 position = Utils::Interpolate(last_translation, next_translation, interpolation_lambda);
				Quat rotation = Utils::Interpolate(last_rotation, next_rotation, interpolation_lambda);
				current_transform = float4x4::FromTRS(position, rotation, float3::one);
				if (j != ClipType::ACTIVE)
				{
					pose[i] = Utils::Interpolate(pose[i], current_transform, weight);
				}
				else
				{
					pose[i] = current_transform;

				}	
			}
			else
			{
				pose[i] = current_transform;
			}
		}
		if (weight >= 1.0f)
		{
			apply_transition = true;
		}
	}

}

void AnimController::UpdateAttachedBones(const std::shared_ptr<Skeleton>& skeleton, const std::vector<math::float4x4>& pose)
{
	if (attached_bones.empty())
	{
		return;
	}
	const auto& joint_channels_map = playing_clips[ClipType::ACTIVE].clip->skeleton_channels_joints_map[skeleton->GetUUID()];
	for (size_t i = 0; i < joint_channels_map.size(); ++i)
	{
		size_t joint_index = joint_channels_map[i];
		for (AttachedBone attached_bones : attached_bones)
		{
			if (attached_bones.first == joint_index)
			{
				const float4x4& new_model_matrix = pose[joint_index];
				attached_bones.second->transform.SetTranslation(new_model_matrix.TranslatePart());
				attached_bones.second->transform.SetRotation(new_model_matrix.TranslatePart());
			}
		}

	}
}

bool AnimController::Update()
{
	ApplyAutomaticTransitionIfNeeded();
	for (auto & playing_clip : playing_clips)
	{
		playing_clip.Update();	 
	}
	if (apply_transition && active_transition)
	{
		FinishActiveState();
	}
	return playing_clips[ClipType::ACTIVE].playing || playing_clips[ClipType::NEXT].playing;
}

void AnimController::ApplyAutomaticTransitionIfNeeded()
{
	if (!applying_automatic_transition && active_transition && active_transition->automatic)
	{

		float animation_time_with_interpolation = playing_clips[ClipType::ACTIVE].current_time + active_transition->interpolation_time;

		if (animation_time_with_interpolation >= playing_clips[ClipType::ACTIVE].clip->animation_time)
		{
			auto& next_state = state_machine->GetState(active_transition->target_hash);
			playing_clips[ClipType::NEXT] = { next_state->clip, next_state->speed, 0.0f,true, static_cast<float>(active_transition->interpolation_time) };
			AdjustInterpolationTimes();
			applying_automatic_transition = true;
		}
	}
}

void AnimController::AdjustInterpolationTimes()
{
	if (!playing_clips[ClipType::ACTIVE].clip->loop)
	{
		float time_left = playing_clips[ClipType::ACTIVE].clip->animation_time - playing_clips[ClipType::ACTIVE].current_time;	
		playing_clips[ClipType::NEXT].interpolation_time = max(min(active_transition->interpolation_time,  time_left), 0.0f);
		APP_LOG_INFO("Interpolation time set to: %f", playing_clips[ClipType::NEXT].interpolation_time);
	}

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
		playing_clips[ClipType::ACTIVE] = { active_state->clip, active_state->speed };
		active_transition = state_machine->GetAutomaticTransition(active_state->name_hash);
	}
}

void AnimController::StartNextState(const std::string& trigger)
{
	if (!active_state)
	{
		return;
	}
	std::shared_ptr<Transition> next_transition = state_machine->GetTriggerTransition(trigger, active_state->name_hash);
	if (!next_transition || next_transition == active_transition)
	{
		return;
	}
	std::shared_ptr<State> next_state;
	active_transition = next_transition;
	next_state = state_machine->GetState(active_transition->target_hash);
	playing_clips[ClipType::NEXT] = { next_state->clip, next_state->speed, 0.0f, true,  static_cast<float>(active_transition ->interpolation_time)};

	if (!playing_clips[ClipType::ACTIVE].playing)
	{
		FinishActiveState();
	}
	else
	{
		AdjustInterpolationTimes();
	}
}

bool AnimController::IsOnState(const std::string& state)
{
	if (!active_state)
	{
		return false;
	}
	uint64_t state_hash = std::hash<std::string>{}(state);
	return active_state.get()->name_hash == state_hash;
}

void AnimController::SetSpeed(float speed)
{
	if(active_state)
	{
		playing_clips[ClipType::ACTIVE].speed = speed;
	}
}

void AnimController::FinishActiveState()
{
	std::shared_ptr<State> next_state = state_machine->GetState(active_transition->target_hash);
	SetActiveState(next_state);
	playing_clips[ClipType::ACTIVE] = playing_clips[ClipType::NEXT];
	playing_clips[ClipType::NEXT] = {};
	apply_transition = false;
	applying_automatic_transition = false;
}

void PlayingClip::Update()
{
	if (!playing || !clip || clip->animation == nullptr)
	{
		return;
	}
	current_time = (current_time + (App->time->delta_time)* speed) ;
	if (current_time >= clip->animation_time)
	{
		if (clip->loop)
		{
			current_time = static_cast<int>(current_time) % static_cast<int>(clip->animation_time);
		}
		else
		{
			playing = false;
		}
	}
}

