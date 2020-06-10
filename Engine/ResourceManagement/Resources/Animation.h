#ifndef _ANIMATION_H_
#define _ANIMATION_H_

#include "Resource.h"

#include "Component/ComponentTransform.h"
#include "ResourceManagement/Manager/AnimationManager.h"

class Animation : public Resource
{
public:
	struct Channel
	{
		std::string name;
		float3 translation;
		Quat rotation;
	};
	struct KeyFrame
	{
		float frame;
		std::vector<Channel> channels;
	};

	struct MorphWeight
	{
		uint32_t target;
		float weight;
	};

	struct MorphKeyFrame
	{
		float frame;
		std::vector<MorphWeight> morph_targets;
	};

	struct MorphChannel
	{
		uint64_t mesh_hash;
		std::vector<MorphKeyFrame> keyframes;
	};

	Animation() = default;
	Animation(uint32_t uuid) : Resource(uuid) {};
	Animation(uint32_t uuid, std::vector<KeyFrame> && keyframes, std::vector<MorphChannel>&& morph_channels, std::string name, float frames, float frames_per_second);
	~Animation() = default;

public:
	std::string name;
	std::vector<KeyFrame> keyframes;
	std::vector<MorphChannel> morph_channels;
	float frames;
	float frames_per_second;
};


namespace ResourceManagement
{
	template<>
	static std::shared_ptr<Animation> Load(uint32_t uuid, const FileData& resource_data)
	{
		return AnimationManager::Load(uuid, resource_data);
	}
}

#endif // _ANIMATION_H_