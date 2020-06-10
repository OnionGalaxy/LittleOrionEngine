#include "Animation.h"

#include "ResourceManagement/Metafile/Metafile.h"

Animation::Animation(uint32_t uuid, std::vector<KeyFrame> && keyframes, std::vector<MorphChannel> && morhp_channels, std::string name, float frames, float frames_per_second)

	: keyframes(keyframes)
	, morph_channels(morhp_channels)
	, name(name)
	, frames(frames)
	, frames_per_second(frames_per_second)
	, Resource(uuid)
{}
